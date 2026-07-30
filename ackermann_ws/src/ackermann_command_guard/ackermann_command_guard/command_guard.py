"""ROS 2 node guarding TwistStamped Ackermann commands."""

import math
import threading
from typing import Optional
from typing import Tuple

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from .guard_core import GuardLimits
from .guard_core import GuardState
from .guard_core import ProjectionResult
from .guard_core import project_command
from .guard_core import stopped_result
from .guard_core import timestamp_is_fresh


class AckermannCommandGuard(Node):
    """Validate, project, restamp, and watchdog Ackermann body commands."""

    def __init__(self) -> None:
        """Create the guarded command pipeline and its fixed-rate timer."""
        super().__init__('ackermann_command_guard')

        self._input_topic = str(
            self.declare_parameter('input_topic', '/cmd_vel_raw').value
        )
        self._output_topic = str(
            self.declare_parameter('output_topic', '/cmd_vel').value
        )
        self._base_frame_id = str(
            self.declare_parameter(
                'base_frame_id', 'base_footprint'
            ).value
        )
        self._publish_rate = float(
            self.declare_parameter('publish_rate', 100.0).value
        )
        self._command_timeout = float(
            self.declare_parameter('command_timeout', 0.5).value
        )
        self._max_input_age = float(
            self.declare_parameter(
                'max_input_age', self._command_timeout
            ).value
        )
        self._future_tolerance = float(
            self.declare_parameter('future_tolerance', 0.05).value
        )

        self._limits = GuardLimits(
            rolling_wheelbase=self._float_parameter(
                'rolling_wheelbase'
            ),
            steering_track_width=self._float_parameter(
                'steering_track_width'
            ),
            traction_track_width=self._float_parameter(
                'traction_track_width'
            ),
            traction_wheels_radius=self._float_parameter(
                'traction_wheels_radius'
            ),
            r_min_left=self._float_parameter(
                'r_min_left'
            ),
            r_min_right=self._float_parameter(
                'r_min_right'
            ),
            front_left_steering_lower=self._float_parameter(
                'front_left_steering_lower'
            ),
            front_left_steering_upper=self._float_parameter(
                'front_left_steering_upper'
            ),
            front_left_steering_velocity=self._float_parameter(
                'front_left_steering_velocity'
            ),
            front_right_steering_lower=self._float_parameter(
                'front_right_steering_lower'
            ),
            front_right_steering_upper=self._float_parameter(
                'front_right_steering_upper'
            ),
            front_right_steering_velocity=self._float_parameter(
                'front_right_steering_velocity'
            ),
            rear_left_wheel_velocity=self._float_parameter(
                'rear_left_wheel_velocity'
            ),
            rear_right_wheel_velocity=self._float_parameter(
                'rear_right_wheel_velocity'
            ),
            rear_left_wheel_acceleration=self._float_parameter(
                'rear_left_wheel_acceleration'
            ),
            rear_right_wheel_acceleration=self._float_parameter(
                'rear_right_wheel_acceleration'
            ),
            zero_linear_epsilon=float(
                self.declare_parameter(
                    'zero_linear_epsilon', 1.0e-4
                ).value
            ),
            max_projection_iterations=int(
                self.declare_parameter(
                    'max_projection_iterations', 32
                ).value
            ),
        )
        self._validate_runtime_parameters()

        qos = QoSProfile(depth=1)
        self._publisher = self.create_publisher(
            TwistStamped, self._output_topic, qos
        )
        self._subscription = self.create_subscription(
            TwistStamped,
            self._input_topic,
            self._command_callback,
            qos,
        )

        self._lock = threading.RLock()
        self._desired_command: Optional[Tuple[float, float]] = None
        self._last_valid_receive_seconds: Optional[float] = None
        self._last_step_seconds: Optional[float] = None
        self._state = GuardState.stopped()
        self._last_status = ''

        self._timer = self.create_timer(
            1.0 / self._publish_rate,
            self._timer_callback,
            clock=self.get_clock(),
        )
        self.get_logger().info(
            'guarding %s -> %s at %.3f Hz'
            % (
                self._input_topic,
                self._output_topic,
                self._publish_rate,
            )
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.declare_parameter(name, 0.0).value)

    def _validate_runtime_parameters(self) -> None:
        if not math.isfinite(self._publish_rate) or self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')
        if (
            not math.isfinite(self._command_timeout)
            or self._command_timeout <= 0.0
        ):
            raise ValueError(
                'command_timeout must be finite and positive'
            )
        if (
            not math.isfinite(self._max_input_age)
            or self._max_input_age < 0.0
        ):
            raise ValueError(
                'max_input_age must be finite and non-negative'
            )
        if (
            not math.isfinite(self._future_tolerance)
            or self._future_tolerance < 0.0
        ):
            raise ValueError(
                'future_tolerance must be finite and non-negative'
            )
        if not self._input_topic or not self._output_topic:
            raise ValueError('input_topic and output_topic must be non-empty')
        if self._input_topic == self._output_topic:
            raise ValueError('input_topic and output_topic must differ')
        if not self._base_frame_id:
            raise ValueError('base_frame_id must be non-empty')

    @staticmethod
    def _all_twist_components_finite(message: TwistStamped) -> bool:
        values = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        )
        return all(math.isfinite(value) for value in values)

    def _command_callback(self, message: TwistStamped) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1.0e-9

        if not self._all_twist_components_finite(message):
            self._force_stop(now, 'non_finite_twist')
            return

        stamp = message.header.stamp
        zero_stamp = stamp.sec == 0 and stamp.nanosec == 0
        stamp_seconds = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        fresh, reason = timestamp_is_fresh(
            now_seconds,
            stamp_seconds,
            self._max_input_age,
            self._future_tolerance,
            zero_stamp=zero_stamp,
        )
        if not fresh:
            self._force_stop(now, reason)
            return

        with self._lock:
            self._desired_command = (
                float(message.twist.linear.x),
                float(message.twist.angular.z),
            )
            self._last_valid_receive_seconds = now_seconds
        self._set_status('command_received', warning=False)

    def _timer_callback(self) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1.0e-9

        with self._lock:
            if (
                self._last_step_seconds is not None
                and now_seconds + 1.0e-12 < self._last_step_seconds
            ):
                self._desired_command = None
                self._last_valid_receive_seconds = None
                result = stopped_result(
                    limited=True, reason='clock_moved_backwards'
                )
            elif (
                self._desired_command is None
                or self._last_valid_receive_seconds is None
            ):
                result = stopped_result(
                    limited=False, reason='no_command'
                )
            elif (
                now_seconds - self._last_valid_receive_seconds
                > self._command_timeout
            ):
                self._desired_command = None
                self._last_valid_receive_seconds = None
                result = stopped_result(
                    limited=True, reason='watchdog_timeout'
                )
            else:
                if self._last_step_seconds is None:
                    dt = 1.0 / self._publish_rate
                else:
                    dt = now_seconds - self._last_step_seconds
                desired_linear, desired_angular = self._desired_command
                result = project_command(
                    desired_linear,
                    desired_angular,
                    self._state,
                    dt,
                    self._limits,
                )

            self._state = result.state
            self._last_step_seconds = now_seconds

        self._publish_result(now, result)
        warning = result.reason not in (
            'accepted',
            'limited',
            'no_command',
            'zero_linear',
        )
        self._set_status(result.reason, warning=warning)

    def _force_stop(self, now, reason: str) -> None:
        now_seconds = now.nanoseconds * 1.0e-9
        with self._lock:
            self._desired_command = None
            self._last_valid_receive_seconds = None
            self._last_step_seconds = now_seconds
            result = stopped_result(limited=True, reason=reason)
            self._state = result.state
        self._publish_result(now, result)
        self._set_status(reason, warning=True)

    def _publish_result(
        self,
        now,
        result: ProjectionResult,
    ) -> None:
        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self._base_frame_id
        output.twist.linear.x = result.linear
        output.twist.angular.z = result.angular
        self._publisher.publish(output)

    def _set_status(self, status: str, *, warning: bool) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        text = f'command guard state: {status}'
        if warning:
            self.get_logger().warning(text)
        else:
            self.get_logger().debug(text)


def main(args=None) -> None:
    """Run the command guard node."""
    rclpy.init(args=args)
    node = None
    try:
        node = AckermannCommandGuard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
