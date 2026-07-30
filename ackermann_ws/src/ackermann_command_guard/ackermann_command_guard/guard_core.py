"""Pure Ackermann command feasibility calculations.

The functions in this module do not depend on ROS.  The ROS node stores the
latest requested body twist and calls :func:`project_command` at a fixed rate.
"""

import math
from dataclasses import dataclass
from typing import Optional
from typing import Tuple


_EPS = 1.0e-12
_TOL = 1.0e-10


def _finite_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and greater than zero')


@dataclass(frozen=True)
class GuardLimits:
    """Geometry and joint limits used by the command projector."""

    rolling_wheelbase: float
    steering_track_width: float
    traction_track_width: float
    traction_wheels_radius: float
    r_min_left: float
    r_min_right: float
    front_left_steering_lower: float
    front_left_steering_upper: float
    front_left_steering_velocity: float
    front_right_steering_lower: float
    front_right_steering_upper: float
    front_right_steering_velocity: float
    rear_left_wheel_velocity: float
    rear_right_wheel_velocity: float
    rear_left_wheel_acceleration: float
    rear_right_wheel_acceleration: float
    zero_linear_epsilon: float = 1.0e-4
    max_projection_iterations: int = 32

    def __post_init__(self) -> None:
        """Reject geometry or limits that cannot define a safe projection."""
        positive_values = {
            'rolling_wheelbase': self.rolling_wheelbase,
            'steering_track_width': self.steering_track_width,
            'traction_track_width': self.traction_track_width,
            'traction_wheels_radius': self.traction_wheels_radius,
            'r_min_left': self.r_min_left,
            'r_min_right': self.r_min_right,
            'front_left_steering_velocity':
                self.front_left_steering_velocity,
            'front_right_steering_velocity':
                self.front_right_steering_velocity,
            'rear_left_wheel_velocity': self.rear_left_wheel_velocity,
            'rear_right_wheel_velocity': self.rear_right_wheel_velocity,
            'rear_left_wheel_acceleration':
                self.rear_left_wheel_acceleration,
            'rear_right_wheel_acceleration':
                self.rear_right_wheel_acceleration,
            'zero_linear_epsilon': self.zero_linear_epsilon,
        }
        for name, value in positive_values.items():
            _finite_positive(name, value)

        steering_limits = (
            (
                'front_left',
                self.front_left_steering_lower,
                self.front_left_steering_upper,
            ),
            (
                'front_right',
                self.front_right_steering_lower,
                self.front_right_steering_upper,
            ),
        )
        for name, lower, upper in steering_limits:
            if not all(math.isfinite(value) for value in (lower, upper)):
                raise ValueError(f'{name} steering limits must be finite')
            if lower >= upper:
                raise ValueError(
                    f'{name} steering lower limit must be below upper limit'
                )
            if lower > 0.0 or upper < 0.0:
                raise ValueError(
                    f'{name} steering limits must contain zero'
                )

        if self.max_projection_iterations < 4:
            raise ValueError('max_projection_iterations must be at least 4')

        # Keeping the ICR outside the steering axle avoids the atan2 branch
        # discontinuity and is also required by this ideal Ackermann model.
        half_steering_track = 0.5 * self.steering_track_width
        if min(self.r_min_left, self.r_min_right) <= half_steering_track:
            raise ValueError(
                'minimum turning radii must exceed half steering track'
            )


@dataclass(frozen=True)
class GuardState:
    """The last command emitted by the guard."""

    linear: float
    curvature: float
    rear_left_velocity: float
    rear_right_velocity: float
    front_left_steering: float
    front_right_steering: float

    @classmethod
    def stopped(cls) -> 'GuardState':
        """Return a fully stopped vehicle state."""
        return cls(
            linear=0.0,
            curvature=0.0,
            rear_left_velocity=0.0,
            rear_right_velocity=0.0,
            front_left_steering=0.0,
            front_right_steering=0.0,
        )


@dataclass(frozen=True)
class ProjectionResult:
    """A safe body twist and the state needed for the next projection."""

    linear: float
    angular: float
    state: GuardState
    limited: bool
    reason: str


def ackermann_steering_angles(
    curvature: float,
    limits: GuardLimits,
) -> Tuple[float, float]:
    """Return left and right steering targets for rear-centre curvature."""
    numerator = limits.rolling_wheelbase * curvature
    half_track_term = 0.5 * limits.steering_track_width * curvature
    left = math.atan2(numerator, 1.0 - half_track_term)
    right = math.atan2(numerator, 1.0 + half_track_term)
    return left, right


def rear_wheel_velocities(
    linear: float,
    curvature: float,
    limits: GuardLimits,
) -> Tuple[float, float]:
    """Return ideal left and right rear-wheel angular velocities."""
    angular = linear * curvature
    half_track = 0.5 * limits.traction_track_width
    left = (linear - angular * half_track) / limits.traction_wheels_radius
    right = (linear + angular * half_track) / limits.traction_wheels_radius
    return left, right


def timestamp_is_fresh(
    now_seconds: float,
    stamp_seconds: float,
    max_input_age: float,
    future_tolerance: float,
    *,
    zero_stamp: bool = False,
) -> Tuple[bool, str]:
    """Validate a command stamp against a ROS/simulation-clock time."""
    values = (
        now_seconds,
        stamp_seconds,
        max_input_age,
        future_tolerance,
    )
    if not all(math.isfinite(value) for value in values):
        return False, 'non_finite_time'
    if max_input_age < 0.0 or future_tolerance < 0.0:
        return False, 'invalid_tolerance'
    if zero_stamp:
        return True, 'zero_stamp_uses_receive_time'

    age = now_seconds - stamp_seconds
    if age > max_input_age + _TOL:
        return False, 'stale_stamp'
    if age < -future_tolerance - _TOL:
        return False, 'future_stamp'
    return True, 'fresh'


def _curvature_bounds(limits: GuardLimits) -> Tuple[float, float]:
    return -1.0 / limits.r_min_right, 1.0 / limits.r_min_left


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _steering_position_feasible(
    curvature: float,
    limits: GuardLimits,
) -> bool:
    left, right = ackermann_steering_angles(curvature, limits)
    return (
        limits.front_left_steering_lower - _TOL
        <= left
        <= limits.front_left_steering_upper + _TOL
        and limits.front_right_steering_lower - _TOL
        <= right
        <= limits.front_right_steering_upper + _TOL
    )


def _project_steering_position(
    target_curvature: float,
    limits: GuardLimits,
) -> float:
    if _steering_position_feasible(target_curvature, limits):
        return target_curvature

    # Zero is guaranteed feasible by GuardLimits validation.  Search on the
    # segment to the requested curvature, retaining the largest feasible part.
    low = 0.0
    high = 1.0
    for _ in range(limits.max_projection_iterations):
        fraction = 0.5 * (low + high)
        candidate = fraction * target_curvature
        if _steering_position_feasible(candidate, limits):
            low = fraction
        else:
            high = fraction
    return low * target_curvature


def _steering_rate_feasible(
    previous_curvature: float,
    candidate_curvature: float,
    dt: float,
    limits: GuardLimits,
) -> bool:
    previous_left, previous_right = ackermann_steering_angles(
        previous_curvature, limits
    )
    candidate_left, candidate_right = ackermann_steering_angles(
        candidate_curvature, limits
    )
    return (
        abs(candidate_left - previous_left)
        <= limits.front_left_steering_velocity * dt + _TOL
        and abs(candidate_right - previous_right)
        <= limits.front_right_steering_velocity * dt + _TOL
    )


def _project_steering_rate(
    previous_curvature: float,
    target_curvature: float,
    dt: float,
    limits: GuardLimits,
) -> float:
    if dt <= 0.0:
        return previous_curvature
    if _steering_rate_feasible(
        previous_curvature, target_curvature, dt, limits
    ):
        return target_curvature

    low = 0.0
    high = 1.0
    delta = target_curvature - previous_curvature
    for _ in range(limits.max_projection_iterations):
        fraction = 0.5 * (low + high)
        candidate = previous_curvature + fraction * delta
        if _steering_rate_feasible(
            previous_curvature, candidate, dt, limits
        ):
            low = fraction
        else:
            high = fraction
    return previous_curvature + low * delta


def _intersect(
    current: Tuple[float, float],
    addition: Tuple[float, float],
) -> Optional[Tuple[float, float]]:
    lower = max(current[0], addition[0])
    upper = min(current[1], addition[1])
    if lower > upper + _TOL:
        return None
    return lower, upper


def _linear_interval_for_wheel(
    coefficient: float,
    previous_velocity: float,
    velocity_limit: float,
    acceleration_limit: float,
    dt: float,
) -> Optional[Tuple[float, float]]:
    interval = (-math.inf, math.inf)
    if abs(coefficient) <= _EPS:
        if abs(previous_velocity) > acceleration_limit * dt + _TOL:
            return None
        return interval

    speed_extent = velocity_limit / abs(coefficient)
    interval = (-speed_extent, speed_extent)

    delta = acceleration_limit * dt
    endpoint_a = (previous_velocity - delta) / coefficient
    endpoint_b = (previous_velocity + delta) / coefficient
    acceleration_interval = (
        min(endpoint_a, endpoint_b),
        max(endpoint_a, endpoint_b),
    )
    return _intersect(interval, acceleration_interval)


def _velocity_interval(
    curvature: float,
    previous: GuardState,
    dt: float,
    limits: GuardLimits,
) -> Optional[Tuple[float, float]]:
    radius = limits.traction_wheels_radius
    half_track_term = 0.5 * limits.traction_track_width * curvature
    left_coefficient = (1.0 - half_track_term) / radius
    right_coefficient = (1.0 + half_track_term) / radius

    left_interval = _linear_interval_for_wheel(
        left_coefficient,
        previous.rear_left_velocity,
        limits.rear_left_wheel_velocity,
        limits.rear_left_wheel_acceleration,
        dt,
    )
    if left_interval is None:
        return None
    right_interval = _linear_interval_for_wheel(
        right_coefficient,
        previous.rear_right_velocity,
        limits.rear_right_wheel_velocity,
        limits.rear_right_wheel_acceleration,
        dt,
    )
    if right_interval is None:
        return None
    return _intersect(left_interval, right_interval)


def feasibility_violations(
    linear: float,
    curvature: float,
    previous: GuardState,
    dt: float,
    limits: GuardLimits,
) -> Tuple[str, ...]:
    """Return all hard-constraint violations for an emitted command."""
    violations = []
    if not all(math.isfinite(value) for value in (linear, curvature, dt)):
        return ('non_finite',)
    if dt <= 0.0:
        return ('non_positive_dt',)

    lower_curvature, upper_curvature = _curvature_bounds(limits)
    if not lower_curvature - _TOL <= curvature <= upper_curvature + _TOL:
        violations.append('curvature')

    left_steer, right_steer = ackermann_steering_angles(
        curvature, limits
    )
    if not (
        limits.front_left_steering_lower - _TOL
        <= left_steer
        <= limits.front_left_steering_upper + _TOL
    ):
        violations.append('left_steering_position')
    if not (
        limits.front_right_steering_lower - _TOL
        <= right_steer
        <= limits.front_right_steering_upper + _TOL
    ):
        violations.append('right_steering_position')

    previous_left = previous.front_left_steering
    previous_right = previous.front_right_steering
    if (
        abs(left_steer - previous_left)
        > limits.front_left_steering_velocity * dt + _TOL
    ):
        violations.append('left_steering_rate')
    if (
        abs(right_steer - previous_right)
        > limits.front_right_steering_velocity * dt + _TOL
    ):
        violations.append('right_steering_rate')

    rear_left, rear_right = rear_wheel_velocities(
        linear, curvature, limits
    )
    if abs(rear_left) > limits.rear_left_wheel_velocity + _TOL:
        violations.append('left_rear_velocity')
    if abs(rear_right) > limits.rear_right_wheel_velocity + _TOL:
        violations.append('right_rear_velocity')
    if (
        abs(rear_left - previous.rear_left_velocity)
        > limits.rear_left_wheel_acceleration * dt + _TOL
    ):
        violations.append('left_rear_acceleration')
    if (
        abs(rear_right - previous.rear_right_velocity)
        > limits.rear_right_wheel_acceleration * dt + _TOL
    ):
        violations.append('right_rear_acceleration')
    return tuple(violations)


def _state_from_command(
    linear: float,
    curvature: float,
    limits: GuardLimits,
) -> GuardState:
    rear_left, rear_right = rear_wheel_velocities(
        linear, curvature, limits
    )
    front_left, front_right = ackermann_steering_angles(
        curvature, limits
    )
    return GuardState(
        linear=linear,
        curvature=curvature,
        rear_left_velocity=rear_left,
        rear_right_velocity=rear_right,
        front_left_steering=front_left,
        front_right_steering=front_right,
    )


def stopped_result(
    *,
    limited: bool,
    reason: str,
) -> ProjectionResult:
    """Create the immediate-stop result used by safety and watchdog paths."""
    return ProjectionResult(
        linear=0.0,
        angular=0.0,
        state=GuardState.stopped(),
        limited=limited,
        reason=reason,
    )


def project_command(
    desired_linear: float,
    desired_angular: float,
    previous: GuardState,
    dt: float,
    limits: GuardLimits,
) -> ProjectionResult:
    """Project a body twist into the simultaneous feasible set.

    Requested stops are intentionally immediate.  For a moving command, the
    function first projects curvature and steering rate, then intersects the
    two rear-wheel velocity and acceleration intervals.  If that intersection
    is empty at the requested curvature, curvature is moved toward the last
    emitted value with a bounded search.  The returned moving command is
    checked again against every hard constraint before it is accepted.
    """
    if not all(
        math.isfinite(value)
        for value in (desired_linear, desired_angular, dt)
    ):
        return stopped_result(limited=True, reason='non_finite')
    if abs(desired_linear) < limits.zero_linear_epsilon:
        return stopped_result(
            limited=abs(desired_linear) > 0.0
            or abs(desired_angular) > 0.0,
            reason='zero_linear',
        )
    if dt <= 0.0:
        return stopped_result(limited=True, reason='non_positive_dt')

    lower_curvature, upper_curvature = _curvature_bounds(limits)
    desired_curvature = desired_angular / desired_linear
    target_curvature = _clamp(
        desired_curvature, lower_curvature, upper_curvature
    )
    target_curvature = _project_steering_position(
        target_curvature, limits
    )
    target_curvature = _project_steering_rate(
        previous.curvature, target_curvature, dt, limits
    )

    interval = _velocity_interval(
        target_curvature, previous, dt, limits
    )
    selected_curvature = target_curvature

    if interval is None:
        # Search from the target toward the known-feasible previous state.
        # A descending grid avoids relying solely on monotonicity; bisection
        # then recovers precision at the first feasible boundary.
        delta = target_curvature - previous.curvature
        feasible_fraction = 0.0
        feasible_interval = _velocity_interval(
            previous.curvature, previous, dt, limits
        )
        if feasible_interval is None:
            return stopped_result(
                limited=True, reason='invalid_previous_state'
            )

        infeasible_fraction = 1.0
        iteration_count = limits.max_projection_iterations
        for index in range(1, iteration_count + 1):
            fraction = 1.0 - index / iteration_count
            candidate_curvature = previous.curvature + fraction * delta
            candidate_interval = _velocity_interval(
                candidate_curvature, previous, dt, limits
            )
            if candidate_interval is not None:
                feasible_fraction = fraction
                feasible_interval = candidate_interval
                infeasible_fraction = min(
                    1.0, fraction + 1.0 / iteration_count
                )
                break

        for _ in range(iteration_count):
            fraction = 0.5 * (
                feasible_fraction + infeasible_fraction
            )
            candidate_curvature = previous.curvature + fraction * delta
            candidate_interval = _velocity_interval(
                candidate_curvature, previous, dt, limits
            )
            if candidate_interval is None:
                infeasible_fraction = fraction
            else:
                feasible_fraction = fraction
                feasible_interval = candidate_interval

        selected_curvature = (
            previous.curvature + feasible_fraction * delta
        )
        interval = feasible_interval

    assert interval is not None
    selected_linear = _clamp(
        desired_linear, interval[0], interval[1]
    )
    state = _state_from_command(
        selected_linear, selected_curvature, limits
    )
    violations = feasibility_violations(
        selected_linear, selected_curvature, previous, dt, limits
    )
    if violations:
        return stopped_result(
            limited=True,
            reason='projection_failed:' + ','.join(violations),
        )

    angular = selected_linear * selected_curvature
    limited = not (
        math.isclose(
            selected_linear,
            desired_linear,
            rel_tol=1.0e-9,
            abs_tol=1.0e-10,
        )
        and math.isclose(
            angular,
            desired_angular,
            rel_tol=1.0e-9,
            abs_tol=1.0e-10,
        )
    )
    return ProjectionResult(
        linear=selected_linear,
        angular=angular,
        state=state,
        limited=limited,
        reason='limited' if limited else 'accepted',
    )
