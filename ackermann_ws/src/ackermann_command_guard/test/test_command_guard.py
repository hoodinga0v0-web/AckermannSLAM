"""Unit tests for pure Ackermann command-guard calculations."""

import math

from ackermann_command_guard.guard_core import GuardLimits
from ackermann_command_guard.guard_core import GuardState
from ackermann_command_guard.guard_core import ackermann_steering_angles
from ackermann_command_guard.guard_core import feasibility_violations
from ackermann_command_guard.guard_core import project_command
from ackermann_command_guard.guard_core import timestamp_is_fresh

import pytest


@pytest.fixture
def limits():
    """Return asymmetric limits that exercise every projection axis."""
    return GuardLimits(
        rolling_wheelbase=1.0,
        steering_track_width=0.6,
        traction_track_width=0.5,
        traction_wheels_radius=0.2,
        r_min_left=1.5,
        r_min_right=2.0,
        front_left_steering_lower=-0.80,
        front_left_steering_upper=0.80,
        front_left_steering_velocity=0.8,
        front_right_steering_lower=-0.60,
        front_right_steering_upper=0.60,
        front_right_steering_velocity=0.7,
        rear_left_wheel_velocity=8.0,
        rear_right_wheel_velocity=7.0,
        rear_left_wheel_acceleration=4.0,
        rear_right_wheel_acceleration=3.0,
        zero_linear_epsilon=1.0e-4,
        max_projection_iterations=36,
    )


def assert_feasible(result, previous, dt, limits):
    """Assert that a projected result satisfies all simultaneous limits."""
    assert math.isfinite(result.linear)
    assert math.isfinite(result.angular)
    assert not feasibility_violations(
        result.linear,
        result.state.curvature,
        previous,
        dt,
        limits,
    )


def test_ackermann_angles_are_mirrored_for_opposite_curvature(limits):
    """Ackermann left/right targets should mirror across turn direction."""
    left_positive, right_positive = ackermann_steering_angles(
        0.4, limits
    )
    left_negative, right_negative = ackermann_steering_angles(
        -0.4, limits
    )
    assert left_positive == pytest.approx(-right_negative)
    assert right_positive == pytest.approx(-left_negative)
    assert left_positive > right_positive > 0.0


@pytest.mark.parametrize(
    ('linear', 'angular', 'expected_bound'),
    [
        (1.0, 10.0, 1.0 / 1.5),
        (1.0, -10.0, -1.0 / 2.0),
        (-1.0, -10.0, 1.0 / 1.5),
        (-1.0, 10.0, -1.0 / 2.0),
    ],
)
def test_direction_specific_curvature_limit(
    limits, linear, angular, expected_bound
):
    """Curvature should respect left/right limits in forward and reverse."""
    previous = GuardState.stopped()
    result = project_command(linear, angular, previous, 1.0, limits)
    assert result.state.curvature == pytest.approx(
        expected_bound, abs=1.0e-8
    )
    assert_feasible(result, previous, 1.0, limits)


def test_zero_linear_forces_zero_angular_immediately(limits):
    """Near-zero requested velocity should bypass ramping and stop."""
    previous_result = project_command(
        0.3, 0.1, GuardState.stopped(), 1.0, limits
    )
    result = project_command(
        1.0e-8, 12.0, previous_result.state, 0.01, limits
    )
    assert result.linear == 0.0
    assert result.angular == 0.0
    assert result.state == GuardState.stopped()


def test_non_finite_input_fails_closed(limits):
    """A non-finite command should fail closed to a zero output."""
    result = project_command(
        math.nan, 0.0, GuardState.stopped(), 0.01, limits
    )
    assert result.linear == 0.0
    assert result.angular == 0.0
    assert result.reason == 'non_finite'


def test_rear_wheel_velocity_is_projected(limits):
    """Rear wheel speed limits should bound body linear velocity."""
    fast_acceleration_limits = GuardLimits(
        **{
            **limits.__dict__,
            'rear_left_wheel_acceleration': 1.0e6,
            'rear_right_wheel_acceleration': 1.0e6,
        }
    )
    previous = GuardState.stopped()
    result = project_command(
        10.0, 2.0, previous, 1.0, fast_acceleration_limits
    )
    assert result.limited
    assert (
        abs(result.state.rear_left_velocity)
        <= fast_acceleration_limits.rear_left_wheel_velocity + 1.0e-9
    )
    assert (
        abs(result.state.rear_right_velocity)
        <= fast_acceleration_limits.rear_right_wheel_velocity + 1.0e-9
    )
    assert_feasible(
        result, previous, 1.0, fast_acceleration_limits
    )


def test_rear_wheel_acceleration_is_projected(limits):
    """Rear wheel acceleration should bound each fixed-rate step."""
    previous = GuardState.stopped()
    dt = 0.1
    result = project_command(2.0, 0.0, previous, dt, limits)
    assert result.limited
    assert abs(result.state.rear_left_velocity) <= (
        limits.rear_left_wheel_acceleration * dt + 1.0e-9
    )
    assert abs(result.state.rear_right_velocity) <= (
        limits.rear_right_wheel_acceleration * dt + 1.0e-9
    )
    assert_feasible(result, previous, dt, limits)


def test_steering_rate_is_projected(limits):
    """Both steering joint rate limits should constrain curvature steps."""
    previous = GuardState.stopped()
    dt = 0.02
    result = project_command(0.5, 0.5, previous, dt, limits)
    assert result.limited
    assert abs(result.state.front_left_steering) <= (
        limits.front_left_steering_velocity * dt + 1.0e-9
    )
    assert abs(result.state.front_right_steering) <= (
        limits.front_right_steering_velocity * dt + 1.0e-9
    )
    assert_feasible(result, previous, dt, limits)


def test_asymmetric_position_limit_is_projected(limits):
    """The tighter of two asymmetric steering bounds should prevail."""
    constrained = GuardLimits(
        **{
            **limits.__dict__,
            'front_left_steering_upper': 0.22,
            'front_left_steering_velocity': 100.0,
            'front_right_steering_velocity': 100.0,
            'rear_left_wheel_acceleration': 1.0e6,
            'rear_right_wheel_acceleration': 1.0e6,
        }
    )
    previous = GuardState.stopped()
    result = project_command(0.5, 5.0, previous, 1.0, constrained)
    assert result.state.front_left_steering <= 0.22 + 1.0e-8
    assert result.state.curvature < 1.0 / constrained.r_min_left
    assert_feasible(result, previous, 1.0, constrained)


def test_bounded_simultaneous_projection_over_command_grid(limits):
    """Every grid result should satisfy all constraints simultaneously."""
    state = GuardState.stopped()
    dt = 0.05
    for linear in (-2.0, -0.4, 0.2, 1.5):
        for angular in (-3.0, -0.2, 0.0, 0.4, 3.0):
            result = project_command(
                linear, angular, state, dt, limits
            )
            assert_feasible(result, state, dt, limits)
            state = result.state


def test_freshness_uses_sim_clock_and_accepts_zero_stamp():
    """Timestamp checks should reject stale/future but accept zero stamps."""
    assert timestamp_is_fresh(
        12.0, 0.0, 0.5, 0.05, zero_stamp=True
    )[0]
    assert timestamp_is_fresh(12.0, 11.6, 0.5, 0.05)[0]
    assert timestamp_is_fresh(12.0, 11.4, 0.5, 0.05) == (
        False,
        'stale_stamp',
    )
    assert timestamp_is_fresh(12.0, 12.1, 0.5, 0.05) == (
        False,
        'future_stamp',
    )


def test_invalid_limit_sets_are_rejected(limits):
    """Impossible geometry and steering ranges should be rejected."""
    with pytest.raises(ValueError, match='contain zero'):
        GuardLimits(
            **{
                **limits.__dict__,
                'front_left_steering_lower': 0.1,
            }
        )
    with pytest.raises(ValueError, match='half steering track'):
        GuardLimits(
            **{
                **limits.__dict__,
                'r_min_left': 0.2,
            }
        )
