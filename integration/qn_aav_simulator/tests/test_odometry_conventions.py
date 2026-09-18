"""Deterministic coordinate-counter-examples for the qn odometry boundary.

These are the tests plan.md asks for instead of more field checks: one pure-axis
case with the identity attitude, and non-zero roll/pitch/yaw cases, checked
against the position derivative and the attitude derivative.
"""

import math
import random

import pytest

from qn_aav_simulator.formation_monitor import OdometrySample
from qn_aav_simulator.odometry import (
    body_velocity_from_world, conventions_consistent, normalize_quaternion,
    ros_odometry_fields, rotation_from_quaternion, world_velocity_from_body,
)
from qn_aav_simulator.qn_dynamics import body_to_map_velocity


def _rotation(q):
    return rotation_from_quaternion(q)


def _matmul(matrix, vector):
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3))
                 for row in range(3))


def _transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _matmul_matrix(first, second):
    return tuple(
        tuple(sum(first[row][inner] * second[inner][column] for inner in range(3))
              for column in range(3))
        for row in range(3))


def _quaternion_from_axis_angle(axis, angle):
    norm = math.sqrt(sum(component * component for component in axis))
    x, y, z = (component / norm for component in axis)
    half = 0.5 * angle
    return (math.cos(half), x * math.sin(half), y * math.sin(half), z * math.sin(half))


def test_identity_attitude_publishes_the_world_velocity_as_the_body_twist():
    """Pure vertical motion with a level attitude.

    The model's *stored* body heave has the opposite sign of the real one; the
    published twist must carry the real one, and R(identity) must map it back to
    the world velocity.
    """
    world_velocity = (0.0, 0.0, -0.8)
    fields = ros_odometry_fields((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0),
                                 world_velocity, (0.0, 0.0, 0.0))
    assert fields.linear_body_velocity == pytest.approx(world_velocity)
    assert world_velocity_from_body(fields.linear_body_velocity,
                                    fields.orientation_quat_wxyz) == \
        pytest.approx(world_velocity)


def test_published_twist_reproduces_the_position_derivative_for_any_attitude():
    random.seed(11)
    for _ in range(200):
        quaternion = normalize_quaternion(
            tuple(random.uniform(-1, 1) for _ in range(4)))
        world_velocity = tuple(random.uniform(-2, 2) for _ in range(3))
        fields = ros_odometry_fields((0.0, 0.0, 0.5), quaternion, world_velocity,
                                     (0.1, -0.2, 0.3))
        recovered = world_velocity_from_body(fields.linear_body_velocity,
                                             fields.orientation_quat_wxyz)
        assert recovered == pytest.approx(world_velocity, abs=1e-12)


def test_non_zero_attitude_is_not_a_plain_sign_flip():
    """With pitch, the published twist is R(q)^T v, not S v."""
    quaternion = _quaternion_from_axis_angle((0.0, 1.0, 0.0), math.radians(30.0))
    world_velocity = (1.0, 0.0, 0.0)
    fields = ros_odometry_fields((0.0, 0.0, 0.5), quaternion, world_velocity,
                                 (0.0, 0.0, 0.0))
    sign_flip = (world_velocity[0], world_velocity[1], -world_velocity[2])
    assert fields.linear_body_velocity != pytest.approx(sign_flip)
    assert fields.linear_body_velocity == pytest.approx(
        body_velocity_from_world(world_velocity, quaternion))
    # Magnitude is preserved by the rotation.
    assert math.dist(fields.linear_body_velocity, (0.0, 0.0, 0.0)) == \
        pytest.approx(math.dist(world_velocity, (0.0, 0.0, 0.0)))


def test_model_body_twist_keeps_its_documented_heave_sign():
    """The model relation stays S*R(q)*v_model: the frozen deviation."""
    quaternion = _quaternion_from_axis_angle((1.0, 2.0, -0.5), math.radians(40.0))
    body = (0.3, -0.4, 0.6)
    model_world = body_to_map_velocity(body, quaternion)
    rotated = _matmul(_rotation(quaternion), body)
    assert model_world == pytest.approx(
        (rotated[0], rotated[1], -rotated[2]), abs=1e-12)


def test_published_fields_match_the_model_world_velocity():
    """The publisher consumes the model's world velocity unchanged."""
    quaternion = _quaternion_from_axis_angle((0.0, 0.0, 1.0), math.radians(75.0))
    body = (0.5, 0.2, -0.7)
    model_world = body_to_map_velocity(body, quaternion)
    fields = ros_odometry_fields((0.0, 0.0, 0.5), quaternion, model_world,
                                 (0.0, 0.0, 0.0))
    assert fields.world_velocity == pytest.approx(model_world)
    assert world_velocity_from_body(fields.linear_body_velocity,
                                    quaternion) == pytest.approx(model_world)


def test_angular_velocity_follows_the_attitude_derivative():
    """dR/dt = R [omega]x for a constant body rate about a tilted axis."""
    rate = 0.7
    raw_axis = (0.3, -0.5, 0.81)
    length = math.sqrt(sum(component * component for component in raw_axis))
    axis = tuple(component / length for component in raw_axis)
    quaternion = _quaternion_from_axis_angle((1.0, 1.0, 0.0), math.radians(25.0))
    fields = ros_odometry_fields((0.0, 0.0, 0.5), quaternion, (0.0, 0.0, 0.0),
                                 tuple(rate * component for component in axis))
    delta = 1e-6
    quarter = _quaternion_from_axis_angle(axis, rate * delta)
    # q(t+dt) = q * dq for a body-frame rate.
    def multiply(first, second):
        w1, x1, y1, z1 = first
        w2, x2, y2, z2 = second
        return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)

    rotated = normalize_quaternion(multiply(fields.orientation_quat_wxyz, quarter))
    current = _rotation(fields.orientation_quat_wxyz)
    later = _rotation(rotated)
    derivative = tuple(tuple((later[row][column] - current[row][column]) / delta
                             for column in range(3)) for row in range(3))
    body_rate = _matmul_matrix(_transpose(current), derivative)
    recovered = ((body_rate[2][1] - body_rate[1][2]) / 2.0,
                 (body_rate[0][2] - body_rate[2][0]) / 2.0,
                 (body_rate[1][0] - body_rate[0][1]) / 2.0)
    assert recovered == pytest.approx(fields.angular_body_velocity, abs=1e-6)


def _sample(velocity, stamp=1.0):
    return OdometrySample(
        stamp=stamp, position=(0.0, 0.0, 0.5), velocity=velocity,
        frame_id="world", body_velocity=(0.0, 0.0, 0.0),
        orientation_quat_wxyz=(1.0, 0.0, 0.0, 0.0), child_frame_id="drone_0/base_link")


def test_consistency_check_rejects_a_sign_flipped_axis():
    """Equal norms, opposite motion: the norm comparison used to pass this."""
    standard = _sample((0.0, 0.0, 1.0))
    flipped = _sample((0.0, 0.0, -1.0))
    assert not conventions_consistent(standard, flipped)
    assert conventions_consistent(standard, _sample((0.0, 0.0, 1.0)))


def test_consistency_check_requires_the_same_stamp():
    assert not conventions_consistent(_sample((1.0, 0.0, 0.0), stamp=1.0),
                                      _sample((1.0, 0.0, 0.0), stamp=2.0))
