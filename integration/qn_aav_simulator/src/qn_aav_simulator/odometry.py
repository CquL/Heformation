"""Translation between the qn native state and the two ROS odometry outputs.

The standard qn Odometry carries world position/attitude and **body-frame**
linear and angular velocity.  Swarm-Formation instead consumes world-frame
linear velocity from ``twist.twist.linear``.  Both are published from one qn
state snapshot; this module performs the coordinate change at the ROS boundary
and refuses to guess when a message does not declare its convention.

Why the published body twist is not simply the model's body twist
----------------------------------------------------------------
The qn 12ODE position kinematics uses a sign-inverted third ("heave") row, so
the model's map velocity satisfies ``dp/dt = S R(q) v_model`` with
``S = diag(1, 1, -1)`` while the model's stored body twist stays ``v_model``.
``S R(q)`` has determinant -1, so no unit quaternion expresses it: the model's
body linear velocity cannot be published next to a right-handed ROS body frame
without contradicting ``dp/dt = R(q) v_body``.

Measured on a recorded run, the model's *attitude* and *body angular velocity*
are the physical ones (pitch is nose-down under forward acceleration, and the
body rate matches ``R(q)^T dR/dt``), while ``v_model`` is not the physical body
velocity.  The published standard Odometry therefore keeps the native attitude
and body rate and publishes the physical body-frame linear velocity
``R(q)^T v_world``, which reproduces ``dp/dt`` exactly.  No controller,
actuator or 6DOF equation is changed, and the dynamics stay untouched.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

from .formation_monitor import OdometrySample

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]

STANDARD_CHILD_SUFFIX = "/base_link"
SWARM_COMPAT_CHILD_SUFFIX = "/swarm_compat"


class OdometryContractError(ValueError):
    """The message does not declare the convention its consumer requires."""


@dataclass(frozen=True)
class RosOdometryFields:
    """The fields a standard ROS Odometry publishes for one qn state."""

    position: Vector3
    orientation_quat_wxyz: Quaternion
    linear_body_velocity: Vector3
    angular_body_velocity: Vector3
    world_velocity: Vector3


def rotation_from_quaternion(quaternion) -> Tuple[Vector3, Vector3, Vector3]:
    """Body-to-world rotation matrix ``R(q)`` for a wxyz quaternion."""
    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def body_velocity_from_world(world_velocity, quaternion) -> Vector3:
    """Express a world-frame velocity in the published body frame."""
    rotation = rotation_from_quaternion(quaternion)
    values = tuple(float(component) for component in world_velocity)
    return tuple(
        rotation[0][row] * values[0] + rotation[1][row] * values[1]
        + rotation[2][row] * values[2]
        for row in range(3)
    )  # type: ignore[return-value]


def ros_odometry_fields(position, quaternion, world_velocity,
                        body_angular_velocity) -> RosOdometryFields:
    """The single mapping from one qn state snapshot to ROS Odometry fields.

    The publisher calls this once per state; no consumer repeats it.  The
    published attitude and body rate are the physically measured ones, and the
    published body linear velocity is ``R(q)^T v_world`` so that a standard
    consumer recovers ``dp/dt = R(q) v_body`` exactly.
    """
    orientation = normalize_quaternion(quaternion)
    world = tuple(float(component) for component in world_velocity)
    if not all(math.isfinite(component) for component in world):
        raise OdometryContractError("world velocity must be finite")
    if len(position) != 3 or not all(math.isfinite(float(c)) for c in position):
        raise OdometryContractError("position must be three finite numbers")
    if len(body_angular_velocity) != 3:
        raise OdometryContractError("body angular velocity must have three components")
    return RosOdometryFields(
        position=tuple(float(component) for component in position),
        orientation_quat_wxyz=orientation,
        linear_body_velocity=body_velocity_from_world(world, orientation),
        angular_body_velocity=tuple(
            float(component) for component in body_angular_velocity),
        world_velocity=world,
    )


def normalize_quaternion(quaternion) -> Quaternion:
    values = tuple(float(component) for component in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise OdometryContractError("quaternion must be four finite numbers")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise OdometryContractError("quaternion norm is zero")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def world_velocity_from_body(body_velocity, quaternion) -> Vector3:
    """Rotate a body-frame velocity into the world frame."""
    vx, vy, vz = (float(component) for component in body_velocity)
    rotation = rotation_from_quaternion(quaternion)
    return tuple(
        rotation[row][0] * vx + rotation[row][1] * vy + rotation[row][2] * vz
        for row in range(3)
    )


def _position(message) -> Vector3:
    point = message.pose.pose.position
    return (float(point.x), float(point.y), float(point.z))


def _quaternion(message) -> Quaternion:
    orientation = message.pose.pose.orientation
    return (float(orientation.w), float(orientation.x),
            float(orientation.y), float(orientation.z))


def parse_standard_odometry(message, agent_id: str) -> OdometrySample:
    """Parse a standard qn Odometry (world pose, body twist)."""
    frame_id = str(message.header.frame_id)
    child_frame_id = str(message.child_frame_id)
    if frame_id != "world":
        raise OdometryContractError(
            "standard odometry frame_id must be world, got {!r}".format(frame_id))
    expected = agent_id + STANDARD_CHILD_SUFFIX
    if child_frame_id != expected:
        raise OdometryContractError(
            "standard odometry child_frame_id must be {!r}, got {!r}".format(
                expected, child_frame_id))
    twist = message.twist.twist
    body_velocity = (float(twist.linear.x), float(twist.linear.y),
                     float(twist.linear.z))
    quaternion = _quaternion(message)
    world_velocity = world_velocity_from_body(body_velocity, quaternion)
    return OdometrySample(
        stamp=float(message.header.stamp.to_sec()),
        position=_position(message),
        velocity=world_velocity,
        frame_id=frame_id,
        body_velocity=body_velocity,
        orientation_quat_wxyz=quaternion,
        child_frame_id=child_frame_id,
    )


def parse_swarm_compat_odometry(message, agent_id: str) -> OdometrySample:
    """Parse the Swarm compatibility input (world-frame linear velocity)."""
    frame_id = str(message.header.frame_id)
    child_frame_id = str(message.child_frame_id)
    if frame_id != "world":
        raise OdometryContractError(
            "swarm compatibility odometry frame_id must be world, got {!r}".format(
                frame_id))
    expected = agent_id + SWARM_COMPAT_CHILD_SUFFIX
    if child_frame_id != expected:
        raise OdometryContractError(
            "swarm compatibility odometry child_frame_id must be {!r}, got {!r}".format(
                expected, child_frame_id))
    twist = message.twist.twist
    world_velocity = (float(twist.linear.x), float(twist.linear.y),
                      float(twist.linear.z))
    return OdometrySample(
        stamp=float(message.header.stamp.to_sec()),
        position=_position(message),
        velocity=world_velocity,
        frame_id=frame_id,
        body_velocity=(0.0, 0.0, 0.0),
        orientation_quat_wxyz=_quaternion(message),
        child_frame_id=child_frame_id,
    )


def conventions_consistent(standard: OdometrySample,
                           compat: OdometrySample, tolerance: float = 1e-6) -> bool:
    """True when both publications agree on pose and on the world velocity vector.

    The comparison is on the three-dimensional velocity, not on its norm: a
    sign-flipped axis keeps the norm while describing the opposite motion, so a
    norm comparison cannot detect a wrong coordinate convention.
    """
    if standard.stamp != compat.stamp:
        return False
    if any(abs(a - b) > tolerance for a, b in zip(standard.position, compat.position)):
        return False
    return all(abs(a - b) <= tolerance
               for a, b in zip(standard.velocity, compat.velocity))
