"""Translation between the two qn odometry conventions.

The standard qn Odometry carries world position/attitude and **body-frame**
linear and angular velocity.  Swarm-Formation instead consumes world-frame
linear velocity from ``twist.twist.linear``.  Both are published from one qn
state snapshot; this module performs the coordinate change at the ROS boundary
and refuses to guess when a message does not declare its convention.
"""

from __future__ import annotations

import math
from typing import Tuple

from .formation_monitor import OdometrySample

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]

STANDARD_CHILD_SUFFIX = "/base_link"
SWARM_COMPAT_CHILD_SUFFIX = "/swarm_compat"


class OdometryContractError(ValueError):
    """The message does not declare the convention its consumer requires."""


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
    w, x, y, z = normalize_quaternion(quaternion)
    vx, vy, vz = (float(component) for component in body_velocity)
    # R(q) * v for wxyz quaternion.
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )
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
    """True when both publications agree on pose and the velocity norm."""
    if standard.stamp != compat.stamp:
        return False
    if any(abs(a - b) > tolerance for a, b in zip(standard.position, compat.position)):
        return False
    return abs(math.dist(standard.velocity, (0.0, 0.0, 0.0))
               - math.dist(compat.velocity, (0.0, 0.0, 0.0))) <= 1.0 + tolerance
