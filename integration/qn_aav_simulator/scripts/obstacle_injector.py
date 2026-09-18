#!/usr/bin/env python3
"""Publish one deterministic obstacle into the map the planners actually read.

plan.md M2 needs a controlled obstacle rather than a random forest: the same
scenario with and without it, so the change in the plan and in the flown
trajectory can be attributed to the obstacle.  The upstream renderer builds the
per-drone local cloud from ``/map_generator/global_cloud``, so adding a second
publisher on that topic puts the obstacle into the same path the planners
already consume (local cloud -> grid map -> trajectory optimisation) without
modifying any upstream package.
"""

from __future__ import annotations

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


def _cloud(frame_id, stamp, points):
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = True
    buffer = bytearray()
    for x, y, z in points:
        buffer.extend(struct.pack("<fff", x, y, z))
    message.data = bytes(buffer)
    return message


def main():
    rospy.init_node("obstacle_injector")
    topic = rospy.get_param("~topic", "/map_generator/global_cloud")
    frame_id = rospy.get_param("~frame_id", "world")
    center = [float(value) for value in rospy.get_param(
        "~center", [-23.0, 0.0, 0.5])]
    size = [float(value) for value in rospy.get_param("~size", [1.0, 1.0, 1.2])]
    resolution = float(rospy.get_param("~resolution", 0.2))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    if resolution <= 0.0 or rate_hz <= 0.0:
        raise ValueError("resolution and rate must be positive")

    points = []
    counts = [max(1, int(round(size[axis] / resolution))) for axis in range(3)]
    for index_x in range(counts[0]):
        for index_y in range(counts[1]):
            for index_z in range(counts[2]):
                points.append((
                    center[0] - 0.5 * size[0] + index_x * resolution,
                    center[1] - 0.5 * size[1] + index_y * resolution,
                    center[2] - 0.5 * size[2] + index_z * resolution,
                ))
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1)
    rospy.loginfo("obstacle injector: %d points around %s on %s",
                  len(points), center, topic)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        publisher.publish(_cloud(frame_id, rospy.Time.now(), points))
        rate.sleep()


if __name__ == "__main__":
    main()
