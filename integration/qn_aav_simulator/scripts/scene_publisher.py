#!/usr/bin/env python3
"""Publish the one complete scene map the planners actually consume.

The upstream CPU renderer builds each drone's local cloud from a single global
map.  This node is the only publisher on that scene topic, so the map is always
one complete, consistent description of the world instead of two publishers
overwriting each other.

The scene is deliberately minimal: an optional axis-aligned obstacle box, and
nothing else.  ``~obstacle`` only adds or removes that box, so the with/without
comparison differs in exactly one thing.  When the box is absent the map is an
explicitly empty cloud, which is a data state ("known to contain no obstacles"),
not a missing message.

The box centre and size are defined once and used for both the sampled cloud and
the analytic clearance the verifier reports.
"""

from __future__ import annotations

import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from qn_aav_simulator.experiment_verdict import box_sample_points

DEFAULT_TOPIC = "/scene/global_cloud"


def cloud_message(frame_id, stamp, points):
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
    for point in points:
        buffer.extend(struct.pack("<fff", *point))
    message.data = bytes(buffer)
    return message


def main():
    rospy.init_node("scene_publisher")
    scene = rospy.get_param("/scene", {})
    topic = rospy.get_param("~topic", scene.get("topic", DEFAULT_TOPIC))
    frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
    obstacle = bool(rospy.get_param("~obstacle", scene.get("obstacle_present", False)))
    center = [float(value) for value in rospy.get_param(
        "~center", scene.get("obstacle_center", [-23.0, 0.0, 0.5]))]
    size = [float(value) for value in rospy.get_param(
        "~size", scene.get("obstacle_size", [1.0, 1.0, 1.2]))]
    resolution = float(rospy.get_param("~resolution", scene.get("resolution", 0.2)))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    if resolution <= 0.0 or rate_hz <= 0.0:
        raise ValueError("resolution and rate must be positive")

    points = box_sample_points(center, size, resolution) if obstacle else []
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    rospy.loginfo("scene publisher: %d points on %s (obstacle=%s, frame=%s)",
                  len(points), topic, obstacle, frame_id)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        publisher.publish(cloud_message(frame_id, rospy.Time.now(), points))
        rate.sleep()


if __name__ == "__main__":
    main()
