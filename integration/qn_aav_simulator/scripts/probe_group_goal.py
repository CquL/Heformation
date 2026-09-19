#!/usr/bin/env python3
"""Stage-1b probe: does a group command reach all three members correctly?

Runs against ``formation_aav3.launch``.  One formation centre is published to the
three per-member goal topics; each member must end at

    centre + scale * slot_i

with the three-member slots from ``config/formation_aav3.yaml`` (a line abreast
2 m apart).  It also checks that each member has exactly one qn publishing its
odometry, i.e. one authoritative dynamics source, and that the members do NOT all
share a single goal topic (which is what made subset commands impossible before).
"""

from __future__ import annotations

import math
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

AGENTS = (0, 1, 2)
CENTRE = (-24.0, 6.0, 0.8)
SCALE = 1.0
SLOTS = {0: (0.0, 0.0, 0.0), 1: (0.0, -2.0, 0.0), 2: (0.0, 2.0, 0.0)}
TOL = 0.5

state = {a: None for a in AGENTS}


def on_odom(msg, agent):
    p = msg.pose.pose.position
    state[agent] = (p.x, p.y, p.z)


rospy.init_node("probe_group_goal", anonymous=True)
for agent in AGENTS:
    rospy.Subscriber("/drone_{}_qn/odometry".format(agent), Odometry, on_odom,
                     callback_args=agent)
publishers = {a: rospy.Publisher("/drone_{}_formation_goal".format(a), PoseStamped,
                                 queue_size=1) for a in AGENTS}

deadline = time.time() + 150
while time.time() < deadline and any(state[a] is None for a in AGENTS):
    time.sleep(0.5)
missing = [a for a in AGENTS if state[a] is None]
if missing:
    print("FAIL: no odometry from", missing, flush=True)
    sys.exit(1)
print("before:", {a: tuple(round(v, 3) for v in state[a]) for a in AGENTS}, flush=True)

expected = {a: tuple(CENTRE[i] + SCALE * SLOTS[a][i] for i in range(3)) for a in AGENTS}
print("publishing centre", CENTRE, "expected endpoints:", expected, flush=True)

end = time.time() + 4
while time.time() < end and not rospy.is_shutdown():
    for agent in AGENTS:
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = CENTRE
        msg.pose.orientation.w = 1.0
        publishers[agent].publish(msg)
    time.sleep(0.2)

print("centre sent; observing 60 s", flush=True)
time.sleep(60)

print("\n=== result ===")
ok = True
for agent in AGENTS:
    actual = state[agent]
    error = math.dist(actual, expected[agent])
    verdict = "PASS" if error < TOL else "FAIL"
    ok = ok and error < TOL
    print("drone_%d expected (%.3f, %.3f, %.3f) actual (%.3f, %.3f, %.3f)  error %.3f m  %s"
          % (agent, expected[agent][0], expected[agent][1], expected[agent][2],
             actual[0], actual[1], actual[2], error, verdict))

print("\n=== one authoritative qn per member ===")
for agent in AGENTS:
    topic = "/drone_{}_qn/odometry".format(agent)
    try:
        out = subprocess.run(["rostopic", "info", topic], capture_output=True,
                             text=True, timeout=20).stdout
        # rostopic marks both publishers and subscribers with "*", so only the
        # lines under the Publishers heading count.
        pubs = []
        section = None
        for line in out.splitlines():
            stripped = line.strip()
            if stripped.startswith("Publishers"):
                section = "publishers"
                continue
            if stripped.startswith("Subscribers"):
                section = "subscribers"
                continue
            if section == "publishers" and stripped.startswith("*"):
                pubs.append(stripped)
        verdict = "PASS" if len(pubs) == 1 else "FAIL"
        ok = ok and len(pubs) == 1
        print("%s publishers=%d %s" % (topic, len(pubs), verdict))
    except Exception as error:
        print("%s could not query (%s) NOT_VERIFIED" % (topic, error))
        ok = False

print("\nGROUP PROBE:", "PASS" if ok else "FAIL")
