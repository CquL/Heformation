#!/usr/bin/env python3
"""Does the selected executor control exactly its own members?

Runs against ``formation_aav3.launch`` and drives the real Action interface, not
the goal topics directly:

  1. send a goal to ``/aav_2/formation_action``, whose unit owns drone_1 only,
     and check that drone_1 moves to the commanded position while the other two
     do not move at all;
  2. send a goal to ``/aav_formation/formation_action``, whose unit owns all
     three, and check that every member ends at centre + its own slot.

The second step is what shows a group unit still works once subset commands
exist, and the first is what shows the plan's "the endpoint follows the selected
executor" claim.
"""

from __future__ import annotations

import math
import sys
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from qn_aav_simulator.msg import FormationAction, FormationGoal

AGENTS = (0, 1, 2)
SLOTS = {0: (0.0, 0.0, 0.0), 1: (0.0, -2.0, 0.0), 2: (0.0, 2.0, 0.0)}
SCALE = 1.0
TOL = 0.5
STANDBY_TOL = 0.05

state = {a: None for a in AGENTS}


def on_odom(msg, agent):
    p = msg.pose.pose.position
    state[agent] = (p.x, p.y, p.z)


def wait_for_state(timeout=180.0):
    deadline = time.time() + timeout
    while time.time() < deadline and any(state[a] is None for a in AGENTS):
        time.sleep(0.5)
    return all(state[a] is not None for a in AGENTS)


def send(endpoint, centre, hold=1.0, timeout=240.0):
    client = actionlib.SimpleActionClient(endpoint, FormationAction)
    if not client.wait_for_server(rospy.Duration(60.0)):
        print("FAIL: no action server at", endpoint, flush=True)
        return None
    goal = FormationGoal()
    goal.task_id = "probe-" + endpoint.strip("/").replace("/", "-")
    goal.formation_center.header.frame_id = "world"
    goal.formation_center.header.stamp = rospy.Time.now()
    goal.formation_center.point.x, goal.formation_center.point.y, \
        goal.formation_center.point.z = centre
    goal.hold_duration = rospy.Duration.from_sec(hold)
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(timeout)):
        print("FAIL: no result from", endpoint, flush=True)
        return None
    return client.get_state()


rospy.init_node("probe_action_routing", anonymous=True)
for agent in AGENTS:
    rospy.Subscriber("/drone_{}_qn/odometry".format(agent), Odometry, on_odom,
                     callback_args=agent)
if not wait_for_state():
    print("FAIL: odometry missing", flush=True)
    sys.exit(1)

before = {a: state[a] for a in AGENTS}
print("before:", {a: tuple(round(v, 3) for v in before[a]) for a in AGENTS}, flush=True)

ok = True

# --- 1. a single-member unit -------------------------------------------------
single_target = (before[1][0] + 1.5, before[1][1], 0.8)
print("\n[1] /aav_2/formation_action (owns drone_1 only) ->", single_target, flush=True)
state_result = send("/aav_2/formation_action", single_target)
print("    terminal state:", state_result, flush=True)
if state_result != GoalStatus.SUCCEEDED:
    print("    NOTE: goal did not report SUCCEEDED; positions are still checked below",
          flush=True)
time.sleep(2.0)
after_single = {a: state[a] for a in AGENTS}
drone1_error = math.dist(after_single[1], single_target)
others = {a: math.dist(before[a][:2], after_single[a][:2]) for a in (0, 2)}
print("    drone_1 error to commanded position: %.3f m" % drone1_error, flush=True)
print("    drone_0/drone_2 displacement:", {a: round(v, 3) for a, v in others.items()}, flush=True)
if drone1_error >= TOL:
    ok = False
    print("    FAIL: the addressed member did not reach its target", flush=True)
if any(v > STANDBY_TOL for v in others.values()):
    ok = False
    print("    FAIL: an unaddressed member moved", flush=True)

# --- 2. the group unit -------------------------------------------------------
centre = (before[0][0] + 1.5, 6.0, 0.8)
expected = {a: tuple(centre[i] + SCALE * SLOTS[a][i] for i in range(3)) for a in AGENTS}
print("\n[2] /aav_formation/formation_action (owns all three) centre", centre,
      "expected", expected, flush=True)
state_result = send("/aav_formation/formation_action", centre)
print("    terminal state:", state_result, flush=True)
time.sleep(2.0)
after_group = {a: state[a] for a in AGENTS}
for agent in AGENTS:
    error = math.dist(after_group[agent], expected[agent])
    verdict = "PASS" if error < TOL else "FAIL"
    ok = ok and error < TOL
    print("    drone_%d expected (%.3f, %.3f, %.3f) actual (%.3f, %.3f, %.3f) error %.3f m %s"
          % (agent, expected[agent][0], expected[agent][1], expected[agent][2],
             after_group[agent][0], after_group[agent][1], after_group[agent][2],
             error, verdict))

print("\nACTION ROUTING PROBE:", "PASS" if ok else "FAIL")
