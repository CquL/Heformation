#!/usr/bin/env python3
"""Action routing and lifecycle: single member -> group -> single member again.

Four verdicts are recorded **separately** for every segment, and the script exits
non-zero unless all of them hold:

  1. routing   - the goal reached only the intended members' goal topics
  2. landing   - the members ended where the command said they should
  3. result    - the Action reached SUCCEEDED and the client received the Result
  4. release   - the Result carries what the next dispatch needs

Landing correctly is not evidence that the action completed, so verdict 2 never
substitutes for verdict 3: a `no result` must fail the run.  When no result
arrives the probe reports where it stalled (client state plus the server's run
state) instead of silently extending the timeout.
"""

from __future__ import annotations

import math
import sys
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from qn_aav_simulator.msg import FormationAction, FormationGoal

AGENTS = (0, 1, 2)
SLOTS = {0: (0.0, 0.0, 0.0), 1: (0.0, -2.0, 0.0), 2: (0.0, 2.0, 0.0)}
SCALE = 1.0
TOL = 0.5
STANDBY_TOL = 0.05
HOLD_S = 2.0
RESULT_TIMEOUT_S = 150.0

STATE_NAMES = {0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED",
               4: "ABORTED", 5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING",
               8: "RECALLED", 9: "LOST"}

state = {a: None for a in AGENTS}
seen_topics = set()


def on_odom(msg, agent):
    p = msg.pose.pose.position
    state[agent] = (p.x, p.y, p.z)


def goal_topics():
    return {a: ["/drone_{}_formation_goal".format(a), "/drone_{}_member_goal".format(a)]
            for a in AGENTS}


def on_goal(_msg, topic):
    seen_topics.add(topic)


def wait_for_state(timeout=180.0):
    deadline = time.time() + timeout
    while time.time() < deadline and any(state[a] is None for a in AGENTS):
        time.sleep(0.5)
    return all(state[a] is not None for a in AGENTS)


def run_segment(name, endpoint, centre, intended, expected_landing, before):
    """Send one goal and record the four verdicts for it."""
    print("\n=== {} -> {} ===".format(name, endpoint), flush=True)
    # A single-member unit commands through the member entry; a group unit
    # commands a formation centre.  Either way the goal must reach exactly the
    # topics belonging to the members it named, and nothing else.
    if len(intended) == 1:
        expected_topics = {"/drone_{}_member_goal".format(intended[0])}
    else:
        expected_topics = {"/drone_{}_formation_goal".format(agent)
                           for agent in intended}
    seen_topics.clear()

    client = actionlib.SimpleActionClient(endpoint, FormationAction)
    if not client.wait_for_server(rospy.Duration(60.0)):
        return {"routing": False, "landing": False, "result": False,
                "release": False, "note": "no action server at " + endpoint}

    goal = FormationGoal()
    goal.task_id = name
    goal.formation_center.header.frame_id = "world"
    goal.formation_center.header.stamp = rospy.Time.now()
    goal.formation_center.point.x, goal.formation_center.point.y, \
        goal.formation_center.point.z = centre
    goal.hold_duration = rospy.Duration.from_sec(HOLD_S)
    client.send_goal(goal)

    got = client.wait_for_result(rospy.Duration(RESULT_TIMEOUT_S))
    terminal = client.get_state() if got else None
    result = client.get_result() if got else None
    note = ""
    if not got:
        # Locate the stall rather than extending the timeout: the client state
        # says whether the goal is still pending or running, and the server's own
        # run state says which phase it is in.
        node = "/" + endpoint.strip("/").split("/")[0] + "_action_server"
        run_state = rospy.get_param(node + "/run_state", "unknown")
        note = ("no result within {:.0f} s; client state={} server run_state={}"
                .format(RESULT_TIMEOUT_S, STATE_NAMES.get(client.get_state(), "?"),
                        run_state))

    time.sleep(2.0)
    routing = bool(seen_topics) and seen_topics == expected_topics
    unaddressed_moved = {a: math.dist(before[a][:2], state[a][:2])
                         for a in AGENTS if a not in intended}
    landing = True
    for agent in intended:
        error = math.dist(state[agent], expected_landing[agent])
        print("  drone_%d expected (%.3f, %.3f, %.3f) actual (%.3f, %.3f, %.3f) error %.3f m"
              % (agent, expected_landing[agent][0], expected_landing[agent][1],
                 expected_landing[agent][2], state[agent][0], state[agent][1],
                 state[agent][2], error), flush=True)
        landing = landing and error < TOL
    if unaddressed_moved and any(v > STANDBY_TOL for v in unaddressed_moved.values()):
        landing = False
        note = (note + "; " if note else "") + "unaddressed members moved: " + \
               str({a: round(v, 3) for a, v in unaddressed_moved.items()})

    completed = terminal == GoalStatus.SUCCEEDED
    # FormationActionResult carries the verdict fields directly (task_id,
    # task_outcome, ...).  "Usable for release" means the result identifies this
    # task and carries a terminal verdict, not merely that a message arrived.
    release = bool(result is not None
                   and getattr(result, "task_id", None) == name
                   and getattr(result, "task_outcome", 0) != 0)

    print("  topics reached: {} (expected subset of {})".format(
        sorted(seen_topics), sorted(expected_topics)), flush=True)
    print("  terminal={} result_received={}".format(
        STATE_NAMES.get(terminal, "None"), result is not None), flush=True)
    if note:
        print("  note: " + note, flush=True)
    return {"routing": routing, "landing": landing, "result": completed,
            "release": release, "note": note}


def main():
    rospy.init_node("probe_action_routing", anonymous=True)
    for agent in AGENTS:
        rospy.Subscriber("/drone_{}_qn/odometry".format(agent), Odometry, on_odom,
                         callback_args=agent)
        for topic in goal_topics()[agent]:
            rospy.Subscriber(topic, PoseStamped, on_goal, callback_args=topic)
    if not wait_for_state():
        print("FAIL: odometry missing", flush=True)
        return 1

    before = {a: state[a] for a in AGENTS}
    print("before:", {a: tuple(round(v, 3) for v in before[a]) for a in AGENTS},
          flush=True)

    segments = []

    single_target = (before[1][0] + 1.5, before[1][1], 0.8)
    segments.append(("single-aav2", "/aav_2/formation_action", single_target, (1,),
                     {1: single_target}))

    # Deliberately a centre every member has to travel to.  If a member were
    # already sitting on its slot the planner might emit no new trajectory, and a
    # task that cannot observe a new trajectory can never confirm adoption - which
    # is a different question from "is the formation term active".
    group_centre = (before[0][0] + 1.5, 10.0, 0.8)
    group_expected = {a: tuple(group_centre[i] + SCALE * SLOTS[a][i] for i in range(3))
                      for a in AGENTS}
    segments.append(("group-formation", "/aav_formation/formation_action",
                     group_centre, AGENTS, group_expected))

    before_third = {a: state[a] for a in AGENTS}
    third_target = (before_third[0][0] + 1.5, before_third[0][1], 0.8)
    segments.append(("single-aav1-again", "/aav_1/formation_action", third_target, (0,),
                     {0: third_target}))

    results = []
    for name, endpoint, centre, intended, expected in segments:
        snapshot = {a: state[a] for a in AGENTS}
        results.append((name, run_segment(name, endpoint, centre, intended, expected,
                                          snapshot)))

    print("\n=== summary ===")
    ok = True
    for name, verdict in results:
        passed = all(verdict[k] for k in ("routing", "landing", "result", "release"))
        ok = ok and passed
        print("{:<20} routing={} landing={} result={} release={}  {}".format(
            name, verdict["routing"], verdict["landing"], verdict["result"],
            verdict["release"], "PASS" if passed else "FAIL"))
    print("\nACTION ROUTING PROBE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
