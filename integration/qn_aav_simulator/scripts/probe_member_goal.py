#!/usr/bin/env python3
"""Stage-1 probe: does a goal addressed to one member land where it says?

Runs against ``probe_member_goal.launch`` (the ordinary seven-member stack with
a member-goal entry enabled on drone_2 only) and checks the three things the
member entry has to get right:

  * the member ends at the commanded position, so the formation slot offset was
    NOT added (drone_2's slot would shift it 4 m in y);
  * the members that were not addressed do not move;
  * the addressed member plans without waiting for a predecessor trajectory
    (which is implied by it arriving at all, since no predecessor has a goal).

Every threshold is declared here rather than derived from the run, and the
verdicts are printed so a failure says which of the three broke.
"""

import math, sys, time
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

AGENTS = range(7)
TOL = 0.05          # standby tolerance: other members must stay inside this
TARGET_DX = 1.5     # small move so the slot offset (4 m in y) is unmistakable

state = {a: None for a in AGENTS}


def on_odom(msg, agent):
    p = msg.pose.pose.position
    state[agent] = (msg.header.stamp.to_sec(), p.x, p.y, p.z)


rospy.init_node("probe_member_goal", anonymous=True)
for a in AGENTS:
    rospy.Subscriber("/drone_{}_qn/odometry".format(a), Odometry, on_odom, callback_args=a)
publisher = rospy.Publisher("/drone_2_member_goal", PoseStamped, queue_size=1)

deadline = time.time() + 120
while time.time() < deadline and any(state[a] is None for a in AGENTS):
    time.sleep(0.5)
missing = [a for a in AGENTS if state[a] is None]
if missing:
    print("FAIL: no odometry from", missing, flush=True)
    sys.exit(1)

before = {a: state[a][1:4] for a in AGENTS}
target = (before[2][0] + TARGET_DX, before[2][1], 0.8)
print("before drone_2 =", ["%.3f" % v for v in before[2]], flush=True)
print("publishing member goal", ["%.3f" % v for v in target], flush=True)

msg = PoseStamped()
msg.header.frame_id = "world"
msg.header.stamp = rospy.Time.now()
msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
msg.pose.orientation.w = 1.0
# publish for a while: latching is off, and the entry is a plain subscription
end = time.time() + 4
while time.time() < end and not rospy.is_shutdown():
    publisher.publish(msg)
    time.sleep(0.2)

print("goal sent; observing 60 s", flush=True)
time.sleep(60)

after = {a: state[a][1:4] for a in AGENTS}
final = after[2]
horizontal = math.dist(final[:2], target[:2])
slot_offset = abs(final[1] - target[1])
print("\n=== result ===")
print("drone_2 target   = (%.3f, %.3f, %.3f)" % target)
print("drone_2 actual   = (%.3f, %.3f, %.3f)" % tuple(final))
print("horizontal error to target = %.3f m" % horizontal)
print("y error (slot offset would be ~4 m) = %.3f m" % slot_offset)
moved = {a: math.dist(before[a][:2], after[a][:2]) for a in AGENTS if a != 2}
print("other members displacement =",
      {a: round(v, 3) for a, v in sorted(moved.items())})
print("\nVERDICT landing:", "PASS" if horizontal < 0.5 else "FAIL")
print("VERDICT no-slot-offset:", "PASS" if slot_offset < 0.5 else "FAIL")
print("VERDICT others-still:", "PASS" if all(v <= TOL for v in moved.values()) else "FAIL")
