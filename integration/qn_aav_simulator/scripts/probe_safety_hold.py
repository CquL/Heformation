#!/usr/bin/env python3
"""Run one declared safety-hold experiment on the three-AAV stack."""
import json
import math
import time
import subprocess
import sys
from pathlib import Path

import actionlib
import rospy
import rosnode
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import CompressedImage
from qn_aav_simulator.msg import FormationAction, FormationGoal, FormationActionGoal, FormationActionResult


def main():
    rospy.init_node("probe_safety_hold")
    output = Path(rospy.get_param("~output_dir", "/experiments/current"))
    mode = rospy.get_param("~mode", "single_cancel")
    supported = {"single_cancel", "single_cancel_accel", "single_cancel_near", "group_cancel",
                 "single_scan", "group_scan", "holding_scan", "idle_scan", "group_rpc_hang", "state_missing",
                 "request_cancel", "request_normal", "client_exit"}
    if mode not in supported:
        raise ValueError("unsupported probe mode: " + mode)
    states = {}
    safety, model = {}, {}
    def odom(message, member):
        p, v = message.pose.pose.position, message.twist.twist.linear
        states[member] = (p.x, p.y, p.z, math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))
    subscribers = [rospy.Subscriber("/drone_{}_qn/odometry".format(i), Odometry,
                                   odom, callback_args=i) for i in range(3)]
    def local_status(msg, member):
        safety[member] = {v.key:v.value for row in msg.status for v in row.values}
    def diagnostics(msg, member):
        model[member] = {v.key:v.value for row in msg.status for v in row.values}
    for i in range(3):
        subscribers.append(rospy.Subscriber("/drone_{}_planning/safety_status".format(i), DiagnosticArray, local_status, callback_args=i))
        subscribers.append(rospy.Subscriber("/drone_{}_qn/diagnostics".format(i), DiagnosticArray, diagnostics, callback_args=i))
    def wait_until(predicate, seconds, description):
        end = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError(description)
            time.sleep(.02)
    deadline = time.monotonic()+120
    while not rospy.is_shutdown():
        if len(states) == 3 and all(rospy.get_param("/"+u+"_action_server/ready", False)
                for u in ("aav_1", "aav_2", "aav_3", "aav_formation")):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("readiness timeout")
        time.sleep(.1)
    if mode == "client_exit":
        results = []
        subscribers.append(rospy.Subscriber("/aav_2/formation_action/result", FormationActionResult,
                                           lambda m: results.append((rospy.Time.now().to_sec(), m))))
        program = '''import rospy,actionlib
from qn_aav_simulator.msg import FormationAction,FormationGoal
rospy.init_node("exiting_safety_client")
c=actionlib.SimpleActionClient("/aav_2/formation_action",FormationAction)
assert c.wait_for_server(rospy.Duration(5))
g=FormationGoal();g.task_id="client_exit";g.formation_center.header.frame_id="world"
g.formation_center.point.x=-18;g.formation_center.point.y=4;g.formation_center.point.z=.8
g.hold_duration=rospy.Duration(4);c.send_goal(g)
input();c.cancel_goal();rospy.sleep(.1)
'''
        process = subprocess.Popen([sys.executable, "-c", program], stdin=subprocess.PIPE)
        wait_until(lambda: states[1][0]>=-27 and states[1][3]>.3, 60, "child client did not move")
        process.stdin.write(b"cancel\n");process.stdin.flush();process.stdin.close()
        wait_until(lambda: process.poll() is not None, 5, "child client did not exit")
        exited = rospy.Time.now().to_sec()
        wait_until(lambda: bool(results), 195, "server did not finish after client exit")
        received, terminal = results[-1]
        evidence = json.loads((output/terminal.result.evidence_file).read_text())
        checks = {"client_exited_before_result": exited < received,
                  "canceled_result": terminal.status.status == 2,
                  "hold_verified": evidence.get("safety_hold", {}).get("verified",False),
                  "resource_locked": not evidence["resource_released"],
                  "local_still_latched": safety[1]["latched"]=="true"}
        summary={"mode":mode,"checks":checks,"passed":all(checks.values()),
                 "client_exited_ros_s":exited,"result_received_ros_s":received}
        (output/"probe.json").write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
        return 0 if summary["passed"] else 1
    if mode.startswith("request_"):
        goals = []
        subscribers.append(rospy.Subscriber("/aav_2/formation_action/goal", FormationActionGoal,
                                           lambda msg: goals.append(msg.goal_id.id)))
        def task_state():
            return json.loads(rospy.get_param("/formation_mission_runner/task_state", "{}"))
        def frame(msg):
            state = task_state()
            action = state.get("current_action") or {}
            name = "live-{}-{}".format(state.get("status", "unknown"), action.get("phase", "none"))
            (output/(name+".jpg")).write_bytes(msg.data)
        subscribers.append(rospy.Subscriber("/mission_dashboard/image/compressed", CompressedImage, frame, queue_size=1))
        with (output/"runner.log").open("w") as log:
            process = subprocess.Popen(["rosrun", "qn_aav_simulator", "formation_mission_runner.py",
                "_planning_mode:=executor", "_request_file:=/workspace/src/src/qn_aav_simulator/config/monitoring_request_coastal.yaml"],
                stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
            # The user authorized this concrete implementation/verification batch.
            process.stdin.write(b"yes\n");process.stdin.flush();process.stdin.close()
            if mode == "request_cancel":
                wait_until(lambda: (task_state().get("current_action") or {}).get("task_id") == "zone_B-obs1"
                           and states[1][1] < 2 and bool(goals), 100, "second request action did not start")
                cancel = rospy.Publisher("/aav_2/formation_action/cancel", GoalID, queue_size=1)
                wait_until(lambda: cancel.get_num_connections()>0, 3, "cancel topic not connected")
                cancel.publish(GoalID(id=goals[-1]))
            wait_until(lambda: process.poll() is not None, 390, "request runner did not terminate")
        time.sleep(1.)  # retain a live final image after the authoritative result update
        metrics = json.loads((output/"metrics.json").read_text())
        if mode == "request_cancel":
            checks = {"runner_failed": process.returncode != 0,
                      "resource_locked": metrics["status"] == "UNKNOWN_LOCKED" and metrics["resource_locks"] == ["aav_2"],
                      "hold_verified": metrics.get("safety_disposition", {}).get("verified", False),
                      "failed_result_received": metrics["executions"][-1]["result"] == "NOT_ACCEPTED",
                      "successor_not_sent": len(metrics["executions"]) == 2}
        else:
            checks = {"request_pass": process.returncode == 0 and metrics["status"] == "PASS_GEOMETRIC_PROXY",
                      "all_results": len(metrics["executions"]) == 5,
                      "coverage": metrics["observed_fraction"] == metrics["delivered_fraction"] == 1.,
                      "released": not metrics["resource_locks"]}
        summary = {"mode": mode, "passed": all(checks.values()), "checks": checks}
        (output/"probe.json").write_text(json.dumps(summary, indent=2));print(json.dumps(summary), flush=True)
        return 0 if summary["passed"] else 1
    group = mode.startswith("group_")
    members = (0,1,2) if group else (1,)
    endpoint = "/aav_formation/formation_action" if group else "/aav_2/formation_action"
    client = actionlib.SimpleActionClient(endpoint, FormationAction)
    assert client.wait_for_server(rospy.Duration(5))
    goal = FormationGoal()
    goal.task_id = mode
    goal.formation_center.header.frame_id = "world"
    goal.formation_center.header.stamp = rospy.Time.now()
    goal.formation_center.point.x = -28.5 if mode in ("idle_scan", "holding_scan") else -18
    goal.formation_center.point.y = 6 if group else 4
    goal.formation_center.point.z = .8
    goal.hold_duration = rospy.Duration(4)
    phases = []
    client.send_goal(goal, feedback_cb=lambda f: phases.append(f.phase))
    if mode == "idle_scan":
        assert client.wait_for_result(rospy.Duration(60)) and client.get_state() == 3
    elif mode == "holding_scan":
        wait_until(lambda: 1 in phases, 60, "no terminal dwelling phase")
    else:
        threshold = -29.7 if mode == "single_cancel_accel" else -18.6 if mode == "single_cancel_near" else -27
        wait_until(lambda: states[1][0] >= threshold and states[1][3] > .15, 60, "no moving trigger state")
    trigger = {"at_ros_s": rospy.Time.now().to_sec(), "state": states[1]}
    if "scan" in mode:
        killed, failed = rosnode.kill_nodes(["/drone_1_pcl_render_node"])
        assert killed and not failed
        trigger["killed"] = killed
    else:
        if mode == "group_rpc_hang":
            def hung_service(_):
                time.sleep(120)
                return TriggerResponse(success=True, message="late test response")
            hung = rospy.Service("/drone_2_planning/safety_hold", Trigger, hung_service)
        client.cancel_goal()
        client.cancel_goal()
        if mode == "state_missing":
            rosnode.kill_nodes(["/drone_1_qn_aav"])
    if mode == "idle_scan":
        wait_until(lambda: safety.get(1, {}).get("latched") == "true", 10, "idle watchdog did not latch")
        wait_until(lambda: model.get(1, {}).get("source_trajectory_id") == safety[1]["trajectory_id"], 5, "idle hold not adopted")
        begin_model = float(model[1]["model_time_s"])
        wait_until(lambda: float(model[1]["model_time_s"])-begin_model >= 4., 8, "idle hold model time")
    else:
        assert client.wait_for_result(rospy.Duration(195)), "native terminal Result missing"
    result = client.get_result()
    evidence = json.loads((output/result.evidence_file).read_text())
    negative = mode in ("group_rpc_hang", "state_missing")
    checks = {"native_terminal": client.get_state() == (3 if mode == "idle_scan" else 4 if "scan" in mode or negative else 2)}
    if mode == "idle_scan":
        checks["old_success_unchanged"] = result.task_outcome == 1 and evidence["resource_released"]
    else:
        checks.update(hold_verdict_matches_expectation=evidence.get("safety_hold", {}).get("verified", False) == (not negative),
                      not_original_success=result.task_outcome != 1,
                      resources_locked=evidence["run_state"] == "UNKNOWN_LOCKED" and not evidence["resource_released"])
    if not negative:
        checks["safety_pass"] = result.safety_outcome == 1
    if mode == "state_missing":
        checks["missing_state_not_safety_pass"] = result.safety_outcome == 3
    if "scan" in mode:
        checks["autonomous_local_trigger"] = safety.get(1, {}).get("reason") == "LOCAL_CLOUD_STALE"
    if mode == "group_rpc_hang":
        hold = evidence["safety_hold"]
        checks["rpc_deadline_bounded"] = hold["observed_wall_s"] < hold["timeout_s"] + 2
        checks["other_members_requested"] = all(safety.get(a, {}).get("reference_published") == "true" for a in (0,1))
    if not negative:
        before = {a:safety[a]["trajectory_id"] for a in members}
        publisher = rospy.Publisher("/drone_1_member_goal", PoseStamped, queue_size=1)
        wait_until(lambda: publisher.get_num_connections()>0, 3, "goal publisher did not connect")
        bogus = PoseStamped();bogus.header.frame_id="world";bogus.header.stamp=rospy.Time.now()
        bogus.pose.position.x=-5;bogus.pose.position.y=4;bogus.pose.position.z=.8
        publisher.publish(bogus)
        time.sleep(.4)
        checks["ordinary_goal_did_not_replace_hold"] = all(safety[a]["trajectory_id"] == before[a] for a in members)
    if "scan" in mode:
        # Restored valid scans must not clear the physical latch.
        from sensor_msgs.msg import PointCloud2
        cloud = rospy.Publisher("/drone_1_pcl_render_node/cloud", PointCloud2, queue_size=1)
        wait_until(lambda: cloud.get_num_connections()>0, 3, "scan restore did not connect")
        for _ in range(10):
            msg=PointCloud2();msg.header.frame_id="world";msg.header.stamp=rospy.Time.now();msg.height=1
            cloud.publish(msg);time.sleep(.05)
        checks["restored_scan_keeps_latch"] = safety[1]["latched"] == "true"
    if mode == "idle_scan":
        import rosgraph
        import xmlrpc.client
        master = rosgraph.Master(rospy.get_name())
        node = "/aav_2_action_server"
        old_pid = xmlrpc.client.ServerProxy(master.lookupNode(node)).getPid(rospy.get_name())[2]
        rosnode.kill_nodes([node])
        restarted = subprocess.Popen(["rosrun", "qn_aav_simulator", "formation_action_server.py", "__name:=aav_2_action_server"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        def replaced():
            try:
                return xmlrpc.client.ServerProxy(master.lookupNode(node)).getPid(rospy.get_name())[2] != old_pid
            except Exception:
                return False
        wait_until(replaced, 10, "Action server did not restart")
        wait_until(lambda: rospy.get_param(node+"/safety_latched_members", []) == [1], 5, "restart forgot member latch")
        checks["server_restart_keeps_unavailable"] = not rospy.get_param(node+"/ready", True)
    for unit in ("aav_2", "aav_formation"):
        blocked = actionlib.SimpleActionClient("/"+unit+"/formation_action", FormationAction)
        assert blocked.wait_for_server(rospy.Duration(5))
        blocked.send_goal(goal)
        assert blocked.wait_for_result(rospy.Duration(3))
        checks[unit+"_rejects_overlap"] = blocked.get_state() == 5
    summary = {"mode": mode, "trigger": trigger, "checks": checks,
               "native_state": client.get_state(), "result_file": result.evidence_file,
               "finished_state": states, "passed": all(checks.values())}
    (output/"probe.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)
    return 0 if summary["passed"] else 1


def analyze(output):
    """Derive timing/displacement from original records; never feed back control."""
    import rosbag
    import re
    local, odometry, sources, trajectories = {}, {}, {}, {}
    with rosbag.Bag(str(output/"execution.bag")) as bag:
        for topic, msg, received in bag.read_messages():
            match = re.match(r"/drone_(\d+)_(planning|qn)/(.*)", topic)
            if not match:
                continue
            a, kind = int(match[1]), match[3]
            if kind == "safety_status":
                values = {v.key:v.value for row in msg.status for v in row.values}
                if values.get("latched") == "true" and a not in local:
                    local[a] = values
            elif kind == "odometry":
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                odometry.setdefault(a, []).append((msg.header.stamp.to_sec(), (p.x,p.y,p.z), math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)))
            elif kind == "diagnostics":
                v = {v.key:v.value for row in msg.status for v in row.values}
                if "source_trajectory_id" in v:
                    sources.setdefault(a, []).append((msg.header.stamp.to_sec(), int(v["source_trajectory_id"]), float(v["model_time_s"])))
            elif kind == "trajectory":
                trajectories.setdefault(a, []).append((msg.start_time.to_sec(), msg.traj_id))
    outcomes = []
    for path in output.glob("*.diagnostics.json"):
        d = json.loads(path.read_text())
        if d.get("safety_hold"):
            outcomes.append({"task_id":d["task_id"], "hold": d["safety_hold"]["outcome"],
                "original_task":d["task_outcome"], "safety":d["safety_outcome"],
                "resource_released":d["resource_released"], "finished_ros_s":d["actual_finish_time"],
                "model_hold":d["safety_hold"].get("model_hold"),
                "hold_started_ros_s":(d["safety_hold"].get("last_snapshot") or {}).get("hold_started")})
    end = max((x["finished_ros_s"] for x in outcomes), default=float("inf"))
    members = {}
    for a, status in local.items():
        latch, scan = float(status["latched_at_s"]), float(status["last_cloud_stamp_s"])
        point = tuple(float(status["hold_"+x]) for x in "xyz")
        tid = int(status["trajectory_id"])
        published = next((t for t, i in trajectories.get(a, []) if i == tid and t >= latch), None)
        adopted = next((t for t, i, _ in sources.get(a, []) if i == tid and t >= latch), None)
        samples = odometry.get(a, [])
        near_scan = min(samples, key=lambda s:abs(s[0]-scan)) if samples else None
        if near_scan and abs(near_scan[0]-scan) > .06:
            near_scan = None  # same alignment window used by this experiment
        first_settled = next((t for t,p,v in samples if adopted is not None and t >= adopted
                              and math.dist(p,point)<=.5 and v<=.25), None)
        replaced = [i for t,i,_ in sources.get(a, []) if adopted is not None and adopted <= t and i != tid]
        members[str(a)] = {"reason":status["reason"], "last_valid_scan_s":scan,
            "latched_s":latch,"published_s":published,"adopted_s":adopted,"first_settled_s":first_settled,
            "detection_latency_s":latch-scan if status["reason"]=="LOCAL_CLOUD_STALE" else None,
            "takeover_latency_s":None if adopted is None else adopted-latch,
            "settle_time_from_trigger_s":None if first_settled is None else first_settled-latch,
            "max_displacement_from_hold_point_m":None if status["point_valid"]!="true" else max((math.dist(p,point) for t,p,v in samples if latch<=t<=end), default=None),
            "max_displacement_since_last_scan_m":None if near_scan is None else max(
                (math.dist(p,near_scan[1]) for t,p,v in samples if scan<=t<=end), default=None),
            "stop_reference_replaced_after_adoption":None if adopted is None else bool(replaced),
            "holding_point":point if status["point_valid"]=="true" else None,"stop_trajectory_id":tid if tid>=0 else None}
    report = {"derived_from":"recorded ROS messages and original Action evidence", "members":members,"outcomes":outcomes}
    (output/"safety-timeline.json").write_text(json.dumps(report,indent=2))
    print(json.dumps({"directory":str(output),"members":len(members),"outcomes":outcomes}),flush=True)


if __name__ == "__main__":
    if len(sys.argv)>2 and sys.argv[1] == "--analyze":
        analyze(Path(sys.argv[2]))
    else:
        raise SystemExit(main())
