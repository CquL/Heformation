#!/usr/bin/env python3
"""ROS Action measurement for the opt-in qn qualification endpoint."""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time
import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from qn_aav_simulator.msg import PlatformTaskAction, PlatformTaskGoal, PlatformSegment
from qn_aav_simulator.srv import TakeReference


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cancel',action='store_true')
    args=parser.parse_args()
    rospy.init_node('probe_platform_fragment',anonymous=True)
    lock=threading.Lock()
    rows=[]
    last={}
    def diag(message):
        with lock:
            for status in message.status:
                values={v.key:v.value for v in status.values}
                if 'model_time_s' in values:
                    values['ros_time_s']=message.header.stamp.to_sec()
                    rows.append(values)
                    last.clear()
                    last.update(values)
    rospy.Subscriber('/qualification_qn/diagnostics',DiagnosticArray,diag,queue_size=1000)
    client=actionlib.SimpleActionClient('/qualification_qn/platform_task',PlatformTaskAction)
    if not client.wait_for_server(rospy.Duration(30)):
        raise RuntimeError('platform Action unavailable')
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        with lock:
            ready=last.get('actual_mode')=='AIR'
        if ready:break
        time.sleep(.05)
    else:raise RuntimeError('actual AIR diagnostics unavailable')
    goal=PlatformTaskGoal(task_id='qualification-continuous',terminal_behavior='FIXED_REFERENCE',execution_timeout=rospy.Duration(150))
    for operation,points,duration in [
        ('ENTER_WATER',[(0,0,.5),(0,0,-.6)],12.),
        ('WATER_PATH',[(0,0,-.6),(.7,0,-.6)],7.),
        ('EXIT_WATER',[(.7,0,-.6),(.7,0,.5)],12.)]:
        path=RosPath()
        path.header.frame_id='map'
        for point in points:
            pose=PoseStamped()
            pose.header.frame_id='map'
            pose.pose.orientation.w=1
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point
            path.poses.append(pose)
        goal.segments.append(PlatformSegment(operation=operation,path=path,duration=rospy.Duration(duration)))
    feedback=[]
    def on_feedback(msg):
        feedback.append(dict(index=msg.segment_index,operation=msg.operation,mode=msg.actual_mode,model_time=msg.model_time_s,generation=msg.reference_generation))
    client.send_goal(goal,feedback_cb=on_feedback)
    if args.cancel:
        deadline=time.monotonic()+35
        while time.monotonic()<deadline and not any(f['mode']=='WATER' for f in feedback):
            time.sleep(.1)
        client.cancel_goal()
        client.cancel_goal()
    received=client.wait_for_result(rospy.Duration(160))
    result=client.get_result() if received else None
    # Continue recording after Action completion: terminal reference must persist.
    time.sleep(2)
    with lock:
        snapshot=list(rows)
        final=dict(last)
    modes={row.get('actual_mode') for row in snapshot}
    status=client.get_state()
    first=next((r for r in snapshot if r.get('reference_source')=='PLATFORM'),None)
    active=[r for r in snapshot if r.get('reference_source')=='PLATFORM']
    drift=max((abs((float(r['model_time_s'])-float(first['model_time_s']))-
                       (r['ros_time_s']-first['ros_time_s'])) for r in active),default=float('inf')) if first else float('inf')
    checks=dict(result_received=received,
                native_terminal=status==(2 if args.cancel else 3),
                terminal_verified=bool(result and result.terminal_verified),
                original_task_outcome=bool(result and result.task_completed != args.cancel),
                resource_lock=bool(result and result.resource_locked == args.cancel),
                medium_evidence=('WATER' in modes if args.cancel else {'AIR','TRANSITION','WATER'}<=modes),
                time_consistent=drift<=.05,
                single_generation=len({r['reference_generation'] for r in active})==1,
                native_reference_persists=final.get('reference_source')=='PLATFORM')
    report={'scope':'single qn native fragment qualification; not full Swarm/request/fleet acceptance',
            'checks':checks,'passed':all(checks.values()),'action_status':status,
            'max_model_ros_drift_s':drift,'result':None if result is None else {
                field:getattr(result,field) for field in result.__slots__},
            'final':final,'feedback':feedback}
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'probe.json').write_text(json.dumps(report,indent=2)+'\n')
    with (args.output/'diagnostics.csv').open('w') as out:
        keys=sorted({k for r in snapshot for k in r})
        writer=csv.DictWriter(out,fieldnames=keys)
        writer.writeheader()
        writer.writerows(snapshot)
    print(json.dumps({'passed':report['passed'],'checks':checks,'result':report['result']},indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':
    raise SystemExit(main())
