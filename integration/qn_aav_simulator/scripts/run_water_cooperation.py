#!/usr/bin/env python3
"""Current water-request entry into the existing planner and MissionRunner.

The three AAV remain in standby. This entry does not claim the full joint
AIR/WATER mission. Native models are declared initial-trim snapshots, checked
against fresh startup positions; arbitrary mid-mission replanning is not offered.
"""
import math
import time
import traceback
import subprocess
import signal
from dataclasses import asdict

import rospy
from formation_mission_runner import MissionRunner, save_json
from mrta_python.executors import Executor, ExecutorTravelTimeProvider
from qn_aav_simulator.pvs_backend import PvsBackend, NATIVE_START_TOLERANCE_M
from qn_aav_simulator.task_line import build_request_executor_plan


def main():
    rospy.init_node('formation_mission_runner')
    rospy.set_param('~planning_mode', 'executor')
    rospy.set_param('~executor_serial', False)
    rospy.set_param('~output_dir', '/experiments/current')
    rospy.set_param('~request_file', rospy.get_param('/mission/request_file'))
    rospy.set_param('~executors', [dict(executor_id=k, physical_agent_ids=[k],
        capabilities=[mode], action_type='PlatformTaskAction', operations=[mode+'_PATH'],
        action_endpoint='/'+k+'/platform_task', odometry_topics={k:'/'+k+'/odometry'})
        for k, mode in [('uuv','WATER'), ('usv','SURFACE')]])
    runner = MissionRunner()
    runner.points = {p.point_id:p.position for r in runner.request.regions for p in r.interest_points}
    runner.weights = {p.point_id:p.weight for r in runner.request.regions for p in r.interest_points}
    status_timer = None
    recorder = None
    recorder_log = None
    try:
        runner.metrics['status'] = 'PLANNING'
        runner._save_executor()
        for unit in runner.units:
            runner._wait_executor_ready(unit)
        def fresh_positions():
            deadline=time.monotonic()+10.
            while not rospy.is_shutdown():
                try:return runner._actual_positions()
                except RuntimeError:
                    if time.monotonic()>=deadline:
                        raise RuntimeError('本机状态未就绪，未派发')
                    time.sleep(.05)
            raise RuntimeError('shutdown before dispatch')
        positions=fresh_positions()
        models = {'uuv':PvsBackend('remus100', (-5.,8.,-2.)),
                  'usv':PvsBackend('otter', (-5.,-8.,0.), initialization_mode='STATIC_TRIM')}
        def check_initial():
            current = fresh_positions()
            for member, backend in models.items():
                if math.dist(current[member], backend.snapshot()['position']) > NATIVE_START_TOLERANCE_M:
                    raise RuntimeError('初始资格状态已改变，需整链重启：'+member)
        check_initial()
        units = [Executor(u.executor_id,u.physical_agent_ids,frozenset(u.capabilities)) for u in runner.units]
        provider = ExecutorTravelTimeProvider({'start':positions['uuv']}, {'uuv':1.,'usv':1.},
            native_models=models, native_efforts={'uuv':500.,'usv':20.})
        states = {k:dict(position=b.snapshot()['position'],mode=b.snapshot()['actual_mode'],available_from=0.)
                  for k,b in models.items()}
        started = time.monotonic()
        runner.plan, tasks = build_request_executor_plan(runner.request,rospy.get_param('/scene'),units,provider,states)
        runner.tasks_by_id = {t.task_id:t for t in tasks}
        runner.observation_tasks = {t.task_id:object() for t in tasks}
        runner.metrics['planning_wall_s'] = time.monotonic()-started
        runner.metrics['status'] = 'AWAITING_CONFIRMATION'
        runner.metrics['selected_plan'] = asdict(runner.plan)
        save_json(runner.output/'confirmed-plan-preview.json', runner.metrics['selected_plan'])
        runner._save_executor()
        print('\n水下协同观测：潜航器作业 → 无人船支援 → 母船接收。三台无人机待命。',flush=True)
        print('当前范围：一个水下样点；不含空中作业、跨介质、复查和返回。',flush=True)
        print('母船为固定接收端；通信为声明的距离／遮挡／有限容量模型。',flush=True)
        for item in runner.plan.items:
            action = item.execution_steps[0].native_action
            print('{}：{}，计划 {:.1f}–{:.1f}s，终点 {}，终端等待 {}s'.format(
                '潜航器' if item.executor_id=='uuv' else '无人船',
                '水下观测' if action.observation_ids else '通信支援',
                item.planned_start,item.planned_finish,action.segments[-1].points[-1],action.terminal_wait_s),flush=True)
        print('确认以上具体计划后输入 yes；其他输入退出，未确认不派发。',flush=True)
        if input().strip() != 'yes':
            runner.metrics['status'] = 'NOT_CONFIRMED'
            runner._save_executor()
            return 2
        check_initial()
        # Record only the confirmed execution, not an unbounded operator wait.
        recorder_log=(runner.output/'recorder.log').open('w')
        recorder=subprocess.Popen(['rosbag','record','--lz4','--buffsize=256',
            '-O',str(runner.output/'handover.bag'),'--regex',
            '/aav_.*/formation_action/.*|/drone_[0-2]_qn/.*|/(usv|uuv)/(odometry|diagnostics|local_products|platform_task/.*)|/mother/received_.*|/scene/(global_cloud|delivery_progress)',
            '__name:=cooperation_recorder'],stdout=recorder_log,stderr=subprocess.STDOUT)
        import rosgraph
        recording_deadline=time.monotonic()+10.
        while True:
            subscribers=dict(rosgraph.Master(rospy.get_name()).getSystemState()[1])
            if all('/cooperation_recorder' in subscribers.get('/'+m+'/platform_task/goal',[]) for m in ('usv','uuv')):
                break
            if recorder.poll() is not None or time.monotonic()>=recording_deadline:
                raise RuntimeError('记录器未就绪，未派发')
            time.sleep(.05)
        runner.metrics['confirmed_plan_revision'] = runner.plan_revision
        runner.metrics['status'] = 'RUNNING'
        runner.epoch = rospy.Time.now().to_sec()
        runner._save_executor()
        status_timer=rospy.Timer(rospy.Duration(1.),lambda _:runner._save_executor())
        runner._execute_parallel_pending()
        passed = (all(i.status=='COMPLETED' for i in runner.plan.items)
                  and not runner.active_executor_ids
                  and set(runner.metrics['results_received'])==set(runner.tasks_by_id)
                  and runner.coverage.delivered_fraction(runner.weights)==1.)
        runner.metrics['status'] = 'PASS_WATER_GEOMETRIC_PROXY' if passed else 'FAILED'
        if not passed and not runner.metrics['failure_reason']:
            runner.metrics['failure_reason'] = '动作、接收或资源终态未全部满足'
        runner._save_executor()
        print('本次水下协作结果：'+runner.metrics['status'],flush=True)
        return 0 if passed else 1
    except (Exception, KeyboardInterrupt) as error:
        (runner.output/'failure-traceback.txt').write_text(traceback.format_exc())
        runner.metrics['status'] = 'FAILED'
        runner.metrics['failure_reason'] = str(error) or type(error).__name__
        # Never clear accepted resources on a UI/client failure.
        runner._save_executor()
        rospy.logerr('Water cooperation: %s',runner.metrics['failure_reason'])
        return 1
    finally:
        if status_timer is not None:status_timer.shutdown()
        if recorder is not None:
            recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                recorder.terminate()
                rospy.logerr('Recorder did not close within 10s; bag may be incomplete')
        if recorder_log is not None:recorder_log.close()


if __name__=='__main__':
    raise SystemExit(main())
