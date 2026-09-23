"""Task-line dispatch contract using native Action boundary doubles."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

from test_formation_action_server import server_module
from mrta_python import ExecutorPlan, ExecutorPlanItem
from mrta_python.models import NativeActionSpec,NativeSegmentSpec,ExecutionStep,Task
from qn_aav_simulator.executor_routing import load_routing, dispatchable_units
from qn_aav_simulator.task_line import load_request, load_formation_phase
from qn_aav_simulator.monitoring_request import expand


@pytest.fixture
def runner_module(server_module, monkeypatch):
    msg = sys.modules["qn_aav_simulator.msg"]
    for name in ("FormationActionGoal", "FormationActionResult"):
        monkeypatch.setattr(msg, name, object, raising=False)
    def goal():
        return SimpleNamespace(formation_center=SimpleNamespace(
            header=SimpleNamespace(), point=SimpleNamespace()))
    monkeypatch.setattr(msg, "FormationGoal", goal, raising=False)
    monkeypatch.setattr(server_module.rospy, "Duration", lambda t: t, raising=False)
    path = Path(__file__).resolve().parents[1] / "scripts/formation_mission_runner.py"
    spec = importlib.util.spec_from_file_location("runner_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_runner(module, tmp_path):
    runner = module.MissionRunner.__new__(module.MissionRunner)
    runner.executor_mutex = threading.RLock()
    runner.executor_serial = True
    runner.output = tmp_path
    entries = [{"executor_id": "aav_"+str(i+1), "physical_agent_ids": ["drone_"+str(i)],
                "capabilities": ["AIR"], "action_endpoint": "/selected_"+str(i)+"/formation_action"}
               for i in range(3)]
    entries.append({"executor_id": "group", "physical_agent_ids": ["drone_0", "drone_1", "drone_2"],
                    "capabilities": ["AIR"], "action_endpoint": "/selected_group/action"})
    runner.routing = load_routing(entries, default_members=(), default_initial_target_ref="start")
    runner.units = dispatchable_units(runner.routing)
    runner.fleet = ["drone_0", "drone_1", "drone_2"]
    runner.member_slots = {u.executor_id: {m: (0, 0, 0) for m in u.physical_agent_ids} for u in runner.units}
    runner.member_slots["group"] = {"drone_0": (0, 0, 0), "drone_1": (0, -2, 0), "drone_2": (0, 2, 0)}
    runner._actual_positions = lambda: {"drone_0": (-30, 6, .8), "drone_1": (-30, 4, .8), "drone_2": (-30, 8, .8)}
    path = Path(__file__).resolve().parents[1] / "config/monitoring_request_coastal.yaml"
    runner.request = load_request(path)
    runner.formation_phase = load_formation_phase(path)
    runner.speed, runner.seed, runner.epoch = 1.5, 0, 0
    runner.active_executor_ids = set()
    runner.final_events = {}
    runner.metrics = {"executions": [], "results_received": []}
    runner._save_executor = lambda: None
    runner._wait_executor_ready = lambda unit: None
    return runner


def native_setup(module,tmp_path):
    runner=make_runner(module,tmp_path)
    unit=load_routing([dict(executor_id='uuv_native',physical_agent_ids=['uuv'],
        capabilities=['WATER'],action_endpoint='/uuv/platform_task',action_type='PlatformTaskAction',
        operations=['WATER_PATH'],odometry_topics={'uuv':'/uuv/odometry'})],
        default_members=(),default_initial_target_ref='start')['uuv_native']
    spec=NativeActionSpec((NativeSegmentSpec('WATER_PATH',((-5.,8.,-2.),(0.,8.,-2.))),),'COAST_STOP',150.)
    prediction=dict(status='FEASIBLE',geometry_checked=True,duration_s=65.,
                    terminal_position=(9.,8.,-2.),terminal_mode='WATER')
    item=ExecutorPlanItem('native-e','native-t','uuv_native',('uuv',),0.,65.,65.,0.,0.,status='RUNNING',
        execution_steps=(ExecutionStep('uuv_native',65.,'water',spec,native_prediction=prediction),))
    runner.routing[unit.executor_id]=unit
    runner.plan=ExecutorPlan([item]);runner.tasks_by_id={'native-t':Task('native-t',frozenset({'WATER'}),1,0.,150.,'water')}
    runner.observation_tasks={};runner.active_executor_ids={unit.executor_id}
    runner._refresh_executor_timing=lambda _:None
    runner.native_result=lambda _:('native-goal',SimpleNamespace(status=SimpleNamespace(status=3)))
    result=SimpleNamespace(task_id=item.execution_id,goal_id='native-goal',task_completed=True,
        terminal_verified=True,actual_mode='WATER',reason='COAST_STOP_VERIFIED_IN_QUALIFICATION',
        resource_locked=False,model_time_s=65.)
    return runner,item,unit,result


def test_native_result_commits_motion_without_creating_coverage(runner_module,tmp_path):
    runner,item,unit,result=native_setup(runner_module,tmp_path)
    runner._commit_executor_result(item,unit,3,result)
    assert runner.plan.item(item.execution_id).status=='COMPLETED'
    assert not runner.active_executor_ids
    assert runner.metrics['native_qualification_only']
    assert runner.metrics['executions'][0]['scope']=='NATIVE_MOTION'
    # A duplicate old Result must not clear a new booking of the same member.
    runner.active_executor_ids.add(unit.executor_id)
    runner._commit_executor_result(item,unit,3,result)
    assert runner.active_executor_ids=={unit.executor_id}


def test_negative_observation_report_must_reach_mother_before_release(runner_module,tmp_path):
    from dataclasses import replace
    runner,item,unit,result=native_setup(runner_module,tmp_path)
    action=replace(item.execution_steps[0].native_action,observation_ids=('water_sample',))
    item.execution_steps=(replace(item.execution_steps[0],native_action=action),)
    runner.observation_tasks={item.task_id:object()}
    runner.metrics['received_products']={}
    runner.metrics['received_terminal_reports']={}
    runner.native_result=lambda _:('native-goal',SimpleNamespace(status=SimpleNamespace(status=4)))
    result.task_completed=False
    result.reason='OBSERVATION_NOT_SATISFIED'
    with pytest.raises(RuntimeError,match='negative observation report missing'):
        runner._commit_executor_result(item,unit,4,result)
    assert unit.executor_id in runner.active_executor_ids
    runner.metrics['received_terminal_reports']['native-goal']={
        'point_ids':['water_sample'],'observed_ids':[]}
    runner._commit_executor_result(item,unit,4,result)
    assert runner.plan.item(item.execution_id).status=='COMPLETED'
    assert unit.executor_id not in runner.active_executor_ids
    assert runner.metrics['executions'][-1]['result']=='OBSERVATION_MISSING'
    assert runner.metrics['results_received']==[item.task_id]


def test_received_product_requires_this_goal_and_never_releases_members(runner_module,tmp_path,monkeypatch):
    from dataclasses import replace
    from qn_aav_simulator.observation_coverage import CoverageResult
    runner,item,unit,_=native_setup(runner_module,tmp_path)
    runner.request=load_request(Path(__file__).parents[1]/'config/monitoring_request_joint.yaml')
    step=item.execution_steps[0]
    item.execution_steps=(replace(step,native_action=replace(step.native_action,observation_ids=('water_sample',))),)
    runner.condition=threading.Condition();runner.goal_ids={item.execution_id:{'current'}}
    runner.coverage=CoverageResult();runner.metrics['received_products']={}
    monkeypatch.setattr(runner_module.rospy,'logerr_throttle',lambda *a:None,raising=False)
    event=dict(product_id='product',request_id=runner.request.request_id,point_id='water_sample',
        producer='uuv',goal_id='old',observed=True,required_bytes=32768,generated_at=100.,received_at=120.,
        result=dict(model='GEOMETRIC_PROXY',dwell_s=1.))
    runner._on_received_product(SimpleNamespace(data=json.dumps(event)))
    assert not runner.metrics['received_products']
    event['goal_id']='current'
    runner._on_received_product(SimpleNamespace(data=json.dumps(event)))
    runner._on_received_product(SimpleNamespace(data=json.dumps(event)))
    assert len(runner.metrics['received_products'])==1
    assert runner.coverage.delivered_fraction({'water_sample':1.})==1.
    assert runner.active_executor_ids=={unit.executor_id} and item.status=='RUNNING'


def test_air_report_and_product_follow_same_mother_receipt_gate(runner_module,tmp_path,monkeypatch):
    from qn_aav_simulator.observation_coverage import CoverageResult
    runner=make_runner(runner_module,tmp_path)
    runner.request=load_request(Path(__file__).parents[1]/'config/monitoring_request_joint.yaml')
    unit=runner.routing['aav_1']
    item=ExecutorPlanItem('air-e','air-t',unit.executor_id,unit.physical_agent_ids,0.,6.,2.,0.,4.,
        status='RUNNING',execution_steps=(ExecutionStep(unit.executor_id,6.,'overview',service_time_s=4.),))
    runner.plan=ExecutorPlan([item]);runner.observation_tasks={'air-t':SimpleNamespace(covers=('air_sample',))}
    runner.condition=threading.Condition();runner.goal_ids={item.execution_id:{'air-goal'}}
    runner.coverage=CoverageResult();runner.active_executor_ids={unit.executor_id}
    runner.metrics.update(received_products={},received_terminal_reports={})
    monkeypatch.setattr(runner_module.rospy,'logerr_throttle',lambda *a:None,raising=False)
    report=dict(event_type='OBSERVATION_TERMINAL',product_id='air-goal:terminal',
        request_id=runner.request.request_id,goal_id='air-goal',producer='drone_0',
        point_ids=['air_sample'],observed_ids=['air_sample'],generated_at=100.,received_at=101.)
    runner._on_received_notification(SimpleNamespace(data=json.dumps(report)))
    assert runner.coverage.delivered_fraction({'air_sample':1.})==0.
    assert unit.executor_id in runner.active_executor_ids
    event=dict(product_id='air-goal:air_sample',request_id=runner.request.request_id,
        point_id='air_sample',producer='drone_0',goal_id='air-goal',observed=True,
        required_bytes=32768,generated_at=100.,received_at=102.,
        result=dict(model='GEOMETRIC_PROXY',dwell_s=1.))
    runner._on_received_product(SimpleNamespace(data=json.dumps(event)))
    assert runner.coverage.delivered_fraction({'air_sample':1.})==1.
    assert unit.executor_id in runner.active_executor_ids


@pytest.mark.parametrize('failure',['rejected','hung_start'])
def test_cooperative_missing_acceptance_or_hung_start_keeps_all_bookings(runner_module,tmp_path,monkeypatch,failure):
    from dataclasses import replace
    from types import ModuleType
    runner,item,unit,_=native_setup(runner_module,tmp_path)
    support=load_routing([dict(executor_id='usv_native',physical_agent_ids=['usv'],capabilities=['SURFACE'],
        action_endpoint='/usv/platform_task',action_type='PlatformTaskAction',operations=['SURFACE_PATH'],
        odometry_topics={'usv':'/usv/odometry'})],
        default_members=(),default_initial_target_ref='start')['usv_native']
    runner.routing[support.executor_id]=support
    spec=NativeActionSpec((NativeSegmentSpec('SURFACE_PATH',((0.,0.,0.),(1.,0.,0.))),),'TRIM_PROPULSION',.15)
    other=replace(item,execution_id='support-e',executor_id=support.executor_id,coalition=('usv',),fulfills_task=False,
        execution_steps=(ExecutionStep(support.executor_id,65.,'surface',spec),))
    item.execution_steps=(replace(item.execution_steps[0],native_action=replace(item.native_action,execution_timeout_s=.15)),)
    runner.plan=ExecutorPlan([item,other],serial=False)
    runner.condition=threading.Condition();runner.goal_ids={item.execution_id:{'work-goal'},other.execution_id:{'support-goal'}}
    runner.active_executor_ids={unit.executor_id,support.executor_id}
    runner.server_nodes={unit.executor_id:'/uuv',support.executor_id:'/usv'}
    runner._executor_goal=lambda *a:SimpleNamespace(prepare_only=False)
    canceled=[];unblock=threading.Event();started=[]
    def client(name,state):
        return SimpleNamespace(send_goal=lambda g,feedback_cb:feedback_cb(SimpleNamespace(operation='PREPARED',reference_generation=1)),
            get_state=lambda:state,cancel_goal=lambda:canceled.append(name))
    runner.clients={unit.executor_id:client('work',1),support.executor_id:client('support',5 if failure=='rejected' else 1)}
    srv=ModuleType('qn_aav_simulator.srv');srv.StartPreparedAction=object
    monkeypatch.setitem(sys.modules,'qn_aav_simulator.srv',srv)
    monkeypatch.setattr(runner_module.rospy,'wait_for_service',lambda *a,**k:None,raising=False)
    def proxy(name,*args):
        def call(*args):
            started.append(name);unblock.wait(2);return SimpleNamespace(accepted=True,reason='late')
        return call
    monkeypatch.setattr(runner_module.rospy,'ServiceProxy',proxy,raising=False)
    try:
        with pytest.raises(RuntimeError):runner._dispatch_cooperative_items([item,other])
    finally:unblock.set()
    assert set(canceled)=={'work','support'}
    assert runner.active_executor_ids=={unit.executor_id,support.executor_id}
    if failure=='rejected':assert not started
    else:assert started==['/usv/start_prepared']  # unknown support start cannot launch the work fragment


@pytest.mark.parametrize('fail_first',[False,True])
def test_composite_retains_booking_between_steps_and_failure_blocks_successor(runner_module,tmp_path,fail_first):
    runner,item,unit,result=native_setup(runner_module,tmp_path)
    item.execution_steps=(item.execution_steps[0],item.execution_steps[0])
    item.travel_time=130.;item.planned_finish=130.
    runner._executor_goal=lambda view,unit:SimpleNamespace(task_id=view.execution_id)
    runner.native_result=lambda _:('native-goal',SimpleNamespace(status=SimpleNamespace(status=3)))
    calls=[]
    def send(view,selected,goal,action):
        assert runner.active_executor_ids=={item.executor_id}
        assert runner.plan.item(item.execution_id).status=='RUNNING'
        assert not runner.metrics['results_received']
        calls.append(goal.task_id)
        return (4 if fail_first else 3),SimpleNamespace(**dict(vars(result),task_id=goal.task_id))
    runner._send_executor_goal=send
    # The actual receipt timestamp must follow this test's planned start.
    runner.epoch=-200.
    if fail_first:
        with pytest.raises(RuntimeError,match='parent remains occupied'):
            runner._dispatch_executor_item(item,reserved=True)
        assert calls==['native-e:step:0']
        assert runner.active_executor_ids=={item.executor_id}
        assert not runner.metrics['results_received']
    else:
        runner._dispatch_executor_item(item,reserved=True)
        assert calls==['native-e:step:0','native-e:step:1']
        assert not runner.active_executor_ids
        assert runner.plan.item(item.execution_id).status=='COMPLETED'
        assert runner.metrics['results_received']==[item.task_id]


def test_slow_dashboard_publication_does_not_hold_execution_lock(runner_module,tmp_path,monkeypatch):
    runner=make_runner(runner_module,tmp_path)
    runner.executor_write_mutex=threading.Lock()
    runner._save_executor_locked=lambda:({'resource_locks':[]},{})
    entered=threading.Event();release=threading.Event()
    def blocked_param(*args):
        entered.set()
        assert release.wait(2)
    monkeypatch.setattr(runner_module.rospy,'set_param',blocked_param,raising=False)
    thread=threading.Thread(target=runner_module.MissionRunner._save_executor,args=(runner,))
    thread.start()
    try:
        assert entered.wait(2)
        assert runner.executor_mutex.acquire(timeout=.1)
        runner.executor_mutex.release()
    finally:
        release.set();thread.join(2)
    assert not thread.is_alive()


@pytest.mark.parametrize('delivered',[False,True])
def test_composite_observation_checks_receipt_for_each_native_goal_before_release(runner_module,tmp_path,delivered):
    from dataclasses import replace
    runner,item,unit,result=native_setup(runner_module,tmp_path)
    step=item.execution_steps[0]
    step=replace(step,native_action=replace(step.native_action,observation_ids=('water_sample',)))
    item.execution_steps=(step,step);item.travel_time=130.;item.planned_finish=130.
    runner.observation_tasks={item.task_id:object()};runner.metrics['received_products']={};runner.epoch=-200.
    runner._executor_goal=lambda view,unit:SimpleNamespace(task_id=view.execution_id)
    runner.native_result=lambda ident:(ident+'-goal',SimpleNamespace(status=SimpleNamespace(status=3)))
    def send(view,*args):
        native=SimpleNamespace(**dict(vars(result),task_id=view.execution_id,goal_id=view.execution_id+'-goal'))
        if delivered:
            runner.metrics['received_products'][view.execution_id]=dict(goal_id=native.goal_id,point_id='water_sample',observed=True)
        return 3,native
    runner._send_executor_goal=send
    if delivered:
        runner._dispatch_executor_item(item,reserved=True)
        assert not runner.active_executor_ids and runner.plan.items[0].status=='COMPLETED'
    else:
        with pytest.raises(RuntimeError,match='without required received products'):
            runner._dispatch_executor_item(item,reserved=True)
        assert runner.active_executor_ids=={unit.executor_id} and item.status=='RUNNING'


def test_native_motion_cannot_fabricate_observation_or_release_unknown_mode(runner_module,tmp_path):
    runner,item,unit,result=native_setup(runner_module,tmp_path)
    result.actual_mode='AIR'
    with pytest.raises(RuntimeError,match='cannot release'):
        runner._commit_executor_result(item,unit,3,result)
    assert unit.executor_id in runner.active_executor_ids
    result.actual_mode='WATER';runner.observation_tasks[item.task_id]=object()
    with pytest.raises(RuntimeError,match='observation products'):
        runner._commit_executor_result(item,unit,3,result)
    assert unit.executor_id in runner.active_executor_ids


def test_native_goal_uses_path_message_and_selected_operation(runner_module,tmp_path,monkeypatch):
    runner,item,unit,result=native_setup(runner_module,tmp_path)
    from types import ModuleType
    nav=ModuleType('nav_msgs.msg');nav.Path=lambda:SimpleNamespace(header=SimpleNamespace(),poses=[])
    monkeypatch.setitem(sys.modules,'nav_msgs.msg',nav)
    geometry=sys.modules['geometry_msgs.msg']
    monkeypatch.setattr(geometry,'PoseStamped',lambda:SimpleNamespace(header=SimpleNamespace(),
        pose=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace())),raising=False)
    messages=sys.modules['qn_aav_simulator.msg']
    monkeypatch.setattr(messages,'PlatformTaskGoal',lambda **kw:SimpleNamespace(segments=[],**kw),raising=False)
    monkeypatch.setattr(messages,'PlatformSegment',lambda **kw:SimpleNamespace(**kw),raising=False)
    goal=runner._executor_goal(item,unit)
    assert goal.task_id==item.execution_id and goal.terminal_behavior=='COAST_STOP'
    assert goal.segments[0].operation=='WATER_PATH'
    assert goal.segments[0].path.poses[-1].pose.position.z==-2.
    assert not hasattr(goal,'formation_center')


def test_parallel_dispatch_books_members_before_workers_and_waits_for_group(runner_module,tmp_path):
    runner=make_runner(runner_module,tmp_path)
    runner.executor_serial=False
    runner.plan=ExecutorPlan([
        ExecutorPlanItem('e0','t0','aav_1',('drone_0',),0,1,0,0,1),
        ExecutorPlanItem('e1','t1','aav_2',('drone_1',),0,1,0,0,1),
        ExecutorPlanItem('eg','tg','group',tuple(runner.fleet),0,1,0,0,1)],
        serial=False,precedence_edges=(('t0','tg'),('t1','tg')))
    barrier=threading.Barrier(2)
    entered=[]
    def dispatch(item,reserved):
        assert reserved and item.executor_id in runner.active_executor_ids
        if item.executor_id!='group':barrier.wait(timeout=2)
        with runner.executor_mutex:
            if item.executor_id=='group':assert set(entered)=={'aav_1','aav_2'}
            entered.append(item.executor_id)
            runner.plan.item(item.execution_id).status='COMPLETED'
            runner.active_executor_ids.remove(item.executor_id)
    runner._dispatch_executor_item=dispatch
    runner._execute_parallel_pending()
    assert entered[-1]=='group'
    assert not runner.active_executor_ids


def test_request_builds_one_serial_plan_including_assembly_and_transfer(runner_module, tmp_path):
    runner = make_runner(runner_module, tmp_path)
    plan = runner._executor_plan(expand(runner.request), include_formation=True)
    assert len(plan.items) == 5
    assert all(b.planned_start >= a.planned_finish for a, b in zip(plan.items, plan.items[1:]))
    assert [i.executor_id for i in plan.items[-2:]] == ["group", "group"]
    assert plan.items[0].coalition == ("drone_1",)
    assert len({i.execution_id for i in plan.items}) == len(plan.items)


def test_unconfirmed_request_never_dispatches(runner_module, tmp_path, monkeypatch):
    runner = make_runner(runner_module, tmp_path)
    runner._dispatch_executor_item = lambda _: pytest.fail("dispatch without confirmation")
    monkeypatch.setattr("builtins.input", lambda _: "no")
    runner._run_executor()
    assert runner.metrics["status"] == "NOT_CONFIRMED"
    assert not runner.active_executor_ids


def dispatch_setup(runner_module, tmp_path):
    runner = make_runner(runner_module, tmp_path)
    runner.plan = runner._executor_plan(expand(runner.request))
    item = runner.plan.items[0]
    unit = runner.routing[item.executor_id]
    runner.server_nodes = {unit.executor_id: "/selected_server"}
    return runner, item, unit


def test_timeout_keeps_members_reserved(runner_module, tmp_path, monkeypatch):
    runner, item, unit = dispatch_setup(runner_module, tmp_path)
    sent = []
    runner.clients = {unit.executor_id: SimpleNamespace(send_goal=lambda g, **_: sent.append(g),
        wait_for_result=lambda _: False, get_state=lambda: 1)}
    times = iter([0, 1000])
    monkeypatch.setattr(runner_module.time, "monotonic", lambda: next(times))
    with pytest.raises(RuntimeError, match="result timeout"):
        runner._dispatch_executor_item(item)
    assert len(sent) == 1
    assert runner.active_executor_ids == {unit.executor_id}
    assert item.status == "RUNNING"  # outer handler records UNKNOWN_LOCKED


def test_selected_client_and_successful_result_release_next_item(runner_module, tmp_path):
    runner, item, unit = dispatch_setup(runner_module, tmp_path)
    result = SimpleNamespace(task_id=item.execution_id, goal_id="native-goal", reason=0,
        task_outcome=1, safety_outcome=1, experiment_validity=1,
        evidence_file="result.json", actual_finish_time=SimpleNamespace(to_sec=lambda: 20))
    (tmp_path / "result.json").write_text(json.dumps({"goal_id": "native-goal",
        "accepted_for_dispatch": True, "resource_released": True}))
    sent = []
    runner.clients = {unit.executor_id: SimpleNamespace(send_goal=lambda g, **_: sent.append((unit.action_endpoint,g)),
        wait_for_result=lambda _: True, get_state=lambda: 3, get_result=lambda: result)}
    runner.native_result = lambda _: ("native-goal", SimpleNamespace(status=SimpleNamespace(status=3)))
    runner._receive_observations = lambda *_: None
    runner._refresh_executor_timing = lambda _: None
    runner._dispatch_executor_item(item)
    assert sent[0][0] == "/selected_1/formation_action"
    assert not runner.active_executor_ids
    assert runner.plan.items[0].status == "COMPLETED"
    assert runner.plan.items[1].planned_start >= 20


@pytest.mark.parametrize("field,value", [("safety_outcome",2), ("experiment_validity",2),
                                        ("reason",6), ("task_outcome",2), ("task_id","old")])
def test_bad_result_cannot_release(runner_module, field, value):
    result = SimpleNamespace(task_id="new", goal_id="goal", reason=0, task_outcome=1,
                             safety_outcome=1, experiment_validity=1)
    setattr(result, field, value)
    assert not runner_module.MissionRunner._release_result_ok(3, result, "new")


def test_serial_completion_repairs_disjoint_successor_and_deduplicates():
    from mrta_python import DelayEvent
    from mrta_python.repair import process_executor_completion
    first = ExecutorPlanItem('e1', 't1', 'u1', ('a',), 0, 2, 1, 0, 1, 'RUNNING')
    second = ExecutorPlanItem('e2', 't2', 'u2', ('b',), 2, 5, 2, 0, 1)
    plan = ExecutorPlan([first, second])
    event = DelayEvent('result', 'e1', 't1', 2, 7)
    events = {}
    updated, changed = process_executor_completion(plan, event, events)
    assert changed and updated.items[1].planned_start == 7
    assert updated.items[1].planned_finish == 10
    assert plan.items[0].status == 'RUNNING'
    repeated, changed = process_executor_completion(updated, event, events)
    assert repeated is updated and not changed
    with pytest.raises(ValueError, match='different terminal result'):
        process_executor_completion(updated, DelayEvent('other', 'e1', 't1', 2, 8), events)


def test_actual_member_position_changes_later_group_cost(runner_module, tmp_path):
    runner = make_runner(runner_module, tmp_path)
    runner.plan = runner._executor_plan(expand(runner.request), include_formation=True)
    for item in runner.plan.items[:-2]:
        item.status = 'COMPLETED'
    runner.plan_revision = 0
    runner.metrics['plan_history'] = []
    # AAV1 was actually left at P; other members remain at their own initial positions.
    runner._actual_positions = lambda: {'drone_0': (-10, 6, .8),
        'drone_1': (-30, 4, .8), 'drone_2': (-30, 8, .8)}
    runner._refresh_executor_timing(50)
    assert runner.plan.items[-2].travel_time == pytest.approx(20 / 1.5)
    assert runner.plan.items[-2].planned_start >= 50


def test_failed_native_result_is_received_but_does_not_release(runner_module, tmp_path):
    runner, item, unit = dispatch_setup(runner_module, tmp_path)
    result = SimpleNamespace(task_id=item.execution_id, goal_id="failed-goal", reason=6,
        task_outcome=2, safety_outcome=2, experiment_validity=2, evidence_file="failure.json")
    (tmp_path / "failure.json").write_text(json.dumps({"goal_id": "failed-goal",
        "reason_text": "inter-agent surface clearance below limit"}))
    runner.clients = {unit.executor_id: SimpleNamespace(send_goal=lambda *a, **k: None,
        wait_for_result=lambda _: True, get_state=lambda: 4, get_result=lambda: result)}
    with pytest.raises(RuntimeError, match="cannot release"):
        runner._dispatch_executor_item(item)
    assert runner.active_executor_ids == {unit.executor_id}
    assert runner.metrics["results_received"] == [item.task_id]
    assert runner.metrics["executions"][0]["result"] == "NOT_ACCEPTED"
    assert not runner.metrics["executions"][0]["plan_updated"]
    assert "inter-agent surface clearance" in runner.metrics["executions"][0]["detail"]
