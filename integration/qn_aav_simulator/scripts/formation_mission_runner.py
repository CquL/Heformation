#!/usr/bin/env python3
"""Dispatch the current Plan and repair it from native Action completion results.

The runner owns the scheduler-side ``execution_id`` and maps it onto the native
Action GoalID.  It also keeps its own trajectory-ownership evidence, so the
Action server's adoption verdict can be cross-checked instead of trusted.

Test C outputs per action: ``plan_updated``, ``updated_plan_used`` and the
release lag in seconds.  In a serial single-resource scenario the release lag
is not a measure of repair: repair changes planned times, while every dispatch
still waits for the previous action's real completion.  What the runner records
instead is which plan revision each dispatch actually read.
"""

import json
import hashlib
import io
import math
import os
import threading
import time
import queue
from concurrent.futures import ThreadPoolExecutor,as_completed
from dataclasses import asdict, replace
from pathlib import Path

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from quadrotor_msgs.msg import PositionCommand
from mrta_python import (
    Agent, DelayEvent, Plan, PlanItem, Task, TravelTimeProvider,
    build_plan, process_completion,
)
from qn_aav_simulator.executor_routing import (
    ExecutionUnit, conflicting_active_unit, dispatchable_units, load_routing,
    unit_for_coalition,
)
from qn_aav_simulator.msg import (
    FormationAction, FormationActionGoal, FormationActionResult, FormationGoal,
)

# Scenario qualification, not a general tracking-error bound: same-source qn
# replay of the actual failed AIR run remained inside the existing 0.5 m
# return ball for this finite continuation (WORKLOG/air-hold-45 evidence).
_QUALIFIED_AIR_RETEST_HOLD_S = 45.0


def json_default(value):
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError("Cannot encode {}".format(type(value).__name__))


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, default=json_default,
                                    allow_nan=False) + "\n")
    temporary.replace(path)


class MissionRunner:
    def __init__(self):
        self.executor_mutex = threading.RLock()
        self.output = Path(rospy.get_param("~output_dir", "/experiments/current"))
        self.output.mkdir(parents=True, exist_ok=True)
        self.planning_mode = rospy.get_param("~planning_mode", "")
        if not self.planning_mode:
            raise ValueError("planning_mode must be explicit; fixed_coalition is a control regression entry")
        if self.planning_mode in ("executor", "joint_request"):
            self._init_executor()
            return
        if self.planning_mode != "fixed_coalition":
            raise ValueError("planning_mode must be fixed_coalition, executor or joint_request")
        self.mode = rospy.get_param("~mode", "mission")
        if self.mode not in ("single", "mission"):
            raise ValueError("mode must be single or mission")
        self.seed = int(rospy.get_param("~seed", 0))
        self.tolerance = float(rospy.get_param("~delay_tolerance", 0.1))
        self.speed = float(rospy.get_param("~nominal_speed_mps", 1.5))
        # One scene definition, shared with the publisher, the action server and
        # the verifier; the launch arg only toggles whether the box is present.
        scene = rospy.get_param("/scene", {})
        self.obstacle_scenario = bool(rospy.get_param(
            "~obstacle_scenario", scene.get("obstacle_present", False)))
        self.scene_topic = str(rospy.get_param(
            "~scene_topic", scene.get("topic", "/scene/global_cloud")))
        self.obstacle_center = [float(value) for value in rospy.get_param(
            "~obstacle_center", scene.get("obstacle_center", [-23.0, 0.0, 0.5]))]
        self.obstacle_size = [float(value) for value in rospy.get_param(
            "~obstacle_size", scene.get("obstacle_size", [1.0, 1.0, 1.2]))]
        self.centers = rospy.get_param("~centers")
        self.initial_ref = rospy.get_param("~initial_target_ref", "start")
        raw_repair_mode = rospy.get_param("~repair_mode", "on")
        if isinstance(raw_repair_mode, bool):
            # YAML 1.1 turns an unquoted on/off into a boolean.
            raw_repair_mode = "on" if raw_repair_mode else "off"
        self.repair_mode = str(raw_repair_mode).strip().lower()
        if self.repair_mode not in ("on", "off"):
            raise ValueError("repair_mode must be on or off")
        self.tasks = [Task(
            task_id=t["task_id"],
            required_capabilities=frozenset(t["required_capabilities"]),
            required_agent_count=t["required_agent_count"],
            service_time=float(t["service_time"]),
            deadline=None if t.get("deadline") is None else float(t["deadline"]), target_ref=t["target_ref"],
        ) for t in rospy.get_param("~tasks")]
        self.agents = [Agent("drone_{}".format(i), frozenset({"AAV"}), 0.0) for i in range(7)]
        self.travel = TravelTimeProvider(self.centers, self.speed)
        if self.mode == "single":
            task = next(t for t in self.tasks if t.task_id == "T1")
            self.tasks = [task]
            coalition = tuple(a.id for a in self.agents)
            transit = self.travel(coalition, self.initial_ref, task.target_ref)
            self.plan = Plan([PlanItem(
                "single-T1", task.task_id, coalition, 0.0,
                transit + task.service_time, transit, 0.0, task.service_time, "PLANNED",
            )])
        else:
            self.plan = build_plan(self.agents, self.tasks, self.travel,
                                   initial_target_ref=self.initial_ref, seed=self.seed)
        self.tasks_by_id = {t.task_id: t for t in self.tasks}
        # Static routing: executor -> member list -> Action endpoint.  A unit
        # without an endpoint can take part in offline planning but is never
        # dispatched, so it can never produce a fabricated "actual completion".
        self.routing = self._load_routing()
        self.online_executors = self._online_executors()
        # Units currently occupied.  Registration may overlap on purpose; being
        # occupied at the same time may not.
        self.active_executor_ids = set()
        # Increments whenever the completion step replaces the plan.  A dispatch
        # records the revision it read, so "the next action used the updated
        # plan" is evidenced instead of assumed.
        self.plan_revision = 0
        self.final_events = {}
        self.goal_ids = {}
        self.native_results = {}
        self.condition = threading.Condition()
        self.command_trajectory = {}
        self.qn_source = {}
        self.client = actionlib.SimpleActionClient("formation_action", FormationAction)
        self.goal_sub = rospy.Subscriber("/formation_action/goal", FormationActionGoal,
                                         self.on_goal, queue_size=10)
        self.result_sub = rospy.Subscriber("/formation_action/result",
                                           FormationActionResult, self.on_result,
                                           queue_size=10)
        for agent_id in range(7):
            rospy.Subscriber("/drone_{}_planning/pos_cmd".format(agent_id),
                             PositionCommand,
                             self.on_command, callback_args=agent_id, queue_size=10)
            rospy.Subscriber("/drone_{}_qn/diagnostics".format(agent_id),
                             DiagnosticArray,
                             self.on_diagnostics, callback_args=agent_id, queue_size=5)
        self.metrics = {
            "mode": self.mode, "status": "STARTING", "seed": self.seed,
            "nominal_speed_mps": self.speed,
            "planner_speed_mps": rospy.get_param("~planner_speed", 1.5),
            "delay_tolerance": self.tolerance, "repair_mode": self.repair_mode,
            "obstacle_scenario": self.obstacle_scenario,
            "obstacle_center": self.obstacle_center,
            "obstacle_size": self.obstacle_size,
            "scene_topic": self.scene_topic,
            "mission_epoch": None,
            "initial_plan": asdict(self.plan), "plan_history": [],
            "executions": [], "events": [], "test_c": {},
        }

    def _init_executor(self):
        """Opt-in task line; preserve the original seven-member runner path."""
        from nav_msgs.msg import Odometry
        from qn_aav_simulator.task_line import load_request, load_formation_phase
        from qn_aav_simulator.observation_coverage import CoverageResult
        self.executor_serial = bool(rospy.get_param("~executor_serial", True))
        self.executor_write_mutex = threading.Lock()
        configured = rospy.get_param("~executors")
        self.routing = load_routing(configured, default_members=(),
                                    default_initial_target_ref="start")
        self.units = dispatchable_units(self.routing)
        if not self.units:
            raise ValueError("no online executors")
        self.request_path = Path(rospy.get_param("~request_file"))
        self.request = load_request(self.request_path)
        self.formation_phase = load_formation_phase(self.request_path)
        self.fleet = sorted({m for u in self.units for m in u.physical_agent_ids})
        self.condition = threading.Condition()
        self.actual = {}
        self.executor_diagnostics={}
        self.goal_ids, self.native_results = {}, {}
        self.active_executor_ids = set(rospy.get_param("~resource_locks", []))
        if self.active_executor_ids:
            raise RuntimeError("UNKNOWN_LOCKED from previous run: restart the whole execution chain")
        self.clients, self.action_subs = {}, []
        self.server_nodes, self.member_slots = {}, {}
        self.coverage = CoverageResult()
        self.final_events = {}
        self.plan_revision = 0
        self.speed = float(rospy.get_param("~nominal_speed_mps", 1.5))
        self.seed = int(rospy.get_param("~seed", 0))
        self.metrics = {"planning_mode": self.planning_mode, "status": "STANDBY",
                        "executor_serial": self.executor_serial, "current_actions": {},
                        "request_id": self.request.request_id, "executions": [],
                        "plan_history": [], "results_received": [], "failure_reason": "",
                        "observation_model": "DECLARED_GEOMETRY_AND_DWELL_ONLY",
                        "payload_quality": "UNVERIFIED", "delivery_model": "ZERO_LATENCY_LOCAL_RESULT",
                        "formation_business_shape_verdict": "NOT_DEFINED"}
        self.plan = None
        self.finite_delivery=bool(rospy.get_param('/mission/request_file',''))
        self.command_delivery_required=(self.finite_delivery and
            self.planning_mode=='joint_request' and self.request.template_id!='OFFSHORE_JOINT')
        if self.finite_delivery:
            from std_msgs.msg import String
            self.metrics['delivery_model']=('TASK_SERVICE' if self.request.template_id=='OFFSHORE_JOINT'
                else 'FINITE_DECLARED_EXPERIMENT')
            self.metrics['received_products']={}
            self.metrics['received_terminal_reports']={}
            self.metrics['received_action_results']={}
            self.action_subs.append(rospy.Subscriber('/mother/received_products',String,
                self._on_received_product,queue_size=100))
            self.action_subs.append(rospy.Subscriber('/mother/received_notifications',String,
                self._on_received_notification,queue_size=100))
            if self.request.template_id=='OFFSHORE_JOINT':
                self.state_claim_requests=rospy.Publisher('/mother/state_claim_requests',String,queue_size=20)
                self.metrics['state_claim_requests']={}
                self.metrics['received_state_claims']={}
            if self.command_delivery_required:
                self.command_requests=rospy.Publisher('/mother/command_requests',String,queue_size=100)
                self.state_claim_requests=rospy.Publisher('/mother/state_claim_requests',String,queue_size=20)
                self.metrics['command_requests']={}
                self.metrics['command_deliveries']={}
                self.metrics['state_claim_requests']={}
                self.metrics['received_state_claims']={}
                self.action_subs.append(rospy.Subscriber('/mother/command_deliveries',String,
                    self._on_command_delivery,queue_size=100))
        for unit in self.units:
            endpoint = unit.action_endpoint.rstrip("/")
            action_type,goal_type,result_type=FormationAction,FormationActionGoal,FormationActionResult
            if unit.action_type=='PlatformTaskAction':
                from qn_aav_simulator.msg import PlatformTaskAction,PlatformTaskActionGoal,PlatformTaskActionResult
                action_type,goal_type,result_type=PlatformTaskAction,PlatformTaskActionGoal,PlatformTaskActionResult
            self.clients[unit.executor_id] = actionlib.SimpleActionClient(endpoint, action_type)
            self.action_subs.extend([
                rospy.Subscriber(endpoint + "/goal", goal_type, self.on_goal, queue_size=20),
                rospy.Subscriber(endpoint + "/result", result_type, self.on_result, queue_size=20)])
        sources={}
        for unit in self.units:
            for member,topic in unit.odometry_topics:
                if member in sources and sources[member]!=topic:raise ValueError('conflicting physical state sources: '+member)
                sources[member]=topic
        for member,topic in sources.items():
            self.action_subs.append(rospy.Subscriber(topic, Odometry,
                self._on_executor_odom, callback_args=member, queue_size=10))
            self.action_subs.append(rospy.Subscriber(topic.rsplit('/',1)[0]+'/diagnostics',DiagnosticArray,
                self._on_executor_diagnostics,callback_args=member,queue_size=10))

    def _on_executor_diagnostics(self,message,member):
        for status in message.status:
            if status.hardware_id!=member:continue
            values={v.key:v.value for v in status.values}
            with self.condition:
                self.executor_diagnostics[member]=(message.header.stamp.to_sec(),values)
                self.condition.notify_all()

    def _on_received_product(self,message):
        """Task authority consumes receiver events, never transport truth/state."""
        from qn_aav_simulator.observation_coverage import PointObservation,record_delivery
        try:
            event=json.loads(message.data);key=event['point_id'];ident=event['product_id']
            if (event['request_id']!=self.request.request_id or event['observed'] is not True or
                    (self.request.template_id!='OFFSHORE_JOINT' and
                     event.get('required_bytes')!=32768) or
                    event['result']['model']!='GEOMETRIC_PROXY' or
                    not all(math.isfinite(event[k]) for k in ('generated_at','received_at')) or
                    event['received_at']<event['generated_at'] or
                    not math.isfinite(event['result']['dwell_s']) or
                    event['result']['dwell_s']+1e-9<self.request.requirement.min_dwell_s):
                raise ValueError('invalid received product contract')
            with self.condition:goals={k:set(v) for k,v in self.goal_ids.items()}
            with self.executor_mutex:
                accepted=False
                for item in self.plan.items if self.plan is not None else ():
                    if item.status not in ('RUNNING','COMPLETED','UNKNOWN_LOCKED'):continue
                    for index,step in enumerate(item.execution_steps):
                        execution_id=item.execution_id if len(item.execution_steps)==1 else item.execution_id+':step:'+str(index)
                        if (step.native_action is not None and key in step.native_action.observation_ids and
                                event['producer'] in item.coalition and event['goal_id'] in goals.get(execution_id,())):
                            accepted=True
                        if (step.native_action is None and
                                key in step.observation_ids and
                                event['producer'] in item.coalition and event['goal_id'] in goals.get(execution_id,())):
                            accepted=True
                if not accepted:raise ValueError('receipt is not from this plan execution')
                report=self.metrics.get('received_terminal_reports',{}).get(event['goal_id'])
                if report is not None and key not in report['observed_ids']:
                    raise ValueError('product contradicts the received local terminal report')
                old=self.metrics['received_products'].get(ident)
                if old is not None:
                    if old!=event:raise ValueError('conflicting duplicate receipt')
                    return
                self.metrics['received_products'][ident]=event
                self.coverage.points[key]=PointObservation(key,True,event['producer'],event['result']['dwell_s'],
                    'received local geometric observation; payload quality unverified')
                record_delivery(self.coverage,[key])
            self._save_executor()
        except (ValueError,TypeError,KeyError) as error:
            rospy.logerr_throttle(2.,'Rejected product receipt: %s',str(error))

    def _on_received_notification(self,message):
        """Only an actual mother receipt can close a negative observation report."""
        try:
            event=json.loads(message.data)
            if event.get('event_type')=='ACTION_TERMINAL':
                if getattr(self,'command_delivery_required',False):self._record_action_terminal(event)
                return
            if event.get('event_type')=='STATE_CLAIM':
                self._record_state_claim(event)
                return
            if event.get('event_type')!='OBSERVATION_TERMINAL':return
            if (event['request_id']!=self.request.request_id or
                    event['product_id']!=event['goal_id']+':terminal' or
                    not all(math.isfinite(event[k]) for k in ('generated_at','received_at')) or
                    event['received_at']<event['generated_at']):
                raise ValueError('invalid terminal receipt')
            with self.condition:goals={k:set(v) for k,v in self.goal_ids.items()}
            with self.executor_mutex:
                matching=[]
                for item in self.plan.items if self.plan is not None else ():
                    if item.status not in ('RUNNING','COMPLETED','UNKNOWN_LOCKED'):continue
                    for index,step in enumerate(item.execution_steps):
                        execution_id=item.execution_id if len(item.execution_steps)==1 else item.execution_id+':step:'+str(index)
                        if (step.native_action is not None and
                                event['producer'] in item.coalition and event['goal_id'] in goals.get(execution_id,())):
                            matching.extend(step.native_action.observation_ids)
                        if (step.native_action is None and
                                event['producer'] in item.coalition and event['goal_id'] in goals.get(execution_id,())):
                            matching.extend(step.observation_ids)
                if sorted(matching)!=event['point_ids'] or not set(event['observed_ids'])<=set(matching):
                    raise ValueError('terminal report is not from this planned observation')
                received={row['point_id'] for row in self.metrics['received_products'].values()
                          if row.get('goal_id')==event['goal_id'] and row.get('observed') is True}
                if not received<=set(event['observed_ids']):
                    raise ValueError('terminal report contradicts already received products')
                previous=self.metrics['received_terminal_reports'].get(event['goal_id'])
                if previous is not None:
                    if previous!=event:raise ValueError('conflicting repeated terminal receipt')
                    return
                self.metrics['received_terminal_reports'][event['goal_id']]=event
            self._save_executor()
        except (ValueError,TypeError,KeyError) as error:
            rospy.logerr_throttle(2.,'Rejected terminal receipt: %s',str(error))

    def _record_state_claim(self,event):
        ident=event['claim_id'];member=event['producer']
        with self.executor_mutex:
            requested=self.metrics['state_claim_requests'].get(ident)
            if requested is None or requested['receiver']!=member or (
                    event['request_id']!=self.request.request_id or
                    event['product_id']!=ident+':state' or
                    event['generated_at']<requested['generated_at'] or
                    event['received_at']<event['generated_at'] or
                    not all(math.isfinite(event[key]) for key in
                            ('generated_at','received_at','model_time_s')) or
                    event['model_time_s']<0 or len(event['position'])!=3 or
                    not all(math.isfinite(v) for v in event['position']) or
                    event['actual_mode'] not in ('AIR','WATER','SURFACE','TRANSITION') or
                    event.get('active_goal_id') or event.get('resource_locked')!='false'):
                raise ValueError('state claim is not the requested idle physical member')
            digest=event.get('terminal_state_digest')
            if member.startswith('drone_') and (
                    not isinstance(digest,str) or len(digest)!=64 or
                    any(char not in '0123456789abcdef' for char in digest)):
                raise ValueError('AIR state claim lacks a local full-state digest')
            previous=self.metrics['received_state_claims'].get(ident)
            if previous is not None:
                if previous!=event:raise ValueError('conflicting repeated finite state claim')
                return
            self.metrics['received_state_claims'][ident]=event
        self._save_executor()

    def _await_state_claims(self,members,deadline):
        """Request only the small state facts consumed by this repair."""
        from std_msgs.msg import String
        requested={}
        for member in members:
            ident=hashlib.sha256('|'.join((self.request.request_id,str(self.plan_revision),
                'repair-state',member)).encode('utf-8')).hexdigest()
            event=dict(request_id=self.request.request_id,claim_id=ident,
                receiver=member,generated_at=rospy.Time.now().to_sec())
            with self.executor_mutex:self.metrics['state_claim_requests'][ident]=event
            requested[member]=ident
            self.state_claim_requests.publish(String(data=json.dumps(event,allow_nan=False)))
        self._save_executor()
        while not rospy.is_shutdown() and time.monotonic()<deadline:
            with self.executor_mutex:
                claims={member:self.metrics['received_state_claims'].get(ident)
                        for member,ident in requested.items()}
            if all(claims.values()):return claims
            time.sleep(.05)
        raise RuntimeError('finite state claim missing before repair deadline; no new Goal')

    def _record_action_terminal(self,event):
        if (event['request_id']!=self.request.request_id or
                event['product_id']!=event['goal_id']+':action_terminal:'+event['producer'] or
                event['terminal_state'] not in ('SUCCEEDED','CANCELED','ABORTED') or
                any(type(event[name]) is not bool for name in
                    ('task_completed','terminal_verified','resource_locked')) or
                not all(math.isfinite(event[name]) for name in ('generated_at','received_at')) or
                event['received_at']<event['generated_at']):
            raise ValueError('invalid received Action terminal notice')
        with self.condition:goals={key:set(value) for key,value in self.goal_ids.items()}
        with self.executor_mutex:
            matches=[step for item in (self.plan.items if self.plan is not None else ())
                if item.status in ('RUNNING','COMPLETED','UNKNOWN_LOCKED')
                for index,step in enumerate(item.execution_steps)
                if event['producer'] in item.coalition and
                event['goal_id'] in goals.get(
                    item.execution_id if len(item.execution_steps)==1 else
                    item.execution_id+':step:'+str(index),())]
            if not matches:raise ValueError('Action notice is not from this Plan GoalID')
            digest=event.get('terminal_state_digest')
            if digest is not None and (not isinstance(digest,str) or len(digest)!=64 or
                    any(char not in '0123456789abcdef' for char in digest)):
                raise ValueError('invalid received native state digest')
            expected={step.native_prediction.get('terminal_state_digest')
                      for step in matches if step.native_prediction.get('terminal_state_digest')}
            if len(expected)>1:raise ValueError('conflicting predicted native terminal states')
            if expected:
                event['nominal_terminal_state_match']=digest==next(iter(expected))
                if not event['nominal_terminal_state_match']:
                    self.plan.validation_scope='EXECUTION_ENTRY_REQUALIFICATION_REQUIRED'
            bucket=self.metrics['received_action_results'].setdefault(event['goal_id'],{})
            previous=bucket.get(event['producer'])
            if previous is not None:
                if previous!=event:raise ValueError('conflicting repeated Action terminal notice')
                return
            bucket[event['producer']]=event
        self._save_executor()

    def _on_command_delivery(self,message):
        try:
            event=json.loads(message.data)
            ident=event['command_id']
            with self.executor_mutex:
                expected=self.metrics['command_requests'].get(ident)
                if (expected is None or any(event.get(k)!=v for k,v in expected.items()) or
                        not math.isfinite(event['received_at']) or
                        event['received_at']<expected['generated_at']):
                    raise ValueError('command receipt is not from this plan and Goal payload')
                previous=self.metrics['command_deliveries'].get(ident)
                if previous is not None:
                    if previous!=event:raise ValueError('conflicting duplicate command receipt')
                    return
                self.metrics['command_deliveries'][ident]=event
            self._save_executor()
        except (ValueError,TypeError,KeyError) as error:
            rospy.logerr_throttle(2.,'Rejected command delivery: %s',str(error))

    def _announce_command(self,item,unit,payload,kind='goal'):
        if not getattr(self,'command_delivery_required',False):return ()
        from std_msgs.msg import String
        encoded=io.BytesIO();payload.serialize(encoded)
        data=encoded.getvalue()
        if not data:raise RuntimeError('empty native command payload')
        digest=hashlib.sha256(data).hexdigest();identities=[]
        for member in unit.physical_agent_ids:
            identity='|'.join((self.request.request_id,str(self.plan_revision),
                               item.execution_id,kind,member,digest))
            ident=hashlib.sha256(identity.encode('utf-8')).hexdigest()
            with self.executor_mutex:
                previous=self.metrics['command_requests'].get(ident)
                if previous is None:
                    previous=dict(command_id=ident,request_id=self.request.request_id,
                        plan_revision=self.plan_revision,execution_id=item.execution_id,
                        receiver=member,goal_digest=digest,required_bytes=len(data),
                        generated_at=rospy.Time.now().to_sec())
                    self.metrics['command_requests'][ident]=previous
            self.command_requests.publish(String(data=json.dumps(previous,allow_nan=False)))
            identities.append(ident)
        self._save_executor()
        return tuple(identities)

    def _await_command_delivery(self,identities,deadline):
        if not identities:return
        while not rospy.is_shutdown():
            with self.executor_mutex:
                requests=self.metrics['command_requests']
                delivered=self.metrics['command_deliveries']
                if any(requests[ident]['plan_revision']!=self.plan_revision for ident in identities):
                    raise RuntimeError('plan changed before finite command delivery; retain member reservation')
                if all(ident in delivered for ident in identities):return
            if time.monotonic()>=deadline:
                raise RuntimeError('finite command delivery unverified before observation deadline; retain member reservation')
            time.sleep(.05)
        raise RuntimeError('shutdown before command delivery; retain member reservation')

    def _on_executor_odom(self, message, member):
        from qn_aav_simulator.odometry import parse_standard_odometry, OdometryContractError
        try:
            sample = parse_standard_odometry(message, member)
        except OdometryContractError:
            with self.condition:
                self.actual.pop(member, None)
            return
        with self.condition:
            self.actual[member] = sample
            self.condition.notify_all()
        hold=getattr(self,'opaque_hold',None)
        if hold is not None and member==hold['member'] and math.dist(sample.position,hold['position'])>hold['radius_m']:
            hold['violation']='retained AIR member left its qualified return ball'

    def _wait_executor_ready(self, unit, timeout=120.0):
        import rosgraph
        endpoint = unit.action_endpoint.rstrip("/")
        deadline = time.monotonic() + timeout
        client = self.clients[unit.executor_id]
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if client.wait_for_server(rospy.Duration(.2)):
                publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
                nodes = publishers.get(endpoint + "/status", [])
                if len(nodes) != 1:
                    raise RuntimeError("Action endpoint must have exactly one owner: " + endpoint)
                node = nodes[0]
                self.server_nodes[unit.executor_id] = node
                if unit.action_type=='PlatformTaskAction':
                    if len(unit.physical_agent_ids)!=1:raise RuntimeError('native endpoint must own one physical member')
                    member=unit.physical_agent_ids[0]
                    declared=rospy.get_param(node+'/agent_id',None)
                    if declared is None:
                        drone=rospy.get_param(node+'/drone_id',None)
                        declared=None if drone is None else 'drone_'+str(drone)
                    if declared!=member:raise RuntimeError('native endpoint member disagrees with routing')
                    with self.condition:
                        stamp,values=self.executor_diagnostics.get(member,(None,{}))
                    now=rospy.Time.now().to_sec()
                    if stamp is not None and 0<=now-stamp<=.25:
                        if any(str(values.get(k,'false')).lower()=='true' for k in
                               ('resource_locked','platform_resource_locked','domain_failure')) or values.get('scene_failure'):
                            raise RuntimeError('native member locked or unsafe: '+member)
                        busy=(values.get('active_goal_id') or values.get('pending_goal_id') or
                              values.get('platform_action_active')=='true' or values.get('reference_active')=='true')
                        if not busy:
                            self.metrics['native_qualification_only']=self.metrics.get('native_qualification_only',False) or bool(rospy.get_param(node+'/qualification_only',True))
                            self.member_slots[unit.executor_id]={member:(0.,0.,0.)}
                            return
                    rospy.sleep(.1)
                    continue
                if rospy.get_param(node + "/safety_latched_members", []):
                    raise RuntimeError("physical member safety hold locked: " + endpoint)
                if (rospy.get_param(node + "/ready", False) and
                        rospy.get_param(node + "/run_state", "") == "READY_IDLE"):
                    declared = rospy.get_param(node + "/agent_ids")
                    if {"drone_{}".format(a) for a in declared} != unit.members():
                        raise RuntimeError("endpoint member configuration disagrees with routing")
                    now=rospy.Time.now().to_sec()
                    with self.condition:
                        physical={member:self.executor_diagnostics.get(member,(None,{}))
                                  for member in unit.physical_agent_ids}
                        actual={member:self.actual.get(member) for member in unit.physical_agent_ids}
                    if any(stamp is None or not 0<=now-stamp<=.25
                           for stamp,_ in physical.values()):
                        rospy.sleep(.1)
                        continue
                    handover=[values for _,values in physical.values()
                              if values.get('reference_handover_enabled')=='true']
                    if any(values.get('reference_active')!='false' or
                           values.get('platform_action_active')!='false' or
                           values.get('actual_mode')!='AIR' or
                           values.get('reference_context_ready')!='true'
                           for values in handover):
                        rospy.sleep(.1)
                        continue
                    settled=True
                    for member,(_,values) in physical.items():
                        if values.get('reference_handover_enabled')!='true':continue
                        tolerance=float(values.get('platform_speed_tolerance_mps','nan'))
                        if not math.isfinite(tolerance) or tolerance<=0:
                            raise RuntimeError('local AIR entry speed contract missing: '+member)
                        sample=actual[member]
                        if (sample is None or not sample.is_fresh(now,.25) or
                                math.sqrt(sum(v*v for v in sample.velocity))>tolerance):
                            settled=False
                    if not settled:
                        rospy.sleep(.1)
                        continue
                    if any(values.get('platform_resource_locked')=='true' or values.get('domain_failure')=='true'
                           for _,values in physical.values()):
                        raise RuntimeError('AIR physical member locked or unsafe: '+endpoint)
                    safety_members = rospy.get_param(node + "/safety_agent_ids", declared)
                    air_members={m for u in self.units if u.action_type=='FormationAction' for m in u.physical_agent_ids}
                    if {"drone_{}".format(a) for a in safety_members} != air_members:
                        raise RuntimeError("endpoint does not monitor whole-fleet collision safety")
                    raw = rospy.get_param(node + "/relative_slots")
                    scale = float(rospy.get_param(node + "/swarm_scale"))
                    self.member_slots[unit.executor_id] = {
                        "drone_" + str(a): tuple(scale*float(x) for x in raw[str(a)]) for a in declared}
                    return
            rospy.sleep(.1)
        node = self.server_nodes.get(unit.executor_id, "")
        raise RuntimeError("endpoint not ready: {}: {}".format(endpoint,
            rospy.get_param(node + "/readiness_reason", "unavailable")))

    def _actual_positions(self):
        now = rospy.Time.now().to_sec()
        # Use the action servers' existing freshness contract.
        timeout = min(float(rospy.get_param(n + "/odom_timeout", .25))
                      for n in self.server_nodes.values())
        with self.condition:
            missing = [m for m in self.fleet if m not in self.actual
                       or not self.actual[m].is_fresh(now, timeout)]
            if missing:
                raise RuntimeError("missing/fresh actual positions: " + str(missing))
            return {m: self.actual[m].position for m in self.fleet}

    def _executor_plan(self, observations, *, include_formation=False, release=0.0):
        from mrta_python import Executor, ExecutorPlan, ExecutorTravelTimeProvider, build_executor_plan
        from qn_aav_simulator.task_line import request_centres, to_plan_tasks
        planning_end = time.monotonic() + float(rospy.get_param("~planning_budget_s", 10.0))
        positions = self._actual_positions()
        centers = request_centres(observations)
        centers["start"] = positions[self.fleet[0]]  # member positions, not this fallback, determine cost
        executors = [Executor(u.executor_id, u.physical_agent_ids, frozenset(u.capabilities),
                              release, self.speed, "start") for u in self.units]
        travel = ExecutorTravelTimeProvider(centers, {u.executor_id: self.speed for u in self.units},
                    member_slots=self.member_slots, member_positions=dict(positions))
        tasks = list(to_plan_tasks(observations))
        plan = build_executor_plan(executors, tasks, travel, initial_target_ref="start", seed=self.seed,
                                  serial=self.executor_serial,
                                  budget_s=None if self.executor_serial else max(1e-9,planning_end-time.monotonic()),
                                  serial_units={u.executor_id for u in self.units}) if tasks else ExecutorPlan(serial=self.executor_serial)
        for item in plan.items:
            for member in item.coalition:
                slot=self.member_slots[item.executor_id][member]
                travel.member_positions[member]=tuple(centers[item.task_id][a]+slot[a] for a in range(3))
        if include_formation and self.formation_phase:
            phase = self.formation_phase
            if not self.executor_serial:
                from qn_aav_simulator.task_line import assembly_member_order
                group=unit_for_coalition(self.routing,self.fleet)
                slots=self.member_slots[group.executor_id]
                targets={m:tuple(phase.path_start[a]+slots[m][a] for a in range(3)) for m in self.fleet}
                node=self.server_nodes[group.executor_id]
                required_distance=(2*float(rospy.get_param(node+'/platform_radius_m',.25))+
                                   float(rospy.get_param(node+'/inter_agent_clearance',.5)))
                order=assembly_member_order(travel.member_positions,targets,required_distance)
                # The native group approach from dispersed survey endpoints
                # failed actual clearance. Use existing member motions in a
                # conflict-filtered order, then retain BOTH group Actions.
                for member in order:
                    task_id=phase.phase_id+'-position-'+member
                    centers[task_id]=targets[member]
                    task=Task(task_id,frozenset({'AIR'}),1,self.request.service_time_s,
                              self.request.deadline_s,task_id,required_members=(member,))
                    from dataclasses import replace
                    staged=[replace(e,available_from=max(release,plan.makespan)) for e in executors]
                    part=build_executor_plan(staged,[task],travel,initial_target_ref='start',seed=self.seed,
                        serial=True,budget_s=max(1e-9,planning_end-time.monotonic()))
                    plan.precedence_edges+=tuple((previous.task_id,task_id) for previous in plan.items)
                    plan.items.extend(part.items)
                    tasks.append(task)
                    travel.member_positions[member]=targets[member]
            # Explicit assembly then transfer stages, appended with the same
            # serial release and physical-member predictions. No online USV clock.
            for suffix, point in (("assemble", phase.path_start), ("transfer", phase.path_end)):
                task_id = phase.phase_id + "-" + suffix
                if task_id in centers:
                    raise ValueError("formation task id collides with observation")
                centers[task_id] = point
                task = Task(task_id, frozenset({"AIR"}), phase.required_agent_count,
                            self.request.service_time_s, self.request.deadline_s, task_id)
                start = max(release, plan.makespan)
                from dataclasses import replace
                staged = [replace(e, available_from=start) for e in executors]
                part = build_executor_plan(staged, [task], travel, initial_target_ref="start", seed=self.seed, serial=True,
                                            budget_s=None if self.executor_serial else max(1e-9,planning_end-time.monotonic()),
                                            serial_units={e.executor_id for e in staged})
                plan.precedence_edges += tuple((previous.task_id,task_id) for previous in plan.items)
                plan.items.extend(part.items)
                for item in part.items:
                    for member in item.coalition:
                        slot=self.member_slots[item.executor_id][member]
                        travel.member_positions[member]=tuple(point[a]+slot[a] for a in range(3))
                tasks.append(task)
        self.centers = getattr(self, "centers", {})
        self.centers.update(centers)
        self.tasks_by_id = getattr(self, "tasks_by_id", {})
        self.tasks_by_id.update({t.task_id: t for t in tasks})
        self.observation_tasks = getattr(self, "observation_tasks", {})
        self.observation_tasks.update({t.task_id: t for t in observations})
        return plan

    def _save_executor(self):
        # Serialise output snapshots without holding the execution lock during
        # filesystem writes or ROS master RPCs. A newer snapshot cannot be
        # overwritten by an older writer that happened to finish later.
        with self.executor_write_mutex:
            with self.executor_mutex:
                metrics,state=self._save_executor_locked()
            save_json(self.output / 'metrics.json',metrics)
            rospy.set_param('~task_state',json.dumps(state,default=json_default,allow_nan=False))
            rospy.set_param('~resource_locks',metrics['resource_locks'])

    def _save_executor_locked(self):
        self.metrics["plan"] = asdict(self.plan) if self.plan is not None else None
        self.metrics["plan_revision"] = self.plan_revision
        self.metrics["resource_locks"] = sorted(self.active_executor_ids)
        self.metrics["coverage"] = asdict(self.coverage)
        self.metrics["observed_fraction"] = self.coverage.observed_fraction(self.weights)
        self.metrics["delivered_fraction"] = self.coverage.delivered_fraction(self.weights)
        self.metrics["updated_at_ros_s"] = rospy.Time.now().to_sec()
        # A compact view of the task authority, using the existing ROS parameter
        # service. No new message definition or independent dashboard completion.
        state = {key: self.metrics[key] for key in (
            "status", "request_id", "plan", "plan_revision", "resource_locks", "coverage",
            "observed_fraction", "delivered_fraction", "results_received", "failure_reason",
            "observation_model", "payload_quality", "delivery_model", "updated_at_ros_s")}
        state["current_action"] = self.metrics.get("current_action")
        state["current_actions"] = self.metrics.get("current_actions", {})
        state["safety_disposition"] = self.metrics.get("safety_disposition")
        for key in ('pending_retest','retest_completed','repair_source','repair_wall_s','return_completion'):
            if key in self.metrics:state[key]=self.metrics[key]
        if 'command_requests' in self.metrics:
            state['command_progress']={
                'requested':len(self.metrics['command_requests']),
                'delivered':len(self.metrics['command_deliveries'])}
        if 'state_claim_requests' in self.metrics:
            state['state_claim_progress']={
                'requested':len(self.metrics['state_claim_requests']),
                'received':len(self.metrics['received_state_claims'])}
        if 'planning_started_monotonic' in self.metrics:
            state['planning_started_monotonic']=self.metrics['planning_started_monotonic']
            state['planning_budget_s']=self.metrics['planning_budget_s']
        import copy
        return copy.deepcopy(self.metrics),copy.deepcopy(state)

    def _refresh_executor_timing(self, release):
        """Fixed remaining assignments, costs refreshed from actual physical members."""
        from mrta_python import ExecutorTravelTimeProvider
        if self.plan.activity_edges or len({i.task_id for i in self.plan.items})!=len(self.plan.items):
            if not any(i.status=='PLANNED' for i in self.plan.items):
                # All remaining activities are already committed. Completion
                # propagation has recorded actual ends; no speculative scalar
                # route refresh is needed or permitted for those activities.
                self.plan_revision+=1
                self.metrics['plan_history'].append({'revision':self.plan_revision,'plan':asdict(self.plan)})
                return
            raise RuntimeError('coordinated activity timing requires method re-evaluation; refusing scalar refresh')
        travel = ExecutorTravelTimeProvider(self.centers,
            {u.executor_id: self.speed for u in self.units}, member_slots=self.member_slots,
            member_positions=self._actual_positions())
        parallel=not self.plan.serial
        availability={}
        finishes={}
        if parallel:
            for committed in self.plan.items:
                if committed.status != "PLANNED":
                    finishes[committed.task_id]=(committed.actual_finish if committed.actual_finish is not None else committed.planned_finish)
                    for member in committed.coalition:
                        availability[member]=max(availability.get(member,release),finishes[committed.task_id])
                        if committed.status=="RUNNING":
                            slot=self.member_slots[committed.executor_id][member]
                            travel.member_positions[member]=(tuple(self._checked_native_prediction(committed)['terminal_position'])
                                if getattr(committed,'native_action',None) else
                                tuple(self.centers[self.tasks_by_id[committed.task_id].target_ref][a]+slot[a] for a in range(3)))
        for item in self.plan.items:
            if item.status != "PLANNED":
                continue
            if len(getattr(item,'execution_steps',()))>1:
                raise RuntimeError('composite plan timing requires complete candidate re-evaluation')
            task = self.tasks_by_id[item.task_id]
            native=getattr(item,'native_action',None)
            if native:
                prediction=self._checked_native_prediction(item)
                item.travel_time=float(prediction['duration_s'])
            else:item.travel_time = travel(item.executor_id, "start", task.target_ref)
            item.planned_start = max(item.planned_start, release)
            if parallel:
                item.planned_start=max(item.planned_start,
                    max((availability.get(m,release) for m in item.coalition),default=release),
                    max((finishes[p] for p,after in self.plan.precedence_edges if after==item.task_id),default=release))
            item.planned_finish = item.planned_start + item.travel_time + item.service_time
            if parallel:
                finishes[item.task_id]=item.planned_finish
                for member in item.coalition:availability[member]=item.planned_finish
            else:
                release = item.planned_finish
            for member in item.coalition:
                slot = self.member_slots[item.executor_id][member]
                travel.member_positions[member] = (tuple(item.native_prediction['terminal_position']) if native else
                    tuple(self.centers[task.target_ref][a] + slot[a] for a in range(3)))
        self.plan_revision += 1
        self.metrics["plan_history"].append({"revision": self.plan_revision,
                                            "plan": asdict(self.plan)})

    @staticmethod
    def _release_result_ok(state, result, execution_id):
        return (state == GoalStatus.SUCCEEDED and result is not None
                and result.task_id == execution_id and bool(result.goal_id)
                and result.reason == 0 and result.task_outcome == 1
                and result.safety_outcome == 1 and result.experiment_validity == 1)

    @staticmethod
    def _checked_native_prediction(item):
        prediction=item.native_prediction
        if 'terminal_backend' in prediction or 'trajectory' in prediction:
            raise RuntimeError('internal model/trajectory must not be copied into plan metadata')
        if prediction.get('status')!='FEASIBLE' or not prediction.get('geometry_checked'):
            raise RuntimeError('native plan lacks a complete checked motion prediction')
        duration=float(prediction['duration_s']);position=prediction['terminal_position']
        if not math.isfinite(duration) or duration<0 or len(position)!=3 or not all(math.isfinite(v) for v in position):
            raise RuntimeError('invalid native motion prediction')
        if prediction.get('terminal_mode')!=item.native_action.final_mode:
            raise RuntimeError('native prediction final mode mismatch')
        if item.service_time!=0:
            raise RuntimeError('native action duration already includes its terminal; scalar service would double count')
        return prediction

    def _executor_goal(self,item,unit):
        """Encode the selected task operation in its actual native Action type.

        This is a codec at the existing runner boundary, not a second plan or
        a backend factory. Physical ownership remains keyed by unit members.
        """
        if item.wait_time>0:
            raise RuntimeError('qualified waiting must execute before motion; refusing to omit it')
        if len(getattr(item,'execution_steps',()))>1:
            raise RuntimeError('composite candidate chain requires step dispatch; refusing to truncate it')
        task=self.tasks_by_id[item.task_id]
        native=getattr(item,'native_action',None)
        if native is None:
            if unit.action_type!='FormationAction':raise RuntimeError('AIR task routed to non-AIR Action')
            goal=FormationGoal();goal.task_id=item.execution_id
            goal.formation_center.header.frame_id='world'
            goal.formation_center.header.stamp=rospy.Time.now()
            step=item.execution_steps[0] if item.execution_steps else None
            target_ref=step.target_ref if step else task.target_ref
            goal.formation_center.point.x,goal.formation_center.point.y,goal.formation_center.point.z=self.centers[target_ref]
            goal.hold_duration=rospy.Duration(step.service_time_s if step else item.service_time)
            observation_ids=(list(step.observation_ids) if step is not None else
                list(getattr(self.observation_tasks.get(item.task_id),'covers',()))) if getattr(self,'finite_delivery',False) else []
            if hasattr(goal,'observation_ids'):
                goal.observation_ids=observation_ids
            elif observation_ids:
                raise RuntimeError('Formation.action lacks observation_ids; rebuild the Noetic message image')
            return goal
        if unit.action_type!='PlatformTaskAction':raise RuntimeError('native fragment routed to AIR Action')
        self._checked_native_prediction(item)
        if not {s.operation for s in native.segments}<=set(unit.operations):
            raise RuntimeError('selected unit does not implement every fragment operation')
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        from qn_aav_simulator.msg import PlatformTaskGoal,PlatformSegment
        goal=PlatformTaskGoal(task_id=item.execution_id,terminal_behavior=native.terminal_behavior,
                              execution_timeout=rospy.Duration(native.execution_timeout_s),
                              observation_ids=list(native.observation_ids),
                              terminal_wait=rospy.Duration(native.terminal_wait_s))
        for segment in native.segments:
            path=RosPath();path.header.frame_id='world'
            for xyz in segment.points:
                point=PoseStamped();point.header.frame_id='world';point.pose.orientation.w=1.
                point.pose.position.x,point.pose.position.y,point.pose.position.z=xyz;path.poses.append(point)
            goal.segments.append(PlatformSegment(operation=segment.operation,path=path,
                duration=rospy.Duration(segment.duration_s),propulsion_effort=segment.propulsion_effort))
        return goal

    @staticmethod
    def _native_motion_result_ok(state,result,execution_id,native):
        reason={'FIXED_REFERENCE':'COMPLETED_LOCAL_FRAGMENT','COAST_STOP':'COAST_STOP_VERIFIED_IN_QUALIFICATION',
                'TRIM_PROPULSION':'TRIM_PROPULSION_VERIFIED_IN_QUALIFICATION'}[native.terminal_behavior]
        return (state==GoalStatus.SUCCEEDED and result is not None and
                result.task_id==execution_id and bool(result.goal_id) and
                result.task_completed and result.terminal_verified and not result.resource_locked and
                result.actual_mode==native.final_mode and result.reason==reason)

    def _member_return_complete(self,member,distance,site):
        if member!='uuv' or not any(item.native_action is not None and
                                    item.native_action.terminal_behavior=='COAST_STOP'
                                    for item in self.plan.items if member in item.coalition):
            return distance<=site['radius_m'],'ACTUAL_TERMINAL_IN_RETURN_SITE'
        # The REMUS endpoint verifies observation-after-departure, re-entry
        # and its safe coast tail before this specific native Result succeeds.
        # Its final position can legitimately lie outside the return ball.
        activities=[item for item in self.plan.items if member in item.coalition]
        if activities:
            item=max(activities,key=lambda item:(item.actual_finish if item.actual_finish is not None
                                                 else item.planned_finish,item.execution_id))
            native=item.native_action
            if (item.fulfills_task and item.status=='COMPLETED' and native is not None and
                    native.terminal_behavior=='COAST_STOP' and native.observation_ids):
                for row in self.metrics['executions']:
                    result=row.get('native_result')
                    if (row.get('execution_id')==item.execution_id and row.get('result')=='SUCCEEDED' and
                            result is not None and result.get('goal_id')==row.get('goal_id') and
                            result.get('task_completed') and result.get('terminal_verified') and
                            not result.get('resource_locked') and result.get('actual_mode')=='WATER'):
                        return True,'NATIVE_REENTRY_AND_COAST_RESULT'
        return False,'NATIVE_REENTRY_AND_COAST_RESULT_MISSING'

    def _wait_observation_receipt(self,goal_id,point_ids,timeout,require_products=True):
        """Wait for this Goal's report and every product it says was observed."""
        wanted=set(point_ids);deadline=time.monotonic()+timeout
        while not rospy.is_shutdown():
            with self.executor_mutex:
                received={event['point_id'] for event in self.metrics.get('received_products',{}).values()
                          if event.get('goal_id')==goal_id and event.get('observed') is True}
                report=self.metrics.get('received_terminal_reports',{}).get(goal_id)
            if report is not None:
                observed=set(report['observed_ids'])
                if not observed<=wanted or not received<=observed:
                    raise RuntimeError('observation report conflicts with received products; keep member reserved')
                if not require_products or observed<=received:return
            if time.monotonic()>=deadline:
                raise RuntimeError('observed products or terminal report not received; keep member reserved')
            time.sleep(.05)
        raise RuntimeError('shutdown before AIR observation receipt; keep member reserved')

    def _wait_native_observation_receipt(self,state,result,native):
        if native is None or result is None or not native.observation_ids:return
        if state==GoalStatus.SUCCEEDED:
            # Native motion may finish before the finite link delivers its
            # product. Keep the physical booking while the receiver event is
            # in flight; the matching GoalID is checked before commit below.
            self._wait_observation_receipt(result.goal_id,native.observation_ids,
                                           native.execution_timeout_s,
                                           require_products=self.request.template_id!='OFFSHORE_JOINT')
        elif (state==GoalStatus.ABORTED and result.reason=='OBSERVATION_NOT_SATISFIED'
                and result.terminal_verified and not result.resource_locked):
            # A partial negative report still names positively observed points.
            # Keep this booking until those products and the report arrive.
            self._wait_observation_receipt(result.goal_id,native.observation_ids,
                                           native.execution_timeout_s,
                                           require_products=self.request.template_id!='OFFSHORE_JOINT')

    def _wait_action_terminal_receipt(self,goal_id,members,state,timeout):
        deadline=time.monotonic()+timeout
        expected={GoalStatus.SUCCEEDED:'SUCCEEDED',GoalStatus.PREEMPTED:'CANCELED',
                  GoalStatus.RECALLED:'CANCELED',GoalStatus.ABORTED:'ABORTED'}.get(state)
        if expected is None:raise RuntimeError('Action has no known terminal status; keep member reserved')
        while not rospy.is_shutdown():
            with self.executor_mutex:
                delivered=dict(self.metrics['received_action_results'].get(goal_id,{}))
            if all(member in delivered for member in members):
                if any(delivered[member]['terminal_state']!=expected for member in members):
                    raise RuntimeError('finite Action notice disagrees with Goal terminal state; keep member reserved')
                return
            if time.monotonic()>=deadline:
                raise RuntimeError('Action terminal notice not received through finite link; keep member reserved')
            time.sleep(.05)
        raise RuntimeError('shutdown before Action terminal notice; keep member reserved')

    def _dispatch_executor_item(self, item, reserved=False,retain_booking=False):
        if len(item.execution_steps)>1:
            return self._dispatch_executor_chain(item,reserved,retain_booking)
        from mrta_python.repair import process_executor_completion
        task=self.tasks_by_id[item.task_id]
        native=getattr(item,'native_action',None)
        operations=tuple(s.operation for s in native.segments) if native else ('AIR_MOVE',)
        unit = unit_for_coalition(self.routing, item.coalition,executor_id=item.executor_id,operations=operations)
        if unit is None or unit.executor_id != item.executor_id:
            raise RuntimeError("plan coalition has no matching online endpoint")
        if not reserved and conflicting_active_unit(self.routing, self.active_executor_ids, unit.executor_id):
            raise RuntimeError("physical members already occupied")
        self._wait_executor_ready(unit)
        goal = self._executor_goal(item,unit)
        action = {"task_id": item.task_id, "execution_id": item.execution_id,
                                         "endpoint": unit.action_endpoint, "phase": "DISPATCHING"}
        with self.executor_mutex:
            item.status = "RUNNING"
            self.active_executor_ids.add(unit.executor_id)
            self.metrics.setdefault("current_actions", {})[item.execution_id] = action
            if getattr(self,"executor_serial",True):self.metrics["current_action"] = action
        self._save_executor()  # reserve before sending, survive runner interruption
        state,result=self._send_executor_goal(item,unit,goal,action)
        self._wait_native_observation_receipt(state,result,native)
        if native is None and getattr(goal,'observation_ids',()) and self._release_result_ok(state,result,item.execution_id):
            node=self.server_nodes[unit.executor_id]
            self._wait_observation_receipt(result.goal_id,goal.observation_ids,
                float(rospy.get_param(node+'/execution_timeout',180.)),
                require_products=self.request.template_id!='OFFSHORE_JOINT')
        # Only result commits hold this mutex, never physical Action waits.
        with self.executor_mutex:
            outcome=self._commit_executor_result(item,unit,state,result,retain_booking=retain_booking)
        self._save_executor()
        return outcome

    def _send_executor_goal(self,item,unit,goal,action,sent_deadline=None,
                            defer_terminal_receipt=False):
        native=item.native_action
        client = self.clients[unit.executor_id]
        node=self.server_nodes[unit.executor_id]
        def feedback(message):
            with self.executor_mutex:
                action["phase"] = (message.operation if native else 'HOLDING' if message.phase==1 else 'MOVING')
        if sent_deadline is None:
            if getattr(self,'command_delivery_required',False):
                command_budget=(native.execution_timeout_s if native else
                    float(rospy.get_param(node+'/execution_timeout',180.)))
                identities=self._announce_command(item,unit,goal)
                self._await_command_delivery(identities,time.monotonic()+command_budget)
            client.send_goal(goal, feedback_cb=feedback)
        disposition_budget = (float(rospy.get_param(node + "/safety_hold_timeout_s", 180))
                              if rospy.get_param(node + "/safety_hold_enabled", False) else 0.0)
        deadline = (sent_deadline if sent_deadline is not None else time.monotonic() +
            (native.execution_timeout_s if native else float(rospy.get_param(node + "/execution_timeout", 180))) + disposition_budget + 10.0)
        while not rospy.is_shutdown():
            if client.wait_for_result(rospy.Duration(.5)):
                break
            disposition = rospy.get_param(node + "/safety_disposition", None)
            if disposition is not None:
                with self.executor_mutex:
                    action["phase"] = "SAFETY_HOLD"
                    self.metrics["safety_disposition"] = json.loads(disposition)
            self._save_executor()
            if time.monotonic() >= deadline:
                raise RuntimeError("result timeout; endpoint={} client_state={} server_state={}".format(
                    unit.action_endpoint, client.get_state(), rospy.get_param(node + "/run_state", "unknown")))
        result = client.get_result()
        state = client.get_state()
        if (getattr(self,'command_delivery_required',False) and not defer_terminal_receipt and
                result is not None and result.goal_id):
            self._wait_action_terminal_receipt(result.goal_id,unit.physical_agent_ids,state,
                native.execution_timeout_s if native else float(rospy.get_param(node+'/execution_timeout',180.)))
        return state,result

    def _dispatch_cooperative_items(self,items):
        """Prepare all native participants before any dependent local start.

        Scheduler already owns one atomic reservation. State remains in the
        plan/current_actions; this is a branch of the existing worker, not a
        second coordinator. RPC completion is bounded even for a hung call.
        """
        if any(item.native_action is None for item in items):
            return self._dispatch_air_support_items(items)
        from qn_aav_simulator.srv import StartPreparedAction
        from mrta_python.executors import activity_predecessors
        dependencies=activity_predecessors(self.plan)
        indices={item.execution_id:index for index,item in enumerate(items)}
        units=[];goals=[];actions=[];sent=[];owned=set()
        for item in items:
            native=item.native_action
            if native is None or len(item.execution_steps)!=1:
                raise RuntimeError('cooperative preparation requires one finite native fragment per participant')
            unit=unit_for_coalition(self.routing,item.coalition,executor_id=item.executor_id,
                operations=tuple(s.operation for s in native.segments))
            if unit is None or owned.intersection(unit.physical_agent_ids):
                raise RuntimeError('cooperative participant has no endpoint or repeats physical members')
            owned.update(unit.physical_agent_ids)
            self._wait_executor_ready(unit)
            goal=self._executor_goal(item,unit);goal.prepare_only=True
            action=dict(task_id=item.task_id,execution_id=item.execution_id,endpoint=unit.action_endpoint,phase='PREPARING')
            units.append(unit);goals.append(goal);actions.append(action)
        # The coordinated booking ends only after its slowest accepted member
        # has a matching terminal Result. Each Action keeps its own timeout;
        # using the shortest one here canceled a valid USV support return.
        deadline=time.monotonic()+max(i.native_action.execution_timeout_s for i in items)
        with self.executor_mutex:
            for item,action in zip(items,actions):self.metrics.setdefault('current_actions',{})[item.execution_id]=action
        self._save_executor()
        try:
            commands=tuple(ident for item,unit,goal in zip(items,units,goals)
                for ident in self._announce_command(item,unit,goal))
            self._await_command_delivery(commands,deadline)
            for item,unit,goal,action in zip(items,units,goals,actions):
                def feedback(message,a=action):
                    with self.executor_mutex:
                        a.update(phase=message.operation,reference_generation=message.reference_generation,
                                 model_time_s=getattr(message,'model_time_s',None))
                sent.append(unit)
                self.clients[unit.executor_id].send_goal(goal,feedback_cb=feedback)
            while not rospy.is_shutdown():
                if time.monotonic()>=deadline:raise RuntimeError('cooperative preparation observation expired')
                if any(self.clients[u.executor_id].get_state() not in (0,1) for u in units):
                    raise RuntimeError('participant rejected or terminated before common preparation')
                with self.condition:
                    identities=[tuple(self.goal_ids.get(i.execution_id,())) for i in items]
                with self.executor_mutex:
                    prepared=all(a['phase']=='PREPARED' and 'reference_generation' in a for a in actions)
                if prepared and all(len(g)==1 for g in identities):break
                time.sleep(.02)
            if rospy.is_shutdown():raise RuntimeError('shutdown before cooperative start')
            with self.executor_mutex:
                generations=[a['reference_generation'] for a in actions]
                prepared_times=[a.get('model_time_s') for a in actions]
                for action in actions:action['all_participants_prepared']=True
                self.metrics.setdefault('events',[]).append(dict(kind='COOPERATIVE_PREPARED',
                    executions=[i.execution_id for i in items],goal_ids=[g[0] for g in identities],
                    generations=generations,at_ros_s=rospy.Time.now().to_sec()))
            self._save_executor()
            replies=queue.Queue()
            def release(index):
                try:
                    item=items[index];unit=units[index]
                    service=self.server_nodes[unit.executor_id]+'/start_prepared'
                    if getattr(self,'command_delivery_required',False):
                        from qn_aav_simulator.srv import StartPreparedActionRequest
                        request=StartPreparedActionRequest(identities[index][0],generations[index])
                        command=self._announce_command(item,unit,request,kind='start')
                        self._await_command_delivery(command,deadline)
                    rospy.wait_for_service(service,timeout=max(.001,deadline-time.monotonic()))
                    proxy=rospy.ServiceProxy(service,StartPreparedAction)
                    response=(proxy(request) if getattr(self,'command_delivery_required',False) else
                              proxy(identities[index][0],generations[index]))
                    replies.put((index,response.accepted,response.reason))
                except Exception as error:replies.put((index,False,str(error)))
            pending=set(range(len(items)));waiting=set()
            supports={index for index,item in enumerate(items) if not item.fulfills_task}
            def support_started(index):
                with self.executor_mutex:a=dict(actions[index])
                if not a.get('start_accepted'):return False
                client=self.clients[units[index].executor_id]
                if client.get_state()==3:
                    result=client.get_result()
                    return (self._native_motion_result_ok(3,result,items[index].execution_id,items[index].native_action)
                            and result.goal_id==identities[index][0])
                return (a.get('reference_generation')==generations[index] and
                        a.get('phase') in set(units[index].operations)|{
                            items[index].native_action.terminal_behavior,'PRECOMMITTED_WAIT'} and
                        prepared_times[index] is not None and a.get('model_time_s') is not None and
                        a['model_time_s']>prepared_times[index])
            while pending or waiting:
                if rospy.is_shutdown() or time.monotonic()>=deadline:
                    raise RuntimeError('cooperative start state unknown at observation deadline')
                if any(self.clients[u.executor_id].get_state() not in (0,1,3) for u in units):
                    raise RuntimeError('participant failed before all planned releases')
                now=rospy.Time.now().to_sec()-self.epoch
                for index in tuple(pending):
                    # A support RPC merely being in flight is not a launch
                    # commitment. Work stays PREPARED until each required
                    # support has acknowledged and advanced its native program.
                    if items[index].fulfills_task and not all(support_started(s) for s in supports):
                        continue
                    predecessors_ready=True
                    for ident in dependencies[items[index].execution_id] & indices.keys():
                        prior=indices[ident];client=self.clients[units[prior].executor_id]
                        result=client.get_result()
                        if (not self._native_motion_result_ok(client.get_state(),result,ident,items[prior].native_action) or
                                result.goal_id!=identities[prior][0]):
                            predecessors_ready=False;break
                    if now>=items[index].planned_start and predecessors_ready:
                        if items[index].fulfills_task and supports:
                            with self.executor_mutex:
                                self.metrics.setdefault('events',[]).append(dict(kind='SUPPORT_START_CONFIRMED',
                                    execution_id=items[index].execution_id,
                                    support_executions=[items[s].execution_id for s in sorted(supports)],
                                    at_ros_s=rospy.Time.now().to_sec()))
                        pending.remove(index);waiting.add(index)
                        threading.Thread(target=release,args=(index,),daemon=True).start()
                try:index,accepted,reason=replies.get(timeout=min(.05,max(.001,deadline-time.monotonic())))
                except queue.Empty:continue
                waiting.remove(index)
                if not accepted:raise RuntimeError('cooperative start failed: '+reason)
                with self.executor_mutex:
                    actions[index]['start_accepted']=True
                    self.metrics.setdefault('events',[]).append(dict(kind='COOPERATIVE_START_ACCEPTED',
                        execution_id=items[index].execution_id,at_ros_s=rospy.Time.now().to_sec()))
            def observe(index):
                item,unit,goal,action=items[index],units[index],goals[index],actions[index]
                state,result=self._send_executor_goal(item,unit,goal,action,sent_deadline=deadline)
                self._wait_native_observation_receipt(state,result,item.native_action)
                # A cooperative member's Result is a fact, but the atomic
                # booking belongs to the complete accepted method. A second
                # task must not take one member while its support still runs.
                with self.executor_mutex:self._commit_executor_result(item,unit,state,result,retain_booking=True)
                self._save_executor()
            workers=ThreadPoolExecutor(max_workers=len(items))
            try:
                futures=[workers.submit(observe,index) for index in range(len(items))]
                for future in as_completed(futures,timeout=max(.001,deadline-time.monotonic())):future.result()
            finally:workers.shutdown(wait=False)
            with self.executor_mutex:
                if any(self.plan.item(item.execution_id).status!='COMPLETED' for item in items):
                    raise RuntimeError('cooperative method lacks all matching terminal Results')
                for unit in units:self.active_executor_ids.discard(unit.executor_id)
                if any(item.status=='PLANNED' for item in self.plan.items):
                    # Existing native endpoints requalify their actual entry;
                    # no centre-distance scalar update may certify a new
                    # multi-activity motion/communication plan.
                    self.plan.validation_scope='EXECUTION_ENTRY_REQUALIFICATION_REQUIRED'
                    self.plan_revision+=1
                    self.metrics['plan_history'].append({'revision':self.plan_revision,
                        'plan':asdict(self.plan)})
            self._save_executor()
        except BaseException:
            # A cancel request does not prove physical termination. Accepted or
            # uncertain participants keep their reservations on this path.
            for unit in sent:
                try:self.clients[unit.executor_id].cancel_goal()
                except Exception as error:rospy.logerr('cooperative cancel request failed: %s',str(error))
            raise

    def _dispatch_air_support_items(self,items):
        """Start accepted USV support before AIR work; release both together."""
        if (len(items)!=2 or len({item.task_id for item in items})!=1 or
                len({item.candidate_id for item in items})!=1):
            raise RuntimeError('AIR support method needs one work and one support activity')
        work=next((item for item in items if item.fulfills_task and item.native_action is None),None)
        support=next((item for item in items if not item.fulfills_task and item.native_action is not None),None)
        if work is None or support is None or set(work.coalition)&set(support.coalition):
            raise RuntimeError('AIR/USV support roles or physical members are invalid')
        support_unit=unit_for_coalition(self.routing,support.coalition,executor_id=support.executor_id,
            operations=tuple(s.operation for s in support.native_action.segments))
        if support_unit is None or support_unit.action_type!='PlatformTaskAction':
            raise RuntimeError('AIR support has no native USV endpoint')
        with ThreadPoolExecutor(max_workers=2) as workers:
            support_future=workers.submit(self._dispatch_executor_item,support,True,True)
            deadline=time.monotonic()+support.native_action.execution_timeout_s
            while not rospy.is_shutdown():
                if support_future.done():
                    support_future.result()
                    raise RuntimeError('AIR support ended before AIR work could start')
                with self.executor_mutex:
                    phase=self.metrics.get('current_actions',{}).get(support.execution_id,{}).get('phase')
                with self.condition:
                    ids=tuple(self.goal_ids.get(support.execution_id,()))
                if len(ids)==1 and phase in set(support_unit.operations)|{
                        support.native_action.terminal_behavior,'PRECOMMITTED_WAIT'}:
                    break
                if time.monotonic()>=deadline:
                    raise RuntimeError('AIR support start not confirmed; all members remain reserved')
                time.sleep(.02)
            if rospy.is_shutdown():raise RuntimeError('shutdown before AIR support confirmation')
            with self.executor_mutex:
                self.metrics.setdefault('events',[]).append(dict(kind='SUPPORT_START_CONFIRMED',
                    execution_id=work.execution_id,support_execution_id=support.execution_id,
                    support_goal_id=ids[0],at_ros_s=rospy.Time.now().to_sec()))
            self._save_executor()
            # AIR has its own verified terminal and received product. Its
            # physical member becomes available then; USV stays reserved until
            # the cooperative support commitment has also resolved.
            work_future=workers.submit(self._dispatch_executor_item,work,True,False)
            # Both clients have their own finite result/receipt waits. A failed
            # side keeps the already reserved physical members locked.
            for future in as_completed((work_future,support_future)):
                future.result()
        with self.executor_mutex:
            if any(self.plan.item(item.execution_id).status!='COMPLETED' for item in items):
                raise RuntimeError('AIR support group lacks complete matching Results')
            self.active_executor_ids.discard(work.executor_id)
            self.active_executor_ids.discard(support.executor_id)
            if (any(item.status=='PLANNED' for item in self.plan.items) and
                    self.plan.validation_scope!='EXECUTION_ENTRY_REQUALIFICATION_REQUIRED'):
                # The completed group may have shifted a successor's start.
                # Keep that execution order, but withdraw the old full-model
                # validation until its native endpoint checks the actual entry.
                self.plan.validation_scope='EXECUTION_ENTRY_REQUALIFICATION_REQUIRED'
                self.plan_revision+=1
                self.metrics['plan_history'].append({'revision':self.plan_revision,'plan':asdict(self.plan)})
        self._save_executor()

    def _dispatch_executor_chain(self,item,reserved=False,retain_booking=False):
        """Run the selected steps under one physical reservation and completion.

        Step views are only the existing ROS encoding boundary. They are never
        added to the plan or allowed to release the parent activity's members.
        """
        from mrta_python.repair import process_executor_completion
        if item.wait_time:
            raise RuntimeError('qualified waiting must execute before motion')
        has_products=any((s.native_action and s.native_action.observation_ids) or s.observation_ids
                         for s in item.execution_steps)
        if item.task_id in self.observation_tasks and not has_products:
            raise RuntimeError('composite motion cannot substitute for received observation products')
        units=[]
        for step in item.execution_steps:
            operations=tuple(s.operation for s in step.native_action.segments) if step.native_action else ('AIR_MOVE',)
            unit=unit_for_coalition(self.routing,item.coalition,executor_id=step.executor_id,operations=operations)
            if unit is None or set(unit.physical_agent_ids)!=set(item.coalition):
                raise RuntimeError('step changes activity physical members or lacks an endpoint')
            units.append(unit)
        with self.executor_mutex:
            if not reserved and conflicting_active_unit(self.routing,self.active_executor_ids,item.executor_id):
                raise RuntimeError('physical members already occupied')
            item.status='RUNNING'
            self.active_executor_ids.add(item.executor_id)
            action=dict(task_id=item.task_id,execution_id=item.execution_id,phase='DISPATCHING')
            self.metrics.setdefault('current_actions',{})[item.execution_id]=action
            if self.executor_serial:self.metrics['current_action']=action
        self._save_executor()
        rows=[];deferred=[]
        for index,(step,unit) in enumerate(zip(item.execution_steps,units)):
            view=replace(item,execution_id='{}:step:{}'.format(item.execution_id,index),
                         executor_id=step.executor_id,travel_time=step.duration_s-step.service_time_s,
                         service_time=step.service_time_s,execution_steps=(step,))
            self._wait_executor_ready(unit)
            goal=self._executor_goal(view,unit)
            with self.executor_mutex:
                action.update(endpoint=unit.action_endpoint,step=index,step_count=len(units),
                              step_execution_id=view.execution_id,phase='DISPATCHING')
            continuing=index+1<len(units)
            if continuing:
                state,result=self._send_executor_goal(view,unit,goal,action,
                    defer_terminal_receipt=True)
            else:
                state,result=self._send_executor_goal(view,unit,goal,action)
            if step.native_action is not None and not continuing:
                self._wait_native_observation_receipt(state,result,step.native_action)
            if (step.native_action is None and not continuing and
                    getattr(goal,'observation_ids',()) and
                    self._release_result_ok(state,result,view.execution_id)):
                node=self.server_nodes[unit.executor_id]
                self._wait_observation_receipt(result.goal_id,goal.observation_ids,
                    float(rospy.get_param(node+'/execution_timeout',180.)),
                    require_products=self.request.template_id!='OFFSHORE_JOINT')
            missing=(step.native_action is not None and state==GoalStatus.ABORTED and result is not None and
                result.task_id==view.execution_id and bool(result.goal_id) and
                result.reason=='OBSERVATION_NOT_SATISFIED' and result.terminal_verified and
                not result.resource_locked and result.actual_mode==step.native_action.final_mode and
                bool(step.native_action.observation_ids))
            if missing and not continuing:
                report=self.metrics.get('received_terminal_reports',{}).get(result.goal_id)
                if (report is None or report['point_ids']!=sorted(step.native_action.observation_ids) or
                        set(report['observed_ids'])==set(step.native_action.observation_ids)):
                    raise RuntimeError('matching negative observation report absent; parent remains occupied')
            ok=(missing or self._native_motion_result_ok(state,result,view.execution_id,step.native_action)
                if step.native_action else self._release_result_ok(state,result,view.execution_id))
            row=dict(activity_id=item.execution_id,execution_id=view.execution_id,endpoint=unit.action_endpoint,
                     native_state=state,goal_id=getattr(result,'goal_id',None),verified=False,
                     observation_missing=bool(missing))
            with self.executor_mutex:self.metrics.setdefault('step_results',[]).append(row)
            if not ok:
                self._save_executor()
                raise RuntimeError('step {} failed or unverified; parent remains occupied'.format(index))
            goal_id,envelope=self.native_result(view.execution_id)
            if goal_id!=result.goal_id or envelope.status.status!=(GoalStatus.ABORTED if missing else GoalStatus.SUCCEEDED):
                raise RuntimeError('step native GoalID/Result envelope mismatch')
            if not step.native_action:
                path=self.output/result.evidence_file
                if path.parent!=self.output:
                    raise RuntimeError('invalid step evidence path')
                evidence=json.loads(path.read_text())
                if (evidence.get('goal_id')!=goal_id or not evidence.get('accepted_for_dispatch') or
                        not evidence.get('resource_released')):
                    raise RuntimeError('AIR step has no verified terminal evidence')
                row['evidence_file']=result.evidence_file
                if step.observation_ids and not continuing:
                    self._receive_observations(view,evidence)
                    if getattr(self,'finite_delivery',False):
                        report=self.metrics['received_terminal_reports'][result.goal_id]
                        row['observation_missing']=set(report['observed_ids'])!=set(step.observation_ids)
            row.update(verified=True,result_received_at=rospy.Time.now().to_sec())
            rows.append(row)
            if continuing:
                deferred.append((view,unit,step,state,result,evidence if not step.native_action else None))
            self._save_executor()
        # A local successful step may lead into an already selected return
        # while its notice/product is still travelling. Keep one reservation
        # for the whole chain; receive and verify every deferred result before
        # committing the parent activity or freeing its member.
        for view,unit,step,state,result,evidence in deferred:
            if getattr(self,'command_delivery_required',False):
                node=self.server_nodes[unit.executor_id]
                timeout=(step.native_action.execution_timeout_s if step.native_action else
                    float(rospy.get_param(node+'/execution_timeout',180.)))
                self._wait_action_terminal_receipt(result.goal_id,unit.physical_agent_ids,state,timeout)
            if step.native_action is not None:
                self._wait_native_observation_receipt(state,result,step.native_action)
                if state==GoalStatus.ABORTED and result.reason=='OBSERVATION_NOT_SATISFIED':
                    report=self.metrics.get('received_terminal_reports',{}).get(result.goal_id)
                    if (report is None or report['point_ids']!=sorted(step.native_action.observation_ids) or
                            set(report['observed_ids'])==set(step.native_action.observation_ids)):
                        raise RuntimeError('matching negative observation report absent; parent remains occupied')
            elif step.observation_ids:
                node=self.server_nodes[unit.executor_id]
                self._wait_observation_receipt(result.goal_id,step.observation_ids,
                    float(rospy.get_param(node+'/execution_timeout',180.)),
                    require_products=self.request.template_id!='OFFSHORE_JOINT')
                self._receive_observations(view,evidence)
                if getattr(self,'finite_delivery',False):
                    report=self.metrics['received_terminal_reports'][result.goal_id]
                    next(row for row in rows if row['execution_id']==view.execution_id)['observation_missing']=(
                        set(report['observed_ids'])!=set(step.observation_ids))
        with self.executor_mutex:
            if (has_products and self.request.delivery_required and
                    self.request.template_id!='OFFSHORE_JOINT'):
                received_products=self.metrics.get('received_products',{}).values()
                for step,row in zip(item.execution_steps,rows):
                    if row['observation_missing']:continue
                    expected=(step.native_action.observation_ids if step.native_action else step.observation_ids)
                    if not expected:continue
                    received_ids={event['point_id'] for event in received_products
                                  if event.get('goal_id')==row['goal_id'] and event.get('observed') is True}
                    if not set(expected)<=received_ids:
                        raise RuntimeError('composite terminal reached without required received products; keep member locked')
            received=rospy.Time.now().to_sec()
            event=DelayEvent(rows[-1]['goal_id'],item.execution_id,item.task_id,item.planned_finish,received-self.epoch)
            self.plan,changed=process_executor_completion(self.plan,event,self.final_events)
            if changed and (self.plan.activity_edges or
                            len({i.task_id for i in self.plan.items})!=len(self.plan.items)):
                self.plan.validation_scope='EXECUTION_ENTRY_REQUALIFICATION_REQUIRED'
                self.plan_revision+=1
                self.metrics['plan_history'].append({'revision':self.plan_revision,'plan':asdict(self.plan)})
            self.metrics['executions'].append(dict(task_id=item.task_id,execution_id=item.execution_id,
                result='OBSERVATION_MISSING' if any(row['observation_missing'] for row in rows) else 'SUCCEEDED',
                scope='COMPOSITE_MOTION',plan_updated=changed,result_received_at=received,
                step_goal_ids=[r['goal_id'] for r in rows]))
            if all(i.status=='COMPLETED' for i in self.plan.items if i.task_id==item.task_id):
                self.metrics['results_received'].append(item.task_id)
            self.metrics['native_qualification_only']=True
            if not retain_booking:self.active_executor_ids.remove(item.executor_id)
            self.metrics['current_actions'].pop(item.execution_id,None)
            if self.executor_serial:self.metrics['current_action']=None
            # Completion propagation preserves remaining commitments. Full
            # method re-evaluation, not scalar centre distance, is needed next.
        self._save_executor()

    def _commit_executor_result(self,item,unit,state,result,retain_booking=False):
        from mrta_python.repair import process_executor_completion
        native=getattr(item,'native_action',None)
        if native is not None:
            return self._commit_native_executor_result(item,unit,state,result,native,retain_booking)
        if not self._release_result_ok(state, result, item.execution_id):
            detail = "missing native Result"
            if result is not None:
                detail = "reason={} task={} safety={} validity={}".format(
                    result.reason, result.task_outcome, result.safety_outcome, result.experiment_validity)
                evidence_path = self.output / result.evidence_file
                if evidence_path.parent == self.output and evidence_path.is_file():
                    failed_evidence = json.loads(evidence_path.read_text())
                    if failed_evidence.get("goal_id") == result.goal_id:
                        detail += "; " + failed_evidence.get("reason_text", "")
                        if failed_evidence.get("safety_hold"):
                            self.metrics["safety_disposition"] = failed_evidence["safety_hold"]
                self.metrics["executions"].append({"task_id": item.task_id,
                    "execution_id": item.execution_id, "endpoint": unit.action_endpoint,
                    "goal_id": result.goal_id, "result_received_at": rospy.Time.now().to_sec(),
                    "evidence_file": result.evidence_file, "result": "NOT_ACCEPTED",
                    "native_state": state, "native_execution_id": result.task_id,
                    "plan_updated": False, "detail": detail})
                self.metrics["results_received"].append(item.task_id)
            raise RuntimeError("native Result cannot release members; endpoint={} state={} {}".format(
                unit.action_endpoint, state, detail))
        goal_id, native = self.native_result(item.execution_id)
        if goal_id != result.goal_id or native.status.status != GoalStatus.SUCCEEDED:
            raise RuntimeError("native GoalID/Result envelope mismatch")
        evidence = json.loads((self.output / result.evidence_file).read_text())
        if (evidence.get("goal_id") != goal_id or not evidence.get("accepted_for_dispatch")
                or not evidence.get("resource_released")):
            raise RuntimeError("server evidence does not authorize physical release")
        # A received local report and its positive products must agree with
        # the actual qn samples before this Result commits the PlanItem.
        self._receive_observations(item, evidence)
        received = rospy.Time.now().to_sec()
        event = DelayEvent(goal_id, item.execution_id, item.task_id, item.planned_finish,
                           max(result.actual_finish_time.to_sec(), received) - self.epoch)
        self.plan, changed = process_executor_completion(self.plan, event, self.final_events)
        self.metrics["executions"].append({"task_id": item.task_id, "execution_id": item.execution_id,
            "endpoint": unit.action_endpoint, "goal_id": goal_id, "result_received_at": received,
            "evidence_file": result.evidence_file, "result": "SUCCEEDED", "plan_updated": changed,
            "formation_geometry": evidence.get("formation_geometry")})
        if all(i.status=='COMPLETED' for i in self.plan.items if i.task_id==item.task_id):
            self.metrics["results_received"].append(item.task_id)
        if (getattr(self,'finite_delivery',False) and len(item.execution_steps)==1 and
                item.execution_steps[0].observation_ids):
            report=self.metrics['received_terminal_reports'][goal_id]
            self.metrics['executions'][-1]['observation_missing']=(
                set(report['observed_ids'])!=set(item.execution_steps[0].observation_ids))
        # No finally-discard: every exceptional/unknown exit keeps the reservation.
        self._refresh_executor_timing(received - self.epoch)
        if not retain_booking:self.active_executor_ids.remove(unit.executor_id)
        self.metrics.setdefault("current_actions", {}).pop(item.execution_id,None)
        if getattr(self,"executor_serial",True):self.metrics["current_action"] = None

    def _commit_native_executor_result(self,item,unit,state,result,native,retain_booking=False):
        """Commit a motion Result; never synthesize observation or delivery."""
        from mrta_python.repair import process_executor_completion
        payload=None if result is None else {key:getattr(result,key) for key in (
            'task_id','goal_id','task_completed','terminal_verified','actual_mode','reason','resource_locked','model_time_s')}
        previous=next((r for r in self.metrics['executions'] if r.get('execution_id')==item.execution_id
                       and r.get('result') in ('SUCCEEDED','OBSERVATION_MISSING')),None)
        if previous is not None:
            expected=GoalStatus.ABORTED if previous['result']=='OBSERVATION_MISSING' else GoalStatus.SUCCEEDED
            if previous.get('native_result')!=payload or state!=expected:
                raise RuntimeError('conflicting repeated native Result')
            return  # May now be booked by another task; never remove its lock.
        missing=(state==GoalStatus.ABORTED and result is not None and
                 result.task_id==item.execution_id and bool(result.goal_id) and
                 result.reason=='OBSERVATION_NOT_SATISFIED' and result.terminal_verified and
                 not result.resource_locked and result.actual_mode==native.final_mode and
                 bool(native.observation_ids))
        if not missing and not self._native_motion_result_ok(state,result,item.execution_id,native):
            detail='missing Result' if result is None else result.reason
            self.metrics['executions'].append(dict(task_id=item.task_id,execution_id=item.execution_id,
                endpoint=unit.action_endpoint,result='NON_SUCCESS',native_state=state,native_result=payload,detail=detail))
            raise RuntimeError('native motion Result cannot release members: '+detail)
        goal_id,envelope=self.native_result(item.execution_id)
        if goal_id!=result.goal_id or envelope.status.status!=(GoalStatus.ABORTED if missing else GoalStatus.SUCCEEDED):
            raise RuntimeError('native GoalID/Result envelope mismatch')
        if missing:
            report=self.metrics.get('received_terminal_reports',{}).get(result.goal_id)
            if (report is None or report['point_ids']!=sorted(native.observation_ids) or
                    set(report['observed_ids'])==set(native.observation_ids)):
                raise RuntimeError('negative observation report missing or inconsistent; keep member reserved')
        if item.fulfills_task and item.task_id in self.observation_tasks and not native.observation_ids:
            raise RuntimeError('native motion Result does not contain validated observation products')
        if (native.observation_ids and self.request.delivery_required and not missing and
                self.request.template_id!='OFFSHORE_JOINT'):
            delivered={event['point_id'] for event in self.metrics.get('received_products',{}).values()
                       if event.get('goal_id')==result.goal_id and event.get('observed') is True}
            if not set(native.observation_ids)<=delivered:
                raise RuntimeError('native terminal reached without required received products; keep member locked')
        received=rospy.Time.now().to_sec()
        event=DelayEvent(goal_id,item.execution_id,item.task_id,item.planned_finish,received-self.epoch)
        self.plan,changed=process_executor_completion(self.plan,event,self.final_events)
        self.metrics['executions'].append(dict(task_id=item.task_id,execution_id=item.execution_id,
            endpoint=unit.action_endpoint,goal_id=goal_id,result_received_at=received,
            result='OBSERVATION_MISSING' if missing else 'SUCCEEDED',plan_updated=changed,
            scope='NATIVE_MOTION',safety_outcome='NOT_VERIFIED',native_result=payload))
        if all(i.status=='COMPLETED' for i in self.plan.items if i.task_id==item.task_id):
            self.metrics['results_received'].append(item.task_id)
        self.metrics['native_qualification_only']=True
        if not retain_booking:
            self._refresh_executor_timing(received-self.epoch)
        if not retain_booking:self.active_executor_ids.remove(unit.executor_id)
        self.metrics.setdefault('current_actions',{}).pop(item.execution_id,None)
        if getattr(self,'executor_serial',True):self.metrics['current_action']=None

    def _execute_parallel_pending(self):
        """One scheduler owns bookings; workers wait independently on endpoints."""
        from mrta_python.executors import activity_predecessors
        running={}
        failure=None
        with ThreadPoolExecutor(max_workers=len(self.units)) as workers:
            while not rospy.is_shutdown():
                if getattr(self,'opaque_hold',None) is not None:
                    try:self._check_opaque_hold()
                    except RuntimeError as error:
                        failure=error
                        for unit_id in tuple(self.active_executor_ids):
                            client=self.clients.get(unit_id)
                            if client is not None:client.cancel_goal()
                dispatch=[]
                for execution_id,(future,activity_ids) in list(running.items()):
                    if future.done():
                        try:future.result()
                        except Exception as error:
                            failure=error
                            with self.executor_mutex:
                                for ident in activity_ids:
                                    if self.plan.item(ident).status=='COMPLETED':continue
                                    self.plan.item(ident).status="UNKNOWN_LOCKED"
                                    action=self.metrics.get("current_actions",{}).get(ident)
                                    if action is not None:action.update(phase="UNKNOWN_LOCKED",failure_reason=str(error))
                        del running[execution_id]
                with self.executor_mutex:
                    completed={i.execution_id for i in self.plan.items if i.status=="COMPLETED"}
                    predecessors=activity_predecessors(self.plan)
                    pending=[i for i in self.plan.items if i.status=="PLANNED"]
                    now=rospy.Time.now().to_sec()-self.epoch
                    if failure is None:
                        for item in pending:
                            if item.status!='PLANNED':continue
                            group=[i for i in pending if i.status=='PLANNED' and i.task_id==item.task_id and
                                   item.candidate_id and i.candidate_id==item.candidate_id] or [item]
                            group_ids={i.execution_id for i in group}
                            if (min(i.planned_start for i in group)>now or
                                any(not (predecessors[i.execution_id]-group_ids)<=completed or
                                    conflicting_active_unit(self.routing,self.active_executor_ids,i.executor_id) for i in group)):
                                continue
                            # One lock covers all physical participants, before
                            # any goal or preparation request can be sent.
                            for activity in group:
                                activity.status="RUNNING"
                                self.active_executor_ids.add(activity.executor_id)
                            dispatch.append(tuple(group))
                    finished=not running and not dispatch and (failure is not None or not pending)
                if dispatch:
                    self._save_executor()
                    for group in dispatch:
                        future=(workers.submit(self._dispatch_cooperative_items,group) if len(group)>1 else
                                workers.submit(self._dispatch_executor_item,group[0],True))
                        running[group[0].execution_id]=(future,tuple(i.execution_id for i in group))
                if finished:break
                time.sleep(.05)
            if failure is not None:raise failure
            if rospy.is_shutdown():raise RuntimeError("runner shutdown with accepted commitments")

    def _check_opaque_hold(self):
        """Monitor a previously accepted AIR return while its model state is unknown."""
        hold=self.opaque_hold
        if hold.get('violation'):raise RuntimeError(hold['violation'])
        if time.monotonic()-hold['started_monotonic']>hold['horizon_s']:
            raise RuntimeError('retained AIR hold exceeded its experimentally qualified horizon')
        now=rospy.Time.now().to_sec()
        with self.condition:
            sample=self.actual.get(hold['member'])
            stamp,values=self.executor_diagnostics.get(hold['member'],(None,{}))
        if sample is None or not sample.is_fresh(now,.25) or stamp is None or not 0<=now-stamp<=.25:
            raise RuntimeError('retained AIR member state or reference is stale')
        if (math.dist(sample.position,hold['position'])>hold['radius_m'] or
                values.get('reference_source')!='AIR_SWARM' or
                values.get('domain_failure')=='true' or values.get('platform_resource_locked')=='true' or
                values.get('air_domain_violation')=='true'):
            raise RuntimeError('retained AIR member left its safe hold/reference contract')
        reference=tuple(float(values['reference_position_'+axis]) for axis in 'xyz')
        if math.dist(reference,hold['position'])>hold['radius_m']:
            raise RuntimeError('retained AIR member adopted a different reference')

    def _receive_observations(self, item, evidence):
        from qn_aav_simulator.observation_coverage import (ObservationSample,PointObservation,
            evaluate_coverage,record_delivery)
        task = self.observation_tasks.get(item.task_id)
        if task is None:
            return
        point_ids=(tuple(item.execution_steps[0].observation_ids)
                   if len(item.execution_steps)==1 and item.execution_steps[0].observation_ids
                   else task.covers)
        # Only this execution's actual successful holding interval is eligible;
        # no old trajectory, inferred target position or previous-task observation.
        hold = evidence.get("successful_hold_window") or {}
        if hold.get("start") is None or hold.get("end") is None:
            raise RuntimeError("successful holding interval missing")
        samples = []
        for member, rows in evidence.get("member_samples", {}).items():
            previous = None
            for row in rows:
                stamp = row["state_stamp_s"]
                if stamp is None or not hold["start"] <= stamp <= hold["end"]:
                    continue
                if stamp == previous:
                    continue
                previous = stamp
                samples.append(ObservationSample("drone_" + member, stamp, tuple(row["position"])))
        coverage = evaluate_coverage(samples, {p: self.points[p] for p in point_ids},
            self.request.requirement, self.obstacles, sample_timeout_s=evidence["sample_timeout_s"])
        if getattr(self,'finite_delivery',False):
            with self.executor_mutex:
                report=self.metrics.get('received_terminal_reports',{}).get(evidence['goal_id'])
                received={event['point_id'] for event in self.metrics.get('received_products',{}).values()
                          if event.get('goal_id')==evidence['goal_id'] and event.get('observed') is True}
            if report is None or set(report['point_ids'])!=set(point_ids):
                raise RuntimeError('AIR local terminal report missing or names other points; keep members reserved')
            observed=set(report['observed_ids'])
            if ((observed!=received if self.request.template_id!='OFFSHORE_JOINT'
                    else not received<=observed) or
                    not observed<={p for p,row in coverage.points.items() if row.observed}):
                raise RuntimeError('AIR local observation or actual receipt conflicts with qn evidence; keep members reserved')
        for point_id, observation in coverage.points.items():
            if getattr(self,'finite_delivery',False) and point_id not in observed:
                observation=PointObservation(point_id,False,None,0.,
                    'received local terminal report says this point was not observed')
            if observation.observed or point_id not in self.coverage.points:
                self.coverage.points[point_id] = observation
        # The local Result carries access to the observation evidence. Receipt is
        # explicit and zero-latency in this simulator; no radio link is claimed.
        if not getattr(self,'finite_delivery',False):record_delivery(self.coverage, task.covers)

    def _run_joint_request(self):
        """Run the fixed five-platform request through the existing task worker.

        Initial model copies are declared qualification states. They are not a
        reconstruction of later controller/actuator state from Odometry, so
        this entry cannot claim arbitrary online re-planning qualification.
        """
        from mrta_python.executors import Executor, ExecutorTravelTimeProvider
        from qn_aav_simulator.contracts import AgentState
        from qn_aav_simulator.experiment_verdict import StaticSceneGeometry
        from qn_aav_simulator.monitoring_request import ObservationTask
        from qn_aav_simulator.observation_coverage import ObstacleBox
        from qn_aav_simulator.pvs_backend import PvsBackend, NATIVE_START_TOLERANCE_M
        from qn_aav_simulator.platform_execution import actual_mode
        from qn_aav_simulator.task_line import build_request_executor_plan, retest_tasks
        timer=None
        self.points={point.point_id:point.position for region in self.request.regions
                     for point in region.interest_points}
        self.weights={point.point_id:point.weight for region in self.request.regions
                      for point in region.interest_points}
        try:
            if os.environ.get('QN_SAME_SOURCE_ACCELERATION')=='true':
                from mrta_python.query_worker import _enable_query_extensions
                if not _enable_query_extensions():
                    raise RuntimeError('planner qn source/ABI differs from the running compiled model')
            from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
            if self.executor_serial or not self.finite_delivery:
                raise RuntimeError('joint request requires parallel executor and actual finite receipt inputs')
            if set(self.fleet)!={'drone_0','drone_1','drone_2','usv','uuv'}:
                raise RuntimeError('current joint qualification requires exactly the declared five physical members')
            for unit in self.units:self._wait_executor_ready(unit)
            scene=rospy.get_param('/scene',{})
            geometry=StaticSceneGeometry.from_mapping(scene)
            if geometry is None or not scene.get('return_sites'):
                raise RuntimeError('joint request needs the declared obstacle scene and return sites')
            positions={};models={}
            for member in ('drone_0','drone_1','drone_2'):
                node='/'+member+'_qn_aav'
                position=tuple(float(rospy.get_param(node+'/init_'+axis)) for axis in 'xyz')
                backend=QnPythonClosedLoopBackend(dict(
                    initialization_mode='STATIC_TRIM',model_step_s=.001,
                    reference_mode='ROUTE_POSITION',
                    water_guidance_mode=rospy.get_param(node+'/water_guidance_mode','QN_ORIGINAL_POSITION'),
                    water_horizontal_controller_mode=rospy.get_param(
                        node+'/water_horizontal_controller_mode','QN_ORIGINAL_RBF_PD')))
                backend.reset(AgentState(member,'AAV',0.,position,(0.,0.,0.)))
                positions[member]=backend.snapshot().position;models[member]=backend
            efforts={}
            for member,model_name in (('usv','otter'),):
                node='/'+member
                if rospy.get_param(node+'/model')!=model_name:
                    raise RuntimeError('native marine model differs from declared executor: '+member)
                position=tuple(float(v) for v in rospy.get_param(node+'/initial_position'))
                backend=PvsBackend(model_name,position,
                    heading_rad=float(rospy.get_param(node+'/initial_heading_rad',0.)),
                    initialization_mode=rospy.get_param(node+'/initialization_mode','NATIVE_ZERO'))
                positions[member]=backend.snapshot()['position'];models[member]=backend
                efforts[member]=float(rospy.get_param(node+'/propulsion_effort'))
            uuv_position=tuple(float(rospy.get_param('/uuv/init_'+axis)) for axis in 'xyz')
            uuv_model=QnPythonClosedLoopBackend(dict(
                initialization_mode='STATIC_TRIM',model_step_s=.001,
                reference_mode='ROUTE_POSITION',
                water_guidance_mode=rospy.get_param('/uuv/water_guidance_mode'),
                water_horizontal_controller_mode=rospy.get_param('/uuv/water_horizontal_controller_mode')))
            uuv_model.reset(AgentState('uuv','UUV',0.,uuv_position,(0.,0.,0.)))
            if actual_mode(uuv_model.snapshot().medium_flag)!='WATER':
                raise RuntimeError('dedicated qn UUV did not initialize in WATER')
            positions['uuv']=uuv_model.snapshot().position;models['uuv']=uuv_model

            def checked_start():
                deadline=time.monotonic()+10.
                while True:
                    try:actual=self._actual_positions();break
                    except RuntimeError:
                        if time.monotonic()>=deadline:raise
                        time.sleep(.05)
                deviations={member:math.dist(actual[member],position)
                            for member,position in positions.items()}
                if any(distance>NATIVE_START_TOLERANCE_M for distance in deviations.values()):
                    raise RuntimeError('declared initial model position differs from actual: '+str(deviations))
                return deviations

            self.metrics['initial_position_deviation_m']=checked_start()
            states={member:dict(position=position,mode=('AIR' if member.startswith('drone_')
                         else 'WATER' if member=='uuv' else models[member].snapshot()['actual_mode']),available_from=0.)
                    for member,position in positions.items()}
            units=[Executor(unit.executor_id,unit.physical_agent_ids,frozenset(unit.capabilities))
                   for unit in self.units]
            provider=ExecutorTravelTimeProvider({'start':positions['drone_0']},
                {unit.executor_id:1. for unit in units},native_models=models,native_efforts=efforts)
            self.metrics['status']='PLANNING_DIAGNOSTIC'
            self.metrics['planning_budget_s']=float(rospy.get_param('~planning_budget_s',10.))
            began=time.monotonic()
            self.metrics['planning_started_monotonic']=began
            self.metrics['qualification_scope']='DECLARED_INITIAL_MODEL_AND_ACTUAL_POSITION_CHECK'
            self._save_executor()
            try:
                self.plan,tasks=build_request_executor_plan(self.request,scene,units,provider,states,
                    budget_s=self.metrics['planning_budget_s'],
                    first_feasible=self.request.template_id!='OFFSHORE_JOINT')
            finally:
                self.metrics['planning_wall_s']=time.monotonic()-began
            # Retain only the terminal backend of the method actually selected.
            # A later repair may reuse it only when the matching finite Action
            # notice confirms that the local terminal state was reached.
            selected_terminals=dict(getattr(self.plan,'_selected_native_terminals',{}))
            # Prepare read-only idle witnesses while the first plan is still
            # awaiting confirmation. Exact qn fixed points and the UUV's
            # declared no-command continuation otherwise consume the later
            # 10 s feedback-repair budget for identical computations.
            standby_fixed={};idle_anchors={}
            for member in ('drone_0','drone_1','drone_2'):
                idle=models[member].predict_idle(5.,geometry,time.monotonic()+10.)
                if idle['status']=='FEASIBLE' and idle['reason']=='QN_EXACT_INITIAL_HOLD_FIXED_POINT':
                    standby_fixed[member]=idle['terminal_backend']
            with self.condition:
                _,uuv_diagnostic=self.executor_diagnostics.get('uuv',(None,{}))
            uuv_time=float(uuv_diagnostic.get('model_time_s','nan'))
            if math.isfinite(uuv_time) and uuv_time>=0.:
                idle=models['uuv'].predict_idle(uuv_time,
                    geometry,time.monotonic()+10.)
                if idle['status']=='FEASIBLE':idle_anchors['uuv']=idle['terminal_backend']
            for key,backend in selected_terminals.items():
                if getattr(backend,'model',None)!='otter':continue
                idle=backend.predict_idle(.1,geometry,time.monotonic()+2.)
                if idle['status']=='FEASIBLE' and idle['terminal_backend'].execution_state_bytes()==backend.execution_state_bytes():
                    standby_fixed[key]=backend
            self.tasks_by_id={task.task_id:task for task in tasks}
            regions={region.region_id:region for region in self.request.regions}
            def target(task):
                point=regions[task.target_ref].interest_points[0].position
                return ((point[0],point[1],self.request.requirement.cruise_altitude_m)
                        if regions[task.target_ref].kind in ('SURFACE','SHORELINE') else point)
            self.observation_tasks={task.task_id:ObservationTask(task.task_id,target(task),
                tuple(point.point_id for point in regions[task.target_ref].interest_points),
                task.target_ref,self.request.service_time_s,self.request.deadline_s,
                task.required_capabilities) for task in tasks}
            self.centers={task.target_ref:target(task) for task in tasks}
            self.centers.update({'transition:'+site['id']:(site['position'][0],site['position'][1],
                self.request.requirement.cruise_altitude_m) for site in scene['transition_sites']})
            self.centers.update({'transition-stage:'+site['id']+':'+str(index):tuple(point)
                for site in scene['transition_sites'] for index,point in enumerate(site.get('air_stages',()))})
            self.centers.update({'return:'+member:tuple(site['position'])
                for member,site in scene['return_sites'].items()})
            self.obstacles=[ObstacleBox(center,size) for _,kind,center,size in geometry.objects
                            if kind=='SOLID']
            conditional_retest=None
            if len(tasks)==1 and regions[tasks[0].target_ref].kind in ('SURFACE','SHORELINE'):
                selected_work=[item for item in self.plan.items if item.fulfills_task and
                    item.task_id==tasks[0].task_id and len(item.coalition)==1 and
                    item.coalition[0].startswith('drone_')]
                possible=retest_tasks(self.request,(),self.coverage,self.weights,
                    delivery_recorded=True,already_retested=False)
                if len(selected_work)==len(possible)==1:
                    held=selected_work[0].coalition[0]
                    future_task=Task(possible[0].task_id,possible[0].required_capabilities,1,
                        possible[0].service_time_s,possible[0].deadline_s,possible[0].region_id)
                    future_models={m:standby_fixed[m] for m in ('drone_0','drone_1','drone_2')
                                   if m!=held and m in standby_fixed}
                    support_item=next((item for item in self.plan.items if 'usv' in item.coalition),None)
                    if support_item is not None and (support_item.execution_id,'usv') in standby_fixed:
                        support_model=selected_terminals.get((support_item.execution_id,'usv'))
                        if support_model is not None:future_models['usv']=support_model
                    if 'uuv' in idle_anchors:future_models['uuv']=idle_anchors['uuv']
                    if len(future_models)==4 and scene['return_sites'][held]['radius_m']==.5:
                        future_states={m:dict(position=(model.snapshot().position if m.startswith('drone_')
                            else model.snapshot()['position']),mode=('AIR' if m.startswith('drone_')
                            else model.snapshot()['actual_mode']),available_from=0.)
                            for m,model in future_models.items()}
                        future_states[held]=dict(position=tuple(scene['return_sites'][held]['position']),
                            mode='AIR',available_from=0.,locked=True,collision_radius_m=.25,
                            opaque_hold_radius_m=.5,opaque_hold_horizon_s=_QUALIFIED_AIR_RETEST_HOLD_S)
                        future_units=[unit for unit in units if held not in unit.physical_agent_ids]
                        future_provider=ExecutorTravelTimeProvider({'start':next(iter(future_states.values()))['position']},
                            {unit.executor_id:1. for unit in future_units},native_models=future_models,
                            native_efforts=efforts)
                        began_future=time.monotonic()
                        try:
                            future_plan,_=build_request_executor_plan(self.request,scene,future_units,
                                future_provider,future_states,budget_s=15.,
                                tasks_override=(future_task,),first_feasible=True)
                        except (ValueError,RuntimeError) as error:
                            self.metrics['prepared_retest_status']='UNKNOWN: '+str(error)
                        else:
                            conditional_retest=(future_task,future_plan,held)
                            self.metrics['prepared_retest_status']='CONDITIONAL_COMPLETE_CANDIDATE'
                        finally:
                            self.metrics['prepared_retest_wall_s']=time.monotonic()-began_future
            self.metrics['selected_plan']=asdict(self.plan)
            self.metrics['predicted_receipts']=dict(getattr(self.plan,'_predicted_receipts',{}))
            self.metrics['predicted_receipt_limits']=dict(getattr(self.plan,'_receipt_limits',{}))
            save_json(self.output/'nominal-plan.json',asdict(self.plan))
            self.metrics['status']='AWAITING_CONFIRMATION';self._save_executor()
            endpoints={unit.executor_id:unit.action_endpoint for unit in self.units}
            preview=[dict(task_id=item.task_id,executor=item.executor_id,
                members=item.coalition,endpoint=endpoints[item.executor_id],
                start_s=item.planned_start,finish_s=item.planned_finish,
                method=[step.target_ref for step in item.execution_steps]) for item in self.plan.items]
            print(json.dumps(dict(request_id=self.request.request_id,activities=preview,
                limits=['geometric observation proxy; payload quality unverified',
                        ('task-level shared support; actual receipt required'
                         if self.request.template_id=='OFFSHORE_JOINT' else
                         'finite experimental delivery; actual receipt required'),
                        'return required; native endpoint decides actual terminal',
                        'complete model/trajectory in nominal-plan.json']),
                indent=2,default=json_default),flush=True)
            try:answer=input('Confirm this exact joint plan? Type yes to dispatch: ')
            except (EOFError,KeyboardInterrupt):answer=''
            if answer.strip().lower()!='yes':
                self.metrics['status']='NOT_CONFIRMED';self._save_executor();return
            self.metrics['confirmation']={'answer':'yes','at_ros_s':rospy.Time.now().to_sec()}
            self.metrics['initial_position_deviation_m']=checked_start()
            self.epoch=rospy.Time.now().to_sec()
            self.metrics['status']='RUNNING_DIAGNOSTIC';self._save_executor()
            timer=rospy.Timer(rospy.Duration(1.),lambda _:self._save_executor())
            self._execute_parallel_pending()
            delivered=self.coverage.delivered_fraction(self.weights)
            missing=retest_tasks(self.request,(),self.coverage,self.weights,
                delivery_recorded=set(self.metrics['results_received'])=={task.task_id for task in tasks},
                already_retested=False)
            if missing:
                self.metrics['pending_retest']=[task.task_id for task in missing]
                if len(missing)!=1 or regions[missing[0].region_id].kind not in ('SURFACE','SHORELINE'):
                    raise RuntimeError('no qualified one-round repair for the received missing-observation report')
                failed_rows=[row for row in self.metrics['executions']
                    if row.get('result')=='OBSERVATION_MISSING' and row.get('task_id') in self.tasks_by_id]
                failed_items=[item for item in self.plan.items if item.fulfills_task and
                    any(row['execution_id']==item.execution_id for row in failed_rows)]
                if (len(failed_items)!=1 or len(failed_items[0].coalition)!=1 or
                        not failed_items[0].coalition[0].startswith('drone_') or
                        any(item.status!='COMPLETED' for item in self.plan.items) or self.active_executor_ids):
                    raise RuntimeError('missing report has no safely retained single-AAV repair boundary')
                failed_member=failed_items[0].coalition[0]
                completed_row=next(row for row in failed_rows if row['execution_id']==failed_items[0].execution_id)
                hold_site=scene['return_sites'][failed_member]
                hold_position=tuple(hold_site['position'])
                if hold_site['radius_m']!=.5:
                    raise RuntimeError('AIR hold qualification belongs to the existing 0.5 m return contract')
                self.opaque_hold=dict(member=failed_member,position=hold_position,radius_m=.5,
                    horizon_s=_QUALIFIED_AIR_RETEST_HOLD_S,started_monotonic=time.monotonic()-max(0.,
                        rospy.Time.now().to_sec()-completed_row['result_received_at']),violation='')
                self._check_opaque_hold()
                repair_started=time.monotonic();repair_deadline=repair_started+10.
                claims=self._await_state_claims(self.fleet,repair_deadline)
                held_claim=claims[failed_member]
                if (held_claim['actual_mode']!='AIR' or
                        held_claim['reference_source']!='AIR_SWARM' or
                        math.dist(held_claim['position'],hold_position)>.5):
                    raise RuntimeError('finite held-AAV claim does not cover its return contract')
                repair_states={failed_member:dict(position=hold_position,mode='AIR',available_from=0.,
                    locked=True,collision_radius_m=.25,opaque_hold_radius_m=.5,
                    opaque_hold_horizon_s=max(.01,_QUALIFIED_AIR_RETEST_HOLD_S-
                        (repair_started-self.opaque_hold['started_monotonic'])))}
                repair_models={}
                for member in self.fleet:
                    if member==failed_member:continue
                    claim=claims[member]
                    with self.condition:
                        stamp,diagnostic=self.executor_diagnostics.get(member,(None,{}))
                        sample=self.actual.get(member)
                    now=rospy.Time.now().to_sec()
                    if (sample is None or not sample.is_fresh(now,.25) or stamp is None or
                            not 0<=now-stamp<=.25 or diagnostic.get('resource_locked')=='true' or
                            diagnostic.get('platform_resource_locked')=='true' or
                            diagnostic.get('domain_failure')=='true' or diagnostic.get('scene_failure')):
                        raise RuntimeError('repair member state unavailable or locked: '+member)
                    if member.startswith('drone_'):
                        backend=standby_fixed.get(member)
                        if backend is None:
                            raise RuntimeError('standby AAV has no exact initial-hold model: '+member)
                        expected=hashlib.sha256(backend.execution_state_bytes()).hexdigest()
                        if (claim['actual_mode']!='AIR' or
                                claim['terminal_state_digest']!=expected or
                                claim['reference_source']!='INITIAL_HOLD'):
                            raise RuntimeError('finite standby AAV state differs: '+member)
                        mode='AIR'
                    else:
                        prior=next((item for item in self.plan.items if member in item.coalition),None)
                        if prior is None:
                            backend=idle_anchors.get(member,models[member])
                        else:
                            backend=selected_terminals.get((prior.execution_id,member))
                            row=next((row for row in self.metrics['executions']
                                if row.get('execution_id')==prior.execution_id),None)
                            notice=(self.metrics['received_action_results'].get(row['goal_id'],{}).get(member)
                                    if row and row.get('goal_id') else None)
                            if (backend is None or
                                    (notice is None or notice.get('nominal_terminal_state_match') is not True)
                                    and not (self.request.template_id=='OFFSHORE_JOINT' and
                                        row is not None and row.get('result')=='SUCCEEDED' and
                                        claim.get('terminal_state_digest')==hashlib.sha256(
                                            backend.execution_state_bytes()).hexdigest())):
                                raise RuntimeError('native support terminal state not confirmed: '+member)
                        target_time=float(claim['model_time_s'])
                        if not math.isfinite(target_time) or target_time+1e-6<backend.time_s:
                            raise RuntimeError('native idle model time unavailable: '+member)
                        if prior is None or (prior.execution_id,member) not in standby_fixed:
                            idle=backend.predict_idle(max(0.,target_time-backend.time_s),geometry,repair_deadline)
                            if idle['status']!='FEASIBLE':
                                raise RuntimeError('native idle state cannot be carried into repair: '+member)
                            backend=idle['terminal_backend']
                        if math.dist(claim['position'],backend.snapshot()['position'])>NATIVE_START_TOLERANCE_M:
                            raise RuntimeError('finite native claim differs from predicted full state: '+member)
                        age=max(0.,rospy.Time.now().to_sec()-claim['generated_at'])
                        if age>.001 and (prior is None or (prior.execution_id,member) not in standby_fixed):
                            idle=backend.predict_idle(age,geometry,repair_deadline)
                            if idle['status']!='FEASIBLE':
                                raise RuntimeError('native claim cannot be advanced to repair adoption: '+member)
                            backend=idle['terminal_backend']
                        mode=backend.snapshot()['actual_mode']
                    position=backend.snapshot().position if member.startswith('drone_') else backend.snapshot()['position']
                    if (math.dist(claim['position'],position)>NATIVE_START_TOLERANCE_M
                            if member.startswith('drone_') else
                            math.dist(sample.position,position)>NATIVE_START_TOLERANCE_M):
                        raise RuntimeError('finite repair claim/model or local safety position differs: '+member)
                    repair_models[member]=backend
                    repair_states[member]=dict(position=position,mode=mode,available_from=0.)
                self._check_opaque_hold()
                available=[unit for unit in units if failed_member not in unit.physical_agent_ids]
                repair_task=Task(missing[0].task_id,missing[0].required_capabilities,1,
                    missing[0].service_time_s,missing[0].deadline_s,missing[0].region_id)
                repair_provider=ExecutorTravelTimeProvider({'start':next(iter(repair_states.values()))['position']},
                    {unit.executor_id:1. for unit in available},native_models=repair_models,native_efforts=efforts)
                self.metrics['repair_prepare_wall_s']=time.monotonic()-repair_started
                search_started=time.monotonic()
                try:
                    repair_plan=None
                    if (conditional_retest is not None and
                            conditional_retest[0]==repair_task and conditional_retest[2]==failed_member):
                        from mrta_python.executors import bounded_travel_query
                        prepared=conditional_retest[1]
                        checked_states={member:dict(state,native_backend=repair_models[member])
                                        if member in repair_models else dict(state)
                                        for member,state in repair_states.items()}
                        check=bounded_travel_query(prepared._selected_provider._check_complete_plan,
                            (prepared,prepared._selected_evidence,checked_states,repair_deadline),repair_deadline)
                        self.metrics['prepared_retest_recheck']=check
                        if check['status']=='FEASIBLE':
                            repair_plan=prepared
                            self.metrics['repair_source']='PREPARED_CONDITIONAL_REVALIDATED'
                    if repair_plan is None:
                        repair_plan,_=build_request_executor_plan(self.request,scene,available,repair_provider,
                            repair_states,budget_s=max(0.,repair_deadline-time.monotonic()),
                            tasks_override=(repair_task,),first_feasible=True)
                        self.metrics['repair_source']='CURRENT_STATE_JOINT_SEARCH'
                finally:
                    self.metrics['repair_search_wall_s']=time.monotonic()-search_started
                    self.metrics['repair_wall_s']=time.monotonic()-repair_started
                self._check_opaque_hold()
                if (time.monotonic()>repair_deadline or repair_plan.makespan>
                        _QUALIFIED_AIR_RETEST_HOLD_S-
                        (time.monotonic()-self.opaque_hold['started_monotonic'])):
                    raise RuntimeError('complete repair exceeds its shared budget or retained hold horizon')
                offset=rospy.Time.now().to_sec()-self.epoch
                for item in repair_plan.items:
                    item.planned_start+=offset;item.planned_finish+=offset
                    if any(old.execution_id==item.execution_id for old in self.plan.items):
                        raise RuntimeError('repair Action execution ID collides with prior commitment')
                self.plan.items.extend(repair_plan.items)
                self.plan.precedence_edges+=((failed_items[0].task_id,repair_task.task_id),)
                self.plan.activity_edges+=repair_plan.activity_edges
                self.plan.validation_scope=repair_plan.validation_scope
                self.plan_revision+=1
                self.metrics['repair_plan']=asdict(repair_plan)
                self.metrics['plan_history'].append({'revision':self.plan_revision,'plan':asdict(self.plan)})
                self.tasks_by_id[repair_task.task_id]=repair_task
                self.observation_tasks[repair_task.task_id]=missing[0]
                self.metrics['status']='RUNNING_RETEST';self._save_executor()
                self._execute_parallel_pending()
                if self.coverage.delivered_fraction(self.weights)!=1.:
                    raise RuntimeError('one received retest did not complete required delivery')
                self.metrics['pending_retest']=[]
                self.metrics['retest_completed']=[repair_task.task_id]
                self._save_executor()
                delivered=self.coverage.delivered_fraction(self.weights)
            participants={member for item in self.plan.items for member in item.coalition}
            end=time.monotonic()+10.
            while True:
                now=rospy.Time.now().to_sec()
                with self.condition:samples={member:self.actual.get(member) for member in participants}
                if all(sample is not None and sample.is_fresh(now,.25) for sample in samples.values()):break
                if time.monotonic()>=end:raise RuntimeError('participating final state unavailable')
                time.sleep(.05)
            returns={member:math.dist(sample.position,scene['return_sites'][member]['position'])
                     for member,sample in samples.items()}
            self.metrics['return_distance_m']=returns
            self.metrics['return_completion']={member:dict(completed=completed,evidence=evidence)
                for member,distance in returns.items()
                for completed,evidence in (self._member_return_complete(
                    member,distance,scene['return_sites'][member]),)}
            complete=(all(item.status=='COMPLETED' for item in self.plan.items) and
                not self.active_executor_ids and delivered==1. and
                set(self.metrics['results_received'])==set(self.tasks_by_id) and
                all(row['completed'] for row in self.metrics['return_completion'].values()))
            self.metrics['status']='PASS_GEOMETRIC_PROXY_QUALIFICATION' if complete else 'FAILED'
            if not complete:self.metrics['failure_reason']='actual work, receipt or return incomplete'
            self._save_executor()
        except BaseException as error:
            self.metrics['failure_reason']=str(error)
            self.metrics['status']='UNKNOWN_LOCKED' if self.active_executor_ids else 'FAIL'
            print('Joint request failed: '+str(error),flush=True)
            if self.plan:
                for item in self.plan.items:
                    if item.status=='RUNNING':item.status='UNKNOWN_LOCKED'
            self._save_executor()
            if isinstance(error,(KeyboardInterrupt,SystemExit)):raise
        finally:
            if timer is not None:timer.shutdown()

    def _run_executor(self):
        from qn_aav_simulator.monitoring_request import expand
        from qn_aav_simulator.task_line import retest_tasks
        from qn_aav_simulator.observation_coverage import ObstacleBox
        self.points = {p.point_id: p.position for r in self.request.regions for p in r.interest_points}
        self.weights = {p.point_id: p.weight for r in self.request.regions for p in r.interest_points}
        scene = rospy.get_param("/scene", {})
        self.obstacles = ([ObstacleBox(tuple(scene["obstacle_center"]), tuple(scene["obstacle_size"]))]
                          if scene.get("obstacle_present", False) else [])
        self._save_executor()
        try:
            for unit in self.units:
                self._wait_executor_ready(unit)
            observations = expand(self.request, {c for u in self.units for c in u.capabilities})
            self.plan = self._executor_plan(observations, include_formation=True)
            self.metrics["initial_plan"] = asdict(self.plan)
            self.metrics["status"] = "AWAITING_CONFIRMATION"
            self._save_executor()
            print(json.dumps({"request_id": self.request.request_id, "tasks": asdict(self.plan),
                "targets": self.centers, "endpoints": {u.executor_id: u.action_endpoint for u in self.units},
                "limits": ["declared geometric visibility/dwell only; image quality unverified",
                           "zero-latency local result delivery; no USV/UUV online execution",
                           "formation transfer has no business shape/corridor pass threshold",
                           "one optional retest of uncovered points; no new batch during execution"]},
                 indent=2, default=json_default), flush=True)
            try:
                answer = input("Confirm this request and at most one retest? Type yes to dispatch: ")
            except (EOFError, KeyboardInterrupt):
                answer = ""
            if answer.strip().lower() != "yes":
                self.metrics["status"] = "NOT_CONFIRMED"
                self._save_executor()
                return
            self.metrics["confirmation"] = {"answer": "yes", "at_ros_s": rospy.Time.now().to_sec()}
            self.epoch = rospy.Time.now().to_sec()
            self.metrics["mission_epoch"] = self.epoch
            self.metrics["status"] = "RUNNING"
            self._refresh_executor_timing(0.0)
            retested = False
            while not rospy.is_shutdown():
                if not self.executor_serial:
                    self._execute_parallel_pending()
                pending = next((i for i in self.plan.items if i.status == "PLANNED"), None)
                if pending is None:
                    extra = retest_tasks(self.request, observations, self.coverage, self.weights,
                        delivery_recorded=all(t.task_id in self.metrics["results_received"] for t in observations),
                        already_retested=retested)
                    if extra:
                        retested = True
                        self.plan.items.extend(self._executor_plan(extra, release=rospy.Time.now().to_sec()-self.epoch).items)
                        self.plan_revision += 1
                        continue
                    break
                while rospy.Time.now().to_sec() - self.epoch < pending.planned_start:
                    if rospy.is_shutdown():
                        raise RuntimeError("shutdown before dispatch")
                    rospy.sleep(.1)
                self._dispatch_executor_item(pending)
            if rospy.is_shutdown():
                raise RuntimeError("runner shutdown")
            fraction = (self.coverage.delivered_fraction(self.weights) if self.request.delivery_required
                        else self.coverage.observed_fraction(self.weights))
            elapsed = rospy.Time.now().to_sec() - self.epoch
            self.metrics["deadline_met"] = (None if self.request.deadline_s is None
                                            else elapsed <= self.request.deadline_s)
            self.metrics["status"] = ("FAIL_COVERAGE" if fraction != 1.0 else
                                      "FAIL_DEADLINE" if self.metrics["deadline_met"] is False else "PASS_GEOMETRIC_PROXY")
            if self.metrics.get('native_qualification_only') and self.metrics['status']=='PASS_GEOMETRIC_PROXY':
                self.metrics['status']='QUALIFICATION_ONLY'
            self.metrics["formation_motion_complete"] = all(
                i.status == "COMPLETED" for i in self.plan.items if i.task_id not in self.observation_tasks)
            if fraction != 1.0:
                self.metrics["failure_reason"] = "points remain unobserved after the permitted retest"
            self._save_executor()
        except BaseException as error:
            self.metrics["failure_reason"] = str(error)
            self.metrics["status"] = "UNKNOWN_LOCKED" if self.active_executor_ids else "FAIL"
            if self.metrics.get("current_action"):
                self.metrics["current_action"]["phase"] = self.metrics["status"]
            if self.plan:
                for item in self.plan.items:
                    if item.status == "RUNNING":
                        item.status = "UNKNOWN_LOCKED"
            self._save_executor()
            raise

    # ------------------------------------------------------------ callbacks
    def on_goal(self, message):
        with self.condition:
            self.goal_ids.setdefault(message.goal.task_id, set()).add(
                message.goal_id.id)
            self.condition.notify_all()
        # Publish the native GoalID so UI cancellation targets this action only,
        # even if a later action starts before the cancellation message arrives.
        with self.executor_mutex:
            action=getattr(self,"metrics",{}).get("current_actions",{}).get(message.goal.task_id)
            if action is None:
                action=next((a for a in getattr(self,'metrics',{}).get('current_actions',{}).values()
                             if a.get('step_execution_id')==message.goal.task_id),None)
            if action is not None:action["goal_id"]=message.goal_id.id

    def on_result(self, message):
        with self.condition:
            self.native_results[message.status.goal_id.id] = message
            self.condition.notify_all()

    def on_command(self, message, agent_id):
        with self.lock_condition():
            self.command_trajectory.setdefault(agent_id, []).append(
                (int(message.trajectory_id), float(message.header.stamp.to_sec()),
                 rospy.Time.now().to_sec()))

    def on_diagnostics(self, message, agent_id):
        values = {}
        for status in message.status:
            for entry in status.values:
                values[entry.key] = entry.value
        if "source_trajectory_id" not in values:
            return
        with self.lock_condition():
            self.qn_source.setdefault(agent_id, []).append(
                (int(values["source_trajectory_id"]),
                 int(values.get("used_outer_step", -1)),
                 rospy.Time.now().to_sec()))

    def lock_condition(self):
        return self.condition

    # ------------------------------------------------------------- helpers
    def native_result(self, task_id):
        deadline = time.monotonic() + 10.0
        with self.condition:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                ids = self.goal_ids.get(task_id, set())
                if len(ids) > 1:
                    raise RuntimeError(
                        "more than one Action GoalID for {}".format(task_id))
                if ids:
                    goal_id = next(iter(ids))
                    if goal_id in self.native_results:
                        return goal_id, self.native_results[goal_id]
                self.condition.wait(0.05)
        raise RuntimeError(
            "native Action goal/result envelope missing for {}".format(task_id))

    def action_evidence(self, goal_id):
        index_path = self.output / "action_index.json"
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                index = json.loads(index_path.read_text())
            except (IOError, ValueError):
                index = {}
            name = index.get(goal_id)
            if name:
                try:
                    return json.loads((self.output / name).read_text())
                except ValueError:
                    pass
            rospy.sleep(0.05)
        return None

    def time_alignment_baseline(self):
        """The baseline snapshot that admitted this run."""
        raw = rospy.get_param("/formation_action_server/time_alignment_baseline", None)
        if raw is None:
            return self.time_alignment_session()
        try:
            return json.loads(raw)
        except (TypeError, ValueError):
            return {}

    def time_alignment_session(self):
        raw = rospy.get_param("/formation_action_server/time_alignment_session", None)
        if isinstance(raw, str):
            try:
                return json.loads(raw)
            except ValueError:
                return {}
        return raw if isinstance(raw, dict) else {}

    def save(self):
        summary = self.time_alignment_session()
        if summary:
            self.metrics["time_alignment_summary"] = summary
        self.metrics["final_plan"] = asdict(self.plan)
        self.metrics["planned_makespan"] = self.plan.makespan
        self.metrics["events"] = [asdict(e) for e in self.final_events.values()]
        self.metrics["processed_events"] = sorted(
            e.event_id for e in self.final_events.values())
        save_json(self.output / "metrics.json", self.metrics)

    def wait_ready(self, timeout_s=300.0):
        """Block until the Action server advertises READY_IDLE.

        Used before the first dispatch (30 s baseline qualification) and before
        every later dispatch: a resource that is still being released, or that
        lost a run-time health prerequisite, must not be handed a new task.
        """
        deadline = time.monotonic() + float(timeout_s)
        if not self.client.wait_for_server(rospy.Duration(min(120.0, float(timeout_s)))):
            raise RuntimeError("FormationAction server unavailable")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (rospy.get_param("/formation_action_server/ready", False)
                    and self.goal_sub.get_num_connections() > 0
                    and self.result_sub.get_num_connections() > 0):
                return True
            rospy.sleep(0.1)
        return False

    def wait_ready_or_raise(self):
        if not self.wait_ready(300.0):
            raise RuntimeError(
                "FormationAction never reached READY_IDLE: {}".format(
                    rospy.get_param("/formation_action_server/readiness_reason", "unknown")))
        self.metrics["ready_ros_time_s"] = rospy.Time.now().to_sec()
        self.metrics["baseline_qualification"] = self.time_alignment_baseline()
        save_json(self.output / "config.json", {
            "runner": rospy.get_param(rospy.get_name()),
            "monitor": rospy.get_param("/formation_action_server"),
            "agents": [asdict(a) for a in self.agents],
            "tasks": [asdict(t) for t in self.tasks],
            "use_sim_time": rospy.get_param("/use_sim_time", False),
            "repair_mode": self.repair_mode,
            "planner_parameters": {
                str(i): rospy.get_param("/drone_{}_ego_planner_node".format(i))
                for i in range(7)},
        })

    # ----------------------------------------------------------- execution
    def dispatch_goal_with_retry(self, goal, task):
        """Send one goal, waiting for READY_IDLE before every retry.

        A rejection while the resource is still owned by the previous task (or
        while the server is re-qualifying) is expected P1 behaviour, not an
        experiment failure.  Each retry is recorded; the adoption/dispatch
        evidence uses the attempt that was actually accepted.
        """
        timeout = rospy.get_param(
            "/formation_action_server/execution_timeout", 180.0) + 30.0
        attempts = 0
        while True:
            if attempts and not self.wait_ready(60.0):
                return None, None, rospy.Time.now().to_sec(), False, attempts
            dispatch_time = rospy.Time.now().to_sec()
            self.client.send_goal(goal)
            if not self.client.wait_for_result(rospy.Duration(timeout)):
                return None, None, dispatch_time, True, attempts
            state = self.client.get_state()
            result = self.client.get_result()
            if state == GoalStatus.REJECTED and attempts < 3:
                attempts += 1
                self.metrics.setdefault("dispatch_retries", []).append({
                    "task_id": task.task_id,
                    "attempt": attempts,
                    "ros_time_s": dispatch_time,
                    "server_reason": (result.reason if result is not None else None),
                })
                continue
            self.metrics.setdefault("dispatch_attempts", {})[task.task_id] = attempts + 1
            return state, result, dispatch_time, False, attempts + 1

    def _load_routing(self):
        """Read the static executor routing table from the launch parameters."""
        configured = rospy.get_param("~executors", None)
        return load_routing(
            configured,
            default_members=("drone_{}".format(i) for i in range(7)),
            default_initial_target_ref=self.initial_ref)

    def _online_executors(self):
        """Every unit this process can dispatch to.

        Units without an Action endpoint stay in the routing table - they may be
        planned for - but are never dispatched, so a target-fleet platform with
        no backend cannot produce a completion event.
        """
        online = dispatchable_units(self.routing)
        if not online:
            raise RuntimeError("no executor with a real Action endpoint is configured")
        return online

    def execution_context(self, item, epoch, previous_actual_finish, unit):
        with self.condition:
            pre_trajectory = {
                agent_id: (self.command_trajectory[agent_id][-1][0]
                           if self.command_trajectory.get(agent_id) else None)
                for agent_id in range(7)}
            pre_source = {
                agent_id: (self.qn_source[agent_id][-1][0]
                           if self.qn_source.get(agent_id) else None)
                for agent_id in range(7)}
        return {
            "execution_id": item.execution_id,
            "pre_dispatch_trajectory_ids": pre_trajectory,
            "pre_dispatch_qn_source_trajectory_ids": pre_source,
            "initial_planned_start": item.planned_start,
            "initial_planned_finish": item.planned_finish,
            "previous_actual_finish": previous_actual_finish,
            # Which plan revision this dispatch actually read, together with the
            # planned_start it read.  This is the evidence that the next action
            # used the plan the completion step produced.
            "plan_revision_at_dispatch": self.plan_revision,
            "planned_start_read_at_dispatch": item.planned_start,
            # Routing evidence: which unit the plan selected, which members it
            # owns and which endpoint the task was actually sent to.
            "executor_id": unit.executor_id,
            "action_endpoint": unit.action_endpoint,
            "assigned_members": tuple(item.coalition),
            "executed_members": tuple(unit.physical_agent_ids),
        }

    def collect_post_evidence(self, context, dispatch_ros_time_s, result_finish_s):
        with self.condition:
            post_trajectory = {}
            qn_source = {}
            for agent_id in range(7):
                post_trajectory[agent_id] = sorted({
                    trajectory_id
                    for trajectory_id, _stamp, received in self.command_trajectory.get(agent_id, [])
                    if received >= dispatch_ros_time_s})
                qn_source[agent_id] = [
                    (trajectory_id, step)
                    for trajectory_id, step, received in self.qn_source.get(agent_id, [])
                    if received >= dispatch_ros_time_s]
        context["post_dispatch_trajectory_ids"] = post_trajectory
        context["post_dispatch_qn_source"] = qn_source
        context["new_trajectory_observed"] = {
            agent_id: any(trajectory_id != context["pre_dispatch_trajectory_ids"][agent_id]
                          for trajectory_id in post_trajectory[agent_id])
            for agent_id in range(7)}
        context["qn_adopted_trajectory"] = {
            agent_id: any(trajectory_id != context["pre_dispatch_qn_source_trajectory_ids"][agent_id]
                          for trajectory_id, _step in qn_source[agent_id])
            for agent_id in range(7)}
        context["adoption_confirmed_by_runner"] = all(
            context["new_trajectory_observed"][agent_id]
            and context["qn_adopted_trajectory"][agent_id]
            for agent_id in range(7))

    def run(self):
        if self.planning_mode == "joint_request":
            return self._run_joint_request()
        if self.planning_mode == "executor":
            return self._run_executor()
        self.save()
        try:
            self.wait_ready_or_raise()
            epoch = rospy.Time.now().to_sec()
            self.metrics["mission_epoch"] = epoch
            self.metrics["status"] = "RUNNING"
            previous_actual_finish = 0.0
            while any(i.status == "PLANNED" for i in self.plan.items):
                item = next(i for i in self.plan.items if i.status == "PLANNED")
                task = self.tasks_by_id[item.task_id]
                while (not rospy.is_shutdown()
                       and rospy.Time.now().to_sec() < epoch + item.planned_start):
                    rospy.sleep(min(0.05, max(
                        0.0, epoch + item.planned_start - rospy.Time.now().to_sec())))
                if rospy.is_shutdown():
                    raise RuntimeError("ROS shutdown before dispatch")
                unit = unit_for_coalition(self.routing, item.coalition)
                if unit is None:
                    raise RuntimeError(
                        "no dispatchable executor owns the coalition the plan "
                        "allocated to {}: {}".format(item.task_id, tuple(item.coalition)))
                conflict = conflicting_active_unit(
                    self.routing, self.active_executor_ids, unit.executor_id)
                if conflict is not None:
                    raise RuntimeError(
                        "{} cannot be dispatched while {} is active: they share "
                        "physical members {}".format(
                            unit.executor_id, conflict.executor_id,
                            sorted(conflict.members() & unit.members())))
                context = self.execution_context(item, epoch, previous_actual_finish, unit)
                dispatched_start, dispatched_finish = item.planned_start, item.planned_finish
                goal = FormationGoal()
                goal.task_id = task.task_id
                goal.formation_center.header.frame_id = "world"
                goal.formation_center.header.stamp = rospy.Time.now()
                point = goal.formation_center.point
                point.x, point.y, point.z = self.centers[task.target_ref]
                goal.hold_duration = rospy.Duration.from_sec(task.service_time)
                item.status = "RUNNING"
                # Unknown/failed outcomes retain ownership of the physical members.
                self.active_executor_ids.add(unit.executor_id)
                state, result, dispatch_ros_time_s, timed_out, attempts = \
                    self.dispatch_goal_with_retry(goal, task)
                if timed_out:
                    item.status = "UNKNOWN_LOCKED"
                    self.metrics["status"] = "UNKNOWN_LOCKED"
                    self.metrics["failure"] = (
                        "Action result timeout for {}".format(task.task_id))
                    break
                if state == GoalStatus.REJECTED:
                    # P1: a busy/not-ready server rejects instead of preempting.
                    # The same task is retried after READY_IDLE; only a server
                    # that never returns to READY_IDLE is a run-level fault.
                    item.status = "UNKNOWN_LOCKED"
                    self.metrics["status"] = "UNKNOWN_LOCKED"
                    self.metrics["failure"] = (
                        "{} rejected after {} attempts; server did not return to "
                        "READY_IDLE".format(task.task_id, attempts))
                    break
                goal_id, envelope = self.native_result(task.task_id)
                evidence = self.action_evidence(goal_id)
                execution = dict(context)
                execution.update(
                    task_id=task.task_id,
                    goal_id=goal_id,
                    terminal_status=state,
                    envelope_status=envelope.status.status,
                    goal_dispatch_ros_time_s=dispatch_ros_time_s,
                    dispatch_ros_time_s=dispatch_ros_time_s - epoch,
                    planned_start_at_dispatch=dispatched_start,
                    planned_finish_at_dispatch=dispatched_finish,
                    action_reason=result.reason if result is not None else None,
                    action_task_outcome=(result.task_outcome if result is not None else None),
                    action_safety_outcome=(result.safety_outcome if result is not None else None),
                    action_experiment_validity=(
                        result.experiment_validity if result is not None else None),
                    model_time_hold_seconds=(
                        result.model_time_hold_seconds if result is not None else None),
                    planner_nominal_finish_times=(
                        evidence.get("planner_nominal_finish_times") if evidence else None),
                    adoption_verdict=(evidence.get("adoption_verdict") if evidence else None),
                    verdict=(evidence.get("verdict") if evidence else None),
                    evidence_file=(evidence.get("evidence_file") if evidence else None),
                )
                result_finish_s = rospy.Time.now().to_sec()
                self.collect_post_evidence(context, dispatch_ros_time_s, result_finish_s)
                # plan.md P2: every action records the trajectory_id set that
                # belongs to it.  It can only be assembled after the
                # post-dispatch observation has been collected.
                execution["trajectory_ids"] = sorted({
                    trajectory_id
                    for ids in context.get("post_dispatch_trajectory_ids", {}).values()
                    for trajectory_id in ids})
                execution["runner_adoption_confirmed"] = context[
                    "adoption_confirmed_by_runner"]
                execution["executor_matches_execution"] = (
                    execution["assigned_members"] == execution["executed_members"])
                if state != envelope.status.status:
                    execution["evidence_conflict"] = "Action status/envelope mismatch"
                if result is None:
                    execution["evidence_conflict"] = "Action returned no result"
                if execution.get("evidence_conflict"):
                    self.metrics["executions"].append(execution)
                    self.metrics["status"] = "INVALID"
                    self.metrics["failure"] = execution["evidence_conflict"]
                    break
                actual_start = result.actual_start_time.to_sec() - epoch
                actual_finish = result.actual_finish_time.to_sec() - epoch
                if not (math.isfinite(actual_start) and math.isfinite(actual_finish)
                        and actual_finish >= actual_start >= 0):
                    execution["evidence_conflict"] = "invalid Action timestamps"
                    self.metrics["executions"].append(execution)
                    self.metrics["status"] = "INVALID"
                    self.metrics["failure"] = execution["evidence_conflict"]
                    break
                nominal = (execution.get("planner_nominal_finish_times") or {})
                execution["planner_nominal_finish"] = (
                    max(nominal.values()) - epoch if nominal else None)
                execution.update(
                    actual_start=actual_start, actual_finish=actual_finish,
                    delay_seconds=actual_finish - dispatched_finish,
                    deadline_lateness=(None if task.deadline is None else max(0.0, actual_finish - task.deadline)),
                    scheduled_start_at_dispatch=dispatched_start,
                    expected_release_without_repair=max(
                        previous_actual_finish, context["initial_planned_start"]),
                )
                # How far the dispatch landed from the release boundary it was
                # waiting for.  This is a schedule-tracking figure, not evidence
                # that repair changed anything.
                execution["release_lag_s"] = (
                    execution["dispatch_ros_time_s"]
                    - execution["expected_release_without_repair"])
                execution["dispatch_changed"] = (
                    abs(execution["release_lag_s"]) > self.tolerance)
                self.metrics["executions"].append(execution)
                if state != GoalStatus.SUCCEEDED or result.reason != 0:
                    item.status = "FAILED"
                    self.metrics["status"] = "FAILED"
                    self.metrics["failure"] = "{} ended with status={} reason={}".format(
                        task.task_id, state, result.reason)
                    break
                if not context["adoption_confirmed_by_runner"]:
                    item.status = "REFERENCE_ADOPTION_UNCONFIRMED"
                    self.metrics["status"] = "INCOMPLETE"
                    self.metrics["failure"] = (
                        "runner could not confirm trajectory ownership for {} at "
                        "goal_dispatch_time={:.3f}".format(
                            task.task_id, dispatch_ros_time_s))
                    break
                previous_actual_finish = actual_finish
                if self.mode == "mission":
                    event = DelayEvent(
                        goal_id + ":completion", item.execution_id, task.task_id,
                        dispatched_finish, actual_finish)
                    before = asdict(self.plan)
                    self.plan, changed = process_completion(
                        self.plan, event, self.final_events, tolerance=self.tolerance,
                        repair_timing=(self.repair_mode == "on"))
                    execution["plan_updated"] = changed
                    self.plan_revision += 1
                    execution["plan_revision_after_completion"] = self.plan_revision
                    self.metrics["plan_history"].append({
                        "event_id": event.event_id, "plan_before": before,
                        "plan": asdict(self.plan), "plan_updated": changed,
                        "plan_revision": self.plan_revision})
                    execution["plan_before"] = before
                    execution["plan_after"] = asdict(self.plan)
                    execution["repair_timing"] = self.repair_mode == "on"
                else:
                    item.status = "COMPLETED"
                    execution["plan_updated"] = False
                    execution["plan_revision_after_completion"] = self.plan_revision
                self.active_executor_ids.discard(unit.executor_id)
                rospy.loginfo(
                    "%s complete actual=%.3f planned=%.3f delay=%.3f "
                    "plan_updated=%s dispatch_changed=%s",
                    task.task_id, actual_finish, dispatched_finish,
                    actual_finish - dispatched_finish,
                    execution.get("plan_updated"), execution.get("dispatch_changed"))
                self.save()
            if self.metrics["status"] == "RUNNING":
                self.metrics["actual_makespan"] = max(
                    (e["actual_finish"] for e in self.metrics["executions"]), default=None)
                self.metrics["status"] = "PASS"
            self._summarise_test_c()
        except Exception as error:
            self.metrics["status"] = self.metrics.get("status", "FAILED")
            if self.metrics["status"] == "RUNNING":
                self.metrics["status"] = "FAILED"
            self.metrics["failure"] = str(error)
            raise
        finally:
            self.save()

    def _plan_in_use(self, dispatched_start):
        """True when the next planned item reflects the updated Plan object."""
        remaining = [i for i in self.plan.items if i.status == "PLANNED"]
        if not remaining:
            return True
        after = [i for i in self.plan.items
                 if i.status == "PLANNED" and i.planned_start >= dispatched_start - 1e-6]
        return len(after) == len(remaining)

    def _summarise_test_c(self):
        executions = self.metrics["executions"]
        # A dispatch proves it used the updated plan only by having read the
        # revision the previous completion produced.
        for index, execution in enumerate(executions):
            if index == 0:
                execution["updated_plan_used"] = True
                continue
            previous = executions[index - 1]
            execution["updated_plan_used"] = (
                execution.get("plan_revision_at_dispatch")
                == previous.get("plan_revision_after_completion"))
        self.metrics["test_c"] = {
            "repair_mode": self.repair_mode,
            "plan_revision": self.plan_revision,
            "plan_updated": any(e.get("plan_updated") for e in executions),
            "plan_updated_actions": [e["task_id"] for e in executions
                                     if e.get("plan_updated")],
            "updated_plan_used": all(e.get("updated_plan_used", True) for e in executions),
            "dispatch_changed": any(e.get("dispatch_changed") for e in executions),
            "dispatch_changed_actions": [e["task_id"] for e in executions
                                         if e.get("dispatch_changed")],
            "release_lag_s": {e["task_id"]: e.get("release_lag_s")
                              for e in executions},
            "notes": (
                "release_lag_s is the dispatch offset from the release boundary "
                "it was waiting for; it is a schedule-tracking figure, not a "
                "measure of repair.  updated_plan_used reports that each "
                "dispatch read the plan revision the completion produced."),
        }


def main():
    rospy.init_node("formation_mission_runner")
    runner = MissionRunner()
    runner.run()
    if runner.metrics["status"] not in ("PASS", "PASS_GEOMETRIC_PROXY",
                                        "PASS_GEOMETRIC_PROXY_QUALIFICATION", "NOT_CONFIRMED"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
