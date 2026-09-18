"""Goal-callback separation, busy rejection and cancel semantics (P1).

These tests exercise the Action entry point without a ROS master.
"""

import importlib.util
from pathlib import Path
import queue
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

from qn_aav_simulator.action_lifecycle import (
    ACCEPTED, ACTIVE, READY_IDLE, REJECTED_BUSY, REJECTED_INVALID,
    REJECTED_NOT_READY, UNKNOWN_LOCKED, ActionResourceStateMachine,
)
from qn_aav_simulator.formation_monitor import DEFAULT_RELATIVE_SLOTS


@pytest.fixture
def server_module(monkeypatch):
    class Stamp:
        def __init__(self, seconds=0.0):
            self.seconds = seconds

        def to_sec(self):
            return self.seconds

        def to_nsec(self):
            return round(self.seconds * 1e9)

        @classmethod
        def from_sec(cls, seconds):
            return cls(seconds)

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace()
            self.pose = SimpleNamespace(orientation=SimpleNamespace())

    class TwistStamped(PoseStamped):
        def __init__(self):
            super().__init__()
            self.twist = SimpleNamespace(linear=SimpleNamespace())

    class Time(Stamp):
        @classmethod
        def now(cls):
            return cls(0.0)

        @classmethod
        def from_sec(cls, seconds):
            return cls(seconds)

    class FormationResult(object):
        NONE = 0
        INVALID_TARGET = 1
        ODOMETRY_TIMEOUT = 2
        EXECUTION_TIMEOUT = 3
        REFERENCE_ADOPTION_UNCONFIRMED = 4
        MODEL_TIME_MISMATCH = 5
        UNKNOWN_LOCKED = 6
        TASK_PASS = 1
        TASK_FAIL = 2
        SAFETY_PASS = 1
        SAFETY_FAIL = 2
        SAFETY_NOT_VERIFIED = 3
        VALIDITY_VALID = 1
        VALIDITY_INVALID = 2
        VALIDITY_INCOMPLETE = 3

        def __init__(self):
            self.task_id = ""
            self.goal_id = ""
            self.evidence_file = ""
            self.actual_start_time = None
            self.actual_finish_time = None
            self.reason = 0
            self.task_outcome = 0
            self.safety_outcome = 0
            self.experiment_validity = 0
            self.model_time_hold_seconds = 0.0

    modules = {
        "rospy": dict(
            Time=Time, Rate=object, is_shutdown=lambda: False,
            ROSInterruptException=RuntimeError, get_name=lambda: "/formation_action_server",
            get_param=lambda key, default=None: default,
            set_param=lambda *_args, **_kwargs: None,
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
            logerr=lambda *a, **k: None, loginfo_throttle=lambda *a, **k: None,
            Subscriber=lambda *a, **k: None, Publisher=lambda *a, **k: None,
            rostime=SimpleNamespace(wallsleep=lambda *_: None),
            signal_shutdown=lambda *_: None, sleep=lambda *_: None,
            get_published_topics=lambda: []),
        "actionlib": dict(ActionServer=object),
        "rosgraph": dict(Master=object),
        "actionlib_msgs.msg": dict(GoalStatus=SimpleNamespace(
            ACTIVE=1, PREEMPTING=6, SUCCEEDED=3, ABORTED=4, REJECTED=5)),
        "diagnostic_msgs.msg": dict(DiagnosticArray=object),
        "geometry_msgs.msg": dict(PoseStamped=PoseStamped, TwistStamped=TwistStamped),
        "nav_msgs.msg": dict(Odometry=object),
        "quadrotor_msgs.msg": dict(PositionCommand=object),
        "sensor_msgs.msg": dict(Image=object, PointCloud2=object),
        "std_msgs.msg": dict(Bool=object),
        "qn_aav_simulator.msg": dict(
            FormationAction=object,
            FormationFeedback=SimpleNamespace(MOVING=0, HOLDING=1),
            FormationResult=FormationResult),
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "scripts" / "formation_action_server.py"
    spec = importlib.util.spec_from_file_location("formation_action_server_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeGoalHandle:
    def __init__(self, goal_id="ros-goal-1", goal=None):
        self._goal_id = SimpleNamespace(id=goal_id, stamp=None)
        self._goal = goal or formation_goal()
        self.accepted = False
        self.rejected = None
        self.terminal = None
        self.feedback = []
        self.status = 1

    def get_goal(self):
        return self._goal

    def get_goal_id(self):
        return self._goal_id

    def get_goal_status(self):
        return SimpleNamespace(status=self.status)

    def set_accepted(self):
        self.accepted = True
        self.status = 1

    def set_rejected(self, result, text):
        self.rejected = (result, text)
        self.status = 5

    def set_succeeded(self, result, text):
        self.terminal = ("SUCCEEDED", result, text)
        self.status = 3

    def set_aborted(self, result, text):
        self.terminal = ("ABORTED", result, text)
        self.status = 4

    def set_canceled(self, result, text):
        self.terminal = ("CANCELED", result, text)

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)


def formation_goal(task_id="T1", x=0.0, y=0.0, z=0.5, frame="world", hold=1.0):
    return SimpleNamespace(
        task_id=task_id,
        hold_duration=SimpleNamespace(to_sec=lambda: hold),
        formation_center=SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),
            point=SimpleNamespace(x=x, y=y, z=z)))


def make_server(module, state=READY_IDLE, output_dir=None):
    server = object.__new__(module.FormationActionServer)
    server.lock = threading.RLock()
    server.state_machine = ActionResourceStateMachine(lock=server.lock)
    if state == READY_IDLE:
        server.state_machine.note_readiness(True)
    elif state == ACTIVE:
        server.state_machine.note_readiness(True)
        server.state_machine.request_goal("occupant")
    elif state == UNKNOWN_LOCKED:
        server.state_machine.fault("odometry loss")
    server.work_queue = queue.Queue()
    server.agent_ids = list(range(7))
    server.slots = dict(DEFAULT_RELATIVE_SLOTS)
    server.scale = 2.0
    server.epsilon_p = 0.5
    server.epsilon_v = 0.25
    server.odom_timeout = 0.25
    server.execution_timeout = 180.0
    server.monitor_rate = 20.0
    server.startup_timeout = 180.0
    server.sensor_backend = "CPU_POINTCLOUD"
    server.known_empty_map = False
    server.model_time_window = 0.06
    server.time_baseline_seconds = 30.0
    server.time_rate_lower = 0.95
    server.time_rate_upper = 1.05
    server.max_model_ros_drift = 0.05
    server.max_cross_agent_drift = 0.05
    server.min_valid_sample_ratio = 0.9
    server.inter_agent_clearance = 0.5
    server.obstacle_clearance = 0.2
    server.output_dir = Path(output_dir or "/tmp/formation-action-test")
    server.output_dir.mkdir(parents=True, exist_ok=True)
    server.odom = {}
    server.odom_history = {agent_id: __import__("collections").deque(maxlen=8)
                           for agent_id in range(7)}
    server.odom_counts = dict.fromkeys(range(7), 0)
    server.odometry_errors = {}
    import collections
    server.model_times = module.ModelTimeHistory(
        ["drone_{}".format(i) for i in range(7)])
    server.used_reference = {}
    server.command_trajectory = {}
    server.qn_source = {}
    server.planner_finish = {}
    server.map_cloud = None
    server.group_goal_messages = 0
    server.active_diagnostics = None
    server.max_diagnostics_callback_lag_s = 0.0
    server.session_alignment = server._new_alignment_monitor()
    server.readiness = module.ReadinessEvaluator(
        list(range(7)), sensor_backend="CPU_POINTCLOUD")
    server.health = module.RuntimeHealthMonitor(
        server.readiness, odometry_timeout_s=0.25)
    server._readiness_cache = None
    server._readiness_cache_time = 0.0
    server.goal_pub = SimpleNamespace(publish=lambda *_: None)
    server.master = SimpleNamespace()
    server.planner_finish = {}
    server.odometry_errors = {}
    return server


def test_busy_server_rejects_a_new_goal_without_preempting(server_module):
    server = make_server(server_module, state=ACTIVE)
    handle = FakeGoalHandle(goal_id="ros-goal-2")
    server._goal_callback(handle)
    assert handle.rejected is not None
    assert handle.accepted is False
    assert server.work_queue.empty()
    assert server.state_machine.state == ACTIVE
    assert server.state_machine.goal_id == "occupant"
    assert handle.terminal is None


def test_goal_before_readiness_is_rejected(server_module):
    server = make_server(server_module, state="BOOTING")
    handle = FakeGoalHandle()
    server._goal_callback(handle)
    assert handle.rejected is not None
    assert handle.accepted is False
    assert server.state_machine.state == "BOOTING"


def test_unknown_locked_refuses_goals(server_module):
    server = make_server(server_module, state=UNKNOWN_LOCKED)
    handle = FakeGoalHandle()
    server._goal_callback(handle)
    assert handle.rejected is not None
    assert "UNKNOWN_LOCKED" in handle.rejected[1]
    assert server.state_machine.state == UNKNOWN_LOCKED


def test_invalid_goal_is_rejected_without_reserving(server_module):
    server = make_server(server_module, state=READY_IDLE)
    handle = FakeGoalHandle(goal=formation_goal(frame="map"))
    server._goal_callback(handle)
    assert handle.rejected is not None
    assert server.state_machine.state == READY_IDLE
    assert server.state_machine.goal_id is None
    assert server.work_queue.empty()


def test_valid_goal_is_reserved_and_handed_over_immediately(server_module):
    server = make_server(server_module, state=READY_IDLE)
    handle = FakeGoalHandle()
    server._goal_callback(handle)
    assert handle.accepted is True
    assert server.state_machine.state == ACTIVE
    assert not server.work_queue.empty()
    work = server.work_queue.get_nowait()
    assert work.goal_handle is handle


def test_second_goal_while_reserved_is_rejected_and_first_survives(server_module):
    server = make_server(server_module, state=READY_IDLE)
    first = FakeGoalHandle(goal_id="ros-goal-1")
    second = FakeGoalHandle(goal_id="ros-goal-2")
    server._goal_callback(first)
    server._goal_callback(second)
    assert first.accepted is True and first.terminal is None
    assert second.rejected is not None
    assert server.state_machine.goal_id == "ros-goal-1"


def test_cancel_during_a_run_never_releases_or_ends_the_resource(server_module):
    server = make_server(server_module, state=ACTIVE)
    handle = FakeGoalHandle(goal_id="occupant")
    server._cancel_callback(handle)
    assert server.state_machine.state == ACTIVE
    assert server.state_machine.goal_id == "occupant"
    assert handle.terminal is None
    snapshot = server.state_machine.snapshot()
    assert snapshot["cancel_requested"] is True
    assert snapshot["cancel_ignored_while_running"] is True


def test_cancel_does_not_move_a_finished_task_backwards(server_module):
    server = make_server(server_module, state=READY_IDLE)
    handle = FakeGoalHandle(goal_id="")
    server._cancel_callback(handle)
    assert server.state_machine.state == READY_IDLE
    assert server.state_machine.goal_id is None


def test_worker_loop_reports_a_fault_as_unknown_locked(server_module):
    server = make_server(server_module, state=READY_IDLE)
    handle = FakeGoalHandle()
    server._goal_callback(handle)
    work = server.work_queue.get_nowait()
    server._fail_task(work, "odometry stream lost")
    assert server.state_machine.state == UNKNOWN_LOCKED
    assert handle.terminal[0] == "ABORTED"
    assert handle.terminal[1].reason == server_module.FormationResult.UNKNOWN_LOCKED

def _odometry_message(agent_id=0, stamp=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="world", stamp=SimpleNamespace(
            to_sec=lambda: stamp)),
        child_frame_id="drone_{}/base_link".format(agent_id),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.5),
            orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))))


def _diagnostics_message(model_time_s=1.0, stamp=0.0):
    values = [
        SimpleNamespace(key="model_time_s", value=str(model_time_s)),
        SimpleNamespace(key="source_trajectory_id", value="3"),
        SimpleNamespace(key="used_outer_step", value="10"),
        SimpleNamespace(key="source_command_stamp", value="0.5"),
        SimpleNamespace(key="outer_dt_s", value="0.01"),
        SimpleNamespace(key="velocity_directly_consumed", value="false"),
        SimpleNamespace(key="acceleration_directly_consumed", value="false"),
    ]
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
        status=[SimpleNamespace(values=values)])


def test_odometry_and_diagnostics_mark_readiness_messages(server_module):
    """Level 2 readiness needs the received-message hooks, not only topics."""
    server = make_server(server_module, state=READY_IDLE)
    server._standard_odom_callback(_odometry_message(0), 0)
    server._diagnostics_callback(_diagnostics_message(stamp=7.5), 0)
    assert server.readiness.topics["/drone_0_qn/odometry"].message_received is True
    assert server.readiness.topics["/drone_0_qn/diagnostics"].message_received is True
    assert server.readiness.topics["/drone_0_qn/odometry"].last_stamp_s is not None
    # Alignment must use the publisher's ROS stamp, not this process' receive
    # time, otherwise server scheduling lag is reported as model-time drift.
    sample = server.session_alignment.latest_observation("drone_0")
    assert sample[0] == 7.5
    assert sample[1] == 1.0


def test_message_age_uses_a_clock_read_after_the_planner_probe(server_module,
                                                               monkeypatch):
    """A message arriving during the slow planner-health probe is not stale.

    The readiness loop used to hand ``readiness.evaluate`` a ``now`` captured
    before the master XML-RPC calls; every message delivered during that window
    then had a negative age and was reported stale, so READY_IDLE could never be
    reached even though the qn topics were flowing at ~50-90 Hz.
    """
    clock = {"now": 100.0}

    class Clock(object):
        @staticmethod
        def now():
            return SimpleNamespace(to_sec=lambda: clock["now"])

    monkeypatch.setattr(server_module.rospy, "Time", Clock)
    server = make_server(server_module, state=READY_IDLE)
    server.readiness.note_topic_exists("/drone_0_qn/odometry", True)

    def slow_probe():
        clock["now"] += 0.15
        server._standard_odom_callback(_odometry_message(0), 0)
        return "OK"

    monkeypatch.setattr(server, "_planner_interface_health", slow_probe)
    snapshot = server._readiness_snapshot(now=100.0)
    assert "/drone_0_qn/odometry" not in snapshot["stale_topics"]
    assert snapshot["topic_ages"]["/drone_0_qn/odometry"] == pytest.approx(0.0)


def test_topic_existence_refresh_accepts_master_pairs(server_module, monkeypatch):
    """The master returns [[name, type], ...]; lists are not hashable."""
    server = make_server(server_module, state=READY_IDLE)
    monkeypatch.setattr(server_module.rospy, "get_published_topics",
                        lambda: [["/drone_0_qn/odometry", "nav_msgs/Odometry"]])
    server._refresh_topic_existence()
    assert server.readiness.topics["/drone_0_qn/odometry"].exists is True
    assert server.readiness.topics["/map_generator/global_cloud"].exists is False


def test_windowed_odometry_is_accepted_by_the_ledger_and_obstacle_check(
        server_module):
    """The monitor consumes a sample window; per-tick helpers take the newest.

    GroupCompletionMonitor.evaluate() reads every state received since the
    previous tick so a transient excursion cannot hide between two 20 Hz ticks.
    Regression: the ledger recorder and the sampled obstacle check still
    indexed the window as if it held one OdometrySample, which aborted T1 with
    "'list' object has no attribute 'position'".
    """
    server = make_server(server_module, state=READY_IDLE)
    windows = {}
    for agent_id in range(7):
        windows[agent_id] = []
        for index, stamp in enumerate((1.0, 1.01, 1.02)):
            message = _odometry_message(agent_id, stamp)
            message.pose.pose.position.x = float(agent_id) + 0.001 * index
            sample = server_module.parse_standard_odometry(
                message, "drone_{}".format(agent_id))
            windows[agent_id].append(sample)

    monitor = server_module.GroupCompletionMonitor(
        (0.0, 0.0, 0.5), 5.0, 0.0, agent_ids=list(range(7)),
        relative_slots=DEFAULT_RELATIVE_SLOTS, swarm_scale=2.0)
    member_samples = {agent_id: [] for agent_id in range(7)}
    server.used_reference = {
        agent_id: {"position": (0.0, 0.0, 0.5), "velocity": (0.0, 0.0, 0.0),
                   "ros_time_s": 1.02}
        for agent_id in range(7)}
    server._record_member_samples(member_samples, windows, server.used_reference,
                                  monitor, 1.02)
    assert all(len(series) == 1 for series in member_samples.values())
    assert member_samples[3][0].position[0] == pytest.approx(3.002)
    assert member_samples[3][0].used_reference_position == (0.0, 0.0, 0.5)

    import numpy
    server.map_cloud = numpy.asarray([[0.0, 0.0, 0.5]], dtype=float)
    clearance = server._obstacle_clearance(windows)
    # Agent 0 is the closest member to the single cloud point.
    assert clearance == pytest.approx(0.002)
