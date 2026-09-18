"""Exercise Action side effects and diagnostics without a ROS master."""

import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

from qn_aav_simulator.formation_monitor import DEFAULT_RELATIVE_SLOTS, OdometrySample


@pytest.fixture
def action_server(monkeypatch, tmp_path):
    clock = SimpleNamespace(now=10.0, sleep_hook=lambda: None)

    class Stamp:
        def __init__(self, seconds):
            self.seconds = seconds

        @classmethod
        def now(cls):
            return cls(clock.now)

        @classmethod
        def from_sec(cls, seconds):
            return cls(seconds)

        def to_sec(self):
            return self.seconds

        def to_nsec(self):
            return round(self.seconds * 1e9)

        def __sub__(self, other):
            return Stamp(self.seconds - other.seconds)

    class Rate:
        def __init__(self, rate):
            self.step = 1.0 / rate

        def sleep(self):
            clock.now += self.step
            clock.sleep_hook()

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace()
            self.pose = SimpleNamespace(orientation=SimpleNamespace())

    class Feedback:
        MOVING, HOLDING = 0, 1

    modules = {
        "rospy": dict(Time=Stamp, Rate=Rate, is_shutdown=lambda: False,
                      ROSInterruptException=RuntimeError, get_name=lambda: "/formation_action_server"),
        "actionlib": {}, "rosgraph": {},
        "geometry_msgs.msg": dict(PoseStamped=PoseStamped),
        "nav_msgs.msg": dict(Odometry=object), "std_msgs.msg": dict(Bool=object),
        "qn_aav_simulator.msg": dict(FormationAction=object,
                                     FormationFeedback=Feedback, FormationResult=SimpleNamespace),
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "scripts" / "formation_action_server.py"
    spec = importlib.util.spec_from_file_location("formation_action_server_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FakeAction:
        def __init__(self):
            self.current_goal = SimpleNamespace(get_goal_id=lambda: SimpleNamespace(id="ros-goal-1"))
            self.cancelled = False
            self.feedback = []
            self.terminal = None

        def is_preempt_requested(self):
            return self.cancelled

        def publish_feedback(self, message):
            self.feedback.append(message.phase)

        def finish(self, state, result, text):
            # The runner can end the simulation as soon as it receives this result.
            # Diagnostics must already be durable and readable at this boundary.
            assert list(tmp_path.glob("*.diagnostics.json"))
            self.terminal = state, result, text

        def set_succeeded(self, result, text):
            self.finish("SUCCEEDED", result, text)

        def set_preempted(self, result, text):
            self.finish("PREEMPTED", result, text)

        def set_aborted(self, result, text):
            self.finish("ABORTED", result, text)

    server = object.__new__(module.FormationActionServer)
    server.agent_ids = list(range(7))
    server.slots = dict(DEFAULT_RELATIVE_SLOTS)
    server.scale, server.epsilon_p, server.epsilon_v = 2.0, 0.5, 0.25
    server.odom_timeout, server.execution_timeout, server.monitor_rate = 0.25, 180.0, 20.0
    server.output_dir, server.lock = tmp_path, threading.Lock()
    server.active_diagnostics = None
    server.odom_counts = dict.fromkeys(range(7), 0)
    server.server = FakeAction()
    publications = []
    server.goal_pub = SimpleNamespace(publish=publications.append)

    def refresh(velocity=0.0):
        server.odom = {
            agent_id: OdometrySample(clock.now,
                (2 * slot[0], 2 * slot[1], 0.5),
                (velocity if agent_id == 6 else 0.0, 0.0, 0.0))
            for agent_id, slot in server.slots.items()
        }
        for agent_id in server.agent_ids:
            server.odom_counts[agent_id] += 1

    refresh()
    clock.sleep_hook = refresh
    goal = SimpleNamespace(
        task_id="T1", hold_duration=Stamp(0.0),
        formation_center=SimpleNamespace(
            header=SimpleNamespace(frame_id="world"),
            point=SimpleNamespace(x=0.0, y=0.0, z=0.5)))
    return SimpleNamespace(server=server, module=module, clock=clock, goal=goal,
                           publications=publications, refresh=refresh, output_dir=tmp_path)


def diagnostic(fixture):
    return json.loads(next(fixture.output_dir.glob("*.diagnostics.json")).read_text())


def test_one_valid_action_publishes_exactly_one_native_group_goal(action_server):
    f = action_server
    f.server._execute(f.goal)
    assert len(f.publications) == 1
    assert f.publications[0].header.frame_id == "world"
    assert f.server.server.terminal[0] == "SUCCEEDED"
    record = diagnostic(f)
    assert record["goal_id"] == "ros-goal-1"
    assert record["goal_publish_count"] == 1
    assert record["successful_hold_window"]["required_duration"] == 0.0
    assert next(f.output_dir.glob("*.monitor.csv")).read_text().count("HOLDING") == 1


def test_invalid_goal_does_not_reach_swarm(action_server):
    f = action_server
    f.goal.formation_center.point.z = float("nan")
    f.server._execute(f.goal)
    assert not f.publications
    assert f.server.server.terminal[0] == "ABORTED"
    assert f.server.server.terminal[1].reason == 1
    assert diagnostic(f)["formation_center"][2] is None


def test_stale_member_is_rechecked_before_publishing(action_server):
    f = action_server
    del f.server.odom[5]
    f.server._execute(f.goal)
    assert not f.publications
    assert f.server.server.terminal[1].reason == 2
    assert diagnostic(f)["failed_agent_ids"] == [5]


def test_cancel_before_dispatch_has_timestamps_and_no_group_goal(action_server):
    f = action_server
    f.server.server.cancelled = True
    f.server._execute(f.goal)
    assert not f.publications
    state, result, _text = f.server.server.terminal
    assert state == "PREEMPTED"
    assert result.actual_start_time.to_sec() == 10.0
    assert result.actual_finish_time.to_sec() == 10.0
    assert diagnostic(f)["successful_hold_window"] is None


def test_cancel_during_hold_does_not_report_success(action_server):
    f = action_server
    f.goal.hold_duration.seconds = 0.2
    f.clock.sleep_hook = lambda: setattr(f.server.server, "cancelled", True)
    f.server._execute(f.goal)
    assert len(f.publications) == 1
    assert f.server.server.terminal[0] == "PREEMPTED"
    assert diagnostic(f)["successful_hold_window"] is None


def test_phase_resets_are_published_and_nominal_finish_does_not_complete_task(action_server):
    f = action_server
    f.goal.hold_duration.seconds = 0.1
    f.refresh(velocity=0.3)
    steps = iter((0.0, 0.3, 0.0, 0.0, 0.0, 0.0))

    def advance():
        f.refresh(velocity=next(steps))
        f.server._finish_callback(SimpleNamespace(data=True), 0)

    f.clock.sleep_hook = advance
    f.server._execute(f.goal)
    assert f.server.server.feedback == [0, 1, 0, 1]
    assert f.server.server.terminal[0] == "SUCCEEDED"
    record = diagnostic(f)
    assert record["planner_finished_early"] is True
    assert record["planner_nominal_finish_times"]["0"] < record["actual_finish_time"]
    assert record["successful_hold_window"]["duration"] >= 0.1


def test_readiness_requires_connections_to_named_planners_not_observer_count(action_server, monkeypatch):
    f = action_server
    server = f.server
    server.map_seen, server.depth_streams_seen = True, set(range(7))
    server._planner_configuration_matches = lambda: None
    expected = ["/drone_{}_ego_planner_node".format(i) for i in range(7)]
    publishers = [("/drone_{}_visual_slam/odom".format(i), ["/drone_{}_qn_aav".format(i)])
                  for i in range(7)]
    server.master = SimpleNamespace(
        getSystemState=lambda: (publishers, [("/move_base_simple/goal", expected)], []),
        lookupNode=lambda _: "http://localhost:1234")
    destinations = expected[:6] + ["/an_observer"]

    class Proxy:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def getBusInfo(self, _name):
            return 1, "", [(i, peer, "o", "TCPROS", "/move_base_simple/goal", True)
                           for i, peer in enumerate(destinations)]

    monkeypatch.setattr(f.module.xmlrpc.client, "ServerProxy", lambda *_a, **_k: Proxy())
    assert "TCPROS" in server._readiness_reason()
    destinations[:] = expected
    assert server._readiness_reason() is None
    publishers[0][1].append("/fake_state")
    assert "exactly its qn publisher" in server._readiness_reason()


def test_startup_observes_the_renderers_actual_depth_topics(action_server, monkeypatch):
    f = action_server
    subscriptions = []
    monkeypatch.setattr(f.module.rospy, "get_param", lambda key, default=None:
                        str(f.output_dir) if key == "~output_dir" else default, raising=False)
    monkeypatch.setattr(f.module.rospy, "set_param", lambda *_args: None, raising=False)
    monkeypatch.setattr(f.module.rospy, "AnyMsg", object, raising=False)
    monkeypatch.setattr(f.module.rospy, "Publisher", lambda *_args, **_kw: f.server.goal_pub,
                        raising=False)
    monkeypatch.setattr(f.module.rospy, "Subscriber", lambda topic, *_args, **_kw:
                        subscriptions.append(topic), raising=False)
    monkeypatch.setattr(f.module.rosgraph, "Master", lambda *_args: object(), raising=False)
    monkeypatch.setattr(f.module.actionlib, "SimpleActionServer", lambda *_args, **_kw:
                        f.server.server, raising=False)
    monkeypatch.setattr(f.module.threading, "Thread", lambda *_args, **_kw:
                        SimpleNamespace(start=lambda: None))
    f.module.FormationActionServer()
    assert all("/drone_{}_pcl_render_node/depth".format(i) in subscriptions for i in range(7))
    assert not any(topic.endswith("/cloud") for topic in subscriptions)
