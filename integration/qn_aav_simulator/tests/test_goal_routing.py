"""Goal routing and target meaning (task line: single member vs formation).

Every planner subscribes to its goal topic and the formation entry always adds
the member's slot offset.  A command that addresses one member, or asks it to
hold, must therefore arrive on the member entry with the position used as-is.
These tests pin that contract down without a ROS master.
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def server_module(monkeypatch):
    modules = {
        "rospy": dict(
            Publisher=lambda *a, **k: SimpleNamespace(publish=lambda *a, **k: None),
            Subscriber=lambda *a, **k: None,
            get_param=lambda name, default=None: default,
            get_name=lambda: "/formation_action_server",
            loginfo=lambda *a, **k: None,
            logwarn=lambda *a, **k: None,
            Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 0.0)),
            set_param=lambda *a, **k: None,
        ),
        "actionlib": dict(ActionServer=object),
        "rosgraph": dict(Master=object),
        "actionlib_msgs.msg": dict(GoalStatus=SimpleNamespace(
            ACTIVE=1, PREEMPTING=6, SUCCEEDED=3, ABORTED=4, REJECTED=5)),
        "diagnostic_msgs.msg": dict(DiagnosticArray=object),
        "geometry_msgs.msg": dict(PoseStamped=object, TwistStamped=object),
        "nav_msgs.msg": dict(Odometry=object),
        "quadrotor_msgs.msg": dict(PositionCommand=object),
        "sensor_msgs.msg": dict(Image=object, PointCloud2=object),
        "std_msgs.msg": dict(Bool=object),
        "qn_aav_simulator.msg": dict(
            FormationAction=object,
            FormationFeedback=SimpleNamespace(MOVING=0, HOLDING=1),
            FormationResult=object),
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "scripts" / "formation_action_server.py"
    spec = importlib.util.spec_from_file_location("goal_routing_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def route_for(module, agent_ids, formation_format, member_format):
    server = object.__new__(module.FormationActionServer)
    server.agent_ids = list(agent_ids)
    server.formation_goal_topics = {
        agent: module.goal_topic(formation_format, agent) for agent in agent_ids}
    server.member_goal_topics = (
        {agent: module.goal_topic(member_format, agent) for agent in agent_ids}
        if member_format else {})
    return server.goal_route()


def test_a_template_without_a_placeholder_is_a_broadcast():
    assert "{}" not in "/move_base_simple/goal"
    assert "/move_base_simple/goal" == "/move_base_simple/goal"


def test_broadcast_template_keeps_the_upstream_single_topic(server_module):
    semantics, route = route_for(server_module, range(7), "/move_base_simple/goal", "")
    assert semantics == "FORMATION_CENTRE"
    assert route == [(agent, "/move_base_simple/goal") for agent in range(7)]


def test_per_member_template_resolves_per_agent(server_module):
    semantics, route = route_for(
        server_module, [0, 1, 2], "/drone_{}_formation_goal", "")
    assert semantics == "FORMATION_CENTRE"
    assert route == [(0, "/drone_0_formation_goal"),
                     (1, "/drone_1_formation_goal"),
                     (2, "/drone_2_formation_goal")]


def test_a_single_member_unit_with_a_member_entry_is_a_member_target(server_module):
    """One member plus a member entry means the position is used as-is."""
    semantics, route = route_for(
        server_module, [1], "/drone_{}_formation_goal", "/drone_{}_member_goal")
    assert semantics == "MEMBER_TARGET"
    assert route == [(1, "/drone_1_member_goal")]


def test_a_multi_member_unit_stays_a_formation_centre(server_module):
    """The group case must keep centre semantics even when the entry exists."""
    semantics, route = route_for(
        server_module, [0, 1, 2], "/drone_{}_formation_goal", "/drone_{}_member_goal")
    assert semantics == "FORMATION_CENTRE"
    assert [topic for _agent, topic in route] == [
        "/drone_0_formation_goal", "/drone_1_formation_goal", "/drone_2_formation_goal"]


def test_a_single_member_unit_without_a_member_entry_cannot_use_member_semantics(
        server_module):
    """Without the member entry there is no way to address one member safely."""
    semantics, route = route_for(server_module, [1], "/move_base_simple/goal", "")
    assert semantics == "FORMATION_CENTRE"
    assert route == [(1, "/move_base_simple/goal")]


def test_member_topics_for_covers_both_entries(server_module):
    server = object.__new__(server_module.FormationActionServer)
    server.agent_ids = [1]
    server.formation_goal_topics = {1: "/drone_1_formation_goal"}
    server.member_goal_topics = {1: "/drone_1_member_goal"}
    assert server._member_goal_topics_for(1) == [
        "/drone_1_formation_goal", "/drone_1_member_goal"]


def test_member_topics_for_omits_the_member_entry_when_unconfigured(server_module):
    server = object.__new__(server_module.FormationActionServer)
    server.agent_ids = [1]
    server.formation_goal_topics = {1: "/move_base_simple/goal"}
    server.member_goal_topics = {}
    assert server._member_goal_topics_for(1) == ["/move_base_simple/goal"]
