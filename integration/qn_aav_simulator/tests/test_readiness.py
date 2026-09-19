"""P0 map/health: three-level readiness and CPU vs CUDA sensor backends."""

import pytest

from qn_aav_simulator.readiness import (
    GLOBAL_MAP_TOPIC, ReadinessEvaluator, RuntimeHealthMonitor, depth_topic,
    local_cloud_topic, odometry_topic, qn_diagnostics_topic,
)


def fully_ready(backend="CPU_POINTCLOUD", now=10.0, **kwargs):
    evaluator = ReadinessEvaluator(range(7), sensor_backend=backend, **kwargs)
    evaluator.note_topic_exists(GLOBAL_MAP_TOPIC)
    evaluator.note_message(GLOBAL_MAP_TOPIC, now - 0.1)
    for agent_id in range(7):
        for topic in (local_cloud_topic(agent_id), odometry_topic(agent_id),
                      qn_diagnostics_topic(agent_id)):
            evaluator.note_topic_exists(topic)
            evaluator.note_message(topic, now - 0.1)
    evaluator.note_planner_health("OK")
    return evaluator


def test_cpu_backend_never_requires_a_depth_image():
    evaluator = fully_ready()
    assert depth_topic(0) not in evaluator.topics
    assert local_cloud_topic(0) in evaluator.topics
    assert evaluator.evaluate(10.0).ready


def test_cuda_backend_requires_depth_instead_of_cloud():
    evaluator = ReadinessEvaluator(range(7), sensor_backend="CUDA_DEPTH")
    assert depth_topic(0) in evaluator.topics
    assert local_cloud_topic(0) not in evaluator.topics
    status = evaluator.evaluate(10.0)
    assert not status.ready
    assert depth_topic(0) in status.missing_topics


def test_readiness_separates_existence_message_and_validity():
    evaluator = ReadinessEvaluator(range(2))
    status = evaluator.evaluate(0.0)
    assert not status.ready
    assert GLOBAL_MAP_TOPIC in status.missing_topics
    assert status.topic_exists is False
    assert status.messages_received is False
    # Topic exists but no message has arrived yet: still not ready, and the
    # topic leaves the "missing" list for the "no message" state.
    evaluator.note_topic_exists(GLOBAL_MAP_TOPIC)
    evaluator.note_topic_exists(local_cloud_topic(0))
    status = evaluator.evaluate(0.0)
    assert GLOBAL_MAP_TOPIC not in status.missing_topics
    assert GLOBAL_MAP_TOPIC in status.stale_topics
    assert local_cloud_topic(0) in status.stale_topics
    assert status.messages_received is False
    assert status.inputs_valid is True
    # A message that arrived but is older than its window is also stale.
    evaluator.note_message(GLOBAL_MAP_TOPIC, 100.0)
    status = evaluator.evaluate(0.0)
    assert GLOBAL_MAP_TOPIC in status.stale_topics
    evaluator.note_message(GLOBAL_MAP_TOPIC, 0.0)
    assert GLOBAL_MAP_TOPIC not in evaluator.evaluate(0.0).stale_topics


def test_empty_map_is_rejected_unless_known_empty():
    evaluator = fully_ready()
    evaluator.note_message(GLOBAL_MAP_TOPIC, 9.9, empty=True)
    status = evaluator.evaluate(10.0)
    assert not status.ready
    assert GLOBAL_MAP_TOPIC in status.invalid_inputs
    assert any("empty map" in reason for reason in status.invalid_reasons)
    evaluator.known_empty_map = True
    assert evaluator.evaluate(10.0).ready
    assert evaluator.evaluate(10.0).map_state == "KNOWN_EMPTY"


def test_delivered_empty_scan_is_a_valid_observation():
    """An empty cloud that actually arrived is a completed scan with no returns.

    It proves the scan chain works; it is reported separately and is not read as
    "the environment is empty".  Only *never receiving anything* leaves the
    stream stale.
    """
    evaluator = fully_ready()
    evaluator.note_message(local_cloud_topic(4), 9.9, empty=True)
    status = evaluator.evaluate(10.0)
    assert status.ready
    assert local_cloud_topic(4) in status.empty_scans
    assert local_cloud_topic(4) not in status.invalid_inputs

    never_received = ReadinessEvaluator([0], require_odometry=False)
    never_received.note_topic_exists(GLOBAL_MAP_TOPIC)
    never_received.note_message(GLOBAL_MAP_TOPIC, 10.0)
    for agent_id in range(7):
        never_received.note_topic_exists(local_cloud_topic(agent_id))
        never_received.note_topic_exists(odometry_topic(agent_id))
        never_received.note_topic_exists(qn_diagnostics_topic(agent_id))
        never_received.note_message(odometry_topic(agent_id), 10.0)
        never_received.note_message(qn_diagnostics_topic(agent_id), 10.0)
    assert local_cloud_topic(0) in never_received.evaluate(10.0).stale_topics


def test_scene_source_accepts_an_explicitly_empty_map():
    """A declared scene source may publish a complete but obstacle-free map."""
    evaluator = fully_ready()
    evaluator.scene_source = True
    evaluator.note_message(GLOBAL_MAP_TOPIC, 9.9, empty=True)
    status = evaluator.evaluate(10.0)
    assert status.ready
    assert status.map_state == "KNOWN_EMPTY"

    without_declaration = fully_ready()
    without_declaration.note_message(GLOBAL_MAP_TOPIC, 9.9, empty=True)
    assert not without_declaration.evaluate(10.0).ready


def test_stale_odometry_is_reported_separately_from_missing():
    evaluator = fully_ready()
    evaluator.note_message(odometry_topic(2), 1.0)
    status = evaluator.evaluate(10.0)
    assert odometry_topic(2) in status.stale_topics
    assert odometry_topic(2) not in status.missing_topics


def test_runtime_health_reports_named_items():
    evaluator = fully_ready()
    health = RuntimeHealthMonitor(evaluator).evaluate(10.0)
    assert health["global_map_initialized"] and health["odometry_fresh"]
    assert health["planner_healthy"] and health["qn_state_updated"]
    evaluator.note_planner_health("planner 3 relative slots differ")
    assert RuntimeHealthMonitor(evaluator).evaluate(10.0)["planner_healthy"] is False
