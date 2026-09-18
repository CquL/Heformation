"""Three-level readiness and runtime health for the FormationAction server.

The Action server process starts immediately; only ``READY_IDLE`` accepts a
Goal.  Readiness is separated into

    topic_exists            the topic is advertised by the expected publisher
    message_received        a message actually arrived and is fresh
    input_valid             the payload is usable for this planning request

CPU point-cloud mode never waits for a depth image: the upstream
``local_sensing_node`` only builds the depth renderer when ``ENABLE_CUDA`` is
on, and the CPU build publishes ``pcl_render_node/cloud`` instead.  An empty
cloud is *not* evidence of a known-empty environment unless the run explicitly
sets ``known_empty_map=true``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

SENSOR_BACKEND_CPU = "CPU_POINTCLOUD"
SENSOR_BACKEND_CUDA = "CUDA_DEPTH"
SENSOR_BACKENDS = (SENSOR_BACKEND_CPU, SENSOR_BACKEND_CUDA)

GLOBAL_MAP_TOPIC = "/map_generator/global_cloud"


def local_cloud_topic(agent_id: int) -> str:
    return "/drone_{}_pcl_render_node/cloud".format(agent_id)


def depth_topic(agent_id: int) -> str:
    return "/drone_{}_pcl_render_node/depth".format(agent_id)


def odometry_topic(agent_id: int) -> str:
    return "/drone_{}_qn/odometry".format(agent_id)


def qn_diagnostics_topic(agent_id: int) -> str:
    return "/drone_{}_qn/diagnostics".format(agent_id)


def position_command_topic(agent_id: int) -> str:
    return "/drone_{}_planning/pos_cmd".format(agent_id)


def planner_finish_topic(agent_id: int) -> str:
    return "/drone_{}_planning/finish".format(agent_id)


def swarm_compat_odometry_topic(agent_id: int) -> str:
    """World-linear-velocity odometry kept for the Swarm planners only."""
    return "/drone_{}_visual_slam/odom".format(agent_id)


@dataclass
class TopicHealth:
    name: str
    exists: bool = False
    message_received: bool = False
    last_stamp_s: Optional[float] = None
    valid: bool = True
    empty: bool = False
    detail: str = ""
    message_count: int = 0

    def age_s(self, now: float) -> Optional[float]:
        if self.last_stamp_s is None:
            return None
        return now - self.last_stamp_s


@dataclass
class ReadinessStatus:
    ready: bool
    reason: str
    sensor_backend: str
    missing_topics: Tuple[str, ...] = ()
    stale_topics: Tuple[str, ...] = ()
    invalid_inputs: Tuple[str, ...] = ()
    invalid_reasons: Tuple[str, ...] = ()
    planner_health: str = "UNKNOWN"
    map_state: str = "UNKNOWN"
    known_empty_map: bool = False
    topic_exists: bool = False
    messages_received: bool = False
    inputs_valid: bool = False
    known_empty_sensing: Tuple[str, ...] = ()

    def as_dict(self) -> Dict[str, object]:
        return {
            "ready": self.ready,
            "reason": self.reason,
            "sensor_backend": self.sensor_backend,
            "missing_topics": list(self.missing_topics),
            "stale_topics": list(self.stale_topics),
            "invalid_inputs": list(self.invalid_inputs),
            "invalid_reasons": list(self.invalid_reasons),
            "planner_health": self.planner_health,
            "map_state": self.map_state,
            "known_empty_map": self.known_empty_map,
            "topic_exists": self.topic_exists,
            "messages_received": self.messages_received,
            "inputs_valid": self.inputs_valid,
            "known_empty_sensing": list(self.known_empty_sensing),
        }


class ReadinessEvaluator:
    """Track topic existence, message arrival, freshness and validity."""

    def __init__(self, agent_ids: Iterable[int], *,
                 sensor_backend: str = SENSOR_BACKEND_CPU,
                 known_empty_map: bool = False,
                 cloud_timeout_s: float = 2.0,
                 odometry_timeout_s: float = 0.25,
                 diagnostics_timeout_s: float = 1.0,
                 require_odometry: bool = True) -> None:
        self.agent_ids = tuple(int(agent_id) for agent_id in agent_ids)
        if not self.agent_ids:
            raise ValueError("agent_ids must not be empty")
        backend = str(sensor_backend).strip().upper()
        if backend not in SENSOR_BACKENDS:
            raise ValueError("sensor_backend must be one of {}".format(SENSOR_BACKENDS))
        self.sensor_backend = backend
        self.known_empty_map = bool(known_empty_map)
        for name, value in (("cloud_timeout_s", cloud_timeout_s),
                            ("odometry_timeout_s", odometry_timeout_s),
                            ("diagnostics_timeout_s", diagnostics_timeout_s)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        self.cloud_timeout_s = float(cloud_timeout_s)
        self.odometry_timeout_s = float(odometry_timeout_s)
        self.diagnostics_timeout_s = float(diagnostics_timeout_s)
        self.require_odometry = bool(require_odometry)
        self.topics: Dict[str, TopicHealth] = {}
        self.planner_health = "UNKNOWN"
        self.map_state = "UNKNOWN"
        self._register_required_topics()

    def _register_required_topics(self) -> None:
        self.topics[GLOBAL_MAP_TOPIC] = TopicHealth(GLOBAL_MAP_TOPIC)
        for agent_id in self.agent_ids:
            topic = (depth_topic(agent_id)
                     if self.sensor_backend == SENSOR_BACKEND_CUDA
                     else local_cloud_topic(agent_id))
            self.topics[topic] = TopicHealth(topic)
            if self.require_odometry:
                odom = odometry_topic(agent_id)
                self.topics[odom] = TopicHealth(odom)
            diagnostics = qn_diagnostics_topic(agent_id)
            self.topics[diagnostics] = TopicHealth(diagnostics)

    # -- observation hooks -------------------------------------------------
    def note_topic_exists(self, topic: str, exists: bool = True) -> None:
        self.topics.setdefault(topic, TopicHealth(topic)).exists = bool(exists)

    def note_message(self, topic: str, stamp_s: float, *,
                     valid: bool = True, empty: bool = False,
                     detail: str = "") -> None:
        if not math.isfinite(stamp_s):
            raise ValueError("message stamp must be finite")
        health = self.topics.setdefault(topic, TopicHealth(topic))
        health.message_received = True
        health.last_stamp_s = float(stamp_s)
        health.message_count += 1
        health.valid = bool(valid)
        health.empty = bool(empty)
        health.detail = detail

    def note_planner_health(self, health: str) -> None:
        self.planner_health = str(health)

    # -- evaluation --------------------------------------------------------
    @staticmethod
    def _is_local_sensing(topic: str) -> bool:
        return (topic.endswith("pcl_render_node/cloud")
                or topic.endswith("pcl_render_node/depth"))

    def _timeout(self, topic: str) -> float:
        if topic == GLOBAL_MAP_TOPIC:
            return self.cloud_timeout_s
        if topic.endswith("/diagnostics"):
            return self.diagnostics_timeout_s
        if topic.endswith("/odometry"):
            return self.odometry_timeout_s
        return self.cloud_timeout_s

    def evaluate(self, now: float) -> ReadinessStatus:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        missing: List[str] = []
        stale: List[str] = []
        invalid: List[str] = []
        invalid_reasons: List[str] = []
        known_empty_sensing: List[str] = []
        for topic, health in sorted(self.topics.items()):
            if not health.exists:
                # Level 1: the topic is not advertised at all.
                missing.append(topic)
                continue
            if not health.message_received:
                if self.known_empty_map and self._is_local_sensing(topic):
                    # The upstream CPU renderer only publishes when points are
                    # inside the sensing horizon.  An explicitly configured
                    # known-empty operating region is therefore accepted, and
                    # the absence is reported instead of being treated as a
                    # fresh, trusted measurement.
                    known_empty_sensing.append(topic)
                    continue
                # Level 2: advertised, but nothing has been received yet.
                stale.append(topic)
                continue
            age = health.age_s(now)
            if age is None or age < 0.0 or age > self._timeout(topic):
                stale.append(topic)
                continue
            if topic == GLOBAL_MAP_TOPIC:
                if health.empty:
                    if not self.known_empty_map:
                        invalid.append(topic)
                        invalid_reasons.append(
                            "{}: empty map without known_empty_map=true".format(topic))
                        continue
                    self.map_state = "KNOWN_EMPTY"
                else:
                    self.map_state = "POPULATED"
            if topic.endswith("pcl_render_node/cloud") and health.empty:
                if self.known_empty_map:
                    # The run declared the operating region known-empty, which
                    # is exactly what makes an (observed) empty cloud legal.
                    # Without that declaration an empty cloud is still not
                    # evidence of an empty environment and stays invalid.
                    known_empty_sensing.append(topic)
                    continue
                invalid.append(topic)
                invalid_reasons.append("{}: empty local cloud".format(topic))
                continue
            if not health.valid:
                invalid.append(topic)
                invalid_reasons.append("{}: {}".format(topic, health.detail or "invalid"))
        missing_topics = tuple(missing)
        stale_topics = tuple(stale)
        invalid_inputs = tuple(invalid)
        topic_exists = not missing_topics or all(
            health.exists for health in self.topics.values())
        # "message_received" means every advertised topic has delivered once.
        messages_received = all(health.message_received for health in self.topics.values())
        inputs_valid = not invalid_inputs
        planner_healthy = self.planner_health == "OK"
        reasons: List[str] = []
        if missing_topics:
            reasons.append("missing topics: " + ", ".join(missing_topics))
        if stale_topics:
            reasons.append("stale topics: " + ", ".join(stale_topics))
        if invalid_inputs:
            reasons.append("invalid inputs: " + ", ".join(invalid_inputs))
        if not planner_healthy:
            reasons.append("planner health {}".format(self.planner_health))
        ready = not reasons
        return ReadinessStatus(
            ready=ready,
            reason="ready" if ready else "; ".join(reasons),
            sensor_backend=self.sensor_backend,
            missing_topics=missing_topics,
            stale_topics=stale_topics,
            invalid_inputs=invalid_inputs,
            invalid_reasons=tuple(invalid_reasons),
            planner_health=self.planner_health,
            map_state=self.map_state,
            known_empty_map=self.known_empty_map,
            topic_exists=topic_exists,
            messages_received=messages_received,
            inputs_valid=inputs_valid,
            known_empty_sensing=tuple(known_empty_sensing),
        )


RUNTIME_HEALTH_ITEMS = (
    "global_map_initialized",
    "local_cloud_fresh",
    "odometry_fresh",
    "planner_healthy",
    "qn_state_updated",
)


class RuntimeHealthMonitor:
    """Continuous run-time health checks shared by readiness and execution."""

    def __init__(self, evaluator: ReadinessEvaluator, *,
                 odometry_timeout_s: float = 0.25) -> None:
        self.evaluator = evaluator
        self.odometry_timeout_s = float(odometry_timeout_s)

    def evaluate(self, now: float) -> Dict[str, object]:
        status = self.evaluator.evaluate(now)
        odometry_stale = [topic for topic in status.stale_topics
                          if topic.endswith("/odometry")]
        return {
            "global_map_initialized": status.map_state in ("POPULATED", "KNOWN_EMPTY"),
            "known_empty_sensing": list(status.known_empty_sensing),
            "local_cloud_fresh": not [
                topic for topic in status.stale_topics + status.missing_topics
                if topic.endswith("pcl_render_node/cloud")
                or topic.endswith("pcl_render_node/depth")],
            "odometry_fresh": not odometry_stale and not [
                topic for topic in status.missing_topics if topic.endswith("/odometry")],
            "planner_healthy": status.planner_health == "OK",
            "qn_state_updated": not [
                topic for topic in status.stale_topics + status.missing_topics
                if topic.endswith("/diagnostics")],
            "stale_odometry_topics": odometry_stale,
            "reason": status.reason,
        }
