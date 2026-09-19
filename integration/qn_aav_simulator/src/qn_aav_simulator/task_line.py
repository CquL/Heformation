"""Existing request/planner join and one staged retest.

Formation geometry is diagnostic using Swarm's own normalized graph metric.
Motion success comes from the native Action result; no invented task corridor,
heading or similarity threshold is a business acceptance gate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .monitoring_request import (
    MonitoringRequest, ObservationRequirement, ObservationTask, SurveyRegion,
    InterestPoint, expand,
)
from .observation_coverage import CoverageResult

Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class FormationPhase:
    """A declared group transfer; motion completion comes from native Action."""

    phase_id: str
    path_start: Vector3
    path_end: Vector3
    interest_point_ids: Tuple[str, ...]
    required_agent_count: int = 3


def formation_similarity(positions: Mapping[str, Vector3],
                         slots: Mapping[str, Vector3]) -> float:
    """SwarmGraph::calcMatrices/calcFNorm2, squared Frobenius difference.

    Source: upstream/Swarm-Formation/src/planner/swarm_graph/src/swarm_graph.cpp.
    Translation, rotation and uniform scale invariant. Dimensionless diagnostic;
    no calibrated business threshold has been established.
    """
    members = sorted(slots)
    if len(members) < 2 or set(positions) != set(slots):
        raise ValueError("all declared formation members must be present")

    def laplacian(points):
        values = [points[m] for m in members]
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in values):
            raise ValueError("formation positions must be finite 3-vectors")
        adjacency = [[sum((x-y)**2 for x, y in zip(a, b)) for b in values] for a in values]
        degrees = [sum(row) for row in adjacency]
        if any(d <= 0 for d in degrees):
            raise ValueError("degenerate formation graph")
        return [[1.0 if i == j else -adjacency[i][j] / math.sqrt(degrees[i]*degrees[j])
                 for j in range(len(members))] for i in range(len(members))]

    actual, desired = laplacian(positions), laplacian(slots)
    return sum((a-b)**2 for row, wanted in zip(actual, desired) for a, b in zip(row, wanted))


def formation_interval_metrics(samples, slots, *, sample_timeout_s):
    """Aggregate every aligned sample; missing/invalid data are not zero error.

    Caller uses the Action's existing alignment/ledger and freshness limits.
    This reports measurements, never a shape/task PASS.
    """
    if len(samples) < 2:
        raise ValueError("at least two aligned formation samples are required")
    previous, errors = None, []
    for stamp, positions in samples:
        if not math.isfinite(stamp):
            raise ValueError("formation sample time must be finite")
        if previous is not None and not 0 < stamp - previous <= sample_timeout_s:
            raise ValueError("formation sample times are unordered or missing")
        errors.append(formation_similarity(positions, slots))
        previous = stamp
    return {"metric": "Swarm normalized Laplacian squared Frobenius difference",
            "sample_count": len(errors), "max_similarity_error": max(errors),
            "mean_similarity_error": sum(errors) / len(errors),
            "business_shape_verdict": "NOT_DEFINED"}


def request_centres(tasks: Sequence[ObservationTask]) -> Dict[str, Vector3]:
    """Named travel references, one per observation task.

    The executor planner travels between named references, so every observation
    position the request produced has to be nameable.  Naming them by task id
    keeps the plan and the request aligned without inventing extra references.
    """
    return {task.task_id: task.target for task in tasks}


def to_plan_tasks(tasks: Sequence[ObservationTask]):
    """Convert observation tasks into planner tasks.

    Imported lazily so this module stays usable without the planner package.
    """
    from mrta_python import Task

    return tuple(Task(task.task_id, task.required_capabilities, 1,
                      task.service_time_s, task.deadline_s, task.task_id,
                      allow_larger_unit=task.allow_larger_unit)
                 for task in tasks)


def retest_tasks(request: MonitoringRequest,
                 tasks: Sequence[ObservationTask],
                 coverage: CoverageResult,
                 weights: Mapping[str, float],
                 *,
                 delivery_recorded: bool,
                 already_retested: bool) -> Tuple[ObservationTask, ...]:
    """Observation tasks for the points that were not covered, at most once.

    Release is staged on purpose: nothing is produced until the observation
    result has been *received*, because a retest scheduled before the data
    arrived would be a task waiting for an event a later task has to create.
    A second round is never generated: ``already_retested`` ends it.
    """
    if already_retested:
        return ()
    if not delivery_recorded:
        # The result of the first pass has not arrived, so there is nothing to
        # decide a retest from yet.  This is a wait, not a failure.
        return ()
    missing = set(coverage.uncovered(weights))
    if not missing:
        return ()
    by_region: Dict[str, List[InterestPoint]] = {}
    for region in request.regions:
        for point in region.interest_points:
            if point.point_id in missing:
                by_region.setdefault(region.region_id, []).append(point)
    retests: List[ObservationTask] = []
    for region_id in sorted(by_region):
        region = next(r for r in request.regions if r.region_id == region_id)
        narrowed = SurveyRegion(region.region_id, region.kind, region.corner_a,
                                region.corner_b, tuple(by_region[region_id]))
        for index, task in enumerate(expand(MonitoringRequest(
                request_id="{}-retest".format(request.request_id),
                regions=(narrowed,), requirement=request.requirement,
                required_capabilities=request.required_capabilities,
                service_time_s=request.service_time_s,
                deadline_s=request.deadline_s,
                delivery_required=request.delivery_required))):
            retests.append(ObservationTask(
                task_id="retest-{}-{}".format(region_id, index),
                target=task.target, covers=task.covers, region_id=region_id,
                service_time_s=task.service_time_s, deadline_s=task.deadline_s,
                required_capabilities=task.required_capabilities))
    return tuple(retests)


def load_formation_phase(path: Path) -> Optional[FormationPhase]:
    """Read the optional group stage from the same request file."""
    import yaml

    raw = yaml.safe_load(Path(path).read_text())
    entry = (raw or {}).get("formation_phase")
    if entry is None:
        return None
    allowed = {"phase_id", "path_start", "path_end", "interest_point_ids", "required_agent_count"}
    if set(entry) - allowed:
        raise ValueError("unsupported formation fields: " + str(sorted(set(entry) - allowed)))
    request = load_request(path)
    point_ids = {p.point_id for r in request.regions for p in r.interest_points}
    if not set(entry.get("interest_point_ids", ())) <= point_ids:
        raise ValueError("formation phase references undefined interest points")
    if entry.get("interest_point_ids"):
        raise ValueError("formation observation is not configured; this phase supports transfer only")
    for key in ("path_start", "path_end"):
        if len(entry[key]) != 3 or not all(math.isfinite(x) for x in entry[key]):
            raise ValueError("formation endpoints must be finite 3-vectors")
    if entry.get("required_agent_count", 3) != 3:
        raise ValueError("this online formation phase requires three members")
    return FormationPhase(
        phase_id=str(entry["phase_id"]),
        path_start=tuple(entry["path_start"]),
        path_end=tuple(entry["path_end"]),
        interest_point_ids=tuple(entry["interest_point_ids"]),
        required_agent_count=int(entry.get("required_agent_count", 3)))


def load_request(path: Path) -> MonitoringRequest:
    """Read an operator request from YAML and validate it.

    Missing fields are an error rather than a default: a request that silently
    acquires a service time or a deadline is a request the operator did not make.
    """
    import yaml

    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise ValueError("a monitoring request must be a mapping")
    for field in ("request_id", "requirement", "regions", "required_capabilities",
                  "service_time_s", "deadline_s"):
        if field not in raw:
            raise ValueError("monitoring request is missing '{}'".format(field))
    requirement = ObservationRequirement(**raw["requirement"])
    regions = []
    for entry in raw["regions"]:
        points = tuple(InterestPoint(str(p["point_id"]), tuple(p["position"]),
                                     float(p.get("weight", 1.0)))
                       for p in entry.get("interest_points", ()))
        regions.append(SurveyRegion(str(entry["region_id"]), str(entry["kind"]).upper(),
                                    tuple(entry["corner_a"]), tuple(entry["corner_b"]),
                                    points))
    request = MonitoringRequest(
        request_id=str(raw["request_id"]), regions=tuple(regions),
        requirement=requirement,
        required_capabilities=frozenset(raw["required_capabilities"]),
        service_time_s=float(raw["service_time_s"]),
        deadline_s=float(raw["deadline_s"]),
        delivery_required=bool(raw.get("delivery_required", True)),
        requires_underwater=bool(raw.get("requires_underwater", False)),
        requires_relay_delivery=bool(raw.get("requires_relay_delivery", False)))

    from .monitoring_request import validate_request
    validate_request(request)
    return request
