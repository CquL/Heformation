"""From a monitoring request to an executable plan, and back from results.

This is the join between the request layer (``monitoring_request``), the coverage
verdict (``observation_coverage``) and the executor planner
(``mrta_python.executors``).  It is deliberately ROS-free so the whole task line
can be exercised without a simulator.

Two rules carried from the literature shape the flow:

* a retest is **released in stages** - it is created only after the observation
  result has been *received*, not merely produced (CoCoPlan: the temporal
  relation is known only when the information arrives; the plan is explicit that
  the runner must not wait at the head of the queue for data that a later task
  will produce);
* a formation phase is complete only when the **shape** was held and the points
  were observed.  ``required_agent_count = 3`` is not evidence that a formation
  happened (Swarm-Formation ICRA: the formation is a cost term, so only the
  measured shape error shows it participated).
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
    """A stage that must be flown as a group and observed as a group."""

    phase_id: str
    path_start: Vector3
    path_end: Vector3
    interest_point_ids: Tuple[str, ...]
    shape_tolerance_m: float
    required_agent_count: int = 3


@dataclass
class FormationVerdict:
    complete: bool
    max_shape_error_m: Optional[float]
    reasons: Tuple[str, ...]


def formation_shape_error(positions: Mapping[str, Vector3],
                          slots: Mapping[str, Vector3],
                          centre: Vector3,
                          scale: float) -> float:
    """Largest deviation of the actual formation from the commanded one.

    Compares pairwise differences between members - the shape - rather than each
    member against its nominal slot, so a formation that is translated but not
    deformed is not counted as an error.  This is the quantity the plan asks for
    and the only honest evidence that a formation was flown.
    """
    members = sorted(positions)
    if len(members) < 2:
        raise ValueError("a formation needs at least two members")
    worst = 0.0
    for index, first in enumerate(members):
        for second in members[index + 1:]:
            actual = math.dist(positions[first], positions[second])
            first_slot, second_slot = slots.get(first), slots.get(second)
            if first_slot is None or second_slot is None:
                raise ValueError("missing slot for member {}".format(
                    first if first_slot is None else second))
            wanted = scale * math.dist(
                tuple(centre[i] + first_slot[i] for i in range(3)),
                tuple(centre[i] + second_slot[i] for i in range(3)))
            worst = max(worst, abs(actual - wanted))
    return worst


def evaluate_formation_phase(phase: FormationPhase,
                             positions: Mapping[str, Vector3],
                             slots: Mapping[str, Vector3],
                             centre: Vector3,
                             scale: float,
                             coverage: CoverageResult,
                             weights: Mapping[str, float]) -> FormationVerdict:
    """A formation phase needs the shape held AND its points observed."""
    reasons: List[str] = []
    error: Optional[float] = None
    if len(positions) < phase.required_agent_count:
        reasons.append(
            "{} members present, {} required".format(len(positions),
                                                     phase.required_agent_count))
    else:
        error = formation_shape_error(positions, slots, centre, scale)
        if error > phase.shape_tolerance_m:
            reasons.append("formation shape error {:.3f} m exceeds {:.3f} m".format(
                error, phase.shape_tolerance_m))
    for point_id in phase.interest_point_ids:
        observation = coverage.points.get(point_id)
        if observation is None or not observation.observed:
            reasons.append("interest point {} was not observed".format(point_id))
    return FormationVerdict(complete=not reasons, max_shape_error_m=error,
                            reasons=tuple(reasons))


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
    return FormationPhase(
        phase_id=str(entry["phase_id"]),
        path_start=tuple(entry["path_start"]),
        path_end=tuple(entry["path_end"]),
        interest_point_ids=tuple(entry["interest_point_ids"]),
        shape_tolerance_m=float(entry["shape_tolerance_m"]),
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
    return MonitoringRequest(
        request_id=str(raw["request_id"]), regions=tuple(regions),
        requirement=requirement,
        required_capabilities=frozenset(raw["required_capabilities"]),
        service_time_s=float(raw["service_time_s"]),
        deadline_s=float(raw["deadline_s"]),
        delivery_required=bool(raw.get("delivery_required", True)),
        requires_underwater=bool(raw.get("requires_underwater", False)),
        requires_relay_delivery=bool(raw.get("requires_relay_delivery", False)))
