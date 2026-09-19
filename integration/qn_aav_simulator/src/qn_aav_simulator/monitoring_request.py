"""Declared geometric monitoring surrogate; no calibrated camera/payload model.

Greedy footprint set cover is a project integration choice, not CARIC's planner.
Lengths and dwell times are scenario inputs, not literature-derived thresholds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

SURFACE = "SURFACE"
SHORELINE = "SHORELINE"
UNDERWATER = "UNDERWATER"
KINDS = (SURFACE, SHORELINE, UNDERWATER)

Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class InterestPoint:
    point_id: str
    position: Vector3
    weight: float = 1.0


@dataclass(frozen=True)
class SurveyRegion:
    region_id: str
    kind: str
    corner_a: Vector3
    corner_b: Vector3
    interest_points: Tuple[InterestPoint, ...]


@dataclass(frozen=True)
class ObservationRequirement:
    """What counts as a valid observation.  Declared, not inferred."""

    footprint_radius_m: float
    min_dwell_s: float
    cruise_altitude_m: float
    max_distance_from_altitude_m: float = 3.0

    def __post_init__(self):
        for name in ("footprint_radius_m", "min_dwell_s", "max_distance_from_altitude_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        if not math.isfinite(self.cruise_altitude_m):
            raise ValueError("cruise_altitude_m must be finite")



@dataclass(frozen=True)
class MonitoringRequest:
    request_id: str
    regions: Tuple[SurveyRegion, ...]
    requirement: ObservationRequirement
    required_capabilities: frozenset
    service_time_s: float
    deadline_s: float
    #: result must be received, not merely captured
    delivery_required: bool = True
    #: the request explicitly demands underwater work or a USV relay
    requires_underwater: bool = False
    requires_relay_delivery: bool = False


@dataclass(frozen=True)
class ObservationTask:
    """One dispatchable observation: a position that covers a set of points."""

    task_id: str
    target: Vector3
    covers: Tuple[str, ...]
    region_id: str
    service_time_s: float
    deadline_s: float
    required_capabilities: frozenset
    #: observation tasks never want a larger unit unless they say so
    allow_larger_unit: bool = False


class UnsupportedRequirement(ValueError):
    """The request asks for something no configured platform can do online."""


def _distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def unsupported_reasons(request: MonitoringRequest,
                        online_capabilities: Iterable[str]) -> Tuple[str, ...]:
    """Reasons this request cannot be fully executed online, if any."""
    available = {str(value).upper() for value in online_capabilities}
    reasons: List[str] = []
    if request.requires_underwater or any(r.kind == UNDERWATER for r in request.regions):
        reasons.append("UNDERWATER observation has no supported online expansion/execution adapter")
    if request.requires_relay_delivery:
        reasons.append("delivery via a SURFACE relay has no supported online delivery adapter")
    for region in request.regions:
        if region.kind not in KINDS:
            reasons.append("unknown region kind {} for {}".format(region.kind, region.region_id))
            continue
        if region.kind == UNDERWATER and UNDERWATER not in available:
            continue  # already reported once
        if region.kind in (SURFACE, SHORELINE) and "AIR" not in available:
            reasons.append(
                "region {} needs AIR observation but no AIR endpoint is configured".format(
                    region.region_id))
    return tuple(reasons)


def _observable_from(point: InterestPoint, position: Vector3,
                     requirement: ObservationRequirement) -> bool:
    """Whether a position is within the declared footprint of a point.

    The footprint is a horizontal disc at the fixed cruise altitude: the plan
    states this explicitly, so vertical range is checked separately rather than
    being folded into a 3-D distance.
    """
    horizontal = _distance((position[0], position[1], 0.0),
                           (point.position[0], point.position[1], 0.0))
    vertical = abs(point.position[2] - position[2])
    return (horizontal <= requirement.footprint_radius_m
            and vertical <= requirement.max_distance_from_altitude_m)


def observation_position(anchor: Vector3, requirement: ObservationRequirement) -> Vector3:
    """Where the vehicle actually goes to look at ``anchor``.

    The vehicle flies at the declared cruise altitude and looks down, so an
    observation position is vertically above the interest point rather than at
    the point itself.  This matters: an interest point below the observation band
    then has no flyable position above it, which is exactly the case the
    expansion must refuse instead of pretending to cover.
    """
    return (anchor[0], anchor[1], requirement.cruise_altitude_m)


def observation_candidates(region: SurveyRegion,
                           requirement: ObservationRequirement) -> Dict[str, Tuple[str, ...]]:
    """Observation positions worth flying, and the points each would cover.

    Each interest point contributes one observation position - directly above it
    at the cruise altitude - and that position may cover neighbouring points when
    the footprint reaches.  Nothing is invented between the declared points, so
    every waypoint the system flies is one the request asked about.
    """
    candidates: Dict[str, Tuple[str, ...]] = {}
    for anchor in region.interest_points:
        position = observation_position(anchor.position, requirement)
        covered = tuple(point.point_id for point in region.interest_points
                        if _observable_from(point, position, requirement))
        candidates[anchor.point_id] = covered
    return candidates


def expand(request: MonitoringRequest,
           online_capabilities: Iterable[str] = ("AIR",)) -> Tuple[ObservationTask, ...]:
    """Turn a request into observation tasks, or refuse it.

    Raises UnsupportedRequirement when part of the request cannot be executed by
    the configured platforms: it must be reported, not quietly omitted, because a
    request with an unexecutable part cannot be reported as complete.
    """
    validate_request(request)
    reasons = unsupported_reasons(request, online_capabilities)
    if reasons:
        raise UnsupportedRequirement("; ".join(reasons))
    tasks: List[ObservationTask] = []
    for region in request.regions:
        if region.kind == UNDERWATER:
            # Reachable only when an UNDERWATER endpoint exists, which the
            # reasons check above has already established for this version.
            continue
        remaining = {point.point_id: point for point in region.interest_points}
        candidates = observation_candidates(region, request.requirement)
        while remaining:
            # Greedy set cover: take the candidate that covers the most points
            # still unobserved, ties broken by id so the expansion is stable.
            best = None
            for point_id in sorted(candidates):
                covered = set(candidates[point_id]) & set(remaining)
                if not covered:
                    continue
                if best is None or len(covered) > len(best[1]):
                    best = (point_id, covered)
            if best is None:
                raise UnsupportedRequirement(
                    "region {} has interest points that no declared footprint can "
                    "cover: {}".format(region.region_id, sorted(remaining)))
            anchor_id, covered = best
            anchor = next(p for p in region.interest_points if p.point_id == anchor_id)
            tasks.append(ObservationTask(
                task_id="{}-obs{}".format(region.region_id, len(tasks)),
                target=observation_position(anchor.position, request.requirement),
                covers=tuple(sorted(covered)),
                region_id=region.region_id,
                service_time_s=request.service_time_s,
                deadline_s=request.deadline_s,
                required_capabilities=frozenset(request.required_capabilities),
            ))
            for point_id in covered:
                remaining.pop(point_id, None)
    if not tasks:
        raise UnsupportedRequirement("the request expands to no observable task")
    return tuple(tasks)


def validate_request(request: MonitoringRequest) -> None:
    """Reject unusable geometry/timing before any dispatch or division."""
    if not request.request_id or not request.regions or not request.required_capabilities:
        raise ValueError("request id, regions and capabilities must be nonempty")
    if (not math.isfinite(request.service_time_s)
            or request.service_time_s < request.requirement.min_dwell_s):
        raise ValueError("service_time_s must be finite and at least min_dwell_s")
    if not math.isfinite(request.deadline_s) or request.deadline_s <= 0:
        raise ValueError("deadline_s must be finite and positive")
    region_ids, point_ids = set(), set()
    for region in request.regions:
        if not region.region_id or region.region_id in region_ids or not region.interest_points:
            raise ValueError("region ids must be unique and regions nonempty")
        region_ids.add(region.region_id)
        for vector in (region.corner_a, region.corner_b) + tuple(p.position for p in region.interest_points):
            if len(vector) != 3 or not all(math.isfinite(x) for x in vector):
                raise ValueError("coordinates must be finite 3-vectors")
        for point in region.interest_points:
            if not point.point_id or point.point_id in point_ids:
                raise ValueError("point ids must be globally unique")
            point_ids.add(point.point_id)
            if not math.isfinite(point.weight) or point.weight <= 0:
                raise ValueError("point weights must be finite and positive")
