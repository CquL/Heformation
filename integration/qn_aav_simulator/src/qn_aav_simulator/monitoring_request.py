"""High-level monitoring request and its expansion into observation tasks.

The user supplies where to look, what counts as a valid observation, by when and
whether the result must be delivered.  They do not supply task lists, service
times or per-platform parameters; those are derived here.

The expansion is the part that must not be arbitrary.  CARIC scores inspection
per interest point with a footprint and a line-of-sight test, and the plan says
the number of waypoints is derived from the coverage requirement rather than from
the number of interest points.  So the expansion clusters interest points that a
single observation position can cover - a greedy set cover over the footprint
relation - and emits one task per cluster.  Two points inside one footprint
produce one task, not two.

A request that asks for something no online platform can do (underwater work, or
delivery via a surface relay) is rejected here rather than silently dropped.
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
    #: exposure time used by the blur term; blur is judged on the distance the
    #: point moves across the image plane during it (CARIC's q_blur).
    exposure_s: float = 0.02
    #: how far the member may travel during the exposure before the score decays
    blur_tolerance_m: float = 0.05
    #: the standoff at which resolution is taken to be fully satisfactory, and
    #: the distance at which it reaches zero.  A declared proxy for the resolution
    #: term, anchored on the observation geometry rather than on a camera model
    #: this project does not have.
    nominal_standoff_m: float = 0.8
    max_distance_from_altitude_m: float = 3.0


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
        if UNDERWATER not in available:
            reasons.append(
                "underwater observation was requested but no platform with an "
                "UNDERWATER execution endpoint is configured")
    if request.requires_relay_delivery and SURFACE not in available:
        reasons.append(
            "delivery via a surface relay was requested but no platform with a "
            "SURFACE execution endpoint is configured")
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
