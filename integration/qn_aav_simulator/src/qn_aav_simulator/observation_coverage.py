"""Geometric visibility and continuous dwell on actual samples, then receipt.

This binary surrogate does not establish image quality or payload validity.
No exposure, blur, resolution score or arbitrary positive-score acceptance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ObservationSample:
    """One recorded state of one member during an observation window."""

    member_id: str
    time_s: float
    position: Vector3


@dataclass(frozen=True)
class ObstacleBox:
    centre: Vector3
    size: Vector3

    def blocks(self, start: Vector3, end: Vector3) -> bool:
        """Whether the segment start->end intersects the box (slab test)."""
        low = [self.centre[i] - self.size[i] / 2.0 for i in range(3)]
        high = [self.centre[i] + self.size[i] / 2.0 for i in range(3)]
        t0, t1 = 0.0, 1.0
        for i in range(3):
            direction = end[i] - start[i]
            if abs(direction) < 1e-12:
                if start[i] < low[i] or start[i] > high[i]:
                    return False
                continue
            a = (low[i] - start[i]) / direction
            b = (high[i] - start[i]) / direction
            if a > b:
                a, b = b, a
            t0 = max(t0, a)
            t1 = min(t1, b)
            if t0 > t1:
                return False
        return True


@dataclass(frozen=True)
class PointObservation:
    point_id: str
    observed: bool
    member_id: Optional[str]
    dwell_s: float
    reason: str


@dataclass
class CoverageResult:
    points: Dict[str, PointObservation] = field(default_factory=dict)
    delivered_point_ids: frozenset = frozenset()

    def observed_fraction(self, weights: Mapping[str, float]) -> float:
        total = sum(weights.values())
        if total <= 0:
            raise ValueError("weights must sum to a positive value")
        return sum(weight for point_id, weight in weights.items()
                   if self.points.get(point_id) and self.points[point_id].observed) / total

    def delivered_fraction(self, weights: Mapping[str, float]) -> float:
        """``C_delivered``: observed AND the result has been received.

        The invariant lives here, not only in the caller.  Recording a point as
        delivered cannot raise this fraction on its own: a receipt for something
        that was never observed is not a delivered observation.  Zero latency
        does not merge the two events either, it only makes the receipt immediate.
        """
        total = sum(weights.values())
        if total <= 0:
            raise ValueError("weights must sum to a positive value")
        return sum(weight for point_id, weight in weights.items()
                   if point_id in self.delivered_point_ids
                   and self.points.get(point_id) is not None
                   and self.points[point_id].observed) / total

    def delivered_point_observations(self, weights: Mapping[str, float]) -> Tuple[str, ...]:
        """Points that count towards ``C_delivered``, for reporting."""
        return tuple(sorted(point_id for point_id in weights
                            if point_id in self.delivered_point_ids
                            and self.points.get(point_id) is not None
                            and self.points[point_id].observed))

    def uncovered(self, weights: Mapping[str, float]) -> Tuple[str, ...]:
        return tuple(sorted(point_id for point_id in weights
                            if not (self.points.get(point_id)
                                    and self.points[point_id].observed)))

    def undelivered(self, weights: Mapping[str, float]) -> Tuple[str, ...]:
        return tuple(sorted(point_id for point_id in weights
                            if point_id not in self.delivered_point_ids))


def _visible(member: Vector3, point: Vector3, requirement,
            obstacles: Sequence[ObstacleBox]) -> bool:
    horizontal = math.dist((member[0], member[1], 0.0), (point[0], point[1], 0.0))
    if (horizontal > requirement.footprint_radius_m
            or abs(member[2] - point[2]) > requirement.max_distance_from_altitude_m):
        return False
    for box in obstacles:
        if box.blocks(member, point):
            return False
    return True


def evaluate_coverage(samples: Sequence[ObservationSample],
                      interest_points: Mapping[str, Vector3],
                      requirement,
                      obstacles: Sequence[ObstacleBox] = (),
                      *,
                      sample_timeout_s: float = 0.5) -> CoverageResult:
    """Binary geometric visibility, with the dwell enforced on one member.

    The dwell is not "how long any member was near": it is the longest continuous
    run during which a *single* member kept the point inside the footprint with an
    unblocked line of sight.  A break in the condition, or a gap longer than
    ``sample_timeout_s`` in that member's samples, ends the run and the count
    restarts, so three members contributing 0.4 s each cannot make 1.2 s.
    """
    if not math.isfinite(sample_timeout_s) or sample_timeout_s <= 0:
        raise ValueError("sample_timeout_s must be finite and positive")
    by_member: Dict[str, list] = {}
    for sample in samples:
        if (not math.isfinite(sample.time_s) or len(sample.position) != 3
                or not all(math.isfinite(x) for x in sample.position)):
            raise ValueError("observation samples must be finite")
        rows = by_member.setdefault(sample.member_id, [])
        if rows and sample.time_s <= rows[-1].time_s:
            raise ValueError("member sample times must strictly increase")
        rows.append(sample)

    result = CoverageResult()
    for point_id, point in interest_points.items():
        observed = False
        best_member = None
        best_dwell = 0.0
        reason = "no sample put the point in the footprint"
        for member_id, rows in by_member.items():
            run_start = None
            previous = None
            for sample in rows:
                seen = _visible(sample.position, point, requirement,
                               obstacles)
                if previous is not None and sample.time_s - previous.time_s > sample_timeout_s:
                    run_start = None          # stale: the run does not continue
                previous = sample
                if not seen:
                    run_start = None
                    continue
                if run_start is None:
                    run_start = sample.time_s
                dwell = sample.time_s - run_start
                if dwell + 1e-9 < requirement.min_dwell_s:
                    continue
                observed = True
                best_member, best_dwell = member_id, dwell
                reason = "geometric visibility and continuous dwell; image quality unverified"
                break
            if observed:
                break
        result.points[point_id] = PointObservation(
            point_id=point_id, observed=observed,
            member_id=best_member, dwell_s=best_dwell, reason=reason)
    return result


def record_delivery(result: CoverageResult, point_ids: Iterable[str]) -> None:
    """Record that the results for these points have been received.

    Separate from observation on purpose.  With an ideal link this is called
    immediately, but it is still a distinct event and is recorded as one.
    """
    result.delivered_point_ids = frozenset(result.delivered_point_ids) | set(point_ids)
