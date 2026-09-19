"""Was the interest point actually observed, and was the result delivered?

Two questions, deliberately answered separately.  CARIC scores inspection per
interest point as ``q = q_seen * q_blur * q_res`` and takes the best score any
platform achieved, so "the vehicle was nearby" is not an observation.  The plan
adds two more rules: the dwell must be satisfied continuously by one member, and
``C_delivered`` requires the result to have been *received*, which is a different
event from having been captured.

Everything here is a declared engineering proxy.  The resolution and blur terms
are linear decays with declared bounds rather than camera models, and the line of
sight test uses the declared obstacle geometry.  None of it claims a payload has
been validated.
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
    speed_mps: float


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
    score: float
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


def _q_seen(member: Vector3, point: Vector3, footprint_radius_m: float,
            obstacles: Sequence[ObstacleBox]) -> bool:
    horizontal = math.dist((member[0], member[1], 0.0), (point[0], point[1], 0.0))
    if horizontal > footprint_radius_m:
        return False
    for box in obstacles:
        if box.blocks(member, point):
            return False
    return True


def _q_blur(speed_mps: float, requirement) -> float:
    """Motion blur proxy: image-plane movement during the exposure.

    Approximated by the distance the member travels in the exposure window
    relative to a declared tolerance, because the projection would need a camera
    model this project does not have.  Stated as a proxy, not as the CARIC term.
    """
    moved = speed_mps * requirement.exposure_s
    if moved <= 0:
        return 1.0
    return max(0.0, min(1.0, requirement.blur_tolerance_m / moved))


def _q_res(distance_m: float, requirement) -> float:
    """Spatial resolution proxy anchored on the observation geometry.

    Full score at (or inside) the declared nominal standoff, decaying linearly to
    zero at the outer distance bound.  This is a declared proxy: it says the
    image is best when the vehicle observes from its nominal standoff and worse as
    it stands further off, without pretending to model a camera.
    """
    nominal = requirement.nominal_standoff_m
    outer = requirement.max_distance_from_altitude_m
    if outer <= nominal:
        raise ValueError("max_distance_from_altitude_m must exceed nominal_standoff_m")
    if distance_m <= nominal:
        return 1.0
    if distance_m >= outer:
        return 0.0
    return (outer - distance_m) / (outer - nominal)


def evaluate_coverage(samples: Sequence[ObservationSample],
                      interest_points: Mapping[str, Vector3],
                      requirement,
                      obstacles: Sequence[ObstacleBox] = (),
                      *,
                      sample_timeout_s: float = 0.5) -> CoverageResult:
    """Per-point best score, with the dwell enforced on one member.

    The dwell is not "how long any member was near": it is the longest continuous
    run during which a *single* member kept the point inside the footprint with an
    unblocked line of sight.  A break in the condition, or a gap longer than
    ``sample_timeout_s`` in that member's samples, ends the run and the count
    restarts, so three members contributing 0.4 s each cannot make 1.2 s.
    """
    by_member: Dict[str, list] = {}
    for sample in samples:
        by_member.setdefault(sample.member_id, []).append(sample)
    for rows in by_member.values():
        rows.sort(key=lambda row: row.time_s)

    result = CoverageResult()
    for point_id, point in interest_points.items():
        best_score = 0.0
        best_member = None
        best_dwell = 0.0
        reason = "no sample put the point in the footprint"
        for member_id, rows in by_member.items():
            run_start = None
            previous = None
            for sample in rows:
                seen = _q_seen(sample.position, point, requirement.footprint_radius_m,
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
                distance = math.dist(sample.position, point)
                score = _q_blur(sample.speed_mps, requirement) * _q_res(distance, requirement)
                if score > best_score:
                    best_score = score
                    best_member = member_id
                    best_dwell = dwell
                    reason = ("observed with score {:.3f} after {:.2f} s of "
                              "continuous dwell".format(score, dwell))
                if best_score >= 1.0:
                    break
        observed = best_score > 0.0
        result.points[point_id] = PointObservation(
            point_id=point_id, observed=observed, score=best_score,
            member_id=best_member, dwell_s=best_dwell, reason=reason)
    return result


def record_delivery(result: CoverageResult, point_ids: Iterable[str]) -> None:
    """Record that the results for these points have been received.

    Separate from observation on purpose.  With an ideal link this is called
    immediately, but it is still a distinct event and is recorded as one.
    """
    result.delivered_point_ids = frozenset(result.delivered_point_ids) | set(point_ids)
