"""Task outcome, safety outcome and experiment validity are separate results.

plan.md P1 requires three independent verdicts:

    task_outcome        = PASS | FAIL
    safety_outcome      = PASS | FAIL | NOT_VERIFIED
    experiment_validity = VALID | INVALID | INCOMPLETE

Missing or misaligned samples are reported through an explicit ledger; they are
never simply dropped and then described as a clean statistic.  A collision is a
safety failure even when the formation reaches its slots, and a discrete-point
clearance check is labelled as discrete sampling rather than a continuous-time
guarantee.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import combinations
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Vector3 = Tuple[float, float, float]

TASK_PASS = "PASS"
TASK_FAIL = "FAIL"
SAFETY_PASS = "PASS"
SAFETY_FAIL = "FAIL"
SAFETY_NOT_VERIFIED = "NOT_VERIFIED"
VALIDITY_VALID = "VALID"
VALIDITY_INVALID = "INVALID"
VALIDITY_INCOMPLETE = "INCOMPLETE"

DISCRETE_SAMPLED = "DISCRETE_SAMPLED"
CONTINUOUS = "CONTINUOUS"

_MIN_INTER_AGENT_CLEARANCE_M = 0.5
_OBSTACLE_CLEARANCE_M = 0.2


def _norm(value: Sequence[float]) -> float:
    return math.sqrt(sum(component * component for component in value))


@dataclass(frozen=True)
class MemberSample:
    """One synchronized per-member observation on the common ROS grid."""

    agent_id: int
    ros_time_s: float
    position: Vector3
    world_velocity: Vector3
    used_reference_position: Optional[Vector3] = None
    used_reference_velocity: Optional[Vector3] = None
    target_position: Optional[Vector3] = None
    model_time_s: Optional[float] = None


@dataclass
class SampleLedger:
    expected_sample_count: int = 0
    valid_sample_count: int = 0
    alignment_failure_count: int = 0
    max_continuous_gap_s: float = 0.0
    agent_expected_counts: Dict[int, int] = field(default_factory=dict)
    agent_valid_counts: Dict[int, int] = field(default_factory=dict)
    agent_max_gaps_s: Dict[int, float] = field(default_factory=dict)

    @property
    def valid_sample_ratio(self) -> float:
        if self.expected_sample_count == 0:
            return 0.0
        return self.valid_sample_count / float(self.expected_sample_count)

    def as_dict(self) -> Dict[str, object]:
        return {
            "expected_sample_count": self.expected_sample_count,
            "valid_sample_count": self.valid_sample_count,
            "valid_sample_ratio": self.valid_sample_ratio,
            "alignment_failure_count": self.alignment_failure_count,
            "max_continuous_gap_s": self.max_continuous_gap_s,
            "agent_expected_counts": {str(k): v for k, v in
                                      sorted(self.agent_expected_counts.items())},
            "agent_valid_counts": {str(k): v for k, v in
                                   sorted(self.agent_valid_counts.items())},
            "agent_max_gaps_s": {str(k): v for k, v in
                                 sorted(self.agent_max_gaps_s.items())},
        }


def build_ledger(agent_ids: Iterable[int], grid_times: Sequence[float],
                 samples: Dict[int, Sequence[MemberSample]],
                 *, expected_period_s: float) -> SampleLedger:
    """Account for every expected sample of every member."""
    agent_ids = tuple(agent_ids)
    ledger = SampleLedger()
    for agent_id in agent_ids:
        ledger.agent_expected_counts[agent_id] = len(grid_times)
        ledger.agent_valid_counts[agent_id] = 0
        ledger.agent_max_gaps_s[agent_id] = 0.0
    ledger.expected_sample_count = len(grid_times) * len(agent_ids)
    for agent_id in agent_ids:
        series = sorted(samples.get(agent_id, ()), key=lambda item: item.ros_time_s)
        valid_times: List[float] = []
        for expected in grid_times:
            nearest = None
            best = None
            for sample in series:
                distance = abs(sample.ros_time_s - expected)
                if best is None or distance < best:
                    best, nearest = distance, sample
            if nearest is None or best > expected_period_s:
                ledger.alignment_failure_count += 1
                continue
            ledger.valid_sample_count += 1
            ledger.agent_valid_counts[agent_id] += 1
            valid_times.append(nearest.ros_time_s)
        for first, second in zip(valid_times, valid_times[1:]):
            gap = second - first
            ledger.agent_max_gaps_s[agent_id] = max(
                ledger.agent_max_gaps_s[agent_id], gap)
        ledger.max_continuous_gap_s = max(
            ledger.max_continuous_gap_s, ledger.agent_max_gaps_s[agent_id],
            *([0.0] if not valid_times else []))
    return ledger


@dataclass
class MetricSet:
    final_slot_error_m: Optional[float] = None
    trajectory_tracking_error_m: Optional[float] = None
    velocity_tracking_error_mps: Optional[float] = None
    hold_velocity_mps: Optional[float] = None
    formation_shape_error_m: Optional[float] = None

    def as_dict(self) -> Dict[str, Optional[float]]:
        return {
            "final_slot_error_m": self.final_slot_error_m,
            "trajectory_tracking_error_m": self.trajectory_tracking_error_m,
            "velocity_tracking_error_mps": self.velocity_tracking_error_mps,
            "hold_velocity_mps": self.hold_velocity_mps,
            "formation_shape_error_m": self.formation_shape_error_m,
        }


def compute_metrics(samples: Dict[int, Sequence[MemberSample]], *,
                    scale: float, relative_slots: Dict[int, Vector3],
                    hold_start_s: Optional[float] = None,
                    hold_end_s: Optional[float] = None) -> MetricSet:
    metrics = MetricSet()
    final_positions: Dict[int, Vector3] = {}
    for agent_id, series in samples.items():
        ordered = sorted(series, key=lambda item: item.ros_time_s)
        if not ordered:
            continue
        last = ordered[-1]
        final_positions[agent_id] = last.position
        if last.target_position is not None:
            error = _norm([last.position[index] - last.target_position[index]
                           for index in range(3)])
            metrics.final_slot_error_m = max(metrics.final_slot_error_m or 0.0, error)
        for sample in ordered:
            if sample.used_reference_position is not None:
                error = _norm([sample.position[index] - sample.used_reference_position[index]
                               for index in range(3)])
                metrics.trajectory_tracking_error_m = max(
                    metrics.trajectory_tracking_error_m or 0.0, error)
            if (sample.used_reference_velocity is not None
                    and sample.used_reference_velocity is not None):
                error = _norm([sample.world_velocity[index]
                               - sample.used_reference_velocity[index] for index in range(3)])
                metrics.velocity_tracking_error_mps = max(
                    metrics.velocity_tracking_error_mps or 0.0, error)
            if (hold_start_s is not None and hold_end_s is not None
                    and hold_start_s <= sample.ros_time_s <= hold_end_s):
                metrics.hold_velocity_mps = max(
                    metrics.hold_velocity_mps or 0.0, _norm(sample.world_velocity))
    for first, second in combinations(sorted(final_positions), 2):
        if first not in relative_slots or second not in relative_slots:
            continue
        expected = [scale * (relative_slots[second][index] - relative_slots[first][index])
                    for index in range(3)]
        actual = [final_positions[second][index] - final_positions[first][index]
                  for index in range(3)]
        error = _norm([actual[index] - expected[index] for index in range(3)])
        metrics.formation_shape_error_m = max(metrics.formation_shape_error_m or 0.0, error)
    return metrics


@dataclass
class SafetyResult:
    outcome: str
    evidence_kind: str
    min_inter_agent_distance_m: Optional[float] = None
    min_obstacle_clearance_m: Optional[float] = None
    required_inter_agent_clearance_m: float = _MIN_INTER_AGENT_CLEARANCE_M
    required_obstacle_clearance_m: float = _OBSTACLE_CLEARANCE_M
    collision_pairs: Tuple[Tuple[int, int], ...] = ()
    reasons: Tuple[str, ...] = ()

    def as_dict(self) -> Dict[str, object]:
        return {
            "outcome": self.outcome,
            "evidence_kind": self.evidence_kind,
            "min_inter_agent_distance_m": self.min_inter_agent_distance_m,
            "min_obstacle_clearance_m": self.min_obstacle_clearance_m,
            "required_inter_agent_clearance_m": self.required_inter_agent_clearance_m,
            "required_obstacle_clearance_m": self.required_obstacle_clearance_m,
            "collision_pairs": [list(pair) for pair in self.collision_pairs],
            "reasons": list(self.reasons),
        }


def evaluate_safety(samples: Dict[int, Sequence[MemberSample]], *,
                    obstacle_clearances: Optional[Sequence[float]] = None,
                    required_inter_agent_clearance_m: float = _MIN_INTER_AGENT_CLEARANCE_M,
                    required_obstacle_clearance_m: float = _OBSTACLE_CLEARANCE_M
                    ) -> SafetyResult:
    """Discrete-sample collision and obstacle-separation check.

    ``obstacle_clearances`` is the set of sampled minimum distances between any
    member and the static global cloud.  ``None`` means the check could not be
    performed and yields ``NOT_VERIFIED`` rather than a silent pass.
    """
    samples_by_time: Dict[float, List[MemberSample]] = {}
    for series in samples.values():
        for sample in series:
            samples_by_time.setdefault(sample.ros_time_s, []).append(sample)
    min_distance: Optional[float] = None
    collisions: List[Tuple[int, int]] = []
    for _time, group in sorted(samples_by_time.items()):
        for first, second in combinations(sorted(group, key=lambda s: s.agent_id), 2):
            distance = math.dist(first.position, second.position)
            min_distance = distance if min_distance is None else min(min_distance, distance)
            if distance < required_inter_agent_clearance_m:
                pair = (first.agent_id, second.agent_id)
                if pair not in collisions:
                    collisions.append(pair)
    min_obstacle = None if obstacle_clearances is None else (
        min(obstacle_clearances) if obstacle_clearances else None)
    reasons: List[str] = []
    if collisions:
        reasons.append("inter-agent distance below {:.2f} m: {}".format(
            required_inter_agent_clearance_m,
            ", ".join("({},{})".format(a, b) for a, b in collisions)))
    if min_obstacle is not None and min_obstacle < required_obstacle_clearance_m:
        reasons.append("obstacle clearance {:.3f} m below {:.2f} m".format(
            min_obstacle, required_obstacle_clearance_m))
    if collisions or reasons:
        outcome = SAFETY_FAIL
    elif min_obstacle is None:
        outcome = SAFETY_NOT_VERIFIED
        reasons.append("no obstacle clearance samples were recorded")
    else:
        outcome = SAFETY_PASS
    return SafetyResult(
        outcome=outcome,
        evidence_kind=DISCRETE_SAMPLED,
        min_inter_agent_distance_m=min_distance,
        min_obstacle_clearance_m=min_obstacle,
        required_inter_agent_clearance_m=required_inter_agent_clearance_m,
        required_obstacle_clearance_m=required_obstacle_clearance_m,
        collision_pairs=tuple(collisions),
        reasons=tuple(reasons),
    )


@dataclass
class ExperimentVerdict:
    task_outcome: str
    safety_outcome: str
    experiment_validity: str
    max_position_error_m: Optional[float]
    hold_duration_required_s: float
    model_hold_satisfied: bool
    adoption_state: str
    time_alignment_ok: bool
    metrics: MetricSet
    ledger: SampleLedger
    safety: SafetyResult
    reasons: Tuple[str, ...] = ()

    def as_dict(self) -> Dict[str, object]:
        return {
            "task_outcome": self.task_outcome,
            "safety_outcome": self.safety_outcome,
            "experiment_validity": self.experiment_validity,
            "max_position_error_m": self.max_position_error_m,
            "hold_duration_required_s": self.hold_duration_required_s,
            "model_hold_satisfied": self.model_hold_satisfied,
            "adoption_state": self.adoption_state,
            "time_alignment_ok": self.time_alignment_ok,
            "metrics": self.metrics.as_dict(),
            "sample_ledger": self.ledger.as_dict(),
            "safety": self.safety.as_dict(),
            "reasons": list(self.reasons),
        }


def decide(*, epsilon_p: float, epsilon_v: float,
           motion_completed: bool, adoption_state: str,
           model_hold_satisfied: bool, time_alignment_ok: bool,
           metrics: MetricSet, ledger: SampleLedger,
           safety: SafetyResult,
           max_position_error_m: Optional[float] = None,
           hold_duration_s: float = 0.0,
           min_valid_sample_ratio: float = 0.9,
           min_safety_sample_ratio: float = 0.9) -> ExperimentVerdict:
    """Combine motion, adoption, model time and sampling into three verdicts."""
    task_reasons: List[str] = []
    if not motion_completed:
        task_reasons.append("seven-member formation did not complete its hold")
    if adoption_state != "ADOPTED":
        task_reasons.append("new reference adoption was not proven by trajectory_id")
    if not model_hold_satisfied:
        task_reasons.append("model-time hold was shorter than the requested duration")
    if not time_alignment_ok:
        task_reasons.append("model time and ROS time were not compatible")
    if metrics.final_slot_error_m is not None and metrics.final_slot_error_m > epsilon_p:
        task_reasons.append("final slot error {:.3f} m exceeds epsilon_p".format(
            metrics.final_slot_error_m))
    if metrics.hold_velocity_mps is not None and metrics.hold_velocity_mps > epsilon_v:
        task_reasons.append("hold velocity {:.3f} m/s exceeds epsilon_v".format(
            metrics.hold_velocity_mps))
    task_outcome = TASK_PASS if not task_reasons else TASK_FAIL

    validity_reasons: List[str] = []
    if adoption_state != "ADOPTED":
        validity_reasons.append("REFERENCE_ADOPTION_UNCONFIRMED")
    if ledger.valid_sample_ratio < min_valid_sample_ratio:
        validity_reasons.append("valid sample ratio {:.3f} below {:.3f}".format(
            ledger.valid_sample_ratio, min_valid_sample_ratio))
    if ledger.alignment_failure_count:
        validity_reasons.append("{} samples failed time alignment".format(
            ledger.alignment_failure_count))
    if not time_alignment_ok:
        validity_reasons.append("model/ROS time alignment outside engineering gate")
    if not model_hold_satisfied:
        validity_reasons.append("model-time hold evidence missing")
    if any(reason.startswith("REFERENCE_ADOPTION_UNCONFIRMED") for reason in validity_reasons):
        experiment_validity = VALIDITY_INCOMPLETE
    elif validity_reasons:
        experiment_validity = VALIDITY_INVALID
    else:
        experiment_validity = VALIDITY_VALID

    if safety.outcome == SAFETY_NOT_VERIFIED:
        safety_outcome = SAFETY_NOT_VERIFIED
    else:
        safety_outcome = safety.outcome
    reasons = tuple(task_reasons + validity_reasons + list(safety.reasons))
    return ExperimentVerdict(
        task_outcome=task_outcome,
        safety_outcome=safety_outcome,
        experiment_validity=experiment_validity,
        max_position_error_m=(max_position_error_m
                              if max_position_error_m is not None
                              else metrics.final_slot_error_m),
        hold_duration_required_s=hold_duration_s,
        model_hold_satisfied=model_hold_satisfied,
        adoption_state=adoption_state,
        time_alignment_ok=time_alignment_ok,
        metrics=metrics,
        ledger=ledger,
        safety=safety,
        reasons=reasons,
    )
