"""Cumulative model-time / ROS-time alignment across the seven qn members.

Implements the P0.4 evidence of plan.md:

    e_i(t) = [tau_i(t) - tau_i(t0)] - [t_ROS(t) - t_ROS(t0)]

Seven agents are compared on one common ROS grid.  A grid point whose nearest
sample lies outside the alignment window is *missing*, never silently compared
against a differently stamped message.
"""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

DEFAULT_BASELINE_SECONDS = 30.0
DEFAULT_RATE_LOWER = 0.95
DEFAULT_RATE_UPPER = 1.05
DEFAULT_MAX_ABS_DRIFT_S = 0.05
DEFAULT_MAX_CROSS_AGENT_DRIFT_S = 0.05
DEFAULT_ALIGNMENT_WINDOW_S = 0.05
DEFAULT_GRID_STEP_S = 0.1


@dataclass(frozen=True)
class ModelTimeSample:
    """One qn diagnostics observation at its ROS receive time."""

    agent_id: str
    ros_time_s: float
    model_time_s: float
    wall_time_s: Optional[float] = None

    def __post_init__(self):
        if not self.agent_id:
            raise ValueError("agent_id must not be empty")
        for name in ("ros_time_s", "model_time_s"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError("{} must be finite".format(name))
        if self.wall_time_s is not None and not math.isfinite(self.wall_time_s):
            raise ValueError("wall_time_s must be finite")


@dataclass(frozen=True)
class AlignedPoint:
    ros_time_s: float
    model_time_s: Dict[str, float]
    missing_agents: Tuple[str, ...]


@dataclass
class AlignmentReport:
    started_ros_time_s: Optional[float] = None
    finished_ros_time_s: Optional[float] = None
    duration_s: float = 0.0
    agent_ids: Tuple[str, ...] = ()
    grid_point_count: int = 0
    aligned_point_count: int = 0
    alignment_failure_count: int = 0
    valid_sample_count: int = 0
    expected_sample_count: int = 0
    max_abs_model_ros_drift_s: Optional[float] = None
    max_cross_agent_drift_s: Optional[float] = None
    model_ros_rate: Dict[str, float] = field(default_factory=dict)
    model_wall_rate: Dict[str, float] = field(default_factory=dict)
    worst_model_ros_rate: Optional[float] = None
    worst_model_wall_rate: Optional[float] = None
    reference_speed_mps: Optional[float] = None
    induced_reference_displacement_m: Optional[float] = None
    baseline_complete: bool = False
    within_thresholds: bool = False
    common_start_ros_time_s: Optional[float] = None
    history_truncated: bool = False
    reasons: Tuple[str, ...] = ()

    @property
    def valid_sample_ratio(self) -> float:
        if self.expected_sample_count == 0:
            return 0.0
        return self.valid_sample_count / float(self.expected_sample_count)

    def as_dict(self) -> Dict[str, object]:
        return {
            "started_ros_time_s": self.started_ros_time_s,
            "finished_ros_time_s": self.finished_ros_time_s,
            "duration_s": self.duration_s,
            "agent_ids": list(self.agent_ids),
            "grid_point_count": self.grid_point_count,
            "aligned_point_count": self.aligned_point_count,
            "alignment_failure_count": self.alignment_failure_count,
            "valid_sample_count": self.valid_sample_count,
            "expected_sample_count": self.expected_sample_count,
            "valid_sample_ratio": self.valid_sample_ratio,
            "max_abs_model_ros_drift_s": self.max_abs_model_ros_drift_s,
            "max_cross_agent_drift_s": self.max_cross_agent_drift_s,
            "model_ros_rate": dict(self.model_ros_rate),
            "model_wall_rate": dict(self.model_wall_rate),
            "worst_model_ros_rate": self.worst_model_ros_rate,
            "worst_model_wall_rate": self.worst_model_wall_rate,
            "reference_speed_mps": self.reference_speed_mps,
            "induced_reference_displacement_m": self.induced_reference_displacement_m,
            "baseline_complete": self.baseline_complete,
            "common_start_ros_time_s": self.common_start_ros_time_s,
            "history_truncated": self.history_truncated,
            "within_thresholds": self.within_thresholds,
            "reasons": list(self.reasons),
        }


def _slope_from_sums(count, sum_x, sum_y, sum_xx, sum_xy):
    """Least-squares slope from running sums (x already shifted by its origin)."""
    if count < 2:
        return None
    denominator = count * sum_xx - sum_x * sum_x
    if denominator <= 0.0:
        return None
    return (count * sum_xy - sum_x * sum_y) / denominator


def _interpolate(ros_times, model_times, ros_time_s, gap_limit_s):
    """Model time at a common ROS grid time.

    A grid time is *missing* when the agent has no sample span covering it, or
    when the two bracketing samples are further apart than ``gap_limit_s``.
    Interpolation is used instead of nearest-neighbour snapping: with a 0.1 s
    grid, snapping alone injects up to 0.05 s per agent of pure resampling
    noise, which is the same order as the drift gate.  Interpolation measures
    the actual cumulative divergence, and a genuine outage still shows up as
    missing grid points.
    """
    if not ros_times:
        return None
    index = bisect.bisect_left(ros_times, ros_time_s)
    if index < len(ros_times) and ros_times[index] == ros_time_s:
        return model_times[index]
    if index == 0 or index >= len(ros_times):
        return None
    lower = ros_times[index - 1]
    upper = ros_times[index]
    if upper - lower > gap_limit_s:
        return None
    weight = (ros_time_s - lower) / (upper - lower)
    return model_times[index - 1] + weight * (model_times[index] - model_times[index - 1])


class TimeAlignmentMonitor:
    """Accumulate per-agent model-time samples and report cumulative drift.

    The sample history is append-only and shared between the ROS callbacks and
    the readiness loop, so lookups are ``bisect``-based (O(log n)) and the
    report is computed from a copy taken under a short internal lock.  An
    O(grid x samples) report used to run while holding the action-server lock:
    at 60 s it took ~1.4 s and at 180 s ~13 s, which starved every subscription
    callback and made fresh topics look stale.
    """

    def __init__(self, agent_ids: Iterable[str], *,
                 baseline_seconds: float = DEFAULT_BASELINE_SECONDS,
                 rate_lower: float = DEFAULT_RATE_LOWER,
                 rate_upper: float = DEFAULT_RATE_UPPER,
                 max_abs_drift_s: float = DEFAULT_MAX_ABS_DRIFT_S,
                 max_cross_agent_drift_s: float = DEFAULT_MAX_CROSS_AGENT_DRIFT_S,
                 alignment_window_s: float = DEFAULT_ALIGNMENT_WINDOW_S,
                 grid_step_s: float = DEFAULT_GRID_STEP_S,
                 max_samples_per_agent: int = 200000) -> None:
        self.agent_ids = tuple(agent_ids)
        if not self.agent_ids:
            raise ValueError("agent_ids must not be empty")
        if not math.isfinite(baseline_seconds) or baseline_seconds <= 0.0:
            raise ValueError("baseline_seconds must be finite and positive")
        if not 0.0 < rate_lower <= rate_upper:
            raise ValueError("rate bounds must satisfy 0 < lower <= upper")
        if max_abs_drift_s <= 0.0 or max_cross_agent_drift_s <= 0.0:
            raise ValueError("drift thresholds must be positive")
        if not math.isfinite(alignment_window_s) or alignment_window_s <= 0.0:
            raise ValueError("alignment_window_s must be finite and positive")
        if not math.isfinite(grid_step_s) or grid_step_s <= 0.0:
            raise ValueError("grid_step_s must be finite and positive")
        if max_samples_per_agent < 2:
            raise ValueError("max_samples_per_agent must be at least 2")
        self.baseline_seconds = float(baseline_seconds)
        self.rate_lower = float(rate_lower)
        self.rate_upper = float(rate_upper)
        self.max_abs_drift_s = float(max_abs_drift_s)
        self.max_cross_agent_drift_s = float(max_cross_agent_drift_s)
        self.alignment_window_s = float(alignment_window_s)
        self.grid_step_s = float(grid_step_s)
        self.max_samples_per_agent = int(max_samples_per_agent)
        self._lock = threading.RLock()
        self._ros: Dict[str, List[float]] = {agent_id: [] for agent_id in self.agent_ids}
        self._model: Dict[str, List[float]] = {agent_id: [] for agent_id in self.agent_ids}
        self._ros_sums: Dict[str, Dict[str, float]] = {}
        self._wall_sums: Dict[str, Dict[str, float]] = {}
        self._reference_speeds: List[float] = []
        self._history_truncated = False

    # -- observation -------------------------------------------------------
    def note_sample(self, sample: ModelTimeSample) -> None:
        if sample.agent_id not in self._ros:
            raise ValueError("unknown agent {}".format(sample.agent_id))
        with self._lock:
            ros_times = self._ros[sample.agent_id]
            model_times = self._model[sample.agent_id]
            if ros_times:
                if sample.ros_time_s < ros_times[-1]:
                    raise ValueError("model-time samples must be appended in ROS order")
                if sample.ros_time_s == ros_times[-1]:
                    # A redelivered stamp is not new evidence; keeping it would
                    # bias the least-squares rate sums.
                    return
            if len(ros_times) >= self.max_samples_per_agent:
                self._history_truncated = True
                return
            ros_times.append(float(sample.ros_time_s))
            model_times.append(float(sample.model_time_s))
            self._accumulate(self._ros_sums, sample.agent_id,
                             sample.ros_time_s, sample.model_time_s)
            if sample.wall_time_s is not None:
                self._accumulate(self._wall_sums, sample.agent_id,
                                 sample.wall_time_s, sample.model_time_s)

    def _accumulate(self, sums: Dict[str, Dict[str, float]], agent_id: str,
                    x_value: float, y_value: float) -> None:
        bucket = sums.get(agent_id)
        if bucket is None:
            bucket = {"origin": float(x_value), "count": 0.0, "sx": 0.0, "sy": 0.0,
                      "sxx": 0.0, "sxy": 0.0}
            sums[agent_id] = bucket
        x_value = float(x_value) - bucket["origin"]
        bucket["count"] += 1.0
        bucket["sx"] += x_value
        bucket["sy"] += y_value
        bucket["sxx"] += x_value * x_value
        bucket["sxy"] += x_value * y_value

    def latest_observation(self, agent_id: str):
        """(ros_time_s, model_time_s) of the newest accepted sample."""
        with self._lock:
            ros_times = self._ros.get(agent_id)
            if not ros_times:
                return None
            return (ros_times[-1], self._model[agent_id][-1])

    def sample_count(self, agent_id: str) -> int:
        with self._lock:
            return len(self._ros.get(agent_id, ()))

    def note_reference_speed(self, speed_mps: float) -> None:
        if not math.isfinite(speed_mps) or speed_mps < 0.0:
            raise ValueError("reference speed must be finite and nonnegative")
        with self._lock:
            self._reference_speeds.append(float(speed_mps))

    # -- reporting ---------------------------------------------------------
    @property
    def start_time_s(self) -> Optional[float]:
        with self._lock:
            first = [bucket[0] for bucket in self._ros.values() if bucket]
        return min(first) if first else None

    @property
    def duration_s(self) -> float:
        with self._lock:
            end = self._latest_ros_time_locked()
            first = [bucket[0] for bucket in self._ros.values() if bucket]
        if not first or end is None:
            return 0.0
        return max(0.0, end - min(first))

    def _latest_ros_time_locked(self) -> Optional[float]:
        latest = [bucket[-1] for bucket in self._ros.values() if bucket]
        return max(latest) if latest else None

    def _copy(self):
        with self._lock:
            ros = {agent_id: list(bucket) for agent_id, bucket in self._ros.items()}
            model = {agent_id: list(bucket) for agent_id, bucket in self._model.items()}
            ros_sums = {agent_id: dict(bucket) for agent_id, bucket in self._ros_sums.items()}
            wall_sums = {agent_id: dict(bucket) for agent_id, bucket in self._wall_sums.items()}
            speeds = list(self._reference_speeds)
            truncated = self._history_truncated
        return ros, model, ros_sums, wall_sums, speeds, truncated

    def _grid_times(self, ros):
        first = [bucket[0] for bucket in ros.values() if bucket]
        last = [bucket[-1] for bucket in ros.values() if bucket]
        if not first or not last:
            return []
        start, end = min(first), max(last)
        count = int(math.floor((end - start) / self.grid_step_s + 1e-9)) + 1
        return [start + index * self.grid_step_s for index in range(count)]

    def aligned_grid(self) -> List[AlignedPoint]:
        """Resample all agents onto one common ROS grid."""
        ros, model, _ros_sums, _wall_sums, _speeds, _truncated = self._copy()
        points: List[AlignedPoint] = []
        for ros_time in self._grid_times(ros):
            model_times: Dict[str, float] = {}
            missing: List[str] = []
            for agent_id in self.agent_ids:
                value = _interpolate(ros[agent_id], model[agent_id], ros_time,
                                     2.0 * self.alignment_window_s)
                if value is None:
                    missing.append(agent_id)
                else:
                    model_times[agent_id] = value
            points.append(AlignedPoint(ros_time, model_times, tuple(missing)))
        return points

    def report(self, *, require_baseline: bool = True,
               min_duration_s: float = 0.0) -> AlignmentReport:
        """Summarise drift and rate over the recorded window.

        ``require_baseline`` enforces the pre-task qualification duration
        (plan.md P0.4).  A short *task* interval is not disqualified by it: the
        30 s baseline is checked before READY_IDLE and on the whole-run
        summary, while a task-scope report only checks the drift, rate and
        cross-agent gates.

        ``t0`` is the first grid point where all agents are aligned, so the
        bring-up phase of the seven qn nodes is not charged as drift.
        """
        ros, model, ros_sums, wall_sums, speeds, truncated = self._copy()
        report = AlignmentReport(agent_ids=self.agent_ids)
        grid = []
        for ros_time in self._grid_times(ros):
            model_times: Dict[str, float] = {}
            missing: List[str] = []
            for agent_id in self.agent_ids:
                value = _interpolate(ros[agent_id], model[agent_id], ros_time,
                                     2.0 * self.alignment_window_s)
                if value is None:
                    missing.append(agent_id)
                else:
                    model_times[agent_id] = value
            grid.append(AlignedPoint(ros_time, model_times, tuple(missing)))
        if not grid:
            report.reasons = ("no model-time samples recorded",)
            return report
        first_common = next((point for point in grid if not point.missing_agents), None)
        report.history_truncated = truncated
        if first_common is None:
            report.grid_point_count = len(grid)
            report.expected_sample_count = len(grid) * len(self.agent_ids)
            report.reasons = ("no common ROS grid point where all seven agents "
                              "are aligned",)
            return report
        # The compared window ends at the last time covered by *every* agent.
        # The grid runs to the latest sample of the run, so without this trim
        # the tail of the grid (up to one sample period) is reported as
        # "missing agents" even though nothing was actually lost.
        common_end = min(bucket[-1] for bucket in ros.values() if bucket)
        window = [point for point in grid
                  if first_common.ros_time_s - 1e-9 <= point.ros_time_s
                  <= common_end + 1e-9]
        report.started_ros_time_s = first_common.ros_time_s
        report.finished_ros_time_s = window[-1].ros_time_s
        report.duration_s = window[-1].ros_time_s - first_common.ros_time_s
        report.common_start_ros_time_s = first_common.ros_time_s
        report.grid_point_count = len(window)
        report.expected_sample_count = len(window) * len(self.agent_ids)

        start_models = dict(first_common.model_time_s)
        max_abs_drift = 0.0
        max_cross = 0.0
        for point in window:
            if point.missing_agents:
                report.alignment_failure_count += 1
            report.valid_sample_count += len(point.model_time_s)
            elapsed_ros = point.ros_time_s - first_common.ros_time_s
            drifts = []
            for agent_id, model_time in point.model_time_s.items():
                if agent_id not in start_models:
                    continue
                drift = (model_time - start_models[agent_id]) - elapsed_ros
                drifts.append(drift)
                max_abs_drift = max(max_abs_drift, abs(drift))
            for first in range(len(drifts)):
                for second in range(first + 1, len(drifts)):
                    max_cross = max(max_cross, abs(drifts[first] - drifts[second]))
        report.aligned_point_count = sum(1 for point in window if not point.missing_agents)
        report.max_abs_model_ros_drift_s = max_abs_drift
        report.max_cross_agent_drift_s = max_cross

        for agent_id in self.agent_ids:
            bucket = ros_sums.get(agent_id)
            if bucket is not None:
                report.model_ros_rate[agent_id] = _slope_from_sums(
                    bucket["count"], bucket["sx"], bucket["sy"],
                    bucket["sxx"], bucket["sxy"])
            wall_bucket = wall_sums.get(agent_id)
            if wall_bucket is not None:
                report.model_wall_rate[agent_id] = _slope_from_sums(
                    wall_bucket["count"], wall_bucket["sx"], wall_bucket["sy"],
                    wall_bucket["sxx"], wall_bucket["sxy"])
        rates = [rate for rate in report.model_ros_rate.values() if rate is not None]
        wall_rates = [rate for rate in report.model_wall_rate.values() if rate is not None]
        report.worst_model_ros_rate = min(rates) if rates else None
        report.worst_model_wall_rate = min(wall_rates) if wall_rates else None
        if speeds:
            report.reference_speed_mps = max(speeds)
        if report.max_abs_model_ros_drift_s is not None and report.reference_speed_mps is not None:
            report.induced_reference_displacement_m = (
                report.max_abs_model_ros_drift_s * report.reference_speed_mps)

        reasons = []
        if report.duration_s + 1e-9 < self.baseline_seconds:
            if require_baseline:
                reasons.append("baseline shorter than {:.1f} s".format(
                    self.baseline_seconds))
        if report.duration_s + 1e-9 < float(min_duration_s):
            reasons.append("window shorter than {:.2f} s".format(float(min_duration_s)))
        report.baseline_complete = report.duration_s + 1e-9 >= self.baseline_seconds
        if report.alignment_failure_count:
            reasons.append("{} grid points had missing agents".format(
                report.alignment_failure_count))
        if report.history_truncated:
            reasons.append("model-time history was truncated")
        if report.max_abs_model_ros_drift_s is not None and \
                report.max_abs_model_ros_drift_s > self.max_abs_drift_s:
            reasons.append("model/ROS drift {:.4f} s exceeds {:.4f} s".format(
                report.max_abs_model_ros_drift_s, self.max_abs_drift_s))
        if report.max_cross_agent_drift_s is not None and \
                report.max_cross_agent_drift_s > self.max_cross_agent_drift_s:
            reasons.append("cross-agent drift {:.4f} s exceeds {:.4f} s".format(
                report.max_cross_agent_drift_s, self.max_cross_agent_drift_s))
        for agent_id, rate in sorted(report.model_ros_rate.items()):
            if rate is not None and not (self.rate_lower <= rate <= self.rate_upper):
                reasons.append("{} model/ROS rate {:.4f} outside [{}, {}]".format(
                    agent_id, rate, self.rate_lower, self.rate_upper))
        report.reasons = tuple(reasons)
        # ``baseline_complete`` is reported separately: a short *task* window
        # passes the drift/rate gates without pretending it was a 30 s baseline
        # (that requirement is a pre-task qualification, enforced through
        # ``require_baseline``).
        report.within_thresholds = not reasons
        return report


class ModelTimeHistory:
    """Bounded per-agent history for nearest-time model-time lookups."""

    def __init__(self, agent_ids: Iterable[str], *, horizon_s: float = 300.0) -> None:
        self.agent_ids = tuple(agent_ids)
        if not math.isfinite(horizon_s) or horizon_s <= 0.0:
            raise ValueError("horizon_s must be finite and positive")
        self.horizon_s = float(horizon_s)
        self._lock = threading.RLock()
        self._samples: Dict[str, Deque[ModelTimeSample]] = {
            agent_id: deque() for agent_id in self.agent_ids}
        self._ros: Dict[str, List[float]] = {agent_id: [] for agent_id in self.agent_ids}
        self._head: Dict[str, int] = {agent_id: 0 for agent_id in self.agent_ids}

    def note(self, sample: ModelTimeSample) -> None:
        if sample.agent_id not in self._samples:
            raise ValueError("unknown agent {}".format(sample.agent_id))
        with self._lock:
            bucket = self._samples[sample.agent_id]
            if bucket and sample.ros_time_s < bucket[-1].ros_time_s:
                raise ValueError("model-time samples must be appended in ROS order")
            bucket.append(sample)
            self._ros[sample.agent_id].append(float(sample.ros_time_s))
            cutoff = sample.ros_time_s - self.horizon_s
            while bucket and bucket[0].ros_time_s < cutoff:
                bucket.popleft()
                self._head[sample.agent_id] += 1
            head = self._head[sample.agent_id]
            if head > 4096:
                del self._ros[sample.agent_id][:head]
                self._head[sample.agent_id] = 0

    def latest(self, agent_id: str) -> Optional[ModelTimeSample]:
        with self._lock:
            bucket = self._samples.get(agent_id)
            return bucket[-1] if bucket else None

    def samples_after(self, agent_id: str, ros_time_s: float):
        """All retained samples strictly after ``ros_time_s``, in ROS order."""
        with self._lock:
            ros_times = self._ros.get(agent_id)
            if not ros_times:
                return ()
            head = self._head[agent_id]
            index = bisect.bisect_right(ros_times, float(ros_time_s), head)
            bucket = self._samples[agent_id]
            return tuple(bucket[position - head]
                         for position in range(index, len(ros_times)))

    def model_time_at(self, agent_id: str, ros_time_s: float,
                      window_s: float) -> Optional[float]:
        with self._lock:
            ros_times = self._ros.get(agent_id)
            if not ros_times:
                return None
            head = self._head[agent_id]
            index = bisect.bisect_left(ros_times, float(ros_time_s), head)
            candidates = []
            if index < len(ros_times):
                candidates.append(index)
            if index - 1 >= head:
                candidates.append(index - 1)
            if not candidates:
                return None
            best = min(candidates, key=lambda position: abs(ros_times[position] - ros_time_s))
            if abs(ros_times[best] - ros_time_s) > window_s:
                return None
            bucket = self._samples[agent_id]
            offset = best - head
            if offset < 0 or offset >= len(bucket):
                return None
            return bucket[offset].model_time_s
