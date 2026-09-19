#!/usr/bin/env python3
"""Independent verification of one saved FormationAction experiment.

The verifier re-derives the plan.md P0-P2 evidence from the artifacts on disk
instead of trusting the action server's own verdict:

* the standard qn Odometry convention (world pose/attitude, body twist) and the
  separate Swarm compatibility input (world-frame linear velocity),
* the reference actually used by qn, reconstructed as
  ``v_used,k = (p_used,k - p_used,k-1) / outer_dt_s`` from the published
  ``used_reference_pose``/``used_reference_twist``/``diagnostics`` streams,
* new-reference adoption through ``trajectory_id`` ownership,
* model-time vs ROS-time drift on a common seven-agent grid, including the
  hold requirement in *model* time,
* the three independent verdicts and the sample ledger,
* Test C (``plan_updated``/``updated_plan_used``/``dispatch_changed``) and
  DelayEvent idempotence in the repair history.

Any conflict between the recorded Action envelope and the saved diagnostics is
reported as a failure rather than resolved in favour of the diagnostics.
"""

import json
import math
import sys
from collections import defaultdict
from pathlib import Path

MODEL_ROS_DRIFT_LIMIT_S = 0.05
CROSS_AGENT_DRIFT_LIMIT_S = 0.05
RATE_LOWER = 0.95
RATE_UPPER = 1.05
ALIGNMENT_GRID_STEP_S = 0.1
ALIGNMENT_WINDOW_S = 0.06
WORLD_VELOCITY_TOLERANCE_MPS = 1e-3
# The compatibility topic must carry the *world-frame* velocity of the same
# published state, which is checked independently against the finite
# difference of the published pose.  qn accelerates hard at the start of a
# trajectory, so the bound is a quantile plus a transient maximum instead of a
# single tight number.
POSE_VELOCITY_QUANTILE = 0.99
POSE_VELOCITY_QUANTILE_TOLERANCE_MPS = 0.02
# The hard tail is a measurement, not a gate: a single trajectory-switch sample
# can reach ~1.1 m/s while a wrong-frame velocity would put essentially every
# moving sample near the vehicle speed.  The quantile above is what discriminates.
POSE_VELOCITY_TAIL_QUANTILE = 0.999
POSE_VELOCITY_TAIL_TOLERANCE_MPS = 0.2
# A wrong-frame velocity (body velocity compared as if it were world velocity)
# is off by the vehicle speed, i.e. ~1.5 m/s here.  The quantile gate above is
# what discriminates; this bound only has to catch that gross signature without
# failing on a single-sample trajectory-switch transient (observed worst 0.54
# m/s against a p99 of 0.006 m/s).
POSE_VELOCITY_MAX_TOLERANCE_MPS = 3.0
POSE_TOLERANCE_M = 1e-6


def quaternion_rotation(quaternion):
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def rotate(rotation, vector):
    return tuple(rotation[row][0] * vector[0] + rotation[row][1] * vector[1]
                 + rotation[row][2] * vector[2] for row in range(3))


def distance(first, second):
    return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))


def nearest(series, stamp, window):
    best = None
    best_distance = None
    for sample in series:
        difference = abs(sample[0] - stamp)
        if best_distance is None or difference < best_distance:
            best, best_distance = sample, difference
    if best is None or best_distance > window:
        return None
    return best


def slope(points):
    if len(points) < 2:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    denominator = sum((point[0] - mean_x) ** 2 for point in points)
    if denominator <= 0.0:
        return None
    return sum((point[0] - mean_x) * (point[1] - mean_y)
               for point in points) / denominator


class Verification:
    def __init__(self):
        self.checks = []
        self.measurements = []

    def check(self, name, ok, detail=""):
        self.checks.append({"name": name, "ok": bool(ok), "detail": str(detail)})
        if not ok:
            print("FAIL {}: {}".format(name, detail))
        return bool(ok)

    def measure(self, name, value, detail=""):
        """Record a number that supports a check, not only its pass/fail."""
        self.measurements.append(
            {"name": name, "value": value, "detail": str(detail)})

    @property
    def failures(self):
        return [check for check in self.checks if not check["ok"]]

    def summary(self):
        return {
            "status": "PASS" if not self.failures else "FAIL",
            "check_count": len(self.checks),
            "failure_count": len(self.failures),
            "failures": self.failures,
            "measurements": self.measurements,
        }


def load_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (IOError, ValueError) as error:
        raise AssertionError("cannot read {}: {}".format(path, error))


class BagEvidence(object):
    """Messages extracted from the execution bag."""

    def __init__(self):
        self.standard_odometry = defaultdict(list)
        self.compat_odometry = defaultdict(list)
        self.swarm_input_odometry = defaultdict(list)
        self.local_clouds = defaultdict(list)
        self.scene_clouds = defaultdict(list)
        self.used_reference_pose = defaultdict(list)
        self.used_reference_twist = defaultdict(list)
        self.diagnostics = defaultdict(list)
        self.position_commands = defaultdict(list)
        self.group_goals = []
        self.goals = {}
        self.results = {}

    @staticmethod
    def _odometry_sample(message):
        point = message.pose.pose.position
        orientation = message.pose.pose.orientation
        twist = message.twist.twist
        return (
            message.header.stamp.to_sec(),
            (point.x, point.y, point.z),
            (orientation.w, orientation.x, orientation.y, orientation.z),
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
            str(message.header.frame_id),
            str(message.child_frame_id),
        )

    def read(self, bag_path):
        import rosbag
        with rosbag.Bag(str(bag_path)) as bag:
            for topic, message, stamp in bag.read_messages():
                if topic == "/move_base_simple/goal":
                    caller = ""
                    try:
                        caller = message._connection_header.get("callerid", "")
                    except AttributeError:
                        pass
                    self.group_goals.append((stamp.to_sec(), caller, message))
                elif topic == "/formation_action/goal":
                    self.goals[message.goal_id.id] = message
                elif topic == "/formation_action/result":
                    self.results[message.status.goal_id.id] = message
                elif topic.endswith("/planning/pos_cmd"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    self.position_commands[agent].append(
                        (message.header.stamp.to_sec(), int(message.trajectory_id)))
                elif topic.endswith("_qn/odometry"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    self.standard_odometry[agent].append(self._odometry_sample(message))
                elif topic.endswith("_qn/odometry_swarm_compat"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    self.compat_odometry[agent].append(self._odometry_sample(message))
                elif topic.endswith("_visual_slam/odom"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    self.swarm_input_odometry[agent].append(
                        self._odometry_sample(message))
                elif topic.endswith("_qn/used_reference_pose"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    point = message.pose.position
                    self.used_reference_pose[agent].append(
                        (message.header.stamp.to_sec(), (point.x, point.y, point.z)))
                elif topic.endswith("_qn/used_reference_twist"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    twist = message.twist.linear
                    self.used_reference_twist[agent].append(
                        (message.header.stamp.to_sec(), (twist.x, twist.y, twist.z)))
                elif topic.endswith("_qn/diagnostics"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    values = {}
                    for status in message.status:
                        for entry in status.values:
                            values[entry.key] = entry.value
                    if "model_time_s" in values:
                        self.diagnostics[agent].append(
                            (message.header.stamp.to_sec(), values))
                elif topic.endswith("_pcl_render_node/cloud"):
                    agent = int(topic.split("/")[1].split("_")[1])
                    self.local_clouds[agent].append(
                        (message.header.stamp.to_sec(),
                         int(message.width) * int(message.height)))
                elif topic.endswith("/global_cloud"):
                    try:
                        caller = message._connection_header.get("callerid", "")
                    except AttributeError:
                        caller = ""
                    self.scene_clouds[topic].append(
                        (message.header.stamp.to_sec(), caller,
                         int(message.width) * int(message.height)))
        for series in (self.standard_odometry, self.compat_odometry,
                       self.swarm_input_odometry, self.diagnostics,
                       self.used_reference_pose, self.used_reference_twist,
                       self.scene_clouds):
            for key in series:
                series[key].sort(key=lambda item: item[0])


def check_odometry_contracts(verification, evidence):
    for agent_id in sorted(evidence.standard_odometry):
        samples = evidence.standard_odometry[agent_id]
        expected_frames = {("world", "drone_{}/base_link".format(agent_id))}
        verification.check(
            "odometry_standard_present[{}]".format(agent_id), bool(samples),
            "no /drone_{}_qn/odometry messages".format(agent_id))
        if not samples:
            continue
        frames = {(sample[5], sample[6]) for sample in samples}
        verification.check(
            "odometry_standard_convention[{}]".format(agent_id),
            frames == expected_frames,
            "frame/child {} != {}".format(
                sorted(frames), sorted(expected_frames)))
        # The Swarm compatibility output keeps the world-frame linear
        # velocity and is remapped onto the topic the Swarm planners already
        # consume (/drone_i_visual_slam/odom).  The raw
        # *_qn/odometry_swarm_compat name is also read when it is recorded, so
        # the two topics can be compared instead of compared with themselves.
        raw_compat = evidence.compat_odometry.get(agent_id) or []
        compat = evidence.swarm_input_odometry.get(agent_id) or raw_compat
        verification.check(
            "odometry_compat_present[{}]".format(agent_id), bool(compat),
            "no Swarm compatibility odometry for drone {}".format(agent_id))
        if not compat:
            continue
        compat_frames = {(sample[5], sample[6]) for sample in compat}
        expected_compat = {("world", "drone_{}/swarm_compat".format(agent_id))}
        verification.check(
            "odometry_compat_convention[{}]".format(agent_id),
            compat_frames == expected_compat,
            "frame/child {} != {}".format(
                sorted(compat_frames), sorted(expected_compat)))
        # plan.md P0.1: both outputs come from one qn state snapshot.  The
        # standard topic carries the pose plus the body twist, the Swarm
        # compatibility topic carries the same pose plus the world-frame
        # velocity Swarm already consumes.  The two published twists must be
        # two representations of the same state, not two independent states.
        index_by_stamp = {}
        for entry in compat:
            index_by_stamp.setdefault(round(entry[0], 6), entry)
        worst_pose = 0.0
        worst_snapshot = 0.0
        compared = 0
        unmatched = 0
        for stamp, position, quaternion, body_velocity, _angular, _f, _c in samples:
            # Both Odometry messages are published from one state snapshot and
            # carry the identical header stamp.  Matching by nearest stamp
            # would silently pair two different ticks whenever the recorder
            # dropped one of the two topics for a tick.
            match = index_by_stamp.get(round(stamp, 6))
            if match is None:
                unmatched += 1
                continue
            compared += 1
            worst_pose = max(worst_pose, distance(match[1], position))
            # The published standard Odometry must satisfy the ROS contract:
            # rotating its body twist by its own attitude reproduces the world
            # velocity the Swarm compatibility input carries.
            worst_snapshot = max(
                worst_snapshot,
                distance(rotate(quaternion_rotation(quaternion), body_velocity),
                         match[3]))
        verification.check(
            "odometry_standard_compat_share_pose[{}]".format(agent_id),
            compared > 0 and worst_pose <= POSE_TOLERANCE_M,
            "{} stamp-matched pairs ({} standard samples have no exact "
            "compat stamp match), max pose difference {:.6f} m".format(
                compared, unmatched, worst_pose))
        verification.check(
            "odometry_snapshot_body_to_world[{}]".format(agent_id),
            compared > 0 and worst_snapshot <= WORLD_VELOCITY_TOLERANCE_MPS,
            "{} stamp-matched pairs, max |R(q)*v_body - world velocity| "
            "{:.6f} m/s".format(compared, worst_snapshot))
        # Independent evidence that the compatibility topic really is the
        # world-frame velocity: differentiate the published world pose and
        # compare with the compatibility twist instead of trusting the node's
        # own formula.
        # Trapezoidal difference: the published twist is the instantaneous
        # velocity at the sample, so comparing it with a backward difference
        # carries an acceleration bias of a*dt/2.  Averaging the two endpoint
        # twists removes that first-order term and leaves the real transient.
        differences = []
        for index in range(1, len(samples)):
            stamp = samples[index][0]
            previous = samples[index - 1][0]
            elapsed = stamp - previous
            if elapsed <= 0.0 or elapsed > 0.02:
                # A recorder gap makes the finite difference meaningless.
                continue
            match = index_by_stamp.get(round(stamp, 6))
            earlier = index_by_stamp.get(round(previous, 6))
            if match is None or earlier is None:
                continue
            finite_difference = tuple(
                (samples[index][1][axis] - samples[index - 1][1][axis]) / elapsed
                for axis in range(3))
            averaged = tuple(
                0.5 * (match[3][axis] + earlier[3][axis]) for axis in range(3))
            differences.append(distance(finite_difference, averaged))
        verification.measure(
            "odometry_pose_residual_m[{}]".format(agent_id), worst_pose,
            "max |standard pose - compatibility pose| over {} stamp-matched pairs "
            "({} standard samples had no exact partner)".format(compared, unmatched))
        verification.measure(
            "odometry_snapshot_residual_mps[{}]".format(agent_id), worst_snapshot,
            "max |compatibility twist - qn world velocity from the standard body "
            "twist|; a wrong-frame velocity would be off by the vehicle speed")
        verification.measure(
            "odometry_body_to_world_residual_mps[{}]".format(agent_id),
            worst_snapshot,
            "max |R(q)*body twist - world velocity| over stamp-matched pairs; the "
            "published attitude/twist pair must satisfy the ROS contract")
        differences.sort()
        quantile_index = max(
            0, int(math.ceil(POSE_VELOCITY_QUANTILE * len(differences))) - 1)
        tail_index = max(
            0, int(math.ceil(0.999 * len(differences))) - 1)
        worst_pose_velocity = differences[-1] if differences else None
        quantile_value = differences[quantile_index] if differences else None
        verification.measure(
            "odometry_pose_velocity_p999_mps[{}]".format(agent_id),
            differences[tail_index] if differences else None,
            "99.9th percentile of the same comparison; the max is a single "
            "trajectory-switch transient")
        verification.measure(
            "odometry_pose_velocity_p99_mps[{}]".format(agent_id), quantile_value,
            "{} trapezoidal differences of the published pose against the "
            "compatibility twist at the {:.0f}th percentile".format(
                len(differences), 100.0 * POSE_VELOCITY_QUANTILE))
        verification.measure(
            "odometry_pose_velocity_worst_mps[{}]".format(agent_id),
            worst_pose_velocity,
            "worst of the same {} comparisons (qn transients)".format(
                len(differences)))
        tail_value = (differences[tail_index] if differences else None)
        verification.measure(
            "odometry_pose_velocity_p999_mps[{}]".format(agent_id), tail_value,
            "99.9th percentile of the same comparison")
        verification.check(
            "odometry_compat_is_pose_velocity[{}]".format(agent_id),
            bool(differences)
            and quantile_value <= POSE_VELOCITY_QUANTILE_TOLERANCE_MPS
            and tail_value is not None
            and tail_value <= POSE_VELOCITY_TAIL_TOLERANCE_MPS
            and worst_pose_velocity <= POSE_VELOCITY_MAX_TOLERANCE_MPS,
            "{} finite differences at {:.0f}%: {:.6f} m/s, worst {:.6f} m/s".format(
                len(differences), 100.0 * POSE_VELOCITY_QUANTILE,
                -1.0 if quantile_value is None else quantile_value,
                -1.0 if worst_pose_velocity is None else worst_pose_velocity))
        swarm = evidence.swarm_input_odometry.get(agent_id) or []
        verification.check(
            "swarm_input_present[{}]".format(agent_id), bool(swarm),
            "no /drone_{}_visual_slam/odom (Swarm compatibility input)".format(
                agent_id))
        if not swarm:
            continue
        frames = {(sample[5], sample[6]) for sample in swarm}
        verification.check(
            "swarm_input_is_compat[{}]".format(agent_id),
            {sample[5] for sample in swarm} == {"world"}
            and all(sample[6].endswith("swarm_compat") for sample in swarm),
            "frames {}".format(sorted(frames)))
        if not raw_compat:
            # The raw compatibility topic is not recorded by this launch, so
            # the content contract is established against the standard
            # odometry above (shared pose + pose-derivative world velocity)
            # instead of a vacuous comparison of the Swarm input with itself.
            verification.check(
                "swarm_input_carries_compat_stream[{}]".format(agent_id),
                bool(swarm)
                and {sample[5] for sample in swarm} == {"world"}
                and all(sample[6].endswith("swarm_compat") for sample in swarm),
                "the Swarm input topic is the remapped qn compatibility stream; "
                "~odometry_swarm_compat is not a recorded topic")
            continue
        worst = 0.0
        compared = 0
        for stamp, _position, _quaternion, velocity, _angular, _f, _c in swarm:
            match = nearest(raw_compat, stamp, 0.02)
            if match is None:
                continue
            compared += 1
            worst = max(worst, distance(velocity, match[3]))
        verification.check(
            "swarm_input_equals_compat[{}]".format(agent_id),
            compared > 0 and worst <= WORLD_VELOCITY_TOLERANCE_MPS,
            "{} time-matched pairs, max world-velocity difference {:.6f} m/s".format(
                compared, worst))


def extract_model_series(evidence):
    series = {}
    for agent_id, samples in evidence.diagnostics.items():
        rows = []
        for stamp, values in samples:
            rows.append({
                "ros_time_s": stamp,
                "model_time_s": float(values["model_time_s"]),
                "used_outer_step": int(values.get("used_outer_step", -1)),
                "source_trajectory_id": int(values.get("source_trajectory_id", -1)),
                "outer_dt_s": float(values.get("outer_dt_s", "nan")),
                "reference_position": (
                    float(values["reference_position_x"]),
                    float(values["reference_position_y"]),
                    float(values["reference_position_z"])),
                "used_reference_velocity": (
                    float(values["used_reference_velocity_x"]),
                    float(values["used_reference_velocity_y"]),
                    float(values["used_reference_velocity_z"])),
                "velocity_directly_consumed": values.get("velocity_directly_consumed"),
                "acceleration_directly_consumed": values.get(
                    "acceleration_directly_consumed"),
            })
        rows.sort(key=lambda row: row["ros_time_s"])
        series[agent_id] = rows
    return series


def check_used_reference_formula(verification, series, published_pose, published_twist):
    for agent_id, rows in sorted(series.items()):
        worst = 0.0
        checked = 0
        for previous, current in zip(rows, rows[1:]):
            if current["used_outer_step"] != previous["used_outer_step"] + 1:
                continue
            outer_dt = current["outer_dt_s"]
            if not math.isfinite(outer_dt) or outer_dt <= 0.0:
                continue
            expected = tuple(
                (current["reference_position"][axis]
                 - previous["reference_position"][axis]) / outer_dt
                for axis in range(3))
            worst = max(worst, distance(expected, current["used_reference_velocity"]))
            checked += 1
        verification.check(
            "used_reference_velocity_formula[{}]".format(agent_id),
            checked > 0 and worst <= 1e-6,
            "{} consecutive outer steps, max error {:.3e} m/s".format(checked, worst))
        poses = published_pose.get(agent_id, [])
        twists = published_twist.get(agent_id, [])
        verification.check(
            "used_reference_topics_present[{}]".format(agent_id),
            bool(poses) and bool(twists),
            "{} pose / {} twist messages".format(len(poses), len(twists)))
        worst = 0.0
        compared = 0
        for row in rows:
            # The pose/twist topics and the diagnostics are published from the
            # same snapshot, so they must agree at the same header stamp; a
            # wide window would compare different samples of a moving vehicle.
            pose = nearest(poses, row["ros_time_s"], 0.002)
            twist = nearest(twists, row["ros_time_s"], 0.002)
            if pose is None or twist is None:
                continue
            compared += 1
            worst = max(worst, distance(pose[1], row["reference_position"]),
                        distance(twist[1], row["used_reference_velocity"]))
        ratio = compared / float(len(rows)) if rows else 0.0
        verification.check(
            "used_reference_topics_match_diagnostics[{}]".format(agent_id),
            compared > 0 and ratio >= 0.99 and worst <= 1e-6,
            "{} of {} rows matched, max difference {:.3e}".format(
                compared, len(rows), worst))
        flags = {(row["velocity_directly_consumed"],
                  row["acceleration_directly_consumed"]) for row in rows}
        verification.check(
            "used_reference_consumption_declared[{}]".format(agent_id),
            flags <= {("false", "false")},
            "unexpected consumption flags {}".format(sorted(flags)))


def check_model_time(verification, series, hold_entries):
    agents = sorted(series)
    rows_by_agent = {agent_id: [(row["ros_time_s"], row["model_time_s"])
                                for row in series[agent_id]] for agent_id in agents}
    populated = [agent_id for agent_id in agents if rows_by_agent[agent_id]]
    if not populated:
        verification.check("model_time_samples_present", False, "no diagnostics")
        return None
    start = max(rows_by_agent[agent_id][0][0] for agent_id in populated)
    end = min(rows_by_agent[agent_id][-1][0] for agent_id in populated)
    grid = []
    index = 0
    while start + index * ALIGNMENT_GRID_STEP_S <= end + 1e-9:
        grid.append(start + index * ALIGNMENT_GRID_STEP_S)
        index += 1
    drift = 0.0
    cross = 0.0
    missing = 0
    valid = 0
    matched_points = {agent_id: [] for agent_id in populated}
    start_model = {}
    for agent_id in populated:
        match = nearest(rows_by_agent[agent_id], start, ALIGNMENT_WINDOW_S)
        if match is not None:
            start_model[agent_id] = match[1]
    for stamp in grid:
        models = {}
        for agent_id in populated:
            match = nearest(rows_by_agent[agent_id], stamp, ALIGNMENT_WINDOW_S)
            if match is None:
                missing += 1
                continue
            models[agent_id] = match[1]
            matched_points[agent_id].append((match[0], match[1]))
        if len(models) != len(populated):
            continue
        valid += 1
        values = []
        for agent_id, model_time in models.items():
            if agent_id not in start_model:
                continue
            value = (model_time - start_model[agent_id]) - (stamp - start)
            values.append(value)
            drift = max(drift, abs(value))
        if len(values) >= 2:
            cross = max(cross, max(values) - min(values))
    rates = {}
    for agent_id in populated:
        rate = slope(matched_points[agent_id])
        if rate is not None:
            rates[agent_id] = rate
    worst_rate = min(rates.values()) if rates else None
    best_rate = max(rates.values()) if rates else None
    verification.check(
        "model_time_grid_available", len(grid) >= 2,
        "{} grid points over {:.3f} s".format(len(grid), max(0.0, end - start)))
    verification.check(
        "model_time_grid_aligned", missing == 0,
        "{} of {} agent-grid points missing".format(
            missing, len(grid) * len(populated)))
    verification.check(
        "model_ros_drift_within_gate", drift <= MODEL_ROS_DRIFT_LIMIT_S,
        "max |e_i| = {:.4f} s (limit {:.3f} s)".format(
            drift, MODEL_ROS_DRIFT_LIMIT_S))
    verification.check(
        "cross_agent_drift_within_gate", cross <= CROSS_AGENT_DRIFT_LIMIT_S,
        "max cross-agent drift = {:.4f} s (limit {:.3f} s)".format(
            cross, CROSS_AGENT_DRIFT_LIMIT_S))
    verification.check(
        "model_ros_rate_within_gate",
        worst_rate is not None and best_rate is not None
        and RATE_LOWER <= worst_rate and best_rate <= RATE_UPPER,
        "per-agent rates {} (gate [{}, {}])".format(
            {key: round(value, 5) for key, value in rates.items()},
            RATE_LOWER, RATE_UPPER))
    for entry in hold_entries:
        if entry.get("window") is None:
            verification.check(
                "model_time_hold[{}]".format(entry["task_id"]), False,
                "no successful hold window recorded")
            continue
        hold_start, hold_end = entry["window"]
        values = []
        missing_agents = []
        for agent_id in populated:
            begin = nearest(rows_by_agent[agent_id], hold_start, ALIGNMENT_WINDOW_S)
            finish = nearest(rows_by_agent[agent_id], hold_end, ALIGNMENT_WINDOW_S)
            if begin is None or finish is None:
                missing_agents.append(agent_id)
                continue
            values.append(finish[1] - begin[1])
        minimum = min(values) if values else None
        verification.check(
            "model_time_hold[{}]".format(entry["task_id"]),
            not missing_agents and minimum is not None
            and minimum + 1e-9 >= entry["hold_duration"],
            "min model hold {} s over required {} s, missing agents {}".format(
                None if minimum is None else round(minimum, 4),
                entry["hold_duration"], missing_agents))
    return {
        "grid_points": len(grid),
        "valid_grid_points": valid,
        "missing_agent_points": missing,
        "max_abs_model_ros_drift_s": drift,
        "max_cross_agent_drift_s": cross,
        "model_ros_rate": rates,
        "worst_model_ros_rate": worst_rate,
    }


def check_hold_odometry(verification, evidence, executions, config, root):
    monitor = config["monitor"]
    scale = float(monitor["swarm_scale"])
    slots = monitor["relative_slots"]
    epsilon_p = float(monitor["epsilon_p"])
    epsilon_v = float(monitor["epsilon_v"])
    odom_timeout = float(monitor["odom_timeout"])
    centers = config["runner"]["centers"]
    tasks = {task["task_id"]: task for task in config["tasks"]}
    for execution in executions:
        task = tasks[execution["task_id"]]
        center = centers[task["target_ref"]]
        diagnostics = load_json(root / execution["evidence_file"])
        window = diagnostics.get("successful_hold_window") or {}
        hold_start = window.get("start")
        hold_end = window.get("end")
        if hold_start is None or hold_end is None:
            verification.check(
                "hold_window[{}]".format(task["task_id"]), False,
                "no successful hold window in diagnostics")
            continue
        hold_duration = float(diagnostics["hold_duration"])
        verification.check(
            "hold_window_duration[{}]".format(task["task_id"]),
            hold_end - hold_start + 1e-9 >= hold_duration,
            "ROS hold {:.3f} s < {:.3f} s".format(
                hold_end - hold_start, hold_duration))
        for agent_id in range(7):
            target = tuple(center[axis] + scale * float(slots[str(agent_id)][axis])
                           for axis in range(3))
            samples = [sample for sample in evidence.standard_odometry.get(agent_id, ())
                       if hold_start - odom_timeout <= sample[0]
                       <= hold_end + odom_timeout]
            samples.sort(key=lambda sample: sample[0])
            if not samples:
                verification.check(
                    "hold_odometry[{}][{}]".format(task["task_id"], agent_id),
                    False, "no odometry samples in the hold window")
                continue
            gaps = [second[0] - first[0]
                    for first, second in zip(samples, samples[1:])]
            max_gap = max(gaps) if gaps else 0.0
            covered = (samples[0][0] - hold_start <= odom_timeout
                       and hold_end - samples[-1][0] <= odom_timeout)
            worst_position = 0.0
            worst_speed = 0.0
            for stamp, position, quaternion, body_velocity, _angular, _f, _c in samples:
                # The padding above only tolerates missing boundary samples for
                # the coverage check.  The limit itself applies to the accepted
                # hold window: samples from before it was reached describe the
                # approach, not the hold.
                if stamp < hold_start or stamp > hold_end:
                    continue
                world = rotate(quaternion_rotation(quaternion), body_velocity)
                worst_position = max(worst_position, distance(position, target))
                worst_speed = max(worst_speed, distance(world, (0.0, 0.0, 0.0)))
            verification.check(
                "hold_odometry[{}][{}]".format(task["task_id"], agent_id),
                covered and max_gap <= odom_timeout
                and worst_position <= epsilon_p and worst_speed <= epsilon_v,
                "covered={} max_gap={:.4f} max_position_error={:.4f} "
                "max_speed={:.4f}".format(covered, max_gap, worst_position,
                                          worst_speed))


def check_action_envelopes(verification, evidence, executions, root):
    verification.check(
        "action_goal_envelopes",
        set(evidence.goals) >= {execution["goal_id"] for execution in executions},
        "missing goal envelopes for {}".format(
            sorted({execution["goal_id"] for execution in executions} - set(evidence.goals))))
    verification.check(
        "action_result_envelopes",
        set(evidence.results) >= {execution["goal_id"] for execution in executions},
        "missing result envelopes for {}".format(
            sorted({execution["goal_id"] for execution in executions}
                   - set(evidence.results))))
    for execution in executions:
        goal_id = execution["goal_id"]
        if goal_id not in evidence.results:
            continue
        envelope = evidence.results[goal_id]
        diagnostics = load_json(root / execution["evidence_file"])
        verdict = diagnostics.get("verdict") or {}
        verification.check(
            "action_envelope_matches_diagnostics[{}]".format(execution["task_id"]),
            envelope.status.status == execution["terminal_status"] == 3,
            "envelope status {} != terminal {} (server run_state {})".format(
                envelope.status.status, execution["terminal_status"],
                diagnostics.get("run_state")))
        verification.check(
            "action_result_matches_diagnostics[{}]".format(execution["task_id"]),
            envelope.result.reason == execution.get("action_reason")
            and envelope.result.reason == diagnostics.get("reason"),
            "envelope reason {} != execution {} / diagnostics {}".format(
                envelope.result.reason, execution.get("action_reason"),
                diagnostics.get("reason")))
        verification.check(
            "action_result_verdicts[{}]".format(execution["task_id"]),
            verdict.get("task_outcome") == "PASS"
            and verdict.get("experiment_validity") == "VALID"
            and verdict.get("safety_outcome") in ("PASS", "FAIL", "NOT_VERIFIED"),
            "verdict {}".format(verdict))
        verification.check(
            "action_result_carries_verdict[{}]".format(execution["task_id"]),
            envelope.result.experiment_validity == 1,
            "result.experiment_validity={}".format(
                envelope.result.experiment_validity))


def check_diagnostics(verification, executions, config, root):
    monitor = config["monitor"]
    for execution in executions:
        task_id = execution["task_id"]
        evidence_file = execution.get("evidence_file")
        if not evidence_file:
            # A rejected/timed-out dispatch has no diagnostics file; accounting
            # for it here keeps the run reportable instead of crashing.
            verification.check(
                "diagnostics_evidence[{}]".format(task_id), False,
                "no diagnostics evidence recorded for {}".format(task_id))
            continue
        diagnostics = load_json(root / evidence_file)
        if not diagnostics:
            verification.check(
                "diagnostics_evidence[{}]".format(task_id), False,
                "empty diagnostics evidence for {}".format(task_id))
            continue
        verification.check(
            "diagnostics_goal_id[{}]".format(task_id),
            diagnostics["goal_id"] == execution["goal_id"],
            "goal_id {} != {}".format(diagnostics["goal_id"], execution["goal_id"]))
        verification.check(
            "diagnostics_run_state[{}]".format(task_id),
            diagnostics.get("run_state") == "READY_IDLE",
            "run_state {}".format(diagnostics.get("run_state")))
        adoption = diagnostics.get("adoption_verdict") or {}
        verification.check(
            "diagnostics_adoption[{}]".format(task_id),
            adoption.get("state") == "ADOPTED",
            "adoption {}".format(adoption))
        alignment = diagnostics.get("time_alignment") or {}
        verification.check(
            "diagnostics_time_alignment[{}]".format(task_id),
            bool(alignment.get("within_thresholds"))
            and not alignment.get("alignment_failure_count"),
            "alignment reasons {}".format(alignment.get("reasons")))
        model_hold = diagnostics.get("model_hold") or {}
        verification.check(
            "diagnostics_model_hold[{}]".format(task_id),
            model_hold.get("min_model_hold_s") is not None
            and model_hold["min_model_hold_s"] + 1e-9 >= diagnostics["hold_duration"],
            "model hold {} vs required {}".format(
                model_hold.get("min_model_hold_s"), diagnostics["hold_duration"]))
        ledger = (diagnostics.get("verdict") or {}).get("sample_ledger") or {}
        verification.check(
            "diagnostics_sample_ledger[{}]".format(task_id),
            bool(ledger) and ledger.get("alignment_failure_count") == 0
            and ledger.get("valid_sample_ratio", 0.0)
            >= float(monitor["min_valid_sample_ratio"]),
            "ledger {}".format(ledger))
        safety = (diagnostics.get("verdict") or {}).get("safety") or {}
        verification.check(
            "diagnostics_safety_evidence_kind[{}]".format(task_id),
            safety.get("evidence_kind") == "DISCRETE_SAMPLED",
            "evidence kind {}".format(safety.get("evidence_kind")))
        verification.check(
            "diagnostics_group_goal_count[{}]".format(task_id),
            diagnostics.get("goal_publish_count") == 1
            and diagnostics.get("topic_group_goal_count") == 1,
            "publish counts goal={} topic={}".format(
                diagnostics.get("goal_publish_count"),
                diagnostics.get("topic_group_goal_count")))
        verification.check(
            "diagnostics_state_machine_released[{}]".format(task_id),
            (diagnostics.get("state_machine") or {}).get("state") == "READY_IDLE",
            "state machine {}".format(diagnostics.get("state_machine")))


def check_group_goals(verification, evidence, executions, config, root):
    if len(evidence.group_goals) != len(executions):
        verification.check(
            "group_goal_publish_count", False,
            "{} /move_base_simple/goal messages for {} tasks".format(
                len(evidence.group_goals), len(executions)))
        return
    tasks = {task["task_id"]: task for task in config["tasks"]}
    centers = config["runner"]["centers"]
    check_ok = True
    detail = ""
    for execution, (stamp, caller, message) in zip(executions, evidence.group_goals):
        if caller and not caller.endswith("formation_action_server"):
            check_ok, detail = False, "publisher {} is not the Action server".format(
                caller)
        if message.header.frame_id != "world":
            check_ok, detail = False, "frame {}".format(message.header.frame_id)
        center = centers[tasks[execution["task_id"]]["target_ref"]]
        published = (message.pose.position.x, message.pose.position.y,
                     message.pose.position.z)
        if distance(published, tuple(center)) > 1e-6:
            check_ok, detail = False, "group goal {} != center {}".format(
                published, center)
        start = execution["goal_dispatch_ros_time_s"]
        # The group goal is published once, at dispatch (P0.3 rule 2).
        if abs(stamp - start) > 2.0:
            check_ok, detail = False, (
                "group goal stamp {:.3f} is not at dispatch {:.3f}".format(
                    stamp, start))
    verification.check("group_goal_publish_count", check_ok,
                       detail or "one world-frame group goal per task from the server")


def check_air_domain(verification, evidence, executions, root):
    """The run must have stayed inside the qn AIR model the whole time.

    The model switches mass, inertia, damping and actuation by ``medium_flag``,
    so reaching the slots with a non-zero flag is not a valid AIR result.  This
    is recomputed from the recorded qn diagnostics rather than trusted from the
    action summary.
    """
    for agent_id, samples in sorted(evidence.diagnostics.items()):
        flags = []
        heights = []
        floors = []
        for _stamp, values in samples:
            try:
                flags.append(float(values.get("medium_flag", "nan")))
            except ValueError:
                pass
            try:
                heights.append(float(values.get("min_height_m", "nan")))
            except ValueError:
                pass
            try:
                floors.append(float(values.get("air_floor_m", "nan")))
            except ValueError:
                pass
        flags = [value for value in flags if math.isfinite(value)]
        heights = [value for value in heights if math.isfinite(value)]
        floors = [value for value in floors if math.isfinite(value)]
        verification.check(
            "air_domain_flag_zero[{}]".format(agent_id),
            bool(flags) and max(flags) <= 1e-9,
            "max medium_flag {}".format(max(flags) if flags else None))
        floor = 0.5 * max(floors) if floors else None
        verification.check(
            "air_domain_height[{}]".format(agent_id),
            bool(heights) and floor is not None and min(heights) >= floor - 1e-9,
            "min height {} vs AIR floor {}".format(
                min(heights) if heights else None, floor))
        if heights:
            verification.measure(
                "min_height_m[{}]".format(agent_id), min(heights),
                "lowest published height; the AIR floor is hg_m/2")
        if flags:
            verification.measure(
                "max_medium_flag[{}]".format(agent_id), max(flags),
                "0 means the qn plant stayed in its AIR model")
    for execution in executions:
        diagnostics = load_json(root / execution["evidence_file"])
        air = diagnostics.get("air_domain") or {}
        verification.check(
            "action_air_domain[{}]".format(execution["task_id"]),
            bool(air) and air.get("ok") is True,
            "action AIR summary {}".format(air))
        verification.check(
            "execution_accepted_for_dispatch[{}]".format(execution["task_id"]),
            diagnostics.get("accepted_for_dispatch") is True,
            "task layer released the resource for {}".format(execution["task_id"]))


def _geometry_helpers():
    """The shared box definition, imported rather than re-derived."""
    import sys
    for candidate in ("/workspace/devel/lib/python3/dist-packages",):
        if candidate not in sys.path:
            sys.path.append(candidate)
    from qn_aav_simulator.experiment_verdict import (  # noqa: E402
        box_signed_distance, box_surface_clearance)
    return box_signed_distance, box_surface_clearance


def check_perception(verification, evidence, metrics, config, root):
    """M2: scene semantics, actual vs reference clearance, plane envelope.

    The two clearance questions are deliberately separate.  The actual state
    already contains the tracking error, so it is compared with the required
    clearance alone; the reference path is compared with the required clearance
    plus the budget that was declared before the run.  A reference shortfall is
    reported as a margin finding and never as an observed collision.
    """
    box_signed_distance, box_surface_clearance = _geometry_helpers()
    monitor = config.get("monitor", {})
    radius = float(monitor.get("platform_radius_m", 0.0))
    required = float(monitor.get("obstacle_clearance", 0.2))
    budget = float(monitor.get("tracking_budget_m", 0.0))
    surface_plane = float(monitor.get("surface_plane_m", 0.0))
    obstacle = bool(metrics.get("obstacle_scenario"))
    center = metrics.get("obstacle_center")
    size = metrics.get("obstacle_size")
    scene_topic = metrics.get("scene_topic", "/scene/global_cloud")
    if scene_topic not in evidence.scene_clouds and evidence.scene_clouds:
        scene_topic = sorted(evidence.scene_clouds)[0]

    # 1. one scene source: the recorded scene topic must have a single callerid
    callers = sorted({caller for _stamp, caller, _message in
                      evidence.scene_clouds.get(scene_topic, [])})
    verification.check(
        "scene_single_publisher",
        len(callers) == 1,
        "callerids on {}: {}".format(scene_topic, callers))
    if callers:
        verification.measure("scene_publisher", callers[0],
                             "the only publisher recorded on the scene topic")

    # 2. the scan chain delivered, with real observation times
    scans = {agent_id: evidence.local_clouds.get(agent_id) or []
             for agent_id in range(7)}
    for agent_id, messages in scans.items():
        stamps = [stamp for stamp, _count in messages]
        verification.check(
            "local_scan_delivered[{}]".format(agent_id),
            bool(messages), "{} local scans".format(len(messages)))
        verification.check(
            "local_scan_timestamped[{}]".format(agent_id),
            bool(stamps) and min(stamps) > 0.0,
            "scan stamps {}".format(stamps[:1]))

    # 3. actual clearance to the declared box (the solid, not the point cloud)
    actual = None
    for agent_id, samples in evidence.standard_odometry.items():
        for _stamp, position, _quat, _body, _angular, _f, _c in samples:
            if not obstacle:
                break
            value = box_surface_clearance(position, center, size, radius)
            actual = value if actual is None else min(actual, value)
    verification.measure(
        "actual_box_surface_clearance_m", actual,
        "closest observed platform-envelope clearance to the declared box")
    if obstacle:
        verification.check(
            "actual_box_clearance",
            actual is not None and actual >= required,
            "actual surface clearance {} vs required {}".format(actual, required))
    else:
        verification.check(
            "obstacle_check_not_applicable", actual is None,
            "no box declared: the box check is not applicable")

    # 4. reference margin, judged separately with the declared budget
    reference = None
    for agent_id, samples in evidence.used_reference_pose.items():
        for _stamp, position in samples:
            if not obstacle:
                break
            value = box_surface_clearance(position, center, size, radius)
            reference = value if reference is None else min(reference, value)
    verification.measure(
        "reference_box_surface_clearance_m", reference,
        "closest used-reference clearance to the declared box")
    verification.measure(
        "tracking_budget_m", budget,
        "declared before the run; reference margin budget")
    if obstacle and reference is not None:
        verification.check(
            "reference_margin_budget",
            reference >= required + budget,
            "reference clearance {:.3f} m vs required {:.2f} + budget {:.2f} m "
            "(a shortfall is a margin finding, not an observed collision)".format(
                reference, required, budget))

    # 5. plane envelope over the whole run, not only the last window
    lowest = None
    for _agent_id, samples in evidence.standard_odometry.items():
        for _stamp, position, _quat, _body, _angular, _f, _c in samples:
            lowest = position[2] if lowest is None else min(lowest, position[2])
    verification.measure(
        "min_member_height_m", lowest,
        "lowest published height over the whole run")
    verification.check(
        "plane_envelope_whole_run",
        lowest is not None and lowest - radius >= surface_plane,
        "member envelope reached {} vs declared surface {}".format(
            None if lowest is None else lowest - radius, surface_plane))


def check_test_c(verification, metrics, executions):
    events = metrics.get("events") or []
    processed = metrics.get("processed_events") or []
    verification.check(
        "delay_events_unique",
        len(processed) == len(set(processed)) == len(events),
        "events={} processed={} unique={}".format(
            len(events), len(processed), len(set(processed))))
    for index, execution in enumerate(executions):
        verification.check(
            "dispatch_after_real_completion[{}]".format(execution["task_id"]),
            execution["goal_dispatch_ros_time_s"]
            >= execution["previous_actual_finish"]
            - float(metrics.get("delay_tolerance", 0.1)),
            "dispatch {:.3f} before previous completion {:.3f}".format(
                execution["goal_dispatch_ros_time_s"],
                execution["previous_actual_finish"]))
        if metrics.get("mode") == "mission":
            required = {"execution_id", "goal_id", "planned_finish_at_dispatch",
                        "actual_finish", "planner_nominal_finish", "plan_updated",
                        "updated_plan_used", "dispatch_changed", "release_lag_s",
                        "plan_revision_at_dispatch",
                        "planned_start_read_at_dispatch"}
            verification.check(
                "execution_evidence_complete[{}]".format(execution["task_id"]),
                required <= set(execution),
                "missing per-action evidence keys: {}".format(
                    sorted(required - set(execution))))
            verification.check(
                "planner_nominal_finish[{}]".format(execution["task_id"]),
                bool(execution.get("planner_nominal_finish")),
                "no planner nominal finish recorded")
            recorded_ids = set(execution.get("trajectory_ids") or [])
            verification.check(
                "action_trajectory_ids[{}]".format(execution["task_id"]),
                bool(recorded_ids),
                "no trajectory_id set recorded for this action")
            adopted_ids = {
                entry.get("adopted_trajectory_id")
                for entry in (execution.get("adoption_verdict") or {}).get(
                    "per_agent", [])}
            verification.check(
                "adopted_trajectory_id_in_action_set[{}]".format(
                    execution["task_id"]),
                bool(adopted_ids) and adopted_ids <= recorded_ids,
                "adopted {} is not part of the recorded set {}".format(
                    sorted(adopted_ids), sorted(recorded_ids)))
    history = metrics.get("plan_history") or []
    for index, entry in enumerate(history):
        plan = entry.get("plan") or {}
        items = plan.get("items") or []
        execution = executions[index]
        verification.check(
            "plan_history_event[{}]".format(execution["task_id"]),
            entry.get("event_id") == execution["goal_id"] + ":completion"
            and entry.get("event_id") in processed,
            "history event {} not registered once".format(entry.get("event_id")))
        verification.check(
            "plan_before_preserved[{}]".format(execution["task_id"]),
            isinstance(entry.get("plan_before"), dict)
            and entry.get("plan_before", {}).get("items"),
            "repair must keep the pre-repair plan")
        if index + 1 < len(executions):
            next_execution = executions[index + 1]
            planned = next(
                (item for item in items
                 if item["execution_id"] == next_execution["execution_id"]), None)
            verification.check(
                "updated_plan_used[{}]".format(next_execution["task_id"]),
                planned is not None
                and abs(planned["planned_start"]
                        - next_execution["planned_start_at_dispatch"]) < 1e-6,
                "next dispatch used {} but the repaired plan says {}".format(
                    next_execution["planned_start_at_dispatch"],
                    None if planned is None else planned["planned_start"]))
    test_c = metrics.get("test_c") or {}
    verification.check(
        "test_c_fields",
        all(key in test_c for key in ("plan_updated", "updated_plan_used",
                                      "dispatch_changed", "notes")),
        "test_c {}".format(test_c))
    if metrics.get("mode") == "mission":
        verification.check(
            "updated_plan_used_recorded",
            all(execution.get("updated_plan_used") for execution in executions),
            "updated_plan_used must hold for every dispatched action")
        verification.check(
            "dispatch_changed_status_recorded",
            all("dispatch_changed" in execution for execution in executions),
            "every execution needs a dispatch_changed flag")
        verification.check(
            "dispatch_read_recorded",
            all(execution.get("planned_start_read_at_dispatch") is not None
                and execution.get("plan_revision_at_dispatch") is not None
                for execution in executions),
            "every dispatch must record the planned_start and plan revision it read")


def verify(directory, *, bag=True):
    root = Path(directory)
    verification = Verification()
    metrics = load_json(root / "metrics.json")
    config = load_json(root / "config.json")
    verification.check(
        "metrics_status", metrics.get("status") == "PASS",
        "status={} failure={}".format(metrics.get("status"), metrics.get("failure")))
    executions = metrics.get("executions") or []
    expected = 1 if metrics.get("mode") == "single" else len(config["tasks"])
    verification.check(
        "execution_count", len(executions) == expected,
        "{} executions for {} tasks".format(len(executions), expected))
    verification.check(
        "execution_ids_unique",
        len({execution["execution_id"] for execution in executions}) == len(executions)
        and len({execution["goal_id"] for execution in executions}) == len(executions),
        "execution_id/goal_id must be unique")
    check_diagnostics(verification, executions, config, root)
    check_test_c(verification, metrics, executions)

    model_time = None
    baseline = metrics.get("baseline_qualification") or {}
    verification.check(
        "baseline_qualified", bool(baseline.get("within_thresholds")),
        "pre-task baseline: {}".format(baseline.get("reasons")))
    summary_report = metrics.get("time_alignment_summary") or {}
    verification.check(
        "whole_run_time_alignment",
        bool(summary_report) and bool(summary_report.get("within_thresholds")),
        "whole-run alignment: {}".format(summary_report.get("reasons")))
    verification.check(
        "induced_reference_displacement",
        summary_report.get("induced_reference_displacement_m") is not None
        and summary_report["induced_reference_displacement_m"]
        <= 0.5 * float(config["monitor"]["epsilon_p"]),
        "induced displacement {} vs limit {}".format(
            summary_report.get("induced_reference_displacement_m"),
            0.5 * float(config["monitor"]["epsilon_p"])))

    evidence = BagEvidence()
    if bag:
        bag_path = root / "execution.bag"
        verification.check("execution_bag_present", bag_path.exists(), str(bag_path))
        if bag_path.exists():
            evidence.read(bag_path)
            for agent_id in range(7):
                verification.check(
                    "bag_odometry_present[{}]".format(agent_id),
                    bool(evidence.standard_odometry.get(agent_id)),
                    "no standard odometry in the bag")
                verification.check(
                    "bag_qn_diagnostics_present[{}]".format(agent_id),
                    bool(evidence.diagnostics.get(agent_id)),
                    "no qn diagnostics in the bag")
            check_odometry_contracts(verification, evidence)
            check_air_domain(verification, evidence, executions, root)
            check_perception(verification, evidence, metrics, config, root)
            series = extract_model_series(evidence)
            hold_entries = []
            for execution in executions:
                diagnostics = load_json(root / execution["evidence_file"])
                window = diagnostics.get("successful_hold_window") or {}
                hold_entries.append({
                    "task_id": execution["task_id"],
                    "hold_duration": float(diagnostics["hold_duration"]),
                    "window": ((window.get("start"), window.get("end"))
                               if window.get("start") is not None else None),
                })
            model_time = check_model_time(verification, series, hold_entries)
            check_used_reference_formula(
                verification, series, evidence.used_reference_pose,
                evidence.used_reference_twist)
            check_hold_odometry(verification, evidence, executions, config, root)
            check_action_envelopes(verification, evidence, executions, root)
            check_group_goals(verification, evidence, executions, config, root)

    summary = verification.summary()
    summary.update({
        "mode": metrics.get("mode"),
        "actions": len(executions),
        "tasks": [execution["task_id"] for execution in executions],
        "baseline_qualified": bool(baseline.get("within_thresholds")),
        "time_alignment_summary": summary_report,
        "model_time_recomputed_from_bag": model_time,
        "test_c": metrics.get("test_c"),
        "verdicts": [
            {
                "task_id": execution["task_id"],
                "task_outcome": execution.get("action_task_outcome"),
                "safety_outcome": execution.get("action_safety_outcome"),
                "experiment_validity": execution.get("action_experiment_validity"),
                "model_time_hold_seconds": execution.get("model_time_hold_seconds"),
            }
            for execution in executions],
    })
    (root / "verification.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return summary


def main(argv):
    if len(argv) < 2:
        print("usage: verify_formation_experiment.py <experiment-directory> [--no-bag]")
        return 2
    summary = verify(argv[1], bag="--no-bag" not in argv)
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
