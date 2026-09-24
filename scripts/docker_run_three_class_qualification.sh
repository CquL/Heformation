#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_run_three_class_qualification.sh new-output-directory}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
if [[ -e "$OUTPUT/metrics.json" ]]; then
  echo "Choose a new output directory: $OUTPUT" >&2
  exit 2
fi

# This qualification isolates the real three-class execution path. The normal
# joint request and executor file remain unchanged and still compare AAV WATER.
python3 - "$PROJECT_ROOT" "$OUTPUT" <<'PY'
import pathlib,sys,yaml
root,output=map(pathlib.Path,sys.argv[1:])
config=yaml.safe_load((root/'integration/qn_aav_simulator/config/joint_request_executors.yaml').read_text())
config['executors']=[unit for unit in config['executors']
                     if unit['executor_id']!='aav_1_native']
(output/'qualification-executors.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
PY

# The host NTP daemon caused a recorded ROS wall-clock rollback. Keep its
# original state and restore it on normal exit or Ctrl-C; model time gates stay
# unchanged. This is an experiment condition, not a mission capability.
systemctl is-active systemd-timesyncd > "$OUTPUT/clock-service-before.txt" || true
BEFORE="$(cat "$OUTPUT/clock-service-before.txt")"
restore_clock_service() {
  if [[ "$BEFORE" == active ]]; then
    sudo -n systemctl start systemd-timesyncd
  fi
  systemctl is-active systemd-timesyncd > "$OUTPUT/clock-service-after.txt" || true
}
trap restore_clock_service EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ "$BEFORE" == active ]]; then
  sudo -n systemctl stop systemd-timesyncd
fi

echo '三类接线资格诊断：本次不提供AAV水下备选；普通联合请求配置保持不变。'
echo '打印具体计划后，请在终端输入 yes 才会派发。'
QN_SAME_SOURCE_ACCELERATION=true JOINT_VISUALIZE=true \
  JOINT_SIM_CPUSET=0-11 JOINT_PLANNING_BUDGET_S=180 \
  JOINT_PLANNER_CPUSET=12-15 JOINT_VIEW_CPUSET=24-31 JOINT_VIEW_HOLD_S=120 \
  JOINT_EXECUTORS_FILE=/experiments/current/qualification-executors.yaml \
  bash "$PROJECT_ROOT/scripts/docker_run_joint_request.sh" "$OUTPUT"
