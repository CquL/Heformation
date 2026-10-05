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

# At accelerated speed five_qualification uses the model /clock. It is not
# affected by host NTP adjustment; do not stop a shared server clock service.
# The legacy 1x wall-clock case retains its recorded NTP workaround.
systemctl is-active systemd-timesyncd > "$OUTPUT/clock-service-before.txt" || true
BEFORE="$(cat "$OUTPUT/clock-service-before.txt")"
USES_MODEL_CLOCK=$(python3 -c 'import sys; print(str(float(sys.argv[1]) != 1.0).lower())' "${JOINT_SIM_SPEED:-2.0}")
printf '%s\n' "$USES_MODEL_CLOCK" > "$OUTPUT/uses-model-clock.txt"
restore_clock_service() {
  if [[ "$BEFORE" == active && "$USES_MODEL_CLOCK" != true ]]; then
    sudo -n systemctl start systemd-timesyncd
  fi
  systemctl is-active systemd-timesyncd > "$OUTPUT/clock-service-after.txt" || true
}
trap restore_clock_service EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ "$BEFORE" == active && "$USES_MODEL_CLOCK" != true ]]; then
  sudo -n systemctl stop systemd-timesyncd
fi

echo '远域三类平台联合请求：平台从岸边共同部署，方法和支援由现有求解器选择。'
echo '请在独立任务控制台选区、生成方案并点击“确认并执行”；RViz 单独显示仿真。'
QN_SAME_SOURCE_ACCELERATION=true JOINT_VISUALIZE=true \
  JOINT_SIM_CPUSET="${JOINT_SIM_CPUSET:-0-23}" JOINT_PLANNING_BUDGET_S="${JOINT_PLANNING_BUDGET_S:-10}" \
  JOINT_VISUAL_TIMING_RELAX="${JOINT_VISUAL_TIMING_RELAX:-true}" \
  JOINT_PLANNER_CPUSET="${JOINT_PLANNER_CPUSET:-16-19}" \
  JOINT_VIEW_CPUSET="${JOINT_VIEW_CPUSET:-24-31}" JOINT_VIEW_HOLD_S="${JOINT_VIEW_HOLD_S:-120}" \
  bash "$PROJECT_ROOT/scripts/docker_run_joint_request.sh" "$OUTPUT"
