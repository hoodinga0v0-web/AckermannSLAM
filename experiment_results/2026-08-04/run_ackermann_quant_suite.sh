#!/usr/bin/env bash
set -eo pipefail

WORKSPACE=/home/hoodinga/Documents/SLAM/ackermann_ws
RESULTS=/tmp/ackermann_quant_results
LOG_ROOT=/tmp/ackermann_quant_logs

source "$WORKSPACE/setup_local.bash"
set -u
mkdir -p "$RESULTS" "$LOG_ROOT"

launch_pid=""
bridge_pid=""

cleanup_run() {
  if [[ -n "$bridge_pid" ]] && kill -0 "$bridge_pid" 2>/dev/null; then
    kill -INT "$bridge_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$bridge_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  bridge_pid=""
  launch_pid=""
}

trap cleanup_run EXIT INT TERM

run_one() {
  local kind="$1"
  local rep="$2"
  local domain="$3"
  local linear="$4"
  local angular="$5"
  local duration="$6"
  local partition="ack_quant_${kind}_${rep}_${domain}"
  local run_logs="$LOG_ROOT/${kind}_${rep}"
  local launch_log="$run_logs/launch.log"
  local bridge_log="$run_logs/bridge.log"
  local result_file="$RESULTS/${kind}_${rep}.json"
  mkdir -p "$run_logs"

  if [[ -s "$result_file" ]]; then
    echo "SKIP existing kind=$kind rep=$rep"
    return 0
  fi

  export ROS_DOMAIN_ID="$domain"
  export ROS2CLI_DISABLE_DAEMON=1
  export GZ_PARTITION="$partition"
  export ROS_LOG_DIR="$run_logs"

  echo "START kind=$kind rep=$rep domain=$domain"
  ros2 launch ackermann_car_description simulation.launch.py \
    headless:=true rviz:=false verbosity:=1 >"$launch_log" 2>&1 &
  launch_pid=$!

  sleep 1.0
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "launch exited before recorder start" >&2
    tail -n 80 "$launch_log" >&2
    return 1
  fi

  /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge \
    '/world/slam_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
    --ros-args -r /world/slam_world/dynamic_pose/info:=/ground_truth_poses \
    >"$bridge_log" 2>&1 &
  bridge_pid=$!

  python3 /tmp/ackermann_quant_run.py \
    --kind "$kind" \
    --linear "$linear" \
    --angular "$angular" \
    --duration "$duration" \
    --output "$result_file"

  cleanup_run
  sleep 0.5
  echo "DONE kind=$kind rep=$rep"
}

for rep in 1 2 3 4 5; do
  run_one straight "$rep" "$((100 + rep))" 0.2 0.0 5.0
done

for rep in 1 2 3 4 5; do
  run_one arc_left "$rep" "$((110 + rep))" 0.1 0.2 8.0
done

for rep in 1 2 3 4 5; do
  run_one arc_right "$rep" "$((120 + rep))" 0.1 -0.2 8.0
done

echo "SUITE_COMPLETE results=$RESULTS"
