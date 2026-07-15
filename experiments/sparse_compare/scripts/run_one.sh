#!/bin/bash
# Run one VINS experiment: launch + bag play + save CSV.
#
# Usage:
#   run_one.sh --mode <on|off> --bag <path-to-bag-dir-or-db3> --seq <MH0X>
#
# Behaviour:
#   * launches `vins_estimator` with `enable_sparse:=(1|0)` (does NOT touch vins.yaml)
#   * plays the bag at 1.0x, waits for it to finish naturally
#   * saves output/vins_result_loop.csv to output/<seq>/sparse_<mode>/vins_sparse_<mode>.csv
#
# Example:
#   ./run_one.sh --mode on  --bag datasheet/MH_04_difficult_ros2 --seq MH04
#   ./run_one.sh --mode off --bag datasheet/MH_04_difficult_ros2 --seq MH04
set -e

MODE=""
BAG=""
SEQ=""
EXTRA_WAIT=20      # extra seconds after bag ends, for pose_graph to flush

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE=$2; shift 2 ;;
    --bag)  BAG=$2;  shift 2 ;;
    --seq)  SEQ=$2;  shift 2 ;;
    --extra-wait) EXTRA_WAIT=$2; shift 2 ;;
    -h|--help)
      sed -n '3,16p' "$0"; exit 0 ;;
    *) echo "Unknown arg: $1"; exit 2 ;;
  esac
done

if [ -z "$MODE" ] || [ -z "$BAG" ] || [ -z "$SEQ" ]; then
  echo "Usage: $0 --mode <on|off> --bag <bag_path> --seq <MH0X> [--extra-wait SEC]"
  exit 1
fi

WS=/home/lxy/asr_sdm_robo
OUT_DIR=$WS/output/$SEQ/sparse_${MODE}
VINS_CSV=$WS/output/vins_result_loop.csv
VINS_CSV_NO=$WS/output/vins_result_no_loop.csv
LOG=/tmp/${SEQ}_sparse_${MODE}.log

if [ "$MODE" = "on" ]; then
  ENABLE_SPARSE=1
elif [ "$MODE" = "off" ]; then
  ENABLE_SPARSE=0
else
  echo "ERROR: --mode must be 'on' or 'off' (got '$MODE')"
  exit 1
fi

mkdir -p "$OUT_DIR"

echo "=== [$SEQ / sparse-$MODE] Starting VINS (enable_sparse:=$ENABLE_SPARSE) ==="
echo "    bag : $BAG"
echo "    out : $OUT_DIR"
echo "    log : $LOG"

# Clean previous CSV so we don't keep stale rows
rm -f "$VINS_CSV" "$VINS_CSV_NO"

# Source ROS + workspace
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# Launch VINS in background
ros2 launch vins_estimator vins_launch.py "enable_sparse:=$ENABLE_SPARSE" \
  > "$LOG" 2>&1 &
VINS_LAUNCH_PID=$!
echo "Launch PID: $VINS_LAUNCH_PID"

# Wait for nodes to come up
sleep 8

# Play bag in background
echo "=== [$SEQ / sparse-$MODE] Playing bag ==="
ros2 bag play "$BAG" -r 1.0 >> "$LOG" 2>&1 &
BAG_PID=$!
echo "Bag PID: $BAG_PID"

# Wait for the bag player to finish naturally
echo "Waiting for bag to finish..."
wait $BAG_PID
echo "Bag finished."

# Let pose_graph flush
echo "Cooling down ${EXTRA_WAIT}s..."
sleep $EXTRA_WAIT

# Tear down everything we started
echo "=== [$SEQ / sparse-$MODE] Killing VINS nodes ==="
for pid in $(pgrep -f "vins_estimator|pose_graph|feature_tracker|rviz2"); do
  # avoid killing this very script's tree by filtering on log path indirectly:
  # if its cmdline contains our seq token, kill; otherwise skip.
  cmdline=$(cat /proc/$pid/cmdline 2>/dev/null | tr '\0' ' ' || true)
  if [[ "$cmdline" == *"$SEQ"* ]] || [[ "$cmdline" == *"vins_launch"* ]] \
     || [[ "$cmdline" == *"vins_estimator_node"* ]] \
     || [[ "$cmdline" == *"pose_graph"* ]] \
     || [[ "$cmdline" == *"feature_tracker_node"* ]] \
     || [[ "$cmdline" == *"rviz2"* ]] \
     || [[ "$cmdline" == *"/ros2 bag play"* ]]; then
    echo "  killing PID $pid"
    kill $pid 2>/dev/null || true
  fi
done

# Make sure ros2 launch (the parent python) is gone too
kill $VINS_LAUNCH_PID 2>/dev/null || true

sleep 3

# Save CSV
echo "=== [$SEQ / sparse-$MODE] Saving CSV ==="
if [ -s "$VINS_CSV" ]; then
  cp "$VINS_CSV"  "$OUT_DIR/vins_sparse_${MODE}.csv"
  cp "$VINS_CSV_NO" "$OUT_DIR/vins_sparse_${MODE}_no_loop.csv" 2>/dev/null || true
  echo "  saved $(wc -l < $OUT_DIR/vins_sparse_${MODE}.csv) lines -> $OUT_DIR/vins_sparse_${MODE}.csv"
  echo "  first: $(head -1 $VINS_CSV)"
  echo "  last : $(tail -1 $VINS_CSV)"
else
  echo "ERROR: $VINS_CSV is empty or missing! See $LOG."
  exit 1
fi

echo "=== [$SEQ / sparse-$MODE] Done ==="