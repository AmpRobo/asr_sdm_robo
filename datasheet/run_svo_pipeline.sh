#!/bin/bash
# Orchestration script to run SVO, play bag, record, and then run analysis.
# Usage: bash run_svo_pipeline.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="/home/lxy/asr_sdm_robo/datasheet/airground_rig_s3_ros2"
OUTPUT_DIR="/home/lxy/asr_sdm_robo/datasheet/svo_analysis_output"
REC_DIR="$OUTPUT_DIR/svo_recording"
RATE=0.2

source /opt/ros/jazzy/setup.bash
source /home/lxy/asr_sdm_robo/install/setup.bash

echo "=== Cleaning up old processes ==="
pkill -f "ros2 bag" || true
pkill -f "vo" || true
pkill -f "ros2 launch" || true
sleep 3

echo "=== Creating output directories ==="
mkdir -p "$OUTPUT_DIR" "$REC_DIR"
rm -rf "$REC_DIR"/*

echo "=== Step 1: Starting SVO ==="
ros2 launch svo_ros test_rig3.launch.py \
    rviz:=false rviz_sw:=false fast_type:=10 max_queue_size:=2 drop_frames:=true \
    > "$OUTPUT_DIR/svo.log" 2>&1 &
SVO_PID=$!
echo "SVO PID: $SVO_PID"
sleep 8

echo "=== Step 2: Starting recording ==="
ros2 bag record /svo/pose /Rig -o "$REC_DIR/svo_bag" \
    > "$OUTPUT_DIR/recording.log" 2>&1 &
REC_PID=$!
echo "Recording PID: $REC_PID"
sleep 3

echo "=== Step 3: Playing bag (rate=$RATE, ~$((1624/RATE/60)) min) ==="
ros2 bag play "$BAG_DIR" --rate $RATE \
    > "$OUTPUT_DIR/bag_play.log" 2>&1 &
BAG_PID=$!
echo "Bag play PID: $BAG_PID"

echo "=== Waiting for bag to finish (poll every 30s) ==="
BAG_DURATION_S=$((1624 / RATE))
for i in $(seq 1 100); do
    sleep 30
    if ! kill -0 $BAG_PID 2>/dev/null; then
        echo "Bag play finished after ${i}*30s"
        break
    fi
    echo "  Still playing... ${i}*30s elapsed"
done

echo "=== Step 4: Stopping SVO and recording ==="
kill $REC_PID 2>/dev/null || true
kill $SVO_PID 2>/dev/null || true
sleep 3

echo "=== Step 5: Finding recording files ==="
REC_BAG=$(find "$REC_DIR" -name "*.mcap" -o -name "*.db3" 2>/dev/null | head -1)
echo "Recording bag: $REC_BAG"
echo "Recording dir contents:"
ls -lh "$REC_DIR/"

echo ""
echo "=== Step 6: Running analysis ==="
python3 "$SCRIPT_DIR/run_svo_and_compare.py" \
    --bag "$BAG_DIR" \
    --recording "$REC_BAG" \
    --output "$OUTPUT_DIR" \
    2>&1 | tee "$OUTPUT_DIR/analysis.log"

echo ""
echo "=== Pipeline complete! ==="
echo "Output: $OUTPUT_DIR"
ls -lh "$OUTPUT_DIR/"
