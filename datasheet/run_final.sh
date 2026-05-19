#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/lxy/asr_sdm_robo/install/setup.bash

LOGDIR="/home/lxy/asr_sdm_robo/datasheet/svo_recording_final"
BAG="/home/lxy/asr_sdm_robo/datasheet/airground_rig_s3_ros2"
OUT="$LOGDIR/svo_bag"

mkdir -p "$LOGDIR"
rm -f "$LOGDIR"/*

echo "[$(date)] === Step 1: 启动 SVO ==="
ros2 launch svo_ros test_rig3.launch.py \
    rviz:=false rviz_sw:=false fast_type:=10 max_queue_size:=2 drop_frames:=true \
    > "$LOGDIR/svo.log" 2>&1 &
SVO_PID=$!
echo "SVO PID=$SVO_PID, 等待 8 秒..."
sleep 8

echo "[$(date)] === Step 2: 启动录制 ==="
ros2 bag record /svo/pose /Rig -o "$OUT" \
    > "$LOGDIR/rec.log" 2>&1 &
REC_PID=$!
echo "录制 PID=$REC_PID, 等待 3 秒..."
sleep 3

echo "[$(date)] === Step 3: 播放 bag (rate=1, ~162秒) ==="
ros2 bag play "$BAG" --rate 1.0 \
    > "$LOGDIR/bag.log" 2>&1 &
BAG_PID=$!
echo "Bag PID=$BAG_PID"

echo "[$(date)] === 等待 bag 播放完毕 (最多 300 秒) ==="
for i in $(seq 1 30); do
    sleep 10
    if ! kill -0 $BAG_PID 2>/dev/null; then
        echo "[$(date)] Bag 播放完毕, 第 ${i}*10 秒"
        break
    fi
    echo "[$(date)] 播放中... 第 ${i}*10 秒"
done

echo "[$(date)] === Step 4: 关闭录制和 SVO ==="
kill $REC_PID 2>/dev/null || true
sleep 2
kill $SVO_PID 2>/dev/null || true
sleep 2

echo "[$(date)] === Step 5: 查找录制文件 ==="
RECFILE=$(find "$LOGDIR" -name "*.mcap" -o -name "*.db3" 2>/dev/null | head -1)
echo "录制文件: $RECFILE"
ls -lh "$LOGDIR/"

echo "[$(date)] === Step 6: 运行分析 ==="
cd /home/lxy/asr_sdm_robo/datasheet
python3 run_svo_and_compare.py \
    --bag "$BAG" \
    --recording "$RECFILE" \
    --output /home/lxy/asr_sdm_robo/datasheet/svo_analysis_output \
    2>&1 | tee "$LOGDIR/analysis.log"

echo "[$(date)] === 完成 ==="
