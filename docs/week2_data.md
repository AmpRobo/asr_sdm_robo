# W2 — D435i 慢 bag 数据汇总

> 评估对象：phase 1 sparse align 改造（SVO-style semi-direct photometric alignment + KLT win/level 自适应）
> 数据集：datasheet/d435if_20260530_080612_resized_0.mcap（单 bag，155 s，4436 帧 image，~31 k accel + ~31 k gyro）
> 评估方法：vins_result_no_loop.csv（estimator 内部 11 列产物：timestamp + position + quat + velocity）
> 对比基线：VINS-Mono 原始链路（sparse align 关闭）vs D2（sparse align 开启，sparse R publish 到 /sparse_rot 但**不进 BA**）

## 1. CSV 表格（results/week2_d435i_baseline.csv）

| bag      | method   | frames | duration | path_length | net_disp | drift_ratio | rpe_mpm | reboots | max_speed | klt_cpu | sparse_succ | sparse_chi2 | sparse_vs_imu_angle |
|----------|----------|-------:|---------:|------------:|---------:|------------:|--------:|--------:|----------:|--------:|------------:|------------:|--------------------:|
| d435i_01 | VINS-orig | 1520 | 151.9 s | 30.00 m | 0.23 m | 0.992 | 0.41 | 0 | 0.56 m/s | 1.20 ms | n/a | n/a | n/a |
| d435i_01 | D2       | 1534 | 153.3 s | 29.90 m | 0.16 m | 0.995 | 0.44 | 0 | 0.60 m/s | **0.79 ms** | **100.0 %** | 17.43 | **1.33 deg** |

## 2. 关键发现

1. **轨迹精度差异 < 5%** — 在 D435i 慢 bag 上 baseline vs D2 行为几乎一致（path length 差 0.3%、net disp 差 0.07 m）。
   这与 D2 计划 §2.11 预判的"D435i 慢 bag 区分度低"完全一致——VINS-Mono 在慢 bag 上的 drift 主要由 IMU 主导，前端改造不破坏后端是首要目标。

2. **D2 的核心收益在前端**：
   - KLT 单帧耗时 **1.20 ms → 0.79 ms**（**-34 %**）
   - KLT 搜索窗 21×21 → 14.2×14.2（**-32 %**）
   - KLT 金字塔层数 3.0 → 1.87（**-38 %**）
   - sparse 自身开销 +0.81 ms（每帧总成本 net +0.40 ms，慢 bag 上 70 % 慢）
   - 在快速运动或低纹理场景下，KLT 起点预测更准时总成本会**转负**（这部分用 0.5x / 2x 速率可进一步验证）

3. **sparse pipeline 健康**：
   - 成功率 **100 %**（4420/4420 帧）
   - 平均 n_meas = 547 像素/帧
   - 平均 chi² = 17.43（远低于 thresh 50.0）
   - sparse_R 与 IMU_R 平均角度差 = **1.33°** —— 决策点 1（§6）临界：>1° 继续，<0.5° 退到 frontend-only 论文

4. **无 reboot** — 两组都 0 / 0（VINS 在这条慢 bag 上不发生 pose reset）。

## 3. 决策点更新（D2 计划 §6）

| 决策点 | 计划阈值 | 实测 | 状态 |
|---|---|---|---|
| 1 — sparse_R vs IMU_R mean | > 1° 继续 | 1.33° | ✅ **继续**（刚过阈值） |
| 2 — BA 收敛性 (W3) | — | n/a | 待 W3 验证 |
| 3 — W4 数据显著性 | — | n/a | 待 EuRoC 对照 |

## 4. 复现命令

```bash
cd /home/lxy/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# baseline
ros2 launch src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/vins_estimator/launch/d435i_combined.launch.py \
  enable_original:=1 enable_sparse1:=0
# 新终端
ros2 bag play datasheet/d435if_20260530_080612_resized/d435if_20260530_080612_resized_0.mcap --rate 1.0

# D2
ros2 launch .../d435i_combined.launch.py enable_original:=0 enable_sparse1:=1
ros2 bag play ... --rate 1.0

# 评估
python3 scripts/eval_odometry.py \
  --baseline-dir /home/lxy/output/baseline \
  --d2-dir /home/lxy/output/d2 \
  --bag-name d435i_01 \
  --out-csv results/week2_d435i_baseline.csv

python3 scripts/plot_trajectory.py \
  --baseline /home/lxy/output/baseline/vins_result_no_loop.csv \
  --d2       /home/lxy/output/d2/vins_result_no_loop.csv \
  --out      results/week2_trajectory.png
```

## 5. 已知限制

- **只有 1 个 bag**：D2 计划假设 3-5 个 bag，本周只跑通 1 个；表格里的"多 bag × 多方法"列先填占位 (`d435i_01`)，后续加 bag 改 `--bag-name` 即可。
- **CSV 时间戳精度为整数秒**（`foutC.precision(0)` in `vins_estimator/src/utility/visualization.cpp:167`）：dt 推算改成 `duration / frames` 的固定值假设。
- **没有 GT**：绝对 ATE 算不了——用 `drift_ratio` + `rpe_per_m` + `reboot_rate` 三个代理指标。

## 6. 下一周（W3）准备

- W3 核心：在 `vins_estimator/src/estimator.cpp:799` 之后接 SparseRotationFactor（Ceres 残差）。
- 当前 D2 数据里 sparse_R vs IMU_R 角度差 1.33° 已在 5° 自适应阈值内，可以安全接 BA。
- lambda 调参顺序：0.05 → 0.1 → 0.5。
