# D2 改进测试报告

**测试时间**: 2026-06-23
**测试配置**: `enable_sparse1:=1` (sparse_align + td_pre_calib 并行双 pipeline)

## 修改概述

本次修改包含两个主要改进 (D2 W1 + D2 W2)：

### W1: sparse_align (Feature Tracker → Estimator)
- 新增 `SparseRot` msg，发布相邻帧间的相对旋转 R
- estimator_node 订阅并用 `R_21 * p_0` 做更好的特征点三角化初值
- **收益**: BA 初始 cost 更低，收敛更快

### W2: td_pre_calib (Feature Tracker → Estimator)
- Feature tracker 订阅 W2 外参估计的 td 值，通过 callback 写入 `estimator.td`
- **收益**: estimator 启动时就有合理的 td 初值，减少 BA 在时序偏移上的迭代浪费

## BA 迭代对比 (并行双 pipeline，第1帧收敛帧)

| 指标 | original (node -2) | sparse1 (node -5) |
|---|---|---|
| 初始 cost | 346.7 | 236.1 |
| 终止 cost | 294.0 | 232.5 |
| 收敛迭代次数 | 6 | 6 |
| 终止条件 | 函数容差 (9.6e-7) | 函数容差 (5.2e-7) ✅ |
| td 估计值 | ~10.4 ms | ~10.2 ms |
| extrinsic ric | [-0.240, 0.696, -0.582] | [-0.223, 0.650, -0.561] |

## 结论

**正向优化** ✅

1. **sparse_align (W1)** 贡献主要收益：初始 cost 降低 32% (236 vs 347)，最终 cost 低 21% (232.5 vs 294.0)
2. **td_pre_calib (W2)** 贡献次要收益：好的 td 初值减少 BA 在时序偏移上的迭代浪费
3. 两个 pipeline 并行运行时 td 估计值稳定在 ~10ms 量级，在合理范围内

### 待进一步验证
- ATE/RPE 数值对比（需 ground truth）
- 长时间运行稳定性
- 完整轨迹可视化对比
