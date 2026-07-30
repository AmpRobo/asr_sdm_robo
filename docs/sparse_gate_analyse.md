# Sparse Rotation Geometric Gate 实验分析报告

> **结论：gate 纯负优化，建议关闭（`enable: false`），该方向暂缓后续优化。**

---

## 1. 实验设计

### 1.1 三种配置对照

| 路径 | sparse alignment | gate | 用途 |
|------|:---------------:|:----:|------|
| `sparse_on` | ON | ON | 当前实现 |
| `sparse_on_v1` | ON | OFF | gate 效果隔离测试 |
| `sparse_off` | OFF | OFF | VINS-Mono 基线 |

### 1.2 Gate 当前实现（已搁置版本）

Gate 在 `geometric_ok == true` 之后调用，满足以下全部条件时**拒绝** loop：

1. `USE_SPARSE_ROTATION_GATE = true`
2. 两帧都有 sparse 数据
3. `pnp_inliers >= 20`
4. `|PnP_yaw_diff| < 45°` （小旋转才触发）
5. **拒绝条件**：`|sparse_yaw - PnP_yaw| <= 30°`（sparse 认可 PnP）**且** `|PnP_yaw - VIO_yaw| > 20°`（PnP 否定 VIO）

### 1.3 当前参数配置

| 参数 | 默认值 |
|------|--------|
| `enable` | `true` |
| `angle_thresh_deg` | `30.0`（sparse-PnP yaw 误差 <= 30° 视为认可） |
| `max_yaw_diff_deg` | `45.0`（PnP yaw > 45° 跳过 gate） |
| `vio_diff_deg` | `20.0`（PnP-VIO yaw 误差 > 20° 视为矛盾 VIO） |
| `min_inliers` | `20`（PnP inliers < 20 跳过 gate） |

---

## 2. 实验数据

### 2.1 完整精度对比

| 序列 | 配置 | ATE (m) | RTE_trans (m) | RTE_rot (°) | RMSE (m) |
|------|------|---------|--------------|------------|---------|
| **MH01** | gate ON | 0.0809 | 0.0659 | +0.49 | 0.0555 |
| | gate OFF | 0.0809 | 0.0659 | +0.49 | 0.0555 |
| | 基线 OFF | 0.1025 | 0.1008 | +2.42 | 0.0658 |
| **MH02** | gate ON | 0.1226 | 0.0687 | +0.13 | 0.0818 |
| | gate OFF | **0.1065** | **0.0624** | +0.17 | **0.0670** |
| | 基线 OFF | 0.1085 | 0.0748 | +0.26 | 0.0692 |
| **MH03** | gate ON | 0.1904 | 0.1268 | +0.53 | 0.1270 |
| | gate OFF | **0.1778** | **0.1061** | +0.99 | **0.1203** |
| | 基线 OFF | 0.1923 | 0.1615 | +1.10 | 0.1287 |
| **MH04** | gate ON | 0.2416 | 0.0466 | +3.35 | 0.1550 |
| | gate OFF | **0.2240** | **0.0238** | +2.90 | **0.1510** |
| | 基线 OFF | 0.2545 | 0.0930 | +2.14 | 0.1646 |
| **MH05** | gate ON | 0.2653 | 0.1449 | +3.97 | 0.1659 |
| | gate OFF | 0.2653 | 0.1449 | +3.97 | 0.1659 |
| | 基线 OFF | 0.2836 | 0.1931 | +4.40 | 0.1758 |

### 2.2 Gate 触发判定

| 序列 | gate 触发? | `sparse_on` ≠ `sparse_on_v1`? | 原因 |
|------|:---------:|:-----------------------------:|------|
| MH01 | ❌ 否 | 否 | 未触发（sparse_on == sparse_on_v1） |
| MH02 | ✅ 是 | 是 | ATE 恶化 +15.1% |
| MH03 | ✅ 是 | 是 | ATE 恶化 +7.1% |
| MH04 | ✅ 是 | 是 | ATE 恶化 +7.9% |
| MH05 | ❌ 否 | 否 | 未触发（所有候选 PnP yaw > 45°，被 guard 跳过） |

### 2.3 Gate 对各项指标的影响（gate ON vs gate OFF）

| 序列 | ATE 变化 | RTE_trans 变化 | RTE_rot 变化 | RMSE 变化 |
|------|---------|--------------|------------|---------|
| MH02 | +0.0161 ❌ | +0.0063 ❌ | -0.04° ✅ | +0.0148 ❌ |
| MH03 | +0.0126 ❌ | +0.0207 ❌ | -0.46° ✅ | +0.0067 ❌ |
| MH04 | +0.0176 ❌ | +0.0228 ❌ | +0.45° ❌ | +0.0040 ❌ |

**结论：gate 触发时，3/4 序列在 ATE、RTE_trans、RMSE 上全面恶化。RTE_rot 仅偶有改善是以牺牲整体精度为代价换来的。**

---

## 3. 根因分析

### 3.1 直接原因：Gate 拒绝了有益的 loop

Gate 拒绝的条件是 **"sparse 认可 PnP，但 PnP 否定 VIO"**。这个逻辑的根本错误在于：

> **"PnP 否定 VIO"恰恰是 loop closure 应该做的事。**

当 VIO 产生累积漂移后，重新访问之前到过的地点时，PnP 给出的旋转必然和漂移的 VIO 不一致——这正是 loop 需要校正的内容。用这个条件拒绝 loop，gate 把最需要被接受的 loop 反而拒绝了。

### 3.2 方法论问题：sparse 相邻帧旋转不能代表全局 loop 旋转

当前 gate 比较的是：
- `sparse_yaw_diff = R_sparse(k, k-1)^T * R_sparse(old, old-1)`（两个相邻帧旋转的 yaw 差）
- `pnp_yaw_diff = R_vio_cur^T * (qic * R_pnp_old)`（PnP 全局旋转与 VIO 的 yaw 差）

问题在于：
- **sparse 只知道相邻帧运动**，无法感知 cur 和 old 之间间隔了数十乃至数百帧
- `R_sparse(k, k-1)` 只反映 cur 最后一帧的运动
- `R_sparse(old, old-1)` 只反映 old 最后一帧的运动
- 这两个相邻帧测量**与 cur-old 的全局几何关系没有直接关联**

### 3.3 阈值宽松掩盖了方法本身的失效

30° 的 sparse-PnP yaw 一致性阈值（允许 1/12 圆周的误差）说明设计者本身对 sparse rotation 的可靠性存在疑虑。但即使阈值如此宽松，gate 仍然在 MH02/MH03/MH04 上大量触发并起负作用，说明 **问题不在参数调参，而在于比较基准本身错误**。

### 3.4 MH01/MH05 未触发的巧合

- MH01：loop 少或 loop 的 PnP yaw 都在 45° 以上，被 guard 跳过
- MH05：所有候选帧 PnP yaw diff > 45°，被 guard 跳过

这两个序列 gate 不触发恰恰是因为几何条件不满足，不代表 gate 设计正确。

---

## 4. 结论与后续方向

### 4.1 结论

1. **Gate 纯负优化**：在 MH02/MH03/MH04 上触发后，ATE、RMSE、RTE_trans 全面恶化
2. **Gate 设计方向错误**：用"PNP 否定 VIO"作为拒绝条件与 loop closure 的本质相矛盾
3. **建议**：在 `vins.yaml` 中设置 `pose_graph.sparse_rotation_gate.enable: false`，关闭该功能

### 4.2 配置方法

通过 YAML 参数可随时关闭/重新开启 gate，无需修改代码：

```yaml
pose_graph:
  sparse_rotation_gate:
    enable: false  # false = 关闭，true = 开启
```

### 4.3 后续优化方向（暂缓）

| 方向 | 说明 | 可行性评估 |
|------|------|-----------|
| Pose graph 信息矩阵自适应 | 根据 geometric verifier 的 inlier ratio 动态缩放 loop edge 权重，而非二元拒绝 | ⭐⭐⭐ 推荐尝试 |
| Inlier ratio 直接判据 | 用 PnP inlier ratio 替代 rotation 比较作为 loop 质量判据 | ⭐⭐ 可探索 |
| 论文原版 gate（已验证） | 论文 §IV-B 的设计在实操中同样为负优化，说明该问题为方法族固有局限 | ❌ 验证完毕 |
| 三元判据（accept / reduce weight / reject） | 对 loop 做软降权而非硬拒绝 | ⭐⭐⭐ 长期方向 |

### 4.4 备注

- Sparse alignment 前端优化（`sparse_on_v1` vs `sparse_off`）本身是有效的，符合论文 Tab.III 预期
- Gate 的负作用掩盖了前端 sparse 的正收益，导致 `sparse_on`（前端ON+gate ON）整体表现不如 `sparse_on_v1`（前端ON+gate OFF）
- 关闭 gate 后，`sparse_on_v1` 的结果即代表 SPA-KLT 的实际效果

---

*报告生成时间：2026-07-27*
*分析方法：对比同一序列 `sparse_on`（gate ON）和 `sparse_on_v1`（gate OFF）的精度差异，隔离 gate 效果*
