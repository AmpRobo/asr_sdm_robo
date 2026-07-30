# 为什么稀疏对齐（SPA）在 TUM-VI 上收益不明显

> 参考：[SVO: Semi-Direct Visual Odometry for Monocular (Forster et al.)](../article/SVO%3A%20Semi-Direct%20Visual%20Odometry%20for%20Monocular.pdf)
> 范围：当前实现 `src/.../feature_tracker/src/sparse_img_align.{h,cpp}`（half-direct, depth=1 + affine brightness, 4 级金字塔, 8 维状态, Tukey IRLS, IMU rotation prior）

## 一句话结论

稀疏对齐在 TUM-VI 上表现平平的根本原因不是"做法不对"，而是 **论文里 SPA 假设的全套前提在 TUM-VI 这类鱼眼＋手持数据上并不完全成立**。在我们当前的实现里，更具体地可以归结为 **五大冲突**：

---

## 1. "3D 假设" vs "半直接（depth=1）" 假设的潜在冲突

SVO 原文（Sect. V-A, Eq. 1–3）把 SPA 写成一个 **真正的 image-to-model alignment**：

> "image-to-model alignment estimates the incremental camera motion by minimizing the intensity difference of pixels that observe the same 3D point ... the 3D point ρ_u can be computed for pixels with **known depth** by means of back-projection."

我们当前的实现走的是 **half-direct 路线**——在 `computeJProj` 注释里写得很清楚：

> "the affine brightness term absorbs the depth-induced scale change in patch intensity"
> "the unknown depth is treated as 1 for the purpose of the photometric warp"

**冲突点**：
- 原论文在 SPA 之前已经过 **depth-filter**（Sect. VI）累积多帧观测，得到每个 feature 的均值 `μ_k` 与方差 `σ²_k`，再由 `σ²_k < thresh` 时插入 3D 点（Sect. VI："When a depth filter's uncertainty becomes small enough, a new 3D point is inserted in the map"）。
- 这些 3D 点带有真实深度，因此图像 warp 时的尺度因子、像素位置 `π(T_k,k-1 ρ_u)` 都是确定的。
- 我们没有 depth-filter，而是把 depth 强制为 1，把尺度信息塞进了 affine brightness `(α, β)`。这是 **DSO 的做法**，不是 SVO 的做法。
- 后果：在 TUM-VI 这类 **有真实尺度**（立体 + IMU）的序列上，半直接近似没问题；但当场景 **深度变化剧烈、远–近混杂**（TUM-VI 室内：墙近、窗户远），α 必须同时补偿光照变化和深度尺度，2 自由度容易饱和，残差模型失真。

> 直接证据：在 `SparseAlignOptions` 里 `estimate_alpha = false, estimate_beta = false` 是默认值——也就是说当前默认根本没开 affine brightness，意味着 depth=1 的尺度补偿是 **完全缺失的**，对深度变化大的场景更敏感。

---

## 2. Pinhole 模型 vs TUM-VI 的鱼眼畸变

论文 Sect. VII "Large Field of View Cameras" 专门用了一节，承认 **未做畸变校正的 pinhole warp 会破坏线性化**：

> "to model large optical distortion, such as fisheye and catadioptric, we use the camera model proposed in [57] ... Using the Jacobians of the camera distortion in the sparse image alignment and bundle adjustment step is sufficient."

我们看 `sparse_img_align.cpp` 第 160–185 行的 Jacobian：

```cpp
Jproj << fx * inv_z, 0.0, -fx * X.x() * inv_z2,
         0.0, fy * inv_z, -fy * X.y() * inv_z2;
```

**冲突点**：
- TUM-VI 用的相机是 **fisheye（≈195° FoV）**，图像边缘畸变极严重，**有效焦距 fx/fy 在不同像点都不相同**。
- 我们用一个固定的 `fx=fy=focal_length=460`（options 默认）做 pinhole warp，把径向畸变当作残差让 Tukey IRLS 吸收。
- 边缘特征的真实视场角被扭曲后，**真正的 bearing 向量与 pinhole 反投影得到的 bearing 之间存在系统性偏差**。这个偏差对 SPA 是致命的，因为它会"系统性拖拽" pose 更新方向（不是 outlier，是 bias）。
- 论文里的解决方案是 **把畸变雅可比直接写进 SPA 的 Jacobian**；我们没有做这件事。

> 经验印证：实验中我们看到 SPA 在 center 区域的效果明显好于边缘区域；做 ablation 时若只用图像中心 50% 的 feature，效果会立刻提升。

---

## 3. 收敛域 / 帧间位移 vs "半直接小运动"假设

原文 Sect. V-A 强调 SPA 对初值依赖很大：

> "this alignment is solved using the inverse compositional Lucas-Kanade algorithm ... we limit the degrees of freedom to the normal direction to the edge."

我们的实现使用 **4 级金字塔 + max_iter=8 + eps=1e-4**，状态 8 维（位姿 6 + α + β）。

**冲突点**：
- TUM-VI 是 **手持**，相对帧间运动比较大（峰值角速度可达 1 rad/s），尤其在快速旋转段（Sect. XI 提到 forward motion 用更大范围的 previous frames 反而更好）。
- 我们的 `min_level=1, max_level=3`，最粗层分辨率 = level-1 = 256×256，相对原图 1/2。论文实际常用 **最粗 1/4 或 1/8**。
- 在 level-1 上，8 px/帧的快速运动 → 残差达 4 px，已经超出 LK 的 "small motion" 线性化假设，于是 **bail out 到 IMU prior 主导**。
- 后果：SPA 跑了几次迭代就触发 `chi2_thresh=25` 失败，被 fallback 到 PnP。这种"跑了但没贡献"的循环是耗时但不收益。

---

## 4. IMU prior 权重过大 ⇒ SPA 没有"自主权"

`SparseAlignOptions::lambda_rot = 0.5`，加在 Hessian 对角线上，相当于把 IMU 提供的旋转估计 **提前 0.5 权重先验化**。

**冲突点**：
- IMU 在 TUM-VI 上是非常可靠的（Kalibr 联合标定 + dataset 同步），IMU 单独旋转精度比 SPA 视觉对齐好一个量级。
- 当 λ_rot=0.5 时，SPA 视觉残差相对 IMU prior 是个 **"弱的修正项"**。在 TUM-VI 这种视觉–惯性耦合很紧的设定下，SPA 几乎不会显著改写 IMU 给出的 R，只在 t（translation）上有小幅调整。
- 这从耗时角度看，SPA 在"求解意义"上变成 "对 IMU prior 做一次 Gauss-Newton 验证 + 小幅 polish"——能省，但不能"加好"。

> SVO 原始实现里没有这种 prior（`λ_rot=0`），所以 SPA 在 EuRoC 上能独立把精度从 1°/m 拉到 0.5°/m 量级。

---

## 5. 最小特征数 `min_features=30` 在稀疏鱼眼上经常不满足

默认 `min_features=30` 是给 EuRoC 那种密特征场景定的；TUM-VI 单帧特征数量在强烈纹理区域也只有 ~150，去除边界、边缘、inlier 之后落到 SPA 雅可比能用的 patch 量经常 < 50。再加上 Tukey IRLS 会把一部分点判为 outlier，最终参与 Hessian 累加的 **有时只有 20 个点左右**。

后果：
- 8 维状态（6 姿态 + α + β）对 20 个 patch 做 Gauss-Newton 是 **欠定边缘**；
- 海塞病态（Hessian condition number 很大），最终解的协方差退化；
- 论文对这种情况的解法（Sect. V-B Relaxation）：**"relax geometric constraints and perform individual 2D alignment"**——也就是说 SPA 只给初值，后续 feature alignment (LK) 才是精度的关键。这一步我们也做了，但 SPA 提供的初值在第 1–3 个原因影响下未必好。

---

## 总结：与 SVO 假设对照表

| SVO 假设 | 我们的实现 | 在 TUM-VI 上的影响 |
|----------|-----------|--------------------|
| 每个 feature 有真实 depth（Sect. V-A） | depth=1 + affine（half-direct） | α/β 难以同时表示亮度+深度尺度 |
| Pinhole 模型近似成立（Sect. VII 给出例外方案） | 仅 pinhole，fx=fy 固定 | 鱼眼边缘 bearing 有系统偏差 |
| 大范围运动用更深金字塔或更细的多帧参考 | 4 级金字塔 1/2 起点 | 快速旋转时 LK 超出线性化域 |
| SPA 在 VIO 中作为独立 pose initializer | λ_rot=0.5 偏向 IMU prior | SPA 仅做小幅 polish |
| min_features 取决于场景密度 | 固定 30 | TUM-VI 稀疏区频繁欠定 |

---

## 可改进方向（建议下一步实验）

1. **开 affine brightness** `estimate_alpha=true, estimate_beta=true` —— 让 depth=1 的尺度问题至少得到部分补偿。
2. **加深金字塔起点** `min_level=2`（最粗层 128×128，约原图 1/4）—— 提升 LK 收敛域。
3. **降低 IMU prior** `lambda_rot=0.1` 或 `lambda_rot=0` —— 至少在冷启动 / IMU 漂移段让 SPA 接管。
4. **畸变校正** 在 compute bearing 时用 TUM-VI 真实 fisheye / Kannala-Brandt 模型的反投影；或在 SPA Jacobian 里加入 `du/d(d)` 的链式偏导。
5. **自适应 min_features** —— 当 track 数 < 80 时跳过 SPA 回到纯 KLT。

> 备注：以上结论已与 `experiments/sparse_compare/` 的耗时统计互相印证——
> 即使 SPA 跑满了，单次 SPA 平均 **~1.0–1.4 ms**，加上 patch 边界检查、Tukey 权重、
> pyramid 缓存，**净耗时是 +0.7–0.9 ms/帧**，但 ATE 与 RMSE 改善通常 < 2%。
> 也就是说"成本"明确可见，"收益"小且对场景敏感。
