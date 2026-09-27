# asr_sdm_local_path_modifier

Topological local path search around obstacles, in the style of Fast-Planner
`TopologyPRM`.

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

### Overview

The package builds one shared library, `asr_sdm_local_path_modifier`, that
contains two independent classes, plus an RViz test node.

| Class | Header | Used by |
|---|---|---|
| `TopologyPRM` | `asr_sdm_local_path_modifier/topo_prm.h` | `asr_sdm_planning_manager` (production replanning) |
| `TopoPathModifier` | `asr_sdm_local_path_modifier/topo_path_modifier.hpp` | Test node `local_path_modifier_test`; not linked into the manager yet |

Both search a local roadmap for topologically distinct detours around the
blocked part of a path. They do **not** share code or parameters.
Non-uniform B-spline optimization is not part of this package; the manager
runs it on the paths `TopologyPRM` returns. The grid A* front end has its own
package, [`asr_sdm_guidance_planner`](../asr_sdm_guidance_planner/).

### Package structure

```text
asr_sdm_local_path_modifier/
├── include/asr_sdm_local_path_modifier/
│   ├── topo_prm.h               # TopologyPRM
│   └── topo_path_modifier.hpp   # TopoPathModifier
├── src/
│   ├── topo_prm.cpp
│   └── topo_path_modifier.cpp
├── test/local_path_modifier_test.cpp
├── config/local_path_modifier.yaml
├── launch/local_path_modifier_test.launch.py
└── rviz/local_path_modifier_test.rviz
```

### TopologyPRM (production)

`PlanningManager::topoReplan` calls it when the current local B-spline is in
collision:

```text
PlanningManager::findCollisionRange
  -> start, end, start_pts, end_pts
TopologyPRM::findTopoPaths(start, end, start_pts, end_pts, ...)
  createGraph       sample region + Visibility PRM (guards / connectors)
  searchPaths       DFS from start to end, keep paths with fewest nodes
  shortcutPaths     visibility shortcut, push off obstacles along the ESDF gradient
  pruneEquivalent   drop topologically equivalent paths
  selectShortPaths  keep the shortest ones, splice start_pts / end_pts back on
```

It reads the ESDF map through `EDTEnvironment` (`asr_sdm_esdf_map`). Its
parameters are declared on the manager node under `topo_prm.*` and live in
`asr_sdm_planning_manager/config/topo_replan.yaml`.

### Sample region (TopologyPRM)

#### Where `start` and `end` come from

`findCollisionRange` walks the initial local segment (about
`manager.local_segment_length` long, cut from the global min-snap trajectory)
every 0.05 s. A point is unsafe when its ESDF distance is below
`topo_prm.clearance`.

- `start` is the sample just **before** the first unsafe point.
- `end` is the first safe sample **after** the last unsafe stretch.

With several separate collision stretches, one box covers all of them. The safe
parts outside the window (`start_pts`, `end_pts`) are not sampled; they are
spliced back onto every candidate in `selectShortPaths`.

#### Box frame and size

`createGraph` builds an oriented box centred between `start` and `end`:

```text
c         = (start + end) / 2
x_hat     = normalize(end - c)                    # along start -> end
y_hat     = normalize(x_hat × (0, 0, -1))         # horizontal, left of x_hat
z_hat     = x_hat × y_hat                         # "up", perpendicular to x_hat
R         = [x_hat  y_hat  z_hat]

sample_r  = ( |end - start| / 2 + sample_inflate_x,
              sample_inflate_y,
              sample_inflate_z )                  # half extents

p_world   = R · p_local + c,   p_local ∈ [-sample_r, +sample_r]
```

Using world "down" as the reference keeps `y_hat` horizontal, so left/right
detours stay in the horizontal plane. For a level heading `z_hat` is world up;
on a slope it tilts with `x_hat`.

Only the x half-length follows the blocked window. The y and z half-extents
are fixed and do not grow with the obstacle, so a detour wider than
`sample_inflate_y` (or taller than `sample_inflate_z`) cannot be found.

If `start -> end` is vertical, `x_hat × (0, 0, -1)` is zero and the box
collapses onto the x axis. The robot normally moves near the horizontal plane,
so this is not handled.

#### Quadrant pruning

With `topo_prm.prune_quadrant: true`, the box is split by its local x-y and
x-z planes. Both planes contain the start -> end axis, so each quadrant still
spans the whole blocked window.

```text
view from start, looking along +x (towards end)

                 +z (up)
                   |
   Q0 left-up      |      Q1 right-up
   (y >= 0, z >= 0)|      (y < 0, z >= 0)
  +y --------------x-------------- -y
   Q2 left-down    |      Q3 right-down
   (y >= 0, z < 0) |      (y < 0, z < 0)
                   |
                 -z
```

`lowestEsdfQuadrant()` runs once per `createGraph`:

1. Lay a grid of cell centres over the box in the local frame, spacing
   `quadrant_eval_step` (never finer than the map resolution). The y and z
   cell counts are even, so no centre sits on a splitting plane and every
   quadrant gets the same number of cells.
2. For each cell inside the map, read `ESDFMap::getDistance()` (a
   precomputed voxel value, no interpolation), cap it at `quadrant_dist_cap`,
   and add it to that quadrant's sum. Cells outside the map add nothing.
3. The quadrant with the **lowest** sum is the one most filled by obstacles.
   It is removed, and the log shows
   `[Topo]: quadrant esdf sum: a b c d, removed N`.

The cap is required. Voxels that no ESDF update has reached hold `10000`, and
an update whose box contained no obstacle leaves values around `1e153`;
uncapped, a single such voxel would decide the comparison.

`getSample()` then samples the remaining three quadrants directly: x is
uniform in `[-r_x, r_x]`, |y| and |z| are uniform in `[0, r_y]` and
`[0, r_z]`, one of the three kept quadrants is picked uniformly, and the signs
of y and z are set from its index. All quadrants have the same volume, so the
result is uniform over their union and no draw is discarded.

With `prune_quadrant: false`, no ESDF sum is computed and all four quadrants
are sampled, which is the same distribution as the original whole-box
sampling.

#### What happens to each sample

Sampling stops at `max_sample_time` (accumulated) or `max_sample_num`,
whichever comes first; with the default 5 ms budget, the time limit is usually
hit first (see `[Topo]: sample num: N`).

1. Reject the sample if its ESDF distance is `<= clearance`.
2. Find visible guards (line of sight through ESDF voxels, blocked at
   `<= resolution`).
   - None visible: add it as a new **guard**.
   - Exactly two visible: add it as a **connector** between them, unless an
     existing connector of the same two guards is topologically equivalent
     (then keep the shorter of the two).
   - Otherwise: discard.
3. After sampling, repeatedly delete nodes with at most one neighbour.

#### Caveats

- The ESDF is only valid inside the map's latest update box: the bounding box
  of obstacle points within `esdf_map.local_update_range` of the sensor, plus
  the sensor position. Box cells outside it read `10000` or stale values,
  which the cap turns into "free". Parts of the box within 1 m of that
  boundary therefore look slightly more open than they are.
- The removed quadrant is deterministic for a given map. If the only passage
  lies in it, every retry fails with `No path.`; set `prune_quadrant: false`
  in that environment.
- Nodes keep `clearance` (0.3 m) from obstacles, but edges are only checked
  against one voxel (0.1 m). The B-spline clearance cost in the manager pushes
  the final trajectory away.

#### Parameters (`topo_prm.*` in `topo_replan.yaml`)

| Parameter | Value | Role |
|---|---|---|
| `sample_inflate_x` | 1.0 | Extra half-length beyond `start` / `end` along x [m] |
| `sample_inflate_y` | 3.0 | Half-width along y (left / right) [m] |
| `sample_inflate_z` | 3.0 | Half-height along z (up / down) [m] |
| `clearance` | 0.3 | Min ESDF distance for a node; also the collision-range threshold in the manager [m] |
| `max_sample_time` | 0.005 | Accumulated sampling budget [s] |
| `max_sample_num` | 2000 | Sample count cap |
| `prune_quadrant` | true | Remove the lowest-ESDF quadrant before sampling |
| `quadrant_dist_cap` | 1.0 | Per-cell ESDF cap before summing [m] |
| `quadrant_eval_step` | 0.2 | Grid spacing for the quadrant sums [m] |
| `max_raw_path` | 300 | DFS raw path cap |
| `max_raw_path2` | 25 | Raw paths kept, fewest nodes first |
| `reserve_num` | 6 | Max selected paths |
| `ratio_to_short` | 5.5 | Max length ratio to the shortest selected path |
| `parallel_shortcut` | true | One thread per raw path during shortcutting |
| `short_cut_num` | 1 | Read but unused |

Parameters are read once in `TopologyPRM::init()`; restart the manager node
after editing the yaml. With `--symlink-install` the installed yaml is a
symlink to the source file, so no rebuild is needed.

### TopoPathModifier (test node library)

Given an input path and a collision checker:

1. If the path is collision-free, return it unchanged.
2. Otherwise, only the blocked local segment is modified. The class builds a
   guard/connector roadmap, searches raw paths, shortcuts them, prunes
   topologically equivalent paths, applies `selectShortPaths`, and splices
   the selected local detours back into the original waypoints.

`TopoModifierResult::candidate_paths` holds every selected candidate as a
complete path from input start to goal. `path` is the best one, kept for
backward compatibility.

#### Sample region (TopoPathModifier)

This class does not use the oriented box or quadrant pruning above.

- **Box**: world-axis-aligned. It is the bounding box of `start`, `goal`, the
  input waypoints in the window, and every collision hint centre padded by
  `radius + collision_clearance + local_window_padding`; the result is then
  grown by `local_window_padding` on every side.
- **Seed nodes**: the interior input waypoints of the window, plus 8 points
  around each hint at `radius + collision_clearance + detour_margin` along
  ±`axis1`, ±`axis2` and the four diagonals, where
  `axis1 = dir × z` and `axis2 = dir × axis1`.
- **Random samples**: `max_sample_num` uniform draws. Points in collision or
  closer than `waypoint_spacing / 2` to an existing node are dropped. With
  `deterministic_sampling`, the generator is reseeded with `random_seed` on
  every call.

Parameters are `topo.*` in `config/local_path_modifier.yaml`.

### Test node `local_path_modifier_test`

The library does not publish. The test node does.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | Latest guidance waypoints (reliable + transient_local, so it works even if started after the guidance planner) |
| `/planning/add_virtual_obstacle` | `geometry_msgs/msg/PointStamped` | RViz `Publish Point`: add a temporary spherical obstacle |
| `/planning/clear_virtual_obstacles` | `std_msgs/msg/Empty` | Clear all temporary obstacles |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/planning/virtual_obstacles` | `visualization_msgs/msg/Marker` | Temporary spherical obstacles |
| `/planning/topo_candidate_paths` | `visualization_msgs/msg/MarkerArray` | All selected candidate detours (yellow) |

It does **not** publish a single `/planning/modified_path`.

### Launch

Start guidance first so it opens the shared RViz page:

```bash
ros2 launch asr_sdm_guidance_planner_dev astar_lbfgs_planner.launch.py
```

Then start the test node without a second RViz window:

```bash
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py
```

Standalone, with its own RViz:

```bash
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py use_rviz:=true
```

Clear all temporary obstacles:

```bash
ros2 topic pub --once /planning/clear_virtual_obstacles std_msgs/msg/Empty "{}"
```

`TopologyPRM` has no launch file of its own; it runs inside
`ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py`.

### Library usage

`TopologyPRM`, as the manager uses it:

```cpp
#include <asr_sdm_local_path_modifier/topo_prm.h>

auto topo_prm = std::make_shared<amprobo::TopologyPRM>();
topo_prm->setEnvironment(edt_environment);  // before init(): init() reads the map resolution
topo_prm->init(node);                       // declares and reads topo_prm.*

topo_prm->findTopoPaths(
  start, end, start_pts, end_pts, graph, raw_paths, filtered_paths, select_paths);
```

`TopoPathModifier`, with a collision checker instead of RViz obstacles:

```cpp
#include <asr_sdm_local_path_modifier/topo_path_modifier.hpp>

amprobo::TopoPathModifier modifier;
modifier.setOptions(options);

amprobo::CollisionChecker checker;
checker.pointInCollision = ...;
checker.segmentInCollision = ...;
checker.clearance = ...;

amprobo::TopoModifierResult result = modifier.modify(input_path, checker);
// or: modifier.modify(input_path, map);  // MapQueryInterface overload
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_local_path_modifier asr_sdm_planning_manager \
  --symlink-install
source install/setup.bash
```

Rebuild `asr_sdm_planning_manager` whenever `topo_prm.h` changes: it allocates
the object itself (`new TopologyPRM`), so the class size is compiled into the
manager.

---

<a id="中文"></a>

## 中文

### 概述

这个包做的是 Fast-Planner `TopologyPRM` 风格的局部拓扑绕障路径搜索。它编译出一个
共享库 `asr_sdm_local_path_modifier`，里面有两个互相独立的类，另外带一个 RViz 测试节点。

| 类 | 头文件 | 使用方 |
|---|---|---|
| `TopologyPRM` | `asr_sdm_local_path_modifier/topo_prm.h` | `asr_sdm_planning_manager`（正式栈的重规划） |
| `TopoPathModifier` | `asr_sdm_local_path_modifier/topo_path_modifier.hpp` | 测试节点 `local_path_modifier_test`；还没有接进 manager |

两者都在路径被挡住的那一段附近建局部路线图，搜索拓扑上不同的绕行路径，但**不共享**
代码和参数。非均匀 B 样条优化不在这个包里，由 manager 对 `TopologyPRM` 返回的路径
去做。栅格 A* 前端在单独的包
[`asr_sdm_guidance_planner`](../asr_sdm_guidance_planner/) 里。

### 目录结构

```text
asr_sdm_local_path_modifier/
├── include/asr_sdm_local_path_modifier/
│   ├── topo_prm.h               # TopologyPRM
│   └── topo_path_modifier.hpp   # TopoPathModifier
├── src/
│   ├── topo_prm.cpp
│   └── topo_path_modifier.cpp
├── test/local_path_modifier_test.cpp
├── config/local_path_modifier.yaml
├── launch/local_path_modifier_test.launch.py
└── rviz/local_path_modifier_test.rviz
```

### TopologyPRM（正式栈）

当前局部 B 样条检测到碰撞时，`PlanningManager::topoReplan` 调用它：

```text
PlanningManager::findCollisionRange
  -> start, end, start_pts, end_pts
TopologyPRM::findTopoPaths(start, end, start_pts, end_pts, ...)
  createGraph       采样区域 + Visibility PRM（guard / connector）
  searchPaths       从 start 到 end 做 DFS，优先保留节点少的路径
  shortcutPaths     按可见性捷径，碰撞点沿 ESDF 梯度推开
  pruneEquivalent   去掉拓扑等价的路径
  selectShortPaths  保留最短的几条，把 start_pts / end_pts 拼回首尾
```

它通过 `EDTEnvironment`（`asr_sdm_esdf_map`）读取 ESDF 地图。参数在 manager 节点上以
`topo_prm.*` 声明，写在 `asr_sdm_planning_manager/config/topo_replan.yaml` 里。

### 采样区域（TopologyPRM）

#### `start` 和 `end` 是什么点

`findCollisionRange` 每隔 0.05 s 扫描一遍初始局部段（从全局 min-snap 轨迹上截出，
长度约为 `manager.local_segment_length`）。ESDF 距离小于 `topo_prm.clearance` 的点
算不安全。

- `start`：第一次进入不安全区之**前**的那个采样点。
- `end`：最后一段不安全区之**后**的第一个安全采样点。

有好几段互不相连的碰撞区间时，一个采样盒会把它们全部包进去。碰撞窗口外的安全部分
（`start_pts`、`end_pts`）不参与采样，最后在 `selectShortPaths` 里拼回每条候选路径。

#### 采样盒的坐标系和尺寸

`createGraph` 以 `start` 和 `end` 的中点为中心，建一个有朝向的长方体：

```text
c         = (start + end) / 2
x_hat     = normalize(end - c)                    # 沿 start -> end
y_hat     = normalize(x_hat × (0, 0, -1))         # 水平，指向 x_hat 的左侧
z_hat     = x_hat × y_hat                         # “上”，垂直于 x_hat
R         = [x_hat  y_hat  z_hat]

sample_r  = ( |end - start| / 2 + sample_inflate_x,
              sample_inflate_y,
              sample_inflate_z )                  # 半边长

p_world   = R · p_local + c,   p_local ∈ [-sample_r, +sample_r]
```

用世界坐标的“竖直向下”作参考，可以保证 `y_hat` 一定水平，左右绕行始终在水平面内。
前进方向水平时 `z_hat` 就是世界的竖直向上；有坡度时 `z_hat` 跟着 `x_hat` 一起倾斜。

只有 x 方向的半边长随碰撞窗口变化。y、z 方向的半边长固定，不会随障碍物变大，所以
比 `sample_inflate_y` 更宽（或比 `sample_inflate_z` 更高）的绕行搜不到。

如果 `start -> end` 是竖直方向，`x_hat × (0, 0, -1)` 为零，采样盒会塌成 x 轴上的一条线。
机器人通常在接近水平的面上运动，代码里没有处理这种情况。

#### 象限剔除

`topo_prm.prune_quadrant: true` 时，用局部坐标系的 x-y 平面和 x-z 平面把采样盒切成四块。
两个平面都包含 start -> end 轴，所以每个象限都沿 x 方向覆盖整个碰撞窗口。

```text
站在 start，沿 +x 看向 end

                 +z（上）
                   |
   Q0 左上         |      Q1 右上
   (y >= 0, z >= 0)|      (y < 0, z >= 0)
  +y --------------x-------------- -y
   Q2 左下         |      Q3 右下
   (y >= 0, z < 0) |      (y < 0, z < 0)
                   |
                 -z
```

每次 `createGraph` 调用一次 `lowestEsdfQuadrant()`：

1. 在局部坐标系下按 `quadrant_eval_step` 的间距铺格点中心（间距不会小于地图分辨率）。
   y、z 方向的格点数取偶数，这样没有格点中心落在切割平面上，四个象限分到的格点数相同。
2. 对地图范围内的每个格点，读 `ESDFMap::getDistance()`（地图预先算好的体素值，不插值），
   截断到 `quadrant_dist_cap` 后累加到所在象限。地图外的格点不计入。
3. 总和**最小**的象限就是被障碍物占得最多的象限。把它去掉，并打印日志
   `[Topo]: quadrant esdf sum: a b c d, removed N`。

截断是必须的：从未被 ESDF 更新过的体素值是 `10000`，某次更新的盒子里一个障碍物都没有时
会留下约 `1e153` 的值。不截断的话，只要有一个这样的体素，它就决定了比较结果。

随后 `getSample()` 直接在剩下的三个象限里取点：x 在 `[-r_x, r_x]` 上均匀取值，|y| 和 |z|
分别在 `[0, r_y]`、`[0, r_z]` 上均匀取值；再从三个保留象限里等概率选一个，按它的编号
设定 y、z 的符号。四个象限体积相同，所以结果在三个象限的并集上是均匀的，而且不会有
被丢弃的抽样。

`prune_quadrant: false` 时，不计算 ESDF 总和，四个象限都参与采样，分布和原来的整盒
均匀采样相同。

#### 每个采样点怎么处理

采样在累计时间达到 `max_sample_time` 或采样数达到 `max_sample_num` 时停止，哪个先到
算哪个。默认的 5 ms 预算下，通常是时间先到（看日志 `[Topo]: sample num: N`）。

1. ESDF 距离 `<= clearance` 的点直接丢掉。
2. 找它能看到的 guard（沿 ESDF 体素做视线检查，距离 `<= resolution` 就算被挡）：
   - 一个都看不到：加为新的 **guard**。
   - 正好看到两个：加为连接这两个 guard 的 **connector**；但如果这两个 guard 之间已有
     一个拓扑等价的 connector，就不加，只保留两者中更短的那个。
   - 其他情况：丢掉。
3. 采样结束后，反复删除邻居不超过 1 个的节点。

#### 注意事项

- ESDF 只在地图最近一次更新的盒子里有效。这个盒子是传感器周围
  `esdf_map.local_update_range` 内障碍物点的包围盒，再并上传感器位置。采样盒落在它外面
  的格点读到的是 `10000` 或旧值，截断后按“空闲”计入。所以采样盒里离这个边界 1 m 以内的
  部分，会显得比实际更空旷一些。
- 对同一张地图，去掉哪个象限是确定的。如果唯一的通道恰好在被去掉的象限里，每次重试都会
  失败并打印 `No path.`。遇到这种环境时，把 `prune_quadrant` 设为 `false`。
- 节点离障碍物至少 `clearance`（0.3 m），但边只按一个体素（0.1 m）检查。最终轨迹靠
  manager 里 B 样条的距离代价推开。

#### 参数（`topo_replan.yaml` 里的 `topo_prm.*`）

| 参数 | 值 | 作用 |
|---|---|---|
| `sample_inflate_x` | 1.0 | x 方向在 `start` / `end` 之外多伸出的长度 [m] |
| `sample_inflate_y` | 3.0 | y 方向（左右）半宽 [m] |
| `sample_inflate_z` | 3.0 | z 方向（上下）半高 [m] |
| `clearance` | 0.3 | 节点离障碍物的最小 ESDF 距离；也是 manager 判断碰撞区间的阈值 [m] |
| `max_sample_time` | 0.005 | 累计采样时间预算 [s] |
| `max_sample_num` | 2000 | 采样数上限 |
| `prune_quadrant` | true | 采样前是否去掉 ESDF 总和最小的象限 |
| `quadrant_dist_cap` | 1.0 | 求和前每个格点 ESDF 值的上限 [m] |
| `quadrant_eval_step` | 0.2 | 计算象限总和的格点间距 [m] |
| `max_raw_path` | 300 | DFS 枚举的原始路径上限 |
| `max_raw_path2` | 25 | 保留的原始路径数，节点少的优先 |
| `reserve_num` | 6 | 最终保留的路径数上限 |
| `ratio_to_short` | 5.5 | 相对最短路径的最大长度比 |
| `parallel_shortcut` | true | 捷径时每条原始路径一个线程 |
| `short_cut_num` | 1 | 会读取，但没有被使用 |

参数只在 `TopologyPRM::init()` 里读一次，改完 yaml 要重启 manager 节点。用
`--symlink-install` 编译时，安装目录下的 yaml 是指向源码的软链接，不需要重新编译。

### TopoPathModifier（测试节点用的库）

输入一条路径和一个碰撞检查器：

1. 路径无碰撞时，原样返回。
2. 有碰撞时，只修改被挡住的那一段：建 guard/connector 路线图，搜索原始路径，做捷径，
   去掉拓扑等价的路径，执行 `selectShortPaths`，再把选出的局部绕行拼回原始路点。

`TopoModifierResult::candidate_paths` 保存所有选出的候选，每条都是从输入起点到终点的
完整路径。`path` 是其中最好的一条，为了向后兼容而保留。

#### 采样区域（TopoPathModifier）

这个类不使用上面的有向采样盒，也没有象限剔除。

- **采样盒**：与世界坐标轴对齐。取 `start`、`goal`、窗口内的输入路点，以及每个碰撞提示点
  中心向外扩 `radius + collision_clearance + local_window_padding` 后的包围盒，再在每个
  方向上额外扩 `local_window_padding`。
- **种子节点**：窗口内部的输入路点；以及每个碰撞提示点周围、距离
  `radius + collision_clearance + detour_margin` 的 8 个点，方向是 ±`axis1`、±`axis2` 和
  四个对角方向，其中 `axis1 = dir × z`，`axis2 = dir × axis1`。
- **随机采样**：均匀抽 `max_sample_num` 个点。落在碰撞里、或离已有节点不到
  `waypoint_spacing / 2` 的点丢掉。开启 `deterministic_sampling` 时，每次调用都用
  `random_seed` 重新播种。

参数是 `config/local_path_modifier.yaml` 里的 `topo.*`。

### 测试节点 `local_path_modifier_test`

库本身不发 topic，测试节点发。

**Subscribe**

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | 最新引导路点（reliable + transient_local，比 guidance 后启动也能收到） |
| `/planning/add_virtual_obstacle` | `geometry_msgs/msg/PointStamped` | RViz `Publish Point`：加一个临时球形障碍 |
| `/planning/clear_virtual_obstacles` | `std_msgs/msg/Empty` | 清空所有临时障碍 |

**Publish**

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/virtual_obstacles` | `visualization_msgs/msg/Marker` | 临时球形障碍 |
| `/planning/topo_candidate_paths` | `visualization_msgs/msg/MarkerArray` | 所有选出的候选绕行路径（黄色） |

不发布单条的 `/planning/modified_path`。

### Launch

先启动 guidance，由它打开共用的 RViz 页面：

```bash
ros2 launch asr_sdm_guidance_planner_dev astar_lbfgs_planner.launch.py
```

再启动测试节点，不开第二个 RViz 窗口：

```bash
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py
```

单独运行，并打开自己的 RViz：

```bash
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py use_rviz:=true
```

清空所有临时障碍：

```bash
ros2 topic pub --once /planning/clear_virtual_obstacles std_msgs/msg/Empty "{}"
```

`TopologyPRM` 没有自己的 launch 文件，它运行在
`ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py` 里。

### 库的用法

`TopologyPRM`，与 manager 的用法相同：

```cpp
#include <asr_sdm_local_path_modifier/topo_prm.h>

auto topo_prm = std::make_shared<amprobo::TopologyPRM>();
topo_prm->setEnvironment(edt_environment);  // 必须在 init() 之前：init() 要读地图分辨率
topo_prm->init(node);                       // 声明并读取 topo_prm.*

topo_prm->findTopoPaths(
  start, end, start_pts, end_pts, graph, raw_paths, filtered_paths, select_paths);
```

`TopoPathModifier`，用碰撞检查器代替 RViz 临时障碍：

```cpp
#include <asr_sdm_local_path_modifier/topo_path_modifier.hpp>

amprobo::TopoPathModifier modifier;
modifier.setOptions(options);

amprobo::CollisionChecker checker;
checker.pointInCollision = ...;
checker.segmentInCollision = ...;
checker.clearance = ...;

amprobo::TopoModifierResult result = modifier.modify(input_path, checker);
// 或者：modifier.modify(input_path, map);  // MapQueryInterface 重载
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_local_path_modifier asr_sdm_planning_manager \
  --symlink-install
source install/setup.bash
```

`topo_prm.h` 有改动时，要连同 `asr_sdm_planning_manager` 一起重新编译：对象是 manager
自己 `new TopologyPRM` 出来的，类的大小在编译 manager 时就确定了。
