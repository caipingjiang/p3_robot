# slam_toolbox 建图参数说明

> 对应配置文件：`slam_toolbox_params.yaml`（`p3dx_slam` 包）
> 节点：`sync_slam_toolbox_node`（online 同步 SLAM，一边扫一边建图）
>
> 这个文件在 `gazebo_slam.launch.py` 里被读取，作为 `slam_toolbox` 节点的参数。

## 整体作用

slam_toolbox 做的是「基于图优化的 2D 激光 SLAM」：把每一帧激光点云当作一个位姿（节点），通过**扫描匹配**算相邻帧的相对位姿（边），再用 Ceres 求解器做全局图优化，最后输出 `map→odom` 的 TF 变换和占据栅格地图。

它的核心工作流：

```
激光/scan → 判断是否该处理(平移/旋转/时间阈值) → 扫描匹配 → 更新图 → 优化 → 发布地图
```

---

## 1. Solver（图优化求解器）

| 参数 | 当前值 | 含义 / 作用 |
|---|---|---|
| `solver_plugin` | `solver_plugins::CeresSolver` | 选择图优化的后端求解器，固定用 Ceres Solver 库（Google 的通用非线性最小二乘库），一般不用改 |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | Ceres 的线性求解器类型。`SPARSE_NORMAL_CHOLESKY` = 对稀疏法方程做 Cholesky 分解，适合中等规模 SLAM 图，速度和精度平衡好 |
| `ceres_preconditioner` | `SCHUR_JACOBI` | 预条件器，加速线性方程收敛。`SCHUR_JACOBI` 针对位姿+路标这种带 Schur 补结构的系统 |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | 信赖域策略。LM 算法在 Gauss-Newton 和梯度下降之间自适应切换，最常用、最稳 |
| `ceres_diagonal_type` | `BLOCK_DIAGONAL` | 预条件时的对角块类型。`BLOCK_DIAGONAL` 按位姿分块，比逐元素更快 |
| `ceres_loss_function` | `None` | 鲁棒核函数（损失函数），用于抑制离群约束。`None` = 不启用，匹配异常时图优化可能被个别坏边拉偏。想更鲁棒可改成 `HuberLoss` |

> 这一整段 99% 场景都不用动，是求解器内部性能参数。

---

## 2. Frames & topic（坐标系与话题）

| 参数 | 当前值 | 含义 / 作用 |
|---|---|---|
| `odom_frame` | `odom` | 里程计坐标系名。slam_toolbox 订阅 `odom→base_link` 的 TF，并输出 `map→odom` |
| `map_frame` | `map` | 地图坐标系名，SLAM 输出的地图就挂在这个帧上 |
| `base_frame` | `base_link` | 机器人基座坐标系。注意：本机器人**没有** `base_footprint`，激光直接相对 `base_link` |
| `scan_topic` | `/scan` | 订阅的激光话题名。Gazebo 里 hokuyo（`lms100`）发到 `/scan` |

> 这些必须和机器人 TF 树、激光话题完全一致，否则报「找不到帧」或收不到数据。

---

## 3. Map（地图）

| 参数 | 当前值 | 含义 / 作用 |
|---|---|---|
| `mode` | `mapping` | 运行模式。`mapping` = 建图模式（会持续创建新节点/回环）。导航时切 `localization` 模式（只在已有图上定位，不更新图） |
| `resolution` | `0.05` | 栅格分辨率，米/格。0.05 m = 5 cm，越小地图越精细但占用内存越大、处理越慢。0.05 是室内建图的常用值 |
| `map_update_interval` | `2.0` | 地图发布/更新的时间间隔（秒）。越大 RViz 里地图刷新越慢，越小越吃 CPU |
| `max_laser_range` | `12.0` | 激光最大可用距离（米）。超过这个距离的激光点被丢弃，不用来建图 |
| `min_laser_range` | `0.1` | 激光最小可用距离（米）。小于这个距离（机器人自身反射等）的点丢弃 |
| `minimum_time_interval` | `0.5` | 两帧参与扫描匹配的最小时间间隔（秒）。配合下面的平移/旋转阈值一起，决定「多久处理一帧」 |
| `transform_timeout` | `0.2` | 等待一个 TF 变换的超时时间（秒） |
| `tf_buffer_duration` | `30.0` | TF 缓存保留时长（秒）。回环检测可能需要回溯较早的位姿，缓存越长能查的历史越久 |
| `stack_size_to_use` | `40000000` | 处理线程的栈大小（字节，40 MB）。slam_toolbox 内部开单独线程跑匹配，栈不够会崩 |

---

## 4. Scan matching（扫描匹配）

> 这是**决定建图质量、以及「转了多少度才更新地图」的核心段落**，也是之前调试的重点。

| 参数 | 当前值 | 含义 / 作用 |
|---|---|---|
| `use_scan_matching` | `true` | 是否启用扫描匹配。关掉后只靠里程计位姿建图，误差会累积，一般必须开 |
| `use_scan_barycenter` | `true` | 匹配时是否用扫描点云的「质心」辅助对齐，能提高匹配稳定性和速度 |
| `minimum_travel_distance` | `0.5` | **最小平移距离阈值（米）**：机器人平移超过 0.5 m 才把新一帧加入图 |
| `minimum_travel_heading` | `0.05` | **最小旋转角度阈值（弧度）**：机器人旋转超过 0.05 rad（≈2.86°）才处理新一帧。之前设成默认 0.5 rad（≈28.6°）导致转 15° 不更新地图，现已调小 |
| `scan_buffer_size` | `10` | 扫描缓冲区大小，保留最近 N 帧用于匹配/回环 |
| `scan_buffer_maximum_scan_distance` | `10.0` | 缓冲区里允许的扫描与当前位姿的最大距离（米） |
| `link_match_minimum_response_fine` | `0.1` | **相邻帧关联的最小精细响应值（0~1）**：扫描匹配得到一个响应分数，低于此值认为「两帧匹配失败」，不建立关联边。**这是「匹配响应阈值」，可手动调**——调小=更容易匹配成功，但可能误关联 |
| `link_scan_maximum_distance` | `1.5` | 关联的两帧之间最大允许距离（米），超过就认为不构成关联 |
| `loop_search_maximum_distance` | `3.0` | 回环检测时，在当前位姿多大范围内搜索历史节点 |
| `do_loop_closing` | `true` | 是否做回环检测。回环能消除累积漂移，让走一圈后地图闭合，强烈建议开 |
| `loop_match_minimum_chain_size` | `10` | 形成一个「链」至少需要多少帧，才尝试做回环匹配 |
| `loop_match_maximum_variance_coarse` | `3.0` | 回环粗匹配允许的最大方差，粗筛候选回环 |
| `loop_match_minimum_response_coarse` | `0.35` | 回环**粗匹配**的最小响应值（0~1），粗筛用，阈值较低 |
| `loop_match_minimum_response_fine` | `0.45` | 回环**精细匹配**的最小响应值（0~1），确认回环用，阈值较高更严格 |

---

## 5. 调参建议（针对本项目）

- **地图更新太慢 / 转小角度不更新**：调小 `minimum_travel_heading`（已设 0.05）和 `minimum_travel_distance`。
- **匹配失败、地图断层**：调小 `link_match_minimum_response_fine`（已设 0.1，已经比较宽松），或调大 `link_scan_maximum_distance`。
- **地图漂移、走一圈闭不上**：确认 `do_loop_closing: true`，可调小 `loop_match_minimum_response_fine` 让回环更容易触发。
- **对称环境（本场景是规整矩形）旋转时匹配退化**：这是激光在对称结构下旋转方向不可观测的固有问题，靠调参数只能缓解（缩小 heading 阈值让更多角度参与匹配），本质要靠增加环境纹理或走弧线轨迹。
