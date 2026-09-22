# nav2 导航参数说明

> 对应配置文件：`nav2_params.yaml`（`p3dx_nav` 包）
> 由 `nav_bringup.launch.py` 通过 `params_file` 传给 nav2 各节点。
>
> nav2 用**组合节点**（`/nav2_container`），下面这些 amcl / planner / controller / bt / costmap 等都是容器里的**生命周期节点**，`ros2 node list` 只显示一个容器，但参数是按各自名字分开配置的。

## 整体架构

导航时数据流：

```
激光/scan ──► amcl ──► map→odom TF（定位）
                   │
全局代价地图 global_costmap（map 帧）──► 全局规划器 planner（A*/Dijkstra 找全局路径）
                   │
局部代价地图 local_costmap（odom 帧）──► 局部控制器 controller（DWB 跟随路径发 cmd_vel）
```

行为树 `bt_navigator` 把上面串起来：收到目标 → 规划 → 跟随时监控 → 卡住则恢复行为（原地转/后退）。

---

## 1. amcl（自适应蒙特卡洛定位）

> 粒子滤波定位：在已知地图上，用激光匹配来估计机器人在 map 帧下的位姿，发布 `map→odom`。

### 帧与话题

| 参数 | 值 | 含义 |
|---|---|---|
| `global_frame_id` | `map` | 全局（地图）坐标系 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `base_frame_id` | `base_link` | 机器人基座帧（本项目无 base_footprint） |
| `scan_topic` | `/scan` | 激光话题 |
| `tf_broadcast` | `true` | 是否广播 `map→odom`。**必须为 true**，否则 TF 树没有 map 帧 |
| `use_sim_time` | `True` | 使用仿真时间（Gazebo 必须开） |

### 运动模型（里程计噪声）

| 参数 | 值 | 含义 |
|---|---|---|
| `alpha1`~`alpha4` | `0.2` | 里程计运动模型的 4 个噪声系数：α1 旋转→旋转、α2 平移→平移、α3 平移→旋转、α4 旋转→平移。越大代表里程计越不可信，粒子散布越宽 |
| `alpha5` | `0.2` | 差分模型额外噪声项（对 DifferentialMotionModel 有效） |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | 差分驱动模型（Pioneer 3DX 是差分轮） |
| `update_min_d` | `0.25` | 平移超过 0.25 m 才做一次滤波更新 |
| `update_min_a` | `0.2` | 旋转超过 0.2 rad 才做一次滤波更新 |

### 粒子滤波

| 参数 | 值 | 含义 |
|---|---|---|
| `max_particles` | `2000` | 粒子数量上限。越多越准越慢 |
| `min_particles` | `500` | 粒子数量下限（KLD 自适应采样时最少保留这些） |
| `pf_err` | `0.05` | 粒子滤波误差容忍（KLD 采样判断粒子是否够用的误差上限） |
| `pf_z` | `0.99` | 粒子滤波置信度 z 值（KLD 采样的置信水平） |
| `resample_interval` | `1` | 每多少次更新做一次重采样 |
| `recovery_alpha_fast` | `0.0` | 快速恢复模式注入的随机噪声（全局定位失败时的恢复，0=关闭） |
| `recovery_alpha_slow` | `0.0` | 慢速恢复模式的随机噪声 |
| `transform_tolerance` | `1.0` | TF 时间戳容忍（秒），激光/里程计时间戳差在此内就接受 |

### 激光似然模型（likelihood_field）

| 参数 | 值 | 含义 |
|---|---|---|
| `laser_model_type` | `likelihood_field` | 激光模型类型。`likelihood_field`（似然域）比 `beam` 更快更平滑，推荐 |
| `laser_max_range` | `100.0` | 激光最大范围（用于归一化） |
| `laser_min_range` | `-1.0` | 激光最小范围（-1 表示不考虑近处反射） |
| `laser_likelihood_max_dist` | `2.0` | 似然域计算时到障碍物的最大距离，超过按此截断 |
| `max_beams` | `60` | 每次更新实际使用的激光束数量（在整圈里均匀抽 60 条），减少计算量 |
| `sigma_hit` | `0.2` | 命中高斯的标准差（米），激光测量噪声 |
| `lambda_short` | `0.1` | 短读数（提前被障碍挡住）的指数分布参数 |
| `z_hit` / `z_short` / `z_max` / `z_rand` | `0.5`/`0.05`/`0.05`/`0.5` | 四种测量来源的权重：命中/短读/最大距离/随机。总和应≈1，这里 0.5+0.05+0.05+0.5=1.1，命中+随机占大头 |
| `do_beamskip` | `false` | 是否跳过「明显被挡住」的光束（省计算） |
| `beam_skip_threshold` | `0.3` | 判断光束是否可跳过的阈值 |
| `beam_skip_distance` | `0.5` | 光束跳过时的距离参数 |
| `beam_skip_error_threshold` | `0.9` | 光束跳过的误差阈值 |

### 初始位姿（本项目新增）

| 参数 | 值 | 含义 |
|---|---|---|
| `set_initial_pose` | `true` | 启动时**自动设置初始位姿**，避免手动在 RViz 点 2D Pose Estimate |
| `initial_pose` | x=y=z=yaw=0 | 初始位姿。机器人 spawn 在 Gazebo 原点 (0,0)、朝 +x，恰好对应 map 原点 |

> 这几行是关键修复：没有初始位姿，amcl 不发 `map→odom`，整个导航报「找不到 map 帧」。

---

## 2. bt_navigator（行为树导航）

| 参数 | 值 | 含义 |
|---|---|---|
| `global_frame` | `map` | 全局坐标系 |
| `robot_base_frame` | `base_link` | 机器人基座帧 |
| `odom_topic` | `/odom` | 里程计话题 |
| `bt_loop_duration` | `10` | 行为树主循环周期（ms） |
| `default_server_timeout` | `50` | 等待行为树里各 action server 响应的默认超时（ms）。已从默认 20 调大，避免偶发超时判失败 |
| `wait_for_service_timeout` | `1000` | 等待依赖 service 的超时（ms） |
| `plugin_lib_names` | （一长串） | 加载的 BT 插件节点库，提供 `NavigateToPose`、`FollowPath`、`Spin`、`Recovery` 等行为的实现。**一般不用动** |

---

## 3. controller_server（局部控制器，DWB）

> DWB（Dynamic Window Based）局部规划器：在局部代价地图上，实时生成速度指令 `cmd_vel` 去跟随全局路径。

### 通用

| 参数 | 值 | 含义 |
|---|---|---|
| `controller_frequency` | `20.0` | 控制循环频率（Hz），每秒 20 次计算并下发速度 |
| `controller_plugins` | `["FollowPath"]` | 启用的控制器插件（这里只一个 FollowPath） |
| `goal_checker_plugins` | `["general_goal_checker"]` | 目标判定器 |
| `progress_checker_plugin` | `progress_checker` | 进度检查器（判断是否卡住） |
| `min_x/theta_velocity_threshold` | `0.001` | 判定「是否在动」的最小线速度/角速度 |
| `min_y_velocity_threshold` | `0.5` | 横向速度阈值（差分机器人无横向运动，故设大） |
| `failure_tolerance` | `0.3` | 允许的失败容忍度（进度检查的容错） |

### 进度 / 目标判定

| 参数 | 值 | 含义 |
|---|---|---|
| `progress_checker.required_movement_radius` | `0.5` | 规定时间内必须移动超过 0.5 m，否则判定卡住 |
| `progress_checker.movement_time_allowance` | `10.0` | 判定卡住的时间窗口（秒） |
| `general_goal_checker.xy_goal_tolerance` | `0.25` | 到达目标的位置容差（米），距离目标 0.25 m 内即算到 |
| `general_goal_checker.yaw_goal_tolerance` | `0.25` | 到达目标的朝向容差（弧度），朝偏差 0.25 rad 内即算正 |
| `general_goal_checker.stateful` | `True` | 有状态目标判定（一旦判定到就不再回退） |

### DWB 参数（FollowPath）

| 参数 | 值 | 含义 |
|---|---|---|
| `plugin` | `dwb_core::DWBLocalPlanner` | DWB 局部规划器 |
| `max_vel_x` | `0.26` | 最大前进线速度（m/s）。P3DX 跑得不快，设保守值 |
| `min_vel_x` | `0.0` | 最小前进线速度 |
| `max_vel_y` / `min_vel_y` | `0.0` | 横向速度（差分机器人恒 0） |
| `max_vel_theta` | `1.0` | 最大角速度（rad/s） |
| `min_speed_xy` / `max_speed_xy` | `0.0` / `0.26` | 线速度幅值范围 |
| `min_speed_theta` | `0.0` | 角速度幅值下限 |
| `acc_lim_x` / `decel_lim_x` | `2.5` / `-2.5` | 线加速度/减速度上限（m/s²） |
| `acc_lim_theta` / `decel_lim_theta` | `3.2` / `-3.2` | 角加速度/减速度上限（rad/s²） |
| `vx_samples` | `20` | 采样多少档线速度 |
| `vy_samples` | `5` | 采样多少档横向速度 |
| `vtheta_samples` | `20` | 采样多少档角速度 |
| `sim_time` | `1.7` | 前向仿真时长（秒），预测轨迹的长度 |
| `linear_granularity` | `0.05` | 轨迹上相邻点的线距离 |
| `angular_granularity` | `0.025` | 轨迹上相邻点的角距离 |
| `transform_tolerance` | `0.2` | TF 容忍 |
| `xy_goal_tolerance` | `0.25` | DWB 内部的目标位置容差 |
| `trans_stopped_velocity` | `0.25` | 低于此速度视为「停下」 |
| `short_circuit_trajectory_evaluation` | `True` | 轨迹评分到足够好就提前停止，省计算 |
| `stateful` | `True` | 有状态 |
| `critics` | （7 个） | 轨迹评分器（见下） |

### DWB critics（评分器）——决定轨迹「好坏」

每个 critic 给候选轨迹打分，加权求和选最优：

| 参数 | 值 | 含义 |
|---|---|---|
| `BaseObstacle.scale` | `0.02` | 障碍物代价权重（贴近障碍扣分） |
| `PathAlign.scale` | `32.0` | 与全局路径对齐度权重（高=紧贴路径） |
| `GoalAlign.scale` | `24.0` | 朝向目标对齐度权重 |
| `PathDist.scale` | `32.0` | 到路径距离代价权重 |
| `GoalDist.scale` | `24.0` | 到目标距离代价权重 |
| `RotateToGoal.scale` | `32.0` | 原地转向目标的权重（到达后转正朝向） |
| `RotateToGoal.slowing_factor` | `5.0` | 转向时减速系数 |
| `RotateToGoal.lookahead_time` | `-1.0` | 前瞻时间（-1 = 自动） |
| `PathAlign/GoalAlign.forward_point_distance` | `0.1` | 前瞻点距离 |

> `scale` 越大该项影响越大。想让机器人更贴路径就调大 PathAlign/PathDist；想更快到达就调大 GoalDist。

---

## 4. local_costmap（局部代价地图，滚动窗口）

> 机器人周围的动态局部地图，跟随机器人滚动，控制器用它避障。

| 参数 | 值 | 含义 |
|---|---|---|
| `global_frame` | `odom` | 局部代价地图挂在 odom 帧（随机器人动） |
| `robot_base_frame` | `base_link` | 机器人基座帧 |
| `update_frequency` | `5.0` | 代价地图更新频率（Hz） |
| `publish_frequency` | `2.0` | 代价地图发布频率（Hz） |
| `rolling_window` | `true` | 滚动窗口模式（地图窗口随机器人移动） |
| `width` / `height` | `3` / `3` | 窗口尺寸（米），3×3 m 局部窗口 |
| `resolution` | `0.05` | 栅格分辨率（m） |
| `robot_radius` | `0.25` | 机器人半径（m），用于膨胀 |
| `plugins` | `["voxel_layer","inflation_layer"]` | 图层：体素障碍层 + 膨胀层 |

### inflation_layer（膨胀层）

| 参数 | 值 | 含义 |
|---|---|---|
| `cost_scaling_factor` | `3.0` | 代价衰减系数。越大，代价随距离衰减越快（离障碍稍远就安全） |
| `inflation_radius` | `0.55` | 膨胀半径（米）。障碍物周围 0.55 m 内都被标记为有代价，规划时绕开 |

### voxel_layer（体素障碍层，3D）

| 参数 | 值 | 含义 |
|---|---|---|
| `enabled` | `True` | 启用该层 |
| `publish_voxel_map` | `True` | 发布体素地图（RViz 可视化用） |
| `origin_z` | `0.0` | 体素 Z 起点 |
| `z_resolution` | `0.05` | Z 方向分辨率（m） |
| `z_voxels` | `16` | Z 方向体素数 |
| `max_obstacle_height` | `2.0` | 多高的障碍算障碍（米），高于此忽略 |
| `mark_threshold` | `0` | 标记为障碍的阈值 |
| `observation_sources` | `scan` | 数据源 |
| `scan.topic` | `/scan` | 激光话题 |
| `scan.data_type` | `LaserScan` | 数据类型 |
| `scan.marking` / `clearing` | `True` | 标记障碍 / 清除障碍（扫过没东西就清掉） |
| `scan.raytrace_max_range` | `3.0` | 射线追踪最大距离（判断可通行） |
| `scan.obstacle_max_range` | `2.5` | 障碍标记最大距离 |
| `scan.obstacle_min_range` | `0.0` | 障碍标记最小距离 |

---

## 5. global_costmap（全局代价地图）

> 整个地图的静态代价地图，挂在 map 帧，全局规划器用它找全局路径。

| 参数 | 值 | 含义 |
|---|---|---|
| `global_frame` | `map` | 挂在 map 帧 |
| `update_frequency` / `publish_frequency` | `1.0` | 更新/发布频率（Hz），全局图变化慢，1 Hz 够 |
| `track_unknown_space` | `true` | 跟踪未知区域（未知格子不当作自由空间） |
| `resolution` | `0.05` | 栅格分辨率 |
| `robot_radius` | `0.25` | 机器人半径 |
| `plugins` | `["static_layer","obstacle_layer","inflation_layer"]` | 图层：静态地图层 + 障碍层 + 膨胀层 |

- `static_layer`：加载 map_server 提供的静态地图，`map_subscribe_transient_local: True` 保证能收到 transient local 的地图话题。
- `obstacle_layer`：和 local 的 voxel_layer 类似，把激光点标记成障碍（`enabled: True`，`observation_sources: scan`，参数同 local 的 scan 段）。
- `inflation_layer`：膨胀层，参数同 local（`cost_scaling_factor: 3.0`，`inflation_radius: 0.55`）。

---

## 6. planner_server（全局规划器）

| 参数 | 值 | 含义 |
|---|---|---|
| `expected_planner_frequency` | `20.0` | 期望规划频率（Hz） |
| `planner_plugins` | `["GridBased"]` | 规划器插件 |
| `GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | NavFn 全局规划器（基于 Dijkstra/势场法） |
| `GridBased.tolerance` | `0.5` | 路径容差（米） |
| `GridBased.use_astar` | `false` | 是否用 A*（false = 用 Dijkstra）。Dijkstra 保证最短，A* 更快但可能不最优 |
| `GridBased.allow_unknown` | `true` | 允许路径穿过未知区域。false 则必须全程已知 |

---

## 7. smoother_server（路径平滑）

| 参数 | 值 | 含义 |
|---|---|---|
| `smoother_plugins` | `["simple_smoother"]` | 平滑器插件 |
| `simple_smoother.plugin` | `nav2_smoother::SimpleSmoother` | 简单路径平滑器（对全局路径做平滑，去掉急弯） |
| `simple_smoother.tolerance` | `1.0e-10` | 平滑迭代收敛阈值 |
| `simple_smoother.max_its` | `1000` | 最大迭代次数 |
| `simple_smoother.do_refinement` | `True` | 是否做细化平滑 |

---

## 8. behavior_server（恢复行为）

> 机器人卡住时触发的恢复动作：原地转、后退等。

| 参数 | 值 | 含义 |
|---|---|---|
| `costmap_topic` | `local_costmap/costmap_raw` | 使用的局部代价地图话题 |
| `footprint_topic` | `local_costmap/published_footprint` | 机器人轮廓话题 |
| `cycle_frequency` | `10.0` | 行为执行循环频率（Hz） |
| `behavior_plugins` | `spin, backup, drive_on_heading, assisted_teleop, wait` | 启用的恢复行为 |
| `global_frame` | `odom` | 全局帧 |
| `robot_base_frame` | `base_link` | 基座帧 |
| `transform_tolerance` | `0.1` | TF 容忍 |
| `simulate_ahead_time` | `2.0` | 前向仿真时间（检查是否会撞） |
| `max_rotational_vel` / `min_rotational_vel` | `1.0` / `0.4` | 恢复旋转的速度上下限（rad/s） |
| `rotational_acc_lim` | `3.2` | 旋转加速度上限 |

各行为插件含义：
- `spin`：原地旋转，转到朝向目标或找到出路。
- `backup`：后退一段距离（退出死胡同）。
- `drive_on_heading`：沿固定朝向直行。
- `wait`：原地等待。
- `assisted_teleop`：辅助遥控（手动接管）。

---

## 9. map_server / map_saver（地图加载 / 保存）

| 参数 | 值 | 含义 |
|---|---|---|
| `map_server.yaml_filename` | `""` | 要加载的地图 yaml 路径。实际由 launch 文件的 `map` 参数覆盖传入 |
| `map_saver.save_map_timeout` | `5.0` | 保存地图等待地图消息的超时（秒） |
| `map_saver.free_thresh_default` | `0.25` | 判定「自由空间」的占据率阈值（低于此=自由） |
| `map_saver.occupied_thresh_default` | `0.65` | 判定「占用」的占据率阈值（高于此=障碍） |
| `map_saver.map_subscribe_transient_local` | `True` | 订阅 transient local 地图话题（配合 map_server 用） |

---

## 10. velocity_smoother（速度平滑器）

> 在控制器输出和实际下发 `cmd_vel` 之间做加减速平滑，避免速度突变打滑。

| 参数 | 值 | 含义 |
|---|---|---|
| `smoothing_frequency` | `20.0` | 平滑循环频率（Hz） |
| `scale_velocities` | `False` | 是否按比例整体缩放速度（保持方向不变地降速） |
| `feedback` | `OPEN_LOOP` | 反馈方式。开环：不依赖里程计反馈直接输出；闭环 `CLOSED_LOOP` 会用里程计修正 |
| `max_velocity` | `[0.26, 0.0, 1.0]` | 速度上限 [vx, vy, vtheta] |
| `min_velocity` | `[-0.26, 0.0, -1.0]` | 速度下限 |
| `max_accel` | `[2.5, 0.0, 3.2]` | 加速度上限 |
| `max_decel` | `[-2.5, 0.0, -3.2]` | 减速度上限 |
| `odom_topic` | `odom` | 里程计话题（闭环反馈用） |
| `odom_duration` | `0.1` | 里程计数据保留时长 |
| `deadband_velocity` | `[0,0,0]` | 死区速度（低于此视为 0，防止抖动） |
| `velocity_timeout` | `1.0` | 速度指令超时（秒），超时则停机保护 |

---

## 11. waypoint_follower（航点跟随）

| 参数 | 值 | 含义 |
|---|---|---|
| `loop_rate` | `20` | 执行循环频率（Hz） |
| `stop_on_failure` | `false` | 某航点失败是否整体停下 |
| `waypoint_task_executor_plugin` | `wait_at_waypoint` | 航点任务执行器 |
| `wait_at_waypoint.enabled` | `True` | 启用「到航点等待」 |
| `wait_at_waypoint.waypoint_pause_duration` | `200` | 每个航点停留时长（ms） |

---

## 调参速查（本项目常用）

| 想改什么 | 改哪里 |
|---|---|
| 机器人跑太快/太慢 | `controller_server` 的 `FollowPath.max_vel_x`、`max_vel_theta` |
| 转弯太急/太缓 | `acc_lim_theta`、`max_vel_theta` |
| 靠障碍太近才绕 | `inflation_radius`（local/global 的 inflation_layer） |
| 到达点后朝向不对 | `general_goal_checker.yaw_goal_tolerance` |
| 卡住后恢复 | `behavior_server` 的 spin/backup |
| 全局路径穿未知区域 | `planner_server.GridBased.allow_unknown` |
| 定位飘 | `amcl` 的 `alpha*`（调大=更不信任里程计，粒子更散） |
