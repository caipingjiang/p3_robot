# Pioneer 3DX nav2 导航功能设计

## 目标

在现有 `p3dx_slam`（slam_toolbox 建图）基础上，新增 `p3dx_nav` 包，用 **nav2** 实现"建图 → 保存地图 → 加载地图定位 → 规划导航"的经典两步流程。

## 架构

经典两步法，两个阶段、两套 launch：

```
【建图阶段 · p3dx_slam 现有】
gazebo_slam.launch.py → gazebo + 机器人 + slam_toolbox(mapping) + rviz
        │  遥控建图 → map_saver_cli 保存
        ▼
p3dx_nav/maps/p3dx_map.{pgm, yaml}

【导航阶段 · p3dx_nav 新增】
nav_bringup.launch.py → gazebo + 机器人 + map_server + amcl + nav2 + rviz
```

两阶段唯一区别：`map` 帧的来源。建图时由 `slam_toolbox` 发 `/map` 和 `map→odom` TF；导航时由 `map_server` 发 `/map`、`amcl` 发 `map→odom` TF。

## 组件与数据流（导航阶段）

TF 树：`map ←amcl→ odom ←diff_drive→ base_link`（`diff_drive` 已发布 `odom→base_link`）。

| 节点 | 订阅 | 发布 | 说明 |
|---|---|---|---|
| `map_server` | - | `/map` | 加载预存地图 |
| `amcl` | `/scan` `/map` `/odom` + TF | `map→odom` TF, `/amcl_pose` | 粒子滤波定位 |
| `planner_server` | `/map` + TF | `/plan` | 全局路径（NavFn） |
| `controller_server` | `/scan` `/plan` `/odom` + TF | `/cmd_vel` | 局部控制（DWB） |
| `bt_navigator` | - | 编排各 server | 行为树导航 |
| `diff_drive`（已有） | `/cmd_vel` | `/odom` + TF | 驱动轮子 |

## 包结构

```
p3dx_slam/（改造）
  launch/bringup.launch.py        # 新增：共享的 gazebo + 机器人启动
  launch/gazebo_slam.launch.py    # 改造：include bringup + slam_toolbox + rviz

p3dx_nav/（新建）
  package.xml / CMakeLists.txt
  launch/nav_bringup.launch.py    # include p3dx_slam/bringup + nav2_bringup + rviz
  config/nav2_params.yaml         # 基于 nav2_bringup 默认定制
  config/nav_rviz.rviz            # 导航 RViz（2D Nav Goal + 2D Pose Estimate）
  maps/                           # 预存地图（建图后保存到这里）
```

### 共享 bringup.launch.py 的抽取

从现有 `gazebo_slam.launch.py` 抽出前 6 步（与导航共用）：

1. `SetEnvironmentVariable(GAZEBO_MODEL_PATH)`（package:// mesh 修复）
2. `gazebo`（加载 world）
3. `robot_description`（xacro 生成）
4. `spawn_entity`（spawn 机器人）
5. `robot_state_publisher`
6. `joint_state_publisher`

`gazebo_slam.launch.py` 改为：`include bringup` + `slam_toolbox` + `rviz`（建图配置）。

## 技术选型

| 项 | 选择 | 备选 |
|---|---|---|
| 全局规划器 | `NavFnPlanner` | `SmacPlanner2D` |
| 局部控制器 | `dwb_core::DWBLocalPlanner` | `RegulatedPurePursuit` / `MPPI` |
| 定位 | AMCL，`likelihood_field` 激光模型，`DifferentialMotionModel` | - |
| 行为树 | `navigate_to_pose_w_replanning_and_recovery`（默认） | - |
| 足迹 | 圆形 `robot_radius: 0.25`（Pioneer 3DX 保守值） | `footprint` 多边形 |

## 参数配置要点（nav2_params.yaml 相对 nav2_bringup 默认的改动）

| 键 | 默认值 | 改为 | 原因 |
|---|---|---|---|
| `amcl.base_frame_id` | `base_footprint` | `base_link` | 本机器人无 base_footprint |
| `amcl.scan_topic` | `scan` | `/scan` | 激光 topic 是绝对名 `/scan` |
| `amcl.global_frame_id` | `map` | `map`（不变） | |
| `amcl.odom_frame_id` | `odom` | `odom`（不变） | |
| `*_costmap.robot_radius` | `0.22` | `0.25` | 保守，避免刮蹭 |
| `*_costmap.obstacle_layer.scan.topic` | `/scan` | `/scan`（不变） | |
| 全部节点 `use_sim_time` | - | `True` | gazebo 仿真时间 |

其余（DWB 参数、代价地图膨胀半径、恢复行为、AMCL 噪声模型）沿用 nav2_bringup 默认值。

## 工作流程

1. **建图**：`ros2 launch p3dx_slam gazebo_slam.launch.py`，用 `teleop_twist_keyboard` 遥控建图，满意后：
   ```
   ros2 run nav2_map_server map_saver_cli -f ~/p3_ws/src/p3dx_nav/maps/p3dx_map
   ```
   生成 `p3dx_map.pgm` + `p3dx_map.yaml`。
2. **导航**：`ros2 launch p3dx_nav nav_bringup.launch.py`，RViz 中：
   - 用 **2D Pose Estimate** 在机器人实际位置给初始位姿（AMCL 收敛）
   - 用 **2D Nav Goal** 点目标点，机器人自动规划并行驶

## 依赖

`p3dx_nav` 的 `package.xml` 需要 `exec_depend`：`nav2_bringup`、`rviz2`、`xacro`、`p3dx_slam`（共享 bringup）、`p3dx_description_ros`。

## 验收标准

- [ ] `gazebo_slam.launch.py` 建图正常，`map_saver_cli` 能保存出地图文件
- [ ] `nav_bringup.launch.py` 启动后 RViz 能显示地图 + 激光 + 机器人模型
- [ ] 2D Pose Estimate 给位姿后 AMCL 能收敛（粒子聚拢）
- [ ] 2D Nav Goal 点目标点后，机器人规划路径并行驶到目标（含避障绕柱）
