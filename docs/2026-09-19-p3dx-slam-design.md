# p3dx_slam 功能包设计

- 日期：2026-09-19
- 状态：设计已获批，待实现
- 目标环境：ROS2 Humble + Gazebo 11（classic）+ gazebo_ros_pkgs 3.9.0

## 1. 目标

建立 `p3dx_slam` 功能包，用 `slam_toolbox` 实现 Pioneer 3DX 的定位建图基本功能，配合 Gazebo 仿真。为未来引入 nav2 预留接口（frame/topic 命名对齐 nav2 惯例），**本期不实现 nav2**。

## 2. 技术路线

| 组件 | 选择 | 说明 |
|---|---|---|
| 差速驱动 | `gazebo_ros_diff_drive` 插件 | 不用 ros2_control，简单够用 |
| 激光雷达 | `gazebo_ros_ray_sensor` 插件 | 输出 `sensor_msgs/LaserScan` |
| 建图 | `slam_toolbox` online sync 模式 | 边遥控边实时建图 |
| 遥控 | `teleop_twist_keyboard` | 发 `/cmd_vel` 驱动仿真车 |
| 仿真 | Gazebo 11 + gazebo_ros_pkgs | 经典 gazebo 路线（非 ros_gz） |

## 3. TF 树与数据流

```
teleop_twist_keyboard ──/cmd_vel──▶ gazebo_diff_drive ──发布 odom→base_link──▶ TF
gazebo_ray_sensor ──/scan──▶ slam_toolbox ──发布 map→odom + /map──▶ TF + rviz
robot_state_publisher ──发布 base_link→lms100 等静态 TF──▶ TF
```

TF 命名（对齐 nav2 惯例）：

```
map  ←slam_toolbox→  odom  ←gazebo_diff_drive→  base_link  ←rsp→  lms100
```

## 4. 项目结构

```
p3dx_slam/
├── package.xml                 # 依赖 gazebo_ros / slam_toolbox / robot_state_publisher
│                               #      / joint_state_publisher / xacro / rviz2
├── CMakeLists.txt              # install: launch config worlds
├── launch/
│   └── gazebo_slam.launch.py   # gazebo + spawn 车 + rsp + jsp + slam_toolbox + rviz
├── config/
│   ├── slam_toolbox_params.yaml
│   └── rviz_slam.rviz
├── worlds/
│   └── p3dx_world.world        # 地面 + 围墙 + 障碍物 + 灯光
└── urdf/                       # 留空，直接复用描述包的 xacro（见第 5 节）
```

## 5. 对 p3dx_description_ros 的改造（方案 A）

在现有描述包内做 ROS2 化，`p3dx_slam` 通过 `xacro:include` 复用其几何：

1. `urdf/pioneer3dx.gazebo`：
   - `libgazebo_ros_diff_drive.so` 参数由 camelCase 改为 snake_case，改用 `<ros>` 标签
   - `libgazebo_ros_p3d.so`（ground truth）删除，避免 `map` frame 与 slam_toolbox 冲突
   - `libgazebo_ros_laser.so` → `libgazebo_ros_ray_sensor.so`
   - `libgazebo_ros_control.so` 删除
2. `urdf/pioneer3dx_wheel.xacro`：移除 `<transmission>`（ros1 遗留）
3. 纠正左右轮关节对应：`left_joint=base_left_wheel_joint`、`right_joint=base_right_wheel_joint`
   （原 ROS1 配置写反为 `leftJoint=base_right_wheel_joint`）

## 6. 关键参数

| 项 | 值 |
|---|---|
| diff_drive 左/右轮关节 | `base_left_wheel_joint` / `base_right_wheel_joint` |
| wheel_separation / wheel_diameter | `0.39` / `0.15` |
| odometry_frame / robot_base_frame | `odom` / `base_link` |
| 激光 topic / frame | `/scan` / `lms100` |
| slam_toolbox frame | `map` / `odom` / `base_link`，scan_topic=`/scan` |

## 7. 验证方式

1. `ros2 topic list` 出现 `/scan` `/cmd_vel` `/map` `/odom`
2. `ros2 run tf2_ros tf2_echo map base_link` 有变换且随车移动
3. rviz 遥控绕场一圈，`Map` 逐渐铺满 → 建图成功
4. 用 slam_toolbox 的 `save_map` 服务（或临时用 `map_saver_cli`）导出 `.pgm/.yaml`

## 8. 非目标（本期不做）

- nav2 骨架与集成
- ros2_control
- ground truth / 定位精度评估
