# 地图目录

建图后用 map_saver_cli 把地图保存到这里：

    ros2 run nav2_map_server map_saver_cli -f ~/p3_ws/src/p3dx_nav/maps/p3dx_map

生成 p3dx_map.pgm + p3dx_map.yaml，导航 launch 会加载 p3dx_map.yaml。
