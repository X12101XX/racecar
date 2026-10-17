# 小车端工作空间（ROS2 Humble）

本目录是交给小车编译的工作空间骨架。**`racelib/` 与 `racecar_mission/config/*.json`
由 `tools/deploy.sh` 从仓库根目录同步进来**，不要在两侧各改一份。

## 编译

```bash
cd ~/racecar_crossfire/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行（真车）

```bash
# 1) 按原有方式启动底盘/雷达/IMU/相机
ros2 launch racecar Run_car_one.launch.py

# 2) 另开终端：接口自检（确认话题名与频率）
ros2 run racecar_mission probe_topics

# 3) 跑任务
ros2 launch racecar_mission mission.launch.py route:=A          # 蓝卡路线
ros2 launch racecar_mission mission.launch.py route:=B          # 红卡路线
ros2 launch racecar_mission mission.launch.py use_card:=true    # 抽色卡时
```

任务结果会写进 `~/mission_result.json`（含分段时间、罚时、事件流）。
