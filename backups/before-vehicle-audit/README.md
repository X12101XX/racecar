# 穿越火线：本机 ROS2 仿真与独立实车包（v2）

当前交付针对**所有物料均不区分颜色的线上赛**。显式选择路线 A 或 B，不识别起步色卡，不依赖地面色块，所有锥桶统一避障。A：起点 → 左上据点 A → 停稳后等待至少 3 秒 → 雷区 → 右下营地 A；B 对称前往左下据点 B、右上营地 B。

历史运行记录直接证明小车使用 **ROS2 Humble**。保存的底盘源码订阅 `/car_cmd_vel`，把 `linear.x` 换算成油门 PWM，把 `angular.z` 直接换算成转向舵机 PWM。因此本版使用**阿克曼运动约束**，禁止差速车的原地转向。规划先寻找全程前进路径，倒车仅作受距离上限约束的机动备选；前置摄像头始终与车头同向，不随倒车自动转向。历史源码在 `docs/evidence/`，当前小车二进制、舵机比例和轴距仍须核实。


## 新增：现场配置界面与运动预设

```bash
bash tools/configure.sh
```

打开 http://127.0.0.1:8765/ ，可拖动起点/区域/锥桶、修改尺寸和非等距封锁线，填写略歪围栏的四角；保存历史版本，导出仿真/实车共用配置 ZIP。保守、标准、激进三种运动预设和细调参数均接入控制器。界面会显示实车已标定上限，默认不会解除实车输出锁定。

详见 [现场配置与运动调参](docs/05_现场配置与运动调参.md)。例如：

```bash
bash tools/run_ros_sim.sh --profile measured_example --route A
# 也可直接传入界面导出的源 JSON（路径可含空格/中文）
bash tools/run_ros_sim.sh --profile "/绝对路径/scene.json" --route B
bash tools/run_ros_sim.sh --route A --motion-preset aggressive
```

## 先运行本机仿真

本机已构建 Docker 镜像 `crossfire-sim:humble-v2`。没有向宿主安装 ROS2 系统包，没有改 `.bashrc`、网络或小车代码。Docker 访问权限按电脑原来的配置使用；不要为了此项目修改 docker.sock 权限。

```bash
cd '/home/x12101x/文档/ChatGPT/racecar'
bash tools/run_ros_sim.sh --route A --outdir /output/ros_demo_A
bash tools/run_ros_sim.sh --route B --outdir /output/ros_demo_B
```

仿真有真实的 ROS2 Humble 进程和 DDS 消息链路：

```text
赛场与自行车运动学 → LaserScan / Imu / Odometry / Image
                       ↓
                任务节点：定位、建图、Hybrid A*、路径跟踪
                       ↓ /crossfire/desired_cmd（SI 速度）
                独立安全网关：数据超时、车体扫掠、限速、急停
                       ↓ /crossfire/sim_cmd
                仿真车辆（无硬件接口）
```

仿真结束后终端会打印本次视频的完整路径和 `mpv` 命令。最近一次已结束的结果还可通过 `output/latest/video.mp4` 打开，路线 A/B 分别有 `output/latest_A/`、`output/latest_B/`。传 `--profile` 时还会生成该配置专用入口，例如 `output/latest_measured_example_A/video.mp4`，其他配置的运行不会覆盖它。这些入口会指向对应结果目录；旧 `output/demo_routeA/` 不会变成本次结果。未生成新结果时保留原入口并明确报错，不能据此判断新任务已通过。

视频俯视图的橙色箭头/扇区表示车头与前摄朝向，青色箭头表示实际移动方向；顶部明确显示 `FORWARD / REVERSE / STOP`。`summary.json` 新增 `travel_distance_m.forward/reverse`，可检查是否长距离倒车。

每条命令完成后生成 `output/ros_demo_A/` 或 `output/ros_demo_B/`：`video.mp4`、`final.png`、`summary.json`、`trajectory.csv`、`events.json`、`gateway.json`、各节点日志。退出码 0 才表示该次真实 ROS2 闭环通过。视频为仿真证据，不能代替比赛要求的真车录像。

默认 `--scenario demo` 使用公开在源码中的固定合规演示布局（12 个封锁锥、5 个其他障碍，3 条线间距 1.5m）。控制器只读模拟传感器，不接收仿真真值或地雷位置。所有物料渲染成灰色；图中的轨迹与车体使用青色便于查看。

```bash
# 随机场地；退出码非零可能是安全停车/规划失败，不能算完赛
bash tools/run_ros_sim.sh --scenario random --route A --seed 0 --outdir /output/ros_random_A

# 重建隔离镜像（依赖下载只发生在镜像内）
bash tools/setup_sim.sh

# 快速算法仿真，无 ROS2，适合修改规划参数后快速检查
python3 -m sim.cli --route A --no-video
python3 -m sim.cli --scenario random --route B --seed 0 --no-video

# 正式的单元、固定场景和 ROS2 闭环验证
bash tools/selfcheck.sh
```

物理模型包括转向半径、速度响应延迟、车辆矩形碰撞、雷达射线、传感器噪声和里程计误差。它不是 Gazebo，也没有模拟真实轮胎打滑、舵机滞后和电调死区；这些需要实车标定。默认车体 0.42×0.30m、轴距 0.30m、最大舵角 0.52rad 是**可修改的仿真假设**。

## 交付到小车

使用 `release/crossfire-car-v2.run`；它是可审计的自解压源码安装包，不含电脑的 x86 二进制，可在已有 Python 3.10+ / ROS2 Humble 的昇腾 ARM64 板上构建。它不安装系统依赖、不修改原有 ROS 工作空间、不添加自启动、不写串口或 GPIO。

把 `.run` 复制到小车后，用普通用户执行：

```bash
bash crossfire-car-v2.run
# 解压成功会打印一个全新的目录，例如 ~/crossfire-releases/crossfire-20261017-...
cd '上一步打印的目录'
bash tools/build_car.sh
bash tools/run_car.sh A --preflight  # 只订阅传感器，不发任何速度（连零速度也不发）
bash tools/run_car.sh A              # 默认只读规划，Ctrl+C 结束
```

只读模式不会接管底盘，也不会自动停止已有导航。底盘与传感器仍按原来的方式启动。部署器不会猜密码、自动 SSH 或调用 `rsync --delete`。校验失败、缺依赖或版本不符会报错退出；不会自动用 apt/pip 修改小车。

**实车动作在交付包中默认锁定。** 首次架空核实和标定后，需要在 `config/vehicle_profile.json` 填写当前话题、尺寸、限速，确认看门狗、舵机映射、传感器坐标和起点/场地配置。倒车默认关闭；只有完成后向覆盖与底盘倒车验证，才可显式打开。检查项全部满足后：

```bash
bash tools/run_car.sh A --arm
# 或 B；只有 --arm 才会启动可发布 /car_cmd_vel 的安全网关。
```

该命令不会杀死原有控制器。若发现另一发布者或 `/teleop_cmd_vel` 发布者，网关锁存停车，须结束本次任务、由现场操作者处理控制权后重新启动。独立包不能改变原底盘固件的安全性；突然断电、物理急停失效、原驱动无超时保护等情况，不能由应用层程序作绝对保证。

## 安全边界与验证

- 默认任务节点与网关不创建底盘控制发布者；即便零速度也可能干扰旧程序，所以只读模式完全不发。
- 任务节点只允许发布私有的 `/crossfire/desired_cmd`，不能把输出直接改到原底盘话题。
- 网关在独立进程中用墙钟监控：控制指令超过 0.30s、传感器超过 0.50s、无有效后向覆盖、预测车体扫掠有障碍时归零。
- IMU 碰撞、非法控制数字、急停与控制权冲突会触发保护。急停话题为 `/crossfire/estop`，Bool true 锁存，false 不会解除。
- 路径无解时停车，不盲目前进或原地转向。计算重任务在后台线程执行。进程崩溃后的最终保护依赖底盘自身已验证的超时停车。
- 每次运行独立日志/配置副本；只用进程级 ROS 环境，不修改 shell 初始化文件。退出只结束自己的进程。

当前实测数据、失败场景和上车前还缺的验证见 [交付验证报告](docs/03_测试报告.md)，完整部署步骤见 [车辆部署指南](docs/02_车辆部署指南.md)。旧版差速/颜色方案保留在 `backups/source-before-20261017.tar.gz`，其旧成功率和旧部署命令不适用于 v2。

## 文件与恢复

- `racelib/ackermann.py`：带前进/倒车、车体扫掠约束的 Hybrid A* 和无颜色任务。
- `ros2_ws/src/racecar_mission/`：ROS2 适配、独立网关、只读接口探测。
- `config/field.json`、`route_a.json`、`route_b.json`：按现场实测改场地和路线。
- `config/vehicle_profile.json`：实车专用接口及确认项，初始不允许动作。
- `containers/Dockerfile`：固定官方基础镜像摘要；运行容器断网、无设备、无特权、只读根目录。
- `release/`：可搬运的仿真源码包、实车 `.run` 和 SHA256 校验文件。

撤销实车安装：Ctrl+C 结束任务，然后删除本次新建的 release 目录即可。它没有覆盖原文件。宿主 ROS2 环境也没有被改变；需要清理时可删除项目的 `.runtime` 并按 Docker 正常操作移除本项目镜像（不要使用全局 prune）。

离线搬运到另一台电脑时，先解压 `release/crossfire-sim-v2.tar.gz`，再恢复镜像：

```bash
docker load -i crossfire-humble-amd64-image.tar.gz
cd crossfire-sim-v2
bash tools/run_ros_sim.sh --route A
```

这两个仿真文件只面向电脑 amd64；小车使用独立的 `crossfire-car-v2.run`。文件完整性可在原项目中运行 `bash tools/check_release.sh` 检查。
