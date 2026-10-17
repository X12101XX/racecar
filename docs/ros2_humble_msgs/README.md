# ROS 2 Humble 消息定义（本地固化副本）

来源：`https://github.com/ros2/common_interfaces` 分支 `humble`（Apache-2.0），
文件名格式 `包名__消息名.msg`，方便离线查阅与自动化核对。

用途：`tests/test_ros_msg_contract.py` 用它来核对本仓库 ROS2 节点里用到的
每个消息字段（例如 `Odometry.twist.twist.linear.x`、`Marker.points`），
避免"上车才发现字段名/常量写错"。这套核对是本机唯一能做的真车接口验证，
**仍然不能替代在小车上跑一次 `ros2 run racecar_mission preflight`**。

核对范围（本程序实际用到的）：
sensor_msgs/Image、sensor_msgs/LaserScan、sensor_msgs/Imu、sensor_msgs/CompressedImage、
nav_msgs/Odometry、geometry_msgs/{Twist,TwistWithCovariance,Pose,PoseWithCovariance,Point,Vector3,Quaternion}、
std_msgs/{Header,ColorRGBA,String}、visualization_msgs/{Marker,MarkerArray}
