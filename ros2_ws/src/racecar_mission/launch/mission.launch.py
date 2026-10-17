"""穿越火线任务启动文件。

    ros2 launch racecar_mission mission.launch.py route:=A
    ros2 launch racecar_mission mission.launch.py route:=B use_card:=true
    ros2 launch racecar_mission mission.launch.py publish_cmd:=false   # 架空测试

参数说明：
    route         A / B，对应 config/route_a.json 与 config/route_b.json
    use_card      true 时先识别蓝/红色卡再决定路线（线下/总决赛）
    publish_cmd   false 时只跑感知与规划、不发布速度（架空空转自检）
    record        true 时同时启动录像节点（相机 + 识别框 + 播报文字 -> ~/mission_video.mp4）
    tts           auto / none（没有语音模块时用显示代替播报）
    params_file   额外参数文件（默认包内 config/car.yaml）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("racecar_mission"), "config", "car.yaml"])
    args = [
        DeclareLaunchArgument("route", default_value="A"),
        DeclareLaunchArgument("use_card", default_value="false"),
        DeclareLaunchArgument("monochrome", default_value="true"),
        DeclareLaunchArgument("ai_backend", default_value="auto"),
        DeclareLaunchArgument("publish_cmd", default_value="false"),
        DeclareLaunchArgument("tts", default_value="auto"),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("video_out", default_value="~/mission_video.mp4"),
        DeclareLaunchArgument("params_file", default_value=default_params),
    ]
    node = Node(
        package="racecar_mission",
        executable="mission_node",
        name="racecar_mission",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "route": LaunchConfiguration("route"),
                "use_card": LaunchConfiguration("use_card"),
                "monochrome": LaunchConfiguration("monochrome"),
                "ai_backend": LaunchConfiguration("ai_backend"),
                "publish_cmd": LaunchConfiguration("publish_cmd"),
                "tts": LaunchConfiguration("tts"),
                "frame_id": LaunchConfiguration("frame_id"),
            },
        ],
    )
    recorder = Node(
        package="racecar_mission",
        executable="record_node",
        name="mission_recorder",
        output="screen",
        condition=IfCondition(LaunchConfiguration("record")),
        parameters=[{"out_path": LaunchConfiguration("video_out"),
                     "draw_detections": True}],
    )
    return LaunchDescription(args + [node, recorder])
