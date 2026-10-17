FROM ros:humble-ros-base@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-numpy python3-scipy python3-pil python3-opencv python3-colcon-common-extensions ros-humble-visualization-msgs ffmpeg && rm -rf /var/lib/apt/lists/*
ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /workspace
