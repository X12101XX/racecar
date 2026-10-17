"""RobotIO 抽象接口：仿真与 ROS2 实车共用同一套任务逻辑。"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence, Tuple

import numpy as np

from .lidarproc import Scan


@dataclass
class ImuSample:
    t: float
    yaw: Optional[float] = None            # 绝对航向（弧度，可选）
    yaw_rate: float = 0.0                  # 角速度（弧度/秒，z 轴）
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class OdomSample:
    t: float
    v: float = 0.0                         # 前进速度（米/秒）
    w: float = 0.0                         # 角速度（弧度/秒）


class RobotIO(abc.ABC):
    """平台适配层：仿真与 ROS2 节点分别实现。"""

    @abc.abstractmethod
    def now(self) -> float:
        """单调时间（秒）。"""

    def scan(self) -> Optional[Scan]:
        """最新一帧激光（无新数据返回 None）。"""
        return None

    def image(self) -> Optional[Tuple[np.ndarray, float]]:
        """最新一帧图像 (RGB uint8, 时间戳)。"""
        return None

    def imu(self) -> Optional[ImuSample]:
        return None

    def odom(self) -> Optional[OdomSample]:
        return None

    @abc.abstractmethod
    def cmd(self, v: float, w: float) -> None:
        """发布速度指令。"""

    def speak(self, text: str) -> None:
        """语音播报（实车接 TTS / 语音模块）。"""

    def display(self, lines: Sequence[str]) -> None:
        """在显示屏/日志上显示文本。"""

    def log(self, event: str, **fields: Any) -> None:
        """记录结构化事件（供赛后分析与技术文档使用）。"""
