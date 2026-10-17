"""仿真平台的 RobotIO 实现：把仿真传感器接到任务状态机上。"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from racelib.io import ImuSample, OdomSample, RobotIO
from racelib.lidarproc import Scan


class SimIO(RobotIO):
    """带时间戳的传感器缓存：每次只交付“未消费过”的最新样本。

    The mission sees only these simulated sensor messages; world truth is not
    exposed here.
    """

    def __init__(self, t0: float = 0.0):
        self._t = float(t0)
        self._scan: Optional[Scan] = None
        self._scan_served = -1.0
        self._image: Optional[np.ndarray] = None
        self._image_t = -1.0
        self._image_served = -1.0
        self._imu: Optional[ImuSample] = None
        self._imu_served = -1.0
        self._odom: Optional[OdomSample] = None
        self._odom_served = -1.0
        self.last_cmd: Tuple[float, float] = (0.0, 0.0)
        self.cmd_history: List[Tuple[float, float, float]] = []
        self.speaks: List[Tuple[float, str]] = []
        self.displays: List[Tuple[float, List[str]]] = []
        self.events: List[Dict[str, Any]] = []

    # ---- 时间 ----
    def now(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += float(dt)

    # ---- 传感器注入 ----
    def put_scan(self, scan: Scan) -> None:
        self._scan = scan

    def put_image(self, img: np.ndarray, t: float) -> None:
        self._image = img
        self._image_t = float(t)

    def put_imu(self, imu: ImuSample) -> None:
        self._imu = imu

    def put_odom(self, odom: OdomSample) -> None:
        self._odom = odom

    # ---- RobotIO ----
    def scan(self) -> Optional[Scan]:
        if self._scan is not None and self._scan.t > self._scan_served:
            self._scan_served = self._scan.t
            return self._scan
        return None

    def image(self) -> Optional[Tuple[np.ndarray, float]]:
        if self._image is not None and self._image_t > self._image_served:
            self._image_served = self._image_t
            return self._image, self._image_t
        return None

    def imu(self) -> Optional[ImuSample]:
        if self._imu is not None and self._imu.t > self._imu_served:
            self._imu_served = self._imu.t
            return self._imu
        return None

    def odom(self) -> Optional[OdomSample]:
        if self._odom is not None and self._odom.t > self._odom_served:
            self._odom_served = self._odom.t
            return self._odom
        return None

    def cmd(self, v: float, w: float) -> None:
        self.last_cmd = (float(v), float(w))
        self.cmd_history.append((self._t, float(v), float(w)))

    def speak(self, text: str) -> None:
        self.speaks.append((self._t, text))

    def display(self, lines: Sequence[str]) -> None:
        self.displays.append((self._t, list(lines)))

    def log(self, event: str, **fields: Any) -> None:
        rec = {"t": self._t, "event": event}
        rec.update(fields)
        self.events.append(rec)
