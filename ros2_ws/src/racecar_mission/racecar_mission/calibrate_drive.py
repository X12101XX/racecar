"""底盘标定：测量"指令速度 -> 实际速度"，并算出标定系数与死区（在车上运行）。

    # 只看当前配置、不动车（默认就是干这个）
    ros2 run racecar_mission calibrate_drive

    # 真正标定（**请把车放在空旷处、或架起驱动轮**，会慢速前进与原地转）
    ros2 run racecar_mission calibrate_drive --ros-args -p confirm:=true

流程：以阶梯指令（0.05→0.45 m/s，每档 1.2s）前进，再从 0.1→1.5 rad/s 原地转，
用 /odom 的实际速度做最小二乘拟合，得到：
    - 死区：多小的指令开始有反应 -> 填 plan.json 的 min_linear_cmd_mps / min_angular_cmd_rps
    - 增益：达到目标速度需要放大多少倍 -> 填 cmd_gain_linear / cmd_gain_angular
结果同时写入 ~/drive_calib.json，并把可直接粘贴的 JSON 片段打印出来。

任何异常/退出都会连续下发零速度指令停车。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from .ros_io import RosRobotIO
from .topics import detect_topics


def fit_cmd_to_actual(cmds: List[float], actuals: List[float]
                      ) -> Tuple[Optional[float], Optional[float]]:
    """最小二乘拟合 actual = a*cmd + b，返回 (a, b)。样本不足返回 (None, None)。"""
    if len(cmds) < 3:
        return None, None
    x = np.asarray(cmds, dtype=float)
    y = np.asarray(actuals, dtype=float)
    keep = y > 0.02
    if keep.sum() < 3:
        return None, None
    x, y = x[keep], y[keep]
    a, b = np.polyfit(x, y, 1)
    return float(a), float(b)


def summarize_test(cmds: List[float], actuals: List[float], unit: str) -> Dict[str, float]:
    """给出死区与标定系数建议。"""
    a, b = fit_cmd_to_actual(cmds, actuals)
    deadband = 0.0
    for c, y in zip(cmds, actuals):
        if y > 0.03:
            deadband = float(c)
            break
    out = {"deadband": round(deadband, 3)}
    if a is not None and a > 0.15:
        out["slope"] = round(a, 3)
        out["intercept"] = round(b, 3)
        out["suggest_gain"] = round(min(3.0, max(0.5, 1.0 / a)), 2)
        out["suggest_min_cmd"] = round(max(0.0, -b / a), 3)
    return out


class CalibrateNode(Node):
    def __init__(self, confirm: bool, steps_linear: float, steps_angular: float,
                 dt_step: float):
        super().__init__("calibrate_drive")
        if confirm:
            raise ValueError("此旧标定动作仅适用于差速车；当前阿克曼部署包禁止自动标定，请先核实舵机映射和底盘看门狗")
        self.confirm = False
        self.steps_linear = steps_linear
        self.steps_angular = steps_angular
        self.dt_step = dt_step
        self.declare_parameter("topics.odom", "")
        self.declare_parameter("topics.cmd_vel", "")
        prefer = {"odom": str(self.get_parameter("topics.odom").value),
                  "cmd_vel": str(self.get_parameter("topics.cmd_vel").value)}
        self.topics = detect_topics(self, prefer)
        self.io = RosRobotIO(self, self.topics, {"tts": "none"})
        if not self.confirm:
            self.get_logger().warn(
                "当前是【只读模式】：不会发速度指令。确认环境安全后加 "
                "-p confirm:=true 才会真正标定。")
        self.get_logger().info(f"话题：{self.topics}")

    # ------------------------------------------------------------------ #
    def _run_step(self, v: float, w: float, seconds: float) -> Tuple[float, float]:
        """保持一个指令一段时间，返回后半段 /odom 的平均 (v, w)。"""
        t0 = time.monotonic()
        vs: List[float] = []
        ws: List[float] = []
        while time.monotonic() - t0 < seconds:
            self.io.cmd(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)
            odom = self.io._odom
            if time.monotonic() - t0 > seconds * 0.4:
                vs.append(abs(float(odom.v)))
                ws.append(abs(float(odom.w)))
            time.sleep(0.03)
        self.io.cmd(0.0, 0.0)
        if time.monotonic() - t0 < seconds + 0.4:
            t_stop = time.monotonic()
            while time.monotonic() - t_stop < 0.4:
                rclpy.spin_once(self, timeout_sec=0.05)
        return (float(np.mean(vs)) if vs else 0.0,
                float(np.mean(ws)) if ws else 0.0)

    def calibrate(self) -> Dict[str, object]:
        result: Dict[str, object] = {"linear": {}, "angular": {}}
        lin_cmds = [round(0.05 + 0.05 * i, 2) for i in range(int(self.steps_linear))]
        ang_cmds = [round(0.10 + 0.20 * i, 2) for i in range(int(self.steps_angular))]
        if not self.confirm:
            print("将测试前进指令：", lin_cmds)
            print("将测试转动指令：", ang_cmds)
            return result

        print("\n=== 直线测试（车会慢速前进，请确保前方 ≥1.5m 空旷）===")
        v_actual: List[float] = []
        for c in lin_cmds:
            va, _ = self._run_step(c, 0.0, self.dt_step)
            v_actual.append(va)
            print(f"  指令 {c:.2f} m/s -> 实测 {va:.3f} m/s"
                  + ("（没动，属于死区）" if va < 0.03 else ""))
        result["linear"] = {"cmds": lin_cmds, "actual": [round(v, 3) for v in v_actual],
                            **summarize_test(lin_cmds, v_actual, "m/s")}

        print("\n=== 转向测试（原地转，请确保四周 ≥0.5m 空旷）===")
        w_actual: List[float] = []
        for c in ang_cmds:
            _, wa = self._run_step(0.0, c, self.dt_step)
            w_actual.append(wa)
            print(f"  指令 {c:.2f} rad/s -> 实测 {wa:.3f} rad/s"
                  + ("（没转，属于死区）" if wa < 0.05 else ""))
        result["angular"] = {"cmds": ang_cmds, "actual": [round(v, 3) for v in w_actual],
                             **summarize_test(ang_cmds, w_actual, "rad/s")}
        return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="底盘指令标定")
    ap.add_argument("--confirm", action="store_true",
                    help="确认真实驱动（默认只读，不动车）")
    ap.add_argument("--steps-linear", type=float, default=9)
    ap.add_argument("--steps-angular", type=float, default=8)
    ap.add_argument("--dt-step", type=float, default=1.2)
    args = ap.parse_args(argv)

    rclpy.init()
    node = CalibrateNode(args.confirm, args.steps_linear, args.steps_angular, args.dt_step)
    code = 0
    try:
        res = node.calibrate()
        if node.confirm:
            lin = res.get("linear") or {}
            ang = res.get("angular") or {}
            patch = {
                "cmd_gain_linear": lin.get("suggest_gain", 1.0),
                "min_linear_cmd_mps": lin.get("suggest_min_cmd", 0.0),
                "cmd_gain_angular": ang.get("suggest_gain", 1.0),
                "min_angular_cmd_rps": ang.get("suggest_min_cmd", 0.0),
            }
            out = os.path.join(os.path.expanduser("~"), "drive_calib.json")
            with open(out, "w", encoding="utf-8") as f:
                json.dump({"result": res, "plan_patch": patch}, f,
                          ensure_ascii=False, indent=2)
            print("\n=== 建议写入 config/plan.json 的 control 段 ===")
            print(json.dumps(patch, ensure_ascii=False, indent=2))
            print(f"（完整结果：{out}）")
            if not lin.get("suggest_gain"):
                print("⚠ 直线部分没有测到有效数据：确认 /odom 有速度输出、"
                      "底盘确实在转轮子，再重跑一次。")
    finally:
        for _ in range(10):          # 无论出什么事都停车
            node.io.cmd(0.0, 0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
