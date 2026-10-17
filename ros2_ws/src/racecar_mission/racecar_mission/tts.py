"""语音播报：四级兜底，保证"发车前那句播报"一定能出声或在屏幕上显示。

播放优先级（找到能用的就用）：
  1. 随包发布的录音文件 `assets/announce_A.wav` / `announce_B.wav`
     （本仓库已用 espeak-ng 中文音源生成；**可以直接替换成你自己的录音**，
      发音会更自然，赛题对播报内容没有音色要求）
  2. 现场 TTS 合成：espeak-ng / espeak（中文音源 cmn）
  3. spd-say（Speech Dispatcher）
  4. 都不可用：只打印 + 发布 /mission/announce（赛题允许"播报或显示"）

命令：
    ros2 run racecar_mission speak_test "我将前往路线A，到达据点A并穿越雷区返回营地A。"
    ros2 run racecar_mission speak_test --list
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from typing import List, Optional

PLAYERS = (("paplay", []), ("aplay", ["-q"]), ("pw-play", []))
TTS_ENGINES = (("espeak-ng", ["-v", "cmn", "-s", "150"]),
               ("espeak", ["-v", "cmn", "-s", "150"]))


def assets_dir() -> str:
    """录音文件目录。

    优先用 ament 的包共享目录（`colcon build` 后录音被安装到
    `<prefix>/share/racecar_mission/assets/`，这是车上唯一可靠的路径），
    其次才是源码树里的 assets（本地测试/未安装时用）。
    """
    candidates: List[str] = []
    try:                                  # 安装后（车上）的规范路径
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory("racecar_mission"),
                                       "assets"))
    except Exception:                     # noqa: BLE001 - 没装 ROS2 时走下面的源码路径
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates += [
        os.path.join(os.path.dirname(here), "assets"),          # 源码树：包目录/assets
        os.path.join(here, "assets"),
        os.path.normpath(os.path.join(here, "..", "..", "..", "share",
                                      "racecar_mission", "assets")),
        "/opt/racecar_crossfire/assets",
    ]
    for cand in candidates:
        if cand and os.path.isdir(cand):
            return os.path.normpath(cand)
    return os.path.normpath(os.path.join(os.path.dirname(here), "assets"))


def pick_wav(text: str, directory: Optional[str] = None) -> Optional[str]:
    """按播报文本挑对应的录音文件（路线 A/B）。"""
    directory = directory or assets_dir()
    if not os.path.isdir(directory):
        return None
    keys: List[str] = []
    for rid in ("A", "B"):
        if f"路线{rid}" in text or f"路线 {rid}" in text:
            keys.append(rid.lower())
    for key in keys:
        path = os.path.join(directory, f"announce_{key}.wav")
        if os.path.exists(path):
            return path
    return None


def _play_file(path: str) -> bool:
    for player, args in PLAYERS:
        if shutil.which(player) is None:
            continue
        try:
            subprocess.Popen([player, *args, path],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:  # noqa: BLE001
            continue
    return False


def _speak_tts(text: str) -> bool:
    for engine, args in TTS_ENGINES:
        if shutil.which(engine) is None:
            continue
        try:
            subprocess.Popen([engine, *args, text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:  # noqa: BLE001
            continue
    if shutil.which("spd-say"):
        try:
            subprocess.Popen(["spd-say", "-l", "zh", text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:  # noqa: BLE001
            pass
    return False


def speak(text: str, prefer_recording: bool = True) -> str:
    """播报（非阻塞）。返回实际使用的方式：recording / tts / display。"""
    if prefer_recording:
        wav = pick_wav(text)
        if wav and _play_file(wav):
            return "recording"
    if _speak_tts(text):
        return "tts"
    print(f"[tts] 无可用播放器，改用显示：{text}")
    return "display"


def available() -> List[str]:
    out: List[str] = []
    d = assets_dir()
    if os.path.isdir(d):
        out += [f for f in sorted(os.listdir(d)) if f.endswith(".wav")]
    out += [p for p, _ in PLAYERS if shutil.which(p)]
    out += [e for e, _ in TTS_ENGINES if shutil.which(e)]
    return out


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] in ("--list", "-l"):
        print("可用资源：", ", ".join(available()) or "（无）")
        print("录音目录：", assets_dir())
        return 0
    text = args[0] if args else "我将前往路线A，到达据点A并穿越雷区返回营地A。"
    how = speak(text)
    print(f"[speak] 使用方式：{how} | {text}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
