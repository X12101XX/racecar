"""环境检查：列出关键依赖是否可用（自检脚本第一步）。"""

from __future__ import annotations

import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main() -> int:
    print("python:", sys.version.split()[0], "->", sys.executable)
    need_numpy = True
    for mod in ("numpy", "PIL", "scipy", "cv2", "matplotlib"):
        try:
            m = __import__(mod)
            print(f"  {mod:10s} OK   {getattr(m, '__version__', '')}")
        except Exception as exc:  # noqa: BLE001
            tag = "必需" if mod in ("numpy",) else "可选"
            print(f"  {mod:10s} --   缺失（{tag}）：{exc.__class__.__name__}")
            if mod == "numpy":
                need_numpy = False
    print("ffmpeg:", shutil.which("ffmpeg") or "缺失（只影响生成 mp4，PNG 帧仍会保留）")
    try:
        from racelib import imageops
        print("图像后端:", imageops.BACKEND, "（cv2 > scipy > numpy 自动选择）")
    except Exception as exc:  # noqa: BLE001
        print("导入 racelib 失败：", exc)
        return 1
    return 0 if need_numpy else 1


if __name__ == "__main__":
    raise SystemExit(main())
