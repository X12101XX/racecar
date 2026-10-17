#!/usr/bin/env bash
# 生成（或重新生成）发车播报录音。想用自己的声音就照这个命令录一遍：
#
#   bash tools/record_announcements.sh                  # 用 espeak-ng 中文音源生成
#   bash tools/record_announcements.sh --your-voice     # 交互式：你读一遍，保存到同路径
#
# 生成的文件：ros2_ws/src/racecar_mission/assets/announce_{a,b}.wav
# 部署脚本会把它们一起同步到小车；播报时优先播放录音，找不到才用现场 TTS。
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS="$REPO_DIR/ros2_ws/src/racecar_mission/assets"
mkdir -p "$ASSETS"

TEXT_A="我将前往路线A，到达据点A并穿越雷区返回营地A。"
TEXT_B="我将前往路线B，到达据点B并穿越雷区返回营地B。"

if [[ "${1:-}" == "--your-voice" ]]; then
  for pair in "a:$TEXT_A" "b:$TEXT_B"; do
    key="${pair%%:*}"; text="${pair#*:}"
    echo "请朗读（回车开始 / 回车结束需要手动按 Ctrl+C，建议用 arecord -d 8）："
    echo "  $text"
    arecord -f cd -d 8 -t wav "$ASSETS/announce_${key}.wav"
    echo "已保存 $ASSETS/announce_${key}.wav"
  done
  exit 0
fi

if ! command -v espeak-ng >/dev/null; then
  echo "本机没有 espeak-ng，请在有小车/有 TTS 的机器上生成，或用 --your-voice 自己录。"
  exit 1
fi
espeak-ng -v cmn -s 145 -w "$ASSETS/announce_a.wav" "$TEXT_A"
espeak-ng -v cmn -s 145 -w "$ASSETS/announce_b.wav" "$TEXT_B"
ls -l "$ASSETS"
echo "完成。可用以下命令试听/自检："
echo "  bash tools/record_announcements.sh   # 重新生成"
echo "  ros2 run racecar_mission speak_test --list"
