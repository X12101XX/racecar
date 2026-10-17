#!/usr/bin/env bash
# Optional ONNX dependencies and model export are isolated in Docker.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
docker build -f containers/Dockerfile.vision -t crossfire-sim:vision-v2 .
mkdir -p output/vision-models models
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
 --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,nosuid,size=64m \
 --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
 --mount "type=bind,src=$ROOT/output/vision-models,dst=/models" \
 -w /workspace crossfire-sim:vision-v2 python3 tools/export_model.py --output /models/card_classifier.onnx
cp output/vision-models/card_classifier.onnx models/card_classifier.onnx
cp output/vision-models/card_classifier.json models/card_classifier.json
echo 'ONNX 验证：CROSSFIRE_SIM_IMAGE=crossfire-sim:vision-v2 bash tools/run_ros_sim.sh --use-card --ai-backend onnx --ai-model /workspace/models/card_classifier.onnx'
