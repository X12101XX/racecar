# 色卡识别与 NPU / ONNX 后端

检查日期：2026-10-17。本机已验证 NumPy 颜色判断、ONNX Runtime CPU 真推理和 ROS2 闭环。尚未验证昇腾板、CANN/ATC 转换或 CUDA GPU 加速；不再保留旧文档中无板端证据的 NPU 延迟、准确率、CPU 占用率及“零拷贝”数值。

## 当前实际分工

- `fast_color` / `cpu`：CPU NumPy 颜色阈值。默认 `auto` 没有模型时使用它，报告具体原因。
- `onnx`：实际调用 `InferenceSession.run`，报告会话的 `get_providers()`。安装 CPU 版运行库得到的仍是 CPU 推理，不能称为 GPU 加速。CUDA provider 的存在也不代表每个算子均在 GPU 上执行，需在目标机做 profiling。
- `npu`：使用已安装的 Ascend ACLLite 的 `AclLiteResource.init`、`AclLiteModel.execute` 加载并执行 `.om` 色卡分类模型。接口已实现，板端兼容性与性能待实测；不会安装驱动、CANN 或修改全局环境。
- ONNX / NPU 目前只加速**色卡分类**。锥桶和地贴颜色仍由 CPU 处理，障碍坐标来自雷达。所有锥桶均避让，不按颜色放宽碰撞检查。

显式 `--ai-backend onnx` 或 `npu` 缺少模型、依赖或协议不匹配时直接报错，不再悄悄换成 CPU。`auto` 允许回退，原因记录在 `summary.json` 的 `metrics.ai_details` 中。无颜色模式不加载模型；`ai_backend=disabled`。

## 无颜色线上赛与 CPU 色卡模式

在项目根目录运行：

```bash
# 所有物料无颜色，发车前手动选路
bash tools/run_ros_sim.sh --route A
bash tools/run_ros_sim.sh --route B

# 色卡自动选择；仿真默认展示 --route 对应的卡
bash tools/run_ros_sim.sh --use-card --route A --ai-backend fast_color
bash tools/run_ros_sim.sh --use-card --route B --ai-backend fast_color

# 特意让初始路线与色卡相反，验证实际切换：最终应为 B
bash tools/run_ros_sim.sh --route A --use-card --card-color red --ai-backend fast_color

# 手动选路，同时识别物料颜色
bash tools/run_ros_sim.sh --route A --color
```

色卡必须连续两帧同色且达到置信度门槛。空帧或换色会重置；超时会停车失败，不再按预设路线偷偷发车。摄像头帧率决定确认所需时间，不能把推理耗时当成两帧确认耗时。

## 可复现的 ONNX 模型与隔离依赖

仓库附带 `models/card_classifier.onnx`、说明 JSON 和导出源码。它是**确定性的 RGB 阈值计算图**，不是训练好的 CNN 或 YOLO。它用于完成可复现的颜色推理闭环，现场仍须检查光照、曝光、背景和阈值。旧导出工具创建随机初始化 CNN 的行为已移除。

```bash
# 依赖只装入新的 Docker 镜像，不安装到电脑或实车系统
bash tools/prepare_vision.sh

CROSSFIRE_SIM_IMAGE=crossfire-sim:vision-v2 \
  bash tools/run_ros_sim.sh --use-card --route A --ai-backend onnx \
  --ai-model /workspace/models/card_classifier.onnx

# 含真正 ONNX 推理的完整测试；也会重新编译当前容器的 C++ 库
CROSSFIRE_SIM_IMAGE=crossfire-sim:vision-v2 \
  bash tools/test_in_container.sh python3 -m unittest discover -s tests -v

# 多轮 ROS 闭环、实测配置、激进预设和视频完整解码
CROSSFIRE_SIM_IMAGE=crossfire-sim:vision-v2 python3 tools/validate_simulations.py
```

`crossfire-sim:vision-v2` 安装的是 ONNX Runtime CPU 包。当前笔记本 GPU 和昇腾 NPU 均没有纳入已验证结果。

## 模型协议与板端配置

输入仅支持一个 `float32` 张量 `[1,3,224,224]`。预处理：画面中央 70% ROI，最近邻缩放至 224×224，RGB 顺序，除以 255，NCHW。输出一个 `[1,3]` 分数张量，顺序为 `[background, blue, red]`；最大分数对应色卡且分数至少 0.8 才送入两帧确认。

训练模型需要使用完全相同的协议、标签顺序及分数尺度。YOLO 检测输出不符合此协议，不能直接替换模型路径，需另做检测解码、NMS、相机标定与融合适配。

修改 `config/vision.json` 的 `ai` 节点（不要删除已有相机参数）：

```json
"ai": {
  "backend": "onnx",
  "model_path": "models/card_classifier.onnx"
}
```

实车命令支持 `--ai-backend`、`--ai-model` 覆盖；先保持只读：

```bash
bash tools/run_car.sh A --use-card --ai-backend onnx --ai-model models/card_classifier.onnx
```

实车必须已有兼容推理库；部署包不会自动安装。完成 [实车适配与标定清单](06_实车适配与标定清单.md) 后，才可按部署指南启用输出。

昇腾 `.om` 需由目标板匹配的 CANN / ATC 编译，`--soc_version` 必须从实机确认，不能从 8 TOPS 截图猜。示例模板（替换 `实际芯片型号`）：

```bash
atc --model=models/card_classifier.onnx --framework=5 \
    --output=models/card_classifier --input_format=NCHW \
    --input_shape="input:1,3,224,224" --soc_version=实际芯片型号
```

阈值图中的比较、布尔、归约算子是否受目标 CANN 支持，需要实际转换验证；转换失败时保留 CPU 路径。ACLLite 需与 CANN/pyACL 版本配套。当前预处理在 CPU 上执行，不能声称 DVPP 或零拷贝。

接口参考：[ONNX Runtime Python API](https://onnxruntime.ai/docs/api/python/api_summary.html)、[Execution Providers](https://onnxruntime.ai/docs/execution-providers/)、[Ascend 官方 ACLLite 模型实现](https://gitee.com/ascend/ACLLite/blob/master/python/acllite_model.py)。
