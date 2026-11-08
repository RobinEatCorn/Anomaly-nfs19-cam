# NFS 2015 Chase Cam 2.0.1

适用：Anomaly 2.7.0+ Runtime，NTE。

## 主要修复

- 将 route B 的车辆锚点改为 **VehicleMesh ComponentToWorld**。同帧探针确认它与最终
  render proxy 的误差为 `0.00 cm`；旧的包围盒中心/渲染快照会落后约一帧，造成车抖。
- 相机仍然只在 `PrepareViewStateForVisibility` 之前写入一次。删除 transform 之后和
  `SetupUniformBufferParameters` 的相机写入，避免裁剪、运动矢量和 TAA 历史使用旧机位。
- 早段相机使用 NFS presenter pose，并把 X/Y/Z **全部**对齐到当前帧 VehicleMesh：
  - 不再只对齐水平轴，修复上下坡时车辆相对镜头上下抖动；
  - 保留 NFS 2015 的弹簧、阻尼、FOV 和车身姿态；
  - 车辆变换与画车使用同一帧数据，不做速度/位移预测或补偿。
- 修复“车身原点偏移”滑块：完整锚定到 VehicleMesh 会抵消 presenter 中已有的偏移，
  现在按默认值施加显式垂直 trim，滑块重新生效。
- 仅在驾驶相机活跃时接管视角；下车后立即停止 route B，原版人物相机恢复，不再固定在
  最后一帧车内机位。
- `FSceneView::UpdateViewMatrix` 的 pattern 对 2.7 的 RIP 位移使用通配；失败时仍保持
  fail-closed，不会写入未知 view。

## 验证

用户实机确认：

- 平路和坡道无闪屏、无沙沙/透明墙；
- 车辆无抖动，上下坡无上下抖动；
- “车身原点偏移”滑块生效；
- 下车后镜头恢复跟随人物，再上车可正常切回追车相机。

构建和发布包校验：

```sh
./build-mingw.sh --sdk <Anomaly-2.7.0-sdk> --zip
./verify-release.sh --allow-missing-host-tools
```

发布包根目录：

```text
manifest.json
plugin.dll
locales/zh-CN.json
```
