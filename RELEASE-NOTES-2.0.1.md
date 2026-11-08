# NFS 2015 Chase Cam 2.0.1

## 根因

2.0.0 的 route B 有三个相机写入点：pre-visibility、`OnTransformChanged` 之后和
`SetupUniformBufferParameters`。后两个可能晚于 `PrepareViewStateForVisibility`，使裁剪、
运动矢量和 TAA 历史描述旧机位，而最终画面使用新机位；较大偏差表现为一帧整屏重影/闪屏，
较小偏差表现为箭头、广告牌、墙面单帧消失。

2.7 实机还暴露了两个版本/时序问题：

- `FSceneView::UpdateViewMatrix` pattern 把 RIP 相对位移写死，2.7 位移变化后早段 hook
  无法安装，route B 被 fail-closed 关闭；
- 早段若使用 transform 通知快照，实测快照年龄可达 23.56 ms（约一帧半），相机会骑在
  上一帧车辆变换上，重新出现车抖。

## 修复

- 只保留 `PrepareViewStateForVisibility` 之前一个相机写入点；删除 transform 后和晚段相机
  写入，删除手写 `PrevFrameViewInfo`。
- 用引擎 `FViewMatrices` 构造函数整体重建矩阵，刷新 `InvProjection`、`ProjectionNoAA`、
  `ScreenToClip` 等派生矩阵。
- `UpdateViewMatrix` pattern 对 RIP 位移使用通配，2.7 可正常安装早段 hook。
- 早段优先直接读 Game-tick 验证过的车辆 proxy；transform 快照只做兜底/距离 sanity，
  不再用落后一帧的 snapshot 决定相机位置。
- 读取撕裂时短暂保持上一帧 route B 机位（`held`），不把单帧交还游戏相机。
- 没有任何速度预测或补偿。

## 验收

日志应出现：

```text
pre-visibility route B hook installed
route B early=N late=0
```

且 `early_age` 应为 0（direct 路径）。面板 `路线B` 的 `early` 应持续增长，`fallback` 不增长；
目标是无闪屏、无车抖。
