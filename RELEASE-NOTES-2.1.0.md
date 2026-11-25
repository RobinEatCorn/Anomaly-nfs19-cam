# NFS 2015 Chase Cam 2.1.0

适用：Anomaly 2.7.0 / 3.0.0 / 3.1.0 / 3.1.1 Runtime，NTE。
本版的实机验证在 **Anomaly 3.1.1** 上完成；3.1.1 与 3.1.0 使用同一份 NTE profile
(`08a3feb9...249a`)，门禁直接 ready。

2.1.0 是一次相机架构迁移，不新增镜头风格。漂移偏航弹簧、pivot 环绕、坡度跟随、
漂移推近/下压、速度 FOV、震动、烧胎特写、停车环绕和 Z 三档循环都保留。

## 1. 相机改到游戏线程的场景视图相位

旧路径在渲染线程改写已经构造好的视图结构。2.1.0 改成：

```text
Game tick
  └ 发布车辆样本、车辆存在性和车头朝向，清本帧门闩

UWorld::Tick 早段
  └ UMG/Slate 世界坐标投影 → GetViewPoint
      拷贝上一场景发布的 pose（此时车辆也是上一帧最终值）

UGameViewportClient::Draw
  └ CalcSceneView
      └ CalcSceneViewInitOptions 进入
          读 VehicleMesh ComponentToWorld（本帧最终）
          AdvancePresenter 一次
          发布 Location / Rotation / FOV / DesiredFOV
      ├ GetProjectionData → GetViewPoint：拷贝本帧 pose
      ├ CalcSceneView 第二次 GetViewPoint：拷贝本帧 pose
      └ HUD PostRender：拷贝本帧 pose
      └ InitOptions 返回后写 bInCameraCut = 原值 || pending_cut
```

关键点：

- 相机和引擎即将绘制的车共享同一个 `ComponentToWorld`，不再有渲染时钟插值或
  `SampleRing` 滞后；
- 早段 UI 用上一帧 pose，场景/HUD 用本帧 pose，和游戏自己的两阶段行为一致；
- 只写调用方的 `FMinimalViewInfo`、`FSceneViewInitOptions::bInCameraCut` 和插件自己的状态；
- 不写渲染线程视图，不写 `CameraCache.POV`，不覆盖 `GetCameraViewPoint` 出参；
- 不调用 `SetGameCameraCutThisFrame()`，避免 UE 5.5.4 的永久 cut 闩锁；
- 车辆变换短暂读不到时最多外推 2 帧 / 150 ms，超时请求 cut 并 fail-open。

## 2. 两个钩子是一个整体

插件现在只安装：

- `ULocalPlayer::GetViewPoint`：写视点；
- `ULocalPlayer::CalcSceneViewInitOptions`：推进模型、写 cut。

任一 pattern 不命中或 hook 创建失败，就释放另一个，整套回到游戏原相机，不留下半条路径。
版本门禁仍按 `profile_hash` fail-closed；不匹配时不安装任何一个钩子。

## 3. 验证结果

### 3.1 E4b 相机-车偏移（300 帧）

采集时车辆在高速行驶，速度范围 50–275 km/h，因此原始偏移方差包含有意的速度-距离/FOV
变化；下表同时给出高频残差和“相邻帧偏移变化 ÷ 车辆位移”，后者才用于判断一帧滞后/抖动。

```text
n = 300
speed_kmh       mean 155.5   min 50.0   max 275.4
dt_ms           mean 24.76   sd 6.16
step_cm         mean 106.97

               mean       sd      hp_sd    dmean   dp95   dmax   ratio_mean  ratio_p95  ratio_max
along_cm     -724.83    68.76     6.12     2.97   8.81  40.09      0.035      0.120      0.320
lateral_cm    -14.50    49.76     8.55     3.21   8.74  81.32      0.038      0.122      0.650
vertical_cm   172.21    29.19     3.57     1.28   8.22  13.34      0.012      0.063      0.119
```

- 高频残差 `hp_sd` 分别为 **6.12 / 8.55 / 3.57 cm**；
- 相邻帧偏移变化只有车辆位移的 **3.5% / 3.8% / 1.2%**（均值），95% 分位约 **12%**；
- 原始偏移 lag1 为 **0.994 / 0.989**，是慢速趋势而不是高频振荡。

### 3.2 车辆变换读取失败注入

自动强制 `ReadVehicleComponentTransform` 失败：

| 连续失败帧 | 外推帧 | 超时 | cut | pose 作废 |
|---:|---:|---:|---:|---:|
| 1 | 1 | 0 | 0 | 0 |
| 2 | 2 | 0 | 0 | 0 |
| 4 | 2 | 1 | 1 | 1 |

1/2 帧内继续沿已知速度外推；第 3 帧超过上限后请求 cut、清空历史、作废旧 pose，后续帧
fail-open 到游戏相机。测试结束后相机自动恢复追车视角。

### 3.3 fail-open

把 `ULocalPlayer::GetViewPoint` 的 pattern 临时改成不存在的字节序列后：

- 日志输出 `GetViewPoint pattern not found; game-thread camera disabled for this session`；
- `CalcSceneViewInitOptions` 钩子也没有安装（两个钩子整体释放）；
- 用户实机确认画面是**游戏原版第三人称相机**，不是 NFS 追车效果；
- 恢复 pattern 后重新加载，两个钩子正常安装。

### 3.4 实机矩阵

| 项目 | 结果 | 备注 |
|---|---|---|
| Anomaly 3.1.1 加载、profile 门禁 | PASS | `08a3feb9...249a` |
| 直行、加速、转弯，车辆不抖 | PASS | 3.1.1 实机 |
| Z 循环、上下车、重置、切预设、传送 | PASS | 3.1.1 实机 |
| 拍照模式、过场不接管 | PASS | 3.1.1 实机 |
| HUD 世界标记（相机距离变化） | PASS | 用户目视未发现错位 |
| 停车环绕 / 漂移 | PASS | 3.1.1 最终候选包实机 |
| 帧生成开 / 关 | BLOCKED | 本机 RTX 3080 不支持帧生成；只验证了帧生成关闭 |
| 10 分钟 FSR3 连续驾驶 | BLOCKED | 同上 |

> 帧生成开/关的原始验收目标没有在本机完成，发布说明不能把它写成 PASS。相机与车共享
> 同一 `ComponentToWorld`、cut 逐帧写入是这次迁移的直接目标；有 AMD 帧生成环境的用户
> 仍应实机复测这一项。

## 4. 升级与兼容

- 插件 ID、设置目录不变；设置 schema 仍是版本 1。
- 2.1.0 删除 `anchorSource` / `borrowStrength` / `borrowFallback` / `diagnosticZeroLag`
  四个旧高级键；旧配置中的这些键会被忽略，其余 8 组设置和窗口状态保留。
- 与 Camera Tools / BetterPose 仍互斥：2.1.0 占用 `ULocalPlayer::GetViewPoint`，同一时间
  只能有一个相机插件。
- 只支持 Windows x64 与 Anomaly Plugin API v1；不改 Anomaly 本身，也不写游戏目录文件。

## 5. 产物

```text
dist/Nfs2015ChaseCam-2.1.0.zip
  manifest.json
  plugin.dll
  locales/zh-CN.json
```

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `plugin.dll` | 6,615,384 | `6ee0cc0f14aedef1b06ed953003e9f9a6e99e99aa713bf7ab7292289ea232577` |
| `Nfs2015ChaseCam-2.1.0.zip` | 1,978,794 | `bd6850a69c769edac212d2440a64060e880b7097feabdf7c62f1e82bfc22966a` |

最终候选包在 Anomaly 3.1.1 上重载后，用户确认驾驶、漂移、停车环绕、HUD 隐藏、
上下车、Z 循环、重置、预设、传送、拍照和过场均正常。完整的离线门禁与宿主工具结果见
`RELEASE-CHECKLIST-2.1.0.md`。
