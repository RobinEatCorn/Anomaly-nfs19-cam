# Phase 3 报告：presenter / route B 集成

- 日期：2026-11-10
- 状态：**PASS**
- 范围：在 render clock 上推进 `BurnoutShot`，把 chase / shot 的混合 view 同时用于 route B、presenter pose 和 lens
- 产物：`plugin.cpp` 集成、`burnout_shot.hpp` 实机机位调整、实机截图验证

## 1. 集成位置

`AdvancePresenter()` 原来直接：

```cpp
const CameraState view = context.chase.Update(elapsed, sample.car);
PublishRouteBModelPose(context, sample, view, now_ns);
```

现在：

```cpp
const CameraState chase_view =
    context.chase.Update(elapsed, sample.car);

BurnoutShotConfig config = context.burnout_shot.config();
config.slope_follow = context.slope_follow.load(...);
config.camera_scale = sample.camera_scale;
context.burnout_shot.SetConfig(config);

const BurnoutShotResult burnout =
    context.burnout_shot.Update(elapsed, sample.car, sample.burnout,
                                sample.aspect_ratio);
CameraState view = BlendBurnoutCamera(
    chase_view, burnout.view, sample.car, burnout.blend);

PublishRouteBModelPose(context, sample, view, now_ns);
```

关键的“同一个 view”约束已经满足：

- `PublishRouteBModelPose()` 使用混合后的 `view`；
- `location` / `rotation` 使用混合后的 `view`；
- `lens` 使用混合后的 `view.HorizontalFovDegrees()`（`fovEnabled=false` 时仍为 0）；
- `PublishPresenterPose()` 使用同一组 `location/rotation/lens`；
- 车体 scale / origin offset 仍按原有路径应用；
- `presenter_drift` 仍保留 chase 的 drift 值（诊断用）。

`sample.burnout` 来自 Phase 2 的 Game-tick 检测；`sample.car` 是已经过 anchor / borrow 修正的同一份车体状态，因此 shot 的后轮不会和渲染车体分离。

## 2. 重置与安全门

在以下位置调用 `context.burnout_shot.Reset()`：

- `!sample.driving`：离开载具/车辆消失；
- presenter 初始化或 `presenter_epoch != sample.epoch`：世界切换、传送、preset 变化、`RequestPresenterReset`；
- `ResetMotion()` 内部已经通过 `RequestPresenterReset()` 触发 epoch 变化，所以车辆 identity 变化也会重置。

另外：

- `context.presenter_burnout_blend` 在 reset 时清零；
- `context.presenter_burnout_side` 重置为默认右侧 `+1`；
- `presenter_burnout_fallbacks` 在 shot blend > 0 且 route B 取不到 pose 时累加，供 Phase 4 诊断面板使用；
- `BurnoutShot::SetConfig()` 每帧只更新配置，不重置正在进行的 shot，所以调整 FOV/shake 不会硬切。

## 3. 实机机位调整

Phase 1 的 NFS 参考机位是：

```text
eye    = (-2.2, side*1.9, 0.40)
target = (-0.9, side*0.75, 0.10)
```

第一次实机截图显示：目标点落在车体内侧，镜头视线穿过翼子板，画面被车身填满，后轮只出现在边缘。原因是 NTE 的车比 NFS 参考车更宽，且参考 target 不是在轮面外侧。

Phase 3 调整为：

```text
eye    = (-2.8, side*2.0, 0.55)
target = (-1.45, side*1.0, 0.35)
```

- target 改到后轮外侧面，避免翼子板遮挡；
- eye 后移、外移、略抬高，让镜头看到后轮 + 后翼子板 + 尾灯；
- 仍保持水平 FOV 45°、entry 0.5+1.0 s、exit 0.2+2.5 s、shake 参数不变。

实机截图（见 `capture/`，未提交）：

| 文件 | 时刻 | 画面 |
|---|---|---|
| `burnout-0.8.png` | 按下后 0.8 s | 仍是 chase，entry delay 尚未结束 |
| `burnout-2.6.png` | 2.6 s | 已进入特写；轮胎烟很浓，遮住轮面 |
| `burnout-4.1.png` | 4.1 s | 特写保持；烟雾、后轮、翼子板 |
| `burnout-exit-1.png` | 松开后 1 s | 烟雾散去，后轮/尾灯/翼子板构图清晰 |
| `burnout-exit-3.5.png` | 松开后 3.5 s | 已回到 chase |

`burnout-exit-1.png` 是最能代表目标构图的帧：后轮位于右侧，后翼子板和尾灯入镜，没有穿模。燃烧时的浓烟是 NTE 的烟雾效果，属于预期画面；贴墙停车时烟雾会更集中。

## 4. 输入验证

本轮使用高完整性 PowerShell + `PostMessage` 成功触发：

```text
WM_ACTIVATE(WA_ACTIVE)
WM_KEYDOWN(VK_LSHIFT)
WM_KEYDOWN('W')
等待 2.5 s
WM_KEYUP('W')
WM_KEYUP(VK_LSHIFT)
```

关键点：

- 目标窗口是 `UnrealWindow` 顶层窗口；
- `lParam = (scan << 16) | 1`，keyup 再置 bit30/31；
- 必须先 `SetForegroundWindow` 或发送 `WM_ACTIVATE`；
- 该路径不受 UIPI 的 `SendInput` 限制，但需要管理员权限。

日志确认：

```text
burnout sample active=1 direct=0 fallback=1 ... throttle=1.00 HB=1
burnout sample active=1 direct=1 fallback=1 ... wheels=0c
burnout active ... angular=.../1718.9/1718.9 speed=0.00
```

随后松开按键，shot 按 0.2 s exit delay + 2.5 s exit time 回到 chase。

## 5. 验证

构建：

```text
./build-mingw.sh --sdk ../Download/Anomaly-2.7.0-sdk --run-tests
plugin.dll 3.1M，PE32+ x86-64
```

单测：

```text
ctest --test-dir build.mingw --output-on-failure
1/2 nfs2015_chase_cam_model ... Passed
2/2 nfs2015_burnout_shot_model ... Passed
100% tests passed, 0 tests failed out of 2
```

实机：

- 安装后 plugin_generation 16；
- `route B early ... fallbacks=0`，shot 激活期间 route B 未丢 pose；
- 截图确认 chase → 特写 → chase 的完整进出；
- 未出现崩溃、TAA 冻结或相机卡死。

## 6. 已知限制

- 没有 wall raycast：贴墙停车时，后侧机位仍可能进入墙体或烟雾体积；当前只做了 eye Z 下限和最小半径保护。
- NTE 的轮胎烟是较不透明的体积/网格效果，镜头贴近后轮时会被烟遮住；这是画面风格问题，不是相机状态错误。
- `CarState.on_ground` 目前由 `UpdateCamera` 固定为 `true`，所以“腾空立即退出”还没有真实 ground 信号；目前依赖 `!driving` 和速度门。
- `presenter_burnout_blend/side/fallbacks` 已经写入 Context，但 Phase 4 才放到诊断面板。
- 没有 wall / vehicle bounds 的 collision query；这是后续可选增强。

## 7. 结论

Phase 3 **PASS**。chase 与 shot 已在同一 render clock 上混合，route B / presenter pose / lens 使用同一 view；实机可看到后轮特写并平滑退出。下一阶段补设置、UI、本地化和诊断收尾。
