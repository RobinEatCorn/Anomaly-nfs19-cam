# NFS 2015 停车环绕：构建、回归与实机验收清单

> 对应计划：`PLAN-nfs2015-parking-orbit-cam.md` 任务组 6；
> `PLAN-nfs2015-orbit-handheld-shake.md` 任务组 10。
> 本文的验收表来自计划 §6.2 与手持抖动补充；构建结果来自本机 WSL2 + MinGW-w64
> 交叉编译。
> 代码与单元测试已完成，**实机目视项仍需按下面的表逐条确认**。

## 1. 交付状态

- 停车环绕已接入 presenter，默认开启。
- 设置项：`orbitEnabled / orbitHideUi / orbitEntryDelay / orbitPeriod /
  orbitRadius / orbitHeight / orbitWeave / orbitFov / orbitShakeStrength /
  orbitShakeFrequency`。
- 诊断面板：`Parked orbit` 行显示 `state / blend / hold / clock / phase0 /
  input / speed / shake`；`Orbit UI hide` 行显示 HUD 隐藏状态、读回和请求计数；
  状态切换打一条 INFO 日志。
- 旧设置文件缺少 orbit 字段时回退默认值，不需要重置配置；已有的 `orbitFov`
  会保留旧值（旧默认 43°），需要 45° 时请手动修改或恢复默认。
- 本次未发版，未写 `RELEASE-NOTES-<version>.md`，未 bump 版本号。

## 2. 构建与回归结果

执行：

```sh
cd nfs2015-chase-cam
./build-mingw.sh --fresh --run-tests
ctest --test-dir build.mingw --output-on-failure
```

结果：

| 项目 | 结果 |
|---|---|
| fresh 配置 + 编译 | 通过 |
| `nfs2015_chase_cam_test` | 通过 |
| `nfs2015_burnout_shot_test` | 通过 |
| `nfs2015_orbit_shot_test` | 通过 |
| `nfs2015_orbit_handheld_test` | 通过 |
| `nfs2015_wheel_steer_test` | 通过 |
| `nfs2015_orbit_ui_hide_test` | 通过 |
| `ctest` | **6/6** 通过 |
| `plugin.dll` 导入表 | 仅 `KERNEL32.dll`、`msvcrt.dll` |
| 导出 | `AnomalyPluginEntryV1` |

`--run-tests` 现在会依次运行 chase / burnout / orbit / handheld / wheel steer /
orbit UI hide 六个夹具，不再只跑 chase。

## 3. 实机验收清单（计划 §6.2 + 手持抖动补充）

| 检查项 | 期望 | 不对时调什么 |
|---|---|---|
| 进入时机 | 停稳且松手约 8 s 开始动，约 10 s 完全进入 | `进入延迟` |
| 一圈时长 | 从完全进入算约 60 s 回到同一角度 | `环绕周期` |
| 车的画面占比 | 与原版 `contact_open.png` 相近；FOV 45 后比旧默认 43 略小约 3% | `环绕半径` / `视野` |
| 环绕高度 | 默认约 1.83 m；调高后俯视更多，调低更接近平视；默认机位到注视点仍为 5.05 m | `环绕高度` |
| 车身在画面里的高度 | 车顶略高于画面中心、能看到一点路面 | 先调 `环绕高度`；仍不对 → 按 §3 汇报 |
| 地平线倾斜 | ≈ 0（相机的 up 是世界 +Z，roll=0） | 若有倾斜 → 按 §3 汇报 |
| 15 s / 45 s | 横向摆动最明显，且**不忽远忽近** | `横向摆动`；若忽远忽近说明 weave 加错轴 |
| 30 s | 车尾正对镜头（继承进场相机方向） | 方向/相位问题 → 按 §3 汇报 |
| 手持感 | 静止画面有约 0.5° 的缓慢竖直游移，平滑、不是高频噪点 | `手持抖动强度` / `手持抖动频率` |
| 抖动方向 | 主要是上下，横向很小（约 1:4.5） | 方向不对 → 按 §3 汇报 |
| 抖动不是整机平移 | 抖动时机位不移动，只有注视点偏移；机位到车的距离不变 | 按 §3 汇报 |
| 入场/出场 | 抖动随混合淡入淡出，无明显跳变 | 按 §3 汇报 |
| 手感太尖/太抖 | 先降强度（×0.5） | `手持抖动强度` |
| 手感太飘 | 降频率到 3–4 Hz | `手持抖动频率` |
| 起步 | 车速一上来 1 s 内平滑回追车相机，不甩 | 按 §3 汇报 |
| 原地不动只按油门/转向 | 不进入环绕；进入中则退出 | 说明输入信号无效 → 按 §3 汇报 |
| 烧胎（Shift+W） | 停着烧胎走特写，不走环绕 | 按 §3 汇报 |
| Z 切第一/第三人称 | 环绕不生效 | 按 §3 汇报 |
| 环绕 HUD 隐藏 | `Enter`/`Orbit` 隐藏；`Exit` 过渡开始恢复 | `orbitHideUi` |
| 菜单热键 | F1/F2/F3/F4/F5/B/C/ESC 第一次按即可打开 | 按 §3 汇报 |
| 第一人称/过场/拍照/下车 | HUD 恢复，无残留隐藏 | 按 §3 汇报 |

建议按下面的顺序执行，便于定位问题：

1. 找一块空旷、平坦、没有贴墙的地方停车。
2. 松开油门、刹车、转向和手刹，等待 8 s；观察进入过程是否约 2 s。
3. 完整看一圈（60 s），重点看 15 / 30 / 45 s。
4. 在环绕中轻踩油门或转向，确认 1 s 内回到追车。
5. 停稳后按 `Shift + W` 触发烧胎特写，确认特写优先于环绕。
6. 用 `Z` 切到第一/第三人称，确认环绕不生效；切回追车并重新停稳。
7. 重启游戏，确认十个设置项的值保留；用旧配置文件启动一次，确认不报错。

## 4. 设置项与构图调整

十个设置项都在设置窗口的「停车环绕」小节，修改即时生效并持久化。

| 字段 | 默认 | 范围 | 调整效果 |
|---|---:|---:|---|
| `orbitEnabled` | `true` | 开/关 | 总开关；关闭时输出与未启用该功能时一致 |
| `orbitHideUi` | `true` | 开/关 | 环绕进入/保持时隐藏 HUD；关闭后环绕继续但 HUD 保持显示 |
| `orbitEntryDelay` | `8.0` | 0–60 s | 停稳并松开全部输入后，等多久开始进入 |
| `orbitPeriod` | `60.0` | 10–600 s | 一圈的时间；越小转得越快 |
| `orbitRadius` | `5.05` | 2–12 m | 环绕半径；到注视点的 3D 距离；越小车越大 |
| `orbitHeight` | `1.83` | 0.3–3.0 m | 相机相对车原点的高度；升高时水平半径缩短 |
| `orbitWeave` | `1.0` | 0–3 m | 切向横向摆动幅度；0 是干净的正圆 |
| `orbitFov` | `45.0` | 20–70° | 垂直 FOV；16:9 下约 72° 水平 |
| `orbitShakeStrength` | `1.0` | 0–2 | 手持注视点抖动强度；0 关闭 |
| `orbitShakeFrequency` | `5.0` | 2–8 Hz | 手持抖动频率；越高越颤 |

构图建议：

- **车太小、环境太多**：先减小 `orbitRadius`，或减小 `orbitFov`（更长焦）。
- **车太大、构图太挤**：增大 `orbitRadius`，或增大 `orbitFov`（更广）。
- **想调机位高度**：调 `orbitHeight`；它保持到注视点的 5.05 m 3D 半径，升高会缩短水平半径。
- **想要完全稳定的轨道**：把 `orbitWeave` 和 `orbitShakeStrength` 都设为 `0`。
- **想更强/更弱的手持感**：调整 `orbitShakeStrength`；频率在 2–8 Hz 内调。
- **想更快看到环绕**：减小 `orbitEntryDelay`；进入过渡仍有固定的 2 s。
- **想慢慢看**：增大 `orbitPeriod`，或把 `orbitFov` 调小获得更紧的视角。
- 默认值针对 `contact_open.png` 的车身占比；先只动半径/FOV/高度，不要同时大改两个以上旋钮。

进入条件不是“停车”本身，而是：

```text
水平速度 <= 0.14 m/s 且 油门/刹车踏板/转向/手刹都松开
```

所以拉了手刹、脚还放在刹车踏板上，或者方向盘有输入时，8 s 计时会重新开始。

手持抖动只改变注视点，不移动相机机位；因此抖动本身不会改变机位到车的距离。
默认强度 1.0 时，注视点在局部右/上方向分别偏移约 ±0.011 m / ±0.05 m，频率
5 Hz；幅度随进入/退出混合淡入淡出。静止看约是 0.5° 的缓慢游移，不是高频噪点。
`orbitShakeStrength` 为 0 时完全关闭，`orbitShakeFrequency` 可在 2–8 Hz 调整。
调整顺序是先强度、后频率：太尖/太抖先把强度减半；太飘把频率降到 3–4 Hz。
方向或机位不对不是这两个旋钮能修的，按 §3 汇报。
烧胎特写的震动是另一套效果，作为外层镜头不会叠加到环绕上。

## 5. 已知差异与限制

- **旧配置的 `orbitFov`**：已有设置文件里的 43° 会原样保留；新的 45° 默认只对
  缺键配置生效。需要 45° 时在设置里改成 45，或恢复默认设置。
- **30 s 相位**：`phase0` 从进入时的追车相机反推，因此当前实现在入场时位于车后，
  30 s 时位于车头。这与计划 §6.2 的“30 s 车尾正对镜头”相反，属于已知的相位约定
  差异；不是设置项能修的。实机确认后如果仍希望照参考图，需要规划决定是否把
  `phase0` 加 π（会改 orbit 单测的 phase0 断言）。
- `orbitPeriod` 只改变角速度；`orbitWeave` 仍按固定的 60 s 轨道运行。把周期改到
  60 s 以外时，weave 不会随周期一起缩放。
- 不做镜头碰撞 / 射线检测，贴墙或贴地时可能穿模。
- 坡度跟随开启时，相机随车身 pitch 绕车；上坡侧不保证不会贴地。
- 诊断面板的 `input` 是 `BurnoutSample.input_active`，刹车取自
  `GetInputInfo` 的 `BrakeInput`（float @8），不是 `GetVehicleBrake`
  （后者在停车松油门时读 1.0，不能当踏板）。
- 环绕中触发烧胎特写时，特写是外层镜头；特写结束后如果车仍停着，需要重新走
  8 s 才再次环绕。
- HUD 隐藏只作用于游戏的 HUD 层，不会隐藏菜单层或 Anomaly 自己的窗口。插件
  Stop 在 Anomaly 的 Lifecycle 线程执行，不能直接调用 Game 线程的
  `ProcessEvent`；停止时的恢复通过下一次游戏 tick 或下次插件启动的 recovery
  marker 完成。详细边界见 `REPORT-orbit-hide-ui.md`。

## 6. 实机失败时请提供

如果某一项不符合预期，按计划 §3 汇报，至少附上：

1. 出问题的检查项和发生时间；
2. 设置窗口里 `Parked orbit` 的整行内容（`state / blend / hold / clock /
   phase0 / input / speed / shake`）；
3. 对应时间段的插件日志，尤其是 `nfs2015 chase cam: orbit state=...` 那条；
4. 是否同时触发了烧胎特写、Z 视角切换或进出载具。

有了 `state`、`hold`、`input`、`speed` 以及 `shake`，通常能直接区分是输入门、
速度门、计时器、相位还是抖动参数问题。
