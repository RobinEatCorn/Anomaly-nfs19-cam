# 烧胎特写 → 起步回正：NFS 2015 数据与手感修正

- 日期：2026-11-10
- 状态：**PASS（未发布）**
- 范围：`BurnoutShot` 速度退出、`burnoutLaunchExitTime` 设置、NFS 2015 prefab 对照、实机截图
- 产物：`burnout_shot.hpp`、`plugin.cpp`、`tests/burnout_shot_test.cpp`、`locales/zh-CN.json`、本报告

## 1. 问题

烧胎特写结束后，如果直接松手刹起步，镜头回正太快，像从轮边“切”回追车视角。
最初把速度退出的快回从 0.5 s 调成 1.8 s，手感明显变好；随后按要求去 NFS 2015
资产里查真实过渡，发现 `WheelSpin` 的退出过渡其实是 **0.2 s 延迟 + 2.5 s**。
按 2.5 s 实装后，在 NTE 里又显得太慢，因此最终采用：

- **保留 NFS 的 0.2 s 延迟和 `EULER_ORBIT` 结构**；
- **起步回正时长回到实测更好的 1.8 s**；
- 1.8 s 只用于正常起步，不用于异常状态。

## 2. NFS 2015 证据

文件：

```text
nfs2015-game-files/extracted/race/cameras/chasecameraprefab.txt
```

模式：

| 模式 | 实例 | DefaultTransition | OutTransition |
|---|---:|---:|---:|
| `LeftWheelSpin` | `Instance104` | `Instance136` | `Instance138` |
| `RightWheelSpin` | `Instance107` | `Instance142` | `Instance139` |
| `WheelSpin` | `Instance108` | `Instance137` | `Instance140` |

进入过渡（`Instance136/137/142`）：

```text
TimeToDelay 0.5
Time 1.0
EaseInControl 0.6
EaseOutControl 0.4
Blend CINEBOT_BLEND_EULER_ORBIT
```

退出过渡（`Instance138/139/140`）：

```text
TimeToDelay 0.2
Time 2.5
EaseInControl 0.6
EaseOutControl 0.4
Blend CINEBOT_BLEND_EULER_ORBIT
```

结论：

1. NFS 2015 **没有**为起步单独准备 0.5 s 快回；左右轮和通用 `WheelSpin`
   共用同一组 2.5 s 退出过渡。
2. `LowSpeed` 的 `DefaultTransition Instance141` 是 0.0 s 延迟 + 2.0 s
   `CUBIC_ORBIT`，不是 WheelSpin 的退出过渡。
3. NFS 的 2.5 s 从 `WheelSpin` 模式退出时开始计时。NTE 这边的速度门在车已经
   起步、超过 `burnoutHoldSpeed` 后才触发；直接照搬 2.5 s 会把“模式退出前”
   的时间也算进去，实机总时长接近 3 s，所以看起来太慢。

### 2.1 镜头倾斜 / roll 回正

资产里**没有**单独的“倾斜回正”动画或曲线。具体看：

| 项 | NFS 2015 数据 | 说明 |
|---|---|---|
| `LeftWheelSpin` / `RightWheelSpin` / `WheelSpin` | `Roll 0.0` | 特写模式本身不设基础 roll |
| `FixedTargetComposer` `Instance6/7/8` | `Up (0,1,0)` | 用固定 up 构图，不主动做 Dutch |
| WheelSpin 的控制器 | `BasicController` + target noise + 相机输出 | 没有 `CINEBOT_REACTION_OUTPUT_DUTCH` |
| 普通 Chase `Instance105` | `DUTCH`（`Instance40`）、`NOISE_ROLL`（`Instance16`）等 | 正常追车才挂 roll/侧倾控制器 |
| 退出过渡 `Instance140` | `EULER_ORBIT`，0.2 s 延迟 + 2.5 s | 位置和朝向一起混合 |

因此：

- 特写镜头本身是低机位、固定 up 的镜头；画面里明显的“倾斜”主要来自低机位
  的 pitch / 透视，而不是一个持续旋转的 Dutch roll。
- 退出时没有第二条“roll 弹簧”或“先回正再拉远”的时间线。`EULER_ORBIT`
  在同一个 2.5 s 过渡里同时插值位置和朝向，所以低机位 pitch、横向 offset
  以及（如果端点有差异的）roll 都会随这条过渡一起渐进回正。
- 如果 WheelSpin 通过 `InheritControllers` 继承了 Chase 的 DUTCH/roll 控制器，
  roll 也仍由 Chase 的弹簧体系处理，而不是 WheelSpin 独有的回正段。

当前实现对应关系：

- `BuildShotView()` 的 `view.roll = 0.0`，特写不做额外 Dutch；
- `BlendBurnoutCamera()` 对 up 向量和标量 roll 都按 `blend` 插值，并让机位
  沿 pivot 做圆柱轨道运动；
- 因此起步时看到的是“低机位 pitch + 位置逐步抬起”，而不是一条独立的
  roll 回正曲线；这与 prefab 里只有 `EULER_ORBIT` 的结构一致。

## 3. 实现

### 3.1 `BurnoutShotConfig`

```cpp
double exit_delay{0.2};
double exit_time{2.5};
double launch_exit_time{1.8};
double fast_exit_time{0.5};
```

`launch_exit_time` 是正常起步（速度超过保持阈值）使用的回正时长；
`exit_time` 仍用于普通条件消失；`fast_exit_time` 只留给异常状态。

### 3.2 退出判定

```cpp
const bool speed_exit = std::abs(speed) >= hold_speed;
const bool unsafe_exit = !on_ground || !direction_ok;
const bool force_exit = unsafe_exit || speed_exit;
```

- `speed_exit`：使用 `exit_delay`（NFS 的 0.2 s）和 `launch_exit_time`（1.8 s）；
- 普通条件消失：使用 `exit_delay` + `exit_time`；
- `unsafe_exit`（腾空/倒车）：0 延迟 + `fast_exit_time`（0.5 s），避免异常状态
  拖住镜头。

这样不会再出现“速度一过 3 m/s 就从 2.5 s 过渡突然切成 0.5 s”的硬切；
同时正常起步不会像完整 2.5 s 那样拖沓。

### 3.3 设置与 UI

新增设置字段：

```json
"burnoutLaunchExitTime": 1.8
```

- 范围：0.1–5.0 s；
- 默认：1.8 s；
- 中文 UI：`起步回正时间（秒）`；
- 已接入 schema、`Context`、`DisplayState`、校验、持久化、加载、恢复默认和滑杆。

schema 当前为 42 个属性。

## 4. 验证

### 构建与单测

```text
./build-mingw.sh --sdk ../Download/Anomaly-2.7.0-sdk --run-tests
ctest --test-dir build.mingw --output-on-failure

1/2 nfs2015_chase_cam_model ... Passed
2/2 nfs2015_burnout_shot_model ... Passed
100% tests passed, 0 tests failed out of 2
```

单测覆盖：

- 起步 0.4 s 后仍保留大部分特写（`phase > 0.8`）；
- 1.8 s 后回到 chase（`phase == 0`）；
- 倒车/异常状态仍走 0.5 s 安全退出；
- 原有进入延迟、退出延迟、重触发、侧别锁定、FOV 和有限值测试保持通过。

### 实机截图

安装当前构建后运行 `capture/run_launch.ps1`（Shift+W 2.3 s，松手刹、保持油门）：

| 时间 | 画面 |
|---:|---|
| 0.00 s | 烧胎特写，轮边近景 |
| 0.25–1.40 s | 车身/轮边近景，镜头仍保持 |
| 1.90 s | 仍能看到轮边和车身下部，尚未切回追车 |
| 2.50 s | 已回到车尾追车视角 |
| 3.20 s | 正常追车视角 |

本次截图确认：**没有在 3 m/s 处硬切**；回正从 1.9 s 左右开始明显收尾，
2.5 s 前完成。测试车在第二组截图时已靠近路肩，最后碰到路边杆件，但
0.25–2.5 s 的镜头过渡仍完整可读。

当前配置没有持久化 `burnoutLaunchExitTime`，因此使用默认 1.8 s；
如果用户已有旧配置，也会按默认值加载，不需要迁移。

## 5. 与 NFS 的差异

| 项目 | NFS 2015 | 当前实现 |
|---|---|---|
| 退出延迟 | 0.2 s | 0.2 s（正常/起步） |
| 退出时长 | 2.5 s | 普通 2.5 s；起步 1.8 s |
| 混合 | `EULER_ORBIT` | 现有轨道混合（位置 + up/roll 插值） |
| 特写 roll | 0.0，无专用 roll 控制器 | 0.0；roll/up 随轨道过渡 |
| 缓动 | ease in 0.6 / out 0.4 | `SmoothStep` |
| 异常退出 | 未单独建模 | 腾空/倒车 0.5 s 安全退出 |

1.8 s 是针对 NTE 的速度门触发时机做的感知修正，不是 NFS 原始数值；
需要完全按 prefab 时可以再把滑杆调回 2.5 s。

## 6. 结论

起步回正现在保留 NFS 的 0.2 s 延迟和轨道退出结构，去掉了 0.5 s 硬切；
时长采用实测更好的 1.8 s，并暴露为可调设置。构建、CTest 和实机截图均通过。
