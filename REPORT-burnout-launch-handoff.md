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
| 混合 | `EULER_ORBIT` | 现有轨道混合（等价目标） |
| 缓动 | ease in 0.6 / out 0.4 | `SmoothStep` |
| 异常退出 | 未单独建模 | 腾空/倒车 0.5 s 安全退出 |

1.8 s 是针对 NTE 的速度门触发时机做的感知修正，不是 NFS 原始数值；
需要完全按 prefab 时可以再把滑杆调回 2.5 s。

## 6. 结论

起步回正现在保留 NFS 的 0.2 s 延迟和轨道退出结构，去掉了 0.5 s 硬切；
时长采用实测更好的 1.8 s，并暴露为可调设置。构建、CTest 和实机截图均通过。
