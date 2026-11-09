# Phase 0 报告：NTE 烧胎信号确认

- 日期：2026-11-10
- 状态：**PASS**，可以进入 Phase 1
- 目标：确认 NTE 的 burnout / 手刹 / 车轮信号，给 `nfs2015-chase-cam` 的烧胎特写提供可靠输入
- 原则：只读诊断，不改变相机行为；NFS 2015 游戏目录未做任何修改

## 0. TL;DR

1. `GetVehicleIsBurnout()` 可用，静止烧胎时为 `1`，怠速/停车时为 `0`。
2. `GetWheelIsBurnout()` / `GetWheelIsBurnoutRep()` 可用，需要 wheel index；本车烧胎时掩码为 **`0x0c`**，即 **车轮 2、3（后轮）**。
3. `GetInputInfo()` 返回值是 16 字节的 `HTVehicleInputInfo`，已用反射确认布局：
   - `ThrottleInput` float @ **0**
   - `SteeringInput` float @ **4**
   - `BrakeInput` float @ **8**
   - `bHandbrakeInput` bool @ **12**
   - 因此之前怀疑的 offset 0 不是手刹，而是油门 float 的低字节；手刹只应读 **offset 12**。
4. 烧胎时后轮 `GetWheelAngularSpeed()` 为 **1718.9**，前轮为 0；车速仍为 **0.00 m/s**，可以明确区分“原地烧胎”和正常行驶。
5. 左右打方向时 `GetRawSteeringInput()` 分别出现 `-1.0` / `+1.0`，可用来选左/右后轮；本次两后轮的 burnout 掩码始终都是 true，所以侧别不能只靠 burnout 掩码。
6. 推荐 v1 触发：**直接 burnout flag 为主，手刹 + 油门 + 速度门为 fallback**；不要仅凭油门触发。
7. `HTVehicleWheelInfo` 里还有 `AxleType`、`SideType`、`Radius`、`WorldTransform`、`ContactPointLocation`、`bIsBurnout`、`RotateSpeed` 等字段，Phase 1/2 可用 `GetWheelInfo()` 做精确选轮和机位定位。

## 1. 测试环境

| 项 | 值 |
|---|---|
| 游戏 | NTE / `HTGame.exe` |
| PID | `23296` |
| Anomaly runtime | `2.7.0` |
| 插件 | `robineatcorn.nfs2015-chase-cam` `2.0.2` + Phase 0 probe |
| 构建 SDK | `Download/Anomaly-2.7.0-sdk` |
| 构建命令 | `./build-mingw.sh --sdk ../Download/Anomaly-2.7.0-sdk --run-tests` |
| 日志 | `C:\Program Files (x86)\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly\logs\anomaly-runtime.jsonl` |
| 反射工具 | `Download/Anomaly-2.7.0-tools/anomaly-cli.exe --pid 23296` |

日志时间使用 UTC；本地时间约为 UTC+8。报告里保留日志原始 UTC 时间，便于 grep。

## 2. 已确认的 UFunction 契约

全部通过运行中游戏的 UE 反射表确认，探针实际调用成功率为 **12/12 resolved，12/12 ok，0 failed**。

| 函数 | Owner | numParms | parmsSize | returnOffset | 参数 / 返回 |
|---|---|---:|---:|---:|---|
| `GetVehicleIsBurnout` | `HTVehicleMovementComponent` | 1 | 1 | 0 | bool @0 |
| `GetVehicleThrottle` | `HTVehicleMovementComponent` | 1 | 4 | 0 | float @0 |
| `GetVehicleBrake` | `HTVehicleMovementComponent` | 1 | 4 | 0 | float @0 |
| `GetRawSteeringInput` | `HTVehicleMovementComponent` | 1 | 4 | 0 | float @0 |
| `GetVehicleSkidAngle` | `HTVehicleMovementComponent` | 1 | 4 | 0 | float @0 |
| `GetPlayerThrottleInput` | `HTWheeledVehicleDrivable` | 1 | 4 | 0 | float @0 |
| `GetWheelIsBurnout` | `HTVehicleMovementComponent` | 2 | 5 | 4 | uint8 index @0，bool @4 |
| `GetWheelIsBurnoutRep` | `HTWheeledVehicleDrivable` | 2 | 5 | 4 | uint8 index @0，bool @4 |
| `GetWheelAngularSpeed` | `HTVehicleMovementComponent` | 2 | 8 | 4 | uint8 index @0，float @4 |
| `GetCurrentRuntimeInfo` | `HTVehicleRuntimeInfoInterface` | 1 | 8 | 0 | UObject* @0 |
| `GetWheelCount` | `HTWheeledVehicleRuntimeInfoBase` | 1 | 4 | 0 | int32 @0 |
| `GetInputInfo` | `HTVehicleRuntimeInfoBase` | 1 | 16 | 0 | `HTVehicleInputInfo` 按值返回 @0 |
| `GetWheelInfo` | `HTWheeledVehicleRuntimeInfoBase` | 2 | 528 | 16 | index，`HTVehicleWheelInfo`（512 B）；尚未在探针中调用 |

精确路径：

```text
/Script/HTGame.HTVehicleMovementComponent:GetVehicleIsBurnout
/Script/HTGame.HTVehicleMovementComponent:GetVehicleThrottle
/Script/HTGame.HTVehicleMovementComponent:GetVehicleBrake
/Script/HTGame.HTVehicleMovementComponent:GetRawSteeringInput
/Script/HTGame.HTVehicleMovementComponent:GetVehicleSkidAngle
/Script/HTGame.HTWheeledVehicleDrivable:GetPlayerThrottleInput
/Script/HTGame.HTVehicleMovementComponent:GetWheelIsBurnout
/Script/HTGame.HTWheeledVehicleDrivable:GetWheelIsBurnoutRep
/Script/HTGame.HTVehicleMovementComponent:GetWheelAngularSpeed
/Script/HTGame.HTVehicleRuntimeInfoInterface:GetCurrentRuntimeInfo
/Script/HTGame.HTWheeledVehicleRuntimeInfoBase:GetWheelCount
/Script/HTGame.HTVehicleRuntimeInfoBase:GetInputInfo
/Script/HTGame.HTWheeledVehicleRuntimeInfoBase:GetWheelInfo
```

注意：不是 `HTWheeledVehicleBase` / `HTVehicleBase`。旧计划里的候选路径已过时，Phase 2 直接用上表。

## 3. `HTVehicleInputInfo` 布局

通过 `GetInputInfo` 的返回属性 `FStructProperty` 和 `HTVehicleInputInfo` 的 `ChildProperties` 反射链读出：

| 字段 | 类型 | 偏移 | 说明 |
|---|---|---:|---|
| `ThrottleInput` | float | 0 | 油门输入，0..1 |
| `SteeringInput` | float | 4 | 转向输入，-1..+1 |
| `BrakeInput` | float | 8 | 刹车输入，0..1 |
| `bHandbrakeInput` | bool | 12 | 手刹，1 = 按下 |

结构体大小 16 字节。`GetInputInfo` 是**返回值**，不是 out 参数；返回值 offset 0。

运行时原始字节（节选）：

```text
// 松手刹、无输入
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

// 手刹按下，油门约 1.0：offset 0 = 00 00 80 3f (1.0f)，offset 12 = 01
00 00 80 3f 00 00 00 00 00 00 00 00 01 00 00 00

// 手刹 + 油门 + 左打方向：offset 4 = 00 00 80 bf (-1.0f)
00 00 80 3f 00 00 80 bf 00 00 00 00 01 00 00 00

// 手刹 + 油门 + 右打方向：offset 4 = 00 00 80 3f (+1.0f)
00 00 80 3f 00 00 80 3f 00 00 00 00 01 00 00 00
```

**重要修正**：早期探针把 offset 0 当作手刹候选。反射证明 offset 0 是 `ThrottleInput` 的 float 低字节，不是手刹。当前代码最终只从 **offset 12** 解析手刹；诊断里的 `B0` 仅作为旧假设的对照保留，不参与 `resolvedHB`。

## 4. 实机日志证据

### 4.1 怠速 / 停车

```text
2026-11-09T18:54:47.011816Z
burnout probe resolved=12/12 ok=12 failed=0
isBurnout=0 HB12=0 HB0=0 throttle=0.000 brake=1.000
steer=+0.000 playerThr=0.000 skid=90.4 wheels=4
burnoutMask=00 repMask=00 angular=0.0/-0.0/0.0/-0.0
input=00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

`GetVehicleIsBurnout` 为 0，两后轮掩码为 0，角速度为 0。

### 4.2 手刹按下、尚未给油

```text
2026-11-09T18:55:27.280711Z
GetInputInfo raw=00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00
burnout state=0 HB12=1 HB0=0 resolvedHB=1 throttle=0.000 brake=1.000 wheels=00
```

说明手刹可以独立于 burnout 被读到；此时 `isBurnout` 仍为 0。

### 4.3 手刹 + 油门，开始烧胎

```text
2026-11-09T18:55:27.530969Z
burnout state=1 HB12=1 resolvedHB=1 throttle=0.700 brake=0.000 wheels=00

2026-11-09T18:55:27.564049Z
burnout state=1 HB12=1 resolvedHB=1 throttle=0.700 brake=0.000 wheels=0c
```

`isBurnout` 在油门约 0.7 时变为 1；后轮掩码在同一轮询窗口内从 `00` 变为 `0c`。因此 v1 需要短 hold（约 0.25–0.3 s）来跨过这段单帧/轮询延迟。

另一组：

```text
2026-11-09T18:56:03.887529Z
burnout state=1 HB12=1 throttle=0.879 wheels=00

2026-11-09T18:56:03.905204Z
burnout state=1 HB12=1 throttle=0.879 wheels=0c
```

### 4.4 左/右打方向

```text
2026-11-09T18:56:05.822021Z
input=00 00 80 3f 00 00 80 bf 00 00 00 00 01 00 00 00
// throttle=1.0, steering=-1.0, brake=0, handbrake=1

2026-11-09T18:56:10.723085Z
input=00 00 80 3f 00 00 80 3f 00 00 00 00 01 00 00 00
// throttle=1.0, steering=+1.0, brake=0, handbrake=1
```

两次都处于烧胎中，`GetWheelIsBurnout` 掩码仍为 `0x0c`（两后轮都为 true），没有出现单侧掩码。结论：

- **侧别选择不能只靠 burnout 掩码**；
- 用 `GetRawSteeringInput` 的符号选边，进入特写后 latch；
- 后续可用 `GetWheelInfo` 的 `SideType` / `WorldTransform` 做更精确的左右确认。

### 4.5 角速度与车速时间序列

用户最后一次保持静止、手刹、油门约 5 秒，未打方向：

```text
2026-11-09T18:57:24.975226Z
burnout active throttle=0.700 brake=0.000 steer=+0.000
playerThr=1.000 skid=89.9 HB12=1 wheels=00 rep=00
angular=-0.0/-0.0/-0.0/-0.0 speed=0.00 fwd=0.00

2026-11-09T18:57:25.477440Z
burnout active throttle=1.000 brake=0.000 steer=+0.000
playerThr=1.000 skid=89.8 HB12=1 wheels=0c rep=0c
angular=-0.0/-0.0/1718.9/1718.9 speed=0.00 fwd=0.00

2026-11-09T18:57:26.975295Z
burnout active throttle=1.000 ... HB12=1 wheels=0c rep=0c
angular=0.0/-0.0/1718.9/1718.9 speed=0.00 fwd=0.00

2026-11-09T18:57:28.473343Z
burnout active throttle=1.000 ... skid=-90.0 HB12=1 wheels=0c rep=0c
angular=0.0/-0.0/1718.9/1718.9 speed=0.00 fwd=-0.00
```

释放后：

```text
2026-11-09T18:57:29.875798Z
burnout state=0 HB12=0 B0=0 resolvedHB=0
throttle=0.167 brake=0.500 wheels=00 speed=0.00 fwd=-0.00
```

观察：

- **车轮 2、3 角速度 = 1718.9**，车轮 0、1 = 0；
- `speed=0.00`，说明是原地烧胎，不是行驶/漂移；
- `skid` 在约 `+90` 和 `-90` 之间翻转，不能单独作为稳定触发信号；
- 松手刹后 `HB12` 立刻回 0，`isBurnout` 也回 0，掩码回 `00`。

角速度单位尚未最终确认（可能是 deg/s、rad/s 或引擎自定义单位；1718.9 的数值本身先作为相对强度用）。Phase 1 如果要拿它做震动幅度，建议先用“后轮角速度 / 某个参考值”做归一化，不要直接把 1718.9 当 rad/s。

## 5. 最终信号表

| 信号 | 来源 | 类型 / 偏移 | 烧胎观察 | 可靠性 | 建议用途 |
|---|---|---|---|---|---|
| 直接 burnout | `GetVehicleIsBurnout` | bool @0 | 静止手刹+油门时为 1；怠速为 0 | 高 | **v1 主触发**，叠加速度门 |
| 每轮 burnout | `GetWheelIsBurnout` | uint8 index @0，bool @4 | 后轮 2/3 = true，掩码 `0x0c` | 高 | 选轮、强度、确认 |
| 每轮 burnout（复制） | `GetWheelIsBurnoutRep` | uint8 index @0，bool @4 | 同样 `0x0c` | 高 | 交叉校验 |
| 手刹 | `GetInputInfo` | `bHandbrakeInput` bool @12 | 按下 1，松开 0 | 高 | fallback / 确认 |
| 油门（车辆） | `GetVehicleThrottle` | float @0 | 0.7–1.0 | 高 | fallback / 强度 |
| 油门（玩家） | `GetPlayerThrottleInput` | float @0 | 1.0 | 高 | fallback / 强度 |
| 刹车 | `GetVehicleBrake` | float @0 | 烧胎中 0；松油门后 0.4–1.0 | 高 | 排除误触发 |
| 转向 | `GetRawSteeringInput` | float @0 | 0 或 ±1.0 | 高 | 选左/右侧 |
| 打滑角 | `GetVehicleSkidAngle` | float @0 | 约 ±90，符号会翻 | 低（会翻转） | 只做诊断，不做触发 |
| 车轮角速度 | `GetWheelAngularSpeed` | uint8 index @0，float @4 | 后轮 1718.9，前轮 0 | 高（数值单位待定） | 强度 / 震动 |
| 车速 | 现有 `GetForwardSpeedKmH` / 速度查询 | float | 0.00 m/s | 高 | 进入/保持速度门 |
| 车轮数 | `GetWheelCount` | int32 @0 | 4 | 高 | 安全门 / 轮询范围 |

## 6. 推荐的 Phase 1/2 判定逻辑

```text
speed_ok      = |speed_mps| < 1.0
on_ground_ok  = car.on_ground
direction_ok  = forward_speed > -0.5

primary       = GetVehicleIsBurnout()
fallback      = handbrake && throttle > 0.15
raw_active    = (primary || fallback) && speed_ok && on_ground_ok && direction_ok
```

- **主触发用 direct flag**；fallback 只在 direct flag 不可用时启用。
- 手刹是实际烧胎条件的一部分：日志中手刹先按下、`isBurnout=0`，给油到约 0.7 后才变 1。不要只凭油门触发。
- 需要 `burnout_hold` 约 0.25–0.3 s：进入瞬间 `wheels` 可能还是 `00`，同一轮询窗口后才变 `0c`。
- 侧别：
  1. 如果左右后轮的 burnout 掩码不同，选 true 的一侧；
  2. 否则用 `GetRawSteeringInput` 的符号（左负右正）；
  3. 进入特写后 latch，完全退出后才允许换边；
  4. 后续可用 `GetWheelInfo` 的 `SideType` 做最终确认。
- 强度：v1 可先用 `throttle`；若使用角速度，先做相对归一化，单位确认前不要写死物理常量。
- 速度超过 hold 门限（建议 3.0 m/s）时快速退出；倒车/腾空/离开载具立即 reset。

## 7. `HTVehicleWheelInfo`：Phase 1 可直接用的字段

通过 `HTVehicleWheelInfo` 的反射布局读出（`GetWheelInfo` 尚未在探针中调用，但契约已确认）：

| 字段 | 类型 | 偏移 | 用途 |
|---|---|---:|---|
| `PhysicalWheelIndex` | int32 | 0 | 物理轮序 |
| `AxleType` | enum | 4 | 前/后轴 |
| `SideType` | enum | 5 | 左/右 |
| `bAffectedBySteering` | bool | 6 | 转向轮识别 |
| `Radius` | float | 8 | 轮半径 |
| `Width` | float | 12 | 轮宽 |
| `RotateSpeed` | float | 28 | 旋转速度 |
| `bIsGrounded` | bool | 36 | 接地 |
| `GroundVelocity` | Vector | 40 | 接地点速度 |
| `bIsBurnout` | bool | 65 | 该轮 burnout |
| `bIsSkidding` | bool | 66 | 打滑 |
| `bIsSlipping` | bool | 67 | 滑移 |
| `SlipRatio` | float | 68 | 滑移率 |
| `SlipAngle` | float | 72 | 滑移角 |
| `bReverse` | bool | 80 | 反转 |
| `ForceFromFriction` | Vector | 88 | 摩擦力 |
| `ContactPointLocation` | Vector | 112 | 接地点世界坐标 |
| `WorldTransform` | Transform | 144 | 车轮世界变换 |
| `bIsInWater` / `bIsInOcean` | bool | 504 / 505 | 环境安全门 |

`GetWheelInfo` 的元数据是 `numParms=2, parmsSize=528, returnOffset=16`；按当前理解是 wheel index @0，512 字节的 wheel info @16。Phase 2 接入前应先用一次探针校验这个布局。

## 8. 代码与安装产物

- 探针代码：`nfs2015-chase-cam/plugin.cpp`
  - 12 个精确函数路径；
  - 直接 getter 20 Hz；
  - runtime info 10 / 4 Hz；
  - 3 个 wheel getter × 最多 8 轮，4 Hz；
  - 诊断面板 6 行；
  - 汇总日志 + 状态变化日志 + 烧胎中 0.5 s 的 `burnout active` 采样；
  - `GetInputInfo` 手刹最终只读 offset 12，`B0` 仅作旧假设对照。
- 构建：`./build-mingw.sh --sdk ../Download/Anomaly-2.7.0-sdk --run-tests`，全部 chase-cam 测试通过。
- 校验：`anomaly-plugin validate` 通过；`anomaly-test-host --reload 2 --ticks 20` 通过。
- 热安装脚本：
  - `install_phase0.ps1`（提权复制 DLL + locales，备份 `plugin.dll.phase0-backup`）
  - `run_install_phase0.ps1`（触发 UAC）
- 运行中日志确认：**12/12 resolved, 12/12 ok, 0 failed**。

## 9. 结论与下一步

Phase 0 **通过**。v1 已经有足够的可靠输入：

- direct flag 做主触发；
- 手刹 + 油门 + 速度门做 fallback；
- 每轮 burnout 做后轮确认/强度；
- steering 做左右侧选择；
- `GetWheelInfo` 为 Phase 1 的精确机位/车轮变换留好了接口。

Phase 1 可以开始实现 `BurnoutShot` 的纯相机模型和单测；Phase 2 再把本报告的信号接进 `CarSample` / presenter 管线。
