# 前轮视觉反打：入弯、速度回差与角度修正方案

状态：方案待实现  
适用：`robineatcorn.nfs2015-chase-cam`，版本仍为 **2.0.3**，不发布、不升版本。  
范围：只改前轮视觉姿态，不改车辆物理、输入、底盘、移动组件或游戏目录。

---

## 1. 目标

修正当前视觉反打的四个观感问题：

1. 高速手刹进入尖锐 U 型弯时，前轮没有先向弯内转，而是直接反打。
2. 下坡 U 型弯中速度从 90 km/h 降到 48 km/h 以下再恢复到 80 km/h 时，前轮出现
   “反打 → 不反打 → 再反打”的闪断；漂移角过大时前向速度接近 0，也会触发同样的闪断。
3. 部分漂移中反打角偏大，前轮看起来打得过满。
4. NTE 的车速整体低于 NFS 2015，30 mph（48 km/h）的准入门槛偏高。

同时保持现有约束：

- 两前轮使用同一个视觉角度，不分别做 Ackermann。
- 不改物理；所有退出路径恢复原厂 `RelativeRotation` 和车轮动画开关。
- 普通高速转弯不应因为滑移角刚过阈值就冻结原厂前轮动画。
- 低速原地掉头、低速手刹掉头仍不进入反打。

---

## 2. 当前实现的问题定位

当前 `wheel_steer.hpp` 的核心逻辑：

```text
valid = enabled && speed >= 30 mph && forward_speed > 0.1
start = valid && |travel_angle| >= 6.5°
counter = travel_angle * gain * smoothstep(slip)
angle = Lerp(stock, counter, smoothstep(blend) * counter_authority)
```

### 2.1 为什么 U 型弯没有“先向弯内”

- `counter` 直接来自**车身中心速度方向**与车身前向的夹角；手刹一拉，车身快速旋转，
  这个夹角很快变大，目标立刻变成反打。
- 状态机里没有“入弯保持”阶段。`blend` 和 120°/s 的速率限制只是让反打晚几十毫秒，
  并不是“先保持左转、再反打”的语义。
- 插件在反打目标达到 `counter_authority` 后才接管车轮；接管时读取的“原厂角”可能已经是
  0° 或已经偏向反打，真正入弯时的左转角没有被捕获并保持。
- 结果：玩家看到的是“直接反打”，而不是人类驾驶时的“先入弯、后救车”。

### 2.2 为什么下坡 U 型弯会闪断

- 当前 `valid` 把 **30 mph 准入速度**当成了**持续漂移条件**。
- 速度低于 48 km/h 时 `valid = false`，状态机立即进入 `releasing`，目标回到原厂角；
  速度恢复且滑移仍在时又重新 `entering`，于是出现反打/不反打/再反打。
- 更严重的是 `valid` 还要求 `forward_speed > 0.1`。当漂移角接近 90° 时，车辆总速度
  仍然很高，但前向分量可以接近 0；这会被误判为“没在移动”，同样触发释放。
- 退出逻辑把“进入速度/前向速度门槛”和“漂移是否继续”混在了一起，不符合 NFS 2015
  只在进入时看速度、退出时看滑移角的做法。

### 2.3 为什么反打角会偏大

- 目标角使用**车身中心速度方向**。刚体上不同位置的速度并不相同：

  ```text
  v(P) = v(center) + omega × r(P)
  ```

  前轴在车身前方，漂移时前轴侧向速度与车身中心不同；用中心速度算出的反打角会偏大。
- 当前默认最大角 40°，而 NFS 2015 的方向盘最大角本身是 35°；视觉反打不需要达到 40°。
- 当前滑移角到反打角的曲线在 35° 就饱和；NFS 的 `DriftAngleForCounterSteerRate` 是 40°，
  说明原作的反打增长更晚、更缓。

### 2.4 准入速度偏高

- NFS 2015 是 30 mph（48 km/h），因为 NFS 车辆和赛道速度更高。
- NTE 的常见漂移速度更低，48 km/h 会让一些本应进入漂移的弯道被挡掉。
- 但降低准入速度不能重新引入低速掉头反打，因此必须同时保留低速退出条件和入弯阶段。

---

## 3. NFS 2015 原作如何处理

### 3.1 进入条件：速度只在“进入”时检查

`NFS15Revival` 的 `DriftComponent.cpp` 中，`CheckForEnteringDrift()` 的关键判断是：

```cpp
if (speedMph <= Minimum_speed_to_enter_drift
    || mbIsDrifting
    || ...)
{
    return;
}
```

也就是说：

- 速度门槛只用于**从非漂移进入漂移**；
- 已经处于 drift 时，这个函数直接返回，不会因为速度降到 30 mph 以下而退出；
- 进入条件是**平均后轮滑移角**超过阈值，而不是车身中心滑移角；
- 手刹、刹车、斯堪的纳维亚甩尾有各自的进入滑移阈值。

NFS 的默认值：

```text
Minimum_speed_to_enter_drift             30.0 mph
Slip_angle_to_enter_drift                6.5°
Slip_angle_to_enter_drift_when_handbraking 6.0°
Drift_angle_to_exit_drift                3.0°
Steering_amount_on_exit_drift            0.35
```

因此，NFS 不会在 48 km/h 上下反复进出漂移；它只会在**后轮滑移角低于 3° 并持续一段时间**后退出。
这正好是问题 2 需要的语义。

### 3.2 退出条件：滑移角 + 锁定时间，而不是准入速度

`UpdateDriftState()` 的退出条件包括：

- 腾空、倒车、碰撞、空挡/极低速、手刹、失去轮胎抓地；
- 进入后至少锁定 **0.5s**；
- 锁定结束后，平均后轮滑移角低于 `Drift_angle_to_exit_drift`（默认 3°）；
- 退出时用 `Steering_amount_on_exit_drift`（0.35）缩回方向盘，而不是瞬间回正；
- 空挡判断使用**总速度**：`isInNeutral = |speed| < 0.5 m/s && clutch < 0.5`。
  它不会把“前向速度接近 0、但横向仍在高速滑动”的大角度漂移误判为停车。

没有“速度低于 30 mph 就退出”这一条。NFS15Revival 的复刻代码另有一个约
**13 mph（21 km/h）**的低速退出线，但这不是我们这次要采用的通用维持门槛：按你的
下坡 U 型弯场景，只要车辆还有实际运动、后轮还在滑，就应该继续反打。因此本方案
**不设置维持速度下限**，退出只由滑移角和物理失效条件决定。

### 3.3 反打角：方向盘输入 + 滑移角重映射 + 有限速率

NFS 的 native `RemapSteeringForDrift()` 接收：

```text
steeringInput
slipAngleDegrees
maxSteeringAngle
SteeringComponent
```

转向配置中与反打相关的字段：

```text
MaxSteeringAngle                 35.0°
DriftAngleForCounterSteerRate    40.0°
DriftCountersteerRate            120.0°/s
ReverseCounterSteerRate          80.0°/s
Max_countersteering              -1.0
Max_countersteering_at_low_slip_angle   -0.1 ~ -0.2
Max_countersteering_at_high_slip_angle  -0.8 ~ -1.0
Low_slip_angle_for_counter_steering     3.0° ~ 5.0°
High_slip_angle_for_counter_steering    40.0° ~ 45.0°
Max_counter_steering_power        1.0 ~ 1.5
```

可以确定的设计意图：

- 反打不是“速度方向一偏就瞬间满舵”；
- 方向盘输入仍然是主要意图信号，滑移角决定反打的允许范围和速率；
- 反打以 **120°/s** 的有限速率建立，因此从入弯的左打到反打之间必然有一段过渡；
- 最大方向盘角只有 35°，反打角不应长期超过这个量级；
- 退出时方向盘按 0.35 的比例收回，和 0.5s 锁定一起形成平滑交接。

> 说明：`NFS16.exe` 是 Steam 版静态文件，`RemapSteeringForDrift` 所在的
> `0x1441967A0` 在静态文件中是打包/加密状态；Ghidra 12.1.2 headless 反编译得到的是
> bad instruction，无法直接还原 native 公式。以上结论来自 NFS15Revival 的逆向代码、
> `DriftComponent` 反编译源码和解包后的 tuning EBX，而不是对静态 exe 的臆测。
> 若后续必须拿到 native 精确公式，需要对**运行中的已解密进程**做内存 dump，而不是继续
> 分析静态 `NFS16.exe`。

### 3.4 NFS 的“入弯感”来自哪里

NFS 没有把前轮目标直接设成“车身速度方向”：

- 进入漂移前，前轮仍跟随方向盘输入，玩家能看到向弯内的转向；
- 进入漂移后，`RemapSteeringForDrift` 才把方向盘输入、滑移角和反打速率组合起来；
- 120°/s 的速率限制保证车轮不会一帧从左侧跳到右侧。

当前实现缺少的是第一段“方向盘输入仍在支配前轮”的时间，以及“从入弯角到反打角”的
显式过渡。仅靠 `blend` 和速率限制不够，因为目标从第一帧就已经是反打。

---

## 4. 正常车辆逻辑

### 4.1 刚体上不同位置的速度不同

设车身参考点为 `C`，角速度为 `omega`，某点相对参考点的位置为 `r`：

```text
v(P) = v(C) + omega × r
```

前轴中心 `F` 在车身前方 `a`：

```text
v(F) = v(C) + omega × (forward * a)
```

在车身坐标系（x 前、y 右、z 上）中，近似为：

```text
v_x(F) = v_x(C)
v_y(F) = v_y(C) + r * a
```

其中 `r` 是绕 up 轴的 yaw rate。漂移时 `r` 和 `v_y` 的组合会让前轴侧向速度与车身中心
明显不同，所以：

**前轮位置的速度方向不等于车身中心的速度方向。**

### 4.2 两个前轮也不完全相同

左前轮和右前轮相对前轴中心还有 `±track/2` 的横向偏移：

```text
v_y(FL) = v_y(F) + r * track/2
v_y(FR) = v_y(F) - r * track/2
```

所以严格来说，左前轮和右前轮的速度方向也不同；真实车辆用 Ackermann 让内轮多转一点。
本项目明确要求**两个前轮视觉角度相同**，因此只能选一个代表点：

- 推荐使用**前轴中心**的速度方向；
- 左右轮共享同一个角度，残余的轮胎滑移角由视觉接受；
- 这样比使用车身中心更接近真实前轮的运动方向，也不会让左右轮出现不一致。

### 4.3 用前轴速度算反打

对前轴中心速度 `v(F)` 做地面投影，得到前轴滑移角：

```text
beta_front = atan2(v_y(F), v_x(F))
counter_target = -beta_front
```

当前实现用的是 `v(C)`，在漂移中通常会把反打角算大。改用 `v(F)` 后，yaw 产生的
`r * a` 会部分抵消车尾外甩带来的侧向速度，反打角自然变小，且随车速、yaw rate 和
轴距变化。

### 4.4 进入/退出用后轴滑移更接近 NFS

后轴中心：

```text
v(R) = v(C) - omega × (rear_offset)
```

后轴滑移角通常比车身中心更大，更接近 NFS 使用的“平均后轮滑移角”。因此建议：

- **状态机进入/退出**：优先用后轴滑移角；读取不到车轮位置时退回车身中心滑移角；
- **前轮目标角**：始终用前轴中心速度；
- 这样“何时算漂移”和“前轮打多少”各用更合适的物理量。

---

## 5. 最终方案

### 5.1 状态机

在现有 `grip → entering → drifting → releasing → grip` 中增加 `turn_in`：

```text
grip
  ├─ 达到进入速度 + 进入滑移 + 方向盘向弯内
  │     └─> turn_in（保持原厂/入弯角）
  └─ 达到进入速度 + 进入滑移 + 方向盘已反打/未知
        └─> entering（直接进入反打混合）

turn_in
  ├─ 保持时间到 / 后轴滑移继续增大 / 方向盘不再向弯内
  │     └─> entering
  └─ 滑移低于退出阈值并持续退出确认时间
        └─> releasing

entering
  └─ blend 到 1 ─> drifting

drifting
  └─ 滑移低于退出阈值持续 0.25s
        └─> releasing

releasing
  └─ blend 到 0 且手动角收敛到原厂角 ─> grip
```

### 5.2 入弯阶段（turn_in）

进入漂移的第一帧：

- 计算后轴滑移、前轴速度、原始反打目标；
- 如果原始反打目标有足够幅度（避免普通高速转弯冻结原厂动画），才允许接管；
- 判断方向盘意图：
  - `steering_input * raw_counter < 0`：方向盘指向弯内（例如左漂中仍打左），进入 `turn_in`；
  - `steering_input * raw_counter > 0`：玩家已经在反打，跳过 `turn_in`，直接 `entering`；
  - 方向盘未知：短暂 `turn_in` 或不保持，按保守策略处理。

`turn_in` 期间：

- 目标角 = `steering_input * 35°`（方向盘入弯角），没有方向盘信号时用实测原厂角；
- 接管动画，但**不向反打目标混合**；
- 保持 `0.15–0.25s`，默认 `0.18s`；
- 如果后轴滑移已经很大（例如 >18°）或方向盘已经回正/反打，可以提前结束；
- 保持结束后进入 `entering`，用 0.20s 从入弯角平滑混合到反打角。

这样高速 U 型弯的视觉顺序是：

```text
左打（入弯） → 保持约 0.18s → 经过中心 → 右打（反打）
```

而不是：

```text
0° 或直接右打
```

### 5.3 速度只用于进入，不用于维持

```text
entry_valid = enabled
              && total_speed >= entry_speed
              && forward_speed > 0.1

engaged     = turn_in / entering / drifting / releasing
```

- 只有 `grip` 状态使用 `entry_valid`；
- **一旦进入 turn_in/entering/drifting，不再因为速度、前向速度下降而退出**；
- 漂移角接近 90° 时 `forward_speed` 接近 0 是正常现象，不能作为退出条件；
- 维持阶段只由后轴滑移角低于 `exit_slip` 并持续 `exit_hold` 决定退出；
- 倒车、腾空、失去抓地/碰撞等物理失效仍然退出；
- 释放后设置 0.5s 的再进入冷却，避免真正退出后在阈值附近来回闪断。

推荐值：

```text
entry_speed = 40 km/h (11.1 m/s)
```

40 km/h 比 NFS 的 48 km/h 低约 17%，适合 NTE 的速度。如果实机仍觉得 40 km/h 偏高，
可以再降到 35 km/h，但不建议低于 30 km/h，否则低速掉头会重新进入反打。

> **关于“总速度几乎为零”**：不再设置任何维持速度阈值。只有总速度矢量退化到无法
> 定义方向（例如小于 0.5 m/s）时，才保留最后一次 counter 目标最多 `0.5s`；速度恢复后
> 继续使用同一 drift 状态，超过 `0.5s` 仍无法定义方向才释放。这个 0.5 m/s / 0.5s
> 是数值退化保护，不是“低于 20 km/h 就退出”的玩法门槛。

### 5.4 前轴目标角

新增纯函数，输入：

```text
car_forward, car_up
world_velocity
yaw_rate
front_axle_offset (从车辆参考点到前轴中心)
rear_axle_offset
```

计算：

```text
omega = car_up * yaw_rate
v_front = world_velocity + cross(omega, front_axle_offset)
v_rear  = world_velocity + cross(omega, rear_axle_offset)

front_travel = travel_angle(car_forward, car_up, v_front)
rear_travel  = travel_angle(car_forward, car_up, v_rear)
```

- 状态机用 `|rear_travel|` 进入/退出；
- 反打目标用 `front_travel`；
- 读取不到车轮位置时，`front/rear_offset = 0`，退回当前车身中心逻辑；
- 读取不到 yaw rate 时，使用 0 或上一帧缓存，并记录降级状态。

### 5.5 反打目标曲线

```text
slip_for_curve = |rear_travel|          // 没有后轴时用 |front_travel|
normalized = clamp((slip_for_curve - enter_slip) /
                   (full_slip - enter_slip), 0, 1)
curve = smoothstep(normalized)
raw_counter = clamp(front_travel * gain * curve, -max_angle, +max_angle)
```

建议：

```text
full_slip = 40°       // 对齐 NFS DriftAngleForCounterSteerRate
max_angle = 32°       // 默认值，不再默认 40°
gain      = 1.0       // 前轴修正已经会自然减小角度
```

如果实机仍觉得大，再把默认 `max_angle` 降到 30°，而不是直接把 `gain` 调得很低；
`gain` 会改变小角度响应，`max_angle` 只限制极限。

### 5.6 角度混合

- `turn_in`：目标为入弯角；
- `entering`：

  ```text
  blend = smoothstep(t / blend_in_seconds)
  angle = Lerp(stock_angle, counter_angle, blend)
  ```

- `drifting`：目标为 `counter_angle`；
- `releasing`：

  ```text
  blend 从 1 回到 0
  angle = Lerp(stock_angle, last_counter_angle, blend)
  ```

- 仍然使用 `counter_authority` 作为“是否值得接管”的门槛；一旦接管，`turn_in` 和
  `releasing` 期间保持接管，直到手动角收敛到原厂角附近才恢复动画。
- 退出时保留最后一次反打目标（现有 `state.counter_angle` 逻辑），避免低速时先回正再回原厂。

### 5.7 车轮位置与 yaw rate 的读取

现有偏移：

```text
SceneComponent ComponentToWorld rotation: +0x1F0 (quat, 4×double)
SceneComponent ComponentToWorld translation: +0x210 (3×double, cm)
```

实现时：

1. 在 `ResolveWheelSteerGeometry` 或每帧读取四个车轮和 mesh 的 translation；
2. 前轴中心 = 两个前轮 translation 的平均；
3. 后轴中心 = 两个后轮 translation 的平均；
4. `front_offset = (front_center - mesh_translation) / 100.0`（米）；
5. `rear_offset = (rear_center - mesh_translation) / 100.0`；
6. yaw rate 用 mesh quaternion 的帧间差分估计：

   ```text
   delta = q_now * conjugate(q_prev)
   angle = 2 * atan2(|delta.xyz|, delta.w)
   axis = normalize(delta.xyz)
   yaw_rate = dot(axis, car_up) * angle / dt
   ```

   做低通/限幅，避免悬挂和噪声造成抖动；
7. 任一步失败时退回中心速度，不阻塞功能。

---

## 6. 参数汇总

| 参数 | 当前 | 方案 | 说明 |
|---|---:|---:|---|
| 进入速度 | 30 mph / 48 km/h | **40 km/h** | 降低 NTE 准入门槛 |
| 维持速度 | 同进入速度 + 前向速度门槛（bug） | **无** | 进入后只由滑移角退出 |
| 进入滑移 | 车身 6.5° | 后轴 6.5°，回退车身 | NFS 用后轮滑移 |
| 退出滑移 | 车身 3° | 后轴 3°，回退车身 | 保持 NFS 量级 |
| 入弯保持 | 无 | **0.18s**（0.12–0.25） | 只在本方向盘向弯内时 |
| 入弯最大保持 | 无 | **0.30s** | 防止高速 U 型弯拖太久 |
| 入弯提前结束 | 无 | 后轴滑移 >18° 或方向盘回正/反打 | 避免过度延迟 |
| 反打混合 | 0.20s | 0.20s | 保持现有平滑 |
| 反打速率 | 120°/s | 120°/s | 对齐 NFS |
| 全滑移角 | 35° | **40°** | 对齐 NFS `DriftAngleForCounterSteerRate` |
| 默认最大角 | 40° | **32°** | 降低视觉过角 |
| 最大角范围 | 20–50° | 20–50°（默认 32°） | 保留用户可调 |
| 退出确认 | 0.25s | 0.25s | 保持 |
| 漂移锁定 | 0.5s | 0.5s | 保持 |
| 再进入冷却 | 无 | **0.5s** | 防止真正退出后阈值附近闪断 |
| 静止退化保持 | 无 | **0.5s** | 仅总速度 <0.5 m/s 时保留最后目标 |
| 恢复容差 | 1.5° | 1.5° | 保持 |

> `entry_speed` 先作为内部常量，不增加 UI 滑条；如果实机发现不同车辆差异明显，再考虑
> 增加“漂移准入速度”高级选项。`stay_speed` 不实现。

---

## 7. 实现步骤

### 7.1 `wheel_steer.hpp`

1. 增加 `WheelSteerPhase::turn_in`。
2. `WheelSteerConfig` 增加：
   - `entry_speed_mps`
   - `turn_in_seconds`
   - `turn_in_max_seconds`
   - `turn_in_release_slip_radians`
   - `reentry_cooldown_seconds`
   - `standstill_grace_seconds`（默认 0.5s，仅用于速度矢量退化）
3. `WheelSteerDriftState` 增加：
   - `turn_in_seconds`
   - `reentry_cooldown`
   - `entry_steering_sign`（可选）
4. 增加 `WheelSteerAxleMotion` / `ComputeAxleTravel` 纯函数：
   - 输入中心速度、yaw rate、前后轴偏移；
   - 输出 `front_travel`、`rear_travel`；
   - 离轴位置速度用 `v + cross(up * yaw_rate, offset)`。
5. `AdvanceWheelSteerTarget` 改为接收：
   - `front_travel_angle`
   - `rear_travel_angle`
   - `steering_input`
   - `entry_valid`
   或者在函数内部接收速度/yaw/offset，保持纯函数可测。
6. `grip` 状态分别判断：
   - `entry_valid && rear_slip >= enter`
   - `steering_into = steering_input * raw_counter < 0`
   - `turn_in` 或 `entering`
7. `turn_in` 目标角使用方向盘入弯角（`steering_input * stock_max`），不混合 counter。
8. `releasing` 保留最后 counter；`grip` 清零状态并启动 `reentry_cooldown`。

### 7.2 `plugin.cpp`

1. 新增常量：
   - `kWheelSteerEntrySpeedMps = 40.0 / 3.6`
   - 不再新增 `kWheelSteerStaySpeedMps`；维持阶段没有速度下限
   - `kWheelSteerTurnInSeconds = 0.18`
   - `kWheelSteerTurnInMaxSeconds = 0.30`
   - `kWheelSteerTurnInReleaseSlipDegrees = 18.0`
   - `kWheelSteerReentryCooldownSeconds = 0.5`
   - `kWheelSteerStandstillGraceSeconds = 0.5`（仅用于总速度退化保护）
2. 默认最大角 `40 → 32`；全滑移 `35 → 40`。
3. 在 `WheelSteerState` 增加：
   - 前后轴中心位置/偏移
   - 上一帧 mesh quaternion、时间
   - 平滑后的 yaw rate
   - 当前是否已有 yaw rate
4. 每帧读取 mesh 和车轮 translation（+0x210），计算前后轴中心；失败回退 0 偏移。
5. 计算 yaw rate，传给纯函数。
6. 在 `grip → turn_in` 时立即 `MeasureStockWheelSteer`，不要等到反打 authority 达标才测；
   入弯角从这里开始接管。
7. `UpdateWheelSteer` 日志增加：
   - `phase`
   - `steering`
   - `front_slip`
   - `rear_slip`
   - `yaw_rate`
   - `turn_in`
   - `counter`
8. 退出/切换载具/关闭功能时仍走 `RestoreWheelPose`。

### 7.3 测试

扩展现有 `tests/wheel_steer_test.cpp`：

1. **入弯**：40 km/h 以上、后轴滑移 20°、方向盘向弯内，前 0.1s 目标是入弯角，
   不是反打；0.2s 后开始向反打混合。
2. **已反打**：同样滑移但方向盘已反打，跳过 turn_in，直接 entering。
3. **速度回差**：48→45→80 km/h，滑移保持 10°，状态保持 `drifting`，counter 不重置。
4. **大角度近零前向速度**：总速度仍高、`forward_speed ≈ 0`、后轴滑移 30°，状态必须
   保持 `drifting`，counter 不归零；总速度恢复到正常值后继续同一 drift。
5. **完全静止退化**：总速度降到 0.5 m/s 以下时保留最后一次 counter 短暂保持；不因
   “低于某个 km/h”立即释放；速度恢复后仍可继续。
6. **前轴修正**：相同车身滑移下，加入正 yaw rate 和前轴偏移后，前轴目标角小于中心目标角；
   yaw rate=0 时两者一致。
7. **两轮同角**：左右前轮使用同一个 `target_forward`。
8. **默认角**：默认最大角为 32°，60° 滑移仍被钳制。
9. **NFS 值**：进入 6.5°、退出 3°、锁定 0.5s、速率 120°/s 保持。
10. **普通高速转弯**：小 counter 目标不接管动画，保留原厂角。

### 7.4 实机验证

按以下顺序测试，并保存日志：

1. 90 km/h 左 U 型弯，手刹进入：前轮先左打约 0.15–0.25s，再平滑反打。
2. 下坡 U 型弯，速度 90→48 以下→80 km/h：状态保持 `drifting`，无反打/不反打/再反打。
   同时测试漂移角接近 90°、前向速度接近 0 但总速度仍高的时刻，反打不能消失。
3. 大角度漂移：反打目标不超过默认 32°，目视不再“打满”。
4. 30–35 km/h 低速掉头/手刹掉头：不进入反打。
5. 40 km/h 以下普通转弯：不接管动画。
6. 漂移退出并继续打方向：平滑交回原厂角，无跳变。
7. 连续快速进出：0.5s 锁定 + 0.5s 再进入冷却，不闪烁。
8. 离开载具/关闭功能：原厂 `RelativeRotation` 和动画开关恢复。

日志应能看出：

```text
grip -> turn_in speed=... steer=... rear_slip=... yaw=...
turn_in -> entering ... 
entering -> drifting ...
drifting -> releasing ...
releasing -> grip ...
```

而不是：

```text
grip -> entering -> drifting -> releasing -> grip -> entering ...
```

---

## 8. 验收标准

- 高速手刹 U 型弯：入弯阶段至少有 0.12s 的向弯内前轮姿态，之后才反打。
- 进入后速度在 48 km/h 上下变化、甚至前向速度接近 0，只要总速度有效且后轴滑移
  未低于退出阈值：状态不退出 drift，反打不消失。
- 默认反打角不超过 32°；大滑移下也不超过 UI 设定上限。
- 30–35 km/h 低速掉头不进入反打；40 km/h 以上才可能进入。
- 普通高速转弯不因为 6.5° 滑移而冻结原厂动画。
- 退出、切车、关闭功能、停止插件全部恢复原厂车轮姿态。
- `ctest` 3/3 通过，`anomaly-plugin validate` 和 `anomaly-test-host` 通过。

---

## 9. 风险与待定

1. **后轴滑移是估算值**：NTE 没有直接暴露后轮轮胎滑移，后轴刚体滑移是近似。若进入
   过于敏感，可把进入阈值从 6.5° 调到 8°，或先只用车身滑移，后轴只用于目标角。
2. **车轮 translation 偏移**：`+0x210` 已在现有代码中用于 mesh/车轮位置读取，但需要在
   实机确认四个车轮的 translation 都是世界坐标、单位是 cm；失败时回退中心速度。
3. **yaw rate 噪声**：quaternion 差分需要低通；如果悬挂/碰撞导致尖峰，应限幅并在
   0.1s 内保持上一有效值。
4. **turn_in 接管过早**：如果 40 km/h 普通转弯也触发，说明 counter authority 门槛太低；
   应保持现有“只有反打目标有实际幅度才接管”的门槛。
5. **方向盘符号**：UE 轴和 `travel_angle` 的符号必须用测试锁定；实现时先写一个
   “左漂 + 方向盘左”的用例，确认 `steering * raw_counter < 0`。
6. **默认 32° 是否够**：如果用户喜欢更大的视觉角，UI 仍可调到 50°；但默认值应保守。
7. **NFS native 公式未静态还原**：若需要精确复刻，需要运行中进程 dump；当前方案以
   NFS 配置量级 + 正常车辆运动学为准。
8. **取消维持速度后的低速边界**：车辆真正停住或总速度接近 0 时，速度矢量无法定义
   反打方向。此时保留最后一次 counter 一小段时间，速度恢复则继续；如果长期停住，
   再由滑移角/静止条件释放。不要为了处理这个边界重新引入 20 km/h 的维持门槛。

---

## 10. 相关文件

```text
wheel_steer.hpp                  状态机、纯几何、目标角
plugin.cpp                       yaw rate/轴位置读取、动画接管、日志
tests/wheel_steer_test.cpp       入弯/回差/前轴修正测试
PLAN-wheel-steer-drift-state.md  2.0.3 已实现方案（历史）
PLAN-wheel-steer-turn-in-speed.md 本方案
```

参考：

```text
nfs2015-research/NFS15Revival/.../DriftComponent.cpp
nfs2015-research/extracted/defaultdriftconfig.txt
nfs2015-research/extracted/tuning/defaultsteeringconfig.txt
nfs2015-research/NFS15Revival/.../RevivalDriftComponent.cpp
```
