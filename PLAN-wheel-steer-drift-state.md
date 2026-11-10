# 漂移前轮反打重设计方案

状态：已实现（2.0.3）  
目标版本：2.0.3  
范围：只改前轮视觉表现，不修改车辆物理、输入、底盘或移动组件。

---

## 1. 问题

当前 2.0.3 的视觉前轮反打有两个问题：

1. **低速起漂/原地掉头也会反打。**
   - 低速掉头时，前轮应该往弯内转，而不是反打。
   - 当前实现只比较“车身前向”和“速度方向”的夹角，无法区分正常低速转弯和后轮真正失去抓地。

2. **漂移/非漂移切换时前轮角度瞬间变化。**
   - 进入时，原厂前轮角可能已经在弯内 20–30°，但插件从 0° 开始写反打。
   - 退出时，插件在大约 0.2° 时直接恢复原厂 `RelativeRotation` 和车轮动画，原厂角可能仍有 15–25°。
   - 当前 360°/s 的反打/回正速度太快，接近瞬时。

---

## 2. 调研范围与方法

NFS 2015 游戏目录只读，没有修改：

```text
D:\Application\Steam\steamapps\common\Need for Speed
```

需要分析的资产只读解包到：

```text
nfs2015-research/extracted/tuning/
```

主要文件：

```text
nfs2015-research/extracted/tuning/defaultdriftconfig.txt
nfs2015-research/extracted/tuning/defaultsteeringconfig.txt
nfs2015-research/extracted/tuning/defaultsteeringwheelconfig.txt
nfs2015-research/extracted/tuning/car_*_driftconfig.txt
nfs2015-research/extracted/tuning/car_*_steeringconfig.txt
```

行为参考了社区逆向工程 NFS15Revival 的 `DriftComponent.cpp`：

```text
nfs2015-research/NFS15Revival/NFS15Revival/FBTypes/Engine/Physics/Vehicle/EAGR/PhysicsComponents/DriftComponent.cpp
```

置信度说明：

- 游戏资产里的字段和数值：高置信度，直接来自 NFS 2015 数据。
- `DriftComponent.cpp` 的进入/退出逻辑：高置信度，社区逆向，但不是官方源码。
- “NFS 2015 前轮自动反打”的视觉表现：来自游戏配置、`RemapSteeringForDrift` hook 和社区观察；未完整反编译视觉层。

---

## 3. NFS 2015 原版怎么处理这两个问题

### 3.1 低速起漂：有硬性速度门槛

`defaultdriftconfig.txt`：

```text
Minimum_speed_to_enter_drift 30.0
Slip_angle_to_enter_drift 6.5
Slip_angle_to_enter_drift_when_handbraking 6.0
Slip_angle_to_enter_drift_when_braking 6.0
Drift_angle_to_exit_drift 3.0
Starting_drift_scale 0.009999999776482582
Donut_speed_limit_high 50.0
```

`DriftComponent.cpp` 中：

```cpp
float speedMph = MpsToMph(nfsVehicle.m_forwardSpeed);
...
if (speedMph <= Minimum_speed_to_enter_drift || ...)
    return;

Vec4 vfAvgRearSlipAngle =
    RadiansToDegrees((tireSlipAngle[2] + tireSlipAngle[3]) * 0.5f);

if (vfAngleToEnterDrift < abs(vfAvgRearSlipAngle)) {
    EnterDrift(...);
}
```

所以：

```text
速度 < 30 mph（约 48 km/h / 13.4 m/s）
    → 不进入 drift
后轮平均滑移 < 6.5°
    → 不进入 drift
后轮滑移 > 6.5°
    → 进入 drift，开始反打
后轮滑移 < 3° 持续
    → 退出 drift
```

**原地掉头/低速掉头在 NFS 里根本不是 drift。**  
速度门会直接排除这种情况，前轮保持原厂转向，也就是往弯内转。

NFS 还有 donut 相关字段：

```text
Donut_yaw_torque
Donut_speed_limit_low
Donut_speed_limit_high
DonutLongitudinalGripMin/Max
DonutLateralGripMin/Max
```

但这些主要用于已经进入 drift 后的低滑移续接/甜甜圈，不是从低速直接起漂。

### 3.2 反打不是“直接指向速度方向”

NFS 的 steering config 里有独立的 countersteer 数据：

```text
Default_steering 0.25
Default_steering_remapping 0.8
Counter_steering_remapping 0.4
Max_countersteering -1.0
Max_countersteering_at_low_slip_angle -0.2
Max_countersteering_at_high_slip_angle -1.0
Low_slip_angle_for_counter_steering 5.0
High_slip_angle_for_counter_steering 40.0
Max_counter_steering_power 1.5
```

steering config 里还有：

```text
CounterSteerRate
ReverseCounterSteerRate 80.0
DriftCountersteerRate 120.0
DriftAngleForCounterSteerRate 40.0
MaxSteeringAngle 35.0
```

注意：

- `Max_countersteering...` 是归一化/remap 量，不是直接角度。
- `DriftCountersteerRate` 是反打速率配置；每辆车不同（例如 BRZ 为 95，默认/多数车为 120）。
- `DriftAngleForCounterSteerRate 40°` 是反打速率曲线的滑移角节点。

社区逆向定位到了游戏里的：

```text
RemapSteeringForDrift
```

这说明 NFS 2015 的前轮角不是简单的：

```text
前轮 = 速度方向
```

而是由 drift 状态、转向输入、滑移角和反打速率共同 remap 出来的。

### 3.3 退出：有锁定期和混合

`DriftComponent.cpp` 中：

```cpp
// the player is locked in to a drift for 0.5 secs before they can exit
else if (mbIsDrifting && mvfTimeDrifting.x > 0.5f) {
    ...
    if (abs(rearSlipAngle) < Drift_angle_to_exit_drift) {
        ...
        ExitDrift(lpSteeringComponent);
    }
}
```

NFS 的退出不是滑移角一低于 6.5° 就切：

- 进入后至少 **0.5 秒**不能退出或接下一个漂移；
- 退出滑移角阈值是 **3°**；
- 有 `Time_to_blend_damping`（玩家车普遍 0.25s）做阻尼混合；
- 有 `Steering_amount_on_exit_drift` 处理退出时的前轮角；
- 反打本身有 rate 曲线，不是一帧到位。

所以 NFS 的观感是：

```text
进入：原厂前轮角 → 逐渐建立反打
保持：反打量随滑移角/速度变化
退出：反打逐渐收回 → 回到原厂转向
```

不是当前的“开关式”切换。

---

## 4. 正常车辆逻辑

### 4.1 低速/小侧滑

前轮由驾驶员方向盘控制，走 Ackermann 几何：

```text
左转：左前轮往左，右前轮也往左，内侧轮角度更大
右转：反之
```

后轮还有抓地力，车辆只是正常转弯，不需要反打。  
原地掉头时更应该如此：前轮指向转弯方向，而不是反打。

### 4.2 后轮失去抓地后

反打是驾驶员对“车尾过度旋转”的修正：

```text
后轮滑移角建立
→ 车身横摆角速度超过路径需求
→ 驾驶员开始反打
→ 前轮指向与车身相反的修正方向
→ 控制 yaw，让车沿想要的路径走
```

目标前轮角可以用自行车模型近似：

```text
δ_front ≈ β_rear + atan(L · yaw_rate / v_x)
```

或按“前轮零滑移”理解：

```text
δ_front ≈ β_rear + (l_f · yaw_rate) / v_x
```

低速时 `v_x` 很小，公式会发散，所以必须做速度门控；低速直接回到原厂转向。

### 4.3 正确时机

- **进入**：车尾先滑，反打随后建立；不是一开始就把前轮打到反方向。
- **保持**：反打量随滑移角和横摆角速度变化，方向盘仍然控制路径。
- **退出**：驾驶员逐渐回正，前轮从反打经过中位回到正常转向；不能一帧跳回。

---

## 5. 当前实现的问题定位

### 5.1 只按速度方向判定

`wheel_steer.hpp` 的 `ComputeWheelSteerTarget`：

```text
target = 车身前向与 world_velocity 的夹角
speed >= 3 m/s 即可激活
enter_slip = full_slip * 0.32
```

问题：

- 低速原地掉头时，车身在转，速度方向和车头有夹角，但这个夹角不代表后轮失控。
- 3 m/s 只有约 6.7 mph，远低于 NFS 的 30 mph。
- 没有后轮滑移角、yaw rate、drift 状态或手刹状态作为门控。

### 5.2 进入时从 0° 开始

`plugin.cpp` 的 `UpdateWheelSteer`：

```text
state.applied_angle 初始为 0
target.active 后立刻 EnsureWheelAnimationDisabled
```

原厂前轮可能已经在弯内 20–30°。停动画后车轮先回正到 0，再走向反打，于是出现“先回正再反打”的跳变。

### 5.3 退出时直接恢复原厂姿态

当前逻辑：

```text
target 不再 active
→ desired = 0
→ MoveTowards(applied, 0, 360°/s)
→ |applied| < 0.2° 时 RestoreWheelPose
```

`RestoreWheelPose` 会：

- 写回捕获的原厂 `RelativeRotation`；
- 把 `bEnableWheelAnimation` 恢复为 1。

如果此时原厂前轮角是 15–25°，就会看到瞬间跳回。  
另外 360°/s 对 40° 只需约 0.11 秒，本身就太接近瞬时。

### 5.4 没有漂移保持时间

当前 target 由每帧速度方向计算，滑移角抖动时会反复 active/inactive。每次切换都会停/恢复动画，造成闪烁。

---

## 6. 推荐方案：NFS 式 drift 状态机 + 连续混合 + 原厂姿态交接

### 6.1 状态机

```text
Grip
  ↓ 速度 >= 最小起漂速度
    且后轮滑移 >= 进入阈值，稳定 0.1s
Entering
  ↓ blend 从 0 到 1
Drifting
  ↓ 滑移 < 退出阈值持续 0.25s
    或速度过低 / 倒车 / 腾空 / 碰撞
Releasing
  ↓ applied 接近原厂角，blend 到 0
Grip
```

状态机负责：

- 进入/退出滞回；
- 最短保持时间；
- 防止滑移角抖动导致反打反复开关；
- 低速时完全不进入 `Entering`。

### 6.2 连续 blend

不再用 `target.active` 做二值开关，而是：

```text
counter_target = clamp(反打目标, -max_angle, +max_angle)
target = lerp(stock_steer_angle, counter_target, drift_blend)
applied = MoveTowards(applied, target, rate * dt)
```

其中：

- `stock_steer_angle` 是原厂前轮角，不是 0；
- `drift_blend` 从 0 到 1 平滑变化；
- 低速时 `drift_blend = 0`，前轮完全交给原厂动画；
- 反打目标仍然可以用速度方向/滑移角计算，但它只占 blend 的一部分；
- 反打目标接近 0 时保留原厂角，不把方向盘回正；只有目标达到约 6° 才真正接管动画。

### 6.3 进入：保留原厂角度

在关闭车轮动画之前：

```text
读取当前前轮世界朝向
→ 计算当前原厂前轮 steer 角
→ applied_angle = 原厂角
→ 再停动画
```

这样第一帧手动姿态和原厂姿态一致，不会出现“先回正再反打”。

如果当前前轮角读取不可靠，可以用已有的：

```text
GetRawSteeringInput
```

乘以原厂最大转向角 35° 作为进入初值。

### 6.4 退出：平滑交接，不立即恢复动画

退出漂移时：

```text
target = 当前原厂前轮角估计值
applied 继续按速率回落到 target
保持动画停用
直到 |applied - stock| < 1~2°，且方向盘接近稳定
再恢复 bEnableWheelAnimation
```

如果原厂角估计不可用，就等到方向盘接近中位、`applied` 接近 0 再恢复，避免从大角度一帧跳回。

### 6.5 建议参数

先用 NFS 原版风格：

| 参数 | 建议默认 | 说明 |
|---|---:|---|
| 最小进入速度 | **30 mph / 13.4 m/s** | NFS 原值；可做成 20–40 mph 可调 |
| 进入后轮滑移角 | **6.5°** | 手刹 6.0°、刹车 6.0° |
| 退出后轮滑移角 | **3.0°** | 配合 0.25–0.5s 持续 |
| 最短漂移保持 | **0.4–0.5s** | 防抖 |
| 最大视觉角 | **40°** | 保持当前 |
| 达到全角的滑移角 | **35–40°** | 当前 25° 太早满舵，建议加大 |
| 反打/回正速率 | **120°/s 左右** | 当前 360°/s 太快；可 attack 150、release 90 |
| 进入 blend | **0.15–0.2s** | 从原厂角平滑到反打 |
| 退出 blend | **0.25–0.3s** | 从反打平滑回原厂 |
| 恢复动画容差 | **1–2°** | 角度接近才交接 |

这些是建议默认值；`wheelSteerMaxDegrees`、`wheelSteerGain`、`wheelSteerFullSlipDegrees`、`wheelSteerRateDegreesPerSecond` 可以继续保留在 UI 里，新增的速度门和保持时间可以先做成内部常量，后续再暴露。

### 6.6 低速掉头策略

推荐先做 **严格 NFS 模式**：

```text
速度 < 30 mph：
    不进入 drift
    不接管前轮
    原厂前轮往弯内转
```

这样：

- 原地掉头：前轮正常往弯内，不再反打；
- 低速手刹掉头：不会误触发；
- 只有真正高速起漂、后轮滑移建立后，才出现反打。

如果以后想要“低速甜甜圈也有反打”，可以加一个默认关闭的 **donut 分支**：

```text
速度低
+ 手刹/大油门
+ 高 yaw rate
+ 后轮滑移高
→ 允许一个小角度反打（例如最大 15°）
```

但不要用它替代正常的低速转向，否则又会回到“原地掉头反打”的问题。

### 6.7 可选：左右前轮分别计算 Ackermann

当前两个前轮用同一个角度。真实车辆和 NFS 都有内外轮差：

```text
内轮转角 > 外轮转角
```

这可以在实现反打时顺便做，但不是这两个问题的主因，优先级低于状态机和混合。

---

## 7. 实现步骤

1. **新增漂移视觉状态**
   - `VisualDriftState { Grip, Entering, Drifting, Releasing }`
   - 保存 `state_timer`、`hold_timer`、`blend`、`stock_steer_angle`。

2. **替换目标函数**
   - `ComputeWheelSteerTarget` 不再单独用 `world_velocity` 直接激活。
   - 输入改为：速度、后轮/车身滑移角、yaw rate、steering input、当前 drift 状态。
   - 输出：`counter_target`、`blend`、`active`。

3. **进入时捕获原厂角**
   - 在 `EnsureWheelAnimationDisabled` 之前读取前轮当前世界朝向。
   - 计算 `stock_steer_angle`，设置 `applied_angle = stock_steer_angle`。
   - 写入第一帧时保留原厂角，再按 rate 过渡到反打。

4. **退出交接**
   - `target.active` 变 false 时不立即 `RestoreWheelPose`。
   - 进入 `Releasing`，目标改为当前原厂角估计值。
   - 保持动画停用，直到角度差在容差内且方向盘稳定。
   - 再恢复 `bEnableWheelAnimation` 和原厂 `RelativeRotation`。

5. **加滞回和保持时间**
   - 进入 6.5°，退出 3°；
   - 进入后 0.4–0.5s 内不退出；
   - 退出条件需要持续 0.25s，避免单帧抖动。

6. **降低角速度**
   - 默认从 360°/s 降到 120°/s 左右；
   - attack/release 可以不同，release 稍慢。

7. **保持所有退出路径恢复原厂状态**
   - 离开载具；
   - 关闭插件/功能；
   - 切换载具；
   - 几何读取失败；
   - 游戏暂停/重载。

---

## 8. 测试计划

### 8.1 单元测试

在 `tests/wheel_steer_test.cpp` 增加：

1. 低速 5 m/s + 30° 速度夹角 → 不激活。
2. 30 mph + 6.5° 后轮滑移 → 激活。
3. 6.5° 进入 / 3° 退出滞回。
4. 进入后 0.4s 内不退出。
5. 每 tick 角度变化不超过 rate。
6. 进入初值等于原厂角，首帧不出现大跳变。
7. 退出恢复前 `|applied - stock| < 容差`。
8. 倒车/腾空/低速不激活。

### 8.2 实机测试

1. 低速原地掉头：前轮往弯内，不反打，动画不被接管。
2. 低速手刹掉头：不进入 drift，前轮保持原厂。
3. 30 mph 以下正常转弯：无变化。
4. 高速起漂：前轮从原厂角平滑过渡到反打，没有回正跳变。
5. 漂移中松油/回正：反打逐渐收回，不闪烁。
6. 漂移退出时方向盘仍在转：前轮平滑交回原厂角，不跳。
7. 连续快速进出漂移：0.4–0.5s 保持时间生效，不反复开关。
8. 离开载具/关闭功能：原厂动画和 `RelativeRotation` 正确恢复。

### 8.3 日志

在现有日志基础上增加一次性/低频状态日志：

```text
wheel drift state=Grip/Entering/Drifting/Releasing
speed=... rear_slip=... yaw_rate=... steer=...
stock=... target=... applied=... blend=...
```

不要每帧刷日志；状态变化或 0.5s 汇总一次。

---

## 9. 验收标准

- 低速掉头/低速手刹掉头不再反打。
- 正常低速转弯前轮仍然往弯内。
- 高速起漂时反打平滑建立，无瞬间跳角。
- 漂移退出时反打平滑收回，无瞬间跳角。
- 漂移状态不会因滑移角抖动而频繁开关。
- 所有退出路径恢复原厂动画和原厂 `RelativeRotation`。
- 只改视觉，不修改车辆物理、输入、底盘或移动组件。

---

## 10. 已采用的决定（2.0.3）

1. 采用严格 NFS 模式：**低于 30 mph 完全不反打**。
2. 本次不实现“低速 donut 反打”选项；donut 分支默认关闭，后续有需要再加。
3. 进入时读取原厂前轮实际世界朝向；退出时用 `GetRawSteeringInput × 35°` 估算原厂角，
   手动角收敛到原厂角附近（1.5°）后才恢复动画。
4. 本次不实现左右前轮 Ackermann；两个前轮仍使用同一视觉反打角。

---

## 11. 相关文件

当前实现：

```text
nfs2015-chase-cam/wheel_steer.hpp
nfs2015-chase-cam/plugin.cpp
nfs2015-chase-cam/tests/wheel_steer_test.cpp
```

NFS 2015 调研资产：

```text
nfs2015-research/extracted/tuning/defaultdriftconfig.txt
nfs2015-research/extracted/tuning/defaultsteeringconfig.txt
nfs2015-research/extracted/tuning/car_*_driftconfig.txt
nfs2015-research/extracted/tuning/car_*_steeringconfig.txt
nfs2015-research/NFS15Revival/NFS15Revival/FBTypes/Engine/Physics/Vehicle/EAGR/PhysicsComponents/DriftComponent.cpp
```

NFS 2015 游戏目录只读，未修改。
