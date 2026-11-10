# 前轮 turn-in 修正计划：按漂移强度缩放 stock 角

> 状态：已实现（源码、离线模型测试与插件构建均已完成）。  
> 目标文件：`nfs2015-chase-cam`  
> 性质：仅视觉动画覆盖，不修改车辆物理、输入、轮胎力或 Chaos 车轮状态。

## 实现摘要

- `wheel_steer.hpp`：新增 `turn_in_alpha` / `turn_in_duration` 的滑移缩放、视觉目标合成和可测试的 applied-angle 扫角 helper。
- `plugin.cpp`：接管时只保存测量到的 stock 角作为起始姿态，不再覆盖 raw stock target；turn_in 按 alpha 缩放目标与持续时间，并使用独立扫角速度；释放回到 entry stock。
- 实际默认参数：`turn_in_start=8°`、`turn_in_full=22°`、`minimum alpha=0.08`、turn-in rate `300°/s`（最大 `900°/s`）。
- 已通过 `ctest` 的三个离线夹具，并用 Anomaly 2.7.0 SDK 成功构建 `plugin.dll`。
- 未安装到游戏目录。

---

## 1. 背景

当前 `nfs2015-chase-cam` 的前轮反打逻辑里有一个 `turn_in` 阶段：

```text
漂移触发
  -> turn_in：先保持/显示向弯内的 stock 角
  -> entering：混合到 countersteer
  -> drifting：保持反打
```

这个阶段的目的是解决：

> 高速进入尖锐 U 型弯、拉手刹触发漂移时，如果直接反打，看不到前轮先向弯内打，观感反逻辑。

但当前实现有两个问题：

### 1.1 turn_in 实际被冻结在“接管瞬间的小角度”

`UpdateWheelSteer` 接管时会把目标角覆盖成测量到的实际 stock 角：

```cpp
target.stock_angle_radians = measured_stock;
target.angle_radians = Lerp(
    target.stock_angle_radians,
    target.counter_angle_radians,
    target.angle_blend);
state.applied_angle = target.stock_angle_radians;
```

`turn_in` 阶段 `angle_blend = 0`，于是：

```text
目标角 = 接管瞬间实际角
当前角 = 同一个实际角
```

而 NTE 原厂前轮有约 1 秒的渐进。高速 U 型弯拉手刹时，接管可能发生在实际前轮只有 3–8° 的时候，于是 `turn_in` 冻结在 3–8°，看起来几乎没有“正打”。

**问题不是 0.12 秒太短，而是这 0.12 秒里前轮没有目标可走。**

### 1.2 turn_in 对所有漂移使用同一个 full stock 角

另一个问题是，如果简单改成“turn_in 目标 = raw input × 35°”，小角度漂移也会出现：

```text
前轮先明显打向弯内 35°
再反打
```

这对小甩尾来说过于夸张。小角度漂移不需要和高速 U 型大漂移一样明显的 stock 角。

---

## 2. 目标

1. 高速 U 型大漂移：仍然能看到明显的前轮向弯内打，再反打。
2. 小角度漂移：turn_in 很轻、很短，甚至直接跳过；不要固定 35°。
3. turn_in 阶段从“冻结”改为“快速扫向目标”。
4. 退出时仍然平滑回到接管瞬间的实际 stock 角，避免跳变。
5. 不修改任何物理字段，只写前轮组件的 `RelativeRotation` 和动画实例的 `bEnableWheelAnimation`。

---

## 3. 总体设计

引入一个按后轴滑移角缩放的 `turn_in_alpha`：

```text
turn_in_alpha = SmoothStep(
    turn_in_start_slip,
    turn_in_full_slip,
    abs(rear_travel_angle)
)
```

建议初始值：

```text
turn_in_start_slip = 8–10°
turn_in_full_slip  = 20–25°
```

行为：

| 后轴滑移角 | turn_in_alpha | 行为 |
|---:|---:|---|
| 6.5–9° | 0–0.1 | 基本跳过 turn_in，直接短混合到反打 |
| 10–15° | 0.15–0.45 | 轻微向弯内带一点 |
| 15–22° | 0.45–1.0 | 逐渐明显 |
| >22–25° | 1.0 | 完整 stock 角，明显正打后再反打 |

关键原则：

> `turn_in_alpha` 同时缩放 **目标角** 和 **turn_in 持续时间**，而不是只缩放其中一个。

---

## 4. 具体行为

### 4.1 接管时保存实际 stock 角，但只作为起始姿态

接管瞬间测量实际前轮角：

```cpp
double measured_stock = 0.0;
const bool have_measured =
    MeasureStockWheelSteer(context, state, car_forward, car_up,
                           measured_stock);
```

保存为 entry stock：

```cpp
state.entry_stock_angle =
    have_measured
        ? Clamp(measured_stock,
                -config.max_angle_radians,
                config.max_angle_radians)
        : target.stock_angle_radians;
state.entry_stock_valid = true;
state.applied_angle = state.entry_stock_angle;
```

**不要**再覆盖 `target.stock_angle_radians` 和 `target.angle_radians`。  
`target.stock_angle_radians` 继续表示 raw input 对应的 stock 目标。

### 4.2 turn_in 目标角按 alpha 缩放

设：

```text
raw_stock = steering_input × stock_max_angle
entry_stock = 接管瞬间实际角
```

则：

```cpp
turn_in_target =
    Lerp(entry_stock, raw_stock, turn_in_alpha);
```

效果：

- `alpha = 0`：目标 = 实际角，不额外打；
- `alpha = 0.5`：目标 = 实际角和 raw stock 的中间值；
- `alpha = 1`：目标 = 完整 raw stock。

### 4.3 entering 的起点也使用缩放后的 stock

如果只在 `turn_in` 阶段缩放，进入 `entering` 时又会从 raw stock 开始，导致小漂移仍然先跳到 full stock。

因此 `entering` 的混合起点应使用：

```cpp
turn_in_target =
    Lerp(entry_stock, raw_stock, turn_in_alpha);

entering_target =
    Lerp(turn_in_target, counter_angle, angle_blend);
```

也就是说，把状态机给出的 `target.angle_radians` 在运行时替换为：

```cpp
target.angle_radians =
    Lerp(turn_in_target,
         target.counter_angle_radians,
         target.angle_blend);
```

这样：

- 小漂移：从实际角附近直接混合到反打；
- 大漂移：从完整 stock 角混合到反打。

### 4.4 小漂移直接跳过 turn_in

当 `turn_in_alpha` 很低时，不需要进入 `turn_in`：

```cpp
constexpr double kMinimumTurnInAlpha = 0.08;
if (turn_in_alpha < kMinimumTurnInAlpha) {
  state.phase = WheelSteerPhase::entering;
  state.turn_in_seconds = 0.0;
}
```

或者在 `AdvanceWheelSteerTarget` 的 `grip` 分支里直接选择：

```cpp
state.phase =
    steering_into && turn_in_alpha >= kMinimumTurnInAlpha
        ? WheelSteerPhase::turn_in
        : WheelSteerPhase::entering;
```

这样小角度漂移不会出现“先冻结一小下”的动作。

### 4.5 turn_in 持续时间按 alpha 缩放

建议：

```text
alpha = 0     -> 0–0.03s
alpha = 0.5   -> 0.07–0.10s
alpha = 1     -> 0.14–0.18s
```

实现上可以在进入 `turn_in` 时保存：

```cpp
state.turn_in_duration = Clamp(
    config.turn_in_seconds * turn_in_alpha,
    config.turn_in_min_seconds,
    config.turn_in_max_seconds);
```

`turn_in` 的退出条件从固定的 `config.turn_in_seconds` 改为 `state.turn_in_duration`。

原有 `turn_in_min_seconds` 可以保留为硬下限，例如 0.06–0.08s，防止高滑移时动作完全看不见。

### 4.6 turn_in 扫角速度

`turn_in` 阶段需要保证在持续时间结束前扫到目标：

```cpp
double rate = config.rate_radians_per_second;

if (target.phase == WheelSteerPhase::turn_in) {
  const double duration =
      std::max(state.turn_in_duration, 1e-3);
  const double needed_rate =
      std::fabs(target.angle_radians - state.applied_angle) / duration;
  rate = std::max(rate, needed_rate);
}

state.applied_angle = MoveTowards(
    state.applied_angle, target.angle_radians, rate * step);
```

建议将 turn_in 的角速度上限单独考虑，避免高滑移时一帧跳太多：

```text
turn_in rate range: 250–400°/s
countersteer rate: 保持现有 120°/s
```

如果不想新增配置，可以直接用 `needed_rate`，但建议加一个上限。

### 4.7 释放阶段回到 entry stock

`releasing` 阶段不要回到 raw stock，而是回到接管瞬间的实际角：

```cpp
if (target.phase == WheelSteerPhase::releasing &&
    state.entry_stock_valid) {
  target.stock_angle_radians = state.entry_stock_angle;
  target.angle_radians =
      Lerp(state.entry_stock_angle,
           target.counter_angle_radians,
           target.angle_blend);
}
```

这样退出时：

```text
反打角 -> 接管时实际角 -> 原厂动画
```

不会因为 raw input 仍然是 35° 而把视觉轮卡在 35° 再突然还给原厂。

---

## 5. 代码改动点

### 5.1 `wheel_steer.hpp`

#### `WheelSteerConfig`

新增：

```cpp
double turn_in_start_slip_radians{};
double turn_in_full_slip_radians{};
```

可选新增：

```cpp
double turn_in_rate_radians_per_second{};
```

#### `WheelSteerTarget`

新增：

```cpp
double turn_in_alpha{};
```

#### `WheelSteerDriftState`

新增：

```cpp
double turn_in_duration{};
double turn_in_alpha{};
```

#### `AdvanceWheelSteerTarget`

计算：

```cpp
const double turn_in_alpha = SmoothStep(
    config.turn_in_start_slip_radians,
    config.turn_in_full_slip_radians,
    absolute_slip);
```

`grip -> turn_in` 时保存：

```cpp
state.turn_in_alpha = turn_in_alpha;
state.turn_in_duration = Clamp(
    config.turn_in_seconds * turn_in_alpha,
    config.turn_in_min_seconds,
    config.turn_in_max_seconds);
```

低 alpha 时直接进入 `entering`。

`turn_in` 退出条件使用 `state.turn_in_duration`，并保留 `turn_in_min_seconds` 硬下限。

`WheelSteerTarget` 输出：

```cpp
result.turn_in_alpha = turn_in_alpha;
```

### 5.2 `plugin.cpp`

#### 常量

新增：

```cpp
constexpr double kWheelSteerTurnInStartSlipDegrees = 8.0;
constexpr double kWheelSteerTurnInFullSlipDegrees = 22.0;
constexpr double kWheelSteerMinimumTurnInAlpha = 0.08;
```

可选：

```cpp
constexpr double kWheelSteerTurnInRateDegreesPerSecond = 300.0;
```

#### `WheelSteerState`

新增：

```cpp
double entry_stock_angle{};
bool entry_stock_valid{};
```

#### `UpdateWheelSteer` 接管块

从：

```cpp
target.stock_angle_radians = measured_stock;
target.angle_radians = Lerp(...);
state.applied_angle = target.stock_angle_radians;
```

改为：

```cpp
state.entry_stock_angle = measured_stock;
state.entry_stock_valid = true;
state.applied_angle = state.entry_stock_angle;

// 不覆盖 target.stock_angle_radians / target.angle_radians
```

#### 目标角替换

在 `AdvanceWheelSteerTarget` 之后：

```cpp
const double raw_stock = target.stock_angle_radians;
const double turn_in_target =
    Lerp(state.entry_stock_angle, raw_stock, target.turn_in_alpha);

target.angle_radians =
    Lerp(turn_in_target,
         target.counter_angle_radians,
         target.angle_blend);
```

`releasing` 时使用 `entry_stock_angle` 作为回退目标。

#### 恢复时清理

`RestoreWheelPose` 中清理：

```cpp
state.entry_stock_angle = 0.0;
state.entry_stock_valid = false;
```

---

## 6. 参数建议

### 6.1 turn_in 强度

| 参数 | 建议值 |
|---|---:|
| `turn_in_start_slip` | 8–10° |
| `turn_in_full_slip` | 20–25° |
| `minimum_turn_in_alpha` | 0.08 |
| `turn_in_seconds` | 0.14–0.18s |
| `turn_in_min_seconds` | 0.06–0.08s |

如果希望小漂移更干净：

```text
turn_in_start_slip = 10°
turn_in_full_slip  = 24–26°
```

如果希望中等漂移也更有 NFS 味：

```text
turn_in_start_slip = 8°
turn_in_full_slip  = 18–20°
```

### 6.2 扫角速度

```text
turn_in_rate = 250–350°/s
countersteer_rate = 120°/s（保持现状）
```

### 6.3 可选的 UI 配置

后续可以暴露：

```text
wheelSteerTurnInStartSlipDegrees
wheelSteerTurnInFullSlipDegrees
wheelSteerTurnInRateDegreesPerSecond
```

第一版建议先用内部常量，避免 UI 和状态机同时改动过大。

---

## 7. 测试计划

### 7.1 纯状态机测试

在 `tests/wheel_steer_test.cpp` 增加：

1. 小滑移：
   - `rear_slip = 8°`
   - `turn_in_alpha ≈ 0`
   - 不进入 `turn_in`，直接 `entering`
   - 目标角接近 entry stock，不接近 full stock

2. 中等滑移：
   - `rear_slip = 15°`
   - `0 < turn_in_alpha < 1`
   - turn_in 目标角在 entry stock 和 raw stock 之间

3. 大滑移：
   - `rear_slip = 22–25°`
   - `turn_in_alpha ≈ 1`
   - turn_in 目标角接近 raw stock

4. 高滑移 U 型弯：
   - 左打、手刹、后轴滑移大
   - 必须进入 `turn_in`
   - 目标为左 stock，不能立即 countersteer

5. 已反打输入：
   - 玩家已经向右反打
   - 跳过 `turn_in`，直接 `entering`

### 7.2 运行时/集成测试

如果不好做完整游戏集成测试，可以抽出纯函数：

```cpp
ComputeTurnInTarget(entry_stock, raw_stock, alpha)
```

测试：

```text
entry = -5°
raw = -35°
alpha = 0    -> -5°
alpha = 0.5  -> -20°
alpha = 1    -> -35°
```

以及：

```text
entry = -5°
raw = -35°
duration = 0.12s
期望 applied 在 0.12s 内接近 -35°
```

### 7.3 手工验收

1. **高速 U 型左漂**
   - 左打 + 手刹
   - 应看到前轮明显向左，再反打
   - 不能一上来就直接反打

2. **小角度左甩尾**
   - 轻微左打 + 手刹
   - 前轮不应先打到 35°
   - 动作应短、轻，直接进入轻微反打

3. **中等角度漂移**
   - 前轮有部分左打，再反打
   - 不应在 full stock 和零之间跳变

4. **退出漂移**
   - 反打角平滑回到 stock
   - 不能从反打直接跳回原厂动画

5. **插件关闭**
   - 恢复原厂前轮动画
   - 不残留手动 `RelativeRotation`

---

## 8. 风险与对策

| 风险 | 对策 |
|---|---|
| 小漂移仍然出现明显 stock | `turn_in_alpha` 低时跳过 turn_in，目标接近 entry stock |
| 大漂移看不到正打 | `turn_in_alpha = 1`，目标 raw stock，按 duration 扫过去 |
| turn_in 一帧跳太多 | 限制 turn_in rate 上限 |
| 退出时跳变 | `releasing` 使用 `entry_stock_angle`，不要用 raw stock |
| 滑移角噪声导致 turn_in 抖动 | 对 `rear_travel_angle` 做低通，或对 alpha 做滞回 |
| 视觉与物理前轮不一致 | 仅视觉覆盖，物理不变；这是插件既有设计 |
| 高速时 full stock 视觉过夸张 | 可选按速度或 `steering_input` 缩放 raw stock；第一版先观察 |

---

## 9. 非目标

本计划明确不做：

- 不修改 `HTVehicleMovementComponent` 的 `+0x33c` / `+0x354`
- 不修改 Chaos 车轮物理 state
- 不修改轮胎、悬挂、发动机、变速箱、刹车
- 不修改 ESP、AutoCenter、漂移辅助的物理参数
- 不修改输入映射

只修改：

```text
前轮组件的 RelativeRotation
车辆动画实例的 bEnableWheelAnimation
```

---

## 10. 实施顺序

1. 在 `wheel_steer.hpp` 增加 `turn_in_alpha` / duration 计算，先不接运行时。
2. 补纯状态机测试，确认小/中/大滑移分档。
3. 修改 `UpdateWheelSteer` 接管逻辑：
   - measured stock 只作为起始姿态；
   - 不再覆盖 target stock；
   - 目标角按 alpha 缩放。
4. 加入 turn_in 扫角速度。
5. 修改 releasing 使用 `entry_stock_angle`。
6. 手工测试 U 型漂移、小甩尾、中等漂移、退出。
7. 如需要，再把三个参数暴露到插件 UI。
