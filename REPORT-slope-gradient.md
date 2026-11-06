# NFS 2015 追车相机：上下坡高度处理调查与实施记录

调查对象：`nfs2015-chase-cam` 当前移植版为什么在上下坡时“上坡偏高、下坡偏低”，以及 NFS 2015 原版用什么机制避免这个问题。

> 实施状态（v1.1.0）：任务组 1–5 已完成并经过用户实机确认。§1–§8 保留实施前的调查和推断，
> 其中“当前移植”指旧版；最终代码、符号、单位和实测值以 §9 为准。

结论先行：

1. **NFS 2015 不是把相机固定在世界水平高度上。** 它的 rest/pivot/look 是一组 **车体局部坐标（car-local）**，由 `ExternalDrivingControllerEntityData.Anchor` 的完整 `LinearTransform`（含 pitch/roll）带动的可能性很高。
2. **`GradientAdjustments` 是第二层、幅度有限的构图微调，不是高度问题的主修复。** 它把坡度限制在 ±10°，再按 `Upwards/Downwards` 对 rest 高度和 look-at 角度做非对称偏置。
3. 当前移植只用了 `RotateYaw`，所以整组 rig 永远平行于世界水平面。10° 坡、车后 5.1 m 时，相机相对路面的高度会从约 1.49 m 变成 **上坡约 2.39 m / 下坡约 0.59 m**，峰峰值接近 1.8 m。这就是用户看到的现象。
4. 修复顺序应该是：**先把 position/orbit 放进车体 local frame，再考虑 gradient trim**。只把 `GradientAdjustments` 加到现有 yaw-only 模型上，量级不够，不能真正解决问题。

> 证据分级：本文中“资产值”均来自解包出的 EBX/JSON，可直接复核；“机制重建”来自字段名、结构、几何和实机视频，属于高/中置信度推断。游戏主程序 `NFS16.exe` 受 Denuvo 保护，静态代码不可读，因此没有逐行代码实证。

---

## 1. 为什么没有直接给出原版代码

- `NFS16.exe` 的 `.rdata` 段熵约为 **8.0**，`.xtls` / `.data1` 段没有文件数据；这是 Denuvo 加密/虚拟化后的典型形态。
- 可读的字符串表里能确认类型和字段名，例如：
  - `ExternalDrivingControllerEntityData`
  - `ExternalDrivingCameraParameters`
  - `CameraGradientAdjustment`
  - `CameraGradientParams`
  - `Downwards` / `Upwards`
  - `LookAtAngleScalar` / `RestPositionScalar`
  - `MaximumGradient`
  - `AboveGroundTestDistance` / `IsAboveGroundTestValid` / `TimeInAir` / `IsAirborne`
- 但真正的更新函数在加密段里，无法用静态反汇编确认符号、单位、正负号。下面关于公式的部分会明确标注“推断”。

---

## 2. 资产里确实存在的坡度结构

### 2.1 `GradientAdjustments`

全局 prefab（`nfs2015-game-files/extracted/race/cameras/chasecameraprefab.txt` 3821-3828）和每辆车 blueprint 里都有：

```text
GradientAdjustments::CameraGradientAdjustment
    Downwards::CameraGradientParams
        LookAtAngleScalar   ...
        RestPositionScalar  ...
    Upwards::CameraGradientParams
        LookAtAngleScalar   ...
        RestPositionScalar  ...
    MaximumGradient 10.0
```

字段含义从命名和相邻结构看：

| 字段 | 合理含义 |
|---|---|
| `MaximumGradient` | 参与调整的最大坡度角，所有样本都是 **10°** |
| `Upwards` | 坡度朝上/车头上仰时的参数组 |
| `Downwards` | 坡度朝下/车头下俯时的参数组 |
| `LookAtAngleScalar` | 坡度对 **视线俯仰角** 的缩放比例 |
| `RestPositionScalar` | 坡度对 **rest 高度** 的缩放/偏移量 |

`Downwards`/`Upwards` 也可能被实现成“相机向下/向上修正”的方向，而不一定是“上坡/下坡”。从全局值的不对称和 FH6 复刻 Mod 的行为看，前者（上坡/下坡）更自然；但这是本文的不确定点之一。

### 2.2 `Anchor` 是完整车体变换，不只是 yaw

`ExternalDrivingControllerEntityData` 的输入里有（`chasecameraprefab.txt` 3675-3710）：

```text
Anchor::LinearTransform
    right::Vec3
    up::Vec3
    forward::Vec3
    trans::Vec3
```

也就是说相机控制器拿到的是一整套车体基向量，而不是一个单独的 yaw 角。相机参数里的位置又是明确的 local 位置：

```text
CameraLocalRestPosition
CameraLocalPivotPosition
CameraLocalLookAtPosition
```

以及运行时偏移：

```text
LocalRestPositionOffset
LocalPivotPositionOffset
LocalLookAtPositionOffset
```

（Civic 的 dump 约 7720-7745 行。）

这套命名强烈暗示：**rest/pivot/look 先定义在车体局部空间，再由 `Anchor` 的完整旋转带入世界**。如果原版只用 yaw，通常不会同时提供完整 `Anchor` 和 `Local*PositionOffset`。

### 2.3 地面/腾空输入

控制器里还有：

```text
AboveGroundTestDistance
IsAboveGroundTestValid
TimeInAir
IsAirborne
```

这些更像“地面测试/腾空状态”的输入，用于处理起跳、落地、离地时不要继续把相机死死贴在地形上；它们不是坡度参数本身。当前移植把 `car.on_ground` 永远设为 `true`（`plugin.cpp` 约 5717 行），所以腾空行为也还没有接。

---

## 3. 实际数值：三组 gradient 预设

### 3.1 全局 prefab

来源：`chasecameraprefab.txt` 3821-3828；`camera-values-prefab.json` 86-95。

| 方向 | LookAtAngleScalar | RestPositionScalar |
|---|---:|---:|
| Downwards | 0.5 | 1.5 |
| Upwards | 0.5 | 4.0 |
| MaximumGradient | 10.0 | |

### 3.2 37 辆车的 action-camera 预设

51 辆车里，37 辆使用：

| 方向 | LookAtAngleScalar | RestPositionScalar |
|---|---:|---:|
| Downwards | 0.3 | 0.25 |
| Upwards | 0.5 | 2.0 |
| MaximumGradient | 10.0 | |

### 3.3 14 辆车的 1.0 预设

另外 14 辆使用：

| 方向 | LookAtAngleScalar | RestPositionScalar |
|---|---:|---:|
| Downwards | 1.0 | 1.0 |
| Upwards | 1.0 | 1.0 |
| MaximumGradient | 10.0 | |

这 14 辆是：

```text
car_bmw_m2_2016
car_bmw_m3e46_2003
car_bmw_m3e92coupe_2012
car_chevrolet_camaroz28_2014
car_ferrari_458italia_2014
car_ferrari_f40_1988
car_ford_focusrs_2015
car_lotus_exiges_2006
car_mclaren_570s_2015
car_mercedesbenz_amggt_2015
car_nissan_silviaspecr_2002
car_porsche_911carreras993_1996
car_srt_vipergts_2014
car_subaru_imprezawrx_2010
```

每辆车 blueprint 里通常有**两个** `ExternalDrivingParametersEntityData` 实例（例如 Civic 的 rest z 分别是 -5.1 和 -4.1，可能是普通/近距或不同相机模式），但**同一辆车的 gradient 值相同**。所以 37/14 的分组是逐车的预设差异，不是“两个相机模式”的差异。

### 3.4 这些数值说明什么

以 37 车预设、10° 坡（0.1745 rad）为例：

```text
上坡 rest 高度偏移 = 0.1745 * 2.0  ≈ +0.35 m
下坡 rest 高度偏移 = 0.1745 * 0.25 ≈ -0.04 m

上坡 look 俯仰偏移 = 0.1745 * 0.5  ≈ +5.0°
下坡 look 俯仰偏移 = 0.1745 * 0.3  ≈ +3.0°（符号取决于实现）
```

**上坡修正明显强于下坡**。这和“上坡需要看到坡顶、下坡要避免相机埋进路面”的直觉一致，也解释了为什么值不是对称的。

但 0.35 m / 0.04 m 这个量级，**远小于**当前 yaw-only 模型的 0.9 m 高度误差。因此如果 NFS 原版只靠这套 gradient，它不可能把上下坡处理得那么稳；前面“车体 local frame”才是主机制。

---

## 4. 机制重建

### 4.1 主机制：rig 在车体局部空间

最合理的更新顺序是：

```text
1. 从 Anchor 取车体 forward / up / right（含 pitch/roll）。
2. rest/pivot/look 使用 CameraLocal* 和运行时 Local*Offset。
3. 用相机自己的 yaw lag 绕车体 up 轴旋转，形成 orbit。
4. 位置 = 车体位置 + 车体旋转 * local offset（再叠 yaw lag）。
5. look-at 目标也按同一套 local 定义计算。
6. 最后用 pitch/yaw/roll 弹簧平滑相机姿态。
```

关键点：**相机的“高度”是相对车体/坡面的，不是相对世界水平面的。** 上坡时车尾向下、下坡时车尾向上，相机跟着车体一起抬/降，因此相机到路面的高度基本保持 `rest.z ≈ 1.49~1.55 m`。

这并不等于“相机会跟着车一起俯仰看天”：

- **位置/orbit** 可以跟车体 pitch；
- **view up** 仍可保持世界向上，让地平线不歪；
- look-at 目标可以保持水平，或只加一部分坡度角。

当前移植把这两件事都做成了 yaw-only，所以地平线是稳的，但高度错了。

### 4.2 次机制：`GradientAdjustments` 的构图偏置

在车体 local frame 已经成立的前提下，gradient 只做小幅偏置。一个和字段名/数值都吻合的重建是：

```cpp
const double slope = Clamp(car.pitch, -10.0° , +10.0°);  // 车头上仰为正
const auto& g = (slope >= 0.0) ? cfg.gradient_up : cfg.gradient_down;

// rest：沿车体 local up 抬高/降低
rest.z += slope_rad * g.rest_position_scalar;   // 37 车：上坡 +0.35 m，下坡 -0.04 m

// look：额外视线俯仰
look_pitch += slope_rad * g.look_at_angle_scalar; // 上坡约 +5°，下坡约 +3°（符号待实机确认）
```

`MaximumGradient = 10°` 说明这是有上限的“构图调味”，不是无限跟随。坡再陡，超出的部分不再加，避免相机动作过大。

> 单位说明：`slope_rad * RestPositionScalar` 按“米/弧度”使用是最自然的量纲解释；`LookAtAngleScalar` 按无量纲角度比例使用。原版也可能用 `sin(slope)`、按 rest 高度比例缩放，或把 offset 写进 `LocalRestPositionOffset`；数值形态相同，差别主要在缩放。不要把这里的 `0.35 m` 当成逐行实证。

### 4.3 为什么上坡强、下坡弱

如果 rig 已经跟随坡面，相机的**绝对高度**在上下坡都会保持稳定；gradient 只决定构图：

- 上坡：多抬高一点，越过坡顶看得更远；
- 下坡：几乎不降，避免相机离路面太近；
- 两者的不对称来自“看不到路”比“相机略高”更严重。

FH6 的 NFS 2015 复刻 Mod（`/tmp/fh6cam/x/mediapc/camera/CameraPhysics.xml`）里有启用的 `PitchAngleDegrees` layer，输出 car-space 的位置和 YP offset；上坡层的幅度也明显大于下坡层。这不能证明 NFS 的代码，但和上面的重建方向一致。

---

## 5. 当前移植到底错在哪

### 5.1 代码位置

`nfs2015-chase-cam/chase_cam.hpp`：

- `CarState` 只有 `yaw`，没有 pitch/roll（约 400-435 行）。
- `DesiredPosition()`：

```cpp
const Vec3 pivot_world =
    Add(car.position, RotateYaw(car.yaw, config_.pivot));
const Vec3 offset = Sub(rest, config_.pivot);
return Add(pivot_world, RotateYaw(position_azimuth, offset));
```

- look target：

```cpp
const Vec3 look_world =
    Add(car.position, RotateYaw(look_azimuth, look));
```

两处都只做了绕世界 Z 的 yaw 旋转。结果：**整组 rest/pivot/look 永远躺在世界水平面上**。

`plugin.cpp` 的 render-thread transform hook 其实已经拿到了带坡度的 forward：

```cpp
// proxy matrix row 0 = local +X (nose)
forward[0] = matrix[0];
forward[1] = matrix[1];
forward[2] = matrix[2];
```

但后面只用了 `atan2(forward[1], forward[0])` 取 yaw，`forward[2]` 的坡度信息被丢掉了。

> 实现细节：当前 `RenderTransformDetour` 用 `1 / sqrt(forward.x² + forward.y²)` 归一化三个分量，所以 `forward[2]` 实际是 `tan(pitch)` 量级，不是单位向量的 `sin(pitch)`。以后取 pitch 应该用 `atan2(forward[2], 1.0)`（在水平归一化后），或者直接用原始矩阵行做完整归一化，不能直接 `asin(forward[2])`。

### 5.2 量化误差

以 `GamePreset` 为例：

```text
rest.x = -5.10 m   （车后 5.1 m）
rest.z =  1.49 m   （车体上方 1.49 m）
坡度 θ = 10°
```

当前 yaw-only：

```text
上坡：相机世界高度 1.49 m；车后 5.1 m 处路面比车低 5.1*tan(10°) ≈ 0.90 m
      相机相对路面 ≈ 1.49 + 0.90 = 2.39 m

下坡：车后路面比车高 0.90 m
      相机相对路面 ≈ 1.49 - 0.90 = 0.59 m
```

峰峰值约 **1.80 m**，而且方向正好是“上坡偏高、下坡偏低”。

车体 local frame：

```text
上坡/下坡都 ≈ 1.49 m（相对坡面）
```

再叠 37 车 gradient trim（10°）：

```text
上坡 ≈ 1.49 + 0.35 = 1.84 m
下坡 ≈ 1.49 - 0.04 = 1.45 m
```

所以 NFS 原版即使在 10° 坡上，相机相对路面的高度也只在 **1.45~1.84 m** 之间变化；当前移植是 **0.59~2.39 m**。这就是体感差异的来源。

---

## 6. 推荐实现

### 6.1 第一步：把 pitch 带进模型

给 `CarState` 增加一个标量即可（roll 可选）：

```cpp
struct CarState final {
  ...
  double yaw{};          // 现有
  double pitch{};        // 新增：车头上仰为正，弧度
  // double roll{};      // 可选：车体右倾为正
};
```

`plugin.cpp` 从 render transform 取：

```cpp
// forward 已经被水平归一化：
// forward = (cos yaw, sin yaw, tan pitch)
const double pitch = std::atan2(forward[2], 1.0);
```

更稳妥的是保留原始矩阵行，做完整归一化后再 `asin`/`atan2`，避免水平归一化带来的 tan 误差。也可以从 `ReadVehicleParentAnchor()` 读到的四元数取 pitch，但 render proxy 的矩阵和实际画车帧更一致。

`CarSample` 的 `BlendCarSample` 需要补一行 pitch 插值；`Reset()` 里把 `pitch` 作为当前工作点的一部分。

### 6.2 第二步：用 car basis 构建 position

核心思路：local offset 先按车体 basis 旋转，再叠相机 yaw lag（绕车体 up 轴）。

```cpp
// 车体基向量（含 pitch；roll 可选）
const Vec3 fwd   = ForwardFromYawPitch(car.yaw, car.pitch);
const Vec3 right = RightFromYawPitch(car.yaw, car.pitch);
const Vec3 up    = Cross(fwd, right);  // 或按手性调整

const Vec3 pivot_world = Add(car.position, CarLocalToWorld(config_.pivot, ...));
Vec3 offset = Sub(rest, config_.pivot);
// 相机 yaw lag 绕车体 up 轴，而不是世界 Z
offset = RotateAxis(offset, up, position_azimuth - car.yaw);
const Vec3 desired = Add(pivot_world, offset);
```

如果暂时不想引入完整 3×3 basis，也可以只做“pitch 修正”：

```cpp
// 车后偏移 rest.x 为负；上坡时把相机沿世界 Z 降低 |rest.x|*tan(pitch)
desired.z -= rest.x * std::tan(car.pitch);
```

但完整 basis 更干净，也能自然处理 roll 和以后的地形法线。

### 6.3 第三步：look target 的选择

有两种可选策略：

**A. 稳定地平线（推荐先做）**

- position 用车体 local frame；
- look target 保持现在的 yaw-only 世界水平目标；
- 这样相机被坡面抬/降，但视线仍看向车，地平线不会随车 roll/pitch 大幅翻滚。

```cpp
const Vec3 look_world =
    Add(car.position, RotateYaw(look_azimuth, look));
```

**B. 更接近 NFS 的 local look**

- look target 也用同一套 car basis；
- 再叠 `GradientAdjustments` 的 look pitch；
- 视线会随坡面俯仰，但地平线仍可保持水平（只 pitch，不 roll）。

建议先做 A，实机对比 NFS 后再决定是否切 B。

### 6.4 第四步：叠 `GradientAdjustments`

在 position/look 已经车体化之后，再加一个小幅 trim：

```cpp
const double max_slope = 10.0 * kRadiansPerDegree;
const double slope = Clamp(car.pitch, -max_slope, +max_slope);
const auto& g = slope >= 0.0 ? config_.gradient_up
                             : config_.gradient_down;

// 37 车 action preset
// gradient_up   = {0.5, 2.0}
// gradient_down = {0.3, 0.25}

rest.z += slope * g.rest_position_scalar;          // 车体 local up
slope_look_pitch_ = slope * g.look_at_angle_scalar; // 叠加到 view pitch
```

注意：

- `rest.z` 是车体 local up，不是世界 Z；
- `slope_look_pitch_` 是额外视线 pitch，不要和刹车 pitch 混在一起；
- 先做车体化，再开这个 trim；否则只是把 0.9 m 的误差改成 0.55 m，仍然明显。
- 如果使用 14 车 `1.0/1.0` 预设，效果会更弱；建议默认用 37 车的 `0.3/0.25/0.5/2.0`。

### 6.5 配置建议

在 `Config` 中新增：

```cpp
struct GradientParams {
  double look_at_angle_scalar{};
  double rest_position_scalar{};
};

GradientParams gradient_up{0.5, 2.0};     // 37 车 action preset
GradientParams gradient_down{0.3, 0.25};
double max_gradient_degrees{10.0};
bool slope_position_follow{true};         // 主修复开关
bool slope_gradient_trim{true};           // 次修复开关
```

这样可以在游戏里分别验证“只车体化”和“车体化 + gradient trim”的差异。

---

## 7. 测试计划

### 7.1 单元测试（加在 `tests/chase_cam_test.cpp`）

构造一个 10° 上坡/下坡的 `CarState`，让 `pitch = ±10°`，yaw lag 为零，跑若干帧后检查：

```text
相机位置到“过车体、沿坡面的平面”的距离
```

期望：

| 模型 | 上坡 10° | 下坡 10° |
|---|---:|---:|
| 当前 yaw-only | ≈ 2.39 m | ≈ 0.59 m |
| 只做车体化 | ≈ 1.49 m | ≈ 1.49 m |
| 车体化 + 37 车 trim | ≈ 1.84 m | ≈ 1.45 m |

测试容差可以先给 ±0.05 m。这样能直接钉住“上坡偏高/下坡偏低”的修复。

### 7.2 实机对比

- 找 NFS 2015 的同一段上坡/下坡（例如视频里的山路下坡）；
- 对比三件事：
  1. 相机到路面的高度是否稳定；
  2. 车在画面中的垂直位置是否稳定；
  3. 地平线是否仍然水平（roll 不应随坡面滚）。
- 如果车体化后视线过于随坡俯仰，改用 6.3 A（look 保持水平）；如果车显得“飘”，再切 6.3 B 或调 `GradientAdjustments`。

---

## 8. 未证实部分与风险

1. **原版代码不可见。** 本文的“车体 local frame”是从 `Anchor`、`CameraLocal*`、`Local*Offset` 和几何行为推断的高置信度结论，不是逐行反编译。
2. **gradient 的符号/单位未实证。** `RestPositionScalar` 按“米/弧度”使用是自然量纲，但原版可能是 `sin(slope)`、按 rest 高度比例、或写入 `LocalRestPositionOffset`；数值形态相同，缩放可能不同。
3. **`Upwards/Downwards` 的命名歧义。** 也可能是“相机向上/向下修正”，而不是“上坡/下坡”。从数值不对称和 FH6 复刻 Mod 看，按坡度方向解释更自然；实机验证时优先看“上坡是否更强”。
4. **roll 未处理。** 当前模型 `CarState` 没有 roll；如果原版 position 也跟随车体 roll，漂移/倾斜路面会有横向位移。建议先只加 pitch，roll 作为下一步。
5. **腾空未处理。** `AboveGroundTestDistance` / `TimeInAir` / `IsAirborne` 说明原版会在离地时混合掉地面跟随；当前 `on_ground` 永远为 true。上下坡修复后，跳跃/坡顶起飞可能还需要单独处理。
6. **14 辆 1.0 车的含义未确认。** 可能是另一套“中性/非 action”预设，也可能是默认值。做 NFS action camera 复刻时，优先用 37 车的 `0.3/0.25/0.5/2.0`。

---

## 附录 A：证据文件

| 内容 | 路径 | 位置 |
|---|---|---|
| 全局 `GradientAdjustments` | `nfs2015-game-files/extracted/race/cameras/chasecameraprefab.txt` | 3821-3828 |
| 全局 JSON | `nfs2015-game-files/camera-values-prefab.json` | 86-95 |
| `Anchor` 完整变换 | `nfs2015-game-files/extracted/race/cameras/chasecameraprefab.txt` | 3675-3710 |
| Civic 两个相机实例的 gradient | `nfs2015-game-files/extracted/vehicles/player/car_honda_civictyper_2000/car_honda_civictyper_2000.txt` | 7704-7711、7918-7925 |
| Civic local offset 字段 | 同上 | 约 7720-7745 |
| 当前 yaw-only position | `nfs2015-chase-cam/chase_cam.hpp` | `DesiredPosition()` 约 790-810 |
| 当前 yaw-only look | `nfs2015-chase-cam/chase_cam.hpp` | `Update()` 约 660 |
| 车体 forward（含坡度） | `nfs2015-chase-cam/plugin.cpp` | render transform hook 约 2410-2490 |
| route B 只取 yaw | `nfs2015-chase-cam/plugin.cpp` | `BuildRouteBPoseFromCar()` 约 2660 |

## 附录 B：51 辆车逐车 gradient 提取结果

- 37 辆：`Down 0.3 / 0.25，Up 0.5 / 2.0`
- 14 辆：`Down 1.0 / 1.0，Up 1.0 / 1.0`
- 全部：`MaximumGradient = 10.0`

逐车明细可用 `/tmp/gradient_values.json` 复现；提取脚本基于 `/tmp/fbs/frostbite3/ebx.py` 和现有的 51 个 `vehicle-blueprints/*.ebx`。

---

## 9. 实施与验收记录（v1.1.0）

> §1–§8 保留实施前的调查与推断；其中“当前移植”指旧的 yaw-only 版本。最终实现以本节为准。

### 9.1 最终采用的车体坐标架

`CarState.pitch` 表示车头上仰，单位为弧度。相机模型使用 roll = 0 的车体基向量：

```text
forward = (cos pitch * cos yaw, cos pitch * sin yaw, sin pitch)
right   = (-sin yaw, cos yaw, 0)
up      = Cross(forward, right)

pivot_world  = car.position + basis * pivot
offset_world = basis * (rest - pivot)
offset_world = RotateAxis(offset_world, basis.up,
                          WrapPi(position_azimuth - car.yaw))
desired      = pivot_world + offset_world
```

`basis * v` 即 `CarLocalToWorld(v, basis)`。pivot、rest 和 orbit 都在车体局部空间里完成，
所以上坡时相机跟着车尾一起降低、下坡时一起升高。`pitch == 0` 仍走旧的 yaw-only 构造，
保证平路输出逐分量不变。

注视目标不放进车体坐标架，而是继续使用世界水平的车头偏移：

```text
look_world = car.position + RotateYaw(look_azimuth, look)
view_pitch = brake_pitch - slope_look_pitch
```

因此位置跟坡、地平线不跟着 roll，也不引入坡度造成的左右倾斜。`view_pitch` 只绕相机
right 轴旋转。

### 9.2 符号、单位与 gradient 合成

最终采用的约定如下：

| 量 | 单位/符号 | 约定 |
|---|---|---|
| `CarState.pitch` | 弧度 | 车头上仰为正；来自渲染 proxy 的 forward |
| `EffectivePitch()` | 弧度 | `slope_follow` 关闭时为 0；开启时限制在 ±89° |
| `slope` | 弧度 | 只用于 gradient trim，限制在 `±slopeMaxGradientDegrees` |
| `rest.z` | 米 | 车体局部 up 方向；基础值 1.49 m |
| `slope_look_pitch_` | 弧度 | 正数表示视线向上；弹簧系数 6.0 |
| `pitch_`（刹车） | 弧度 | 正数表示视线向下；与坡度合成时做减法 |
| `position_azimuth - car.yaw` | 弧度 | 绕 `basis.up` 的相机 yaw 滞后 |

gradient 只对限幅后的坡度生效，基础坐标架仍使用完整车身 pitch：

```text
effective_pitch = slope_follow ? clamp(car.pitch, -89°, +89°) : 0
slope = clamp(effective_pitch, -max_gradient, +max_gradient)
params = slope >= 0 ? slope_up : slope_down

rest.z += slope * params.rest_position_scalar * slope_strength
slope_look_target =
    slope * params.look_at_angle_scalar * slope_strength
```

默认值仍是 37 车 action-camera 预设：

```text
Upwards:   LookAtAngleScalar = 0.5, RestPositionScalar = 2.0
Downwards: LookAtAngleScalar = 0.3, RestPositionScalar = 0.25
MaximumGradient = 10.0
slopeFollow = true
slopeStrength = 1.0
```

`slopeStrength` 和四个 gradient scalar 只作用于次级微调；`slopeStrength = 0` 或
`MaximumGradient = 0` 时仍保留完整车身坐标架。`slopeFollow = false` 时
`EffectivePitch()` 直接返回 0，行为退回旧的 yaw-only 机架。

### 9.3 pitch 来源与 route B

渲染线程的 `RenderTransformDetour()` 把 proxy 的 forward 做完整三维归一化，
因此 `forward.z = sin(pitch)`。读取端同时兼容旧的水平归一化格式（`z = tan(pitch)`）：

```text
pitch = atan2(forward.z, hypot(forward.x, forward.y))
```

快照沿用现有 seqlock、`transform_valid`、`transform_orientation_valid` 和
`transform_ns`：50 ms 内的快照优先；移动中最多保持最后值 250 ms；停车时保持最后一次
朝向；换车、传送和重置会丢弃缓存。没有新增锁、内存分配或硬编码偏移。

route B 的两个路径分工如下：

- `BuildRouteBPoseFromCar()` 固定回退使用 `MakeCarBasis(car_yaw, body_pitch)` 变换
  厘米单位的 rest；look 仍用 yaw-only 世界水平偏移；不叠 gradient。基准仍是
  `kRouteBRestXM = -5.20`、`kRouteBRestZM = 1.55` 和 70 cm 锚点中心高度。
- `BuildRouteBPoseFromModel()` 只把模型 eye 的偏移平移到 fresh car；模型 eye 已含坡度，
  不再应用 pitch，避免双算。三个入口
  `BuildRouteBPoseDirect()` / `ApplyRouteBAtRenderTransform()` / `ApplyRouteBEarly()`
  共用这两个函数。

### 9.4 配置、UI 与诊断

settings schema 增加 7 个可选字段：

```text
slopeFollow = true
slopeStrength = 1.0
slopeMaxGradientDegrees = 10.0
slopeUpLookAt = 0.5
slopeUpRest = 2.0
slopeDownLookAt = 0.3
slopeDownRest = 0.25
```

`kSettingsSchemaVersion` 保持 1；旧配置缺少这些字段时使用默认值，不会重置用户设置。
`LoadSettings()` 的字段数上限已从 24 提到 32。窗口只暴露“坡度跟随（车身俯仰）”总开关
和“坡度构图强度”滑杆（0–2）；其余 scalar 留在设置文件。切换 Game/Cinematic preset 后
会重新应用用户的坡度设置，避免被 `SetConfig()` 覆盖。

`diagnosticFlatPose` 同时关闭模型层坡度和 route B 固定回退的 pitch，保持
“position and yaw only”的诊断语义。

### 9.5 实测几何与平路回归

测试 helper 把车放在原点、yaw 固定，`pitch = ±10°` 或 `±20°`，速度/加速度/漂移为 0，
`Reset()` 后测量相机到“过车体、法线为车体 up 的坡面”的垂直距离。容差 ±0.05 m。

| 场景 | 实测坡面距离 |
|---|---:|
| 10° 上坡，`slopeStrength = 0` | 1.490 m |
| 10° 下坡，`slopeStrength = 0` | 1.490 m |
| 10° 上坡，默认 gradient | 1.839 m |
| 10° 下坡，默认 gradient | 1.446 m |
| 10° 上坡，`slopeFollow = false` | 2.353 m |
| 10° 下坡，`slopeFollow = false` | 0.582 m |
| 20° 上坡，默认 gradient | 1.839 m（trim 仍按 10°） |
| 20° 下坡，默认 gradient | 1.446 m（trim 仍按 10°） |

20° 场景下，实际相机位置与完整 20° 车身基向量的期望位置误差为 **0.000 m**，确认
`MaximumGradient` 没有限制基础坐标架。

视线 pitch（`CameraState.pitch_degrees`，正数向上）：

```text
下坡 -14.047°, 平地 -2.578°, 上坡 +7.798°
```

`slopeFollow = false` 时上坡视线回到平地的 -2.578°。上坡/下坡额外 roll 均为 0.000°。

`BlendCarSample()` 的 pitch 线性插值测试使用 -10° → +30°：

```text
25%: 0.000 rad (0°)
50%: 0.175 rad (10°)
75%: 0.349 rad (20°)
```

平路回归仍通过原有容差，关键直线值没有漂移：

| 检查 | 实测 | 期望 |
|---|---:|---:|
| 电影直线水平距离 | 5.285 m | 5.280 m |
| 游戏直线水平距离 | 6.361 m | 6.350 m |
| 电影直线高度 | 1.569 m | 1.570 m |
| 游戏直线高度 | 1.781 m | 1.780 m |
| 电影直线垂直 FOV | 56.696° | 56.800° |
| 游戏直线垂直 FOV | 47.367° | 48.200° |

漂移、刹车 pitch、FOV、震动、reset、插值等原有检查全部通过；测试夹具共 **107 项检查**。

### 9.6 构建与 Windows 侧验证

本机为 WSL2，Windows PE 可以直接通过互操作运行。使用的命令：

```sh
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -I. \
    tests/chase_cam_test.cpp -o /tmp/chase_cam_test
/tmp/chase_cam_test

g++ -std=c++20 -O1 -fsanitize=undefined \
    -fno-sanitize-recover=all -I. tests/chase_cam_test.cpp \
    -o /tmp/chase_cam_test_ubsan
/tmp/chase_cam_test_ubsan

./build-mingw.sh --run-tests
ctest --test-dir build.mingw --output-on-failure

pkg="$(wslpath -w build.mingw/package/Nfs2015ChaseCam)"
anomaly-plugin validate "$pkg"
anomaly-test-host --plugin "$pkg" --reload 10 --ticks 60
```

本机这两个 Windows 工具位于 `../build-ninja/bin`；`wslpath -w` 只转换传给 Windows
程序的包目录参数。

结果：

- 原生测试：107/107 通过，`-Wall -Wextra -Wpedantic` 无警告；UBSan 无报告。
- `./build-mingw.sh --run-tests`：通过；WSL2 直接执行 PE，纯 Linux 才回退到 wine。
- `plugin.dll`：2.7 MB，PE32+ x86-64，只导入 `KERNEL32.dll` 和 `msvcrt.dll`，
  导出 `AnomalyPluginEntryV1`。
- `ctest`：1/1 通过。
- `anomaly-plugin validate`：
  `valid id=io.github.robineatcorn.nfs2015-chase-cam version=1.1.0 entry=plugin.dll`
- `anomaly-test-host`：
  `ok active=1 reloads=10 ticks=660 windows=11 text=132`，
  停止后 `resource_leases_after_stop=0`、`resource_staging_after_stop=0`、
  `hotkeys_after_stop=0`；`private_growth_bytes=1,519,616`（不同运行略有浮动），
  预算 10,485,760。

包目录为 `build.mingw/package/Nfs2015ChaseCam`。构建产物只留在忽略的 `build.mingw/`
下；没有写入游戏目录，也没有提交 DLL、截图或临时探针。

### 9.7 实机确认与交接

用户反馈实机测试未发现问题。此前提供的验收清单覆盖 10° 上/下坡、坡顶起飞、刹车、
漂移、倒车，以及相机高度、地平线、抖动和跳坡跟随。原计划需要实机确认的 gradient
符号/缩放和一帧 pitch 延迟，用户未报告可见异常。

仍然存在的边界：

- **没有实现车身 roll 和腾空混合。** `onGround` 仍恒为 `true`；跳坡时相机继续按 proxy
  的 pitch 跟随车头。本次实机未报告跳坡异常，但这仍是下一阶段的工作。
- **route B 固定回退不叠 gradient。** 它只在模型 pose 短暂不可用时生效；与正常模型路径
  可能有细微高度/视线差。模型合成路径没有二次叠加坡度。
- **首次加载/重置后**还没有新的 proxy 变换时，会先按平地姿态运行。
- gradient 的 `rad * scalar` 单位、`Upwards/Downwards` 的符号仍是基于资产字段和几何行为
  的重建，不是 `NFS16.exe` 逐行反编译实证；当前数值经过实机观感确认，后续对照资产时
  以这里的符号和表格为准。
- 配置只公开总开关和构图强度；上下坡原始 scalar 需要编辑设置文件，符合 Q4 的最少界面控制。

相关提交为 `5c957e6`、`30e9551`、`6bcf556`、`61e7167`、`97df83c`、`c310f97`、
`dfd8d29`；文档收尾提交见 `git log` 最新记录。回滚时 revert 对应提交即可；坡度设置字段
全部可选，旧配置不会因回滚或升级被清空。
