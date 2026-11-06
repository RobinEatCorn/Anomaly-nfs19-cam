# 🚀 任务实施计划：NFS 2015 坡度相机逻辑移植

> 交接对象：执行 AI
> 依据：`nfs2015-chase-cam/REPORT-slope-gradient.md`，以及用户在宏观决策轮次中确认的 Q1–Q4 选择。
> 当前阶段：只做计划，不改代码。

## 1. 目标与宏观共识

把 NFS 2015 的上下坡相机高度逻辑移植到 `nfs2015-chase-cam`：

1. **相机位置/轨道跟随车身俯仰。** 相机不再固定在世界水平面，而是随车体前后俯仰一起抬降，使相机到坡面的高度基本稳定。
2. **保留世界水平的注视目标。** 位置跟坡，但注视点继续使用当前的“车头水平偏移”，让地平线不左右歪；上坡时视线自然略抬，下坡时自然略低。
3. **叠加 NFS 的 `GradientAdjustments` 次级微调。** 只对 ±10° 内的坡度生效，上坡强、下坡弱，默认使用 37 辆车的 `0.3 / 0.25 / 0.5 / 2.0`。
4. **模型路径和 route B 固定回退路径都修。** 正常模型路径、`BuildRouteBPoseFromCar()` 回退路径都要使用车身坐标架；route B 的模型合成路径不能重复叠加坡度。
5. **平路行为保持不变。** 现有直线、漂移、刹车俯仰、FOV、震动验收数值不能被改坏。
6. **本次不做车身 roll 和腾空混合。** 当前 `CarState` 没有 roll，`on_ground` 永远为 `true`；硬做会引入假数据。`slopeFollow` 总开关保留，后续可扩展。

### 已确认的宏观决策

| 决策 | 选择 |
|---|---|
| Q1 范围 | 做“车身坐标架 + gradient 微调”，不做 roll/腾空；模型路径和 route B 回退同步修 |
| Q2 视线 | 位置跟随车身俯仰，注视点保持世界水平偏移；只叠很小的坡度注视偏置 |
| Q3 坡度来源 | 渲染线程车辆 proxy 的 forward 向量；处理 `tan(pitch)` 归一化问题；gradient 限幅 ±10° |
| Q4 可调性 | 配置文件完整暴露；界面只加“坡度跟随”总开关和“坡度构图强度”滑杆；默认 37 车数值、默认开启 |

### 关键实现约束（执行前必读）

- **模型不在渲染 transform 之后立即推进。** 相机模型在视点 detour 中推进，而渲染 proxy 的 forward 在渲染线程 transform hook 中发布；两者可能相差一帧。实现时使用“最新已发布的渲染 pitch 快照”，允许最多一帧延迟。稳态上下坡不受影响，快速起伏最多晚约 16 ms。若实机发现明显滞后，按异常汇报处理。
- **`forward` 目前按水平分量归一化。** `plugin.cpp` 的 `RenderTransformDetour()` 用 `1/sqrt(x²+y²)` 缩放三个分量，所以 `forward.z` 实际是 `tan(pitch)`。要改成完整三维归一化，或者用 `atan2(forward.z, hypot(x,y))` 取 pitch；不要直接 `asin(forward.z)`。
- **gradient 限幅只限制微调，不限制基础坐标架。** 基础位置跟随完整车身 pitch；`MaximumGradient = 10°` 只用于 `GradientAdjustments` 的 rest/look 偏置。
- **正负号约定。** `CarState.pitch` 正数表示车头上仰；`slope_look_pitch_` 正数表示视线向上；现有刹车俯仰 `pitch_` 正数表示视线向下。合成时注意 `pitch_ - slope_look_pitch_`。
- **route B 不能双算。** `BuildRouteBPoseFromModel()` 使用的是模型 eye 与模型 car 的差值；模型 eye 已包含坡度。只在 `BuildRouteBPoseFromCar()` 固定回退里补坐标架，不要在模型合成路径再补一次。
- **不要改游戏目录，不要新增硬编码内存偏移，不要改版本门禁和 hook ABI。**

### 建议的数学模型

```text
pitch = atan2(forward.z, hypot(forward.x, forward.y))   // 正数 = 车头上仰
basis = MakeCarBasis(yaw, effective_pitch)               // forward/right/up，roll = 0

pivot_world = car.position + basis * pivot
offset_local = rest - pivot
offset_world = basis * offset_local
offset_world = RotateAxis(offset_world, basis.up, WrapPi(position_azimuth - car.yaw))
desired = pivot_world + offset_world
```

gradient 微调：

```text
slope = clamp(car.pitch, -10°, +10°)
params = slope >= 0 ? Upwards : Downwards
rest.z += slope * params.RestPositionScalar * slopeStrength
slope_look_pitch = slope * params.LookAtAngleScalar * slopeStrength
view_forward = RotateAxis(view_forward, camera_right, pitch_ - slope_look_pitch)
```

默认值：

```text
Upwards:   LookAtAngleScalar = 0.5, RestPositionScalar = 2.0
Downwards: LookAtAngleScalar = 0.3, RestPositionScalar = 0.25
MaximumGradient = 10.0
slopeFollow = true
slopeStrength = 1.0
```

## 2. 微观决策授权范围

以下细节由执行 AI 自行决定，不需要再问用户：

- `Config` 里用嵌套 `GradientParams` 还是平铺字段；helper 的命名和放置位置。
- `DesiredPosition()` 内部的具体实现方式，只要平路结果与现有实现一致。
- pitch 插值、低通/弹簧的具体速率；建议先依赖现有 position spring，必要时再加很小的滤波。
- 渲染 pitch 快照的读取方式、过期阈值、首次无值时的回退策略；默认“保持最后一个有效值一小段时间，然后归零”，并在传送/世界切换时清空。
- route B 固定回退使用的具体常量和是否叠 gradient；建议固定回退只做基础坐标架，不叠 gradient，避免第二套参数。
- JSON 字段顺序、UI 控件位置、本地化文案；建议沿用现有 `localizer.Label()` 模式。
- 测试夹具的具体断言、容差和输出格式。
- 构建工具选择（MSVC 或 MinGW）、是否跑 Wine、是否生成 ZIP；只在用户明确要求时安装到游戏目录。
- 是否在诊断面板增加坡度读数；加的话保持只读、不参与相机计算。

## 3. ⚠️ 异常汇报机制

俗话说计划赶不上变化。执行 AI 如果在实施过程中发现以下情况，**立即停止并向用户汇报**，不要自行扩大范围：

1. 渲染 proxy pitch 无法稳定取得，或一帧延迟在实机上造成明显可见的滞后/抖动。
2. 平路验收数值发生变化，且不是浮点误差级别。
3. 车身坐标架导致地平线左右歪、漂移构图明显变化，或车辆腾空时相机跟着车头乱翻。
4. route B 模型路径出现坡度重复叠加，或固定回退与模型路径结果明显不一致。
5. 配置 schema 变更会导致用户已有设置被重置，或设置字段数量/校验逻辑需要破坏兼容性。
6. 任何需要修改游戏目录、版本门禁、hook ABI 或新增硬编码偏移的情况。

汇报时请说明：触发了哪条假设、当前现象、已尝试的方案、建议的重新规划方向。

## 4. 任务步骤分解

### 📦 任务组 1：相机模型的车身坐标架与坡度数学

* **难度评级**：⭐⭐ 中等
* **目标**：让 `ChaseCam` 在不改变平路输出的前提下，按车身 pitch 放置相机，并支持 NFS gradient 微调。
* **涉及文件**：`chase_cam.hpp`、`car_sample.hpp`
* **具体行动项**：
  - [ ] 在 `CarState` 增加 `double pitch{}`，注释明确“车头上仰为正，弧度”；不要复用现有刹车 `pitch_` 的含义。
  - [ ] 在 `BlendCarSample()` 中插值 `car.pitch`。pitch 在 ±90° 内，用线性插值即可；不要用 `WrapPi`。
  - [ ] 增加可复用的车体基向量 helper，例如 `MakeCarBasis(yaw, pitch)` 和 `CarLocalToWorld(local, basis)`；约定 `forward = (cos p cos y, cos p sin y, sin p)`、`right = (-sin y, cos y, 0)`、`up = Cross(forward, right)`。
  - [ ] 重写 `DesiredPosition()`：pivot 用完整 basis；`rest - pivot` 先转世界，再绕 `basis.up` 旋转 `position_azimuth - car.yaw`。验证 `pitch = 0` 时结果与旧实现逐分量一致。
  - [ ] 在 `Config` 增加坡度参数：`slope_follow`、`slope_max_gradient_degrees`、`slope_strength`、`slope_up`（look/rest）、`slope_down`（look/rest）、可选的坡度注视弹簧。默认值使用 37 车预设。
  - [ ] 在 `GamePreset()` 和 `CinematicPreset()` 中显式设置上述默认值；两个预设都默认开启。
  - [ ] 在 `Update()` 里按第 1 节的公式计算 `slope`、选择 Up/Down、把 rest 高度偏置加到 local `rest.z`，并在 clamp 之前完成。
  - [ ] 增加 `slope_look_pitch_` 状态：目标为 `slope * LookAtAngleScalar * slopeStrength`，用一个小弹簧平滑；`Reset()` 时归零并由现有 settle 循环收敛。
  - [ ] 在构建 view forward 时把刹车 pitch 和坡度 look pitch 合成：`pitch_ - slope_look_pitch_`；注意正负号，并保持现有 `CameraState.pitch_degrees` 的计算方式。
  - [ ] `slope_follow = false` 时让 effective pitch 和 gradient 都为 0，行为退回当前 yaw-only 版本；`slope_strength = 0` 时只关闭 gradient 微调，基础跟随仍然生效。
  - [ ] 确保 `rest.x` 的车内保护、`rest.z` 最低高度、速度/加速/漂移/甜甜圈偏移与旧逻辑顺序一致，不改变平路数值。

### 📦 任务组 2：渲染线程 pitch 接线与 route B 回退

* **难度评级**：⭐⭐⭐ 困难
* **目标**：把渲染 proxy 的真实车身 pitch 送进相机模型，并让 route B 固定回退也使用车身坐标架。
* **涉及文件**：`plugin.cpp`
* **具体行动项**：
  - [ ] 修改 `RenderTransformDetour()` 的 forward 归一化：从“只按水平分量归一化”改为完整三维归一化，保证 `forward.z = sin(pitch)`；或者保留现有归一化但在读取端用 `atan2(z, hypot(x,y))`。
  - [ ] 增加一个只读 helper，用现有 `transform_sequence` seqlock、`transform_valid`、`transform_orientation_valid`、`transform_ns` 读取最新 pitch；检查有限值、新鲜度和水平分量，避免除零。
  - [ ] 在 `UpdateCamera()` 构建 `CarState` 时设置 `car.pitch`：优先使用最新渲染快照；快照无效/过期时保持最后一个有效值一小段时间，再归零；世界切换、传送、重置时清空缓存。
  - [ ] 不要在渲染线程写相机模型；pitch 只通过现有原子快照传递，不新增锁、不分配内存。
  - [ ] 修改 `BuildRouteBPoseFromCar()`：从 fresh `forward` 取 `pitch`，eye 用 `MakeCarBasis(car_yaw, pitch)` 变换 rest；look 仍用 yaw-only 的世界水平偏移。注意该函数输入可能是厘米，模型 helper 是米，保持现有单位转换。
  - [ ] `BuildRouteBPoseFromCar()` 的 eye 高度要使用 `kVehicleAnchorCenterHeightCm` 和现有 `kRouteBRestXM/ZM`，不要改变回退机架的基准点。
  - [ ] 检查 `BuildRouteBPoseFromModel()`：模型 eye 已包含坡度，只做“模型 offset 平移到 fresh car”，不能再次应用 pitch；确认没有双算。
  - [ ] 检查 `BuildRouteBPoseDirect()` / `ApplyRouteBAtRenderTransform()` / `ApplyRouteBEarly()` 都通过上述两个函数，不需要各自复制坡度逻辑。
  - [ ] 首次渲染、模型 pose 过期、forward 无效时保持旧回退行为；所有回退仍走现有计数器，不静默降级。
  - [ ] 可选：在诊断面板显示当前 `pitch` 和 `slope_look`，只读，便于实机调参。

### 📦 任务组 3：配置、界面、持久化与本地化

* **难度评级**：⭐⭐ 中等
* **目标**：让用户能开关坡度跟随、调坡度构图强度，同时保持旧配置文件可读。
* **涉及文件**：`plugin.cpp`、`locales/zh-CN.json`、`README.md`
* **具体行动项**：
  - [ ] 在 settings schema 增加字段，建议命名：`slopeFollow`、`slopeStrength`、`slopeMaxGradientDegrees`、`slopeUpLookAt`、`slopeUpRest`、`slopeDownLookAt`、`slopeDownRest`。
  - [ ] 在 `SetDefaults()`、`LoadSettings()`、`PersistSettings()`、`SettingsValid()` 中接入；旧文档缺少这些字段时使用默认值，不要因为缺字段而拒绝加载。
  - [ ] 检查 `LoadSettings()` 顶部的 `field_count > 24` 限制；新增 7 个字段后正好 24，建议把上限提高到 32，避免以后加字段时再踩一次。
  - [ ] 尽量保持 `kSettingsSchemaVersion = 1`，避免升级插件时清空用户设置；只有配置服务明确要求时才升版本。
  - [ ] 在 `Context` 增加相应 atomic（或一个小的设置快照），在 `UpdateCamera()` 中应用；切换 preset 后要重新应用用户坡度设置，不能被 `SetConfig(GamePreset/CinematicPreset)` 覆盖。
  - [ ] `slopeFollow` 用 `ui->checkbox`；`slopeStrength` 用 `ui->slider_float`，范围 0–2。放在相机距离/高度滑杆附近。
  - [ ] 新增本地化 key，例如 `setting.slope_follow`、`setting.slope_strength`，补 `locales/zh-CN.json` 中文；英文 fallback 直接写在 `localizer.Label()` 里。
  - [ ] `diagnosticFlatPose` 打开时强制关闭坡度跟随和 gradient，保持“position and yaw only”的诊断语义。
  - [ ] 更新 `README.md` 的“使用”和“已知限制”：默认开启、默认 37 车数值、界面开关/滑杆、未做 roll/腾空。
  - [ ] 不要在 UI 里暴露上下坡的每个原始 scalar；这些只放配置文件，符合 Q4 的“最少界面控制”。

### 📦 任务组 4：测试、构建与实机验收

* **难度评级**：⭐⭐ 中等
* **目标**：用离线测试钉住几何和符号，用构建验证插件，再给用户一份实机验收清单。
* **涉及文件**：`tests/chase_cam_test.cpp`、构建脚本
* **具体行动项**：
  - [ ] 增加坡度测试 helper：车放在原点、yaw 固定、pitch = ±10°、速度 0、无漂移；settle 后计算相机相对“过车体、法线为车体 up 的坡面”的垂直距离。
  - [ ] 基础坐标架测试：`slope_strength = 0` 时，上坡/下坡的坡面距离都应接近 `rest.z = 1.49 m`（容差 ±0.05 m）。当前 yaw-only 对应值约为 2.35 m / 0.58 m（同一测量方式），可作为回归对照。
  - [ ] 默认 gradient 测试：上坡约 `1.49 + 0.1745 * 2.0 = 1.84 m`，下坡约 `1.49 - 0.1745 * 0.25 = 1.45 m`（容差 ±0.05 m）。
  - [ ] 限幅测试：pitch = 20° 时，gradient 部分仍按 10° 计算；基础坐标架仍跟随完整 20°。
  - [ ] 总开关测试：`slopeFollow = false` 时，pitch 10° 的输出回到旧 yaw-only 行为（或至少坡面距离明显大于基础跟随值）。
  - [ ] 视线测试：上坡的 `view.pitch_degrees` 应明显高于平路，下坡应明显低于平路；确认 `roll_degrees` 不因坡度产生额外 roll。
  - [ ] 插值测试：`BlendCarSample()` 对 pitch 做线性插值，跨样本不跳变。
  - [ ] 现有 `CheckStraight()` / `CheckDrift()` / 刹车 pitch / FOV / 震动测试必须全部保持通过；平路输出不应因新增字段改变。
  - [ ] 用 `g++ -std=c++20` 或现有 `build-mingw.sh --run-tests` 跑测试；记录实际输出值。
  - [ ] 构建完整插件，确认只导入系统 DLL、导出 `AnomalyPluginEntryV1`；不自动安装到游戏目录。
  - [ ] 给用户一份实机验收清单：10° 上/下坡、坡顶起飞、刹车、漂移、倒车；检查相机高度、地平线、是否抖动、跳坡是否跟随车头乱翻。
  - [ ] 如果实机跳坡明显变差，记录为“腾空层未实现”的已知限制，不要在本任务里临时加假数据。

### 📦 任务组 5：文档、清理与交付

* **难度评级**：⭐ 简单
* **目标**：让改动可复现、可回滚、可交接。
* **具体行动项**：
  - [ ] 在 `README.md` 记录新的配置字段、默认值、UI 控件和“未做 roll/腾空”的限制。
  - [ ] 在 `REPORT-slope-gradient.md` 或新计划文档中记录最终采用的符号、单位、实测数值，方便以后对照 NFS 资产。
  - [ ] 检查 `git diff`：只包含预期文件；不包含游戏目录、构建产物、临时截图。
  - [ ] 记录测试命令和结果；如果生成了包，说明包路径，不主动覆盖用户游戏目录。
  - [ ] 最终交接说明：做了什么、没做什么、哪些值需要实机确认（尤其 gradient 符号/缩放和一帧 pitch 延迟）。

## 5. 验收标准

满足以下条件才算完成：

1. 平路现有验收测试全部通过，关键直线/漂移数值在旧容差内。
2. 10° 上坡/下坡、`slopeStrength = 0` 时，相机到坡面距离约为 1.49 m（±0.05）。
3. 默认 37 车 gradient 下，10° 上坡约 1.84 m、下坡约 1.45 m（±0.05）。
4. `slopeFollow = false` 能退回旧 yaw-only 行为。
5. 视线 pitch 上坡高于平路、下坡低于平路；坡度不引入额外 roll。
6. 正常模型路径和 route B 固定回退都能看到坡度效果，且模型路径没有双算。
7. 插件构建通过，无新增编译警告/错误；没有修改游戏目录。
8. 用户实机确认：上下坡高度稳定、地平线不歪、没有明显一帧滞后或跳坡异常；若有，按异常汇报重新规划。
