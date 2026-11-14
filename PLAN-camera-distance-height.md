# 🚀 阶段性实施计划：机位距离 / 高度双参数（方案 A）

> 目标仓库：`Anomaly-nfs19-cam/`
> 本文档记录本阶段计划与实施状态；与实现一并提交。本阶段不修改版本号。
>
> **实施状态：** 任务组 0–4 的代码改动、迁移/面板/分组测试、因子纯函数单测，以及任务组 5
> 的构建、`ctest` 6/6、包校验和 host 冒烟已完成；游戏内手测（默认构图对比、双轴拖动、
> route B 锚点切换、真实旧版降级加载）仍需在实机执行。

## 1. 当前阶段目标与宏观共识

### 1.1 背景：现有两个参数为什么难用

当前“机位几何”组里有两个互相纠缠的参数：

- `cameraScale`：对相机相对车身原点的**整体**偏移做统一缩放，水平距离和垂直高度一起变。
- `originOffsetCm`：只对**原点**做垂直平移，看起来像“高度”，但它同时还是缩放的支点。

内部变换是：

```text
origin = 车身锚点 + originOffsetCm
eye    = origin + (model_eye - origin) × cameraScale
```

默认 `originOffsetCm = -70` 的真实作用是把“车身中心”锚点换到“地面”支点；`cameraScale`
再以这个支点缩放整个机位。于是：

- 想把相机拉远，必须动 `cameraScale`，但高度也会跟着变；
- 想单独调高度，只能动 `originOffsetCm`，但它又会改变缩放的支点；
- 用户无法直接表达“水平 6.0 m、高度 1.2 m”这种组合；
- “车身原点偏移”这个词对普通用户没有意义。

用户反馈的核心不是数值范围，而是**自由度分配**：一个控件同时管两个轴，另一个控件只
管一个轴，怎么调都凑不出想要的结果。

### 1.2 本阶段目标

把上述两个参数替换成两个**米制绝对值**滑杆，并让它们真正各管一个轴：

| 界面名称 | JSON 字段 | 默认值 | 建议范围 | 含义 |
|---|---|---:|---|---|
| 水平距离（静止，车后） | `cameraDistanceM` | 5.10 m | 3.0 – 8.2 m | 静止时相机在车后的纵向距离 |
| 高度（静止，离地） | `cameraHeightM` | 1.49 m | 0.45 – 4.6 m | 静止时相机离地高度 |

必须做到：

1. 静止时滑杆读数就是真实值：距离 5.10 就是 5.10 m，高度 1.49 就是 1.49 m。
2. 拖动“距离”只改变水平位置，静止高度不变；拖动“高度”只改变垂直位置，静止距离不变。
3. 速度、漂移、加速、坡度带来的动态增量，按各自轴的比例跟着缩放，不再由另一个轴
   的参数决定。
4. 默认配置（5.10 / 1.49）在 NFS 预设下与当前版本逐像素一致。
5. 旧配置可以无损/近似迁移；新配置写回旧字段后，旧版本仍能加载。
6. 不新增第三个“比例”或“动态幅度”参数。

### 1.3 方案 A 的核心设计

**一句话：两个米制绝对值滑杆，按轴独立缩放（x/y 用距离因子、z 用高度因子），模型原点
固定在地面，默认 5.10 / 1.49；不碰模型内部的姿态与动力学，只改 presenter / route B
和设置层。**

#### 1.3.1 基准与因子

设当前活动预设的基础姿态为 `preset_rest`，来自 `ChaseCam::config().rest`：

- NFS：`{-5.10, 0.0, 1.49}`
- Cinematic：`{-5.20, 0.0, 1.55}`

定义两个因子：

```text
fx = cameraDistanceM / (-preset_rest.x)     // 水平（x、y）因子
fz = cameraHeightM   /  preset_rest.z       // 垂直（z）因子
```

眼睛的世界位置从**地面原点** `ground` 出发：

```text
eye.x = ground.x + (model_eye.x - ground.x) × fx
eye.y = ground.y + (model_eye.y - ground.y) × fx
eye.z = ground.z + (model_eye.z - ground.z) × fz
```

其中 `model_eye` 是模型当前输出的、相对模型原点的姿态（基础姿态 + 速度/漂移/加速/坡度
增量）。因为模型原点现在固定在地面，所以：

- 静止时 `model_eye - ground = preset_rest`，乘完因子正好是 `(-cameraDistanceM, 0, cameraHeightM)`；
- 速度后拉、漂移推近等**水平**增量乘 `fx`；
- 速度抬升、漂移下压等**垂直**增量乘 `fz`；
- 两个滑杆互不影响。

#### 1.3.2 地面原点

删除用户可调的 `originOffsetCm` 语义。模型原点固定为：

```text
origin_cm[2] = base_cm[2] - kVehicleAnchorCenterHeightCm   // 即 -70 cm
```

`base_cm` 仍然是车身中心（锚点），`kVehicleAnchorCenterHeightCm = 70` 把车身中心换到
地面。这个常量以前只是 `originOffsetCm` 的默认值，现在变成内部约定，用户不再看到。

#### 1.3.3 相机朝向

**不改。** 模型仍然用预设基础姿态（NFS 5.10/1.49，Cinematic 5.2/1.55）计算旋转；
presenter / route B 只缩放眼睛位置，不改变旋转。因此：

- 默认 NFS 下，画面与今天完全一致；
- 抬高/拉远时，车在画面里移动，而不是重新瞄准车——这与今天 `originOffset` /
  `cameraScale` 的观感一致；
- 非默认配置的构图变化只来自“按轴缩放”本身，不额外引入“重新瞄准”的行为变化。

如果未来要做“相机始终瞄准车”，那是另一个增强项（方案 B），不在本阶段。

#### 1.3.4 与 `speedDistanceScale` 的关系

`speedDistanceScale`（速度/加速距离倍率）**保留**。它仍然在模型内部乘在速度/加速的
水平增量上；随后 presenter 再乘 `fx`。因此：

```text
速度/加速的有效水平幅度 = speedDistanceScale × fx
```

两者职责不同：`speedDistanceScale` 是“动态距离倍率”，`cameraDistanceM` 是“静态机位
距离”。不要把它们合并。

#### 1.3.5 与预设的关系

因子以**当前预设自己的** `preset_rest` 为分母，所以静止时眼睛始终精确落在
`(cameraDistanceM, cameraHeightM)`，对 NFS 和 Cinematic 都成立。

但要注意 Cinematic 的细节：

- 它的基础姿态是 5.2 / 1.55，用户默认值是 5.10 / 1.49；
- 默认时 `fx = 5.10 / 5.2 ≈ 0.9808`，`fz = 1.49 / 1.55 ≈ 0.9613`；
- 静态位置精确是 5.10 / 1.49；
- 模型旋转仍按 5.2 / 1.55 的姿态计算，所以默认构图会比今天偏约 10 cm / 6 cm；
- 它的动态增量在默认时会缩小约 2%–4%。

这个差异很小，本阶段接受并在文档中注明。不要为此再引入“预设偏移”或第二套默认值。

### 1.4 迁移与兼容

#### 1.4.1 旧值 → 新值

```text
cameraDistanceM = 5.10 × 旧 cameraScale
cameraHeightM   = 1.49 × 旧 cameraScale + (旧 originOffsetCm + 70) / 100
```

示例：

| 旧 `cameraScale` | 旧 `originOffsetCm` | 新距离 | 新高度 | 说明 |
|---:|---:|---:|---:|---|
| 1.0 | -70 | 5.10 | 1.49 | 默认，完全等价 |
| 0.8 | -70 | 4.08 | 1.192 | `fx = fz = 0.8`，静态/动态都等价 |
| 1.0 | 0 | 5.10 | 2.19 | 静态高度等价；垂直动态按 2.19/1.49 放大 |
| 1.0 | -150 | 5.10 | 0.69 | 静态高度等价；垂直动态按 0.69/1.49 缩小 |

- 只调过 `cameraScale`、`originOffsetCm` 仍为 -70 的用户：`fx = fz = 旧 scale`，完全等价。
- 只调过 `originOffsetCm` 的用户：静态高度等价；垂直动态现在按高度比例缩放，量级在
  厘米到十几厘米，写进迁移说明。
- 默认 `scale=1, offset=-70`：新默认 5.10 / 1.49，完全等价。

#### 1.4.2 新值 → 旧字段（降级写回）

为了旧版本仍能加载，持久化时同时写旧字段：

```text
oldScale  = clamp(cameraDistanceM / 5.10, 0.6, 1.6)
oldOffset = clamp(100 × cameraHeightM - 70 - 149 × oldScale, -150, 150)
```

**必须夹紧。** 例如高度 4.6、距离 3.0 时，反算偏移约为 +301，超出旧 schema 的
`[-150, 150]`；旧版本的 `SettingsValid` 会因此拒绝整份文档。夹紧后旧版本至少能加载，
但极端组合的静态高度会与新版不同。这是降级的已知近似，写进文档。

#### 1.4.3 读取顺序

- 先读旧字段 `cameraScale` / `originOffsetCm`（缺失时用 1.0 / -70）；
- 再读新字段 `cameraDistanceM` / `cameraHeightM`；
- 新字段存在且合法 → 直接用；
- 新字段缺失或非法 → 按 1.4.1 从旧字段推导，再夹紧到新范围；
- 两者都缺失 → 默认 5.10 / 1.49；
- 最后用新值做 `SettingsValid` 校验。

#### 1.4.4 schema 策略

- **版本保持 1**，不升级。
- 新增 `cameraDistanceM`、`cameraHeightM` 两个可选 number 属性，范围 3.0–8.2 /
  0.45–4.6。
- `required` 列表保持原样（仍包含 `cameraScale`、`originOffsetCm`），因为我们持久化时
  总会写旧字段；旧文档也一定带旧字段。
- `additionalProperties:false` 保持不变。
- 旧版本读取新文档：按当前实现 `read` 不做 schema 校验（以任务组 0 复核为准），未知字段
  会被忽略；旧版本重新写入时会丢掉新字段，这是预期行为，不做额外兼容。

### 1.5 非目标

- 不做模型内部的“基础姿态替换 / 重新瞄准”（方案 B）。
- 不新增第三个“比例”或“动态幅度”参数。
- 不改弹簧、坡度、漂移前轮、烧胎、FOV 分组的逻辑。
- 不改 `speedDistanceScale` 的语义。
- 不升级 schema 版本。
- 本阶段提交代码、测试、文档和本计划；不修改版本号。

## 2. 微观决策授权范围

执行 AI 可以自行决定，不必逐项请示：

- 新字段的最终命名（建议 `cameraDistanceM` / `cameraHeightM`）。
- UI widget id（建议 `chase-camera-distance` / `chase-camera-height`）与滑杆顺序。
- 本地化 key 的具体命名与中文措辞；英文 fallback 必须保留。
- 是否在状态行增加“相机高度”实时值；建议加，便于用户理解“静止”与实际值的差异。
- 因子计算的辅助函数放在 `presenter.cpp`、`internal.hpp` 还是 `chase_cam.hpp`。
- 因子安全夹紧的具体范围（建议 `[0.05, 10.0]`，或只做 `std::isfinite` 检查）。
- 测试组织方式、断言粒度、是否新增独立迁移测试。
- 旧字段反算的代码位置与注释风格。

**不要自行改变**：按轴独立缩放的语义、地面原点固定、迁移公式、保留旧字段用于降级、
默认值 5.10 / 1.49、不新增第三参数、相机旋转行为不变。

## 3. ⚠️ 异常汇报机制

执行过程中如果出现以下情况，**立即停止后续执行并向用户汇报**，不要自行改成别的方案：

- host 配置服务对“同 schema 版本追加可选字段”有额外限制，或 `additionalProperties:false`
  导致新文档无法写入。
- route B 的地面基准无法与锚点换算对齐（例如必须改模型内部才能消除 70 cm 跳变）。
- 默认配置无法做到与当前版本等价（例如模型旋转/姿态本身依赖被删除的偏移）。
- 旧配置无法加载，或降级写回即使夹紧后仍会被旧版本拒绝。
- 构建/测试基线本身不通过，或改动导致超过 2 个非预期测试失败。
- 文档大小接近 `kMaximumSettingsBytes`。

汇报时请给出：冲突点、已尝试方案、可选路径、建议。

## 4. 任务步骤分解

### 🔍 任务组 0：前期探查与基线验证（必须优先执行）

- **难度评级**：⭐ [简单]
- **目标**：确认方案 A 依赖的所有前提在真实代码/宿主中成立。
- **关键位置**：
  - `src/constants.hpp`（schema、默认常量）
  - `src/settings.cpp`（`LoadSettings` / `PersistSettings` / `SettingsValid`）
  - `src/presenter.cpp`、`src/routeb.cpp`、`src/game_tick.cpp`、`src/render_x_probe.cpp`
- **具体行动项**：
  - [ ] 运行基线构建与测试：`./build-mingw.sh --run-tests`，确认 `ctest` 6/6 通过，记录当前产物与版本。
  - [ ] 核对 `Anomaly/src/plugin/scoped_platform_services.cpp` 的 `register_schema` / `read` / `write_atomic`：确认同版本追加可选字段、`additionalProperties:false`、旧文档读取的实际行为。
  - [ ] 估算新增 2 个字段后的 JSON 大小，确认仍远小于 `kMaximumSettingsBytes`。
  - [ ] 用 `grep` 列出所有 `camera_scale` / `origin_offset_cm` / `cameraScale` / `originOffsetCm` 的读写点，确认没有遗漏的消费者（包括测试与诊断）。
  - [ ] 核对 route B 的三套路径（`BuildRouteBPoseFromModel`、`BuildRouteBPoseFromPresenter`、`ReadRouteBDirectCar`）与 `vehicle_anchor_cm` 的语义：确认 `car_cm` 是地面、`vehicle_anchor_cm` 是车身中心、`presenter_car` 是模型原点。
  - [ ] 核对 `ChaseCam::config().rest` 在 presenter/route B 的可读取性，以及预设切换时的重置路径。
  - [ ] 记录 `render_x_probe.cpp` 中所有 `+ kVehicleAnchorCenterHeightCm` 的用途，判断哪些需要保持、哪些需要随地面原点调整。
- **⚠️ 探查汇报**：完成本组后向用户输出“探查完毕”，总结上述结论；若任何一项推翻假设，停止并汇报。

### 📦 任务组 1：常量、schema、设置模型与显示

- **难度评级**：⭐⭐ [中等]
- **目标**：引入新字段和有效值原子量，先把设置层改到可编译、默认值正确。
- **关键位置**：
  - `src/constants.hpp`：`kDefaultCameraScale` / `kMinimumCameraScale` / `kMaximumCameraScale`（约 25–27 行）、`kDefaultOriginOffsetCm` / 范围（约 54–56 行）、`kSettingsSchema`（约 575 行起）。
  - `src/settings_groups.hpp`：`CustomSettings`（约 80–81 行）、`RigOnDefaults`（约 145 行）。
  - `src/state.hpp`：`Context` 原子量（约 1292/1300 行）、`DisplayState`（约 915/923 行）。
  - `src/display.cpp`：`UpdateDisplay`（约 492/508 行）。
- **具体行动项**：
  - [ ] 在 `src/constants.hpp` 增加：
    - `kDefaultCameraDistanceM = 5.10`
    - `kMinimumCameraDistanceM = 3.0`
    - `kMaximumCameraDistanceM = 8.2`
    - `kDefaultCameraHeightM = 1.49`
    - `kMinimumCameraHeightM = 0.45`
    - `kMaximumCameraHeightM = 4.6`
  - [ ] 在 `kSettingsSchema` 增加两个可选 number 属性，范围同上；`required` 和 `additionalProperties` 不动。
  - [ ] `CustomSettings` 用 `camera_distance_m` / `camera_height_m` 替换 `camera_scale` / `origin_offset_cm`；`RigOnDefaults` 比较新默认值。
  - [ ] `Context` 删除 `camera_scale` / `origin_offset_cm`，新增 `camera_distance_m` / `camera_height_m` 原子量；默认值分别为 5.10 / 1.49。
  - [ ] `DisplayState` 删除 `camera_scale` / `origin_offset_cm`，新增 `camera_distance_m` / `camera_height_m`；`UpdateDisplay` 复制新值。
  - [ ] 更新所有因删除旧原子量而编译失败的引用（先修到能编译，行为在后续组完成）。
- **验收**：编译通过；`Context` 默认值是 5.10 / 1.49；`DisplayState` 能读到这两个值。

### 📦 任务组 2：持久化、迁移与旧配置推断

- **难度评级**：⭐⭐⭐ [困难]
- **目标**：新字段可往返；旧字段可推导；降级写回被夹紧；Rig 组的旧配置推断仍正确。
- **关键位置**：
  - `src/settings.cpp`：`SettingsValid`（约 23 行）、`PersistSettings`（约 199 行）、`LoadSettings`（约 597 行）、`ApplyGroup` 的 Rig 分支（约 487 行）、`SetDefaults` / `RestoreDefaults`。
- **具体行动项**：
  - [ ] `SettingsValid` 的参数 `camera_scale` / `origin_offset_cm` 改为 `camera_distance_m` / `camera_height_m`，校验有限性 + 新范围；其余参数不动。
  - [ ] `PersistSettings`：
    - [ ] 从 `custom_settings` 读新字段；
    - [ ] 写 `cameraDistanceM` / `cameraHeightM`；
    - [ ] 按 1.4.2 反算并写 `cameraScale` / `originOffsetCm`（夹紧）；
    - [ ] 继续保持“分组参数写 custom 快照，enable 写有效值”的现有约定。
  - [ ] `LoadSettings`：
    - [ ] 旧字段改为可选读取（缺失用 1.0 / -70），不再因旧字段缺失直接失败；
    - [ ] 读新字段；存在且合法则用，缺失/非法则按 1.4.1 推导；
    - [ ] 推导结果夹紧到新范围；
    - [ ] 用最终值做 `SettingsValid`，写入 `custom_settings`；
    - [ ] Rig 组的旧配置推断（4 个值是否等于默认）使用推导后的距离/高度。
  - [ ] `ApplyGroup` 的 Rig 分支把 `context.camera_distance_m` / `camera_height_m` 写为有效值（Custom 用 custom，Default 用默认）；`aspect_ratio` / `speed_distance_scale` 不变。
  - [ ] `SetDefaults` / `RestoreDefaults` 重置新字段为 5.10 / 1.49。
  - [ ] 检查 `PersistSettings` 里的 `SettingsValid` 调用，确保用新参数；保证反算旧字段始终在旧范围内。
  - [ ] 增加/更新测试（`tests/settings_persistence_test.cpp`）：
    - [ ] 新字段往返；
    - [ ] 只有旧字段的文档 → 新值正确（默认、scale-only、offset-only 三组）；
    - [ ] 只有新字段的文档 → 能加载；
    - [ ] 反算旧字段的夹紧（距离 8.2 / 高度 4.6 等极端组合）；
    - [ ] Cinematic 预设下默认值仍为 5.10 / 1.49；
    - [ ] 旧版本可加载新文档所需的字段齐全性（用现有 `SettingsValid` 模拟）。
- **验收**：旧配置加载后 Rig 组行为正确；新配置重启后新值保留；降级字段始终在旧 schema 范围内；测试通过。

### 📦 任务组 3：presenter / route B / 地面基准

- **难度评级**：⭐⭐⭐⭐ [最困难]
- **目标**：把运行时几何从“统一 scale + 垂直偏移”改成“x/y 用 fx、z 用 fz、地面原点固定”，并保证 route B 与锚点没有 70 cm 跳变。
- **关键位置**：
  - `car_sample.hpp`：`CarSample`（约 42 行）、`BlendCarSample`（约 97 行）。
  - `src/state.hpp`：`RenderXProbe` 的 route B 字段（约 618–619 行）、重置逻辑（约 762–763 行）。
  - `src/game_tick.cpp`：`origin_cm[2] = base_cm[2] + origin_offset_cm`（约 99–107 行）。
  - `src/presenter.cpp`：`PublishRouteBModelPose`（约 129–134 行）、锚点路径（约 278/291/307 行）、`AdvancePresenter` 的缩放（约 466–472 行）、`UpdateCamera` 的读取（约 513–514 行）、样本填充（约 949 行）。
  - `src/routeb.cpp`：`BuildRouteBPoseFromModel`（约 330–390 行）、`BuildRouteBPoseFromPresenter`（约 430–465 行）。
  - `src/render_x_probe.cpp`：锚点换算（约 184–190 行、544/574/628 行）。
- **具体行动项**：
  - [ ] `car_sample.hpp`：
    - [ ] 用 `camera_distance_m` / `camera_height_m` 替换 `camera_scale`；
    - [ ] `BlendCarSample` 对两个新字段做线性插值；
    - [ ] 更新注释。
  - [ ] `src/state.hpp`：
    - [ ] `route_b_model_camera_scale` / `route_b_model_origin_offset_cm` 替换为 `route_b_model_distance_scale` / `route_b_model_height_scale`；
    - [ ] 重置逻辑把两个因子置 1.0。
  - [ ] `src/game_tick.cpp`：
    - [ ] 删除读取 `origin_offset_cm`；
    - [ ] `origin_cm[2] = base_cm[2] - kVehicleAnchorCenterHeightCm`；
    - [ ] 更新注释：`base_cm` 是车身中心，`origin_cm` 固定在地面。
  - [ ] `src/presenter.cpp`：
    - [ ] 增加一个统一计算因子的 helper：`fx = distance / (-config.rest.x)`、`fz = height / config.rest.z`；对结果做有限性检查（必要时夹紧到安全范围）。
    - [ ] `PublishRouteBModelPose` 发布两个因子，删除 origin offset 字段。
    - [ ] `AdvancePresenter`：
      - [ ] `origin = sample.origin_cm`（地面）；
      - [ ] `location[x/y] = origin + (model - origin) * fx`；
      - [ ] `location[z] = origin + (model - origin) * fz`；
      - [ ] 删除 `eye[2] += origin_offset_cm` 一类的逻辑。
    - [ ] 锚点路径三处 `raw_est_cm[2] += origin_offset_cm` 改为 `raw_est_cm[2] -= kVehicleAnchorCenterHeightCm`（把车身中心换到地面）。
    - [ ] `UpdateCamera` 读取新原子量并填入 `sample.camera_*`；删除旧 `camera_scale` 读取。
    - [ ] 保留 `shot_config.camera_scale = 1.0`（烧胎 shot 自己的尺度），但更新注释：presenter 在混合后统一按轴缩放。
  - [ ] `src/routeb.cpp`：
    - [ ] `BuildRouteBPoseFromModel`：
      - [ ] `fresh_car` 改为 proxy 平移（地面），删除 `+ kVehicleAnchorCenterHeightCm`；
      - [ ] 读两个因子；`eye[x/y] = fresh_car + model_offset * fx`，`eye[z] = fresh_car + model_offset * fz`；
      - [ ] 删除 `eye[2] += origin_offset_cm`；
      - [ ] 因子做有限性检查，不再用旧的 0.6–1.6 夹紧（否则高度 0.45 会被抬到 0.6）。
    - [ ] `BuildRouteBPoseFromPresenter`：
      - [ ] 删除 `current_offset - kDefaultOriginOffsetCm` 的重新补偿；
      - [ ] 只保留 `eye += fresh_car - presenter_car` 的平移；
      - [ ] 更新注释。
    - [ ] 检查 `ReadRouteBDirectCar` 的注释：“without the anchor centre height” 仍然成立。
  - [ ] `src/render_x_probe.cpp`：
    - [ ] 逐一核对 `parent_cm[2] += kVehicleAnchorCenterHeightCm` 与 `rendered_cm + height` 的语义；确认 `vehicle_anchor_cm` 仍是车身中心；
    - [ ] presenter 侧的 `-70` 换算与之配对，不要重复或遗漏；
    - [ ] 如果某个字段要改成地面基准，必须同时改它的消费者和测试。
  - [ ] 更新 `tests/judder_repro.cpp`（`sample.camera_scale = 1.0` 改为新字段）以及所有编译失败处。
- **验收**：
  - 默认配置下，`eye` 与当前版本一致（NFS 5.10/1.49）。
  - 距离/高度滑杆在静止时精确等于读数。
  - route B 三条路径没有 70 cm 跳变；锚点来源切换时不漂移。
  - 烧胎 shot 的混合结果仍按用户机位缩放一次。
  - 进入游戏实测通过（见任务组 5 的清单）。

### 📦 任务组 4：UI、本地化与文档

- **难度评级**：⭐ [简单]
- **目标**：把两个滑杆、摘要、状态行和 README 改成新语义。
- **关键位置**：
  - `src/ui.cpp`：`DrawRigGroup`（约 827–850 行）、状态行绘制。
  - `locales/zh-CN.json`。
  - `README.md` 的“设置面板”和字段说明。
- **具体行动项**：
  - [ ] `DrawRigGroup` 的 4 个滑杆改为：
    - `setting.camera_distance` / `"Camera distance at rest (m)"` / `chase-camera-distance` / `camera_distance_m` / 3.0–8.2；
    - `setting.camera_height` / `"Camera height at rest (m)"` / `chase-camera-height` / `camera_height_m` / 0.45–4.6；
    - 保留 `setting.speed_distance` 与 `setting.aspect_ratio`。
  - [ ] 摘要行改为“距离、高度、视口宽高比与速度距离倍率”。
  - [ ] 状态行增加“相机高度”实时值（与现有“镜头-车辆距离”并列）；如果空间不够，至少保留距离行并在 README 说明高度是静止值。
  - [ ] `locales/zh-CN.json`：
    - [ ] 删除 `setting.camera_scale`、`setting.origin_offset`；
    - [ ] 新增 `setting.camera_distance`、`setting.camera_height`；
    - [ ] 更新 `setting.rig_summary`；
    - [ ] 如加状态行，新增对应文案。
  - [ ] `README.md`：
    - [ ] 字段表改为两个米制字段；
    - [ ] 说明“静止”的含义：速度/漂移时实际距离和高度会变化；
    - [ ] 给出迁移公式与降级说明；
    - [ ] 说明 `speedDistanceScale` 仍然存在，且与 `cameraDistanceM` 职责不同。
  - [ ] 更新 `tests/settings_panel_test.cpp`：
    - [ ] Rig 组仍是 4 个滑杆；
    - [ ] id 改为 `chase-camera-distance` / `chase-camera-height`；
    - [ ] 拖动距离只改 `camera_distance_m`，高度不变；
    - [ ] 拖动高度只改 `camera_height_m`，距离不变；
    - [ ] 默认值断言 5.10 / 1.49；
    - [ ] 摘要文本断言更新。
- **验收**：面板显示两个米制滑杆；拖动一个轴不会改变另一个轴；中英文案完整；README 能解释“静止”和迁移。

### 📦 任务组 5：构建、测试与交付验证

- **难度评级**：⭐⭐ [中等]
- **目标**：全量构建、测试、打包，并给出可执行的游戏内验收清单。
- **具体行动项**：
  - [ ] 运行 `./build-mingw.sh --fresh --run-tests`（或至少 `--run-tests`），确认编译与测试通过。
  - [ ] 运行 `ctest`，确认 6/6（或包含新增用例后的实际数量）通过。
  - [ ] 确认 `plugin.dll` 的导入/导出校验通过，`build.mingw/package/robineatcorn.nfs2015-chase-cam/` 内容完整。
  - [ ] 检查 `locales/zh-CN.json` 是合法 JSON。
  - [ ] 用两份手工 JSON 验证：
    - [ ] 旧版默认（scale=1, offset=-70）→ 加载为 5.10/1.49；
    - [ ] 旧版 scale-only（scale=0.8, offset=-70）→ 4.08/1.192；
    - [ ] 旧版 offset-only（scale=1, offset=0）→ 5.10/2.19；
    - [ ] 新版极端值（8.2/4.6）→ 写回的旧字段被夹紧且能在旧版加载。
  - [ ] 输出游戏内验收清单：
    - [ ] 默认 NFS：画面与 2.0.4 默认一致（对比截图/距离读数）。
    - [ ] 拖动“距离”：车在画面中前后移动，状态行高度不变。
    - [ ] 拖动“高度”：车在画面中上下移动，状态行距离不变。
    - [ ] 加速/漂移：滑杆值不变，实时距离/高度按预期变化；没有 70 cm 跳变。
    - [ ] route B 开/关、锚点来源切换：相机与车的相对位置不跳变；坡道上不上下抖动。
    - [ ] Cinematic 预设：默认 5.10/1.49，动态幅度约小 2%–4%，可接受。
    - [ ] 旧配置加载后行为与旧版一致（scale-only 完全一致；offset-only 仅垂直动态略有差异）。
- **验收**：构建、测试、打包、游戏内清单全部通过，或明确记录未通过项与原因。

## 5. 验收标准（Definition of Done）

### 5.1 功能

- [ ] 静止时 `cameraDistanceM` / `cameraHeightM` 就是眼睛相对地面的位置，NFS 与 Cinematic 都精确。
- [ ] 距离滑杆只改变 x/y，高度滑杆只改变 z；两个轴互不影响。
- [ ] 速度/漂移/加速/坡度的水平增量乘 `fx`、垂直增量乘 `fz`。
- [ ] 默认 5.10 / 1.49 在 NFS 下与当前版本一致。
- [ ] 模型旋转行为不变（不重新瞄准）。
- [ ] 地面原点固定为 `base - 70`，用户不再需要配置 `originOffsetCm`。

### 5.2 兼容

- [ ] 旧文档（只有 `cameraScale` / `originOffsetCm`）能加载并推导出新值。
- [ ] 只调过 scale 的旧配置：`fx = fz = scale`，静态与动态完全等价。
- [ ] 只调过 offset 的旧配置：静态高度等价；垂直动态按新高度比例缩放，已在文档说明。
- [ ] 新文档同时写新旧字段；旧字段被夹紧在旧范围内。
- [ ] 旧版本能加载新文档；极端组合的静态高度允许近似。
- [ ] schema 版本仍为 1，`required` 与 `additionalProperties` 未变。

### 5.3 UI

- [ ] 机位几何组仍是 4 个滑杆：距离、高度、速度距离、宽高比。
- [ ] 标签带“静止”，单位是米，默认显示 5.10 / 1.49。
- [ ] 摘要、README、locale 同步更新。
- [ ] 可选：状态行显示实时相机高度。

### 5.4 测试与构建

- [ ] `settings_groups_test`、`settings_persistence_test`、`settings_panel_test` 更新并通过。
- [ ] `chase_cam_test`、`burnout_test`、`wheel_steer_test` 不回归。
- [ ] 新增迁移/夹紧/双轴独立用例。
- [ ] `./build-mingw.sh --run-tests` 通过；包内容完整。
- [ ] 不修改版本号。

## 6. 风险与待验证项

| 风险 | 影响 | 缓解 |
|---|---|---|
| route B 地面基准（+70/−70）换算错误 | 相机整体高/低 70 cm，或锚点切换跳变 | 任务组 0 全量核对；任务组 3 逐路径修改；游戏内实测 route B 开/关与锚点来源切换 |
| 反算旧 `originOffsetCm` 超出 ±150 | 旧版本拒绝整份配置 | 持久化时夹紧；测试极端组合；文档说明降级近似 |
| 默认 Cinematic 基础姿态被绝对值覆盖 | 默认构图差 10 cm / 6 cm；动态小 2%–4% | 接受并写入文档；不引入预设偏移 |
| 旧 0.6–1.6 夹紧残留 | 高度 0.45 被抬到 0.6，滑杆撒谎 | 删除 presenter/route B 对因子的旧范围夹紧；只做有限性检查 |
| 模型自身夹紧 `x ≤ -1.6`、`z ≥ 0.45` | 极端滑杆值在动态中被静默夹紧 | 滑杆范围已避开静态夹紧；文档说明动态极端情况 |
| 删除旧原子量导致测试/诊断遗漏 | 编译失败或行为回退 | 任务组 0 全量 grep；任务组 1 先修编译；CI 全量测试 |
| `speedDistanceScale` 与 `fx` 叠加后幅度过大 | 速度/加速时相机过远 | 文档说明两者叠加；默认值不变；必要时后续单独调 `speedDistanceScale` |
| 烧胎 shot 的 `camera_scale` 与新因子混淆 | 烧胎画面缩放两次或一次 | 保留 `shot_config.camera_scale = 1.0`；presenter 在混合后只按轴缩放一次 |
| 文档/配置大小超限 | 写入失败 | 任务组 0 估算；字段都是标量，风险低 |

## 7. 附：关键代码位置速查

| 文件 | 位置/函数 | 用途 |
|---|---|---|
| `src/constants.hpp` | 25–27、54–56 | 旧 scale / offset 常量与范围 |
| `src/constants.hpp` | 约 575 行起 | `kSettingsSchema` |
| `src/settings_groups.hpp` | 约 80–81、145–146 | `CustomSettings`、`RigOnDefaults` |
| `src/settings.cpp` | 23、199、487、597 | `SettingsValid`、`PersistSettings`、`ApplyGroup`、`LoadSettings` |
| `src/state.hpp` | 约 618–619、762–763、915/923、1292/1300 | route B 字段、重置、`DisplayState`、`Context` |
| `src/display.cpp` | 约 492/508 | `UpdateDisplay` 复制 |
| `src/game_tick.cpp` | 约 99–107 | `origin_cm` 计算 |
| `src/presenter.cpp` | 约 129–134、278/291/307、466–472、513–514、949 | 发布、锚点、缩放、采样 |
| `src/routeb.cpp` | 约 330–390、430–465 | `BuildRouteBPoseFromModel`、`BuildRouteBPoseFromPresenter` |
| `src/render_x_probe.cpp` | 约 184–190、544/574/628 | 锚点/渲染高度换算 |
| `src/ui.cpp` | 约 827–850 | `DrawRigGroup` |
| `car_sample.hpp` | 约 42、97 | `CarSample`、`BlendCarSample` |
| `camera_rig.hpp` | `CameraRigFactorsFor` | 纯函数：每轴因子与静态等效 |
| `chase_cam.hpp` | 312/391、539 | 预设 `rest`、`config()` |
| `tests/settings_groups_test.cpp` | Rig 组用例 | 有效值/默认值 |
| `tests/settings_persistence_test.cpp` | 持久化用例 | 往返/迁移/夹紧 |
| `tests/settings_panel_test.cpp` | `CheckRigDetails` | 滑杆 id/数值/摘要 |
| `tests/chase_cam_test.cpp` | `CheckCameraRigFactors` | 默认/Cinematic/按轴因子 |
