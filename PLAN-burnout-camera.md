# 烧胎特写相机实现计划（NFS 2015 WheelSpin Camera）

> 目标仓库：`nfs2015-chase-cam`
> 目标游戏：NTE（`HTGame.exe`）
> 写作日期：2026-11-10
> 状态：**Phase 4 已完成（PASS，未发布）**
> Phase 0 报告：[`REPORT-burnout-phase0.md`](REPORT-burnout-phase0.md)
> Phase 1 报告：[`REPORT-burnout-phase1.md`](REPORT-burnout-phase1.md)
> Phase 2 报告：[`REPORT-burnout-phase2.md`](REPORT-burnout-phase2.md)
> Phase 3 报告：[`REPORT-burnout-phase3.md`](REPORT-burnout-phase3.md)
> Phase 4 报告：[`REPORT-burnout-phase4.md`](REPORT-burnout-phase4.md)
> 原则：**只读游戏目录；所有分析副本放在当前项目内；不写游戏目录。**

---

## 0. 结论（TL;DR）

1. **NFS 2015 确实有烧胎特写镜头。** 它不是独立的过场，而是 Action Camera 内部的一个相机 mode。`Race/Cameras/ChaseCameraPrefab` 里明确存在：
   - `WheelSpin`
   - `LeftWheelSpin`
   - `RightWheelSpin`
   - 以及低速绕车用的 `LowSpeed`
2. 三个 WheelSpin mode 的镜头参数几乎就是我们要抄的东西：
   - 水平 FOV **45°**；
   - 机位在车尾侧后方约 **±1.9 m 外侧、0.4 m 高、2.2 m 后**；
   - 看向后轮/后轮接地点；
   - 进入：**0.5 s 延迟 + 1.0 s orbit blend**；
   - 退出：**0.2 s 延迟 + 2.5 s orbit blend**；
   - 进入后叠加明显的轮胎/引擎震动。
3. 玩家实测证据与资产一致：NFS 2015 的 Action Camera 在同时踩刹车/手刹和油门做 burnout 时会改变角度，镜头会先拉近到车轮，然后开始抖。
4. **不要把 `CameraLocalDonutOffset` / `DonutScale` 当成烧胎特写。** 那是低速甜甜圈/静止绕车的连续偏移，不是切到后轮的特写。真正要抄的是 `WheelSpin` 系列 mode。
5. 本计划把烧胎特写做成 `nfs2015-chase-cam` 内部的一个**可平滑进入/退出的 shot 状态**，与现有 chase 相机共用同一条 render-clock / route B 路径。不是硬切，不是第二个相机插件。

---

## 1. NFS 2015 资产调查结果

### 1.1 相机 mode（直接证据）

本地分析副本：

```text
nfs2015-game-files/extracted/race/cameras/chasecameraprefab.txt
```

`CinebotCameraModeEntityData` 里的 mode：

| ModeName | Priority | FOV | DefaultTransition | OutTransition | 备注 |
|---|---:|---:|---|---|---|
| `LeftWheelSpin` | 10 | 45 | delay 0.5 s / 1.0 s | delay 0.2 s / 2.5 s | 左侧后轮特写，优先级最高 |
| `RightWheelSpin` | 5 | 45 | delay 0.5 s / 1.0 s | delay 0.2 s / 2.5 s | 右侧后轮特写 |
| `WheelSpin` | 0 | 45 | delay 0.5 s / 1.0 s | delay 0.2 s / 2.5 s | 通用 wheelspin fallback |
| `LowSpeed` | 0 | 45 | delay 0.0 s / 2.0 s (cubic orbit) | delay 0.0 s / 1.0 s | 低速绕车，不是烧胎特写 |

Transition 的 blend 类型是 `CINEBOT_BLEND_EULER_ORBIT`，ease 控制约为 `0.6 / 0.4`。这解释了 NFS 的观感：镜头不是直线推过去，而是绕着车尾侧方划过去。

### 1.2 机位与构图（从 transform 节点推断）

`ChaseCameraPrefab` 里有一组 `TransformMultiplierEntityData`，其中左右轮特写的位置为：

```text
右侧：trans = ( 1.9, 0.4, -2.2)
左侧：trans = (-1.9, 0.4, -2.2)
```

NFS local 坐标约定：`x` 横向、`y` 向上、`z` 向前。因此换到 UE 车体局部坐标（`+X` 前、`+Y` 右、`+Z` 上）是：

```text
eye_local = (X = -2.2, Y = side * 1.9, Z = 0.4)
```

transform 的 `forward` 向量约为 `(±0.621, 0.166, -0.766)`（左侧镜像）。从数值和构图看，它描述的是从目标指向机位的臂方向，镜头实际看向车内/车头方向，落点在后轮附近；不是看向车外。

> 结论：**车尾侧后方、略高于轮心、看向后轮/接地点**。精确的 look-at 点不需要逐位复刻，按下面的构图参数实现即可。

### 1.3 镜头参数

- **FOV：45° 水平**。
  - NFS 的 `MinFieldOfView 70 / MaxFieldOfView 95` 是水平 FOV，换算到 16:9 才是 43.0 / 63.1 垂直。因此 `WheelSpin` 的 45 也应理解为**水平 45°**，在 16:9 下约 **26.5° 垂直**，是很紧的长焦特写。
- **Roll：0**。
- **OcclusionRadius / ClipTestRadius：0.1 / 0.4 m**（左右轮 mode）。说明 NFS 对特写镜头有基本的遮挡/穿模测试。
- **震动**：WheelSpin mode 接入了多个 `ImpulseController` / `NoiseController`。玩家描述是“镜头拉近到车轮后开始抖”，与资产结构一致。

### 1.4 触发与观感（玩家证据）

- NFS 2015 的 Action Camera 在 burnout 时会改变角度；玩家描述：同时刹车/手刹和油门做 burnout，镜头会先 zoom 到车轮，然后 shake。
- 同一台相机在 5–25 mph 低速巡航时会绕车旋转，这是 `LowSpeed` / donut 逻辑，不是 WheelSpin 特写。
- 因此实现时要区分：
  - **烧胎特写**：车静止 + 手刹 + 油门 → 后轮 close-up；
  - **低速绕车**：车在低速移动 → 连续 orbit，不需要切 mode。

参考：

- Reddit: <https://www.reddit.com/r/needforspeed/comments/e886s3/apparently_in_nfs_201516_if_you_cruisea_at/>
- EA Forums（Action Camera 与 burnout 角度变化）: <https://forums.ea.com/discussions/need-for-speed-franchise-discussion-en/re-wheres-the-action-camera-feature-from-nfs-2015-why-isnt-it-in-payback-/9407702>

### 1.5 哪些能抄，哪些不能直接抄

**能直接抄：**

- 后轮特写的机位比例（外侧 1.9 m、高 0.4 m、后 2.2 m）；
- 水平 FOV 45°；
- 进入 0.5 s 延迟 + 1.0 s orbit blend；
- 退出 0.2 s 延迟 + 2.5 s orbit blend；
- 左右轮分别取景 + 侧别 latch；
- 拉近后的震动。

**不能直接抄：**

- `CameraLocalDonutOffset` / `DonutScale`：这是低速甜甜圈/静止绕车的连续偏移，不是 WheelSpin 特写。
- 游戏 executable 里的 `CloseupCameraPosition` / `OrbitCameraPosition` 字段：它们和车辆展示/菜单相机同组，不是 gameplay 烧胎镜头。
- 固定的世界坐标/固定车型尺寸：NFS 的这个 prefab 是全局数据，实际游戏可能还有 per-car 调整；NTE 的车宽/轴距不同，第一版用固定 NFS 比例 + 可调参数，后续再按包围盒微调。

---

## 2. NTE 目标行为

### 2.1 触发条件

用户定义：

> 车静止时，手刹 + 前方向 = 烧胎。

实现上使用“信号 + 滞回”，不要用单帧布尔：

```text
burnout_condition =
    on_ground
    && |forward_speed| < 1.0 m/s          // 静止
    && forward_input > 0.15               // 前方向/油门
    && handbrake_pressed                  // 手刹
```

同时允许 NTE 自己的 burnout 标志作为更强证据：

```text
burnout_condition = direct_burnout_flag || (speed/handbrake/throttle fallback)
```

推荐优先级：

1. `GetVehicleIsBurnout()`（如果可用）——最直接；
2. 每轮 `GetReplicatedWheelBurnout()` / `GetWheelIsBurnoutRep()`（如果可用）——还能选左右轮；
3. `GetVehicleThrottle()` + `HTVehicleInputInfo.bHandbrakeInput` + 速度门；
4. 全部不可用：功能自动禁用并在诊断面板显示“burnout signal unavailable”，**不要猜**。

### 2.2 侧别选择

- 如果每轮 burnout 标志可用：选择后轮中 burnout 更强/为 true 的一侧。
- 否则用 `GetRawSteeringInput()` 的符号：
  - 左打方向 → `LeftWheelSpin`；
  - 右打方向 → `RightWheelSpin`；
  - 回正 → 保持上一次侧别；第一次默认右侧（可配置）。
- **进入特写后 latch 侧别**，不要因为手抖/方向盘微动在左右镜头之间跳。
- 只有在 shot 完全退出（blend 回到 0）后，下一次烧胎才重新选边。

### 2.3 速度/状态安全门

- 进入：`|speed| < 1.0 m/s`。
- 保持：`|speed| < 3.0 m/s`；超过则强制快速退出（0.4–0.6 s），不要等 2.5 s。
- 腾空、离开载具、切世界、传送：立即重置 shot 状态。
- 倒车/反向：不触发（`forward_speed < -0.5 m/s` 直接退出）。

---

## 3. 与现有插件的结合点

当前链路（不要破坏）：

```text
Game tick
  UpdateCamera()
    QueryPlayer / QueryVehiclePresence / QueryVehicleSpeed
    组 CarState
    写 CarSample 到 SampleRing

Render / detour
  AdvancePresenter()
    SampleRing.AtSmooth()
    ChaseCam::Update()
    PublishRouteBModelPose()
    scale around origin
    PublishPresenterPose()

  PrepareViewStateForVisibility
    BuildRouteBPoseFromPresenter() 优先
    失败才 BuildRouteBPoseDirect() / BuildRouteBPoseFromCar()
    ApplyRouteBPoseToView()
```

烧胎特写必须**插在 `AdvancePresenter()` 里 `ChaseCam::Update()` 之后、`PublishRouteBModelPose()` 之前**：

```text
chase_view = chase.Update(elapsed, sample.car)
shot_view  = burnout_shot.Update(elapsed, sample)   // 新模块
view       = BlendCameraState(chase_view, shot_view, shot.blend)
PublishRouteBModelPose(sample, view, now_ns)
```

这样：

- 位置、朝向、FOV 仍然只经过现有 route B 写入点；
- 不会出现“相机和车不同帧”；
- 特写和普通 chase 共用同一个 render clock，退出时不会跳回旧 pose；
- 不需要新的 hook，不需要新 capability。

**不要**在 `PrepareViewStateForVisibility` 之后单独写特写 pose；那会破坏裁剪/运动矢量/TAA 一致性。

---

## 4. 数据与检测实现

### 4.1 `CarSample` 扩展

在 `car_sample.hpp` 增加，不要污染 `CarState` 的纯相机模型：

```cpp
struct BurnoutSample final {
  bool active{};        // 当前是否处于烧胎条件
  bool left{};          // 左后轮 burnout
  bool right{};         // 右后轮 burnout
  double intensity{};   // 0..1，平滑后的强度
  double throttle{};
  double handbrake{};
  double steering{};    // -1..1
  double speed_mps{};   // 当前前进速度
};

struct CarSample final {
  // ... existing ...
  BurnoutSample burnout;
};
```

`BlendCarSample()`：

- `active/left/right` 取较新的 sample，不插值；
- `intensity/throttle/handbrake/steering` 线性插值；
- `epoch` 变化时 shot 状态重置。

### 4.2 NTE 查询

不新增服务，复用现有：

- `anomaly.ue5.objects::find_exact`
- `anomaly.ue5.process-event::invoke`
- 现有 `VehicleIsWheeled()` 排除水上载具

Phase 0 已确认的精确函数（全部 12/12 调用成功）：

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

已验证的契约：

- `GetVehicleIsBurnout` → bool；`GetVehicleThrottle/Brake/RawSteeringInput/SkidAngle` → float。
- `GetWheelIsBurnout` / `GetWheelIsBurnoutRep`：uint8 wheel index @0，bool @4。
- `GetWheelAngularSpeed`：uint8 wheel index @0，float @4。
- `GetInputInfo` → 16 字节 `HTVehicleInputInfo` 按值返回：`ThrottleInput` @0、`SteeringInput` @4、`BrakeInput` @8、`bHandbrakeInput` @12。
- `GetWheelInfo`：index，512 字节 `HTVehicleWheelInfo`；`AxleType`、`SideType`、`Radius`、`WorldTransform`、`bIsBurnout` 等字段可用。

注意：

- 旧计划里的 `HTWheeledVehicleBase` / `HTVehicleBase` 路径不正确，Phase 2 直接使用上面的精确路径。
- `invoke` 输出缓冲按已验证的 `parmsSize` 给足；struct 只读已验证的字段偏移。
- `GetVehicleIsBurnout` 可能对漂移/一般 wheelspin 也为 true，所以**必须叠加速度门**。
- 每轮查询只在 `burnoutShotEnabled && vehicle_present && wheeled` 时执行；关闭功能时零额外开销。
- 缓存 handle 必须纳入现有 `ResetVehicleObjectCaches()` / `RefreshVehicleObjectGeneration()` 的 generation 失效逻辑。

建议新增：

```cpp
struct BurnoutSignals final {
  bool known{};
  bool is_burnout{};
  bool left{};
  bool right{};
  bool handbrake{};
  double throttle{};
  double brake{};
  double steering{};
};
```

挂在 `VehiclePresence` 上，和 20 Hz 车速查询共用同一个 poll 窗口，避免额外 20 Hz 开销。

### 4.3 Game tick 侧状态

在 `UpdateCamera()` 里：

```text
raw_active = burnout_signals.is_burnout
             || (throttle > 0.15 && handbrake && |speed| < 1.0)
raw_active &= on_ground && |speed| < entry_speed
raw_active &= forward_speed > -0.5
```

再做短保持（bridge 20 Hz poll gap）：

- `burnout_hold` 0.25–0.30 s：条件短暂丢失时不立刻结束；
- `intensity` 上升 12/s、下降 8/s；
- 侧别在 Game tick 侧只提供 `left/right`，最终 latch 在 render 侧 shot 状态机里做。

这样 Game tick 只发布“事实”，render 侧决定“怎么拍”。

---

## 5. 特写镜头模型

新增 `burnout_shot.hpp`（纯头文件、可单测），不要塞进 `chase_cam.hpp` 的 chase 弹簧逻辑里。

### 5.1 机位

NFS 值换到 UE 车体局部坐标：

```cpp
constexpr double kShotBackM       = 2.2;
constexpr double kShotOutboardM   = 1.9;
constexpr double kShotHeightM     = 0.4;
constexpr double kShotTargetXM    = -0.9;   // 后轮附近，车体局部 X
constexpr double kShotTargetYM    = 0.75;   // 轮距的一半
constexpr double kShotTargetZM    = 0.10;   // 接地点附近
```

```text
eye_local    = (-kShotBackM, side * kShotOutboardM, kShotHeightM)
target_local = (kShotTargetXM, side * kShotTargetYM, kShotTargetZM)
```

用 `CarBasis`（`MakeCarBasis(car.yaw, car.pitch)`）转到世界：

```text
eye_world    = car.position + CarLocalToWorld(eye_local, basis)
target_world = car.position + CarLocalToWorld(target_local, basis)
```

- `slope_follow` 关闭时用 yaw-only basis。
- 车体 pitch 可用时让特写跟着车身俯仰，避免上坡时镜头穿地/穿车。
- 目标点在第一版用固定值；后续可选按 ESP 包围盒的 half-width/half-length 微调。

### 5.2 FOV

```text
horizontal_fov = 45.0
vertical_fov = 2 * atan(tan(45°/2) / aspect_ratio)
```

- 用**水平 FOV**做权威值，避免不同宽高比下构图漂移。
- `CameraState` 内部仍存垂直 FOV；`HorizontalFovDegrees()` 已有换算。
- 如果用户关闭 `fovEnabled`：只切机位，不改游戏 FOV；特写会显得更宽，但不强改用户设置。

### 5.3 震动

不要复用 chase 的 speed shake（车静止时为 0）。新增 `burnout shake`：

```text
shake_gate = SmoothStep(0.55, 1.0, blend)
shake_amp  = 0.010 m * intensity * shake_gate * shake_scale
shake_rot  = 0.20°  * intensity * shake_gate * shake_scale
```

- 进入后延迟约 0.10–0.15 s 再起震，模拟“先 zoom，再抖”；
- 用已有 `Noise1` 的多频正弦，频率 10–25 Hz；
- 位置抖动加在 eye 上，旋转抖动加在 view direction/up 上；
- 上限：位置 ≤ 0.03 m、旋转 ≤ 0.5°，避免晕；
- UI 只给一个“烧胎震动强度”滑杆，默认 1.0。

### 5.4 穿模/地面保护（第一版最小实现）

- eye 的 Z 不低于 `car.position.z + 0.25 m`（车体原点通常在地面附近）；
- 如果 `target` 与 eye 的距离 < 0.5 m，回退到 chase view，避免退化成零向量；
- 左右轮特写沿用 NFS 的 `clipTestRadius 0.4 m` 作为未来碰撞检测的半径；
- 真正的 wall raycast 可以放到 Phase 4；第一版先保证不穿地、不穿车体中心。

---

## 6. 平滑切换（核心）

### 6.1 状态机

在 render 侧维护（`Context` 内，`presenter_lock` 保护）：

```cpp
struct BurnoutShotState final {
  double phase{};          // 0 = 纯 chase，1 = 纯特写
  double hold_time{};      // 条件持续时间
  double inactive_time{};  // 条件消失后的时间
  bool latched{};
  int side{1};             // +1 右，-1 左
  double blend{};          // eased phase
};
```

更新逻辑：

```text
if (active && speed < hold_speed && !force_exit) {
    inactive_time = 0;
    hold_time += dt;
    if (latched || hold_time >= entry_delay) {
        if (!latched) { latched = true; side = SelectSide(); }
        phase = min(1, phase + dt / entry_time);
    }
} else {
    hold_time = 0;
    if (phase > 0) {
        inactive_time += dt;
        if (inactive_time >= exit_delay) {
            phase = max(0, phase - dt / exit_time);
        }
    } else {
        latched = false;
    }
}
blend = Ease(phase);       // SmoothStep / 近似 cubic-bezier(0.6,0.4)
```

关键行为：

- **进入前延迟**：NFS 的 0.5 s。短点一下不会触发，避免日常刹车/起步误切。
- **退出前延迟**：NFS 的 0.2 s。烧胎刚停时不立刻拉走。
- **退出 2.5 s**：慢慢回到 chase，不硬切。
- **退出中再次烧胎**：`phase` 从当前值回升，不重置、不跳变。
- **速度超限**：`force_exit`，用 0.4–0.6 s 快速回 chase。
- **phase 到 0**：才清 `latched` / 允许重新选边。

### 6.2 Orbit blend（不是直线插值）

NFS 的 `EULER_ORBIT` 是绕车转过去。实现：

1. 以车体附近一点为 pivot（建议 `car.position + up * 0.8 m`）。
2. 把 chase eye 和 shot eye 都转成车体局部柱坐标：
   ```text
   r     = hypot(local.x, local.y)
   theta = atan2(local.y, local.x)
   z     = local.z
   ```
3. 对 `r`、`theta`（最短弧）、`z` 做插值。
4. 转回世界坐标。

这样从车后 `theta≈180°` 到右后 `theta≈140°` 是绕着右后翼子板划过去，而不是穿过车尾。

### 6.3 视角/Up/FOV 混合

```text
eye      = orbit_blend(chase.eye, shot.eye, t)
dir      = normalize(lerp(chase_dir, shot_dir, t))   // 或 slerp
up       = normalize(lerp(chase_up, shot_up, t))
up       = normalize(up - dir * dot(up, dir))         // 重新正交化
target   = eye + dir * 1.0
fov_y    = lerp(chase.fov_y, shot.fov_y, t)
roll     = lerp(chase.roll, shot.roll, t)
```

不要直接线性插值 `CameraState.target` 后不归一化；否则中段视线长度变化会导致 pitch/yaw 抖动。

### 6.4 route B 一致性

- `PublishRouteBModelPose()` 使用**混合后的** `view`，因此模型路径拿到的就是特写/回归后的 pose。
- `AdvancePresenter()` 里的 `location/rotation/lens` 也从同一个 `view` 生成。
- 不要在 `PrepareViewStateDetour` 里再判断“是否烧胎”并二次覆盖。
- 如果 `BuildRouteBPoseFromPresenter()` 不可用、回退到 `BuildRouteBPoseFromCar()`，特写可能丢失；第一版接受这个降级，但要在诊断面板计数（`shot_fallback`），Phase 4 再决定是否给 fallback 单独算 shot pose。

### 6.5 `cameraScale` 的处理

现有 `cameraScale` 会把整段 pose 绕模型原点缩放。特写机位本来就离车很近，`cameraScale=0.6` 时可能进入车内。

第一版策略：

- 特写沿用现有 `cameraScale`（默认 1.0，绝大多数用户不受影响）；
- 在 `BurnoutShot` 里给 eye 的横向/后向距离加一个安全下限：`max(1.3 m, 1.9 m * cameraScale)`；
- 如果实机在 0.6–0.8 下仍然穿车，再加独立的 `burnoutShotScale`，在 presenter 层单独缩放，不动 route B 的 scale 路径。

---

## 7. 设置、UI、本地化

### 7.1 设置字段

新增到现有 settings JSON（保持 schema v1，缺省字段用默认值，不强制迁移）：

```json
{
  "burnoutShotEnabled": true,
  "burnoutEntryDelay": 0.5,
  "burnoutEntryTime": 1.0,
  "burnoutExitDelay": 0.2,
  "burnoutExitTime": 2.5,
  "burnoutFov": 45.0,
  "burnoutShake": 1.0,
  "burnoutSideMode": 0,
  "burnoutEntrySpeedMps": 1.0,
  "burnoutHoldSpeedMps": 3.0
}
```

- `burnoutSideMode`：0 = auto，1 = 强制左，2 = 强制右，3 = 居中。
- `burnoutFov` 用水平 FOV 保存；UI 显示“烧胎特写视野（水平）”。
- 所有字段都走现有 `SettingsValid()` / `PersistSettings()` / `LoadSettings()` / `SetDefaults()` 模式；不要引入第三方 JSON。

### 7.2 UI

在设置窗口加一个独立小节：

```text
[✓] 烧胎特写（Action Camera WheelSpin）
    进入延迟      0.5 s
    退出时间      2.5 s
    特写视野      45°
    震动强度      1.0
    侧别         自动 / 左 / 右 / 居中
```

第一版只把 **开关 + 震动强度 + 侧别** 放到主面板；时间/FOV 放在 JSON 或高级区，避免设置窗口膨胀。

### 7.3 本地化

`locales/zh-CN.json` 增加：

```json
"setting.burnout_shot": "烧胎特写（NFS 2015 WheelSpin）",
"setting.burnout_shake": "烧胎镜头震动",
"setting.burnout_side": "烧胎特写侧别",
"burnout.side.auto": "自动",
"burnout.side.left": "左后轮",
"burnout.side.right": "右后轮",
"burnout.side.center": "居中",
"status.burnout": "烧胎",
"status.burnout_blend": "特写混合"
```

诊断面板增加：

- `burnout signal`（direct / fallback / unavailable）
- `burnout active`
- `burnout side`
- `shot blend`
- `shot fallback`（route B 回退次数）

---

## 8. 测试与验收

### 8.1 单元测试（新增 `tests/burnout_shot_test.cpp` 或并入现有）

必须覆盖：

1. `phase` 在 `entry_delay` 前保持 0；
2. 条件持续后 `phase` 单调上升到 1；
3. 条件消失后先保持 `exit_delay`，再单调下降到 0；
4. 退出中途重新触发，`phase` 从当前值回升，无跳变；
5. 侧别 latch：shot 中改变 steering 不换边；完全退出后下一次可换边；
6. 速度超过 `hold_speed` 时快速退出；
7. `blend=0` 时输出与 chase 完全一致；`blend=1` 时输出与 shot 完全一致；
8. FOV 45° 水平在 16:9 下约 26.5° 垂直；
9. 所有输出有限，无 NaN，target 与 eye 距离 > 0.5；
10. 上坡/下坡 pitch 下 shot eye 不穿地（Z 下限）。

### 8.2 集成测试

- `CarSample` 布尔字段不被插值；
- epoch 变化重置 shot；
- route B model pose 与 presenter pose 使用同一 `view`；
- shot 进入/退出期间 `ViewHistoryJumped()` 不触发 snap（< 60°/帧，< 5000 cm/帧）；
- 功能关闭时 `AdvancePresenter` 不调用 burnout 逻辑，开销可忽略。

### 8.3 实机验收

- 静止 + 手刹 + 油门：
  - 约 0.5 s 后平滑绕到后轮；
  - 左/右打方向选择对应侧；
  - 镜头拉近后开始抖；
  - 松开后约 2.5 s 平滑回 chase。
- 漂移、正常行驶、倒车、腾空：不触发。
- 烧胎中轻点方向：不左右跳镜头。
- 烧胎后立刻起步：快速回 chase，不拖 2.5 s。
- 贴墙烧胎：不穿墙（至少不穿地/不穿车体中心；wall clip 作为已知风险记录）。
- 与现有 `cameraScale`、`slopeFollow`、`preset`、F7 开关共存。

---

## 9. 分阶段任务

### Phase 0：NTE 信号确认（0.5–1 天）

> **结果（2026-11-10）：PASS。** 12/12 反射函数解析成功，实机怠速/手刹/烧胎/左右转向日志已确认 direct flag、每轮掩码、手刹 offset 12、转向和车轮角速度。漂移/正常刹车/倒车仍留给 Phase 1/2 的实机矩阵。详见 [`REPORT-burnout-phase0.md`](REPORT-burnout-phase0.md)。

- 在诊断面板/日志中临时打印：
  - `GetVehicleIsBurnout` 是否可解析、返回值；
  - `GetVehicleThrottle` / `GetVehicleBrake` / `GetRawSteeringInput`；
  - `GetInputInfo` 的返回字节；
  - `GetReplicatedWheelBurnout` / `GetWheelIsBurnoutRep` 的调用方式。
- 实机做：静止 + 手刹 + 油门、漂移、正常刹车、倒车。
- 产出：一份信号表，确认哪个组合能稳定区分“烧胎”与其他状态。
- **失败即停**：如果所有信号都不可用，功能默认关闭，不进入 Phase 1 之后的集成。

### Phase 1：纯相机模型（1–2 天）

> **结果（2026-11-10）：PASS。** `burnout_shot.hpp` 已实现状态机、机位、水平 FOV、震动、orbit blend 和安全下限；`tests/burnout_shot_test.cpp` 覆盖计划中的 10 项验收，CTest 全通过。详见 [`REPORT-burnout-phase1.md`](REPORT-burnout-phase1.md)。

- 新增 `burnout_shot.hpp`：机位、FOV、震动、orbit blend、状态机。
- 新增单测，不接插件。
- 产出：给定 `CarState + BurnoutSample + dt` 可得到稳定的 shot view 和 blend。

### Phase 2：检测与 sample 管线（1–2 天）

> **结果（2026-11-10）：PASS。** `BurnoutSignals` / `BurnoutDetectionState` 已接入 20 Hz probe；`CarSample::burnout` 随 sample ring 发布，`BlendCarSample()` 对布尔取新样本、连续值插值；诊断面板可见检测与样本。构建、CTest 与 `validate` 通过。实机 `active=1` 因 UIPI 输入注入受限，待手动 Shift+W 确认。详见 [`REPORT-burnout-phase2.md`](REPORT-burnout-phase2.md)。

- 扩展 `CarSample` / `BlendCarSample`（没有扩展 `VehiclePresence`）。
- 在 `QueryVehiclePresence` 的 20 Hz 窗口内查询 burnout 信号。
- 在 `UpdateCamera` 组 sample 时填 `burnout`。
- 缓存/失效/退避纳入现有 generation 机制。
- 产出：诊断面板能看到 burnout 状态，但相机还未使用。

### Phase 3：presenter / route B 集成（1–2 天）

> **结果（2026-11-10）：PASS。** `AdvancePresenter` 已在 `chase.Update` 后推进 `BurnoutShot`，`BlendBurnoutCamera` 的混合 view 同时用于 route B、presenter pose 和 lens；epoch/reset/inactive 会重置 shot。实机用高完整性 `PostMessage` 触发 Shift+W，截图确认 chase → 后轮特写 → chase，route B `fallbacks=0`。详见 [`REPORT-burnout-phase3.md`](REPORT-burnout-phase3.md)。

- 在 `AdvancePresenter` 里 `chase.Update` 后调用 shot，混合 `view`。
- 保证 `PublishRouteBModelPose` / presenter pose / lens 全部使用混合后的 view。
- 处理 epoch、reset、inactive、vehicle identity 变化。
- 产出：实机可看到特写，先不做 UI。

### Phase 4：设置/UI/本地化/收尾（1 天）

> **结果（2026-11-10）：PASS（未发布）。** 11 个 burnout 设置字段已加入 schema/持久化/加载/默认值；设置面板提供开关、FOV、震动、进入/退出/起步回正时间、速度和侧别；诊断显示 shot blend/side/fallback。旧 31 字段配置可加载，`burnoutEnabled=false` 实机验证保持 chase；全量构建、CTest 通过。详见 [`REPORT-burnout-phase4.md`](REPORT-burnout-phase4.md)；起步回正的 NFS 证据见 [`REPORT-burnout-launch-handoff.md`](REPORT-burnout-launch-handoff.md)。

- 设置字段、持久化、UI 开关/滑杆、zh-CN 文案。
- 诊断计数、日志、README/RELEASE-NOTES。
- 全量单测 + 实机矩阵 + 性能检查（20 Hz 查询不增加明显 tick 开销）。
- 产出：先不发布 。

---

## 10. 风险与开放问题

| 风险 | 影响 | 处理 |
|---|---|---|
| 反射函数路径/签名与旧计划不符 | 无法读取 burnout/handbrake | **已解决**：Phase 0 确认精确路径与字段偏移，见报告 |
| `GetVehicleIsBurnout` 在漂移时也为 true | 误触发 | 叠加速度门 + 手刹/油门确认 |
| 20 Hz 检测延迟 | 特写晚 50 ms | 0.5 s 进入延迟掩盖；sample hold 0.25 s |
| 贴墙/贴地穿模 | 镜头进入几何体 | 第一版 Z 下限 + 距离下限；wall raycast 后续 |
| `cameraScale` 很小时镜头进车内 | 构图破坏 | 第一版安全下限；必要时加独立 shot scale |
| 特写退出太慢影响驾驶 | 起步时视野被占 | 速度超限走 1.8 s 起步回正；异常状态 0.5 s |
| 左右侧别抖动 | 镜头左右跳 | 进入后 latch，退出归零才允许换边 |
| FOV 45 在超宽屏过窄 | 车占满画面 | 用水平 FOV 权威值；必要时按 aspect 限制最小垂直 FOV |

**Phase 0 已确认 / 剩余问题：**

1. `GetVehicleIsBurnout` 在静止手刹+油门烧胎时为 true，怠速为 false；漂移/一般 wheelspin 尚未单独采样，仍应保留速度门。
2. `GetInputInfo` 是返回值，不是 out 参数；`HTVehicleInputInfo` 是 16 字节，字段为 `ThrottleInput` @0、`SteeringInput` @4、`BrakeInput` @8、`bHandbrakeInput` @12。
3. `GetWheelIsBurnout` / `GetWheelIsBurnoutRep` 需要 wheel index；本车后轮为 2、3，烧胎掩码 `0x0c`；两后轮可能同时为 true，侧别优先看 steering，必要时用 `GetWheelInfo::SideType`。
4. 实测烧胎为“手刹 + 油门”；手刹按下但未给油时 `isBurnout=0`。fallback 应要求 handbrake && throttle，不要只凭油门。
5. 用户是否希望烧胎特写默认开启？建议默认开启，但 UI 明显可关。

---

## 附录 A：NFS 2015 关键数值速查

```text
Camera modes: LeftWheelSpin / RightWheelSpin / WheelSpin
FOV: 45.0 (horizontal)
Eye local (NFS): (+/-1.9, 0.4, -2.2)   // x lateral, y up, z forward
Entry: delay 0.5 s, blend 1.0 s, EULER_ORBIT, ease 0.6/0.4
Exit:  delay 0.2 s, blend 2.5 s, EULER_ORBIT, ease 0.6/0.4
Clip:  OcclusionRadius 0.1 m, ClipTestRadius 0.4 m
```

本地证据：

```text
nfs2015-game-files/extracted/race/cameras/chasecameraprefab.txt
  - CinebotCameraModeEntityData 约 5995–6145 行
  - CinebotTransitionData 约 6729–6808 行
  - TransformMultiplierEntityData 约 4990–5160 行
```

## 附录 B：不要碰的东西

- 不要修改 `D:\Application\Steam\steamapps\common\Need for Speed` 下的任何文件。
- 不要用 `CameraLocalDonutOffset` 代替 WheelSpin 特写。
- 不要在 `PrepareViewStateForVisibility` 之后二次写相机 pose。
- 不要为特写新增第二套 hook；复用现有 `AdvancePresenter` / route B。
- 不要在没有 Phase 0 证据的情况下硬编码 NTE 函数返回值布局。
