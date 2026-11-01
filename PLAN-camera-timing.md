# 相机时序重构计划（临时文件）

> **这是临时工作文档，不参与发布。** 全部任务组完成（或整体放弃）后删除本文件；
> 不要提交进 `main`。行号以当前 HEAD `4d0639b` 为准，改动后会漂移，用函数名定位。

---

## 0. 背景

现状（症状）：

- 漂移时相机有跳变、不流畅；
- CPU 被抢占（游戏 tick 掉到 ~30 Hz）时跳变加剧，且转向手感变"粘滞"。

诊断结论（代码层面已确认，非猜测）：

1. **模型跑在 game tick，画面取在 render 调用**，中间靠 `view_blend.hpp` 补缝；而
   `plugin.cpp:1207` 在 `update_hz < 57` 时把 `interpolation_delay` 置 0，导致 alpha 恒为 1，
   **插值在低帧率下完全空转** → 输出是 30 Hz 阶梯。
2. **车头朝向是"二选一"而非"估计"**（`SelectCarYaw`，`plugin.cpp:2051`）：比较游戏相机 yaw 与
   速度方向，超过 80° 就整体换成速度 yaw；无迟滞，且观测来自另一线程（`ReadObservedPose`
   读失败会静默换源）。漂移时两者夹角天生接近阈值 → 每 tick 翻转 → `car_yaw` 阶跃被
   6 rad/s 限速拉成锯齿。
3. **模型对 `dt` 敏感**：所有平滑器是 `1-exp(-rate·dt)`，dt 变大时单步滤波强度退化
   （`roll_spring=60` 在 20 Hz 下每 tick 只吃掉 5% 误差），噪声直接穿透。
4. **帧同步用错对象**：`sync_delta`（`plugin.cpp:1255`）用游戏相机的帧间位移补偿车架错帧；
   漂移时游戏相机侧摆远大于车体位移 → 注入横向跳动。
5. **渲染路径可能整体让位**：`if (leased && active)`（`plugin.cpp:1111`）在拿不到 hook lease 时
   直接退回游戏相机 → 一帧透出，下一帧切回。
6. **game tick 上有 O(n) 反射**（按属性名读 `component_bounds`、翻页、每 2 s 最多 4096 条扫描），
   抢占游戏自身 tick 预算 → 手感粘滞。

结论：**问题在管道形状，不在参数。** 打补丁只能治症。

---

## 1. 目标 / 非目标

### 目标

- **G1** 相机行为不依赖游戏 tick 率：同一次驾驶在 20/30/60/144 Hz 下重放，输出在共同时间轴上
  一致（RMS 差 ≤ 2 cm、0.5°）。
- **G2** 除显式不连续事件（换世界、重生、玩家实体换代）外，相邻帧位姿满足连续性上界
  `Δangle ≤ kMaxAngularRate·dt`、`Δpos ≤ kMaxLinearRate·dt`。稳态驾驶 `snap_count == 0`。
- **G3** 漂移段：`yaw_source_switches == 0`，`drift_` 无逐 tick 翻转，roll 无 ±clamp 削顶抖动。
- **G4** game tick 内 `UpdateCamera` 成本 p99 < 1.5 ms（当前未测，T0 建立基线）。
- **G5** 上述全部由离线测试守护（CI 内可执行，不需要游戏）。

### 非目标（明确不做）

- 不做镜头碰撞 / 防穿墙（保持现状）。
- 不改 `chase_cam.hpp` 的核心方程与参数语义（只改输入质量与调度），除 T5 的 roll 输入替换。
- 不引入 GObjects 扫描 / `ProcessEvent`（保持纯 C ABI 约定）。
- 不追求"更像 NFS 2015"的观感调参——本轮只做时序与稳定性。

---

## 2. 设计原则（四条，所有任务组必须遵守）

1. **单一时钟。** 相机是有状态的积分器，只在一个线程、一个时钟上推进（render 调用）。
   测量在 game tick 采样，但以带时间戳的样本形式进入。
2. **输入即样本。** 任何进入模型的量都必须带 `time_ns`、`tick`、`generation`、有效性位。
   没有时间戳的量禁止进模型。
3. **估计，不选择。** 慢变量（slip、锚点）用带创新门控的滤波器维护，不允许"本 tick 读不到就换源"。
4. **不连续必须显式。** 只有 `generation` 变化允许重置模型；禁止用 `snap` 旗标绕过连续性。

---

## 3. 任务组总览

| ID | 任务组 | 工作量 | 波次 | 风险 | 依赖 | 主要收益 |
|----|--------|--------|------|------|------|----------|
| T0 | 仪表与基线 | 0.5 d | 第一波（快赢） | 无 | — | 用数据定主因 |
| T1 | Presenter 契约 | 0.5 d | 第一波（快赢） | 低 | T0 | 消除"透出游戏相机"的跳变 |
| T2 | 单时钟 presenter | 3.0 d | 第二波（结构） | **高** | T0, T1 | G1、阶梯、sync_delta、dt 退化 |
| T3 | 测量层（slip / 锚点） | 2.0 d | 第二波（结构） | 中 | T0 | G3、漂移跳变 |
| T4 | Tick 成本 | 1.0 d | 第三波（收尾） | 低 | T2, T3 | G4、手感粘滞 |
| T5 | 模型质量 | 1.0 d | 第三波（收尾） | 中低 | T2 | G3 残余抖动 |
| T6 | 不变量与 CI | 2.0 d | 第三波（收尾） | 低 | T2, T3 | G5，回归防线 |
| T7 | 清理与文档 | 0.5 d | 第三波（收尾） | 低 | 全部 | 可维护性 |

合计约 **10.5 人日**（单人 2–2.5 周，含调试验证）。T0/T1 可当天上线。

---

## 4. 任务组详情

### T0 — 仪表与基线（0.5 d，零行为变更）✅ 代码已完成，待一局数据

**动机**：现在所有结论都靠读代码推断，需要一局数据确认主因排序。

**改动点**：`plugin.cpp` 的 `Diagnostics` / `DisplayState` / `Context` / `UpdateCamera` /
`Update` / `Draw`；新增 `latency_window.hpp`；`tests/chase_cam_test.cpp` 补测试。

**步骤**

1. `Context` 增加计数器（`std::atomic<uint64_t>`，只增不减）：
   - `yaw_source_switches` — `SelectCarYaw` 每次换源（含 `ReadObservedPose` 失败退回）
   - `observed_read_failures` — `ReadObservedPose` 4 次重试全失败
   - `lease_failures` — detour 里 `begin_callback` 失败
   - `view_read_torn` — detour 里 `sequence != after`
   - `base_source_flips` — `plugin.cpp:2347` 处 `camera_base_source` 变化
   - `snap_count[reason]` — `view_snap = true` 的六处各记一个原因位
     （`1884` ResetMotion / `1985` 瞬移 / `2082` PublishInactive / `2291` 换预设 /
     `2302` reset 请求 / `2348` 基座源变化）
   - `detour_calls`、`detour_frames_without_publish`
2. `Update` 里用 QPC 包住 `UpdateCamera`，记录 `tick_ms` 的 last/avg/p99（p99 用 64 槽直方图即可）。
3. `DisplayState` 增加对应字段，`Draw` 加一个折叠的 **Diagnostics** 段（默认折叠，不占版面）。
4. **基线录制**：分别录 3 段各 15 s——直行、正常过弯、连续漂移；每段再录一次"开满 chrome +
   一个编译任务"的负载版本。存盘留作 T2/T3 前后对比。

**验收**

- 计数器全部可见、归零路径正确（`Start` 时清零）。
- 相机手感与改动前无差别（对照录屏）。
- `UpdateCamera` 自身计时开销 < 0.05 ms。

**实施记录（已完成）**

| 计数器 | 埋点 | 回答 |
|---|---|---|
| `snap_by_reason[6]` | `PublishView` 消费 `view_snap` 时统计 | 重置是否由真实不连续触发 |
| `yaw_source_switches` | `SelectCarYaw` → `NoteYawSource` | 漂移跳变的主因 |
| `observed_read_failures` | `ReadObservedPose` 重试耗尽 | 跨线程观测可靠性 |
| `lease_failures` | detour `begin_callback` 失败 | Q3 |
| `view_read_torn` | detour seqlock 校验失败 | 快照撕裂频率 |
| `detour_no_publish` | detour `active && !leased` | 游戏相机透出的帧数 |
| `detour_threads` | detour 每次调用 | Q1 |
| `detour_hz` vs `update_hz` | `Update` 的 0.5 s 窗口 | Q1 / Q4 |
| `tick_ms` last/avg/p99/max | `Update` 包住 `UpdateCamera`，128 槽直方图 | G4 基线 |
| `vehicle_polls` | `RefreshVehicleSample` 真正调用宿主 | T4.4 验收（应 ≈ tick Hz / 20） |
| `vehicle_gap_ticks` | `driving && !vehicle_valid` | Q6 |
| `player_sequence_jumps` | `UpdateMotion` | Q5 |

UI 入口：窗口底部 **Diagnostics** 复选框（默认折叠），六类 snap 只在非零时显示。

**跑一局后怎么读**（按主次）：

1. `Car yaw source switches` > 0 且漂移时增长 → 主因是 `SelectCarYaw`，先做 T3.2。
2. `Torn view reads` / `Hook lease failures` 与卡顿同步增长 → 先做 T1。
3. `Camera tick p99` > 1.5 ms → 先做 T4，否则 T2 的收益会被 tick 抖动吃掉。
4. `Render calls / Game tick` 明显高于 `Camera update rate` → Q1 结论为"每帧多次调用"，
   T2 必须先做幂等。
5. `Vehicle service polls` 明显高于 tick Hz / 20 → `kVehicleQueryIntervalSeconds` 没生效。

**人工基线（仍未做，需要游戏）**：录 3 段 15 s（直行 / 过弯 / 连续漂移），每段再来一次
带负载（开满 Chrome + 后台编译）的版本，存到仓库外，供 M3 前后对比。

**回滚**：纯增量，`git revert` 单个 commit。

---

### T1 — Presenter 契约（0.5 d，立即收益）

**动机**：诊断第 5 条——拿不到 lease 就退回游戏相机，本身是一次硬跳。

**步骤**

1. detour（`plugin.cpp:1039`）重排为：**先无条件尝试组装并写回插件位姿**，
   lease 只用于 `begin_callback/end_callback` 的生命周期保护，不再作为"是否接管"的开关。
   - 触及的只有本插件自己的 `std::atomic` POD，不调用宿主回调 → 无 lease 也安全。
   - `g_active` 已在 `ReleaseHook` 前置空，unload 期间不会进入。
2. 快照读取加重试：把 `plugin.cpp:1104` 的一次性 seqlock 读改成最多 4 次（对齐
   `ReadObservedPose` 的做法）；仍失败则用上一次成功组装的位姿（新增 `g_last_pose` POD），
   **绝不 fall through**。
3. `fov_restore` 分支同步调整，保证"不接管"和"恢复游戏 FOV"两件事解耦。

**验收**

- `lease_failures > 0` 的会话里，画面不再出现游戏相机的单帧透出（录屏逐帧核对）。
- `view_read_torn` 仍可能 > 0，但 `detour_frames_without_publish == 0`。

**回滚**：改动集中在一个函数，保留旧分支便于对比。

---

### T2 — 单时钟 presenter —— 已实现（待实机验证）

**已落地**：2.1 `car_sample.hpp`（`CarSample` + `SampleRing`，seqlock，容量 8，可离线单测，
新增 11 条测试）；2.2 `AdvancePresenter`（渲染线程以 1/120 固定步长推进 `chase`，同一
`now_ns` 只推进一次，`atomic_flag` try-lock，抢不到锁就返回缓存位姿）；2.3 Game tick 只
`Write(sample)` 并在锁内应用配置。

**与原描述的两处偏离**：

1. `Presenter` 的状态放在 `Context` 里而不是独立结构 —— 改动面更小，语义相同。
2. `generation` 改名为 `epoch`，由 `RequestPresenterReset()` 在 tick 侧递增，
   渲染侧观察到变化就 `chase.Reset()`。这替代了 `view_snap` 的复位职责。

**已删除**：`PublishView`（161 行）、`sync_delta` 及 `view_game_camera_*`、
`kMaximumGameCameraSyncCm`、`kMaximumViewInterpolationDelaySeconds/GapSeconds`、
`view_previous_*` 十一个字段、`view_location/rotation/fov/distance`、
`smoothed_view_distance_cm`、`raw_view_distance_cm`、`view_interpolated_frames`。

**仍待清理**（不影响行为）：`view_blend.hpp` 与其单测、`view_snap` 布尔位、
`interp%` 指标语义。

`plugin.cpp` 3297 行；构建无 warning，单测 78 项全绿。

---

### T2 — 单时钟 presenter（3.0 d，最大的一刀）

**动机**：诊断第 1、3、4 条的共同根因。完成后 G1 达成，`view_blend` / `sync_delta` / `view_snap`
整层消失。

**2.1 新增 `car_sample.hpp`（纯头文件，可离线单测）**

```cpp
struct CarSample {                       // 全部 POD，无分配
  int64_t  time_ns;
  uint64_t tick;                         // 宿主 tick（AnomalyNteSessionEventV1::tick_sequence）
  uint32_t generation;                   // 世界/玩家/锚点源身份合成
  Vec3     anchor_cm;                    // 车体渲染锚点（VehicleMesh 中心等）
  Vec3     origin_cm;                    // 模型原点（bounds 底 + offset）
  Vec3     velocity_mps;
  double   yaw_rad;                      // 估计值，非选择值
  double   slip_rad;
  double   forward_speed_mps;
  double   brake01;
  double   camera_scale;                 // 原 PublishView 里的缩放，搬到样本里
};

class SampleRing {                       // seqlock 环形缓冲，容量 8
  bool Write(const CarSample&);          // Game tick
  bool Latest(CarSample&) const;         // 只读
  bool At(int64_t t_ns, CarSample&) const;  // 两次采样之间线性插值
  void Clear();
};
```

- `At()` 的延迟策略：固定为"一个 tick 间隔"，`clamp(0, 50 ms)`。
  **常数延迟，不用启发式**——这是替代 `interpolation_delay` 的唯一正确做法。
- 插值内容：位置线性、yaw 走最短弧、slip/brake 线性。`generation` 不同则拒绝插值。

**2.2 新增 `Presenter`（`plugin.cpp` 内部结构，~120 行）**

```cpp
struct Presenter {
  nfs2015_chase::ChaseCam chase{GamePreset()};
  double accumulator{}, last_time_s{};
  bool initialized{};
  CameraState last_view{}; bool has_last{};

  void Advance(int64_t now_ns, const SampleRing&);   // 幂等：同一 now 只积一次
  bool Produce(const CarSample&, double* loc, double* rot, float* lens);
};
```

- `Advance`：`dt = clamp(now - last, 0, 0.1)`；`while (acc >= 1.0/120.0) { chase.Update(1/120, car_at(t)); acc -= 1/120; }`。
  - 同一 `now_ns` 重复调用时 `dt == 0`，返回缓存位姿（多视口/重入安全）。
  - 加一个 `std::atomic_flag` 防真并发重入：拿不到就返回 `last_view`，不推进。
- `Produce`：`eye = sample.anchor_cm + (view.position*100 - sample.origin_cm) * sample.camera_scale`；
  写 `location/rotation/lens`。这是目前散在三处的变换，**统一到这里**。

**2.3 迁移 game tick**

- `PublishView`（`plugin.cpp:2118`）改为 `PublishSample`：只组装 `CarSample` 并 `Write`。
  删除其中的：距离平滑（`2205`）、缩放、`previous_valid` 判定、`ViewHistoryJumped`、时间戳记录。
- `ChaseCam` 实例从 `Context` 移到 `Presenter`（tick 不再持有模型）→ 顺带减 tick 成本。
- 预设切换 / reset 请求：写入 sample 的 `generation`（或加 `reset_epoch` 字段），由 presenter 响应，
  不再有 `view_snap`。
- `camera_scale`、`origin_offset_cm`、`aspect_ratio` 归入样本（UI 改它们时下一个样本生效）。

**2.4 删除清单**（这是本组的验收标志）

| 删除对象 | 位置 |
|---|---|
| `view_blend.hpp` 整个文件 | — |
| `view_previous_*` 六字段、`view_has_previous` | `Context` |
| `view_snap` 及全部 6 个赋值点 | `1884,1985,2082,2291,2302,2348` |
| `interpolation_delay` 与 `update_hz < 57` 分支 | `1207–1213` |
| `kMaximumViewInterpolationDelaySeconds` / `...GapSeconds` | `116,119` |
| `sync_delta` / `kMaximumGameCameraSyncCm` / `view_game_camera_location` | `1255–1270` |
| `smoothed_view_distance_cm` / `kViewDistanceSmoothing` / `PublishView` 内平滑 | `122,2205` |
| `interp%` UI 指标与 `view_interpolated_frames` | `3022` 附近 |

**2.5 生命周期**

- `Stop` / `Unload`：`ring.Clear()`、presenter 复位、`has_last=false`，保证重载后不继承旧相位。
- 无驾驶 / 未启用：presenter 不推进，仍写回**原始游戏位姿**（不覆盖），并处理 FOV 恢复。

**验收**

- G1：T6 之前的临时验证——同一段录制，`--render-rate 20/30/60/144` 重放，共同时间轴 RMS 差
  ≤ 2 cm / 0.5°。
- 60 fps 限帧与 30 fps 限帧下，目视无阶梯；`tick_ms` p99 不升。
- `chase_cam_test` 仍全绿（模型未改）。
- `snap_count` 稳态为 0。

**风险 / 回滚**：这是唯一需要长期分支的任务。建议 `feature/single-clock-presenter` 分支，
保留旧 `PublishView` 路径一个版本（编译期 `NFS2015_LEGACY_TIMING`），确认后再删。

---

### T3 — 测量层 —— 3.1 / 3.2 已完成

**动机**：诊断第 2 条。核心是把"选择测量"换成"连续估计"。

#### 3.1 车头参考量改到 Game tick 上取（已完成，实测**不是**主因）

`QueryPlayer` 已经在每 tick 调 `player->esp_snapshot()`，但只用了 bounds。同一个
struct 里就有相机位姿（`AnomalyNtePlayerEspSnapshotV1::camera_rotation`，FRotator 度数），
它就是 `state.camera_rotation`，与 detour 在渲染线程观测的是**同一个量**，只是采样时钟不同。

改法：`context.camera_yaw_radians / camera_yaw_valid`，由 `QueryPlayer` 填写；
`SelectCarYaw` 优先用它，只有 profile 没有 ESP tail 时才回退到 `observed_*`。
新计数 `未取到本 tick 车头朝向的 tick 数`（`camera_yaw_esp_misses`）。

**实测结果（重要）**：计数恒 0（ESP 路径生效）、直行朝向无变化、手感无变化，但
**抽搐频率完全没变**，`车头朝向源切换` 的增长点也没变。

→ **跨线程混叠不是致因，80° 硬阈值就是全部。** 3.1 保留：它把输入换成了同 tick
同源的量，属于净收益，但以后别再把它当成抽搐的解释。

#### 3.2 slip 限幅（替换 `SelectCarYaw` 的二选一，已完成）

原实现：

```cpp
if (fabs(wrap(camera_yaw - velocity_yaw)) > 80°) return velocity_yaw;   // 跳变 ≤180°
return camera_yaw;
```

跳变被 `kMaximumBodyYawRate`（344°/s）拉成一段 90–300 ms 的鞭打，这就是抽搐。

**实际采用的设计不是原计划的 "gate + slip_est_" 估计器，而是无状态的限幅**：

```cpp
// chase_cam.hpp
inline double NoseFromCameraYaw(camera_yaw, travel_yaw, max_slip) {
    offset = WrapPi(camera_yaw - travel_yaw);
    return WrapPi(travel_yaw + Clamp(offset, -max_slip, max_slip));
}
```

- `|offset| ≤ 80°` → 原样返回相机 yaw（与旧行为完全一致）。
- `|offset| > 80°` → 车头停在「行进方向 ±80°」，并**随行进方向连续移动**。
- 这是 C0 连续的映射（边界处只是折点），所以再大的漂移也不会产生阶跃，
  `kMaximumBodyYawRate` 退回纯安全网。

选它而不是 gate 估计器的理由：无状态（不会因为一次跳过而卡在错误值）、
**连续性可证**（测试直接扫过 0.8 rad 检查最大步进 = 输入步进）、实现只有 4 行。

**倒车分支**：`travel_yaw = velocity_yaw + π` 当检测到倒车时。检测用
「速度在前一次车头方向上的投影」+ 迟滞（进入 −0.5 m/s，退出 +0.8 m/s）。
关键性质：限幅未激活时两个分支都返回相机 yaw，所以误判也只影响 `travel_yaw`，
不会造成跳变。

**副作用（预期内）**：旧实现在 `>80°` 时把 `car.yaw` 换成速度方向，于是
`slip_angle = 0` → `drift = 0` —— 也就是说**漂得最狠的时候漂移效果被关掉了**。
现在会看到 80° 的 slip，`drift_target` 直接饱和（`drift_full = 28°`）。
所以大幅漂移时推近/侧倾/look 偏移会比以前**更明显**，这不是 bug，是恢复了本该有的效果；
若嫌太强，调 preset 的 `drift_start` / `drift_full`，别动限幅。

**测试**（`tests/chase_cam_test.cpp`）：跟随相机 / 两侧限幅 / 扫过边界最大步进
0.0020 rad（= 输入步进，即无跳变）/ 720 个采样点不越界。

#### 3.2b slip 限幅方案已回退（重要教训）

3.2 的"slip 限幅"（车头不得偏离行进方向 >80°）在实机上造成 regression：**前进时相机
经常跑到车前面**。成因是倒车检测用了上一 tick 自己输出的 `last_car_yaw`：

```cpp
const double forward_speed = velocity_mps * cos(velocity_yaw - context.last_car_yaw);
if (forward_speed < -0.5) context.reversing = true;   // ← 自锁
```

一旦误判为倒车，输出整体偏 ~100°；下一 tick 算出的 `forward_speed` 因此继续为负，
永远出不来。相机就绕着车转到前面去了。

**处置**：回退限幅，`SelectCarYaw` 回到「相机 yaw 与行进方向夹角 >80° 时改用行进方向」
的旧语义（该语义从未报过问题），但保留两项改进：

- 3.1 的 Game tick 相机 yaw；
- 倒车用**宿主带符号的前进速度**判定（已实测：倒车为负），无状态、不可能自锁。

教训：任何用"上一 tick 自己的输出"参与判定状态的逻辑都会自锁；宿主已经给了判据时，
不要自己构造。

#### 3.3 锚点连续性契约 —— 已作废

原计划是维护 `last_good_anchor`、允许重定位条件等。T4.4 之后锚点恒为 Pawn 根位置
（只有一个来源、每 tick 都有），`base_source_flips` 这个计数器本身已经删除，
不再需要这套契约。

#### 3.4 抽搐归因 —— 结论与最终修法

##### 实测数据

用 detour 内的指标（`渲染速度 最低/最高`，每样本除以自身帧间隔）：

| 场景 | 眼睛速度 最低/最高 | 转向速率 最低/最高 | 锚点每 tick |
|---|---|---|---|
| 全速直行 | 17.9 / **252.0 m/s** | 0.0 / 7.4 deg/s | 80.1 cm |
| 持续漂移 | **0.0** / **466.4 m/s** | 0.0 / **1350.7 deg/s** | 100.2 cm |

（另一轮：直行 0.0 / 1046.8 m/s，漂移 0.0 / 677.3 m/s、0.0 / 287.1 deg/s、锚点 88.8 cm。）

##### 指标本身是错的，必须作废

车速 36 m/s、帧率约 30 fps 时，相机**每帧本来就要走 1.2 m**，所以"最高 1046 m/s"
只是帧边界；"最低 0" 是同帧多个相机 pass 的重复采样。该指标**无法区分正常帧步进与
抽搐**。写在这里防止以后重复踩。

它仍揭示两件事：

1. 上一版"渲染时钟速度积分"**是死代码**：`render_now − publish_time` 在一帧内几乎
   不变（渲染紧跟发布）→ `dt≈0` → 眼睛整帧停在最新姿态，下一帧再跳。
2. **结构性结论**：模型每 tick 产出一个姿态，而渲染器（<60 fps）采样频率低于 tick。
   任何"从 tick 数据插值/外推"的方案都会留下帧级阶跃 —— 这是这类方案的共同上限，
   不是某个公式写错。五次失败全部落在这个上限上。

##### 已证伪的路线（不要再试）

| 路线 | 结果 |
|---|---|
| 插值锚点 → 相机与车身共享一条曲线 | 直道抽搐 |
| 删掉 `sync_delta` | 直道 + 拐弯都抽 |
| `α × 车身位移` 替换 `sync_delta` | 都抽，高速更明显 |
| `base_step` 全权重替换 `sync_delta` | 最差（峰值 1046 m/s） |
| 渲染时钟速度积分（门控在 `view_kinematics_valid`） | 死代码，从未执行 |

##### 一阶低通（第 7 次）—— 幅度减小但出现"涌入"

实测（直行）：眼睛速度 5.3 / **57.2 m/s**，转向 1.0 / 4.2 deg/s。
峰值比车速高一倍、最低只有五分之一 —— 一阶低通在 target 每帧跳一整拍时**追不上**：
一帧猛冲（57 m/s）、下一帧几乎停住（5 m/s）。画面表现为相机每帧"涌一下"。
用户反馈"抽搐幅度变小"但仍在。

##### 最终修法：前馈 + 校正（predictor-corrector）

```cpp
// 前馈：用发布姿态自身的每 tick 速度，40 ms EMA 平滑
v += (v_target - v) * (1 - exp(-dt_frame / 0.04))
eye += v * dt_frame
// 校正：只修残差，50 ms
eye += (target - eye) * (1 - exp(-dt_frame / 0.05))
```

关键区别：**动量由前馈项搬运，滤波器只修正残差**，所以眼睛以恒定速度前进，
不再"追"。即使 target 每帧阶梯式跳变，输出速度也近似常数。

|target − eye| > 400 cm 视为不连续（传送/换图），直接吸附而不是滑过去。

##### 旧方案：渲染线程一阶低通（已被前馈取代）

```cpp
// detour，按真实帧间隔驱动
constexpr double kSmoothingTimeConstantSeconds = 0.018;
blend = (dt_frame > 0) ? 1 - exp(-dt_frame / 0.018) : 1.0;
eye    += (target - eye) * blend;      // 位置三分量 + pitch/yaw/roll
```

target = "最新车身锚点 + 延迟插值偏移"（不含 `sync_delta`）。
**无论 target 怎么跳，输出都是指数逼近，在构造上平滑**，不需要知道真实帧率、
不需要 tick 间隔、不受多 pass 重复采样影响。代价是约 18 ms 固定滞后。

`view_snap_generation` 在传送/预设/换图时递增，渲染侧观察到变化就重置滤波器，
避免拖一条长尾。

`sync_delta` 保留测量与峰值显示，不施加。

#### 验收

- 掉头 / 倒车 / 撞车时不再鞭打（3.1 已证混叠无关，只剩这一条能解释）。
- `车头朝向：相机 / 打滑限幅` 在大幅漂移时增长是**正常**的（模式切换），不代表故障；
  它不再是"修正手段"，所以增长时画面不应有任何跳变。
- 倒车不再让相机绕着车转一圈。

### T4 — Tick 成本 —— 已完成（4.4）

**症状**：`tick_ms` 均值 2.8–3.0 / p99 4.3 / 峰值 6.40（50 Hz tick 的 14%）。

**走过的弯路（全部有实测、全部撤销）**

| 版本 | 做法 | 结果 |
|---|---|---|
| v1 | 复用上一帧实体句柄 | 命中 0，2.96 ms |
| v2 | 用 vehicle service 对象句柄 | 命中 0，2.87 ms |
| v3 | 玩家实体优先 | 命中 0，**5+ ms** |
| v4 | `EntitySnapshotTickInterval` 1 → 30（改宿主 INI） | **2.9 → 3.0 ms，无变化** |

v4 是关键否证：`实体翻页` 增速随之降到 1/30，tick 却毫无改善 → **重扫和翻页都不是成本**。
v3 的 +2 ms 也说明翻页本身不便宜，但它不是本 tick 的固定项。

**真正的根因（读宿主源码确认）**

宿主每一次内存读都是一次 `ReadProcessMemory`（`src/platform/windows/memory.cpp:95`），
所以开销 ≈ 读次数 × 0.5 µs。据此逐条数调用：

| 每 tick 调用 | 内核读 | 实测量级 |
|---|---|---|
| `player->snapshot` + `esp_snapshot` | ~10（纯缓存拷贝） | 0.01 ms |
| `entities->page` × 32 | 0（`EntityMatches` 纯整型比较 + 结构体拷贝） | 0.1 ms |
| `vehicle->snapshot` | ~160 + **3 次 ProcessEvent** | 0.5–1.0 ms |
| `component_bounds("VehicleMesh")` | **~15K** | 1.5–2.5 ms |

- `component_bounds` → `FindReflectedEntityPropertyLocked`（`ue5_nte_adapter.cpp:13963`）
  沿类的 `propertyLink` 线性找属性名。**每个被跳过的属性节点 = 7 次内核读 + 2 次 FName
  解码 + 2 次 `std::string` 分配**（`ResolveNameSnapshotLocked` 为了拿长度和内容各解码
  一遍；`ReadReflectedObjectNameLocked` 只解一遍，说明这条路径是被漏掉优化的那条）。
  上限 2048 个节点。
- `vehicle->snapshot` → `RefreshVehicleLocked`（:8131）**每次调用**都跑
  `BP_GetCurrentDriveVehicle`（ProcessEvent）、`ObjectClassChainContainsLocked`
  （≤32 层父类，每层一次 FName 解码）、`GetForwardSpeedKmH` 与
  `GetExternalTorqueRatio`（各一次 ProcessEvent）。宿主没有任何缓存或节流。

**实现（4.4，纯插件侧，不改宿主配置）**

1. **不再 lease `entities` / `actors` 服务**。相机从头到尾没用上它们：
   驾驶时 Pawn 根组件就是载具根组件，它的位置与 bounds 已经由 player 服务每 tick
   以缓存拷贝的形式免费给出。删掉后宿主连 `entity_demand` 都不会被置位，
   实体重扫对我们彻底停止。
2. **锚点固定为 Pawn 根位置**。`base_cm` 在 `PublishView` 里被 `render_offset` 减掉，
   detour 又把它加回来，所以它**完全抵消**、不参与世界坐标，只需要稳定和便宜。
   这同时消灭了 `基准源切换` 这一类跳变（记录 `snap_by_reason[5]` 已删除）。
3. **`vehicle->snapshot` 按 50 ms 轮询**（`kVehicleQueryIntervalSeconds`），两次之间
   复用上一次的样本。速度是慢变量，所有消费者本来都在做阻尼。
   新计数 `载具服务轮询次数` 应 ≈ tick Hz / 20。
4. 删除死代码：`QueryVehicleBase` / `ReadVehicleBase` / `ReadEntityMesh` /
   `ReadVehicleRoot` / `ReadComponentTransform` / `FindVehicleEntity` /
   `FindLocalPlayerEntity` / `ScanVehicleEntity` / `CopyEntityBounds` /
   `PlausibleVehicleBounds` / `ProbeEntityFrameCamera`（共 ~440 行）。

**唯一的可见变化**：`origin_cm` 的 z 从「VehicleMesh 底面」变成「根组件底面」。
根组件 bounds 覆盖整车（含轮胎），所以原点更接近地面——这恰好符合模型对
`car.position` 的约定（`chase_cam.hpp:406`「ground/centre」）。若觉得相机比以前
低了几厘米到二十几厘米，用 `Car origin offset (cm)`（±150）加回去即可。

**验收**

- `tick_ms` 均值 < 0.3 ms、p99 < 1 ms。
- `载具服务轮询次数` ≈ tick Hz / 20；`实体翻页 / 载具扫描` 两项已从 UI 移除。
- `基准源翻转` 不再存在；`视图重置次数` 的 `运动状态重置` 应显著下降。

### T5 — 模型质量（1.0 d）

1. **roll 输入替换**：`chase_cam.hpp:669` 的 `lateral_g = Dot(acceleration, right)/9.81`
   改为向心加速度 `lateral_g = -forward_speed * yaw_rate / 9.81`。
   - **符号必须先验证**：录一段固定左弯，对比新旧 `roll_target` 的符号与峰值，
     保持与原行为同向后再替换（`roll_target` 的既有约定见 `671`）。
   - 同时删除 `plugin.cpp:2502` 附近"保留侧向分量"的补丁。
2. **`drift_target > drift` 的 bang-bang**（`chase_cam.hpp:625`）：若 T3 后仍可观测到翻转，
   改为带迟滞的 latch（`±0.02`）并记 `drift_mode_flips`。
3. **死输入清理**：`CarState::donut_scale / nitrous / on_ground` 从未被填充。
   本轮二选一：从 vehicle snapshot 接上（目前只有 `top_speed_ratio`，**接不上 nitrous**），
   或删除对应模型分支并在 README 注明。**建议删除**，避免下一个人再被误导。
4. **测试重新基线**：`tests/chase_cam_test.cpp:151–159` 的期望值随逐帧运行与 roll 输入改变；
   重新生成基线并补两条新测试：
   - 阈值悬停输入（slip 在 6°–28° 间抖动）下 `drift_` 的导数有界；
   - 同一输入以 1/30 与 1/120 步长积分，输出在共同时间轴上一致。

---

### T6 — 不变量与 CI（2.0 d）

**这是"彻底"与"这次修好了"的分界线。**

1. **录制**：UI 加一个 "Record 12 s" 按钮，把 `CarSample` 流以二进制写入
   `Documents/Nfs2015ChaseCam/*.rec`（带版本头 + 采样率）。用于回归与复现。
2. **`tests/presenter_replay_test.cpp`**（不链接 SDK、不需要游戏，与现有夹具同风格）：
   - 从 `.rec`（或内置合成序列）驱动 `Presenter`，渲染频率分别 20 / 30 / 60 / 144 Hz；
   - **G1 断言**：四种输出重采样到共同时间轴，逐帧差 ≤ 2 cm / 0.5°；
   - **G2 断言**：每帧 `Δangle ≤ kMaxAngularRate·dt`、`Δpos ≤ kMaxLinearRate·dt`；
   - **G3 断言**：稳态段 `snap_count == 0`，每次 snap 必须有 `generation` 变化作证；
   - **抖动指标**：漂移段 yaw 的 jerk RMS 低于阈值。
3. **运行时断言**（Debug）：连续性违反时 `assert` 并打印帧号；Release 只计数。
4. `ctest` 接入；README 的"验证"一节补一条命令。

**验收**

- `ctest` 全绿；把 T2 里删掉的 `interpolation_delay` 式启发式人为加回，
   invariance 测试必须变红（做一次变异验证，证明测试有效）。

---

### T7 — 清理与文档（0.5 d）

1. 删除 T2–T5 留下的死常量、死字段、死注释。
2. UI 指标替换：去掉 `interp %`，改为 `snap`、`sample age (ms)`、`tick ms (p99)`、`anchor src`、
   `slip (est/raw)`。
3. `locales/zh-CN.json` 同步新增 key。
4. `README.md`：更新"实现方式"一节的线程模型与数据流图；新增 **反模式清单**（见下）。
5. 版本号：`manifest.json` / `pluginmaster.json` / ZIP 名同步（按 README 发布流程）。

---

## 5. 里程碑检查点

| 里程碑 | 判定条件 | 可否发布 |
|---|---|---|
| M1 = T0 | 计数器齐全，三组基线录制完成，主因排序确认 | 不发（无行为变化） |
| M2 = T1 | 负载会话下无单帧透出 | 可以（1.0.1 热修） |
| M3 = T2 | G1 达成，删除清单全部落地，`chase_cam_test` 全绿 | 可以（1.1.0） |
| M4 = T3+T4 | G3、G4 达成，漂移抖动收敛 | 可以（1.1.x） |
| M5 = T6 | G5 达成，CI 守护生效 | 可以（1.2.0） |

**M3 之前不要动 T5 的参数**——逐帧运行本身会改变观感，混在一起无法归因。

### T0 数据驱动的顺序修订

实测（50 Hz tick / 1400 Hz detour）：

| 现象 | 数据 | 结论 |
|---|---|---|
| 漂移抽搐 | 单 tick slip 跳变仅 0.6–3.0°；**同步偏移峰值 41/100 cm** | **病因是 `sync_delta` 混时钟**，已改为 base+offset 同 α 插值，待验证 |
| 粘滞 | `tick_ms` 均值 2.9 / p99 4.3；改宿主 interval 无变化 | **T4.4 已修**（删掉每 tick 的 `component_bounds`，载具快照改 50 ms 轮询），待一局验证 |
| 单帧透出游戏相机 | lease 失败 = 0、撕裂 = 0 | T1 降级为防御性修复，排到最后 |
| 帧同步错位 | 每帧 28 次 detour 调用 | T2 必须幂等；`sync_delta` 现在对同帧 28 个视图给出不同偏移 |

修订后的顺序：**T4.4（已修）→ 阶梯修复（已修，见下）→ T3.1（已修）→ T3.2（已回退，见 3.2b）→ T3.4 单时钟呈现（已修，待验证）→ 精确 slip（已量化 13–20° 偏差）→ T1 → T6 → T5/T7**。

#### 提前完成：50 Hz 阶梯（T2 的一小块）

`plugin.cpp` 的 `update_hz < 57 → interpolation_delay = 0` 分支已删除。理由：

- 发布的是**相对车的偏移**而不是绝对位姿，detour 用当拍的车位置做锚点，所以延迟只作用在
  **旋转和偏移**上，车本身永远是最新的——那条分支的注释描述的"车比相机快一拍"在改成
  相对偏移之后就不存在了，是旧绝对位姿时代的遗留。
- 保留分支的后果是 `alpha ≡ 1`：同 tick 内 1400 次调用返回同一旋转，漂移时就是 20 ms 一跳。

同时删掉了已经没人读的 `view_update_hz`。`插值命中` 这个指标从这一版起才有意义。

---

## 6. 开放问题（T0 已答）

| # | 问题 | 答案 | 影响 |
|---|---|---|---|
| Q1 | detour 每帧被调用几次？多线程吗？ | **约 1400 Hz vs 50 Hz tick = 每 tick 约 28 次**；线程数待看 `Detour threads` | T2 必须幂等推进；`sync_delta` 目前对同一帧内的 28 个视图各给一个不同偏移 |
| Q2 | 游戏相机位姿能否在 Game tick 上读到？ | **能，而且在 `esp_snapshot` 里**（`camera_position` / `camera_rotation` / `horizontal_fov_degrees`，每 tick 的缓存拷贝）。`entity_frame.camera_rotation` 同样被填充，但那条路要 lease entities 服务，已随 T4.4 一起删除 | T3.1 用 `esp_snapshot.camera_rotation`，无需实体服务 |
| Q3 | `begin_callback` 失败频率 | **0** | T1 降级为防御性修复，不是致因 |
| Q4 | `delta_seconds` 是 tick 还是帧间隔 | 游戏 tick 约 50 Hz（`update_hz` 与 tick 一致） | 相机 tick 就是 20 ms 步长 → 所有 `Damp` 都是 20 ms 一步 |
| Q5 | `player.sequence` 是否每 tick 递增 | **待读** `玩家序号跳变` | 决定位移导数是否除以了错误的 elapsed |
| Q6 | vehicle snapshot 掉帧频率 | **待读** `载具快照过期仍继续运行` | 决定 T3.3 的保活时长 |
| Q7 | yaw 源切换何时发生 | **漂移掉头 / 倒车 / 漂移撞车**；干净漂移不切换 | 直接印证 T3.2：这是测量选择，不是调参问题 |

### Q7 的机制

- 干净漂移：速度方向与游戏相机 yaw 的夹角 < 80°，`SelectCarYaw` 一直返回相机 yaw，不切换。
- 掉头：车头先转，速度还指向原方向，夹角越过 80° → 切到速度 yaw → `car_yaw` 阶跃。
- 撞车：速度骤降/方向突变，`velocity_length > 50 cm/s` 门限来回穿越 → 源在
  "相机 yaw / 速度 yaw / 无观测" 三者间抖动。
- 倒车：速度与车头夹角恒为 ~180°，**必然**越过 80°；同时 `have_velocity` 在速度门限
  附近来回 → 每 tick 都可能切换，使得相机跑到车头前方。

结论：`kMaximumCameraYawDifference` 不是"调大一点"能解决的问题。原式是
"在两个测量里挑一个"，正确形状是"估计 slip 这个慢变量"。
倒车还需要一个显式分支（`forward_speed < 0` 时保持相机 yaw、slip 归零），
而不是让估计器去处理一个物理上 180° 的角。

### 直接读车头 yaw 的可能性（已排查）

SDK 里**没有**载具/角色的旋转：

- `AnomalyNteEntitySnapshotV1` / `AnomalyNteEntityComponentBoundsV1`：只有 AABB
  `bounds_center` + `bounds_extent`。
- `AnomalyNtePlayerSnapshotV1`：只有 `position[3]`。
- 属性服务只有 `component_bounds` / `bool_property` / `fname_property_utf8`，
  没有 float/vector/rotator 读取。
- 整个 `nte.h` 中唯一的 rotation 是相机（`AnomalyNteCameraSnapshotV1`、
  `AnomalyNtePlayerEspSnapshotV1`、`AnomalyNteEntityFrameV1`）。

所以只能估计。三个理论来源：

1. **游戏相机 yaw**（现有 + `entity_frame.camera_rotation`）：它本身就是本模型要复现的
   目标量，只是带上游戏自己的弹簧滞后。T3 用它 + slip 估计器。
2. **AABB 反解**：旋转 yaw θ 的盒子的世界 AABB 满足
   `ext_x = hx|cosθ| + hy|sinθ|`、`ext_y = hx|sinθ| + hy|cosθ|`；
   用长窗口的 `min(ext_x)=hx`、`min(ext_y)=hy` 标定后可以解出 `|θ|`，
   再用速度/上一帧消歧。理论上直接、与相机无关；实际在 0°/90° 附近病态，
   且 LOD/受损会改变 bounds。**只作为倒车时的消歧辅助，不做主源。**
3. 签名 + `core.read_memory`：需要 UE 对象指针，ABI 明确不暴露，且随游戏版本失效。**不做。**

## 7. 验收指标定义

| 指标 | 定义 | 目标 |
|---|---|---|
| 频率不变性误差 | 20/30/60/144 Hz 重放输出，重采样后逐帧差的 RMS/p99 | RMS ≤ 2 cm / 0.5°，p99 ≤ 5 cm / 1.5° |
| 连续性违反 | `Δ 位姿 > kRate·dt` 的帧数 | 稳态 0，全段仅随 `generation` 变化 |
| 源切换 | `yaw_source_switches + base_source_flips` | 稳态 0 |
| snap | 模型重置次数 | 仅随 `generation` 变化 |
| jerk | 漂移段 yaw 三阶差分的 RMS | 比基线下降 ≥ 50% |
| tick 成本 | `UpdateCamera` p99 | < 1.5 ms（负载下 < 3 ms） |

---

## 8. 反模式清单（写进 README，防止复发）

1. 禁止"低帧率下关闭平滑/插值"——帧率低正是最需要它的时候。
2. 禁止无迟滞的二选一（`if (diff > T) use B; else use A`）。
3. 禁止静默降级：每一次 fallback 都必须有计数器。
4. 禁止把相机位姿当车体传感器；需要参考量就用 tick 对齐、带归属校验的样本。
5. 禁止用旗标强制跳过插值/平滑；重置只能由 `generation` 变化触发。
6. 禁止在 game tick 上做 O(n) 反射或整段扫描。
7. 禁止在 render 路径分配内存、加锁、写日志。
8. 禁止把"渲染时刻"和"游戏 tick 时刻"混用在同一次坐标变换里。

---

## 9. 开工前 5 分钟

```sh
git switch -c feature/camera-timing
cd nfs2015-chase-cam
./build-mingw.sh --run-tests          # 确认基线全绿
ctest --test-dir build.mingw          # 现有 421 行夹具
# 录 3 段基线（直行 / 过弯 / 漂移），另存 ~/nfs-baseline/
```

先做 T0，把 `Diagnostics` 段跑出来；把 Q1–Q6 的答案填回本文档第 6 节，再决定 T2 与 T3 的先后。
