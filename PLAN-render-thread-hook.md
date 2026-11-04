# 🚀 任务实施计划：渲染线程车辆变换 Hook（交接给执行 AI）

> 本文件由「规划 AI」产出，取代 `NOTES-render-thread-hook.md`（保留作来源，不再更新）。
> 配套背景：`PLAN-camera-timing.md`（§9 验收指标、§10 反模式、§11 设计原则仍然有效）。
> 基线：`nfs2015-chase-cam` HEAD = `38c98ef`，工作区干净。

---

## 1. 目标与宏观共识

### 1.1 全局目标

消除行驶中**"车抖、相机不抖"**的剩余抖动。已知根因（已二分确认）：相机用 game tick 采样的
`player.position` 计算，而游戏用**渲染线程那一刻的车辆变换 X** 画车，两者不同步；
且从游戏相机位置反推 X 数学上不可逆（相机弹簧是低通，高频信息已丢）。

### 1.2 宏观架构（规划 AI 与用户达成的共识，执行 AI 不得更改）

- **D0 — 路线**：走 hook 路径，直接拿渲染线程的车辆变换 X。必须带版本保险（游戏更新后插件失效、
  **不能崩**）。
- **D1 — 不升级 Anomaly main**：`get_object_handles` + `invoke` 都在 game thread，拿不到渲染线程 X，
  升级 main 解决不了抖动。本期**不改宿主（Anomaly）源码**，全部在插件内完成。
- **Q1 — 主路线 A1（复用已有 detour + 插件内读内存）**，兜底 A2（找专用 hook，见 1.3）。
- **Q2 — X 的用途 B1**：X **只作位置锚点**（替换 borrow 的估计目标），速度/朝向/yaw_rate/slip
  仍走 game-tick 差分——它们是进弹簧的低频通道，1 tick 相位差影响远小于位置。**本期不**把速度/
  朝向迁移到渲染帧差分（记入"待触发"清单，见 §4 任务组 5）。
- **Q3 — 版本保险三层**（详见 §4 任务组 2）：① 运行时 resolve 全部所需 pattern；② 硬编码偏移表 +
  `profile_hash` 门禁；③ 失败**回退现有 borrow**，界面显示当前机制，不弹窗不崩。
- **Q4 — 成败判定**：定量门槛（见 1.5）只是执行 AI 的**自我检查线**；**最终成功必须由用户亲自
  肉眼确认"无可见抖动"**，执行 AI 不得自行宣布成功。

### 1.3 关键发现（执行 AI 必读，直接影响怎么做）

1. **磁盘上的游戏二进制是受保护的**：`HTGame.exe` / `HTGameBase.dll` 全部 10 个节区名都被改成
   `.std`（正常应为 `.text`/`.rdata`）；整个文件 55.4% 是零字节；入口点在最后一个节区；游戏目录
   有 `AntiCheatExpert` 目录。**决定性证据**：插件运行时 hook 的那段相机函数字节码（
   `kCameraViewPointPattern` 前 12 字节）在磁盘文件里**完全搜不到** → 磁盘代码区不是明文。
2. **推论 A**：NOTES 里的 Route 1-2（对磁盘文件解析 PE + 反汇编）是**死路**。要对代码做静态分析，
   必须先做**运行时内存 dump**，再分析 dump。
3. **推论 B**：**A1（运行时读内存）完全不受影响**。游戏启动时把代码解密进内存；宿主的
   `signature.resolve` 扫的是**已解密的内存 `.text`**——现有插件能正常 hook 相机函数就是铁证。
4. **推论 C**：`anomaly-cli.exe` 诊断管道（见附录 C）可用 `chain / ptr / f64 / read / scan / ue`
   等命令**交互式验证偏移链**，省掉大量盲写代码的成本。
5. **推论 D（探针设计的关键）**：宿主 `player.position` 读的是 `rootComponent + 280`
   （`sceneComponent.location` == `sceneComponent.boundsOrigin` == 280，见 `ue5_nte_adapter.cpp`
   `ReadCurrentPlayerLocation`）。这个 280 是 **Bounds.Origin（包围盒中心）**，**不是**渲染线程画车
   用的 `ComponentToWorld` / SceneProxy 变换。因此探针第一步读 280，**可能测出相位差，也可能测不出**
   ——这正是 1.5 的止损点。

### 1.4 假设（待探针验证，不要当结论）

> **H**：渲染线程在画车那一刻读到的车位置，比宿主 game-tick 采样的 `player.position` **更新**
> （相差 1~2 个 game tick 的车位移，30 fps / 36 m/s 下约 1.2 m/tick）。锚到渲染线程读到的值，
> 即可消除车与相机的错帧。

H 有两种成立形态，探针要分辨：
- **H-同字段**：渲染线程读的也是 280（或与 280 同步更新的字段），只是读得比宿主晚。→ 读 280 即可，
  **无需反汇编**。
- **H-异字段**：渲染线程读的是 `ComponentToWorld`（偏移未知，不在 profile 里）。→ 需要任务组 1
  的反向工程找偏移。

### 1.5 定量检查线与止损（执行 AI 的自我检查，不是最终验收）

| 检查项 | 阈值 | 说明 |
|---|---|---|
| 相位差（探针） | 应达几十 cm ~ 1 m+ | 30 fps/36 m/s 一帧约 1.2 m。**若 < ~1 cm，说明抖动另有主因，立即止损汇报** |
| 相对速度峰值（漂移） | < 5 m/s | 现状 borrow 为 24.7 m/s；PLAN §9 验收线 |
| 相对速度峰值（直行） | < 1 m/s | 现状 2.3 m/s |

**止损协议**：探针相位差 < ~1 cm → 抖动另有主因，停止，向用户汇报；最终实现若达不到门槛 → 按
任务组 4 的三档结局处理，**先汇报用户再决定是否冻结路线**。

---

## 2. 微观决策授权范围（执行 AI 自行决定，不必问用户）

- 反汇编/分析工具选择（dump 后 Ghidra / IDA / objdump / capstone 任选）。
- 诊断面板行的命名、文案、配色、阈值显示格式。
- 偏移表在代码里的组织方式（常量表 / 结构体 / 宏）。
- 内存读取的缓存策略细节（例如：是否在 detour 里缓存 `manager→pawn→root` 指针、缓存失效条件、
  每帧读一次的节流）。
- 具体 blend 数学、EMA 系数、重试间隔、探针采样窗口等数值。
- commit 拆分与 message。
- 单测写法（新增探针/保险逻辑的测试夹具，按 `tests/` 现有风格）。
- 附录 A 里未列的、与宏观架构无关的实现细节。

---

## 3. ⚠️ 异常汇报机制（重要）

请执行 AI 注意：计划赶不上变化。若执行中发现实际情况与本计划有明显出入，或遇到可能影响全局的重大
阻碍，**请立即停止执行并向用户汇报**，由用户转述给规划 AI 重新规划。具体包括但不限于：

- 探针相位差 ≈ 0（1.5 止损点触发）；
- 渲染线程 `core->read_memory` 被 AntiCheatExpert 拦截 / 触发崩溃 / 引入卡顿；
- `ue5.GWorld` pattern 在运行时 resolve 失败（区别于预期内的"版本更新"场景，要区分是 bug 还是真的失效）；
- X 锚定实测达不到门槛、或引入新的可见问题；
- 任何涉及"冻结 hook 路线"的结论（任务组 4 结局③）**必须由用户拍板，执行 AI 不得自行冻结**。

**最终成功判定权在用户**：定量达标只是必要条件；用户亲测"肉眼无可见抖动"才算成功。

---

## 4. 任务步骤分解

> 依赖关系：任务组 0 是总闸门；0 通过后才进 2。任务组 1 仅在 0 的"异字段"分支触发。
> 任务组 2 是核心实现；3 是验收与故障注入；4 是用户终审；5 收尾。

### 📦 任务组 0：相位差探针（只读，可行性验证）

- **难度评级**：⭐⭐ [中等]
- **目标**：验证假设 H，并分辨 H-同字段 / H-异字段，给出 Q4 的止损判定。
- **具体行动项**：
  - [ ] 0.1（可选，省时）用 `anomaly-cli.exe` 交互式验证偏移链（见附录 C）：`ue` 或 `scan . .text "<GWorld pattern>"` 拿 GWorld；`chain`/`ptr`/`f64` 逐级走
    `+560 → +56（解引用 TArray.data）→ [0] → +48 → +776 → +456 → +280`，确认最终读出的 3 个 double
    是车当前位置（值合理、随车动）。这一步只验证"偏移正确 + 链可走"，**测不了渲染线程相位差**
    （CLI 走的是管道线程，不是渲染线程）。
  - [ ] 0.2 在 `CameraViewPointDetour`（渲染线程）里加**只读探针**：从 `GWorld` 走链到 `root+280`，
    用 `core->read_memory` 读 3× double，与宿主 game-tick 的 `player.position` 快照逐轴相减，
    面板显示相位差（建议：每帧最大值 + 0.5 s 窗口均值/峰值）。遵守反模式 7：**不分配内存、不加锁、
    不写日志**（用栈缓冲 + 原子变量）。
  - [ ] 0.3 判定并记录：
    - 相位差显著（几十 cm ~ 1 m+）→ **H-同字段成立**，跳到任务组 2，用 280 实现。
    - 相位差 ≈ 0 → **H-异字段嫌疑**，进任务组 1 找 `ComponentToWorld`。
    - 相位差 < ~1 cm → **止损**，向用户汇报。
- **验收**：面板有相位差数字 + 一局数据 + 明确写出"同字段 / 异字段 / 止损"结论。

### 📦 任务组 1（条件触发）：定位渲染变换字段 ComponentToWorld

- **难度评级**：⭐⭐⭐ [困难]
- **触发条件**：任务组 0 判为"异字段"（读 280 无相位差）。否则跳过。
- **目标**：找到渲染线程画车真正读取的变换字段（大概率是 `USceneComponent::ComponentToWorld`，
  也可能在 SceneProxy 缓存里），给出它的偏移和读取方式。
- **具体行动项**：
  - [ ] 1.1 做**运行时内存 dump**（用 `anomaly-cli` 的 `read`/`snapshot`，或进程转储工具）——磁盘文件
    是加密的，**不能**直接反汇编磁盘文件。
  - [ ] 1.2 用 UE5 公开结构知识 + dump 反汇编，确定 `ComponentToWorld` 相对 `rootComponent` 的偏移
    （已知线索：`sceneComponent.location/boundsOrigin=280`、`boundsExtent=304`、`mobility=244`；
    `ComponentToWorld` 是 96 字节 FTransform，通常与这些字段相邻；可用"平移分量≈车位置 + 四元数
    W≈1"在 dump 里反查确认）。
  - [ ] 1.3 把探针改读该字段，重测相位差（回到任务组 0.3 的判定）。
- **验收**：确定的偏移 + 相位差证据（证明它确实与 game-tick 快照有相位差）。

### 📦 任务组 2：实现 X 锚定 + 版本保险（核心）

- **难度评级**：⭐⭐⭐ [困难]
- **目标**：把渲染线程读到的 X 接进 borrow 的估计目标，并落实三层版本保险 + 界面状态行。
- **具体行动项**：
  - [ ] 2.1 `manifest.json` 增加可选服务 `anomaly.ue5.build`（`optional: true`），`capabilities` 同步；
    `plugin.cpp` 里 `Query` 它（拿 `build_id` / `profile_hash`）。注意 SDK 头文件已是 v2.5.0 一致版本
    （见 PLAN §7.7 的教训：不要引入比宿主新的头文件）。
  - [ ] 2.2 启动时（on_update 首次或 init）用 `ResolveSignature` 解析 `ue5.GWorld` pattern
    （附录 A），缓存 GWorld 地址。**任一 resolve 失败 → 新路径 disabled**。
  - [ ] 2.3 硬编码偏移表（`world.gameInstance=560`、`gameInstance.localPlayers=56`(+8 为 count)、
    `localPlayer.controller=48`、`controller.pawn=776`、`actor.rootComponent=456`、
    `sceneComponent.location=280`（或任务组 1 找到的 ComponentToWorld 偏移）），并**记录当前
    `profile_hash`**。启动时用 `anomaly.ue5.build.profile_hash` 比对，**不一致 → disabled**
    （防止"pattern 恰巧还匹配但偏移已变"）。
  - [ ] 2.4 在 `CameraViewPointDetour`（渲染线程）里 walk 链读 X：无分配、无锁、无日志；每帧最多读
    一次（沿用现有 `kPresenterSameFrameNs` 去重）；读失败 → 该帧回退，计数器 +1。
  - [ ] 2.5 接入 borrow：当前 `AdvancePresenter` 里 `raw_est_cm = game_eye_cm − game_offset_cm`；
    新逻辑 = **X 有效时 `raw_est_cm = X`**，无效时回退现有 game_eye 估计。`borrow_strength` 滑块语义
    保留（0 = 纯模型、1 = 完全锚定），`game_offset_cm` 的滞后跟踪机制仅服务于回退分支。
  - [ ] 2.6 界面状态行：显示当前机制 `X-anchored / borrow / disabled / hash-mismatch / resolve-failed`
    之一 + 回退计数（满足用户"不要完全静默"的要求；不弹窗）。
- **验收**：构建通过（`./build-mingw.sh --run-tests` 全绿 + 无 warning）；面板状态行正确切换；
  人为改错一个偏移能观察到 `hash-mismatch`/回退而不崩。

### 📦 任务组 3：定量验收 + 故障注入

- **难度评级**：⭐⭐ [中等]
- **目标**：拿到 1.5 的三条定量数据，并证明版本保险真实生效。
- **具体行动项**：
  - [ ] 3.1 直行 + 漂移各跑一局，记录「相机相对车身位移速度 最高」（每帧只取第一次 detour 调用，
    见 PLAN §4.1 的"最低 0.0 是伪影"教训）、「相对转向峰值」。
  - [ ] 3.2 对照 1.5 门槛，给出"达标 / 有改善未达标 / 无改善"的初步结论（最终以用户肉眼为准）。
  - [ ] 3.3 故障注入：临时把 `profile_hash` 期望值改错 / 把 GWorld pattern 改错 / 把偏移改错，
    验证各自触发 disabled/hash-mismatch/resolve-failed + 回退 borrow + 状态行正确 + 不崩。
- **验收**：三组数据 + 三组故障注入结果，汇成给用户的简报。

### 📦 任务组 4：用户最终确认（终审）

- **难度评级**：⭐ [简单（但决定权在用户）]
- **目标**：把构建交给用户，由用户肉眼确认"无可见抖动"，并执行三档结局。
- **具体行动项**：
  - [ ] 4.1 出构建（`./build-mingw.sh --install "<游戏根目录>"`），给出如何看面板状态行的说明。
  - [ ] 4.2 用户实测后三档结局：
    - ① 达标且用户确认无可见抖动 → 默认启用（带状态行 + 手动开关）。
    - ② 有改善但未达标 → 作为可选手动开关保留，默认仍 borrow。
    - ③ 无改善或更差 → 回退 borrow，把"X 锚定"记入已证伪清单（PLAN §5 风格），**冻结 hook 路线
      不再投入**——此结论必须用户拍板。
- **验收**：用户书面确认终审结果。

### 📦 任务组 5：清理与文档

- **难度评级**：⭐ [简单]
- **目标**：收尾，不留死代码和过期文档。
- **具体行动项**：
  - [ ] 5.1 删除/归档探针临时代码（若探针要保留为诊断面板常驻功能，则明确标注用途）。
  - [ ] 5.2 更新 README（新增状态行、版本保险说明）、本计划文档、`NOTES-render-thread-hook.md`
    标注"已被 PLAN 取代"。
  - [ ] 5.3 "待触发"清单记录：若将来需要，才把速度/朝向/yaw_rate 迁移到渲染帧差分（Q2 的第二步）。
  - [ ] 5.4 若结局③：把结论写进 PLAN-camera-timing.md 的"已证伪路线"表。
- **验收**：文档一致；`git status` 无未跟踪的临时文件（`nte-game-analysis/` 大文件仍不提交）。

---

## 附录 A：符号与偏移速查（执行 AI 直接从这抄，不要再去猜）

**ue5.GWorld pattern**（`profiles/nte/nte-current.json`）：
```
48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01
resolve: kind=rip-rel32, offset=3, instructionSize=7
module=HTGame.exe, section=.text
```

**偏移链**（宿主 `ReadCurrentPlayerLocation` 同款，已验证）：
```
world.gameInstance        = 560
gameInstance.localPlayers = 56     # TArray：data@+56，count@+64
localPlayer.controller    = 48
controller.pawn           = 776
actor.rootComponent       = 456
sceneComponent.location   = 280     # == boundsOrigin，3× double（24 字节）
sceneComponent.boundsExtent = 304
sceneComponent.mobility   = 244
controller.cameraManager  = 896
cameraManager.location    = 880
cameraManager.rotation    = 904
cameraManager.fov         = 928
```

**相机 hook pattern（`kCameraViewPointPattern`）**：已在 `plugin.cpp` 里，勿改。

## 附录 B：版本保险的服务依赖

- `anomaly.interop.signature`（V1 `resolve`）：已用。运行时扫**已解密的内存 .text**，按 VA 命中。
- `anomaly.core`（`read_memory`）：已用。实现是 `ReadProcessMemory(自身进程)`，线程安全；探针在渲染
  线程调用需用栈缓冲、无分配无锁（反模式 7）。
- `anomaly.ue5.build`（**本期新增，optional**）：`build_id`（游戏模块指纹）、`profile_hash`（= profile
  JSON 的 SHA-256，偏移一变 hash 必变）、`feature_state`。用它做硬编码偏移的门禁。
- `resolve_layout`（signature V2）：宿主有，但需要调用方自带 witness pattern（插件没有），**用不上**，
  不要在这上面花时间。

## 附录 C：anomaly-cli 诊断命令（交互式验证用）

```powershell
# PID 记在 Anomaly\logs\anomaly-runtime.log
.\anomaly-cli.exe --pid <PID> status
.\anomaly-cli.exe --pid <PID> ue                    # 活动 Profile、符号解析结果
.\anomaly-cli.exe --pid <PID> scan . .text "48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01"
.\anomaly-cli.exe --pid <PID> chain <base> <off...> # 逐级解引用+偏移，最多 64 级
.\anomaly-cli.exe --pid <PID> ptr <addr>            # 读一个指针
.\anomaly-cli.exe --pid <PID> f64 <addr> [count]    # 读 double（默认 3 个）
.\anomaly-cli.exe --pid <PID> read <addr> <size>    # 读原始字节（<=1048576）
.\anomaly-cli.exe --pid <PID> sections .            # 看运行时节区名（应是 .text，非磁盘的 .std）
```

`.sections .` 应显示运行时为 `.text`（若显示 `.std`，说明解密/节区名恢复有变，立即汇报）。

## 附录 D：纪律与反模式（执行时对照）

沿用 PLAN-camera-timing.md §10（节选最相关的）：

- 禁止在 render 路径分配内存、加锁、写日志（探针/读 X 必须栈缓冲 + 原子变量）。
- 禁止静默降级：每一次 fallback 都必须有计数器（版本保险的回退计数）。
- 禁止把渲染时刻和 game tick 时刻混用在同一次坐标变换里（X 只进锚点，速度/朝向仍走 game tick）。
- 禁止平滑世界坐标（X 是锚点真值，不要再对 X 做低通）。
- 一次只改一个变量；每个结论都要有面板数字支撑。
- 不要修改游戏目录内容；`nte-game-analysis/` 大文件不要提交进 git。

---

## 附录 E：任务组 0 执行记录（2026-11-04）

### E.1 偏移链（anomaly-cli，pid 19036，游戏运行中）

| 步骤 | 结果 |
|---|---|
| GWorld slot | `0x14FCD2818`（与 profile 的 `ue5.GWorld` 地址一致） |
| GWorld 指针 | `0xC99A52D0` = 宿主 `worldPointer` 3382334160 |
| gameInstance | `0xEE037810`；`localPlayers` data `0xEE047800`，count = 1 |
| pawn → root + 280 | `[-171452.6, 88179.2, 7634.9]` cm，随车动 |

结论：附录 A 的偏移表与当前构建完全一致。

### E.2 探针实现

`CameraViewPointDetour` 里每帧最多一次（8 ms 去重）读 `root + 280`，与最近一次
game-tick 的 `player.position` 快照相减；game tick 上用同一条链读一遍作为对照组。
面板 4 行（`X GWorld / X 同链 / X 相位差 / X 沿行进`），无分配、无自有锁、无日志。

### E.3 实测（两局，0.5 s 窗口）

| 场景 | 同链对照 | 相位差 均/峰 | 换算成时间 均/峰 | 沿行进 | 车速 | 样本 |
|---|---|---|---|---|---|---|
| 高速直线 | 0.00 cm / 0 | 23.8 / 83.9 cm | **5.9 / 20.8 ms** | +23.8 cm / +5.9 ms | 145 km/h | n=50 |
| 持续漂移 | 0.00 cm / 0 | 17.2 / 79.2 cm | **7.3 / 33.1 ms** | +17.2 cm / +7.3 ms | 86 km/h | n=31 |

`fails = 0`。两次都是 `gt = n/n`（**所有样本的读取线程 == `on_update` 线程**）。

### E.4 判定

1. **同字段成立**：沿行进分量与相位差均值完全相等（23.8/23.8、17.2/17.2），
   差值 100% 在行进方向上 → 是时间相位，不是读错字段；同链对照恒为 0 也证明
   偏移链正确。
2. **相位幅度是 0 ~ 1 个多 tick，不是计划预期的 1~2 个 tick**：均值约半个 tick
   （5.9~7.3 ms），峰值 20.8~33.1 ms（约 1~2 个 tick，取决于当帧 tick 间隔）。
   远大于止损线（~1 cm），所以**不触发止损**；但也不足以直接宣布
   “1~2 tick 的渲染相位”。
3. **重大偏差：detour 在游戏线程上，不是渲染线程。** `gt = n/n` 且
   `PluginManager::GameUpdate` 由 `GameTickHook` 的回调驱动（宿主自己用
   `game_thread_id` 拒绝其他线程的 tick），所以 `on_update` 是游戏线程；
   detour 与它同线程。UE 的相机姿态 getter 本来就是在游戏线程的相机更新里调用的，
   渲染线程用的是缓存好的 `FMinimalViewInfo`，不会再调这个 getter。
   因此 **A1 能拿到的是“相机计算那一刻的游戏线程车位置”，不是“渲染线程画车用的
   SceneProxy / ComponentToWorld”**。
4. 这不必然否定 X 锚定：相机与锚点只要在**同一时刻**采样，帧间相位就消除了。
   任务组 2 的目标应从“拿渲染线程的 X”改为“把锚点换成 detour 时刻读到的 X”。
   但这是对 §1.2 宏观架构（Q1/A1 的“渲染线程”前提）的实质修改，按 §3 需用户 /
   规划 AI 拍板，执行 AI 不自行决定。

### E.5 待决策

- **(A)** 继续任务组 2，X 重新定义为“detour 时刻的 `root + 280`”（推荐：改动最小，
  直接消掉 0~1+ tick 的采样相位，仍符合“X 只作位置锚点”）；
- **(B)** 改走 A2：找渲染线程专用 hook 拿 SceneProxy / ComponentToWorld（任务组 1，
  成本高，且需要新的 pattern）；
- **(C)** 冻结 hook 路线（必须用户拍板）。
