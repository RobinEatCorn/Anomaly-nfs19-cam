# 🚀 任务实施计划 v2：同刻锚点 → 渲染器变换（交接给执行 AI）

> **v2 修订（2026-11-05）**：任务组 0 探针实测推翻了 v1 的"detour 在渲染线程"前提（详见附录 E）。
> A1 已重定义为**「同刻锚点」**（detour 时刻的 `root+280`），路线改为 **(A) 同刻锚点先行，
> 未消抖则升级 (B) 渲染器变换逆向**。
> 取代 v1（本文件历史版本）与 `NOTES-render-thread-hook.md`（保留作来源）。
> 配套背景：`PLAN-camera-timing.md`（§9 验收指标、§10 反模式、§11 设计原则仍然有效）。
> 基线：`nfs2015-chase-cam` HEAD = `22f7827`（探针 + 差异报告已提交）。

---

## 1. 目标与宏观共识

### 1.1 全局目标

消除行驶中**"车抖、相机不抖"**的剩余抖动。根因（已二分确认）：相机用 game tick 采样的
`player.position` 计算，与游戏画车用的车位置不同步；且从游戏相机位置反推 X 数学上不可逆
（相机弹簧是低通，高频信息已丢）。

### 1.2 宏观共识（规划 AI 与用户达成，执行 AI 不得更改）

- **D0**：走 hook 路径。必须带版本保险（游戏更新后插件失效、**不能崩**）。
- **D1**：不改宿主（Anomaly）源码，全部在插件内完成（`get_object_handles`+`invoke` 都在 game
  thread，升级 main 解决不了抖动）。
- **R0（前提修正，探针实测）**：`CameraViewPointDetour` 跑在**游戏线程**，与 `on_update` 同线程
  （`gt = n/n`，见附录 E.4）。因此"渲染线程 X"作废；**A1 = 同刻锚点** = detour 被调用那一刻读到的
  `root+280`。真正"渲染器画车用的变换"只属于 A2。
- **R1（路线）**：先做 **(A) 同刻锚点**——X = detour 时刻 `root+280`，替换 borrow 的估计目标，
  加 A/B 开关对比。探针已就位，成本最小。
- **R2（升级条件）**：(A) 实测未消抖（未达 1.5 门槛，或用户仍见抖）→ 升级 **(B) A2 渲染器变换逆向**。
- **R3（验收口径）**：1.5 的定量门槛维持不变（是"可见抖动"的代理）；**最终成功必须由用户亲自肉眼
  确认"无可见抖动"**，执行 AI 不得自行宣布成功。
- **Q2（X 的用途 B1）**：X **只作位置锚点**，速度/朝向/yaw_rate/slip 仍走 game-tick 差分
  （低频弹簧通道，1 tick 相位差影响远小于位置）。本期**不**迁移速度/朝向到渲染帧差分（记入"待触发"）。
- **Q3（版本保险三层）**：① 运行时 resolve 全部所需 pattern；② 硬编码偏移表 + `profile_hash` 门禁；
  ③ 失败回退现有 borrow，界面显示当前机制（用户要求：**不静默**，面板有状态行），不弹窗不崩。
- **Q4（成败判定）**：定量门槛只是执行 AI 自我检查线；用户肉眼是终审。

### 1.3 关键发现（执行 AI 必读）

1. **磁盘二进制受保护**：`HTGame.exe`/`HTGameBase.dll` 节区名全为 `.std`，文件 55.4% 零字节，
   相机函数字节码在磁盘上搜不到，游戏目录有 `AntiCheatExpert`。→ 磁盘静态反汇编是死路，只能
   **运行时 dump 后分析**。
2. **运行时读内存不受影响**：游戏启动即解密，`signature.resolve` 扫已解密的 `.text`（现有插件
   hook 成功为证）。
3. **`anomaly-cli` 可交互验证偏移链**（附录 C）。
4. **宿主 `player.position` = `root+280` = `Bounds.Origin`（包围盒中心）**，不是渲染变换
   `ComponentToWorld`/SceneProxy。
5. **偏移链已验证正确**（附录 E.1），不是偏移错误。
6. **detour 在游戏线程**（附录 E.4）：UE 的相机视点 getter 在游戏线程相机更新里调用，渲染线程用
   缓存好的 `FMinimalViewInfo`，不会再调这个 getter。"每帧 27 次"是游戏线程 tick 内的多次查询，
   不是渲染线程多 pass。
7. **相位差实测**（附录 E.3）：均值 17.2~23.8 cm（约半个 tick，5.9~7.3 ms）、峰值 79.2~83.9 cm
   （约 1~2 tick，20.8~33.1 ms），100% 沿行进方向、同链对照恒 0 → **纯时间相位，同字段成立**。
8. **borrow 已做过"同刻估计"**：`38c98ef` 的 `raw_est = game_eye − lowpass(game_eye − our_car)`。
   (A) 不是新方向，而是**把估计目标换成同一时刻的直接读数**（省掉 lowpass 滞后）。

### 1.4 假设（v2，修正后）

> **H-A（同刻锚点，本期验证）**：相机锚点与 detour 读取在**同一时刻**，即可消除宿主采样滞后
> （均值 ~6 ms、17~24 cm）。若渲染侧滞后（相机计算→渲染绘制）不可见，则 H-A 足以消抖。
>
> **H-B（渲染侧滞后，H-A 不足时验证）**：渲染器画车用的是 `ComponentToWorld`/SceneProxy，其与
> 游戏线程相机计算之间还有一段滞后；若 H-A 实测未消抖，说明这段滞后可见，需走 A2。

### 1.5 定量检查线（执行 AI 自我检查，不是最终验收）

| 检查项 | 阈值 | 说明 |
|---|---|---|
| 相对速度峰值（漂移） | < 5 m/s | 现状 borrow 24.7 m/s；PLAN §9 验收线 |
| 相对速度峰值（直行） | < 1 m/s | 现状 2.3 m/s |
| 相对转向峰值（漂移） | < 90 deg/s | 现状 280 deg/s（PLAN §9） |

> 原 v1 的"探针相位差 < ~1 cm 止损"已过时（实测 17~24 cm，不触发）。v2 的**止损/升级线**是：
> (A) 实测未达上表 → 升级 (B)； (B) 也做不出或无效 → 冻结（必须用户拍板）。

---

## 2. 微观决策授权范围（执行 AI 自行决定）

- 反汇编/分析工具选择（dump 后 Ghidra / IDA / objdump / capstone）。
- 诊断面板行命名、文案、配色、阈值显示格式；A/B 开关在 UI 里的呈现方式。
- 偏移表在代码里的组织方式（常量表 / 结构体 / 宏）。
- 读链的缓存策略（是否缓存 `pawn→root` 指针、失效条件、每帧节流）。
- blend 数学、EMA 系数、重试间隔、采样窗口等数值。
- commit 拆分与 message；单测写法（按 `tests/` 现有风格）。
- 附录 A 未列的、与宏观架构无关的实现细节。

---

## 3. ⚠️ 异常汇报机制（重要）

执行中发现与计划明显出入、或遇到影响全局的重大阻碍，**立即停止并向用户汇报**，由用户转述规划 AI
重新规划。包括但不限于：

- (A) 同刻锚点实测未达 1.5 门槛（这是升级 B 的触发，但升级前**先汇报数据结论**）；
- 游戏线程读 `core->read_memory` 被 AntiCheatExpert 拦截 / 崩溃 / 卡顿；
- `ue5.GWorld` pattern 运行时 resolve 失败（区分 bug 与真的版本失效）；
- A2 逆向中找不到稳定的 `ComponentToWorld`/SceneProxy 读取点；
- **冻结 hook 路线（结局③）必须由用户拍板，执行 AI 不得自行冻结**。

**最终成功判定权在用户**：定量达标只是必要条件；用户亲测"肉眼无可见抖动"才算成功。

---

## 4. 任务步骤分解

> 任务组 0 已完成。依赖：1 → 2（判定）→ 4（终审）；3 仅在 2 判"未消抖"时触发。

### 📦 任务组 0：相位差探针 ✅ 已完成

- 结果见附录 E 与 `REPORT-task0-discrepancy.md`。结论：同字段成立、纯时间相位、**detour 在游戏
  线程**、偏移链正确。探针提交 `8ae80f5`。

### 📦 任务组 1：实现「同刻锚点」(A) + 版本保险 + A/B 开关

- **难度评级**：⭐⭐⭐ [困难]
- **目标**：把探针的读链提升为正式锚点路径，落实三层版本保险 + 界面状态行 + A/B 开关。
- **具体行动项**：
  - [x] 1.1 在 `AdvancePresenter` 里接入 X：X 有效时 `raw_est_cm = X`（detour 时刻 `root+280`，
        复用 `TryResolveRenderX`/`WalkPlayerChain`），无效时回退现有 `game_eye − game_offset_cm`
        估计。`borrow_strength` 滑块语义保留（0=纯模型、1=完全锚定）；`game_offset_cm` 滞后跟踪
        仅服务于回退分支。
  - [x] 1.2 版本保险：`manifest.json` 增加可选 `anomaly.ue5.build`（`optional:true`），`capabilities`
        同步；启动用 `profile_hash` 门禁硬编码偏移表；`resolve` 失败 / hash 不匹配 → 新路径 disabled。
        注意 SDK 头文件已是 v2.5.0 一致版本（PLAN §7.7 教训）。
  - [x] 1.3 A/B 开关：新增「锚点来源」开关（`X 锚定 / borrow 估计 / 纯模型`），供任务组 2 对比实验。
        **默认值留到用户终审后再定**（见任务组 4），实验期默认"X 锚定（保险通过时）"。
  - [x] 1.4 界面状态行：`X-anchored / borrow / disabled / hash-mismatch / resolve-failed` 之一 +
        回退计数（用户要求不静默；不弹窗）。
  - [x] 1.5 读链纪律：每帧最多读一次（沿用 8 ms 去重）、栈缓冲、无分配无锁无日志；读失败计数 +1，
        该帧回退。
- **验收**：`./build-mingw.sh --run-tests` 全绿无 warning；状态行随开关正确切换；人为改错偏移 /
  hash / pattern 各自触发对应降级且不崩。
- **执行记录（2026-11-05）**：
  - 1.1–1.5 已实现：`AdvancePresenter` 按 `anchorSource` 选择 X / borrow / model；X 复用
    `SampleRenderXPhase` 的每 8 ms 单次读；`root+280` 在模型空间补 `originOffsetCm`（Z）。
    `borrow_strength` 仍是 0=纯模型、1=完全锚定；X 模式下不再跟踪 `game_offset_cm`，回退时重新初始化。
  - 版本保险：`manifest.json` 增加可选 `anomaly.ue5.build` + `ue5-build`；`kExpectedProfileHash`
    为实测 runtime profile `06e1b79ceb4b9a778cb2b88357fea9ac2a6c34e06ba759d3b62048a979fd11bd`；
    `X 门禁` 状态显示 `ok/hash/pattern/service/offset/off/read`，主窗口 `锚点` 行显示五态 + `fb` 计数。
  - 构建/测试：`./build-mingw.sh` 干净；`nfs2015_chase_cam_test.exe` 全绿；`anomaly-test-host
    --reload 2 --ticks 60` 通过（TestHost 没有 build 服务，验证 service-missing 回退不崩）。
    本机没有 wine，`--run-tests` 未用，验收夹具直接在 Windows 侧运行。
  - 待游戏内验证：安装后看 `锚点`、`X 门禁`、`X GWorld`、`X 回退/读错`；任务组 2 的三档对比。

### 📦 任务组 2：A/B 实测与判定

- **难度评级**：⭐⭐ [中等]
- **目标**：用数据回答"直接读数 vs 相机估计 谁更好、够不够消抖"。
- **具体行动项**：
  - [ ] 2.1 直行 + 漂移各跑，三种「锚点来源」分别记录「相机相对车身位移速度 最高」（每帧只取第一次
        detour 调用，PLAN §4.1 教训）、「相对转向峰值」。
  - [ ] 2.2 对照 1.5 门槛，得出 (A) 的结论：达标 / 有改善未达标 / 无改善。
  - [ ] 2.3 判定：达标或明显优于 borrow → 进任务组 4；未达标 → 把数据写成升级 B 的证据，进任务组 3。
- **验收**：三组对比数据 + 明确判定结论。

### 📦 任务组 3（条件触发）：A2 渲染器变换逆向

- **难度评级**：⭐⭐⭐ [困难]
- **触发条件**：任务组 2 判 (A) 未消抖（未达 1.5 门槛，或用户仍见抖）。
- **目标**：拿到渲染器实际画车用的变换（`USceneComponent::ComponentToWorld` / SceneProxy /
  `FScene::UpdatePrimitiveTransform` 等），给出稳定读取点或专用 hook。
- **具体行动项**：
  - [ ] 3.1 做**运行时内存 dump**（磁盘是加密的，不能直接反汇编磁盘文件）。
  - [ ] 3.2 用 UE5 公开结构知识 + dump 反汇编定位 `ComponentToWorld` 偏移（线索：`location/boundsOrigin
        =280`、`boundsExtent=304`、`mobility=244`；96 字节 FTransform，可用"平移≈车位置 + 四元数 W≈1"
        在 dump 里反查确认），或找渲染线程更新路径的 hook 点。
  - [ ] 3.3 实现读取 + 复用任务组 1 的版本保险与状态行。
- **验收**：确定的渲染器变换读取路径 + 相位/消抖证据。

### 📦 任务组 4：用户最终确认（终审）

- **难度评级**：⭐ [简单（但决定权在用户）]
- **目标**：用户肉眼确认"无可见抖动"，执行三档结局。
- **具体行动项**：
  - [ ] 4.1 出构建（`./build-mingw.sh --install "<游戏根目录>"`）+ 面板状态行说明。
  - [ ] 4.2 三档结局（**用户拍板**）：
    - ① 达标且用户确认无可见抖动 → 默认启用（状态行 + 手动开关）；
    - ② 有改善未达标 → 作为可选手动开关保留，默认仍 borrow；
    - ③ 无改善或更差 → 回退 borrow，记入已证伪清单，冻结 hook 路线。
  - [ ] 4.3 默认开关（on/off）在用户终审后再定（R1 遗留问题）。
- **验收**：用户书面确认终审结果。

### 📦 任务组 5：清理与文档

- **难度评级**：⭐ [简单]
- **目标**：收尾，不留死代码和过期文档。
- **具体行动项**：
  - [ ] 5.1 归档探针临时代码（若保留为诊断面板常驻功能则标注用途）。
  - [ ] 5.2 更新 README（状态行、版本保险、锚点来源开关）、本计划、`NOTES` 标注"已被 PLAN 取代"。
  - [ ] 5.3 "待触发"清单：速度/朝向/yaw_rate 迁移到渲染帧差分（Q2 第二步，仅在将来需要时）。
  - [ ] 5.4 若结局③：结论写进 PLAN-camera-timing.md "已证伪路线"表。
- **验收**：文档一致；`git status` 无未跟踪临时文件（`nte-game-analysis/` 大文件仍不提交）。

---

## 附录 A：符号与偏移速查

**ue5.GWorld pattern**（`profiles/nte/nte-current.json`）：
```
48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01
resolve: kind=rip-rel32, offset=3, instructionSize=7
module=HTGame.exe, section=.text
```

**偏移链**（宿主 `ReadCurrentPlayerLocation` 同款，已由附录 E.1 验证）：
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
- `anomaly.core`（`read_memory`）：已用。`ReadProcessMemory(自身进程)`，线程安全；探针已证明可在
  detour（游戏线程）安全调用，用栈缓冲、无分配无锁。
- `anomaly.ue5.build`（**任务组 1 新增，optional**）：`build_id`（模块指纹）、`profile_hash`
  （profile JSON 的 SHA-256，偏移一变 hash 必变）、`feature_state`。做硬编码偏移的门禁。
- `resolve_layout`（signature V2）：宿主有，但需调用方自带 witness pattern（插件没有），用不上。

## 附录 C：anomaly-cli 诊断命令

```powershell
.\anomaly-cli.exe --pid <PID> status            # PID 在 Anomaly\logs\anomaly-runtime.log
.\anomaly-cli.exe --pid <PID> ue                 # 活动 Profile、符号解析结果
.\anomaly-cli.exe --pid <PID> scan . .text "48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01"
.\anomaly-cli.exe --pid <PID> chain <base> <off...>   # 逐级解引用+偏移
.\anomaly-cli.exe --pid <PID> ptr <addr> / f64 <addr> [count] / read <addr> <size>
.\anomaly-cli.exe --pid <PID> sections .         # 运行时节区名（应为 .text，非 .std）
```

## 附录 D：纪律与反模式（执行时对照）

- 禁止在相机计算路径分配内存、加锁、写日志（读 X 必须栈缓冲 + 原子变量）。
- 禁止静默降级：每次 fallback 必须有计数器（版本保险回退计数）。
- 禁止把不同采样时刻的量混用在同一次坐标变换里（X 只进锚点，速度/朝向仍走 game tick）。
- 禁止平滑世界坐标（X 是锚点真值，不再对 X 做低通）。
- 一次只改一个变量；每个结论都要有面板数字支撑。
- 不修改游戏目录内容；`nte-game-analysis/` 大文件不提交进 git。

## 附录 E：任务组 0 执行记录（2026-11-04，保留备查）

### E.1 偏移链（anomaly-cli，pid 19036）
GWorld slot `0x14FCD2818`；GWorld 指针 `0xC99A52D0`（== 宿主 worldPointer）；gameInstance
`0xEE037810`；`localPlayers` data `0xEE047800` count=1；`pawn → root+280` = `[-171452.6, 88179.2,
7634.9]` cm 随车动。→ 附录 A 偏移表与当前构建一致。

### E.2 探针实现
`CameraViewPointDetour` 里每帧最多一次（8 ms 去重）读 `root+280`，与最近 game-tick 的
`player.position` 快照相减；game tick 上用同链读一遍作对照。面板 4 行，无分配、无自有锁、无日志。
提交 `8ae80f5`。

### E.3 实测（两局，0.5 s 窗口）

| 场景 | 同链对照 | 相位差 均/峰 | 时间 均/峰 | 沿行进 | 车速 | 样本 |
|---|---|---|---|---|---|---|
| 高速直线 | 0.00 cm | 23.8 / 83.9 cm | 5.9 / 20.8 ms | +23.8 cm | 145 km/h | n=50 |
| 持续漂移 | 0.00 cm | 17.2 / 79.2 cm | 7.3 / 33.1 ms | +17.2 cm | 86 km/h | n=31 |

`fails = 0`；两次 `gt = n/n`（所有采样线程 == `on_update` 线程）。

### E.4 判定（已并入 §1.2 R0 / §1.3）
同字段成立（相位 100% 沿行进、同链对照恒 0）；相位幅度 0~1+ tick（非预期 1~2 tick）；**detour 在
游戏线程**（`on_update` 由 `GameTickHook` 驱动，宿主用 `game_thread_id` 拒绝其他线程）。

### E.5 决策（已由规划 AI + 用户拍板）
采纳 (A) 同刻锚点先行，未消抖升级 (B) A2；验收口径维持 §9 + 用户肉眼终审。见 §1.2 R1/R2/R3。
