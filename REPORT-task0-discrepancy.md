# 任务组 0 差异报告：detour 在游戏线程，相位是 0~1+ tick

> 交给规划 AI。结论、证据、对原计划的影响、需要重新拍板的点，都在这里。
> 对应计划：`PLAN-render-thread-hook.md`（任务组 0 / §1.2 Q1 / §1.4 H / §1.5）。
> 实现提交：`8ae80f5`（探针）、`54a1372`（计划附录 E）。基线 `38c98ef`。

---

## 0. 一句话

探针实测：**`CameraViewPointDetour` 的调用线程 == `on_update` 线程 == 游戏线程**
（两局都是 `gt = n/n`），不是计划假设的渲染线程。所以 A1 能拿到的是
**「相机计算那一刻的游戏线程车位置」**，不是「渲染线程画车用的
`SceneProxy` / `ComponentToWorld`」。同字段结论成立，相位是纯时间的：
均值约半个 tick（5.9~7.3 ms），峰值 1~2 个 tick（20.8~33.1 ms）。

---

## 1. 计划前提 vs 实测

| 计划原文 | 位置 | 实测 | 偏差 |
|---|---|---|---|
| detour 在**渲染线程** | §1.2 Q1、§1.4 H、§4 任务组 0.2 | 在**游戏线程**（`gt = n/n`） | **重大**，推翻 A1 的“渲染线程 X”定义 |
| 渲染线程读到的 X 比宿主快照新 **1~2 个 game tick** | §1.4 H | 均值 5.9~7.3 ms（约半个 tick），峰值 20.8~33.1 ms（约 1~2 tick） | 部分成立：峰值对，均值偏小 |
| 相位差应达 **几十 cm ~ 1 m+** | §1.5 | 均值 17.2~23.8 cm，峰值 79.2~83.9 cm | 峰值达到，均值低于预期 |
| 相位差 ≈ 0 → 异字段；< ~1 cm → 止损 | §4 0.3 | 都不是：**同字段、纯时间、0~1+ tick** | 需要新增第三种判定分支 |
| `raw_est_cm = X`（X 有效时） | §2.5 | 仍可做，但 **X 的定义要改成「detour 时刻的 `root + 280`」** | 语义修改，机制不变 |

---

## 2. 实测数据

### 2.1 偏移链（0.1，`anomaly-cli`，pid 19036，游戏运行中）

| 步骤 | 结果 |
|---|---|
| GWorld slot | `0x14FCD2818`（与 profile `ue5.GWorld` 地址一致） |
| GWorld 指针 | `0xC99A52D0` = 宿主 `worldPointer` 3382334160（逐位相同） |
| gameInstance | `0xEE037810`；`localPlayers` data `0xEE047800`，count = 1 |
| pawn → root + 280 | `[-171452.6, 88179.2, 7634.9]` cm，随车动 |

结论：附录 A 的硬编码偏移表与当前构建**完全一致**，本报告的数据不是偏移错误造成的。

### 2.2 探针（0.2，面板 `X` 四行，0.5 s 窗口）

| 场景 | 同链对照 | 相位差 均/峰 | 时间 均/峰 | 沿行进 | 车速 | 样本 |
|---|---|---|---|---|---|---|
| 高速直线 | 0.00 cm / 0 | 23.8 / 83.9 cm | **5.9 / 20.8 ms** | +23.8 cm / +5.9 ms | 145 km/h | n=50 |
| 持续漂移 | 0.00 cm / 0 | 17.2 / 79.2 cm | **7.3 / 33.1 ms** | +17.2 cm / +7.3 ms | 86 km/h | n=31 |

`fails = 0`；两次都是 `gt = n/n`（所有采样都在 `on_update` 线程上）。

### 2.3 判定

1. **同字段成立**：沿行进分量与相位差均值完全相等（23.8=23.8，17.2=17.2），
   差值 100% 在行进方向；同链对照恒为 0。
2. **不触发止损**：远大于 ~1 cm。
3. **不是“≈0 的异字段”**：相位确实存在，而且随采样时刻变化。
4. **相位幅度是 0~1+ tick**：均值约半个 tick，峰值 1~2 tick。
   这正是“宿主在 tick 上采样、相机在 tick 内某个时刻计算”的相位分布。

---

## 3. “渲染线程”这个前提是怎么来的

这不是执行 AI 新引入的假设，而是从 NOTES 一路继承下来的，历史里有两次机会可以验证：

| 时间 | 会话 / 证据 | 发生了什么 |
|---|---|---|
| 11-02 18:25 | `2026-11-02T17-42-37` | 已提出“照着 NTE 源码走 GWorld 链，在 detour 里读 `sceneComponent.location`”。同一段里**明确怀疑过线程**：“UE 的 CameraManager 在渲染线程被调用吗？实际 UE 中…通常在 game thread 调用。但历史说我们的 detour 有多个线程，每帧约 27 次，这更像是渲染线程的多个 pass。”最终按“27 次/帧、多 pass”推断为渲染线程。 |
| 11-04 14:48 | `2026-11-04T14-20-35` | 规划时明确写下 “Let me now also verify one more critical fact: is the CameraViewPointDetour actually called on the render thread? The NOTES claims ‘渲染线程调用’.” 但只看了 `detour_thread_count`（**只数线程个数，不看线程身份**），随后写成 “The diagnostic shows the thread count calling the hooked getter, which is the render thread… The NOTES already confirms this.” |
| 11-04 15:07 | 同上 | 已经分析出 `280 == boundsOrigin`、`ComponentToWorld` 可能在 0xF8/0x118；也预见了“如果两处读的是同一块内存，可能没有相位差”。但没有实际测量。 |

补充：SDK 里一直有 `anomaly.ue5.framework::is_game_thread`；manifest 在 `a571f5f` 里加过
`game-events` capability，`31bb7b9` 又删了，代码从未调用过它。所以“detour 是哪个线程”
**从来没有被实际测量**，`gt` 是第一次。

> 结论：想法提过、计划写过，但**实现和验证都没做过**。不是重复劳动。

---

## 4. 宿主侧证据：为什么这是必然，而不是探针 bug

1. `PluginManager::GameUpdate`（插件 `on_update` 的入口）由嵌入桥的 tick 回调调用：
   `Anomaly/src/render/dx12/embedded_bridge.cpp:856`。
2. 该 tick 回调来自 `Ue5NteAdapter::OnGameTick`，由 `GameTickHook`
   （`ue5.GameTick` 的 MinHook detour）在**游戏线程**调用；宿主自己在
   `OnGameTick` 里把首次调用线程记为 `game_thread_id`，并拒绝其它线程
   （`rejected_thread_ticks`）。
3. 所以 `on_update` = 游戏线程。探针在 detour 里比较
   `GetCurrentThreadId() == on_update 的线程 id`，两局都是 `n/n`。
4. 这与 UE 的行为一致：`APlayerCameraManager::GetCameraViewPoint` 在游戏线程的
   相机更新里调用；渲染线程使用的是已经算好的 `FMinimalViewInfo`，不会再调这个 getter。
   “每帧 27 次”只说明游戏线程在 tick 内多次查询相机，不能推出渲染线程。

---

## 5. 对路线的影响

### 5.1 A1 不是“渲染线程 X”

- A1 能拿到的是：**`root + 280` 在 detour 调用时刻的值**，即“相机计算那一刻的车位置”。
- 它仍然有意义：相机和锚点在同一时刻采样，可以消除 0~1+ tick 的**采样相位**
  （也就是现在测到的 17~24 cm 均值、79~84 cm 峰值）。
- 它不能保证拿到渲染器**实际画车**用的变换：UE 渲染器用的是
  `ComponentToWorld` / scene proxy 的快照，不是 `Bounds.Origin`。
  计划 §1.3(4) 自己已经指出“280 是 `Bounds.Origin`”。

### 5.2 真正拿“渲染器变换”只能走 A2

- `ComponentToWorld` / SceneProxy / `FScene::UpdatePrimitiveTransform`
  等渲染线程路径，需要新的 pattern + 反汇编（任务组 1 / A2）。
- 成本明显高于 A1，且版本敏感。

### 5.3 已有工作：borrow 已经做了“同一时刻锚点”的估计版

`38c98ef`（11-03）的 borrow 已经是：

```text
raw_est = game_eye − lowpass(game_eye − our_car)
```

它在 detour 时刻用游戏相机位置**估计**“游戏画的车在哪”，再锚过去。
所以 A1 不是全新方向，而是**把 borrow 的估计目标换成同一时刻的真实读数**：

- 增量小，可快速验证；
- 如果无效，才说明渲染器用的是 `ComponentToWorld`，再进任务组 1；
- 如果有效，`borrow_strength` 的滑块语义可以保留（0 = 纯模型，1 = X 锚定）。

---

## 6. 建议（供规划 AI 决定）

| 方案 | 内容 | 成本 | 风险 |
|---|---|---|---|
| **(A) 推荐** | 任务组 2 改为“同刻锚点”：X = detour 时刻 `root + 280`；`raw_est_cm = X`；保留 borrow 回退；加开关做 A/B | 最小（探针已就位） | X 可能只是同一字段的 later read，改善幅度可能有限 |
| **(B)** | 走任务组 1 / A2：找渲染线程 `ComponentToWorld` / SceneProxy 的 hook 点 | 高（新 pattern + 逆向 + 版本保险） | 版本敏感、可能找不到稳定点 |
| **(C)** | 冻结 hook 路线 | — | 必须用户拍板 |

无论选哪个，计划文档里“渲染线程 X”的措辞建议改成：

- A1 的 X = **detour 时刻的游戏线程车位置**；
- “渲染线程变换”只属于 A2 / 任务组 1。

---

## 7. 需要规划 AI 明确的开放问题

1. 在“X 只是同一字段的 later read”前提下，§9 的验收门槛（直行 <1 m/s、漂移 <5 m/s）
   是否仍然适用？
2. 是否接受把 A1 重新定义为“同刻锚点”，而不是“渲染线程 X”？
3. 如果 A1 只能拿到均值 6~7 ms 的相位改善，是否值得作为默认开启，还是只保留手动开关？
4. 任务组 1 的触发条件是否从“相位 ≈ 0”改成“相位沿行进方向且均值 < 某个 ms 阈值”？
5. 冻结路线（结局③）是否仍然必须由用户拍板？

---

## 附录：可直接复现的证据

- 探针实现：`plugin.cpp`
  - `RenderXProbe`、`TryResolveRenderX`、`WalkPlayerChain`、`ReadChainLocation`、
    `UpdateRenderXProbe`、`SampleRenderXPhase`；
  - 常量：`kGWorldPattern`、`kWorldGameInstanceOffset` … `kSceneComponentLocationOffset`。
- 面板行：`diag.render_x_state / chain / phase / lead`；
  `gt=%llu/%llu` 是“在游戏线程上采到的样本数 / 总样本数”。
- `anomaly-cli` 验证（可重跑）：
  ```text
  anomaly-cli --pid <PID> ptr 0x14FCD2818
  anomaly-cli --pid <PID> chain 0x14FCD2818 560 56 0 48 776 456 280
  anomaly-cli --pid <PID> f64 <上一步 address> 3
  ```
- 计划附录 E 有同样的数据（`PLAN-render-thread-hook.md`）。
