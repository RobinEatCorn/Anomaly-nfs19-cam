# Phase 2 / H1 实施报告：Z 键三档视角循环

> 版本：`2.0.4`　日期：2026-11-12　对应计划：`PLAN-zkey-phase2-h1.md`（未入库）
> 前置报告：`analysis/view-mode-recon/REPORT-tg{0..4}-*.md`（`analysis/` 不入库）
> 实机环境：NTE + Anomaly 2.7.0 Runtime，Windows 11 + WSL2，插件代次 1–25

## 1. 结论

驾驶时 `Z` 由插件接管，形成 `追车 → 普通第三人称 → 第一人称 → 追车` 的三档循环，
全部实机验收通过（除两项用户主动放弃的用例，见 §8）。**插件只新增一个 hook**
（游戏自己的 `ToggleSwitchCameraViewMode`），所有切档最终都由游戏执行，插件只在
`orig` 返回成功后记账；游戏拒绝、过场、下车、抓拍、换控制器时立即交还相机。

计划外的第三个交付：修掉 **F8 抓拍模式被追车相机覆盖**的真 bug（§6）。

| 交付 | 状态 |
|---|---|
| Z 三档循环（H1 路线） | ✅ 实机验收 |
| 上车规则（TP→追车 / FP→保持 FP） | ✅ 实机验收 |
| 移除 F7 快捷键 | ✅ 实机验收，`hotkeys_after_stop=0` |
| 抓拍/游戏自有相机让位 | ✅ 实机验收 + 逐帧证据 |
| 剧情过场（`Fixed`/`Seat`）目视 | ⏳ 用户决定不做本次验证 |
| 30 分钟长跑 | ⏳ 用户决定不做本次验证 |

## 2. 最终行为

```text
第三人称步行上车       → NFS 追车
Z（第 1 下）           → 普通第三人称（游戏原生）
Z（第 2 下）           → 第一人称（游戏原生）
Z（第 3 下）           → NFS 追车

第一人称步行上车       → 保持第一人称（不强制切档）
Z（第 1 下）           → NFS 追车
Z（第 2 下）           → 普通第三人称
Z（第 3 下）           → 第一人称
```

两种情况是同一个环，只是起点不同。三条硬规则：

1. **追车 ≠ 另一个 tag**：追车档就是"游戏处于 `CameraViewMode.ThirdPerson` + 插件写机位"。
   普通第三人称是同一个 tag 但不写机位，二者靠插件自己的 `view_active` 区分。
2. **不抢戏**：过场（`Fixed`/`Seat`）、下车、传送、换控制器、游戏自己改档、游戏拒绝、
   抓拍/游戏自有相机 —— 任何一条都让插件在当帧或当 tick 停止覆盖。玩家自己按 `Z` 退出
   追车后，插件不会自作主张再接管。
3. **fail-open**：tag 名反查不到、快照读不出来、pattern 不命中、vtable 不符 → `Z` 完全
   原生（保留 2.0.3 的追车行为），插件照常加载，只在日志里留一行 `warning`。

## 3. 实现摘要

### 3.1 唯一的 hook

```text
HTPlayerController::ToggleSwitchCameraViewMode(bool bForward)   → 0x1480BE330
pattern: 40 53 55 57 48 83 EC 30 83 B9 90 1E 00 00 00 0F B6 FA 48 8B D9
第二见证: PC->vtable[+0x15C8] == 解析结果
```

两个调用点都落在同一函数里（车内 `0x1480D4C09`、步行 `0x149939D54`），所以一个 detour
覆盖全部 `Z` 输入，不需要按调用点区分。**没有**新增渲染 hook、没有派生新 tag、
没有调用 `RegisterCameraViewModeClass`。

### 3.2 状态与门禁

| 状态 | 位置 | 语义 |
|---|---|---|
| `view_mode.tag` | `PC+0x1E80` | 游戏当前档位（`FGameplayTag`，文本比较，不硬编码 id） |
| `view_mode.limit` | `PC+0x1E98` | 游戏自己的手动切换闸门（过场时为非 0） |
| `chase_wanted` | 插件 | 玩家想不想要追车（`Z` 与上车规则共同决定） |
| `view_active` | 插件 | 插件现在是否真的在写机位（= 追车档） |

`Z` 只在"解析到切档 API + 档位可信 + 正在驾驶 + tag 是 TP/FP"时被吞；吞下后的动作：

```text
chase + TP  → 交还普通第三人称（吞 Z，不调 orig，游戏 tag 本来就不用动）
chase 之外的 TP → 调 orig（游戏 TP→FP）
FP          → 调 orig（游戏 FP→TP）+ 同一调用内 ActivateView（消除 TP 闪帧）
limit > 0   → 拒绝切换（由插件自己兜，车内包装函数没有这个检查）
```

`orig` 的返回值是权威信号：`ret=0` 表示游戏没动，插件不猜、不补。

### 3.3 每帧读 tag（不是一次解析）

TG-0 观测到游戏会在**零 `Toggle` 调用**的情况下自己改档（传送、迁移）。因此
`MonitorViewMode` 每 Game tick 读一次 `0x1E80..0x1EB0`（0x38 字节），并在渲染线程的
相机守卫里**再现场读一次**——`published_tag` 可能慢一个 tick，而一个 tick 就是一帧可见的
错误画面。守卫的判定是唯一允许写机位的条件，拦下时计 `diag.tag_veto_frames`。

### 3.4 进车规则

```text
ApplyEntryView:  chase_wanted = (游戏 tag == 第三人称)
                 pending_entry = (游戏 tag 是 Fixed / Seat / 读不到)
下车:            NoteVehicleExit → 清除 wanted / pending
进车去抖:        view_off_seconds >= 0.75s 才允许重新接管
```

去抖比"车辆存在性宽限 0.20s"大，所以 tick 抖动、短暂 `veh=0` 不会把玩家刚按 `Z` 退掉的
追车重新拉回来；同时 0.75s 也远小于"下车再上车"的间隔，上下车循环不受影响。

`pending_entry` 覆盖"上车时正好在过场里"：presenter 每 tick 重试，直到游戏回到 TP/FP
再应用一次（日志只打一次）。

### 3.5 交还路径清单

`ChaseSlotAllowed()`（游戏线程，决定是否发布 active）与 `ChaseCameraGuardAllows()`
（渲染线程，决定这一帧是否覆盖）会检查同一组条件：

1. 总开关打开；
2. 玩家在车内（presenter 维护的 `in_vehicle`，**不用 Pawn 类名判断**）；
3. `chase_wanted`；
4. 现场读到的 tag == `ThirdPerson`；
5. **控制器上没有被游戏挂上的电影/抓拍相机**（`PC+0x1760 == 0`）。

任何一条不成立 → `PublishInactive`：route B、FOV 写入、机位全部停止，游戏相机原样输出。

## 4. TG-0 结论（本次复用的关键事实）

1. **`bForward` 从来不是可靠信号**：31 次调用全是 `false`，无论玩家按 `Z` 还是游戏自己
   切档。计划里"用 `bForward` 区分手动/脚本"的判据被放弃，改为"插件自己的状态机 + 现场
   读 tag"。
2. **`orig` 返回 0 时游戏状态完全不动**（tag 与时间戳都不变），这是最可靠的"游戏拒绝"
   信号，比事后比对 tag 安全。
3. **游戏会自己改档**：两次零 Toggle 调用下 tag 变了（传送）。→ 每帧监控 + 渲染线程
   守卫成为硬要求，而不是可选优化。
4. `limit`（`+0x1E98`）在整段测试中一直是 0，只有在过场里才会非 0；插件仍自己兜
   `limit > 0 → 拒绝`，因为车内包装函数没有这个检查。
5. tag id（`third=0x3B1A1`、`first=0x3B1BE`）**只是本构建现场值**：插件运行时按文本
   （`anomaly.ue5.names` 反查）解析，不硬编码。

## 5. 与计划的差异

| 计划 | 实际 | 原因 |
|---|---|---|
| `entryView` 四档配置 + UI | **整组删除**，改为固定规则 | 用户确认"FP 上车保持 FP"是正常循环的一环，不需要配置项（计划 §1.2 Q7 的修订） |
| `bForward` 区分手动/脚本 | 放弃 | TG-0 实测全 `false` |
| 主动传 `entryView` 给游戏 | 插件**只剩 Z detour 一处**调用切档函数 | 少一处能抢玩家的路径 |
| 新增诊断项显示当前档位 | 只写日志 | 用户要求不新增 UI 项 |
| "无闪帧"靠肉眼 | 逐帧 trace + 自动违例检查，**再**由用户肉眼确认 | `-DNFS2015_VIEWMODE_TRACE=ON` 诊断构建 |
| 计划未包含抓拍模式 | 新增 `PC+0x1760` 判据 | 实机发现真 bug（§6） |
| N / H2 备选 | 不实施 | H1 实测通过 |

`bForward` 硬门禁（计划 §5.1 第三条）因此从未触发，按"汇报 + 非路线级分叉"处理。

## 6. 计划外修复：F8 抓拍被追车覆盖

**现象**：按 F8 进抓拍后取景画面仍是追车机位，没法拍照。

**原因**：抓拍模式**完全不改 `CameraViewMode`**（tag `0x3B1A1`、`count=2`、`limit=0`
全程不动），所以"每帧读 tag"的守卫看不见它。

**定位**：游戏自己的抓拍相机是 `HTSelfieCameraActor` / `HTUI_NewSelfie` 体系，但相机管理器
里没有它的指针（不是通过 view target 实现的）。改用**时间轴扫描**——每 0.15s 抓一次
PC 与相机管理器各 16KB，玩家来回切 5 次 F8，找"只有两个值、正好切换 5 次、每次停留很久"
的字节。相机管理器侧全是 FOV/计时器噪声（89.42↔85.0），**`PC+0x1760` 是唯一干净的指针**：
正常游戏为 0，抓拍时指向 `CineCameraComponent`，其 `outer` 就是控制器自己。

**修复**：`PcCineCameraPresent()` 每 tick 读一次、每渲染帧读一次；非空时
`ChaseSlotAllowed()` 返回 false（停掉全部写路径）且 `ChaseCameraGuardAllows()` 返回 false
（渲染线程当帧即停，不等下一个 tick）。读不出来按"没有"处理 → 退化成修复前的行为，
不会把追车相机整个关掉。

**验证**：`cine=1 active=0` 连续 **1262 帧**无覆盖；退出 F8 后 **20 ms**（1 帧发布延迟）
`active=1` 恢复追车；用户肉眼确认抓拍可用、退出后追车自动回来。

## 7. 测试矩阵结果

| # | 场景 | 证据 | 结果 |
|---|---|---|---|
| T1 | TP 上车 → 追车 | `viewmode entry: tag=TP wanted=1` + `slot chase=1` | ✅ |
| T2 | 追车→TP | `dec=chase-to-third`，零游戏切档 | ✅ |
| T3 | TP→FP | `dec=third-to-first` | ✅ |
| T4 | FP→追车无闪帧 | 逐帧：5 帧 `FP/wanted=0/active=0` → 直接 `TP/wanted=1/active=1` | ✅ |
| T5 | 1 秒 6 下 | 0.87s 内 7 次：每次最多一档，`refused` 挡住不跳档，之后照常 | ✅ |
| T6 | 步行 `Z` | 6 次 `dec=on-foot`，tag 正常 TP↔FP | ✅ |
| T7 | FP 上车保持 FP | `entry: tag=FP wanted=0`；第一次 `Z` → `first-to-chase` | ✅ |
| T8 | 追车中下车 | `dec=on-foot veh=0`，覆盖消失 | ✅ |
| T9 | 剧情过场 | 无可用过场 | ⏳ 未做（用户决定） |
| T10 | 主动退出后过场 | 同上 | ⏳ 未做（用户决定） |
| T11 | 禁用 | 追车消失、`Z` 原生 | ✅ |
| T12 | 车内启用 | `entry: tag=TP wanted=1` | ✅ |
| T13 | 热重载 | 6 次，重解析 pattern/vtable/tag，在车内自动回追车 | ✅ |
| T14 | hook 失败注入 | pattern 不命中 / vtable 不符：正常加载 + 一行 `warning` + `Z` 原生 | ✅ |
| T15 | F7 | 无任何反应，`hotkeys_after_stop=0` | ✅ |
| T16 | 无新增 UI 项 | 设置窗口只有原总开关，schema 仍 `v1` | ✅ |
| T17 | 30 min 长跑 | 本次 13 min | ⏳ 缩短（用户决定） |
| T18 | 现有相机零回归 | `wheel drift` 368 行、`burnout state=1` 6 次、route B 正常 | ✅ |
| — | F8 抓拍（计划外） | §6 | ✅ |

逐帧扫描全量：**6414 帧 / 31 个转换窗口 / `violations=0`**（"`active=1` 且 tag ≠ TP" 的帧
一帧都没有）。

## 8. 已知限制与未验证项

1. **剧情过场（T9/T10）未目视验证**：用户本次没有可触发的过场，决定跳过。代码侧有三层
   保护（`limit > 0` 拒绝、tag 不是 TP/FP 就不覆盖、`pending_entry` 重试），且抓拍用的
   cine-camera 判据对过场相机大概率同样成立，但**没有实机证据**。
2. **30 分钟长跑缩短为 13 分钟**：该段 `non-info = 0`，内存增长由离线 TestHost 背书
   （`private_growth_bytes ≈ 1.0 MB`，预算 10 MB）。
3. **偏移常量只对已验收的 profile 有效**（`PC+0x1E80`、`PC+0x1760`、`vtable+0x15C8`）。
   build gate 不匹配时整条 view-mode 路径关闭，不会出现"半读偏移"状态。
4. 抓拍判据只验证了 F8 抓拍这一种"游戏自有相机"；其它游戏相机模式（回放、自由观察）
   若也走同一个成员，则同样被覆盖，但没有逐一测试。
5. 与 Camera Tools / BetterPose 的相机 hook 互斥关系不变。

## 9. 产物与校验

| 项 | 值 |
|---|---|
| 版本 | `2.0.4`（`manifest.json` / `plugin.cpp` / `pluginmaster.json` 三处一致） |
| manifest description | 497 字符（schema 上限 512） |
| 包 | `build.mingw/package/robineatcorn.nfs2015-chase-cam/` |
| ZIP | `dist/Nfs2015ChaseCam-2.0.4.zip` |
| ZIP sha256 | `2c1bd2a269f87430c86fdb411528716e17435ecc5c3f1ac1a8d6191a24feaa0d` |
| plugin.dll sha256 | `3ffab46655bef212cdf04345782461560ab34a9ff067397414bf55ec0ee614b7` |
| DLL 导入 | 只有 `KERNEL32.dll`、`msvcrt.dll` |
| `anomaly-plugin validate` | `valid ... version=2.0.4 entry=plugin.dll` |
| `anomaly-test-host` | `reloads=10 ticks=660 windows=11 text=176 resource_leases_after_stop=0 hotkeys_after_stop=0` |
| `verify-release.sh` | `failed: 0` |

提交：

```text
94789af fix: hand the camera back while the game shows its own cine camera
3ffeeef feat!: drop the F7 hotkey and release 2.0.4
c054487 feat: cycle chase, third and first person with Z while driving
```

未入库：`PLAN-zkey-*.md`、`analysis/`（含 TG-0..TG-4 报告、逐帧 trace、时间轴快照、
按代次切分的实机日志 `analysis/view-mode-recon/raw/logs/`）。
