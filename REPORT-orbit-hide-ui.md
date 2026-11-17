# NTE 环绕隐藏 HUD：实现与最终验收

- 日期：2026-11-18
- 对应计划：`PLAN-nfs2015-orbit-hide-ui.md` 任务组 13–17
- 状态：**实现、单测、实机验收完成**
- 相关报告：`REPORT-orbit-hide-ui-probe.md`（TG-12 探针）

## 0. 结论

停车环绕期间只隐藏游戏的 HUD 层，保留菜单层，因此 F1/F2/F3/F4/F5/B/C/ESC
第一次按下就能打开界面。HUD 在进入环绕过渡的第一帧淡出；在退出环绕过渡的
第一帧开始淡入，而不是等退出过渡结束。关闭环绕、关闭隐藏开关、下车、第一人称/
拍照/过场相机接管、视角重新激活以及插件停止等路径都有恢复或恢复保障。

最终实现没有直接调用 `HTUI_MainForm::HideMenu`。TG-12 验证的
`W_OverallUILayout_C::HideAllLayer/RevertLayerShow` 仍作为回退路径；正常路径对
`GameLayer_Stack` 和 `GameplayMain_Stack` 使用 `RenderOpacity`（并叠加很短的
`RenderTranslation` 位移），这样 HUD 和菜单可以分开处理。

## 1. 最终行为

### 1.1 时间线

```text
Idle
  -> Enter 第一帧：请求隐藏 HUD（0.18 s 淡出 + 18 px 下移）
  -> Orbit：保持隐藏
  -> Exit 第一帧：请求恢复 HUD（0.18 s 淡入 + 回位）
  -> Idle
```

- 进入/退出过渡本身仍由 `OrbitShot` 控制；HUD 隐藏不改变相机时长。
- 退出过渡期间 HUD 已经可见，不是过渡结束后才恢复。
- 菜单层 `GameplayMain_Stack` 始终保留；正常路径下只有 HUD 层
  `GameLayer_Stack` 被压到透明。
- HUD 在隐藏态还会持续重申透明度和位移，避免游戏在菜单关闭后把 HUD 放回来。

### 1.2 菜单与玩家干预

- F1/F2/F3/F4/F5/B/C/ESC 第一次按即可打开对应界面，菜单层没有被
  `HideAllLayer` 一并隐藏。
- 打开菜单时 HUD 保持隐藏；菜单关闭后继续隐藏。
- ESC 打开菜单时，HUD-only 模式不会把菜单可见误判成玩家重新打开了 HUD，
  因此不会提前放弃隐藏所有权。
- 只有读回可用且 HUD-only 模式关闭（全层回退）时，可见读回才作为玩家干预处理。

### 1.3 设置与诊断

设置窗口「停车环绕」新增：

| 字段 | 默认 | 含义 |
|---|---:|---|
| `orbitHideUi` | `true` | 环绕进入/保持时隐藏游戏 HUD |

- 旧设置文件缺少该键时按 `true` 处理，不需要重置配置。
- 关闭开关会立即请求恢复；环绕相机继续运行，但不再隐藏 HUD。
- 诊断面板新增 `Orbit UI hide` 行：
  `phase / was / hidden / int / rb / req / err`。
- 状态变化只打一条日志；失败阶段使用 WARN，其余阶段 INFO。

### 1.4 恢复与 fail-safe

以下路径会恢复 HUD（或保证下一次恢复）：

- `OrbitPhase` 进入 `kExit`；
- `orbitEnabled=false` 或 `orbitHideUi=false`；
- `!driving` / 离开载具；
- 第一人称、拍照/过场相机：使用与相机相同的 `ChaseSlotAllowed` 守卫；
- `view_epoch` 变化（视角重新激活/离开追车槽）；
- 读回不可用：按 Q2=C 在退出时无条件恢复；
- 插件 Stop：在 Lifecycle 线程写 recovery marker，由下一次游戏 tick 或下次
  插件启动恢复；
- 同一模块 Stop 后再次 Start：保留 `startup_restore_pending` 继续重试。

如果 `SetRenderOpacity` / `SetRenderTranslation` 不可用，会回退到
`HideAllLayer` / `RevertLayerShow`；这条回退会同时隐藏菜单层，属于低版本/异常
宿主上的兜底路径。

## 2. 代码结构

| 文件 | 职责 |
|---|---|
| `orbit_ui_hide.hpp` | 纯状态机：进入隐藏、保持、退出恢复、读回、玩家干预、重试 |
| `src/orbit_ui.cpp` | Game 线程解析、层读回、透明度/位移动画、失败回退、marker |
| `src/state.hpp` | `OrbitUiRuntimeState`、`DisplayState` 诊断字段 |
| `src/settings.cpp` | `orbitHideUi` 的 schema/默认值/读写/旧配置兼容 |
| `src/ui.cpp` | 设置复选框与 `Orbit UI hide` 诊断行 |
| `src/display.cpp` | 每 Game tick 复制 HUD 隐藏状态到展示快照 |
| `src/constants.hpp` | 设置 schema 的 `orbitHideUi` 属性 |
| `locales/zh-CN.json` | 设置和诊断中文文案 |
| `tests/orbit_ui_hide_test.cpp` | 13 个状态机场景，含 Exit 第一帧恢复 |

关键点：

- HUD-only 模式只在 `Enter`/`Orbit` 隐藏；`Exit` 被明确排除。
- `ChaseSlotAllowed` 把第一人称、拍照/过场相机和追车槽判断统一到一处。
- 视角 epoch 变化会强制本 tick 不隐藏，让状态机走恢复。
- Stop 不在 Game 线程；当前宿主没有可从 Lifecycle 线程安全投递 ProcessEvent 的
  接口，因此 marker 是唯一不会跨线程调用游戏对象的恢复方式。

## 3. 构建与回归

执行：

```sh
./build-mingw.sh --fresh --run-tests
ctest --test-dir build.mingw --output-on-failure
PATH=/tmp/anomaly-tools:$PATH ./verify-release.sh --allow-missing-host-tools
```

结果：

| 项目 | 结果 |
|---|---|
| fresh 配置 + MinGW 交叉编译 | 通过 |
| `nfs2015_chase_cam_test` | 通过 |
| `nfs2015_burnout_shot_model` | 通过 |
| `nfs2015_orbit_shot_model` | 通过 |
| `nfs2015_orbit_handheld_model` | 通过 |
| `nfs2015_orbit_ui_hide_model` | 通过 |
| `nfs2015_wheel_steer_model` | 通过 |
| `ctest` | **6/6** 通过 |
| `plugin.dll` 导入 | 仅 `KERNEL32.dll`、`msvcrt.dll` |
| 导出 | `AnomalyPluginEntryV1` |
| `anomaly-plugin validate` | 通过 |
| `anomaly-test-host --reload 10 --ticks 60` | 通过；停止后 lease/staging/hotkey 均为 0 |
| 发布检查 | 0 failed，0 blocked |

本次构建：

```text
plugin.dll sha256  582ce896d42a331a5516f7a3016914af13f2c1a80093a9f1d07a6f5d770e5268
plugin.dll size    7465765 bytes
package manifest   f22727dd5b75842e7e428e5cfe19fe38800fdd771a904a5625cb5fa16c0dba53
ZIP                7093501b00e867680e39a298ab09828eb62cbaa5c26cc2246605ef7c1961e06e
```

## 4. 实机验收

用户已确认下列目视结果：

| 检查项 | 结果 |
|---|---|
| 进入环绕过渡时 HUD 隐藏 | 通过 |
| 环绕全程无 HUD | 通过 |
| 退出过渡开始后 HUD 恢复 | 通过 |
| 菜单热键首次按下即可打开 | 通过 |
| F1/F2/F3/F4/F5/B/C/ESC | 通过 |
| 菜单关闭后 HUD 继续隐藏 | 通过 |
| 关闭 `orbitHideUi` 后 HUD 恢复、环绕继续 | 通过 |
| 第一人称/过场/拍照相机接管时 HUD 恢复 | 通过 |
| 下车、关闭环绕、视角重新激活时恢复 | 通过 |
| Stop 后重新启动/重载插件恢复 | 通过 |
| 诊断行显示阶段和计数 | 通过 |

## 5. 已知限制

- **Stop 不是同步恢复**：Anomaly 的 `on_stop` 在 Lifecycle 线程执行，
  `ProcessEvent.invoke` 只允许 Game 线程。当前实现先写 marker，下一次游戏 tick
  或下次插件启动恢复。不要在 Stop 返回后立即假设 HUD 已恢复。
- HUD-only 路径依赖 `SetRenderOpacity` / `SetRenderTranslation`；不可用时回退到
  全层隐藏，此时菜单层也会消失。回退失败按 fail-open 只记日志，不阻塞相机。
- 隐藏仅针对游戏 HUD，不隐藏 Anomaly 自己的设置窗口，也不改变游戏菜单层。
- 当前只有 `zh-CN` 文案；英文宿主使用代码内英文 fallback。
- 读回使用 live layout 子层的 `GetRenderOpacity`，不依赖固定对象地址；宿主/
  游戏更新导致对象或函数不可用时走 Q2=C 恢复，不会留下永久隐藏。
- 若游戏在环绕中自行改变 UI 状态，HUD-only 模式会继续维护自己的透明层；
  只有全层回退模式会把可见读回视为玩家干预。

## 6. 相关提交

```text
ec00f6f feat: add parked-orbit UI hide state machine
8c70cdd feat: hide the parked-orbit HUD
bdbd7b8 feat: fade the orbit HUD without hiding menus
63e10bf fix: restore the orbit HUD when the exit transition starts
579d9f3 feat: expose orbit UI hide settings and diagnostics
e517843 fix: restore the orbit HUD on camera-guard and view exits
```
