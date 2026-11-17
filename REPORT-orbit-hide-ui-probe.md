# NTE 环绕隐藏主 UI：TG-12 探针报告

- 日期：2026-11-17
- 对应计划：`PLAN-nfs2015-orbit-hide-ui.md` 任务组 12
- 状态：**TG-12 通过（功能、读回、实例扫描、耗时均通过）**
- 结论：`W_OverallUILayout_C::HideAllLayer` / `RevertLayerShow` 可以完整隐藏/恢复 HUD（用户已实机确认）。根控件 `GetVisibility` 不变，但子 layer `GameplayMain_Stack` / `GameLayer_Stack` 的 `RenderOpacity` 在隐藏时 `0.0`、恢复时 `1.0`，可作为 Q2 读回。禁用快速 index 后，8k/批注册表扫描在 11 块内找到 `W_OverallUILayout_C`，单块最高 `5.60 ms`。按计划，TG-12 通过后暂停；**不进入 TG-13，直到用户确认**。

## 0. 重要背景纠正

当前运行的是 **NTE（Neverness To Everness，`HTGame.exe`）**，不是 NFS 2015。

`nfs2015-chase-cam` 是 NTE 里模仿 NFS 2015 运镜的 mod；仓库里此前的 NFS 2015 运镜调查报告只作为运镜参考，不能用于本次 NTE UI/输入分析。

本轮所有 UI 结论均来自 NTE PID `9280`、Anomaly 3.0.0 runtime。

## 1. TL;DR

1. F7 已确认绑定到 NTE 的 `IA_HideMainUI`：`IMC_Default_KBM` 中 `Action=0x2A4B8B80`、`Key=F7`。
2. F7 的 `ProcessEvent` 跟踪和三个原生 hook 都没有命中主 HUD 隐藏路径；只命中普通 UI 事件/`BPUI_Bubble_C`。
3. 找到了真正可用的完整隐藏/恢复 API：
   - `W_OverallUILayout_C::HideAllLayer`
   - `W_OverallUILayout_C::RevertLayerShow`
4. 直接调用两者均返回 `status=0`；用户确认 HUD **完整隐藏/恢复**，重复多次无崩溃。
5. 根控件 `W_OverallUILayout_C` 的反射读回失败：隐藏前、隐藏后、恢复后始终 `GetVisibility=4`、`IsVisible=1`。
6. **可靠读回已找到**：实际根 `MainOverlay`（`0xEE018580`）下的 `GameplayMain_Stack` / `GameLayer_Stack` 的 `RenderOpacity`（`+0xE0`）在隐藏时变为 `0.0`，恢复时回到 `1.0`；`Visibility` 不变。
7. 禁用快速 index 后，generation 20 的 8k/批 fallback 扫描在 `11` 块内找到实例（index `95711`），单块 `3.17–5.60 ms`，低于 50 ms/帧；实例解析和性能验证通过。
8. TG-12 已通过；按计划暂停，不进入 TG-13。临时探针最终要删除，不提交。

## 2. 环境与构建

| 项 | 值 |
|---|---|
| 游戏 | NTE / `HTGame.exe`，PID `9280` |
| 运行时 | `Anomaly-3.0.0-runtime` |
| 插件 | `robineatcorn.nfs2015-chase-cam`，探针 generation `18` |
| 构建 | WSL2 + MinGW-w64，`build.mingw`，RelWithDebInfo |
| 日志 | `C:\Program Files (x86)\Neverness To Everness\Anomaly\Anomaly-3.0.0-runtime\Anomaly\logs\anomaly-runtime.jsonl` |
| 日志时间 | UTC；本地时间约为 UTC+8 |

- `cmake --build build.mingw -j4`：通过。
- 探针通过 host 热加载生效；安装脚本需要管理员权限。
- 临时安装脚本：`install_orbit_ui_probe.ps1`，会备份 `plugin.dll.pre-orbit-ui-probe`。

## 3. F7 / 输入证据

### 3.1 F7 绑定

`IMC_Default_KBM` 活体对象 `0x4BE16760`，`Mappings` TArray 在 `+0x30`：

- data `0xEF346050`
- num `99`
- max `102`
- mapping @ `0x1C40`，记录大小 `0x50`
  - `Action = 0x2A4B8B80` → `IA_HideMainUI`
  - `Key FName id = 0x141C`（十进制 `5148`）→ `F7`

所以 F7 确实是 NTE 的 `IA_HideMainUI`。

### 3.2 ProcessEvent 跟踪

两轮 trace（generation 10）：

| 轮次 | 调用数 | 唯一函数 |
|---|---:|---:|
| 1 | 83,303 | 108 |
| 2 | 83,922 | 129 |

没有出现：

- `ShowHideMainFormFunctionByType`
- `SetMainFormVisibilityConfigHidden`
- `HideMenu`
- `ForceHideAllUI`
- `IA_HideMainUI`

只有普通 UI 事件，例如 `OnHideAllInteraction`、`OnShowMapIconsChanged`、`OnGuideHandleReturnMainForm` 和 `BPUI_MainForm_C` 的鼠标事件。

### 3.3 原生 hook

generation 12/14 临时 hook 了：

- `UWidget::SetVisibility` 实际虚函数 `0x143C89CE0`
- `HTHUD::ShowHideMainFormFunctionByType` helper `0x148B75E30`
- `HideMenu` 实际函数 `0x1493B8C50`

20 秒 trace 只命中 `BPUI_Bubble_C` 的 `SetVisibility`，没有主 HUD、`HideMenu` 或 `ByType` 命中。

结论：F7 很可能走整体 UI layout，而不是 `UWidget::SetVisibility` / `HideMenu` 子控件路径；不应继续把这三个 hook 当主路径。

## 4. 完整隐藏/恢复 API（功能已验证）

### 4.1 对象和函数

- 类路径：`/Game/UI/W_OverallUILayout.W_OverallUILayout_C`
- 函数路径：
  - `/Game/UI/W_OverallUILayout.W_OverallUILayout_C:HideAllLayer`
  - `/Game/UI/W_OverallUILayout.W_OverallUILayout_C:RevertLayerShow`

CLI 查询结果：

| 对象 | 地址 / ID |
|---|---|
| UClass `W_OverallUILayout_C` | `0xF92C0400` |
| `HideAllLayer` UFunction | `0xEFC67000` |
| `RevertLayerShow` UFunction | `0xEFC6A500` |
| live instance | `0x1B38A900` |
| registry index | `95711` |
| outer | `HTGameInstance` |
| 插件 `find_exact` 结果 | class `id=22389664609667`；HideAllLayer `id=95622`；RevertLayerShow `id=95624` |

### 4.2 调用结果

用 `events->invoke` 空参数直接调用：

```text
layout HideAllLayer status=0
layout RevertLayerShow status=0
```

用户实机确认：HUD **完整隐藏/恢复**，重复多次无崩溃。

注意：`HTPrimaryGameLayout::HideAllLayer/RevertLayerShow` 也存在（UFunction `0x7FF4DEE72A78/0x7FF4DEE72B58`），但这些 UFunction 的 `Func` 都是通用 Blueprint VM `0x1418D5110`；实际 live widget 是 `W_OverallUILayout_C`。

### 4.3 根控件读回失败

generation 18 日志：

```text
layout instance found at fast index 95711
layout resolved widget id=22776211666400 generation=1 index=0
layout before hide widget=14b7000175e0 GetVisibility=ok(4/0) IsVisible=ok(1/0) hidden=0
layout HideAllLayer status=0
layout after hide widget=14b7000175e0 GetVisibility=ok(4/0) IsVisible=ok(1/0) hidden=1
layout before restore widget=14b7000175e0 GetVisibility=ok(4/0) IsVisible=ok(1/0) hidden=1
layout RevertLayerShow status=0
layout after restore widget=14b7000175e0 GetVisibility=ok(4/0) IsVisible=ok(1/0) hidden=0
```

- `widget id=22776211666400` = hex `0x14b7000175e0`，即 `W_OverallUILayout_C` 实例。
- 根 `GetVisibility` 始终 `4`（`SelfHitTestInvisible`），`IsVisible` 始终 `1`。
- 因此根控件不能作为 Q2 的“隐藏/恢复”读回。

### 4.4 子控件读回：`RenderOpacity`

实际 `W_OverallUILayout_C` live `0x1B38A900` 的 `WidgetTree` 是 `0xEFCF3190`（`WidgetTree + 0x2D8` 指向它）；其根 `MainOverlay` 是 `0xEE018580`（`WidgetTree + 0x30`）。

`MainOverlay` 自身和 42 个 `HTActivatableWidgetContainerStack` 子 layer 的 `Visibility` 在隐藏/恢复前后都**不变**（根和 layer 多数仍为 `1` 或 `4`）。逐字节 diff 后找到真正变化的字段：

| 对象 | 偏移 | 隐藏态 | 恢复态 |
|---|---|---:|---:|
| `GameLayer_Stack`（`0xF09CC4F0`） | `+0xE0` `RenderOpacity` | `0.0` | `1.0` |
| `GameplayMain_Stack`（`0xF09CC220`） | `+0xE0` `RenderOpacity` | `0.0` | `1.0` |

`W_OverallUILayout_C`、`MainOverlay`、`BPUI_MainForm_C` 自身的 `Visibility` 均不变；`HideAllLayer` 是通过 layer 的 `RenderOpacity=0` 实现隐藏，而不是改根控件的 UMG `Visibility`。

因此 Q2 的可靠读回应使用 layer 的 `RenderOpacity`（最终实现优先调用 `/Script/UMG.Widget:GetRenderOpacity`，避免裸偏移）。CLI 裸偏移只是本轮验证证据；最终实现不能写死 `0xF09CC220` 这类本轮地址。

## 5. 实例解析

### 5.1 探针路径

历史探针曾用快速 index `95711`；generation 18 日志确认 `layout instance found at fast index 95711`。但最终实现不能依赖它。

当前探针的 `kOverallWidgetIndex` 已临时设为 `0`，实际只走 fallback 扫描：

1. `find_exact` 拿 `W_OverallUILayout_C` UClass；
2. `snapshot_by_handle` 拿 class `name_id`；
3. 以 `objects->count()` 为总数，每 Game tick 用 `snapshot_at` 扫描 `8192` 个对象；
4. 按 `name_id` 匹配，排除 class CDO；
5. 跨 tick 继续，找到后缓存 `overall_widget`，并记录每块耗时。

### 5.2 扫描设计

- 块大小：`8192` 个对象/Game tick；
- 匹配条件：class `name_id`，不依赖 index 或地址；
- 找到后缓存；未匹配时继续下一块；
- 每块用 `NowNanoseconds()` 计时并写日志。

此前 `BPUI_MainForm` 的 65,536/批扫描最高 **75.72 ms**，不能套用到 8k/批；本轮已重新实测，见 §5.3。

### 5.3 fallback 扫描实测

把 `kOverallWidgetIndex` 临时设为 `0`（强制扫描）后，generation 20 日志：

```text
layout scanning 478789 objects for W_OverallUILayout_C
layout scan chunk 4.62 ms at 8192/478789
layout scan chunk 3.95 ms at 16384/478789
...
layout scan chunk 4.45 ms at 95711/478789
layout resolved widget id=22776211666400 generation=1 index=95711
layout HideAllLayer status=0
```

| 项 | 结果 |
|---|---|
| 扫描块大小 | `8192` |
| 找到位置 | index `95711` |
| 实际块数 | `11`（停在 95711） |
| 单块耗时 | `3.17–5.60 ms` |
| 50 ms 判据 | **通过** |

实例解析和扫描性能均通过。最终实现不能保留硬编码 index `95711`；当前探针的快速 index 已置 0，实际只走扫描。可选稳定解析：

- class name + 分批 registry scan + 缓存；
- 从 `HTGameInstance` / `WidgetTree` 关系拿实例；
- 其他不依赖地址/index 的稳定关系。

在 TG-12 里，快速 index 只能作为探针加速路径，不能作为最终实现。

## 6. TG-12 通过标准对照

| 通过标准 | 结果 |
|---|---|
| 完整隐藏主 UI | **通过**：`HideAllLayer` 用户确认完整隐藏 |
| 完整恢复主 UI | **通过**：`RevertLayerShow` 用户确认完整恢复 |
| 至少一个读回稳定变化 | **通过**：`GameplayMain_Stack` / `GameLayer_Stack` 的 `RenderOpacity` 隐藏 `0.0`、恢复 `1.0` |
| 实例解析不硬编码 | **通过**：禁用快速 index 后，按 class `name_id` 扫描找到；最终只需缓存，不写死 index/地址 |
| 扫描不超 50 ms/帧 | **通过**：8k/批，单块最高 `5.60 ms` |
| 无崩溃/无明显卡顿 | **通过**：多轮隐藏/恢复无崩溃，扫描无可感卡顿 |

结论：**TG-12 通过**。按计划暂停，等用户确认后再进入 TG-13；最终实现还需把 `GameplayMain_Stack` 的稳定解析和 `GetRenderOpacity` 接进状态机。

## 7. 下一步（TG-12 已通过，暂停）

1. 向用户汇报 TG-12 通过；等待用户确认后再进入 TG-13。
2. TG-13 最终实现需要：
   - 用注册表扫描 + 缓存解析 `W_OverallUILayout_C`，不保留 index `95711`；
   - 稳定解析 `GameplayMain_Stack`（或其活动 layer），调用 `/Script/UMG.Widget:GetRenderOpacity` 读回，避免写死 `0xF09CC220` 和裸偏移 `0xE0`；
   - 进入前已隐藏 → 不隐藏、不恢复；隐藏期间玩家恢复 → 尊重玩家；退出时仅在仍由我们隐藏时恢复；读回不可用时按 Q2=C 保守恢复。
3. 清理临时探针：
   - 删除 `Probe: toggle game UI` / layout 按钮；
   - 删除 `RunOrbitUiProbe`、原生 hook、ProcessEvent trace、layout 探针和 `state.hpp` 探针字段；
   - 从 `CMakeLists.txt` 移除 `src/orbit_ui_probe.cpp`；
   - 删除 `install_orbit_ui_probe.ps1` 和临时报告（或保留报告但不提交）。
4. 生成 TG-13 的单测与实现后，再提交。

## 8. 产物与清理

未提交修改：

```text
M CMakeLists.txt
M src/internal.hpp
M src/lifecycle.cpp
M src/presenter.cpp
M src/state.hpp
M src/ui.cpp
?? src/orbit_ui_probe.cpp
?? install_orbit_ui_probe.ps1
?? REPORT-orbit-hide-ui-probe.md
```

无关未跟踪计划文件，不要提交：

- `PLAN-zkey-phase2-h1.md`
- `PLAN-zkey-viewmode-cycle.md`

当前探针 generation `18`；上一个已提交插件 commit：`e53b19e feat: add an adjustable parked-orbit camera height`。

临时探针最终删除；如果 TG-12 最终失败/降级，也要在报告中记录用户决定。
