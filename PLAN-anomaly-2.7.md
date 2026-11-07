# Anomaly 2.7.0 适配计划

目标仓库：`Anomaly-nfs19-cam`（NFS 2015 Chase Cam）
目标宿主：Anomaly `v2.7.0`
当前插件版本：`2.0.0`（任务组 1、2、3 代码已完成；实机验证仍待执行）
编写日期：2026-11-07

---

## 0. 结论（TL;DR）

Anomaly 2.7.0 不是一次普通的小版本更新，它同时触发了 **加载阻断、编译阻断、运行时功能缺失** 三类问题：

1. **插件 ID 规则变了**：`io.github.robineatcorn.nfs2015-chase-cam` 不再符合 `<作者>.<插件名>` 的两段式规则，manifest 校验直接失败，插件不会被加载。
2. **`nte-vehicle` capability 被删除**：即使只修 ID，manifest 仍会因为未知 capability 被拒绝；`LoadBinary` 会直接放弃该插件。
3. **`anomaly.nte.vehicle` 服务和相关 SDK 类型被移除**：源码无法用 2.7.0 SDK 编译；运行时也失去了精确车速、带符号前进速度、刹车俯仰和载具存在性检测的数据源。
4. **活动 Profile 的 hash 变了**：插件的 fail-closed 版本门禁会关闭 route B、FOV 和视点 hook，即使前三条都修好，相机也不会接管。
5. **ID 变更还带来用户数据迁移问题**：旧启用状态、设置目录、窗口位置和旧安装目录都不会自动跟到新 ID。

最小可发布路径是 **任务组 1 + 任务组 2 + 任务组 4**：先让插件在 2.7.0 加载、编译，再用“位置微分 + 载具存在性”恢复相机，最后做回归和发布。任务组 3 的精确车速与 route B advisory witness 已经实现；位置微分保留为缺少 `ue5.objects` / `ue5.process-event` 或水上载具时的回退路径。

---

## 1. 调查结果

### 1.1 2.7.0 的破坏性变更

| # | 2.7.0 变更 | 证据 | 对插件的影响 | 严重度 |
|---|---|---|---|---|
| 1 | 插件 ID 统一为 `<作者>.<插件名>`，固定两段 | `Anomaly` 提交 `71370ea`；`schemas/plugin-manifest.schema.json` 的 `pluginId`；`docs/api-reference/manifest-and-capabilities.md` | 当前 ID `io.github.robineatcorn.nfs2015-chase-cam` 含反域名前缀和四段，manifest 解析失败；插件彻底不加载 | **阻断** |
| 2 | 删除 `nte-vehicle` capability | 提交 `243a95c`；`src/plugin/plugin_capability_policy.cpp` 的已知列表已无该项 | `ResolvePluginCapabilityGrant` 因未知 capability 将 grant 标记为不可执行；`PluginManager::LoadBinary` 直接拒绝插件 | **阻断** |
| 3 | 删除 `anomaly.nte.vehicle` 服务及公开类型 | `243a95c` 的 BREAKING CHANGE 说明；`include/anomaly/sdk/services/nte.h` 中 `AnomalyNteVehicle*` 全部删除；ABI snapshot 删除该表 | 插件引用 `AnomalyNteVehicleServiceV1`、`AnomalyNteVehicleSnapshotV1`、`ANOMALY_NTE_VEHICLE_*`，2.7.0 SDK 下编译失败；运行时精确车速/带符号速度/载具检测丢失 | **阻断/功能缺失** |
| 4 | 活动 NTE Profile 内容变化，profile hash 变化 | `profiles/nte/nte-current.json` 在 2.7.0 删除了 `nte.vehicle` / `vehicle.*`，新增 `ue5.assets` / `ue5.pak` | 插件启动时用 `anomaly.ue5.build::profile_hash` 做精确门禁；hash 不匹配时 `version_gate=false`，route B / FOV / 视点 hook 全部关闭 | **运行时阻断** |
| 5 | 插件包和配置以 ID 为持久键 | 提交 `71370ea` 说明；`PluginManager::Activate` 用 `plugin.view.id` 拼 `config/plugins/<id>` 和 `state/plugins/<id>` | 旧设置、启用状态、窗口位置、收藏不会自动迁移到新 ID；旧安装目录变成孤儿 | **用户迁移** |
| 6 | 新增 `anomaly.mods`、`anomaly.ue5.assets` | `include/anomaly/sdk/services/mods.h`、`ue5.h`；ABI snapshot | 与相机无关；不要顺手加 capability。它们也是 Profile hash 变化的部分原因 | 无直接影响 |

### 1.2 没有变化的 ABI

2.7.0 对本插件实际使用的服务是 **向后兼容的**：

- `anomaly.core`
- `anomaly.config`
- `anomaly.json`
- `anomaly.ui`
- `anomaly.window`
- `anomaly.input`
- `anomaly.localization`
- `anomaly.interop.signature`
- `anomaly.interop.hook`
- `anomaly.nte.session`
- `anomaly.nte.player`
- `anomaly.ue5.build`
- `anomaly.ue5.ahud`

ABI snapshot 的差异只有“删除 vehicle 表”和“增加 mods/assets 表”。`ANOMALY_PLUGIN_API_V1_MINOR` 仍是 `0`，manifest 的 `api` 不需要改。

### 1.3 Profile hash 与布局

关键事实：

- 仓库内 `v2.6.0` 的 `profiles/nte/nte-current.json` SHA-256：`d5af3210c54cfaca906beacb178bea0c295fb8dad8fd6f7c59c266e5ea850ea5`
- 仓库内 `v2.7.0` 的 `profiles/nte/nte-current.json` SHA-256：`c37a7b3d1802c74927797675c1723da01afaf637028315e5e265599d5119db2a`
- 当前插件常量 `kExpectedProfileHash`：`0e3aaa6d640b06ca08a5f89a71ef6377b13165c545fe636deafe1805f222a3f8`

注意：`0e3aaa...` 既不是 v2.6.0 也不是 v2.7.0 仓库文件的 hash，推测它是当时从实际 Runtime 安装目录采集的 profile。**不要直接把 `c37a...` 当成最终值**；适配时必须在 2.7.0 实际运行环境里重新采集，或者确认 Runtime 使用的就是仓库内这份文件。

好消息是，本插件用到的 X 锚点布局在 2.6.0 → 2.7.0 没有变化：

| Profile key | v2.6.0 | v2.7.0 |
|---|---:|---:|
| `world.gameInstance` | 560 | 560 |
| `gameInstance.localPlayers` | 56 | 56 |
| `localPlayer.controller` | 48 | 48 |
| `controller.pawn` | 776 | 776 |
| `actor.rootComponent` | 456 | 456 |
| `sceneComponent.location` | 280 | 280 |

`ue5.GWorld` 的 pattern 也没有变化。因此 **hash 更新后，X 锚点链大概率可以直接工作**；route B 使用的 `FViewInfo` / `FSceneRenderer` 偏移依赖游戏二进制，Anomaly 本身没有改游戏，但必须实机验证 pattern 和字段。

### 1.4 当前插件已经具备的降级能力

代码里其实已经有“没有 vehicle 服务”的分支：

- 车速可由玩家位置微分得到（`UpdateMotion` → `velocity_cmps`）。
- `driving` 在 `!vehicle_service` 时用 `speed_plausible` 判断。
- `SelectCarYaw` 在缺少 `vehicle_speed_signed_kmh` 时仍能用相机 yaw 和速度方向。

但这个降级分支有明确缺口：

1. 静止/低速时不会接管，因为速度门限是 `6 m/s`（约 21.6 km/h），进车后要开起来才切换。
2. 倒车没有带符号速度，反向相机可能退化。
3. 刹车俯仰依赖精确车速导数，位置微分噪声更大，`UpdateBrakeInput` 会直接走“无速度”分支，刹车 pitch 失效。
4. 载具身份变化（换车）没有 `vehicle_handle` 可比较，可能需要用 `AttachParent` 指针代替。

这些就是任务组 2 的核心。

### 1.5 2.7.0 的 `NteVehicle` 插件能不能直接复用？

不能直接复用。`243a95c` 把载具运行时移进了内建插件 `plugins/NteVehicle`，它 **没有对外发布 vehicle 查询服务或 IPC**。其他插件无法通过公开 ABI 直接读取“当前载具/前进速度”。

`NteVehicle` 自己能做到，是因为它：

- 用 `anomaly.ue5.objects.find_exact` 找 UFunction；
- 用 `anomaly.ue5.process-event.invoke` 调用；
- 用 `anomaly.core.read_memory` 读 GObjects 注册表，把裸对象指针转成 `ue5.objects` handle；
- 依赖自己的 `compatibility.profile.json` 和反射校验。

相机插件如果要在 2.7.0 恢复精确车速，需要复制/裁剪这套适配层，属于任务组 3 的高难度工作。

---

## 2. 适配原则

1. **先恢复加载，再恢复功能**：ID/capability/编译是硬阻断，先解决。
2. **保持 fail-closed**：profile hash 不匹配时仍然不能写 `FViewInfo` / `FSceneRenderer`。
3. **不碰宿主内部头文件**：继续只消费安装好的 `AnomalySDK`。
4. **公开 ABI 优先**：能用 `anomaly.nte.player`、`anomaly.ue5.objects`、`anomaly.ue5.process-event` 就不新增强制内存偏移。
5. **速度降级必须可观测**：面板要显示当前速度来源（精确 / 微分 / 不可用），不能静默退化。
6. **用户数据可迁移**：ID 变更不是“重新安装就完事”，要给出设置和启用状态的迁移路径。

---

## 3. 任务分组（按难度）

### 任务组 1 ★ 低难度：让 2.7.0 能识别、加载、编译

**目标**：消除 manifest、capability、SDK 类型三层硬阻断。完成后插件可以加载和编译，但相机可能仍因 profile hash 门禁不工作。

**预计**：0.5–1 天。

#### 1.1 插件 ID 迁移

- `manifest.json`
  - `id` 改为 `robineatcorn.nfs2015-chase-cam`（两段、全小写 kebab）。
  - 版本号建议 `2.0.0`（ID/配置路径迁移属于破坏性变更）；如果不想跳大版本，至少 `1.2.0`，并在 Release 说明里写清迁移。
- `plugin.cpp`
  - `constexpr std::string_view kPluginId` 同步改为新 ID。
  - 确认 descriptor 里的 id、name、author、version 与 manifest 一致。
- `pluginmaster.json`
  - `InternalName` 同步为新 ID。
  - `Version`、`DownloadLinkInstall`、`DownloadLinkUpdate` 指向新 Release。
- `README.md`
  - 安装路径、插件 ID、插件源 URL、兼容版本说明同步。
- 注意：旧 ID 的 Release 资产无法“远程修复”，旧版本仍然会在 2.7.0 加载失败；新 Release 必须明确写“请先卸载旧版”。

**验收**：

- `manifest.json` 能通过 2.7.0 的 `anomaly-plugin validate`。
- 插件列表中出现新 ID，且 `InternalName` 与 manifest 完全一致。

#### 1.2 删除已移除的 capability 和 service

- `manifest.json`
  - 从 `services` 删除 `anomaly.nte.vehicle`。
  - 从 `capabilities` 删除 `nte-vehicle`。
  - 保留 `anomaly.ue5.build` / `ue5-build`，它仍然存在且是 route B 门禁所需。
- 不要因为 2.7.0 新增了 `anomaly.mods` / `anomaly.ue5.assets` 就顺手加 capability；本插件不需要。

**验收**：

- 启动日志不再出现 `unknown-capability` 或 `plugin manifest capability denied`。
- 插件不再因为 capability grant 不可执行而被 `LoadBinary` 拒绝。

#### 1.3 清理已删除的 SDK 类型和代码

需要删除/替换的符号：

- `AnomalyNteVehicleServiceV1`
- `AnomalyNteVehicleSnapshotV1`
- `ANOMALY_NTE_VEHICLE_SERVICE_V1_ID`
- `ANOMALY_NTE_VEHICLE_V1_VALID`
- `ANOMALY_NTE_VEHICLE_V1_HAS_SPEED`
- `ANOMALY_NTE_VEHICLE_CLASS_NAME_MAX_UTF8_BYTES`

建议处理：

- 删除 `VehicleReady()`、`PollVehicle()`、`RefreshVehicleSample()`。
- 删除 `Context::vehicle`、`vehicle_cached_*`、`vehicle_handle`、`vehicle_speed_signed_kmh`、`diag.vehicle_polls`、`diag.vehicle_gaps` 等只服务于旧服务的字段。
- 删除初始化里的 `Query<AnomalyNteVehicleServiceV1>`。
- 删除 UI 里的 `hint.vehicle_service` 文案，或者改成新的速度来源提示。
- 暂时把 `vehicle_valid` / `vehicle_service` 固定为 `false`，让现有降级分支先跑通，后续在任务组 2 正式重写。

**验收**：

- 用 2.7.0 SDK 的 MSVC 和 MinGW 两种构建都能编译通过。
- `anomaly-plugin validate` 通过。
- `anomaly-test-host --plugin ... --reload 10 --ticks 60` 通过（TestHost 没有游戏适配器，NTE 服务保持 UNAVAILABLE，但插件应正常加载/卸载/重载）。

#### 1.4 文档和版本说明

- README 的“实现方式”“依赖服务”“兼容版本”“安装”更新到 2.7.0。
- 在 Release note 里列出：
  - 新 ID；
  - 旧 ID 不再加载；
  - `anomaly.nte.vehicle` 已移除；
  - 需要 Anomaly 2.7.0+；
  - 旧设置迁移方式。

---

### 任务组 2 ★★ 中等难度：恢复无 vehicle 服务时的相机功能

**目标**：在不依赖精确车速的前提下，让相机在 2.7.0 正常接管，并尽量保持原有观感。

**预计**：1–2 天。

**前置**：任务组 1 完成。

#### 2.1 更新 Profile 门禁

- 在 2.7.0 实际运行环境采集 `profile_hash`：
  - `anomaly-cli --pid <pid> ue`，或
  - 看插件日志 `anchor gate: profile hash mismatch (got ..., want ...)` 的 `got`。
- 更新 `kExpectedProfileHash`。
- 建议把单个常量改成可维护的 allowlist：
  ```cpp
  constexpr std::string_view kAcceptedProfileHashes[] = {
      // 2.7.0 runtime profile
      "...",
  };
  ```
  只接受明确验证过的 hash，未知 hash 仍然 fail-closed。
- 保留 `kAnchorInsuranceHashMismatch` 分支和面板显示，方便下一次升级快速定位。

**验收**：

- 面板 `版本门禁 = ok`。
- route B / FOV / 视点 hook 的 pattern 解析成功。
- 人为改一个错误 hash 时，插件仍然完全不写游戏内存。

#### 2.2 无 vehicle 服务时的速度估计

- 速度模长：继续用 `UpdateMotion()` 的 `velocity_cmps`。
- 带符号速度：
  - 优先用游戏相机 yaw 作为车头方向，`forward_speed = velocity · ForwardFromYaw(camera_yaw)`；
  - 相机 yaw 不可用时，退化为“速度方向为正”，并明确标记为未知符号。
- 倒车判定：
  - 现有 `SelectCarYaw` 依赖 `vehicle_speed_signed_kmh < -1`；
  - 没有精确带符号速度时，用相机 yaw 与速度方向的夹角判断：如果速度几乎与相机朝向相反，视为倒车，令 `travel_yaw = velocity_yaw + π`；
  - 加滞后/门限，避免漂移或旋转时误判。
- 刹车俯仰：
  - `UpdateBrakeInput` 目前无 vehicle 速度时直接衰减到 0；
  - 改为接受“估计速度 + 每 tick fresh”，用更强的平滑和现有 deadzone 抑制位置微分噪声；
  - 如果实机噪声仍然大，允许在设置里关闭刹车 pitch，而不是让它抖动。
- 显示：
  - `display.speed_kmh` 改为估计速度；
  - 增加 `speed_source`（`exact` / `derived` / `unavailable`）并在诊断面板显示；
  - 删除“载具服务不可用”的旧提示，改成“速度由位置微分估计”。

**验收**：

- 0–200 km/h 加速时 FOV、速度距离倍率连续变化。
- 重刹时刹车 pitch 可感知，且没有逐 tick 抖动。
- 倒车时相机回到车后，不跑到车头。
- 诊断面板明确显示速度来源。

#### 2.3 用 `AttachParent` 做载具存在性检测

> 实施备注（2026-11-08）：2.7.0 删除的是 `anomaly.nte.vehicle` 服务，不是游戏函数。
> 现已通过 `ue5.objects.find_exact` + `ue5.process-event.invoke` 恢复旧服务内部的
> `BP_GetCurrentDriveVehicle` / `BP_GetCurrentDirvingVehicle` /
> `GetBoardWaterVehicle` getter，并以 20 Hz 轮询。getter 给出明确结果时不再使用
> `speed_plausible`；`AttachParent` 继续提供车辆锚点，并在 getter 不可用时作为后备。
> 因此下面的 `driving = ... || speed_plausible` 只适用于“存在性未知”的降级路径。

旧 `vehicle_service` 还承担“是否在驾驶”的判断。2.7.0 下可以用已有的挂点检测：

- `ReadVehicleParentAnchor()` 已经读取玩家 `SceneComponent` 的 `AttachParent`；
- 它目前只在 X 探针路径里使用，并且依赖 `version_gate`；
- 把“parent 存在且通过距离/朝向校验”发布成 `vehicle_present` 原子量；
- `driving` 判定改为：
  ```text
  driving = vehicle_present || recent_vehicle_grace
            || (presence_unknown && speed_plausible)
  ```
- 用 `vehicle_parent` 指针作为载具身份：
  - 指针变化时触发 `ResetMotion` / `RequestPresenterReset`，替代旧的 `vehicle_handle`；
  - 保留短暂 grace，避免流式加载或一帧读失败导致相机闪回游戏视角。
- 步行时 `AttachParent` 也可能存在（例如角色挂点），因此必须保留现有的距离、变换、`VehicleMesh` 语义校验，不能只看“有 parent”。

**验收**：

- 静止进车后立即接管，不需要先开到 21 km/h。
- 下车后立即恢复游戏相机。
- 步行、攀爬、过场、传送不会误接管。
- 换车时模型重置，没有把旧车姿态带过来。

#### 2.4 实机验证 pattern 和 route B 偏移

需要逐项确认 2.7.0 下仍能解析：

- `kGWorldPattern`
- `kOnTransformChangedPattern`
- `kSetupUniformBufferPattern`
- `kViewMatricesCtorPattern`
- `kPrepareViewStatePattern`
- `kSceneViewUpdateMatrixPattern`
- `kSceneViewSetupFrustumPattern`

再确认 route B 的字段仍有效：

- `kViewMatricesOffset = 0x3E0`
- `kViewLocationOffset = 0xFF0`
- `kViewRotationOffset = 0x1008`
- `kViewStateOffset = 0x2330`
- `kViewFovOffset = 0x2044`
- `kViewDesiredFovOffset = 0x2048`
- `kSceneRendererAllViewsOffset = 0x3C8`
- `kSceneRendererAllViewCountOffset = 0x3D0`
- 车辆 proxy：`0x340` / `0x228` / `0x140` / `0x160`

如果某个 pattern 或 offset 变化，按现有 fail-closed 结构补新的常量分支，不要直接放宽门禁。

**验收**：

- 面板 `渲染钩`、`视图钩 R/G/线程`、`路线B 应用/回退` 均有正常计数。
- `路线B 应用` 持续增长，`回退` 不持续增长。
- `渲染龄`、`视图←变换龄` 在合理范围。
- 没有 `hash / pattern / service / offset / read` 失败。

#### 2.5 用户数据迁移

ID 变更后，旧数据不会自动跟随。需要迁移：

- 插件启用状态：`Anomaly/config/plugin-enablement.json`
- 设置：`Anomaly/config/plugins/<old-id>/config-nfs2015-chase-cam-settings-v1.json`
  → `Anomaly/config/plugins/<new-id>/config-nfs2015-chase-cam-settings-v1.json`
- 窗口位置/收藏：`Anomaly/state/ui-window-state.json` 里的 `pluginId` / `favorites`
- 旧安装目录：
  - 在线安装：`Anomaly/plugins/io.github.robineatcorn.nfs2015-chase-cam`
  - 手动安装：`Anomaly/plugins/Nfs2015ChaseCam`
  → 新在线安装目录：`Anomaly/plugins/robineatcorn.nfs2015-chase-cam`

建议：

- 仓库内提供 `migrate-2.7.ps1`（或至少 README 手动步骤）；
- 迁移脚本只做“预览 → 确认 → 复制/合并”，不删除旧数据；
- 如果不想做脚本，README 要明确写出“旧设置会重置，需要手动复制配置文件”。

**验收**：

- 旧配置复制到新目录后，插件启动能读到原设置。
- 旧文件夹删除/保留都不会导致重复插件或 ID 冲突。
- 新安装和升级安装都能正常启用。

---

### 任务组 3 ★★★ 高难度（可选）：恢复精确车速和刹车信号

**目标**：在不依赖已删除的 `anomaly.nte.vehicle` 的前提下，恢复 `GetForwardSpeedKmH` 级别的精确带符号速度。

**预计**：2–4 天。

**触发条件**：任务组 2 的实机验收中，FOV、刹车 pitch、倒车或低速接管仍然不可接受。否则不应投入。

**实现状态（2026-11-08）**：任务组 3 的代码已完成。3.1 使用公开 ABI 的 `ue5.objects` + `ue5.process-event`，3.3 增加 route B advisory witness。实机验收（HUD 速度一致性、倒车、重刹、换车、tick 耗时）仍需在 Windows 游戏内执行；见任务组 4。

#### 3.1 方案 A：用公开 ABI 重做最小载具查询

实际新增服务/capability：

- `anomaly.ue5.objects` + `ue5-objects`
- `anomaly.ue5.process-event` + `game-events`

未加 `anomaly.ue5.framework`：所有调用都发生在 Game tick，宿主 `invoke` 本身会拒绝
非 Game 线程；也未加 `anomaly.ue5.names`，函数和类通过 `find_exact` 路径解析。

数据流：

1. Game tick 调 `anomaly.nte.player::get_object_handles` 拿 pawn / controller 的 `ue5.objects` handle。
2. 用 `ue5.objects::find_exact` 找 UFunction（路径要用实机 `find_exact` 验证；不同 outer / 水上载具可能需要回退到反射扫描）：
   - `/Script/HTGame.HTPlayerController:BP_GetCurrentDriveVehicle`
   - 备用 `/Script/HTGame.HTPlayerCharacter:BP_GetCurrentDirvingVehicle`（游戏里的拼写就是 `Dirving`）
   - `/Script/HTGame.HTVehicleMovementComponent:GetForwardSpeedKmH`（陆上载具；水上载具是另一套接口，本插件只需要陆上）
   - 速度函数同时尝试 `Class:Function` 和 `Class.Function` 两种写法。
3. 用 `ue5.process-event::invoke` 调 `BP_GetCurrentDriveVehicle`，参数缓冲区 8 字节，读返回的 UObject 指针。
4. 把裸对象指针转成 `ue5.objects` handle：
   - 读对象 `InternalIndex` 和 `NameId`；
   - `objects.snapshot_at(index)`；
   - 比较 snapshot 的 `name_id` 与裸对象读出的 FName id，避免索引槽位变化后把旧指针映射成另一个对象。
   - 不扫描 GObjects；handle 的 serial/generation 由宿主 `snapshot_at` 负责。
5. 用返回的载具 handle 作为 receiver，调 `GetForwardSpeedKmH`，读 float 返回值；调用前用 `HTWheeledVehicleBase` 类链排除水上载具。
6. 不直接校验 `UFunction::ParmsSize`（公开 ABI 不暴露）；使用 32 字节输出缓冲区，让宿主 `invoke` 的参数容量校验失败时回退估计速度。
7. `ue5.objects.generation` 变化时清空函数、movement component、类链和载具缓存；函数/类解析失败按 2 秒退避重试，不扫描 `.text`。

缓存和失效：

- 按 `ue5.objects.generation` 缓存函数 handle 和载具 handle；
- 世界变化、对象 generation 变化、载具指针变化时全部失效；
- 只在 Game 线程调用；`invoke` 失败按 2 秒退避重试；
- 任何一步失败都不影响相机主路径；
- 水上载具或 getter 不可用时回退到任务组 2 的位置微分/角度倒车逻辑。

**验收**：

- 面板速度与游戏 HUD/载具速度一致。
- 倒车符号正确。
- 刹车 pitch 使用精确速度导数，和 1.1.1 行为接近。
- 关闭/缺失 `ue5.objects` 或 `ue5.process-event` 时自动回退，不等待、不崩溃。
- Game tick 没有明显耗时增长（用插件自己的 `相机 tick ms` 观察）。

**代码验收（已完成）**：构建通过、`ctest` 1/1 通过；DLL 内含
`GetForwardSpeedKmH`、`HTWheeledVehicleBase` 和 `anomaly.ue5.objects` /
`anomaly.ue5.process-event` 服务名。实机项仍待任务组 4。

#### 3.2 方案 B：上游提供只读接口

如果不想在相机插件里复制 GObjects/反射代码，可以向 Anomaly 上游请求：

- 让内建 `NteVehicle` 发布只读 IPC（当前载具 handle、`speed_kmh`、`forward_speed_kmh`）；
- 或者恢复一个轻量的只读 `anomaly.nte.vehicle.snapshot` 服务。

优点：相机插件保持干净；缺点：依赖上游排期，不在本仓库控制范围内。建议作为方案 A 的备选，不要阻塞 2.7.0 适配发布。

#### 3.3 版本门禁加固（可选但推荐）

当前门禁用“整份 Profile 的 SHA-256”，2.7.0 只是新增 pak/assets 就导致相机全关，过于脆弱。

**实现状态**：已维护 `kAcceptedProfileHashes[]`（安装版 CRLF + 仓库 LF 两个
2.7.0 hash）。route B 的运行时检查保留为 **advisory witness**：

- 全屏 view / 指针范围仍是应用条件；
- `ViewLocation` / `ViewRotation` / 投影矩阵 / `ViewFov` 的异常只累计到
  route B 行的 `w <count>`，**不跳过该帧**；
- 之前把 witness 作为写入门禁会让晚期 view 偶发回退到游戏相机，重新引入车辆
  抖动；route B 必须在每个符合条件的渲染帧确定性应用。
- 版本门禁仍然只由 hash/pattern/offset 决定，未知 hash 继续 fail-closed。

**验收**：

- 未知 hash 或 pattern/offset 失败时仍然 fail-closed。
- 面板能区分 `hash mismatch`、`pattern failed`；witness 仅作诊断，不改变写入。

**未实现项**：“只改 Profile 里与本插件无关的字段时自动放行”没有实现。公开 SDK 只
暴露整份 profile 的 SHA-256，无法判断变化是否与 route B/FOV 布局有关；为了实现这一条
而放宽 hash 会违背 fail-closed 原则。需要上游提供“相关布局字段的单独 hash/witness”
后再做。当前策略仍是 allowlist；witness 只做诊断。

---

### 任务组 4 ★★ 中等难度：回归、发布与用户迁移

**目标**：把适配从“能跑”变成“可发布”。

**预计**：0.5–1 天。

#### 4.1 测试矩阵

离线：

- `ctest` / `nfs2015_chase_cam_test`
- `anomaly-plugin validate` 新包
- `anomaly-test-host --reload 10 --ticks 60`
- 人为删除 `anomaly.ue5.build`、`anomaly.ue5.objects`、`anomaly.ue5.process-event`，确认降级路径不崩

实机：

| 场景 | 检查点 |
|---|---|
| 静止进车 | 立即接管，不要求先加速 |
| 静止下车 | 立即恢复游戏相机 |
| 0–200 km/h 加速 | FOV / 距离倍率连续 |
| 重刹 | 刹车 pitch 可感知、无抖动 |
| 倒车 | 相机保持在车后 |
| 漂移 | 偏航弹簧、侧倾、推近正常 |
| 上/下坡 | 坡度跟随正常 |
| 换车 | 模型重置，无旧姿态 |
| 地图切换/传送 | 不崩，门禁和锚点重建 |
| 过场/暂停 | 不接管，恢复后正常 |
| F7 / UI | 开关、设置保存、窗口位置正常 |
| 与 Camera Tools/BetterPose 共存 | 冲突提示正确，hook 争用可恢复 |

#### 4.2 发布

- 更新 `manifest.json` 版本和 ID。
- 更新 `pluginmaster.json`。
- 生成新 ZIP，确认根目录直接包含 `manifest.json`。
- Release note 写清：
  - 只支持 Anomaly 2.7.0+；
  - 旧 ID 不再加载；
  - 旧设置迁移方式；
  - 速度来源降级说明（如果任务组 3 未做）。
- 保留旧 Release 但标注“已不兼容 2.7.0”，避免用户继续安装。

#### 4.3 回滚方案

- 保留 1.1.1 的源码 tag 和 Release。
- 新版本如果出现 route B 崩溃/黑屏，用户可以：
  - 回退到 1.1.1 + Anomaly 2.6.0；
  - 或在 2.7.0 下关闭插件，等待修复。
- 发布前至少在两个不同地图、两次完整重启后验证。

---

## 4. 依赖关系与建议顺序

```text
任务组 1（加载/编译）
    │
    ▼
任务组 2（降级速度 + 载具存在性 + 门禁 + 实机验证）
    │
    ├── 验收通过 ──► 任务组 4（回归/发布）
    │
    └── 验收不通过（速度/刹车/倒车不可接受）
            │
            ▼
        任务组 3（精确车速，代码已完成）
            │
            ▼
        任务组 4（回归/发布）
```

建议先做 **1.1 → 1.2 → 1.3 → 2.1 → 2.3 → 2.2 → 2.4 → 4**。  
其中 2.3（载具存在性）比 2.2（速度估计）更能决定“相机是否像原来一样工作”，可以优先。

---

## 5. 风险与待拍板事项

1. **版本号**：ID 迁移是破坏性变更，建议 `2.0.0`；如果不想跳大版本，用 `1.2.0` 并强化迁移说明。
2. **精确车速**：任务组 3 已用公开 ABI 实现（不复制 GObjects 扫描），并保留位置微分回退；剩余风险是实机 HUD 速度一致性、刹车手感、换车后 handle 失效，需在任务组 4 验证。
3. **Profile hash 策略**：短期更新常量；中期改 allowlist 或 witness，避免每次 Anomaly 加功能都把相机全关。
4. **旧数据迁移**：是提供脚本，还是只写手动步骤？建议至少提供只读预览脚本，避免用户丢设置。
5. **route B 偏移**：Anomaly 2.7.0 没有改游戏二进制，但仍必须实机验证；如果 pattern 变化，按 fail-closed 原则更新，不能直接放宽。
6. **上游接口**：如果希望长期不再复制载具查询代码，应向 Anomaly/NteVehicle 提只读 IPC 需求；不要在本仓库硬依赖内建插件实现细节。

---

## 6. 验收定义（Definition of Done）

满足以下全部条件才算 2.7.0 适配完成：

- [ ] manifest ID 为 `robineatcorn.nfs2015-chase-cam`，`pluginmaster.json` 一致。
- [ ] manifest 不再包含 `anomaly.nte.vehicle` / `nte-vehicle`。
- [ ] 2.7.0 SDK 下 MSVC / MinGW 均编译通过。
- [ ] 精确车速 getter 与位置微分回退均可用，面板显示速度来源。
- [ ] `anomaly-plugin validate` 通过。
- [ ] `anomaly-test-host --reload 10 --ticks 60` 通过。
- [ ] 启动日志无 `unknown-capability`、无 capability denied、无必需服务等待死循环。
- [ ] 面板 `版本门禁 = ok`，route B / FOV / 视点 hook 正常。
- [ ] 静止进车立即接管；下车立即恢复。
- [ ] 加速、重刹、倒车、漂移、上下坡、换车、地图切换均通过实机检查。
- [ ] 旧设置迁移路径可用，或者 README 明确说明重置方式。
- [ ] Release note 明确“仅支持 Anomaly 2.7.0+，旧 ID 不再加载”。

---

## 7. 附录：调查命令与证据

```bash
# 2.7.0 关键提交
git -C Anomaly show --stat 71370ea   # 插件 ID 统一
git -C Anomaly show --stat 243a95c   # vehicle 服务移入插件

# 已删除的 capability
rg -n "nte-vehicle" Anomaly/src/plugin/plugin_capability_policy.cpp

# 已删除的 SDK 类型
rg -n "AnomalyNteVehicle" Anomaly/include/anomaly/sdk/services/nte.h

# Profile hash 对比
sha256sum Anomaly/profiles/nte/nte-current.json
git -C Anomaly show v2.6.0:profiles/nte/nte-current.json | sha256sum
git -C Anomaly show v2.7.0:profiles/nte/nte-current.json | sha256sum

# 当前插件中的旧依赖
rg -n "AnomalyNteVehicle|ANOMALY_NTE_VEHICLE|nte-vehicle|anomaly\\.nte\\.vehicle" \
  Anomaly-nfs19-cam
```

关键文件：

- `Anomaly-nfs19-cam/manifest.json`
- `Anomaly-nfs19-cam/plugin.cpp`
- `Anomaly-nfs19-cam/pluginmaster.json`
- `Anomaly-nfs19-cam/README.md`
- `Anomaly/profiles/nte/nte-current.json`
- `Anomaly/plugins/NteVehicle/native_access.hpp`（任务组 3 参考）
- `Anomaly/plugins/NteVehicle/runtime.hpp`（任务组 3 参考）
