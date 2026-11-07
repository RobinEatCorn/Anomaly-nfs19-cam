# NFS 2015 Chase Cam 2.0.0

Anomaly 2.7.0 适配版本，完成任务组 1（加载/编译）、任务组 2（无载具服务时的相机恢复）、任务组 3（精确车速 + route B advisory witness）和任务组 4（发布检查与回滚文档）的代码/文档。完整实机矩阵与 Windows 宿主工具验收仍需补跑。

## 破坏性变更

- 插件 ID 从 `io.github.robineatcorn.nfs2015-chase-cam` 改为
  `robineatcorn.nfs2015-chase-cam`，符合 Anomaly 2.7.0 的两段式
  `<作者>.<插件名>` 规则。
- **旧 ID 不再被 2.7.0 加载**。请先卸载 1.1.1，再安装本版本。
- 公开包目录改为 `Anomaly/plugins/robineatcorn.nfs2015-chase-cam`。旧的手动安装
  目录（通常为 `Anomaly/plugins/Nfs2015ChaseCam`）需要删除或改名。

## Anomaly 2.7.0 变更适配

- 移除已删除的 `anomaly.nte.vehicle` 服务依赖。
- 移除已删除的 `nte-vehicle` capability。
- 移除 `AnomalyNteVehicleServiceV1`、`AnomalyNteVehicleSnapshotV1` 及
  `ANOMALY_NTE_VEHICLE_*` 常量的全部引用。
- 保留 `anomaly.ue5.build` / `ue5-build`：route B、FOV 和视点 hook 仍受 profile hash
  的 fail-closed 门禁保护。
- profile 门禁改为 allowlist，接受两个已验证的 2.7.0 hash：
  - 安装 Runtime 的 CRLF profile：`c6d97f...1f52`
  - 仓库内 LF profile：`c37a7b...db2a`

  未知 hash 仍会关闭 route B / FOV / 视点 hook；日志会打印实际的 `got` 值。

## 任务组 2：相机功能恢复

- **载具存在性**：主路径恢复旧版 `anomaly.nte.vehicle` 内部的
  `BP_GetCurrentDriveVehicle` / `BP_GetCurrentDirvingVehicle` /
  `GetBoardWaterVehicle` getter，通过 `ue5.objects` + `ue5.process-event`
  以 20 Hz 轮询；静止进车立即接管。渲染线程仍从玩家胶囊 `AttachParent`
  （驾驶时的 `VehicleMesh`）读取车辆锚点，并在 getter 服务不可用时作为存在性后备。
  下车后有约 0.2 秒 grace，避免流式加载或单帧读取失败导致闪回。
- **带符号速度**：优先恢复 2.6 的 `HTVehicleMovementComponent:GetForwardSpeedKmH`
  精确值；轮式载具的 movement component 通过反射 handle 调用，失败时才使用
  `anomaly.nte.player` 的位置微分。这样 `car.speed`、倒车和刹车信号在正常情况下
  共用旧版的精确有符号速度。
- **倒车判定**：精确速度可用时直接使用符号（旧版无状态逻辑）；不可用时才用相机
  yaw 与速度方向的夹角推断，进入阈值约 134°、退出阈值
  约 114°，形成滞后；低速时保持上一次判断，避免漂移或相机摆动导致抖动。
- **刹车俯仰**：精确速度可用时使用 2.6 的 1.2 m/s² 死区和 8/12 平滑；只有位置微分
  回退路径才使用 2.2 m/s² 死区和更慢的 4/6 平滑，减少逐 tick 噪声。
- **诊断面板**：增加速度来源（`位置微分` / `精确` / `不可用`）、速度符号是否已知、
  载具存在和倒车状态。精确 getter 可用时显示 `精确`，否则显示 `位置微分`。
- **降级修复**：相机未接管时不再每 tick 清空位置历史，速度门限 fallback 可以正常
  累积速度；否则位置微分永远无法超过门限。
- **周期性卡顿修复**：如果 `FSceneView::UpdateViewMatrix` 之类 pattern 解析失败，
  现在会锁存失败并停留在 late view hook，不再每 2 秒重新扫描整个 `.text`。此前的
  重试会造成约 2 秒一次的帧卡顿，观感类似重刹。
- **长 tick 保护**：刹车俯仰不会把超过 100 ms 的帧间隔当成减速样本，避免一次卡顿
  被解释成重刹。
- **载具判定恢复旧版逻辑**：2.7.0 删除了 `anomaly.nte.vehicle` 服务，但没有删除
  `BP_GetCurrentDriveVehicle` / `BP_GetCurrentDirvingVehicle` /
  `GetBoardWaterVehicle`。插件现在通过 `ue5.objects` 和 `ue5.process-event`
  以 20 Hz 调用这组旧 getter；getter 明确返回空时不再使用速度兜底，因此步行/冲刺
  不会被误判成载具。速度兜底只在 getter 和 `AttachParent` 都不可用时启用。

## 任务组 3：精确车速与 route B witness

- **精确速度恢复**：通过 `ue5.objects.find_exact` 解析
  `HTVehicleMovementComponent:GetForwardSpeedKmH`，用 `ue5.process-event.invoke`
  读取带符号 km/h；同时尝试 `Class:Function` 和 `Class.Function` 两种路径。
- **载具类型门禁**：用 `HTWheeledVehicleBase` 类链排除水上载具，避免对错误的
  movement component 调用速度 getter。
- **缓存失效**：按 `ue5.objects.generation` 失效函数、movement component、载具
  和类链 handle；换图、流式加载或对象槽位重建后不会复用旧 handle。
- **安全解析**：裸车辆指针转 handle 时同时比较对象 `InternalIndex` 与 `NameId`；
  解析/调用失败按 2 秒退避重试，并自动回退位置微分。
- **route B advisory witness**：检查全屏 view、有限 location/rotation、投影矩阵
  行列式和 FOV 范围，但异常只累计到 route B 行的 `w <count>`，**不跳过当前帧**。
  route B 必须在每个符合条件的渲染帧确定性应用；把 witness 当作写入门禁会让
  晚期 view 偶发回退到游戏相机，重新引入车辆抖动。
- **速度显示**：诊断面板的速度直接显示精确 getter 的绝对值；速度来源显示
  `精确` 或 `位置微分`。

## 发布验证

完整矩阵、产物哈希和待执行项见 [RELEASE-CHECKLIST-2.0.0.md](RELEASE-CHECKLIST-2.0.0.md)。

- 离线：MinGW 2.7.0 SDK 构建、`ctest`、manifest schema、ID/版本/pluginmaster 一致性、
  ZIP 根目录、DLL 导入/导出均通过。
- 实机：插件加载、锚点门禁、精确 `GetForwardSpeedKmH` 和 route B 已观察正常；
  用户确认 route B 修复后车辆不再抖动。静止进出车、加速、重刹、倒车、漂移、换车、
  地图切换、F7/UI 与 Camera Tools/BetterPose 共存仍待实机勾选。
- Windows 宿主工具 `anomaly-plugin validate`、`anomaly-test-host --reload 10 --ticks 60`
  当前环境不可用，不能标记为通过；发布前需在 Windows 开发机补跑。

## 回滚

- **回退到 1.1.1**：旧 ID 不被 2.7.0 加载，需要同时回退 Anomaly Runtime 到 2.6.0；
  保留 `v1.1.1` tag/Release，旧设置目录不会被迁移脚本删除。
- **2.7.0 下禁用 2.0.0**：退出游戏，删除或改名
  `Anomaly/plugins/robineatcorn.nfs2015-chase-cam`（或在插件列表禁用后重启），
  然后收集 `anomaly-runtime.jsonl` 和 `anomaly-runtime.log`。
- 如果迁移脚本改过 `plugin-enablement.json` / `ui-window-state.json`，可用脚本生成的
  `.bak-*` 备份恢复旧 ID 条目。
- 出现 route B 黑屏/崩溃时不要放宽 profile hash 或删除门禁；保持 fail-closed 并附日志。

## 从 1.1.1 迁移

仓库新增 `migrate-2.7.ps1`。默认只预览，`-Apply` 才写入；不会删除旧设置或旧安装目录。
如果 Runtime 位于 `Program Files`，请用管理员身份运行 PowerShell：

```powershell
# 预览
powershell -ExecutionPolicy Bypass -File .\migrate-2.7.ps1 `
  -AnomalyRoot "C:\Program Files\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly"

# 写入
powershell -ExecutionPolicy Bypass -File .\migrate-2.7.ps1 `
  -AnomalyRoot "C:\Program Files\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly" `
  -Apply
```

脚本处理：

- `config/plugins/<old-id>/` → `config/plugins/<new-id>/`，只复制缺失文件；
- `config/plugin-enablement.json` 增加新 ID 的启用值；
- `state/ui-window-state.json` 迁移 `pluginWindows` 和 `favorites`；
- 写入前为 JSON 文件生成带时间戳的 `.bak-*` 备份。

手动迁移步骤见 `README.md` 的“从 1.1.1 升级”。

## 已知限制

- 精确车速已通过 `HTVehicleMovementComponent:GetForwardSpeedKmH` 恢复，但仅限轮式载具
  且 getter 调用成功时；水上载具或 getter 失败时，FOV、距离倍率和刹车俯仰仍回退到位置
  微分。精确速度路径尚未在运行中的游戏中逐项验证。
- 2.7.0 的 route B 偏移和 7 个 pattern 尚未在运行中的游戏里逐项实测；代码保持
  fail-closed，pattern/hash/读取失败时不会写入游戏内存。
- 载具存在性优先依赖游戏 getter；如果 `ue5.objects` / `ue5.process-event` 未发布，
  则退回 `AttachParent` 的距离/变换校验，最后才使用速度门限。

## 验证

本地已完成：

- MinGW-w64 使用 Anomaly 2.7.0 SDK 编译通过；
- `plugin.dll` 仅导入 `KERNEL32.dll` / `msvcrt.dll`，导出 `AnomalyPluginEntryV1`；
- `ctest` / `nfs2015_chase_cam_test` 通过；
- manifest 通过 2.7.0 JSON Schema 校验；
- ID/版本/pluginmaster、包内容、ZIP 根目录和字节一致性通过 `verify-release.sh`；
- 安装版 2.7.0 profile hash 已采集并加入 allowlist（`c6d97f...`）；
- `migrate-2.7.ps1` 在临时 Anomaly 目录上完成预览和 `-Apply` 测试。

发布产物：

- `plugin.dll`：2,217,943 bytes，SHA-256
  `7fbf360d3c49440c0a3c67f47d6c18dda8d15fa602850b839612031751c4b01b`；
- `Nfs2015ChaseCam-2.0.0.zip`：817,243 bytes，SHA-256
  `cd28d67b3f214676bf7b6aba8861a664958d7671c42c1b86f332501e263b4708`。

Windows/实机侧仍需复验：

- MSVC 构建（当前环境没有 Visual Studio）；
- `anomaly-plugin validate`；
- `anomaly-test-host --plugin ... --reload 10 --ticks 60`；
- 静止进车/下车、加速、倒车、重刹、漂移、上下坡、换车、地图切换、F7/UI 和
  Camera Tools/BetterPose 共存；至少两个地图、两次完整重启。
