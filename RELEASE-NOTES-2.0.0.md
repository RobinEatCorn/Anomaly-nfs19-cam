# NFS 2015 Chase Cam 2.0.0

Anomaly 2.7.0 适配的第一个版本，只完成“能识别、能加载、能编译”的任务组 1。

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
- 车速暂时改由 `anomaly.nte.player` 的玩家位置微分估计；不再有精确带符号车速，
  因此倒车判定和刹车俯仰目前是降级状态。
- 保留 `anomaly.ue5.build` / `ue5-build`：route B、FOV 和视点 hook 仍受 profile hash
  的 fail-closed 门禁保护。

## 兼容性

- 仅支持 Anomaly 2.7.0+。
- 需要 Anomaly Plugin API v1（manifest 中仍为 1.0）。
- 2.0.0 尚未更新 2.7.0 的 profile hash，也尚未用 `AttachParent` 恢复载具存在性；
  这些属于 `PLAN-anomaly-2.7.md` 任务组 2。在那之前，2.7.0 上的相机可能仍保持
  游戏原相机，或只在车速超过门限后接管。

## 从 1.1.1 迁移

1. 在 **Plugins > 已安装** 中停用旧版并退出游戏。
2. 删除或改名旧安装目录。
3. 复制设置目录：
   - 旧：`Anomaly/config/plugins/io.github.robineatcorn.nfs2015-chase-cam/`
   - 新：`Anomaly/config/plugins/robineatcorn.nfs2015-chase-cam/`

   设置文件名不变：
   `config-nfs2015-chase-cam-settings-v1.json`。
4. 如需保留窗口位置/收藏，把 `state/ui-window-state.json` 中旧 ID 的条目替换为新 ID。
5. 首次启动后重新勾选插件启用状态。

## 验证

- `ctest` / `nfs2015_chase_cam_test`
- `anomaly-plugin validate`
- `anomaly-test-host --plugin ... --reload 10 --ticks 60`
- MinGW 构建通过；MSVC 使用同一个 CMake 工程，发布前需在 Windows/VS2022 环境复验。
