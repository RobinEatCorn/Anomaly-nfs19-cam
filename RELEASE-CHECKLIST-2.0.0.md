# NFS 2015 Chase Cam 2.0.0 发布检查与实机验收

适用计划：[PLAN-anomaly-2.7.md](PLAN-anomaly-2.7.md) 任务组 4  
目标宿主：Anomaly 2.7.0+  
检查日期：2026-11-08

## 状态摘要

**发布候选已准备；离线检查通过；Windows 宿主工具与完整实机矩阵仍待补跑。**

- 代码、manifest 和发布 ZIP 已完成。
- MinGW 构建、`ctest`、manifest ID/版本一致性、包内容、ZIP 根目录、DLL
  导入/导出均已通过。
- 用户已实机确认：route B 抖动修复后车辆不再抖动。
- 当前环境没有 `anomaly-plugin.exe`、`anomaly-test-host.exe`、Visual Studio/MSVC，
  也没有可自动执行的游戏内验收环境；这些项目不能标记为通过。

## 发布产物

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `build.mingw/package/robineatcorn.nfs2015-chase-cam/plugin.dll` | 2,217,943 | `7fbf360d3c49440c0a3c67f47d6c18dda8d15fa602850b839612031751c4b01b` |
| `dist/Nfs2015ChaseCam-2.0.0.zip` | 817,243 | `cd28d67b3f214676bf7b6aba8861a664958d7671c42c1b86f332501e263b4708` |

ZIP 根目录只有：

```text
manifest.json
plugin.dll
locales/zh-CN.json
```

`plugin.dll` 仅导入 `KERNEL32.dll`、`msvcrt.dll`，导出 `AnomalyPluginEntryV1`。
`manifest.json` 的 ID 为 `robineatcorn.nfs2015-chase-cam`，版本为 `2.0.0`。

## 4.1 测试矩阵

### 离线

| 项目 | 状态 | 结果 / 命令 |
|---|---|---|
| MinGW 2.7.0 SDK 构建 | **PASS** | `./build-mingw.sh --sdk /tmp/anomaly-sdk-2.7.0` |
| 相机模型验收 `ctest` | **PASS** | `ctest --test-dir build.mingw --output-on-failure`，1/1 |
| manifest JSON Schema | **PASS** | `verify-release.sh`，Anomaly 2.7.0 schema |
| ID/版本/pluginmaster 一致 | **PASS** | `verify-release.sh` |
| 旧 vehicle 服务/capability 清除 | **PASS** | `verify-release.sh` |
| 包根目录与 locale | **PASS** | `verify-release.sh` |
| ZIP 根目录与字节一致性 | **PASS** | `verify-release.sh`，3 个文件 |
| DLL 导入/导出 | **PASS** | `verify-release.sh` / `objdump` |
| `anomaly-plugin validate` | **BLOCKED** | 当前环境没有该 Windows 工具 |
| `anomaly-test-host --reload 10 --ticks 60` | **BLOCKED** | 当前环境没有该 Windows 工具 |
| 删除可选服务后的降级加载 | **BLOCKED** | 需要 TestHost/实机；代码按可选服务处理，但没有运行时证据 |
| MSVC / VS2022 构建 | **BLOCKED** | 当前环境没有 `cl.exe` |

离线发布门禁：

```sh
./verify-release.sh --allow-missing-host-tools
# 在装有 Anomaly 开发工具的 Windows 环境应去掉 --allow-missing-host-tools，
# 缺工具时脚本会返回 2，而不是把 blocked 当作通过。
```

### 实机

日志位置：

```text
C:\Program Files\Neverness To Everness\Client\WindowsNoEditor\HT\Binaries\Win64\Anomaly\logs\anomaly-runtime.jsonl
```

本次已观察到的启动证据（generation 11–13）：

- `anchor gate: ready`，profile hash 为安装版 `c6d97f...1f52`；
- `exact GetForwardSpeedKmH active`；
- `route B early=0 late=12 fallbacks=0/1`；
- 没有 route B 写入崩溃或 hook 安装失败；
- 用户确认车辆抖动已消失。

`witness=12` 是 advisory 计数：它表示部分 view 字段检查不通过，但 route B 仍然
应用，不再因此跳过帧。该计数不会影响车辆稳定性；如果后续要清理这个诊断噪声，
需要单独确认 FOV/投影字段，而不是恢复“检查失败就跳过”。

| 场景 | 状态 | 检查点 / 备注 |
|---|---|---|
| 插件加载、锚点门禁、精确速度 | **PASS** | 2.7.0 日志；见上 |
| route B 确定性应用、车辆不抖 | **PASS** | 用户实机确认；late=12，fallbacks=0/1 |
| 静止进车立即接管 | **PENDING** | 需要实机 |
| 静止下车立即恢复游戏相机 | **PENDING** | 需要实机 |
| 0–200 km/h 加速，FOV/距离连续 | **PENDING** | 需要实机 |
| 重刹 pitch 可感知、无抖动 | **PENDING** | 需要实机 |
| 倒车时相机保持在车后 | **PENDING** | 需要实机 |
| 漂移时偏航弹簧、侧倾、推近 | **PENDING** | 需要实机 |
| 上/下坡坡度跟随 | **PENDING** | 需要实机 |
| 换车后模型重置、无旧姿态 | **PENDING** | 需要实机；同时观察 generation 变化 |
| 地图切换/传送，不崩、锚点重建 | **PENDING** | 需要实机 |
| 过场/暂停不接管，恢复后正常 | **PENDING** | 需要实机 |
| F7、设置保存、窗口位置 | **PENDING** | 需要实机 |
| Camera Tools / BetterPose 共存 | **PENDING** | 需要实机；观察 hook 争用提示 |
| 两个不同地图、两次完整重启 | **PENDING** | 发布前至少各两次 |

实机验收时请同时记录：

1. 诊断面板的 `版本门禁`、`路线B 应用/回退`、`w`、`渲染龄`；
2. 日志中的 `route B ...` 行和任何 hook/pattern/profile 错误；
3. 出现问题时使用的车辆、地图、速度区间和是否可复现。

## 4.2 发布步骤

1. 确认 `manifest.json`、`pluginmaster.json` 和 `plugin.cpp` 中的 ID/版本一致。
2. 构建并生成 ZIP：

   ```sh
   ./build-mingw.sh --sdk /tmp/anomaly-sdk-2.7.0 --zip
   ```

   没有 `zip` 命令时脚本会使用 Python `zipfile`，并校验 ZIP 根目录。

3. 跑离线门禁：

   ```sh
   ./verify-release.sh --allow-missing-host-tools
   ```

   在 Windows 开发机上跑完整门禁：

   ```powershell
   ctest --test-dir .\build.mingw --output-on-failure
   anomaly-plugin validate .\build.mingw\package\robineatcorn.nfs2015-chase-cam
   anomaly-test-host --plugin .\build.mingw\package\robineatcorn.nfs2015-chase-cam --reload 10 --ticks 60
   ```

4. 上传不可变的 GitHub Release 资源 `Nfs2015ChaseCam-2.0.0.zip`，再更新
   `pluginmaster.json` 的下载链接。先传 ZIP，再改索引，避免用户看到不存在的链接。
   本次已发布：https://github.com/RobinEatCorn/Anomaly-nfs19-cam/releases/tag/v2.0.0
5. Release 说明必须包含：
   - 仅支持 Anomaly 2.7.0+；
   - 旧 ID `io.github.robineatcorn.nfs2015-chase-cam` 不再加载；
   - 旧设置的手动迁移步骤（README）；
   - 速度来源：优先精确 `GetForwardSpeedKmH`，失败时位置微分；
   - route B advisory witness 不跳过帧。
6. 保留旧 `v1.1.1` tag/Release，并将旧 Release 标注为“仅兼容 Anomaly 2.6.0，
   不兼容 2.7.0”，避免用户继续安装。
7. 发布前完成上表至少两个地图、两次完整重启的实机勾选。

## 4.3 回滚方案

### 回退到 1.1.1（旧宿主）

2.0.0 的旧 ID 不会被 2.7.0 加载，1.1.1 也不能在 2.7.0 下工作。需要同时回退
Anomaly Runtime：

1. 关闭游戏和 Anomaly。
2. 保留 `v1.1.1` 的源码 tag 与 Release；按旧 Release 说明安装 1.1.1。
3. 将 Anomaly 回退到 2.6.0 Runtime（或对应的 1.1.1 兼容版本）。
4. 恢复旧设置目录（`config/plugins/io.github.robineatcorn.nfs2015-chase-cam/`）；
   旧目录没有被自动删除，所以数据仍在。
5. 如果手动改过 `plugin-enablement.json` / `ui-window-state.json`，从操作前的备份
   恢复旧 ID 条目。

### 2.7.0 下禁用 2.0.0

如果出现 route B 黑屏/崩溃，但不想回退整个 Runtime：

1. 退出游戏。
2. 删除或改名
   `Anomaly/plugins/robineatcorn.nfs2015-chase-cam`，或在插件列表禁用后重启。
3. 确认没有残留 `plugin.dll` 被加载；重新启动游戏应恢复游戏原相机。
4. 收集 `anomaly-runtime.jsonl` 和 `anomaly-runtime.log`，附上地图、车辆、复现步骤。
5. 在修复前不要通过放宽 profile hash 或删除门禁来“绕过”黑屏；保持 fail-closed。

## 发布签字

| 项目 | 负责人 | 状态 |
|---|---|---|
| 代码/包/manifest | RobinEatCorn | **READY** |
| 离线门禁 | 本地 WSL/MinGW | **PASS（宿主工具 blocked）** |
| Windows 宿主工具 | 待执行 | **BLOCKED** |
| 实机矩阵 | 待执行 | **部分 PASS，其余 PENDING** |
| GitHub Release 上传 | RobinEatCorn | **DONE**：https://github.com/RobinEatCorn/Anomaly-nfs19-cam/releases/tag/v2.0.0 |
