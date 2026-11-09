# Phase 4 报告：设置、UI、本地化与收尾

- 日期：2026-11-10
- 状态：**PASS（未发布）**
- 范围：烧胎特写设置持久化、设置面板、诊断、默认值、文档收尾
- 产物：`plugin.cpp`、`locales/zh-CN.json`、`README.md`、本报告

## 1. 设置字段

`schemaVersion` 仍为 1。新字段全部是可选字段，旧文档只含原来的 31 个属性也能正常加载，
缺省值来自 `BurnoutShotConfig`：

| JSON 字段 | 默认 | 范围 | 说明 |
|---|---:|---:|---|
| `burnoutEnabled` | `true` | bool | 烧胎特写总开关 |
| `burnoutEntryDelay` | 0.5 s | 0..2 | 条件持续多久后 latch |
| `burnoutEntryTime` | 1.0 s | 0.1..4 | 进入 orbit 时长 |
| `burnoutExitDelay` | 0.2 s | 0..2 | 条件消失后的保持 |
| `burnoutExitTime` | 2.5 s | 0.1..5 | 退出 orbit 时长 |
| `burnoutLaunchExitTime` | 1.8 s | 0.1..5 | 正常起步的回正时长 |
| `burnoutFov` | 45° | 20..90 | 特写水平 FOV |
| `burnoutShake` | 1.0 | 0..2 | 特写震动倍率，独立于 chase shake |
| `burnoutSideMode` | 0 | 0..3 | 0 自动，1 左后，2 右后，3 居中 |
| `burnoutEntrySpeed` | 1.0 m/s | 0..10 | 进入速度上限 |
| `burnoutHoldSpeed` | 3.0 m/s | 0..20 | 保持速度上限，且不得低于 entry |

实现：

- `Context` 增加对应 atomics；render 线程每帧拷贝到 `BurnoutShotConfig`；
- `SettingsValid()` 之外新增 `BurnoutSettingsValid()`，检查有限值、范围、
  `sideMode` 整数、`holdSpeed >= entrySpeed`；
- `PersistSettings()` / `LoadSettings()` / `SetDefaults()` / `RestoreDefaults()` 全部覆盖；
- `kMaximumSettingsBytes` 从 2048 增至 4096；
- settings JSON 的字段数上限从 40 增至 60（当前 schema 42 个属性）。

`burnoutEnabled=false` 时 render 侧立即 `BurnoutShot::Reset()`，view 精确回到 chase；
不会留下一个已 latch 的 shot 在重新开启时直接满混合。

`cameraScale` 只由 presenter / route B 在混合之后应用一次。`BurnoutShotConfig.camera_scale`
固定为 1.0，否则 0.6 的全局 scale 会被平方成 0.36，把特写拉进车内。

## 2. 设置面板

在原有 chase 设置之后增加：

- 标题「烧胎特写（Shift + W）」；
- 启用复选框；
- 水平 FOV、震动强度；
- 进入延迟/进入时间、退出延迟/退出时间、起步回正时间；
- 进入速度、保持速度；
- 侧别四个按钮：自动/转向、左后轮、右后轮、居中，当前项用 `> ` 标出。

保持速度滑杆的下界跟随进入速度；调高进入速度时若保持速度更低，会自动抬到进入速度，
避免生成无法通过校验的文档。侧别在 shot latch 时锁定，面板提示「进入时锁定」。

## 3. 诊断

诊断面板新增：

```text
烧胎特写（混合/侧别/回退）
  blend 1.00  side R  fallbacks 0
```

`side` 来自 render 侧 latch，`fallbacks` 是 shot blend > 0 时 route B 取不到 pose 的次数。
原有的检测/样本/探针行保留。

## 4. 验证

### 构建与单测

```text
./build-mingw.sh --sdk ../Download/Anomaly-2.7.0-sdk --run-tests
plugin.dll 3.2M，PE32+ x86-64

ctest --test-dir build.mingw --output-on-failure
1/2 nfs2015_chase_cam_model ... Passed
2/2 nfs2015_burnout_shot_model ... Passed
100% tests passed, 0 tests failed out of 2
```

### 旧配置兼容

现有配置只有原来的 31 个字段。安装 generation 17 后正常加载，没有 settings/schema 错误；
新字段按默认值生效。

### 关闭开关的实机验证

临时写入带 10 个新字段、`burnoutEnabled=false`、`burnoutSideMode=2`、`burnoutFov=50`
的配置，重新加载 generation 18，再用高完整性 `PostMessage` 发送 Shift+W：

- 检测日志仍出现 `burnout sample active=1`；
- 但截图保持在 chase 视角，没有进入特写；
- 证明 `burnoutEnabled=false` 的加载、校验和 render 侧禁用路径有效。

测试后已恢复原配置（旧字段、无 burnout 键），并重新安装/构建，避免把测试状态留给用户。
默认 `burnoutEnabled=true` 的实机截图和路线见 `REPORT-burnout-phase3.md`。

## 5. 已知限制

- 没有 wall raycast：贴墙停车时特写机位可能进入墙体或轮胎烟体积。
- NTE 轮胎烟较不透明，近距离会遮住轮面；这是烟雾效果，不是机位状态错误。
- `CarState.on_ground` 目前固定为 `true`，腾空立即退出还没有真实 ground 信号。
- 侧别在进入时锁定；切换侧别按钮只影响下一次烧胎，避免进行中硬切。
- 没有做发布包/版本号变更；本阶段只提交源码、文档和设置。

## 6. 结论

Phase 4 **PASS（未发布）**。烧胎特写有独立设置、持久化、UI、诊断和旧配置兼容；
全量构建与 CTest 通过，关闭开关与默认开启路径均已实机验证。四个 phase 的实现完成。
