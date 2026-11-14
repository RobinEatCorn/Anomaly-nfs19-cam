# 🚀 阶段性实施计划：设置面板分组与三态配置

> 目标仓库：`Anomaly-nfs19-cam/`
> 本文档只描述本阶段计划，不包含 git 提交动作。

## 1. 当前阶段目标与宏观共识

把当前约 30 个控件堆在同一个长列表里的设置窗口，改造成“基础类型 + 三态模式”的紧凑面板：

- 顶部只保留全局项：
  - **启用相机**（`enabled`）
  - **预设**（`preset`：NFS 2015 / Cinematic）
  - **恢复默认值**
- 其余配置拆成 7 个基础类型：
  1. **NFS 镜头（FOV）**：默认（开）/ 关闭；没有详细参数。
  2. **机位几何**：默认 / 自定义；没有“关闭”态。
  3. **镜头弹簧**：默认 / 自定义；没有“关闭”态；只在 NFS 预设下显示。
  4. **镜头动态**：默认 / 自定义 / 关闭。
  5. **坡度跟随**：默认 / 自定义 / 关闭。
  6. **漂移前轮**：默认 / 自定义 / 关闭。
  7. **烧胎特写**：默认 / 自定义 / 开启。
- **高级** 作为默认折叠的普通区域，不做三态：
  - 锚点来源（`anchorSource`）
  - 锁定游戏相机强度（`borrowStrength`）
  - 诊断开关（`diagnosticFlatPose` / `diagnosticZeroLag` / `diagnosticSkipPosition` / `diagnosticSkipRotation`）
  - 诊断计数面板开关及内容
- 三态语义：
  - **默认**：使用插件内置默认参数 + 默认开关；不显示详细控件。
  - **自定义**：使用该组自定义参数 + 功能强制开启；只有此状态显示详细控件。
  - **反向**：使用内置默认参数 + 与默认相反的开关；不显示详细控件。
- 切换模式时，自定义值保留在内存和配置中但不生效；切回“自定义”时恢复。首次进入“自定义”以默认值为起点。
- **FOV 的归属**：单独作为“NFS 镜头”基础类型，不塞进“机位几何”。原因是“自定义 = 功能强制开启”与“关闭 FOV 但保留自定义几何”会互相冲突；单独成组后语义最干净。
- 预设保留：`默认` 统一指插件内置默认常量，不随预设变化；Cinematic 预设下隐藏“镜头弹簧”组（该预设本身忽略这些弹簧值）。
- 旧配置兼容：保留现有 JSON 键与 schema 版本 1，新增每组 `mode` 字段；旧配置没有 mode 时按值自动推断，不丢用户数据。
- 当前“漂移前轮”的超长说明从 UI 移除，UI 只保留一行摘要，完整说明移到 `README.md`。
- 不提交 git。

## 2. 微观决策授权范围

执行 AI 可以自行决定，不必逐项请示：

- 模式按钮的具体布局（同一行、两行、2 列表格、是否使用 `same_line`/`begin_table`）、间距、分隔线样式。
- 模式字段在 JSON 中使用整数枚举还是字符串；字段名只要保持可读、稳定即可。
- `CustomSettings` 用互斥锁保护还是全部改成原子量；只要保证 UI 线程写、Game tick 读写安全。
- 模式解析函数放在 `settings.cpp` 还是新建 `src/settings_groups.cpp`；如新建文件，自行更新 `CMakeLists.txt`。
- 旧配置推断的具体实现方式、错误兜底策略、日志/注释风格。
- 本地化 key 的命名与具体中文措辞；英文 fallback 必须保留。
- 是否为模式解析/迁移增加小型单元测试；若增加，自行选择测试入口。
- 切换组模式时是否请求相机重置；建议只在大幅改变机位/弹簧时请求重置，其余平滑过渡。
- 缩短后的提示文案、README 的组织方式、发布说明是否顺手更新。
- 构建与验证使用的具体命令和参数。

**不要自行改变**：分组划分、三态语义、“自定义=强制启用”、预设行为、FOV 独立成组、旧配置兼容策略。若认为这些需要调整，按第 3 节汇报。

## 3. ⚠️ 异常汇报机制

**请执行 AI 注意**：计划赶不上变化。如果执行过程中发现实际情况与本计划有明显出入、基础假设被推翻、或遇到重大阻碍（例如 host 配置服务不接受同版本追加字段、UI 服务缺少所需控件、构建环境不可用、旧配置推断会覆盖用户数据），**请立即停止后续执行，并向用户汇报具体冲突点、已尝试的方案和可选路径**。用户会把结论转交给“规划 AI”重新规划。

## 4. 任务步骤分解

### 🔍 任务组 0：前期探查与基线验证（必须优先执行）

- **难度评级**：⭐ [简单]
- **探查目标**：
  - 验证 host 配置服务对“同 schema 版本追加可选属性”的行为。
  - 验证新增字段后文档大小仍低于插件的 `kMaximumSettingsBytes`。
  - 验证 UI 服务具备所需控件，且 430px 宽度下布局可行。
  - 建立当前代码的构建/测试基线。
- **探查行动项**：
  - [x] 阅读 `Anomaly/src/plugin/scoped_platform_services.cpp` 中的 `register_schema` / `read` / `write_atomic`：确认 schema 版本 1 下增加可选属性、旧文档读取、`additionalProperties:false` 的行为；确认热重载时同版本但 schema 文档变化是否会产生注册冲突。若存在冲突风险，先记录，不要直接改 schema 版本。
  - [x] 估算当前 `PersistSettings` 生成的 JSON 大小，并确认新增 7 个 mode 字段后仍小于 `kMaximumSettingsBytes`（4096）。若接近上限，先汇报，不要擅自改成嵌套大对象。
  - [x] 检查 `AnomalyUiServiceV1` 的 `same_line`、`button`、`text`、`separator`、`begin_table` 可选字段，以及托管窗口 `initial_height=0` 时的自动高度行为；确认 430px 宽度下中文模式按钮行是否放得下。
  - [x] 运行当前基线构建：`./build-mingw.sh --run-tests`（若环境只能编译不能跑测试，记录原因）。
- **⚠️ 探查汇报**：完成此任务组后，请立即向用户输出“探查完毕”，并总结上述四项结论。若任何一项推翻本计划假设，停止并汇报，等待规划 AI 的新方案。

### 📦 任务组 1：设置模型与三态解析

- **难度评级**：⭐⭐⭐ [困难]
- **目标**：在不改动现有相机/车轮/烧胎消费者读取方式的前提下，引入“每组 mode + 自定义值快照 + 有效值解析”。
- **关键位置**：
  - `src/state.hpp`：`Context` 设置原子量（约 1285 行起）、`DisplayState`（约 826 行起）。
  - `src/settings.cpp`：`SetDefaults`（约 451 行）、`RestoreDefaults`（约 534 行）。
  - `src/display.cpp`：`UpdateDisplay`（约 10 行起）。
- **具体行动项**：
  - [x] 定义 `SettingsGroup`（NfsLens / Rig / Springs / Dynamics / Slope / WheelSteer / Burnout）和 `GroupMode`（Default / Custom / Opposite）。
  - [x] 定义 `CustomSettings`，只保存“自定义模式下可编辑的值”：
    - 机位几何：`cameraScale`、`originOffsetCm`、`aspectRatio`、`speedDistanceScale`
    - 弹簧：7 个 spring
    - 动态：`rollScale`、`pitchScale`、`shakeScale`、`rollOnlyInDrift`
    - 坡度：`slopeStrength`
    - 前轮：7 个 wheel-steer 参数
    - 烧胎：8 个参数 + `burnoutSideMode`
    - 不保存 enable 布尔量；enable 由 mode 推导（Custom 强制开启）。
  - [x] 在 `Context` 中加入：
    - `std::atomic<std::uint32_t> group_modes[kSettingsGroupCount]`
    - `CustomSettings custom_settings`
    - 保护 `custom_settings` 的互斥量（或等价的线程安全方案）
  - [x] 在 `DisplayState` 中加入 `group_modes[]`，由 `UpdateDisplay` 复制，供 UI 绘制模式按钮。
  - [x] 实现 `ApplyGroup(Context&, SettingsGroup)` / `ApplyAllGroups(Context&)`：
    - 现有 `context.camera_scale`、`context.wheel_steer_enabled` 等原子量继续作为**有效值**，引擎消费者完全不改。
    - `ApplyGroup` 是分组有效值的唯一写入口；UI 不再直接写这些有效值原子量。
    - 解析规则：

      | 组 | 默认 | 自定义 | 反向 |
      |---|---|---|---|
      | NFS 镜头 | FOV 开 | 不支持 | FOV 关 |
      | 机位几何 | 内置几何 | 自定义几何 | 不支持 |
      | 镜头弹簧 | 内置弹簧 | 自定义弹簧 | 不支持 |
      | 镜头动态 | roll/pitch/shake=1，rollOnly=true | 自定义值，rollOnly 自定义，功能开启 | roll/pitch/shake=0，rollOnly=true |
      | 坡度跟随 | slopeFollow=true，strength=1 | slopeFollow=true，strength 自定义 | slopeFollow=false，strength=1 |
      | 漂移前轮 | enabled=true + 默认参数 | enabled=true + 自定义参数 | enabled=false + 默认参数 |
      | 烧胎特写 | enabled=false + 默认参数 | enabled=true + 自定义参数 + 自定义侧别 | enabled=true + 默认参数 + 默认侧别 |

  - [x] `SetDefaults` 改为：重置 master/preset/高级项、把 `custom_settings` 重置为默认、所有 mode 置 Default，然后 `ApplyAllGroups`。
  - [x] `RestoreDefaults` 继续调用 `SetDefaults`，并设置 `reset_requested` + `MarkSettingsDirty`。
  - [x] 注意 `diagnosticFlatPose` 的运行时覆盖逻辑保持不变；它仍在 `presenter.cpp` 中压过动态/FOV/坡度，属于高级诊断。
  - [x] 所有对 `custom_settings` 的读写都遵守“先改自定义值，释放锁，再 `ApplyGroup`，最后 `MarkSettingsDirty`”的顺序，避免死锁和半更新。
- **验收**：编译通过；默认设置与现有默认行为一致；切换 mode 后 `UpdateDisplay` 能看到有效值变化。

### 📦 任务组 2：持久化与旧配置迁移

- **难度评级**：⭐⭐⭐ [困难]
- **目标**：schema 版本 1 下追加每组 mode 字段；旧配置自动推断；自定义值不丢失。
- **关键位置**：
  - `src/constants.hpp`：`kSettingsSchema`（约 575 行）。
  - `src/settings.cpp`：`PersistSettings`（约 199 行）、`LoadSettings`（约 593 行）。
- **具体行动项**：
  - [x] 在 schema JSON 中追加 7 个可选整数字段：
    - `nfsLensMode`、`rigMode`、`springsMode`、`dynamicsMode`、`slopeMode`、`wheelSteerMode`、`burnoutMode`
    - `minimum:0, maximum:2`
    - `required` 列表保持不变；`additionalProperties:false` 保持不变。
  - [x] `PersistSettings` 写入：
    - 每个组的 mode
    - `custom_settings` 中的自定义值写入现有键（`cameraScale`、`positionSpring`、...、`burnout*`、`wheelSteer*`、`slopeStrength`、`rollScale`、`pitchScale`、`shakeScale`、`rollOnlyInDrift`）
    - 组的 enable 布尔量写**有效值**：`fovEnabled`、`slopeFollow`、`wheelSteerEnabled`、`burnoutEnabled`
    - 高级项保持现有键与写法
  - [x] `LoadSettings` 读取：
    - 先读现有键到局部变量/`custom_settings`
    - 用与现有代码相同的方式做范围校验（复用 `SettingsValid` / `BurnoutSettingsValid` / `WheelSteerSettingsValid`）
    - mode 字段用“可选读取”：缺省或非法时留 sentinel，随后推断
    - 最后写 master/preset/高级原子量、写 `custom_settings`、写 mode、`ApplyAllGroups`
  - [x] 旧配置推断规则（mode 缺省时）：
    - **NFS 镜头**：`fovEnabled=true` → Default；`false` → Opposite
    - **机位几何**：4 个值都等于默认 → Default；否则 Custom
    - **镜头弹簧**：7 个值都等于默认 → Default；否则 Custom
    - **镜头动态**：3 个 scale 都为 0 → Opposite；都等于默认且 `rollOnlyInDrift=true` → Default；否则 Custom
    - **坡度跟随**：`slopeFollow=false` → Opposite；`true` 且 strength 等于默认 → Default；否则 Custom
    - **漂移前轮**：`wheelSteerEnabled=false` → Opposite；`true` 且 7 个参数都等于默认 → Default；否则 Custom
    - **烧胎特写**：`burnoutEnabled=false` → Default；`true` 且参数都等于默认 → Opposite；否则 Custom
    - 优先级：先看 enable 是否为默认的反向，再看参数是否等于默认。例如旧配置 `burnoutEnabled=false` 但参数是自定义的，应推断为 Default（功能关，参数保留），不要推断成 Custom 把功能打开。
  - [x] 记住“现有键的双重含义”并写注释：
    - 分组参数键保存**自定义值**（即使当前 mode 不是 Custom，也保留，供切回 Custom 使用）。
    - 组 enable 键保存**有效值**（供旧版本/降级读取时行为一致）。
  - [x] 确认文档大小仍小于 4096；mode 字段不要用嵌套大对象。
  - [x] 手工构造/读取一份旧版 JSON（无 mode 字段）和一份新版 JSON，验证推断与往返持久化不丢值。
- **验收**：旧配置加载后行为不变；新配置重启后 mode 与自定义值都保留；非法 mode 安全回落到 Default。

### 📦 任务组 3：UI 分组重构

- **难度评级**：⭐⭐ [中等]
- **目标**：把 `src/ui.cpp` 的 `Draw` 改成分组面板；非 Custom 状态只显示组标题和模式按钮。
- **关键位置**：
  - `src/ui.cpp`：`Draw`（约 616 行起），当前设置控件从约 670 行到 1200 行。
  - `src/display.cpp`：`UpdateDisplay` 提供 `DisplayState`。
- **具体行动项**：
  - [ ] 把设置部分拆成若干 helper（例如 `DrawNfsLensGroup`、`DrawRigGroup`、`DrawSpringsGroup`、`DrawDynamicsGroup`、`DrawSlopeGroup`、`DrawWheelSteerGroup`、`DrawBurnoutGroup`、`DrawAdvancedGroup`），每个 helper 只负责自己组的标题、模式按钮和 Custom 详情。
  - [ ] 顶部保留：启用相机 checkbox、预设按钮/当前预设、恢复默认值按钮。
  - [ ] 每组绘制：
    - 组标题
    - 模式按钮行：`默认` / `自定义`（如适用）/ `关闭` 或 `开启`（如适用）
    - 当前选中项用 `> ` 前缀或等价的选中样式
    - 只有 `Custom` 时绘制详情
  - [ ] **控件 ID 必须唯一**：多个组都有“默认/自定义/关闭”按钮，使用 `Localizer::Label(key, fallback, "group-<name>-mode-default")` 这类稳定 ID，避免 ImGui ID 冲突。滑块继续使用现有的唯一 `widget_key`。
  - [ ] NFS 镜头组：只画“默认 / 关闭”两态；不画 Custom 详情。
  - [ ] 机位几何 / 镜头弹簧：只画“默认 / 自定义”；Custom 时画各自滑块。
  - [ ] 镜头动态：默认 / 自定义 / 关闭；Custom 时画 roll/pitch/shake + rollOnly 复选框；关闭时隐藏详情。
  - [ ] 坡度跟随：默认 / 自定义 / 关闭；Custom 时只画 strength 滑杆；关闭时隐藏详情。
  - [ ] 漂移前轮：默认 / 自定义 / 关闭；Custom 时画 7 个滑杆 + 一行摘要提示；完整长说明移到 README。
  - [ ] 烧胎特写：默认 / 自定义 / 开启；Custom 时画 8 个滑杆 + 侧别按钮；默认/开启时隐藏详情。注意保留“hold speed 不得低于 entry speed”的现有约束。
  - [ ] 预设为 Cinematic 时隐藏“镜头弹簧”组；切回 NFS 时恢复显示。弹簧的自定义值仍然保留。
  - [ ] 高级区域：默认折叠；展开后按现有逻辑画锚点来源、`borrowStrength`、4 个诊断开关，以及诊断计数 checkbox/内容。
  - [ ] 状态行、重置相机按钮、Hook 冲突提示、底部 Z 键提示保持现有逻辑，位置放在分组之后。
  - [ ] 所有滑杆/复选框修改时：先写 `custom_settings`，再 `ApplyGroup`，再 `MarkSettingsDirty`；不要再直接写有效值原子量。
  - [ ] 模式按钮点击时：更新 `group_modes`，`ApplyGroup`，`MarkSettingsDirty`；是否 `reset_requested` 由执行 AI 按组的视觉影响自行决定。
  - [ ] 保持 `try/catch` 边界和窗口 begin/end 配对逻辑不变。
- **验收**：默认状态下窗口明显变短；每个组的模式都能隐藏/显示详情；切换模式后下次打开窗口状态保持；没有重复控件 ID 导致的误点击。

### 📦 任务组 4：本地化与文档

- **难度评级**：⭐ [简单]
- **目标**：新增分组/模式文案；把长说明移到文档。
- **具体行动项**：
  - [ ] 在 `locales/zh-CN.json` 增加：
    - 7 个组名（例如 `group.nfs_lens`、`group.rig`、`group.springs`、`group.dynamics`、`group.slope`、`group.wheel_steer`、`group.burnout`、`group.advanced`）
    - 模式按钮文案（`mode.default`、`mode.custom`、`mode.off`、`mode.on`）
    - 高级区展开/收起文案
    - 每组 Custom 的一行摘要（尤其是漂移前轮）
  - [ ] 代码中的英文 fallback 保持可读；不要只依赖 zh-CN。
  - [ ] 把当前 `setting.wheel_steer_hint` 的完整内容移到 `README.md` 的前轮反打章节；UI 只保留一行摘要。
  - [ ] 在 README 增加“设置面板”小节，说明：
    - 基础类型与三态语义
    - 默认/自定义/反向分别代表什么
    - FOV 是独立组
    - 弹簧只在 NFS 预设下生效
    - 旧配置会自动推断 mode，不会丢失自定义值
  - [ ] 可选：更新 `RELEASE-NOTES-*.md` 或新增一条简短说明；不作为本阶段硬性要求。
- **验收**：UI 中文文案完整；切到英文/无本地化时 fallback 可读；README 能找到完整前轮说明。

### 📦 任务组 5：构建、测试与交付验证

- **难度评级**：⭐⭐ [中等]
- **目标**：确认改动可编译、可打包，并给用户一份可执行的验收清单。
- **具体行动项**：
  - [ ] 运行 `./build-mingw.sh --run-tests`（或按任务组 0 记录的可用方式构建）。
  - [ ] 运行现有测试：`nfs2015_chase_cam_test.exe`、`nfs2015_wheel_steer_test.exe`、`nfs2015_burnout_shot_test.exe`（若环境支持）。
  - [ ] 确认 `plugin.dll` 的导入/导出校验通过；确认 `build.mingw/package/robineatcorn.nfs2015-chase-cam/` 内容完整。
  - [ ] 检查 `locales/zh-CN.json` 是合法 JSON。
  - [ ] 给用户输出手动验收清单：
    - 默认打开面板时只有组标题和模式按钮，没有大量滑杆。
    - “NFS 镜头”默认/关闭能切换 FOV。
    - 机位几何、弹簧切 Custom 才出现滑杆；切回 Default 后滑杆消失，行为回到默认。
    - 动态/坡度/前轮切“关闭”后对应功能停用；切 Custom 恢复自定义值。
    - 烧胎默认不启用；切“开启”用默认参数；切 Custom 才出现参数。
    - 自定义值在模式切换、重启后保留。
    - 旧配置文件（无 mode 字段）加载后行为与升级前一致。
    - Cinematic 预设下不显示弹簧组。
    - 高级区默认折叠；诊断面板仍可打开。
  - [ ] **不要执行 `git commit` / `git push`**；只保留工作区改动和计划文件。
- **验收**：构建与测试通过；用户可按清单在游戏内验证 UI。

## 5. 完成定义（Definition of Done）

- [ ] 设置面板默认高度显著缩短，日常配置只需看组标题 + 模式按钮。
- [ ] 7 个基础类型按本文档定义工作；FOV 独立成组。
- [ ] 三态语义完全符合第 1 节；“自定义”是唯一显示详情的状态。
- [ ] 旧配置加载不丢值；新配置重启后 mode 与自定义值都保留。
- [ ] 现有相机、前轮、烧胎、坡度、诊断行为在默认设置下与改动前一致。
- [ ] 构建、测试、打包验证完成。
- [ ] 未提交 git。
