<div align="center">

# NFS 2015 Chase Cam

**用 NFS 2015 的 Action Camera 替换 NTE 的驾驶追车视角**

![platform](https://img.shields.io/badge/platform-Windows%20x64-0078D6) ![language](https://img.shields.io/badge/language-C%2B%2B-20-00599C) ![abi](https://img.shields.io/badge/Anomaly%20ABI-v1-2bb5a0) ![license](https://img.shields.io/badge/license-AGPL--3.0--only-blue)

</div>

独立的第三方 Anomaly 插件：用 NFS 2015 Action Camera 的弹簧、pivot 与漂移参数替换游戏驾驶追车视角。
本仓库既是源码仓库，也是 Anomaly 的第三方插件源（根目录的 `pluginmaster.json`）。

## 安装

把下面的地址添加到 Runtime 的 **Plugins > 第三方插件 > 添加插件源**，应用后到
**Plugins > 可用** 刷新并安装：

```text
https://raw.githubusercontent.com/RobinEatCorn/Anomaly-nfs19-cam/main/pluginmaster.json
```

插件在线安装目录为 `<game>/Anomaly/plugins/robineatcorn.nfs2015-chase-cam`。也可以从
[Releases](https://github.com/RobinEatCorn/Anomaly-nfs19-cam/releases) 下载
`Nfs2015ChaseCam-<version>.zip`，把 ZIP **里面的内容**解压到该目录
（`manifest.json` 必须直接在目录根部，不能多套一层文件夹）。

## Anomaly 2.7.0 兼容性

- **仅支持 Anomaly 2.7.0+**。1.1.1 及更早版本使用旧插件 ID，在 2.7.0 下无法加载。
- 2.7.0 删除了 `anomaly.nte.vehicle` 服务和 `nte-vehicle` capability；本版本已同步删除。
  旧版服务里的带符号车速通过 `HTVehicleMovementComponent:GetForwardSpeedKmH` 反射调用恢复：
  轮式载具的 `GetForwardSpeedKmH` 可用时，倒车、刹车俯仰和纵向加速度都使用这个精确值；
  不可用时才退回 `anomaly.nte.player` 的位置微分，并使用更大的死区和更强的平滑抑制噪声。
  函数/movement component 缓存按 `ue5.objects.generation` 失效；先用
  `HTWheeledVehicleBase` 类链排除水上载具，解析失败按 2 秒退避重试。
- 载具存在性恢复为旧版 `anomaly.nte.vehicle` 内部使用的同一组反射 getter：
  `BP_GetCurrentDriveVehicle` / `BP_GetCurrentDirvingVehicle` /
  `GetBoardWaterVehicle`。2.7.0 只是删掉了服务，游戏函数仍在；插件通过
  `anomaly.ue5.objects.find_exact` + `anomaly.ue5.process-event.invoke` 以 20 Hz
  轮询它们。getter 成功返回空表示确实不在载具上，此时不会再被冲刺速度误触发。
  玩家胶囊 `AttachParent` 仍用于车辆锚点，并在 getter 服务不可用时作为存在性后备。
  诊断面板会显示“载具存在”“速度来源”“速度符号”和“倒车”。
- profile 门禁接受两个已验证的 2.7.0 hash：安装包 CRLF 版本
  `c6d97f...1f52` 和仓库 LF 版本 `c37a7b...db2a`。未知 hash 仍然 fail-closed；
  如果面板显示 `hash mismatch`，请把日志里的 `got ...` 发到 issue。
- route B 只在全屏 view 上应用，并且只在 `PrepareViewStateForVisibility` 之前写一次；
  面板的 `w N` 是历史 advisory witness 计数，早段路径不会因它跳过帧。route B 必须确定性
  应用，否则会重新引入车辆抖动；读取撕裂时用 `held` 短暂保持上一帧机位，而不是晚段补写。
- 旧 ID 的启用状态、设置目录和窗口位置不会自动迁移；请按下面的手动步骤复制。
  完整变更列表见 [RELEASE-NOTES-2.0.0.md](RELEASE-NOTES-2.0.0.md)。

### 从 1.1.1 升级

旧 ID 的数据不会自动跟随，需要手动复制。操作前先备份 `Anomaly/config` 和
`Anomaly/state`：

1. 在 **Plugins > 已安装** 中停用旧版，退出游戏。
2. 删除或改名旧安装目录（在线安装为 `Anomaly/plugins/io.github.robineatcorn.nfs2015-chase-cam`，
   手动安装通常为 `Anomaly/plugins/Nfs2015ChaseCam`），避免同时存在两个同名描述符。
3. 把旧设置复制到新 ID 目录：
   `Anomaly/config/plugins/io.github.robineatcorn.nfs2015-chase-cam/`
   → `Anomaly/config/plugins/robineatcorn.nfs2015-chase-cam/`；
   文件名保持不变（`config-nfs2015-chase-cam-settings-v1.json`）。
4. 需要保留窗口位置/收藏时，编辑 `Anomaly/state/ui-window-state.json`，把
   `pluginWindows` 和 `favorites` 里的旧 ID 替换为新 ID；不要删除整个文件。
5. 在 `Anomaly/config/plugin-enablement.json` 中把旧 ID 的启用值复制给新 ID，
   或首次启动后在插件列表里重新勾选。旧设置和旧安装目录不会被自动删除。

## 实现方式

本仓库是**独立的 CMake 工程**，只消费安装好的 `AnomalySDK`，不参与 Anomaly 仓库的构建，
也不包含宿主内部头文件或对 Runtime 的修改。插件通过公开的纯 C ABI v1 服务工作：

- `anomaly.interop.signature` + `anomaly.interop.hook` 解析并 hook 相机/渲染函数；
- `anomaly.nte.player` 读取玩家位置/朝向、游戏相机快照和 Pawn/Controller 对象句柄；
  车速优先由 `GetForwardSpeedKmH` 精确读取，失败时才由位置微分推导；
- `anomaly.ue5.objects` + `anomaly.ue5.process-event`（可选）调用旧版载具 getter，
  判断当前是否在载具上，并调用 `GetForwardSpeedKmH` 读取带符号车速；
- `anomaly.core` 的 `read_memory` 读取 `ue5.GWorld` 链和车辆/相机对象；
- `anomaly.ue5.build`（可选）用 `profile_hash` 给 route B / FOV 的硬编码偏移做版本门禁；
- `anomaly.config` + `anomaly.json` 持久化并解析设置（不依赖第三方 JSON 库）；
- `anomaly.ui` / `anomaly.localization` / `anomaly.input` 提供窗口、翻译和 F7 快捷键；
- `anomaly.window`（可选）使用 2.6+ 的托管窗口；没有时退回直接 UI。

### 相机与渲染路径

相机模型由 `chase_cam.hpp` 推进；Game tick 只发布车辆样本，最终的追车视图在渲染线程应用：

- `FPrimitiveSceneProxy::OnTransformChanged`：只发布这个渲染帧真正用于画车的车辆变换；
  2.7 的 pattern 目前无法安全解析，插件会 fail-closed 不安装这个 hook，route B 在
  pre-visibility 直接读车辆 proxy；
- `FSceneRenderer::PrepareViewStateForVisibility`：**唯一**的机位写入点，在视锥裁剪和 TAA
  历史推进之前写入最终机位；
- `FSceneView::UpdateViewMatrix` / `SetupViewFrustum`：由上面这个写入点调用，重建视图矩阵、
  `ViewFrustum` 和 `CullingFrustum`。

**不要在 `OnTransformChanged` 或 `SetupUniformBufferParameters` 里再写机位。** 这两个位置
可能已经晚于 `PrepareViewStateForVisibility`：裁剪、运动矢量和 TAA 历史仍描述旧机位，而最终
画面使用新机位，表现就是行驶中偶发一帧“摄像头跳到别处/整屏半透明重影”。早段写入失败时，
route B 最多短暂保持上一帧机位（`held`），不会退回晚段写入。

route B 的最终机位由 NFS 模型和渲染线程车辆变换合成：

```text
fresh car = 渲染线程 proxy 平移 + 车身锚点高度
eye       = fresh car + (NFS 模型 eye - NFS 模型 car) * cameraScale
eye.z    += originOffsetCm
rotation  = NFS 模型的 pitch / yaw / roll
```

模型 pose 暂时不可用时回退到固定的 route B 机架；再不可用时短暂保持上一帧的 route B 机位，
仍不可用才保留游戏原相机。这样相机和车体共用渲染帧，不再依赖 game-tick 位置去追渲染帧中的车。

不扫描 GObjects；`ue5.GWorld` 只在 profile pattern 命中后解析一次全局槽位。载具 getter 是
通过 Host 的 `ue5.objects.find_exact` 和 `ue5.process-event.invoke` 调用的，不自行定位
`ProcessEvent` 入口，也不每帧调用（20 Hz 轮询并缓存）。

## 目录

```text
.
├── CMakeLists.txt        独立工程，find_package(AnomalySDK)
├── CMakePresets.json     MSVC x64 preset
├── manifest.json         插件包 Manifest（id / 服务 / capability）
├── pluginmaster.json     第三方插件源条目
├── RELEASE-NOTES-2.0.2.md 弹簧滑块与默认看向值说明
├── RELEASE-NOTES-2.0.1.md 闪屏/重影修复说明
├── RELEASE-NOTES-2.0.0.md 2.7.0 适配与迁移说明
├── mingw.cmake           MinGW-w64 交叉编译工具链文件
├── build-mingw.sh        一键交叉编译 / 校验 / 打包 / 安装（Linux、WSL）
├── plugin.cpp            ABI 入口、生命周期、hooks、相机/渲染接线、UI
├── chase_cam.hpp         NFS 2015 相机模型（两个预设）
├── view_blend.hpp        渲染帧插值
├── localization.hpp      插件自己的本地化 helper（不依赖宿主内部头文件）
├── locales/zh-CN.json    中文文案
├── tests/                相机模型的验收夹具，不依赖游戏
├── LICENSE               AGPL-3.0-only
└── README.md
```

## 构建

只支持 Windows x64 与 Anomaly Plugin API v1。先安装 Anomaly SDK 组件：

```powershell
cmake --install C:\path\to\anomaly-build --prefix C:\AnomalySDK --component SDK

cmake --preset default -DAnomalySDK_DIR=C:/AnomalySDK/lib/cmake/AnomalySDK
cmake --build --preset default
```

等价的手工命令是 `cmake -S . -B build -DAnomalySDK_DIR=...` 加
`cmake --build build --config RelWithDebInfo`。

可加载的包默认写到 `build\package\robineatcorn.nfs2015-chase-cam`，其中包含 `plugin.dll`、`manifest.json`
和 `locales\`。导入库与 PDB 分别写到 `build\lib` 和 `build\symbols`，不会混进包目录。

### 用 MinGW-w64 构建（可选）

插件只跨纯 C ABI，SDK 包里的 `Anomaly::sdk` 也只是 header-only 的 INTERFACE target，所以插件本身
可以用 MinGW-w64 构建；Anomaly Runtime 和官方工具仍然需要 MSVC/VS2022。

在 Windows 上用 MinGW 直接构建：

```powershell
cmake -S . -B build.mingw -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=RelWithDebInfo `
  -DAnomalySDK_DIR=C:/AnomalySDK/lib/cmake/AnomalySDK
cmake --build build.mingw
```

在 Linux/WSL 上交叉编译用仓库里的 `build-mingw.sh`。脚本会定位 SDK、用 `mingw.cmake`
配置并编译，最后校验 `plugin.dll` 只导入系统模块、导出 `AnomalyPluginEntryV1`：

```sh
./build-mingw.sh                        # -> build.mingw/package/robineatcorn.nfs2015-chase-cam
./build-mingw.sh --zip                  # 另出 dist/Nfs2015ChaseCam-<version>.zip
./build-mingw.sh --install D:/Games/NTE # 装到 <game>/Anomaly/plugins/robineatcorn.nfs2015-chase-cam
./build-mingw.sh --run-tests            # 跑验收夹具；WSL2 直接运行，纯 Linux 用 wine
./verify-release.sh --allow-missing-host-tools  # 发布前检查包/ZIP/清单
./build-mingw.sh --help                 # 全部选项
```

需要 `x86_64-w64-mingw32-g++-posix` 和 Python 3（发布检查及无 `zip` 命令时的
ZIP 打包）：GCC 的 win32 线程模型里 libstdc++ 没有 `std::mutex`。SDK 默认取 `../Anomaly/.build/windows-vs2022/package/sdk`，可以用 `--sdk`
或 `ANOMALY_SDK_DIR` 覆盖。等价的手工步骤：

```sh
cmake -S . -B build.mingw -DCMAKE_TOOLCHAIN_FILE=mingw.cmake \
  -DAnomalySDK_DIR=/path/to/AnomalySDK/lib/cmake/AnomalySDK -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build.mingw
```

`CMakeLists.txt` 在 MinGW 下会自动加 `-static -static-libgcc -static-libstdc++`，让
`plugin.dll` 只导入 `KERNEL32.dll` 和 `msvcrt.dll`（两者都是 Windows 系统模块）。否则
`libstdc++-6.dll` / `libgcc_s_seh-1.dll` / `libwinpthread-1.dll` 会被插件加载器当作包内私有依赖：
必须随包分发，还会参与“同名模块已加载”冲突检查，反而更容易导致插件被拒绝。

MinGW 产物没有经过 Anomaly 上游 CI 验证；`anomaly-plugin validate` 和 `anomaly-test-host` 是
MSVC 构建的官方工具，但它们通过 `LoadLibrary` 加载纯 C ABI 的 DLL，可以用于验证 MinGW 包。

## 验证与安装

```powershell
ctest --preset default
anomaly-plugin validate .\build\package\robineatcorn.nfs2015-chase-cam
anomaly-test-host --plugin .\build\package\robineatcorn.nfs2015-chase-cam --reload 10 --ticks 60
```

离线 TestHost 没有游戏适配器，NTE 服务和 build 门禁会保持不可用；它只验证包能正常加载、卸载、
重载，以及门禁关闭时不会留下 hook lease。

WSL2 里可以用 Linux `ctest` 直接执行交叉编译出的 Windows 夹具。`anomaly-plugin` 和
`anomaly-test-host` 是 Windows 程序，包目录参数要先转成 Windows 路径：

```sh
ctest --test-dir build.mingw --output-on-failure
pkg="$(wslpath -w build.mingw/package/robineatcorn.nfs2015-chase-cam)"
anomaly-plugin validate "$pkg"
anomaly-test-host --plugin "$pkg" --reload 10 --ticks 60
```

手动安装时，把 `build\package\robineatcorn.nfs2015-chase-cam` 整个目录复制到
`Anomaly\plugins\robineatcorn.nfs2015-chase-cam`，或者在游戏目录前缀上直接安装：

```powershell
cmake --install build --config RelWithDebInfo --prefix C:\path\to\game --component GameRuntime
```

两种方式都会得到 `<game>\Anomaly\plugins\robineatcorn.nfs2015-chase-cam`，然后在游戏内
**Plugins > 已安装** 里启用。
WSL 里可以用 `./build-mingw.sh --install D:/Games/NTE` 直接装到游戏目录。发布 ZIP 的流程见
下面的[发布](#发布)一节：ZIP 根目录必须直接包含 `manifest.json`。发布前可以运行
`./verify-release.sh --allow-missing-host-tools` 检查 manifest、包内容、ZIP 根目录和 DLL
导入/导出；完整宿主工具与实机矩阵见 [RELEASE-CHECKLIST-2.0.0.md](RELEASE-CHECKLIST-2.0.0.md)。

## 使用

- `F7` 开关相机。插件只在检测到当前驾驶载具时接管视角；未驾驶时保持游戏原相机。
- 默认预设是 **NFS 2015（实机数据）**：车后约 5.10 m、高 1.49 m，look-at 高度 1.225 m，
  FOV 43.0–63.1°（垂直；16:9 下约 70–95° 水平），速度位移在 0–50 mph 内后拉 1.3 m，
  roll 上限 4.7°。
- **电影化重建** 使用 `NFS2015_PRESET`：机位与视野更夸张，漂移推近/下压、侧倾与 NOS FOV 都更大，
  属于观感重建而非实机数据。
- `Camera distance / height scale` 缩放 NFS 模型相对车的机位偏移；`Car origin offset (cm)`
  是最终的垂直微调。route B 以车辆 proxy 的地面 pivot 为锚，所以默认值是 `-70 cm`，用来抵消
  旧 +70 cm 车身中心高度；已有配置会保留用户自己的值，不会被覆盖。
- 窗口中还可调整视口宽高比、速度/加速距离倍率、侧倾强度、刹车俯仰强度、镜头震动强度与
  “仅在漂移时侧倾”。
- **坡度跟随** 默认开启：机位随渲染线程车辆 proxy 的车身俯仰一起抬降，注视点仍保持车头水平方向，
  因此上下坡时相机到坡面的高度基本稳定、地平线不左右歪。NFS 的 `GradientAdjustments` 只做
  ±10° 内的次级微调：上坡 look/rest = `0.5 / 2.0`，下坡 = `0.3 / 0.25`。
- 窗口中的 **坡度跟随（车身俯仰）** 是总开关，**坡度构图强度** 控制次级微调的倍率（0–2；
  0 只保留基础车身跟随）。总开关关闭后回到旧的 yaw-only 机架；`position and yaw only`
  诊断开关也会强制关闭坡度。

### 坡度跟随设置

坡度参数写在插件设置文件里；窗口只暴露总开关和构图强度。旧设置文件缺少这些字段时会使用
下面的默认值，不需要重置：

| 字段 | 默认 | 范围 | 含义 |
|---|---:|---:|---|
| `slopeFollow` | `true` | 开/关 | 车身坐标架与 gradient 的总开关 |
| `slopeStrength` | `1.0` | 0–2 | gradient 微调的倍率；0 只保留基础车身跟随 |
| `slopeMaxGradientDegrees` | `10.0` | 0–30° | 只限制 gradient 微调，不限制基础车身跟随 |
| `slopeUpLookAt` | `0.5` | 0–2 | 上坡注视俯仰系数（每弧度坡度） |
| `slopeUpRest` | `2.0` | 0–4 | 上坡机位高度系数（米/弧度坡度） |
| `slopeDownLookAt` | `0.3` | 0–2 | 下坡注视俯仰系数 |
| `slopeDownRest` | `0.25` | 0–4 | 下坡机位高度系数 |

`slopeStrength` 和四个 gradient scalar 只影响次级微调；即使 `slopeMaxGradientDegrees = 0`
或 `slopeStrength = 0`，相机仍会跟随完整车身俯仰。坡度几何的离线验收值、构建/TestHost
结果和实机确认记录见 [REPORT-slope-gradient.md](REPORT-slope-gradient.md)。

### 锚点来源

驾驶时车辆位置有四路来源，面板的 `Anchor` 行会显示本帧实际使用的来源：

| 选项 | 来源 | 用途 |
|---|---|---|
| **X anchored** | 视点 detour 当刻读取的玩家 `root+280` | 轻量、稳定的车 root 位置 |
| **Vehicle mesh** | 角色胶囊 `AttachParent` 的 `VehicleMesh`；优先 SceneProxy 的 `LocalToWorld`，退回 `ComponentToWorld` | 最贴近渲染器画车用的车身变换；受版本门禁保护；也是默认值 |
| **Borrow estimate** | 用游戏相机位置反推车身（低通估计） | 兼容旧行为；FOV/motion 仍来自模型 |
| **Pure model** | 只使用 Game-tick 的车辆样本 | 完全不让相机锁到游戏相机；用于诊断 |

route B 的最终位置并不直接等于这里选中的锚点：锚点用于模型/presenter 的基准和回退；
渲染线程的车辆 proxy 变换是最终位置的基准。`fb` 是锚点回退次数，不是车辆抖动计数。

### NFS 镜头（FOV）

`Apply NFS lens (FOV)` 打开时，route B 会把 NFS 模型算出的水平 FOV 写入当前 `FViewInfo`，并用
`FViewMatrices` 构造函数重建整组矩阵（包括 `InvProjectionMatrix`、`ProjectionNoAAMatrix` 和
`ScreenToClipMatrix`）；面板 `FOV` 显示实际水平 FOV。关闭后保留游戏原 FOV。

FOV 写入和 route B 一样受版本门禁保护；模型 lens 不可用或值超出 1–179° 时跳过，不会写入。

### 相机模型时间

Game tick 只写车样本；相机模型在视点 detour 中推进并发布 pose，route B 在渲染线程把模型 pose
和这个渲染帧的车辆变换合成。渲染路径无分配、无自有锁、无日志；只用栈缓冲、原子快照和宿主
hook lease。

## 状态行与诊断

主窗口常用状态：

- `Camera` / `View-point hook`：相机是否启用、基础视点 hook 是否成功；
- `Anchor` + `版本门禁`：当前锚点来源、
  `ok / hash / pattern / service / offset / off / read` 门禁结果，以及回退次数；
- `水平 FOV`、`镜头-车辆距离（输出 / 原始）`、`速度`、`侧滑角`、`漂移量`；
- `渲染钩`、`视图钩 R/G/线程`、`路线B 预应用/保持/回退/校验`：route B 各阶段的调用和
  回退计数；`held` 是读取撕裂时短暂保持上一帧机位的次数，`w N` 是 advisory witness 计数；
- `渲染龄` / `视图←变换龄`：route B 使用的车辆变换和视图之间差多少毫秒。

诊断面板里的 `渲染 vs 代理`、`X 相位差`、`相机相对车速度/转向`、`tick 间隔`、`视野` 和
`C px` 是定位渲染相位、帧长和锚点问题的常驻探针。它们只读取计数和样本，不参与相机计算；
排查结束后可以忽略，不需要为了正常使用关闭。

诊断开关（`Diagnostic: ...`）会刻意关闭某些通道，仅用于二分问题：

- `position and yaw only`：关闭 pitch/roll/镜头写入；
- `zero lag, newest tick position`：绕过插值窗口；
- `keep the Game's position` / `rotation`：保留游戏对应通道；
- `Lock to the Game's camera (0 = pure model)`：只影响 borrow 分支的强度。

## 版本保险与安全边界

插件的 route B 会直接写 UE 的 `FViewInfo` / `FSceneRenderer` 字段，因此不能对任意游戏版本
无条件启用。当前策略是 **fail-closed**：

- 启动时读取 `anomaly.ue5.build` 的 `profile_hash`，只有命中 `kAcceptedProfileHashes`
  里的已验证 hash 才打开 `version_gate`；
- 2.6+ 且 build service 存在时，门禁关闭就不创建 route B / `FViewInfo` FOV / 视点 hook，
  也不写任何游戏内存；面板显示 `版本门禁`，相机保持游戏原样；
- 所有需要的函数入口仍通过运行时 pattern 解析；早段 pattern 不命中会先按 2 秒窗口重试
  5 次（避免首帧 signature 服务未就绪造成永久降级），仍失败才锁存并关闭 route B；
  晚段诊断 pattern 不命中不影响 route B；
- 渲染路径再次检查 render thread、全屏 view、view state、generation/新鲜度、有限值和
  proxy 所有权；
- hook 回调使用宿主 generation lease；热重载时先撤 callback 再释放 hook。

**2.5 兼容**：2.5 没有可选的 build service，route B 和 `FViewInfo` FOV 写入关闭，只保留旧的
基础视点 hook（它仍按旧路径写 CameraManager 的镜头）；2.6+ 如果 build service 存在但 hash
不匹配，则相机完全不接管。

这不是“任何未知版本都绝不会崩”的数学保证：如果游戏/Anomaly 更新后 profile hash 没有变化、
但字段实际移动，或者 UE 对象在渲染线程被释放，原始内存访问仍可能触发 Windows access violation；
C++ `try/catch` 捕获不到这类异常。遇到新版本请先把新 hash 加入 allowlist、复验偏移表，
再发布新插件。

## 冲突与限制

- 与 **Camera Tools**、**BetterPose** 的相机 Hook 互斥：CameraManager 的视点函数同一时间只能有一个
  所有者。先停用其他相机插件，或在窗口里点 **Retry hook**。
- 插件不做镜头碰撞射线，贴墙时可能穿模。
- route B 只处理全屏游戏 view；反射、场景捕获、阴影等小 view 保持游戏原相机。
- `Camera distance / height scale` 和 `Car origin offset` 改的是相机，不改变车辆物理或游戏碰撞。
- 本次未实现车身 roll 与腾空混合：`onGround` 仍恒为 true。跳跃/腾空时相机继续按车辆 proxy
  的俯仰跟随车头，不做空中姿态稳定。
- route B 固定回退（模型 pose 短暂不可用时）只使用基础车身坐标架，不叠加
  `GradientAdjustments`；与正常模型路径可能有细微高度/视线差。
- 坡度来自渲染线程车辆 proxy 的 forward。首次加载或重置后还没有新的 proxy 变换时，会先按
  平地姿态运行，直到下一次变换发布。
- 已知遗留：重刹车/漂移时模型仍可能出现一次点头式 pitch，当前不在本版验收范围。
- 版本保险是**有意降级**：profile 不匹配时相机功能不会工作，优先保证不把游戏写崩；更新插件
  后再恢复。
- 前向运动预测已移除（它会把逐帧变化的 `velocity * dt` 叠到机位上，反而放大抖动）。

## 与 Anomaly 内建版本的关系

本包从 Anomaly `local` 分支的内建插件移植而来，但不再属于 Anomaly 源码树：

- 插件 ID、作者和版本独立，不再使用 `anomaly.builtin.*` 命名空间；
- `localization.hpp` 复制进本包，不再引用 `plugins/common/`；
- 设置读写改用宿主 `anomaly.config` + `anomaly.json`，不再链接 nlohmann/json；
- Anomaly 的 `CMakeLists.txt`、`README.md`、`docs/user-guide/built-in-plugins.md` 与
  `plugins/README.md` 均不需要修改。

`local` 分支上另有 ACE loader-hook 冻结策略和全局 `/utf-8` 编译选项两个提交；它们不属于相机特性，
也没有移植到这里。`/utf-8` 只在本工程的 target 上设置，ACE 行为由 Runtime 自己决定。

## 待触发

以下项目只有在未来出现明确需求或新证据时才做，不属于本版：

- 把速度、车头朝向、yaw rate 从 Game-tick 差分迁移到渲染帧差分（当前 X/route B 只解决位置）；
- 刹车重减速的点头 pitch 进行单独的模型/信号修正；
- 如果游戏改变 `FViewInfo` / `FSceneRenderer` 布局，更新 `kAcceptedProfileHashes`、偏移表和
  pattern，而不是放宽门禁；
- 如 Anomaly 改变 hook 服务 ABI，升级时重新验证 `anomaly.interop.hook` 与 lease 行为。

## 发布

`manifest.json`、`pluginmaster.json` 与 `plugin.cpp` 里的描述符必须一致，当前 ID 为
`robineatcorn.nfs2015-chase-cam`，当前版本 `2.0.2`。ID 一旦发布就不应再变。

发布门禁、离线/实机测试矩阵、产物哈希和回滚步骤记录在
[RELEASE-CHECKLIST-2.0.0.md](RELEASE-CHECKLIST-2.0.0.md)。完整发布前至少完成两个不同地图、
两次完整重启的实机验收；当前代码和离线包已准备好，Windows 宿主工具与剩余实机项仍需补跑。

1. 同步升版本：`manifest.json` 的 `version`、`pluginmaster.json` 的 `Version` 和 ZIP 文件名。
2. 构建 Release 包，跑 `ctest`、`anomaly-plugin validate` 和 `anomaly-test-host`（见上一节）。
3. 生成 ZIP。压缩的是包目录**里面的内容**，ZIP 根部必须直接有 `manifest.json` 和 `plugin.dll`：

   ```powershell
   Compress-Archive -Path .\build\package\robineatcorn.nfs2015-chase-cam\* `
     -DestinationPath .\dist\Nfs2015ChaseCam-2.0.2.zip -Force
   ```

   WSL 里用 `./build-mingw.sh --zip`，输出同名的 `dist/Nfs2015ChaseCam-<version>.zip`；
没有 `zip` 命令时脚本会自动使用 Python `zipfile`。发布前运行
`./verify-release.sh --allow-missing-host-tools` 检查 ZIP 根目录和包内容。
4. 先上传不可变的 Release 资源，确认下载链接直接返回 ZIP 而不是网页：

   ```sh
   gh release create v2.0.2 dist/Nfs2015ChaseCam-2.0.2.zip \
     --title "NFS 2015 Chase Cam 2.0.2" \
     --notes "新增镜头弹簧滑块；漂移看向弹簧默认值改为 15"
   ```

5. 最后更新 `pluginmaster.json` 的 `Version` 与 `DownloadLinkInstall` / `DownloadLinkUpdate`，
   提交到 `main`。先传 ZIP 再改索引，避免用户看到还不存在的下载地址。

用户添加的插件源 URL 固定为 `main` 分支的 raw 地址，升级只改 `pluginmaster.json`，不需要
让用户重新添加。本地测试插件源时可以用 `file://` 索引，但公开分发必须用 HTTPS。

## 许可

AGPL-3.0-only。相机模型移植自 NFS 2015 相机分析；原始 TypeScript 参考实现与分析报告不属于
本仓库，也不随插件包分发。
