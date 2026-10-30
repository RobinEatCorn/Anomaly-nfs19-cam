# NFS 2015 Chase Cam

独立的第三方 Anomaly 插件：用 NFS 2015 Action Camera 的弹簧、pivot 与漂移参数替换游戏驾驶追车视角。

本目录是一个**独立的 CMake 工程**，只消费安装好的 `AnomalySDK`，不参与 Anomaly 仓库的构建，
也不包含宿主内部头文件或对 Runtime 的修改。插件通过公开的纯 C ABI v1 服务工作：

- `anomaly.interop.signature` + `anomaly.interop.hook` 接管 CameraManager 的视点函数；
- `anomaly.nte.player` / `anomaly.nte.vehicle` / `anomaly.nte.entities` / `anomaly.nte.actors` 读取相机所需的快照；
- `anomaly.config` + `anomaly.json` 持久化并解析设置（不依赖第三方 JSON 库）；
- `anomaly.ui` / `anomaly.localization` / `anomaly.input` 提供窗口、翻译和 F7 快捷键。

不扫描 GObjects，不调用 `ProcessEvent`。

## 目录

```text
nfs2015-chase-cam/
  CMakeLists.txt        独立工程，find_package(AnomalySDK)
  manifest.json         插件包 Manifest（id / 服务 / capability）
  plugin.cpp            ABI 入口、生命周期、视点 detour、UI
  chase_cam.hpp         NFS 2015 相机模型（两个预设）
  view_blend.hpp        渲染帧插值
  localization.hpp      插件自己的本地化 helper（不依赖宿主内部头文件）
  locales/zh-CN.json    中文文案
  tests/               相机模型的验收夹具，不依赖游戏
```

## 构建

只支持 Windows x64 与 Anomaly Plugin API v1。先安装 Anomaly SDK 组件：

```powershell
cmake --install C:\path\to\anomaly-build --prefix C:\AnomalySDK --component SDK

cmake -S . -B build -DAnomalySDK_DIR=C:/AnomalySDK/lib/cmake/AnomalySDK
cmake --build build --config RelWithDebInfo
```

可加载的包默认写到 `build\package\Nfs2015ChaseCam`，其中包含 `plugin.dll`、`manifest.json`
和 `locales\`。导入库与 PDB 分别写到 `build\lib` 和 `build\symbols`，不会混进包目录。

### 用 MinGW-w64 构建（可选）

插件只跨纯 C ABI，SDK 包里的 `Anomaly::sdk` 也只是 header-only 的 INTERFACE target，所以插件本身
可以用 MinGW-w64 构建；Anomaly Runtime 和官方工具仍然需要 MSVC/VS2022。

在 Windows 上：

```powershell
cmake -S . -B build -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=RelWithDebInfo `
  -DAnomalySDK_DIR=C:/AnomalySDK/lib/cmake/AnomalySDK
cmake --build build
```

在 Linux/WSL 上交叉编译时，用一个 toolchain 文件：

```cmake
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
```

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=mingw-toolchain.cmake \
  -DAnomalySDK_DIR=/path/to/AnomalySDK/lib/cmake/AnomalySDK -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

`CMakeLists.txt` 在 MinGW 下会自动加 `-static -static-libgcc -static-libstdc++`，让
`plugin.dll` 只导入 `KERNEL32.dll` 和 `msvcrt.dll`（两者都是 Windows 系统模块）。否则
`libstdc++-6.dll` / `libgcc_s_seh-1.dll` / `libwinpthread-1.dll` 会被插件加载器当作包内私有依赖：
必须随包分发，还会参与“同名模块已加载”冲突检查，反而更容易导致插件被拒绝。

MinGW 产物没有经过 Anomaly 上游 CI 验证；`anomaly-plugin validate` 和 `anomaly-test-host` 是
MSVC 构建的官方工具，但它们通过 `LoadLibrary` 加载纯 C ABI 的 DLL，可以用于验证 MinGW 包。

## 验证与安装

```powershell
ctest --test-dir build -C RelWithDebInfo --output-on-failure
anomaly-plugin validate .\build\package\Nfs2015ChaseCam
anomaly-test-host --plugin .\build\package\Nfs2015ChaseCam --reload 10 --ticks 60
```

离线 TestHost 没有游戏适配器，NTE 服务会保持 `UNAVAILABLE`；它只验证包能正常加载、卸载和重载。

手动安装时，把 `build\package\Nfs2015ChaseCam` 整个目录复制到 `Anomaly\plugins\Nfs2015ChaseCam`，
或者在游戏目录前缀上直接安装：

```powershell
cmake --install build --config RelWithDebInfo --prefix C:\path\to\game --component GameRuntime
```

两种方式都会得到 `<game>\Anomaly\plugins\Nfs2015ChaseCam`，然后在游戏内 **Plugins > 已安装** 里启用。
打包成第三方插件 ZIP 的流程见 Anomaly 文档 `docs/developer-guide/plugin-distribution.md`：
ZIP 根目录必须直接包含 `manifest.json`。

## 使用

- `F7` 开关相机。插件只在检测到当前驾驶载具时接管视角；未驾驶时保持游戏原相机。
- 默认预设是 **NFS 2015（实机数据）**：车后约 5.10 m、高 1.49 m，look-at 高度 1.225 m，
  FOV 43.0–63.1°（垂直），速度位移在 0–50 mph 内后拉 1.3 m，roll 上限 4.7°。
- **电影化重建** 使用 `NFS2015_PRESET`：机位与视野更夸张，漂移推近/下压、侧倾与 NOS FOV 都更大，
  属于观感重建而非实机数据。
- 窗口中可调整机位距离/高度缩放、车身原点偏移、视口宽高比、速度/加速距离倍率、侧倾强度、
  刹车俯仰强度、镜头震动强度与“仅在漂移时侧倾”。
- 相机模型按 Game tick 运行；`Camera update rate` 显示实际 tick 频率，`View output / interp`
  显示视点 detour 的输出频率和命中渲染帧插值的比例。低于 60 Hz 时 detour 会用最近两次快照
  插值，并对车辆基准做有限前向预测。

## 冲突与限制

- 与 **Camera Tools**、**BetterPose** 的相机 Hook 互斥：CameraManager 的视点函数同一时间只能有一个
  所有者。先停用其他相机插件，或在窗口里点 **Retry hook**。
- 插件不做镜头碰撞射线，贴墙时可能穿模。
- 已知问题：相机更新频率低于 60 Hz 时，车身在画面里仍可能有轻微抖动；当前的预测/插值是对这个
  问题的持续处理，仍待进一步逆向确认。
- 插件只读取公开快照，不修改游戏内存中的相机状态（`FOV` 复选框用于把 NFS 镜头写回 CameraManager
  的 `FMinimalViewInfo`，与内建相机插件的行为一致）。

## 与 Anomaly 内建版本的关系

本包从 Anomaly `local` 分支的内建插件移植而来，但不再属于 Anomaly 源码树：

- 插件 ID、作者和版本独立，不再使用 `anomaly.builtin.*` 命名空间；
- `localization.hpp` 复制进本包，不再引用 `plugins/common/`；
- 设置读写改用宿主 `anomaly.config` + `anomaly.json`，不再链接 nlohmann/json；
- Anomaly 的 `CMakeLists.txt`、`README.md`、`docs/user-guide/built-in-plugins.md` 与
  `plugins/README.md` 均不需要修改。

`local` 分支上另有 ACE loader-hook 冻结策略和全局 `/utf-8` 编译选项两个提交；它们不属于相机特性，
也没有移植到这里。`/utf-8` 只在本工程的 target 上设置，ACE 行为由 Runtime 自己决定。

## 发布

`manifest.json` 与 `plugin.cpp` 中的插件 ID 必须一致，当前为
`io.github.robineatcorn.nfs2015-chase-cam`。发布到自己的插件源时，修改 `id`、`author` 和
`pluginmaster.json` 的 `InternalName` / `Version` 即可，但 ID 一旦发布就不应再变。

## 许可

AGPL-3.0-only。相机模型与文档来自同一仓库的 NFS 2015 相机分析；原始 TypeScript 参考实现与
分析报告不随本插件发布。
