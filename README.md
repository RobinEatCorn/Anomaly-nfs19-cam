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
  mingw.cmake           MinGW-w64 交叉编译工具链文件
  build-mingw.sh        一键交叉编译 / 校验 / 打包 / 安装（Linux、WSL）
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
./build-mingw.sh                        # -> build.mingw/package/Nfs2015ChaseCam
./build-mingw.sh --zip                  # 另出 dist/Nfs2015ChaseCam-<version>.zip
./build-mingw.sh --install D:/Games/NTE # 装到 <game>/Anomaly/plugins/Nfs2015ChaseCam
./build-mingw.sh --run-tests            # 用 wine 跑验收夹具
./build-mingw.sh --help                 # 全部选项
```

需要 `x86_64-w64-mingw32-g++-posix`：GCC 的 win32 线程模型里 libstdc++ 没有
`std::mutex`。SDK 默认取 `../Anomaly/.build/windows-vs2022/package/sdk`，可以用 `--sdk`
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
  插值，不再对车辆基准做前向预测；同时会用原始游戏相机位姿的帧间位移把机位补到当前渲染帧，
  避免车身和机位差一帧。
- 车辆基准优先读 `VehicleMesh` 的 bounds（实际渲染的车身），再回退到 `RootComponent`，最后
  才是 Pawn root。正常驾驶时窗口里的 `Camera base` 应显示 **VehicleMesh**；如果显示
  **Pawn root**，说明 vehicle entity 或 `component_bounds` 不可用，相机跟的是角色而不是车身。

## 冲突与限制

- 与 **Camera Tools**、**BetterPose** 的相机 Hook 互斥：CameraManager 的视点函数同一时间只能有一个
  所有者。先停用其他相机插件，或在窗口里点 **Retry hook**。
- 插件不做镜头碰撞射线，贴墙时可能穿模。
- 如果 `Camera base` 显示 **Pawn root**，相机会跟玩家角色而不是渲染车身，高速时车身可能在
  画面里前后滑动。先确认 Runtime 发布了 `anomaly.nte.vehicle` 和 `anomaly.nte.entities`；
  插件优先用 vehicle service 的 entity id 读取 `VehicleMesh`，找不到时才回退。日志
  `anomaly-platform.log` 里的 `camera base source=...` 可用于定位。
- 前向运动预测已移除（它会把逐帧变化的 `velocity * dt` 叠到机位上，反而放大抖动）；detour
  仍用原始游戏相机位姿的帧间位移补偿发布延迟，这是测得位移而不是速度外推。
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
