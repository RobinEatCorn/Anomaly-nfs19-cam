<div align="center">

# NFS 2015 Chase Cam

**用 NFS 2015 的 Action Camera 替换 NTE 的驾驶追车视角**

![platform](https://img.shields.io/badge/platform-Windows%20x64-0078D6) ![language](https://img.shields.io/badge/C%2B%2B-20-00599C) ![abi](https://img.shields.io/badge/Anomaly%20ABI-v1-2bb5a0) ![license](https://img.shields.io/badge/license-AGPL--3.0--only-blue)

</div>

独立的第三方 Anomaly 插件：用 NFS 2015 Action Camera 的弹簧、pivot 与漂移参数替换游戏驾驶追车视角。
本仓库既是源码仓库，也是 Anomaly 的第三方插件源（根目录的 `pluginmaster.json`）。

## 安装

把下面的地址添加到 Runtime 的 **Plugins > 第三方插件 > 添加插件源**，应用后到
**Plugins > 可用** 刷新并安装：

```text
https://raw.githubusercontent.com/RobinEatCorn/Anomaly-nfs19-cam/main/pluginmaster.json
```

插件会装到 `<game>/Anomaly/plugins/Nfs2015ChaseCam`。也可以从
[Releases](https://github.com/RobinEatCorn/Anomaly-nfs19-cam/releases) 下载
`Nfs2015ChaseCam-<version>.zip`，把 ZIP **里面的内容**解压到该目录
（`manifest.json` 必须直接在目录根部，不能多套一层文件夹）。

## 实现方式

本仓库是**独立的 CMake 工程**，只消费安装好的 `AnomalySDK`，不参与 Anomaly 仓库的构建，
也不包含宿主内部头文件或对 Runtime 的修改。插件通过公开的纯 C ABI v1 服务工作：

- `anomaly.interop.signature` + `anomaly.interop.hook` 接管 CameraManager 的视点函数；
- `anomaly.nte.player` / `anomaly.nte.vehicle` 读取相机所需的快照（两者在宿主侧都是缓存拷贝或
  低频轮询；不租用 `anomaly.nte.entities` / `anomaly.nte.actors`）；
- `anomaly.core` 的 `read_memory` 读取 `ue5.GWorld` 链（X 锚点）；
- `anomaly.ue5.build`（可选）用 `profile_hash` 给硬编码的偏移表做版本门禁；
- `anomaly.config` + `anomaly.json` 持久化并解析设置（不依赖第三方 JSON 库）；
- `anomaly.ui` / `anomaly.localization` / `anomaly.input` 提供窗口、翻译和 F7 快捷键。

不扫描 GObjects，不调用 `ProcessEvent`；`ue5.GWorld` 只按 profile 的 pattern 解析一次全局槽位。

## 目录

```text
.
├── CMakeLists.txt        独立工程，find_package(AnomalySDK)
├── CMakePresets.json     MSVC x64 preset
├── manifest.json         插件包 Manifest（id / 服务 / capability）
├── pluginmaster.json     第三方插件源条目
├── mingw.cmake           MinGW-w64 交叉编译工具链文件
├── build-mingw.sh        一键交叉编译 / 校验 / 打包 / 安装（Linux、WSL）
├── plugin.cpp            ABI 入口、生命周期、视点 detour、UI
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
ctest --preset default
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
WSL 里可以用 `./build-mingw.sh --install D:/Games/NTE` 直接装到游戏目录。发布 ZIP 的流程见
下面的[发布](#发布)一节：ZIP 根目录必须直接包含 `manifest.json`。

## 使用

- `F7` 开关相机。插件只在检测到当前驾驶载具时接管视角；未驾驶时保持游戏原相机。
- 默认预设是 **NFS 2015（实机数据）**：车后约 5.10 m、高 1.49 m，look-at 高度 1.225 m，
  FOV 43.0–63.1°（垂直），速度位移在 0–50 mph 内后拉 1.3 m，roll 上限 4.7°。
- **电影化重建** 使用 `NFS2015_PRESET`：机位与视野更夸张，漂移推近/下压、侧倾与 NOS FOV 都更大，
  属于观感重建而非实机数据。
- 窗口中可调整机位距离/高度缩放、车身原点偏移、视口宽高比、速度/加速距离倍率、侧倾强度、
  刹车俯仰强度、镜头震动强度与“仅在漂移时侧倾”。
- **锚点来源** 有三档：**X 锚定**（默认，在视点 detour 里读同一时刻的 `root+280`）、
  **borrow 估计**（旧行为，用游戏相机位置反推）和**纯模型**（不锚定）。窗口状态行显示当前
  实际使用的锚点与 `fb` 回退次数；诊断里的 `X 门禁` 给出 `ok / hash / pattern / service /
  offset / off` 的原因。
- **版本保险**：启动时用 `anomaly.ue5.build` 的 `profile_hash` 对比本包验证过的 profile。
  hash 不匹配、pattern 解析失败、同链校验连续失败或服务缺失时，X 路径自动禁用并回退到 borrow，
  相机继续工作，状态行不会静默。
- 相机模型按 Game tick 运行；`Camera update rate` 显示实际 tick 频率，`View output / interp`
  显示视点 detour 的输出频率和命中渲染帧插值的比例。低于 60 Hz 时 detour 会用最近两次快照
  插值，不再对车辆基准做前向预测；同时会用原始游戏相机位姿的帧间位移把机位补到当前渲染帧，
  避免车身和机位差一帧。
- 相机基准是 Pawn 根位置（驾驶时它就是载具 actor 的根组件）。它在发布时被 `render_offset`
  减掉、又被 detour 加回来，因此不参与世界坐标，只需要稳定——固定用它换掉了原来每 tick
  一次的 `component_bounds` 反射查找（那是 tick 耗时的主要来源）。

## 冲突与限制

- 与 **Camera Tools**、**BetterPose** 的相机 Hook 互斥：CameraManager 的视点函数同一时间只能有一个
  所有者。先停用其他相机插件，或在窗口里点 **Retry hook**。
- 插件不做镜头碰撞射线，贴墙时可能穿模。
- 相机模型的原点是「根组件 bounds 底面 + 车身原点偏移」。根组件覆盖整车（含轮胎），所以它
  比车身网格底面更贴近地面；若希望相机相对车身更高，把 `Car origin offset (cm)` 调大即可
  （±150 cm）。日志 `anomaly-platform.log` 里的 `vehicle snapshot available: ...` 记录了载具
  服务是否可用及其轮询间隔。
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

`manifest.json`、`pluginmaster.json` 与 `plugin.cpp` 里的描述符必须一致，当前 ID 为
`io.github.robineatcorn.nfs2015-chase-cam`，版本 `1.0.0`。ID 一旦发布就不应再变。

1. 同步升版本：`manifest.json` 的 `version`、`pluginmaster.json` 的 `Version` 和 ZIP 文件名。
2. 构建 Release 包，跑 `ctest`、`anomaly-plugin validate` 和 `anomaly-test-host`（见上一节）。
3. 生成 ZIP。压缩的是包目录**里面的内容**，ZIP 根部必须直接有 `manifest.json` 和 `plugin.dll`：

   ```powershell
   Compress-Archive -Path .\build\package\Nfs2015ChaseCam\* `
     -DestinationPath .\dist\Nfs2015ChaseCam-1.0.0.zip -Force
   ```

   WSL 里用 `./build-mingw.sh --zip`，输出同名的 `dist/Nfs2015ChaseCam-<version>.zip`。
4. 先上传不可变的 Release 资源，确认下载链接直接返回 ZIP 而不是网页：

   ```sh
   gh release create v1.0.0 dist/Nfs2015ChaseCam-1.0.0.zip \
     --title "NFS 2015 Chase Cam 1.0.0" --notes "首个公开版本"
   ```

5. 最后更新 `pluginmaster.json` 的 `Version` 与 `DownloadLinkInstall` / `DownloadLinkUpdate`，
   提交到 `main`。先传 ZIP 再改索引，避免用户看到还不存在的下载地址。

用户添加的插件源 URL 固定为 `main` 分支的 raw 地址，升级只改 `pluginmaster.json`，不需要
让用户重新添加。本地测试插件源时可以用 `file://` 索引，但公开分发必须用 HTTPS。

## 许可

AGPL-3.0-only。相机模型移植自 NFS 2015 相机分析；原始 TypeScript 参考实现与分析报告不属于
本仓库，也不随插件包分发。
