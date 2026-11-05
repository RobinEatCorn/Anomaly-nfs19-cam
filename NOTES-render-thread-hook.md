# 渲染线程车辆变换 Hook — 工作笔记（已归档）

> **已归档（2026-11-06）**：本文件是 v1 工作笔记，已被 `PLAN-render-thread-hook.md` 取代。
> 当前实现、验收结果与安全门禁以 `README.md` / `PLAN-render-thread-hook.md` 为准；这里保留
> 历史来源和当时的逆向线索。

## 背景（一句话）

NTE = 《异环》(Neverness to Everness)，游戏二进制 `HTGame.exe`。
Anomaly 是 NTE 的插件框架。`nfs2015-chase-cam` 是运行在 Anomaly 里的相机 Mod。
问题：行驶中车抖（相机不抖，车抖）。

## 已确认的结论

1. 二分实验（历史会话）：只写位置→抖；只写朝向→不抖；都不写→不抖。**抖在插件写出的位置通道。**
2. 根因：我们算相机用的车位置是 game tick 采样的 `player.position`，游戏画车用的是渲染线程那一刻的变换 X。两者不同步。
3. 精确从游戏相机位置倒推 X：**数学上不可逆**（相机弹簧是低通，高频信息已丢失）。
4. Anomaly main 分支（`get_object_handles` + `invoke`）都在 game thread，拿不到渲染线程的 X。升级 main 解决不了抖。
5. 用户决定：**走 hook 路径，直接拿渲染线程的车辆变换 X**，但必须带版本保险——游戏更新后插件失效、不能崩。

## 已完成并提交的修改（nfs2015-chase-cam）

- `cf5d976` S1：模型原点从包围盒推导点改为刚性 `player.position`；渲染 pass 去重（8ms 同帧只推进一步）。
- `17d7603` 容忍乱序 pass 时间戳。
- `38c98ef` borrow 改为输入端锚定：从 `game_eye` 反推“游戏画的车位置”估计值，模型锚到该估计值。默认 `borrow_strength=1.0`。
- AtSmooth 切向量改为非均匀 Catmull-Rom（在 cf5d976 内）。

当时的 `nfs2015-chase-cam` HEAD = `38c98ef`；该节点之后由 v2 计划继续推进，最终 route B 已
完成用户验收，见 `PLAN-render-thread-hook.md` 任务组 4。

## 游戏文件

游戏路径（只读，不要修改里面内容）：
`/mnt/c/Program Files (x86)/Neverness To Everness/Client/WindowsNoEditor/HT/Binaries/Win64/`

已拷贝到分析目录（不在 git 跟踪范围内，不要提交）：
`/mnt/d/Project/ByTime/D-2026Q1/5-NTE/4-drift/nte-game-analysis/`
- `HTGame.exe` (273,224,800 bytes)
- `HTGameBase.dll` (28,455,832 bytes)

## 下一步：逆向 HTGame.exe

目标：找渲染线程里“车辆/角色渲染变换 X”的 hook 点，或证明我们已有的
`CameraViewPointDetour`（渲染线程调用）可以读到与渲染同步的 X。

### 关键教训

第一次搜 pattern 失败（`kCameraViewPointPattern` 和 `ue5.GWorld` 都 count=0）：
**不能把文件偏移直接当 VA**。PE 磁盘布局和内存 VA 不同，要先解析 PE section
headers，把 RVA/VA 映射到文件偏移再搜字节。

### 已知信息（Anomaly profile，`profiles/nte/nte-current.json`）

- `ue5.GWorld` pattern: `48 8B 1D ?? ?? ?? ?? 48 85 DB 74 ?? 41 B0 01`
- `ue5.AActorProcessEvent` pattern: `48 89 5C 24 10 48 89 6C 24 18 57 48 83 EC 20 F7 82 B0 00 00 00 00 04 00 00 49 8B E8 48 8B DA 48 8B F9 75 06 83 7A 68 00 74 4D 48 89 74 24 30 0F B6 35 ?? ?? ?? ?? E8 ?? ?? ?? ?? 48 85 C0 74 11 48 8B C8 E8 ?? ?? ?? ?? 84 C0 75 0F`
- 插件相机 hook pattern（`plugin.cpp` 的 `kCameraViewPointPattern`）:
  `48 89 5C 24 08 57 48 83 EC 20 48 8B 01 49 8B F8 48 8B DA`
- 关键布局偏移：`controller.cameraManager=896`，`cameraManager.location=880`，
  `cameraManager.rotation=904`，`cameraManager.fov=928`，`actor.rootComponent=456`，
  `sceneComponent.location=280`，`sceneComponent.boundsOrigin=280`。

### 候选路线

1. 解析 PE 节区，正确映射 VA→file offset，搜 `kCameraViewPointPattern`，确认我们 hook 的函数。
2. 用 Ghidra/IDA 类工具（或 objdump/capstone）反汇编该函数，看 `object` 参数（RCX）是不是相机管理器、能否导航到车辆 Pawn / rootComponent。
3. 在 CameraViewPointDetour 里（渲染线程）尝试用 `core->read_memory` 从 GWorld 链读到 `sceneComponent.location`，看是否与宿主快照有相位差。需要插件自己能解析 `ue5.GWorld` 和布局偏移（或硬编码当前版本 + hash 校验）。
4. 版本保险：启动时用 `AnomalySignatureServiceV1::resolve` 按 pattern 解析所有需要的符号；任一失败则禁用新路径、回退现有反推法。插件 manifest 声明可选能力，确保不会因 hook 失败而崩。

### 注意

- 不要修改游戏目录内容。
- `nte-game-analysis/` 下的大文件不要提交进 git。
- 分析产物（反汇编片段、笔记）可以放 `nfs2015-chase-cam/` 或分析目录。
