# NFS 2015 Chase Cam 2.1.1 — Anomaly 3.1.2 适配

- 日期：2026-11-26
- 游戏：NTE `HTGame.exe`（3.1.2 更新后的运行镜像），PID 13620
- 运行时：Anomaly 3.1.2（`Anomaly-3.1.2-runtime`）
- 现象：插件能加载、能读车速和输入，但两个相机钩子都没装上，游戏保持原生相机。

## 1. 一句话结论

3.1.2 换了新的 `HTGame.exe`，也换了 NTE profile 的 hash。插件的 profile 门禁是
fail-closed 的，hash 不在白名单里就直接不装相机钩子，所以这次“hook 不上”不是
pattern 失效，而是门禁先把相机路径关掉了。2.1.1 放行 3.1.2 的两个 profile hash，并
在运行中的 3.1.2 镜像上逐项复核了所有依赖的偏移和 pattern。

## 2. 现场证据

Anomaly 日志（`plugin_id=robineatcorn.nfs2015-chase-cam`）：

```text
profile gate: hash mismatch (got 7341beb0ffae6e43c017d20b51a357d30249d4516dd487d5ac91c0cd2fafdf7d,
want one of 6 accepted runtime profiles); camera hooks disabled until the plugin is updated
```

同时 `burout`/`GetInputInfo`、`GetForwardSpeedKmH` 这些不经过相机钩子的诊断一直在正常
输出，说明插件本体、Anomaly ABI 和车辆反射调用都没问题，问题只在门禁。

`anomaly-cli --pid 13620 ue` 报告的当前 profile：

| 项 | 值 |
|---|---|
| profileHash | `7341beb0ffae6e43c017d20b51a357d30249d4516dd487d5ac91c0cd2fafdf7d` |
| profileSource | `Anomaly\Anomaly-3.1.2-runtime\Anomaly\profiles\nte\nte-current.json` |
| ue5.GWorld RVA | `0x10538488`（3.1.0/3.1.1 是 `0x10528...`，新 exe） |

## 3. 为什么门禁挡住的只是 hash，不是偏移

3.1.1 与 3.1.2 的 `nte-current.json` 做了逐键 diff：3.1.2 只新增 10 个键，全部是
`cooked.vehicle.*` 的计数/schema hash 和几个 FProperty 子结构偏移，**原有 layout 值一个
都没改**：

```text
ADD /layout/cooked.vehicle.currentPropertyCount = 394
ADD /layout/cooked.vehicle.insertedProperty      = 50
ADD /layout/cooked.vehicle.schemaHashHi/Lo       = ...
ADD /layout/fbyteProperty.enum                   = 112
ADD /layout/fmapProperty.keyProp/valueProp       = 112/120
ADD /layout/foptionalProperty.valueProperty      = 112
ADD /layout/fproperty.flags                      = 56
ADD /layout/fsetProperty.elementProp             = 112
ADD /layout/uscriptStruct.flags                  = 176
```

插件实际读取的 `world.gameInstance=560`、`gameInstance.localPlayers=56`、
`localPlayer.controller=48`、`controller.pawn=776`、`actor.rootComponent=456`、
`sceneComponent.location=280` 全部保持原值。

但 exe 确实换了，所以不能只看 profile。下面每一项都在运行中的 3.1.2 镜像上复核过。
镜像通过 Anomaly 的 `read` 命令从进程内存整块导出（`HTGame.exe` 磁盘文件是加密的，
静态分析没有意义）。

## 4. 相机相关偏移与 pattern 的复核

### 4.1 两条相机 pattern 仍然唯一

`anomaly-cli scan HTGame.exe .text "<pattern>"`：

| pattern | 3.1.2 命中 | 3.1.0 地址 | 说明 |
|---|---|---|---|
| `kLocalPlayerGetViewPointPattern` | `0x1444C6950`（1 处） | `0x1444C6720` | 整体 +0x230 |
| `kCalcSceneViewInitOptionsPattern` | `0x1444B9960`（1 处） | `0x1444B9730` | 整体 +0x230 |
| `kToggleSwitchCameraViewModePattern` | `0x1480A87D0`（1 处） | — | 唯一 |
| `kGWorldPattern` | `0x144B822EC`（1 处） | — | 唯一 |

两个相机入口移动的距离完全一样，是同一段 .text 链接位移的结果，不是函数被改写。

### 4.2 `FMinimalViewInfo` 四个字段

3.1.2 `GetViewPoint`（`0x1444C6950`）反汇编：

```text
1444c6975  lea r14, [rdi + 0x30]              ; FOV 输出
1444c697c  lea r9,  [rdi + 0x18]              ; Rotation 输出
1444c69c4  movups xmm0, [rax]
1444c69c7  movups [rdi], xmm0
1444c69ca  movsd  xmm1, [rax + 0x10]
1444c69cf  movsd  [rdi + 0x10], xmm1          ; Location = 3 × double @ +0x00
1444c69d4  movups xmm0, [rax + 0x18]
1444c69d8  movups [rdi + 0x18], xmm0
1444c69dc  movsd  xmm1, [rax + 0x28]
1444c69e1  movsd  [rdi + 0x28], xmm1          ; Rotation = 3 × double @ +0x18
1444c69f0  mov    eax, [rax + 0x34]
1444c69f3  mov    [rdi + 0x34], eax           ; DesiredFOV @ +0x34
1444c6b9e  movss  [r14], xmm0                 ; FOV @ +0x30
1444c6c7e  mov    [rdi + 0x34], eax           ; 退出前 DesiredFOV = FOV
```

结论：`Location +0x00 / Rotation +0x18 / FOV +0x30 / DesiredFOV +0x34` 不变。

### 4.3 `bInCameraCut`

3.1.2 `CalcSceneViewInitOptions`（`0x1444B9960`）反汇编：

```text
1444b9ac8  mov  rax, [rsi + 0x30]
1444b9ad9  shr  eax, 0xb
1444b9adc  and  al, 1
1444b9ade  mov  byte ptr [rdi + 0x2c8], al    ; FSceneViewInitOptions::bInCameraCut
```

结论：`kSceneViewInitOptionsCameraCutOffset = 0x2C8` 不变。

### 4.4 PlayerController 视档块与 vtable 槽

3.1.2 `ToggleSwitchCameraViewMode`（`0x1480A87D0`）反汇编里 `0x1E80..0x1EB0`
全部保持同样的语义：

```text
1480a87d8  cmp dword ptr [rcx + 0x1e90], 0    ; 视档实例数
1480a880f  mov byte ptr [rbx + 0x1eb0], 0     ; grace used
1480a8816  mov qword ptr [rbx + 0x1ea8], rax  ; cooldown clock
1480a8822  lea rdx, [rbx + 0x1e80]            ; tag
1480a882e  lea rcx, [rbx + 0x1e88]            ; array
1480a8842  mov ecx, [rbx + 0x1e90]            ; count
1480a889e  movsd xmm1, [rbx + 0x1ea8]         ; clock
1480a88bf  movss xmm1, [rbx + 0x1e9c]         ; cooldown
1480a88d8  movss xmm0, [rbx + 0x1ea0]         ; grace
1480a88e9  cmp byte ptr [rbx + 0x1eb0], 0     ; grace used
```

运行中的 PlayerController 实读（GWorld → world+560 → gameInstance+56 →
localPlayer+48）：

```text
tag=0x3b1a9  array=0x1db66676c0  count=2  capacity=4
cooldown=0.5 grace=0.2 clock=-1.0 grace_used=0  limit=0  cineCamera(PC+0x1760)=0
vtable=0x14ea4c748  vtable+0x15C8 = 0x1480a87d0  (== pattern 解析结果)
```

结论：`0x1E80..0x1EB0`、`vtable+0x15C8`、`PC+0x1760` 全部不变。

### 4.5 车辆锚点链

运行中实读（玩家在车里）：

```text
pawn+0x1C8(root) → root+0xD8(AttachParent) = VehicleMesh (SkeletalMeshComponent)
parent+0x1F0 quat norm² = 1.0000000000000004
parent+0x210 translation = (-280860.49, 168996.97, 3996.01)
parent+0x118 relative location = (-280859.65, 168995.40, 4064.49)
与玩家胶囊 root 距离 = 40.6 cm
```

同时 `vehicle+0x348 == VehicleMesh`（用 `BP_GetCurrentDriveVehicle` 同一辆车验证），
`mesh+0x938` 指向 `SK_Vehicle_037_Skeleton_AnimBlueprint_C`。结论：
`kSceneComponentAttachParentOffset=0xD8`、`kSceneComponentTransformOffset=0x1F0`、
`kSceneComponentTransformTranslationOffset=0x210`、`kVehicleMeshOffset=0x348`、
`kSceneComponentAnimScriptInstanceOffset=0x938` 不变。

## 5. 改动

1. `src/constants.hpp`：`kAcceptedProfileHashes` 增加
   - 3.1.2 安装包 CRLF：`7341beb0ffae6e43c017d20b51a357d30249d4516dd487d5ac91c0cd2fafdf7d`
   - 3.1.2 仓库 LF：`228b0ac3642a682688442706e517af4963c3700a475386ee7291acde25336501`
   （LF 规则的验证：把 3.1.1 的 CRLF profile 转 LF 后，hash 正好等于已有的
   `9acaf134...5a82`。）
2. `src/constants.hpp`：前/后轮视觉组件 TArray 的偏移 0x12D8/0x12E8 →
   **0x12E0/0x12F0**。3.1.2 在 `FrontWheels` 前插入了一个 8 字节字段，属性声明在基类
   `HTWheeledVehicleDrivable` 上，因此所有轮式载具一起后移。
3. `src/constants.hpp` 注释更新为“3.1.2 反射/反汇编复核”。
4. 版本号 2.1.0 → **2.1.1**（`plugin.cpp` 描述符、`manifest.json`、
   `pluginmaster.json` 与下载链接）。

没有放宽门禁。相机偏移本来就没动；前轮反打的 offset 是这次游戏更新真实移动的。

## 6. 验证

- `./build-mingw.sh --fresh --sdk ../Download/Anomaly-3.1.2-sdk --run-tests`：
  MinGW 全量重建通过，全部验收夹具（chase / burnout / orbit / handheld /
  wheel steer / orbit UI hide / settings groups / persistence / panel）通过。
- 安装到 `Anomaly-3.1.2-runtime\Anomaly\plugins\robineatcorn.nfs2015-chase-cam`。
  首版（只改 hash）`plugin.dll` sha256 `66eed5ed...6193`，热重载到 generation 10
  后相机生效；加入前轮反打偏移修正后重编为 `a90111bc...bbb5`，generation 11。
- 热重载到 generation 10 后，实测日志：

  ```text
  profile gate: ready (7341beb0ffae6e43c017d20b51a357d30249d4516dd487d5ac91c0cd2fafdf7d)
  nfs2015 chase cam: game-thread camera path enabled
  viewmode: player controller resolved at 0x11717A030 (world slot=0x150538488)
  viewmode: tags third=0x3B1A9 first=0x3B1C6 ... (from the controller array, count=2)
  viewmode: toggle hook installed (target=0x1480A87D0, original=0x11EB80D00, PC=0x11717A030)
  nfs2015 chase cam: game-thread GetViewPoint hook installed
  nfs2015 chase cam: CalcSceneViewInitOptions gate hook installed
  nfs2015 chase cam: first camera cut applied
  viewmode slot: chase=1 tag=none wanted=1 veh=1 en=1
  viewmode watch: ? -> 0x3B1A9(TP) count=2 limit=0 veh=1 en=1 active=1 calls=0
  ```

  随后停车环绕（`orbit state=Enter` → `Orbit` → `Exit`，含 UI 隐藏/恢复）和
  驾驶追车都正常，游戏未崩溃。用户实机确认“镜头生效”。
- 前轮反打修正热重载到 generation 11，实测：

  ```text
  nfs2015 chase cam: wheel steering geometry ready
  (front=2 rear=2 mesh=0x1cf2d53340 anim=0x1c59d799a0)
  ```

  不再出现 `wheel steering geometry unavailable`；漂移时的
  `visual wheel override active` 与前后轮转角日志待实机跑一段确认。
- `anomaly-plugin validate`：`valid ... version=2.1.1`；
  `anomaly-test-host --reload 10 --ticks 60`：`ok reloads=10 ticks=660`。

## 7. 前轮反打（同一个 3.1.2 更新里的真实偏移变化）

相机修好后再看 `wheel steering geometry unavailable`：不是 vehicles 没解析，而是读
`+0x12D8/+0x12E8` 拿到的不是组件数组（data=0x43480000，正好是 float 200.0 的位模式，
num 是亿级垃圾值）。用运行中载具类的 `PropertyLink` 反射链逐属性遍历（偏移来自
profile 的 `ffield.name=32` / `ffield.next=24` / `fproperty.offsetInternal=68` /
`fproperty.propertyLinkNext=72` / `ustruct.propertyLink=112`）：

```text
FrontWheels   off=0x12e0  size=16   (TArray)
RearWheels    off=0x12f0  size=16   (TArray)
bEnableWheelAnimation (anim class) off=0x0c98 size=1
RelativeRotation (SceneComponent)  off=0x0168
AttachParent     (SceneComponent)  off=0x00d8
```

实读 `BP_Vehicle_037_Module_C`：

```text
FrontWheels @0x12e0 data=0x1d365ccf60 num=2 cap=4  [Wheel_FR, Wheel_FL]
RearWheels  @0x12f0 data=0x1d365cffc0 num=2 cap=4  [Wheel_RL, Wheel_RR]
```

两个属性声明在 C++ 基类 `HTWheeledVehicleDrivable` 上（`Owner = 0x7ff4de08b070|1`），
不是某辆车的蓝图，所以这次后移对所有轮式载具生效。

改完后 generation 11 的热重载日志：

```text
nfs2015 chase cam: wheel steering geometry ready
(front=2 rear=2 mesh=0x1cf2d53340 anim=0x1c59d799a0)
```

不再报 `geometry unavailable`，也不再向错误地址写入任何东西。
