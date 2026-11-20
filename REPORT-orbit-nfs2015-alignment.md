# 停车环绕对齐 NFS 2015 调查结论：实施报告

> 对应计划（父目录）：`PLAN-nfs2015-orbit-align-current-findings.md`
> 本文件由任务组 0 建立，之后每个任务组完成时在 §7 打勾并补写对应章节。
> 记录日期：2026-11-21（计划日期 2026-11-21，同一轮）
> 构建环境：WSL2 + MinGW-w64 交叉编译，Anomaly SDK 3.0.0

---

## 1. 改动前的基线（任务组 0 采集）

这一节是后面所有对比的基准。**这里的数字必须在做任何代码改动之前采集**，否则
"默认值从 4.0 改到 5.05 导致断言失败" 和 "模型算错了" 会混在一起，无法区分。

### 1.1 构建环境

| 项目 | 值 |
|---|---|
| 源码目录 | `nfs2015-chase-cam` |
| 构建树 | `build.mingw`（`--fresh` 全部重建） |
| Anomaly SDK | `Download/Anomaly-3.0.0-sdk`（版本 3.0.0） |
| 工具链 | `/usr/bin/x86_64-w64-mingw32-g++-posix`（GNU 10.0.0） |
| 构建类型 | `RelWithDebInfo`，12 路并行 |
| 插件版本（manifest 未改动） | `2.0.6` |

### 1.2 构建与测试结果

执行的命令（计划里指定的两条）：

```sh
cd nfs2015-chase-cam
./build-mingw.sh --fresh --run-tests
ctest --test-dir build.mingw --output-on-failure
```

结果：**编译通过，9 个 ctest 全部通过，0 失败。**

```
100% tests passed, 0 tests failed out of 9
Total Test time (real) =   0.32 sec
```

逐个夹具的 `ctest` 结果与断言条数（断言条数指夹具打印的 `ok ...` 行数，用于以后
确认"新加的断言确实跑起来了"、"没有为了变绿删断言"）：

| # | ctest 名称 | 夹具可执行文件 | 结果 | 断言条数 | 结束行 |
|---|---|---|---|---|---|
| 1 | `nfs2015_chase_cam_model` | `nfs2015_chase_cam_test.exe` | Passed | 128 | （无汇总行） |
| 2 | `nfs2015_burnout_shot_model` | `nfs2015_burnout_shot_test.exe` | Passed | 590 | `all burnout shot tests passed` |
| 3 | `nfs2015_orbit_shot_model` | `nfs2015_orbit_shot_test.exe` | Passed | 92 | `all parked orbit shot tests passed` |
| 4 | `nfs2015_orbit_handheld_model` | `nfs2015_orbit_handheld_test.exe` | Passed | 55 | `all parked orbit handheld tests passed` |
| 5 | `nfs2015_orbit_ui_hide_model` | `nfs2015_orbit_ui_hide_test.exe` | Passed | 130 | `all orbit UI-hide tests passed` |
| 6 | `nfs2015_wheel_steer_model` | `nfs2015_wheel_steer_test.exe` | Passed | 1（汇总行） | `wheel_steer_test: ok` |
| 7 | `nfs2015_settings_groups_model` | `nfs2015_settings_groups_test.exe` | Passed | 109 | `all settings group checks passed` |
| 8 | `nfs2015_settings_persistence_model` | `nfs2015_settings_persistence_test.exe` | Passed | 221 | `all settings persistence checks passed` |
| 9 | `nfs2015_settings_panel` | `nfs2015_settings_panel_test.exe` | Passed | 116 | `all settings panel checks passed` |

`ok` 行合计 **1441 条**（不含 `wheel_steer_test` 的单行汇总）。后续任务组跑完回归后，
这个数字只应该变大，不应该变小。

### 1.3 Git 基线

```
$ git -C nfs2015-chase-cam status --short
（无输出，工作区干净）

$ git -C nfs2015-chase-cam log --oneline -3
6778bf9 feat(anchor): default the borrow fallback off and expose it in Advanced
90abae4 修正 shot_blend 注释：过渡曲线由 OrbitShot 评估
5e6424a feat(orbit): TARGET 噪声改用运行时确认的 Instance18 双 octave
```

`HEAD` = `6778bf9`。所有改动都应该在这个提交之后才能算"本轮引入"。

### 1.4 实机基线截图：**跳过（按计划 §4 任务组 0 的授权）**

计划要求：如果 NTE 正在运行并且已经在环绕画面，就截一张当前默认（4.0 / 0.724 /
0.671 / 45°）的图并抄下诊断行；否则跳过，不要为了截图重启游戏。

实测：

- `NTEGame.exe` 确实在运行（pid 844，启动时间 2026-11-20 22:32）。
- 但该进程在四个 Anomaly runtime（2.5.0 / 2.6.0 / 2.7.0 / 3.0.0）的
  `anomaly-runtime.jsonl` 里**一条记录都没有**，也就是说这一次游戏启动没有注入
  Anomaly，插件不可能在跑。最近一次带插件的会话是 pid 36800（日志最后更新
  01:25），其最后几条正是 `orbit state=Orbit blend=1.00 ...`。
- 抓了一张桌面图（`../captures/tg0_baseline_orbit.png`）确认：前台只有 WSL 终端，
  游戏窗口不在前台，画面里没有环绕构图。

结论：**当前没有可截的环绕画面，按计划跳过**，不为此重启游戏。所以本节没有旧默认
构图图，实机 A/B 从任务组 5 之后的新默认开始记录（见 §8）。

---

## 2. 改动前的现状快照（代码里当前生效的值）

这些是任务组 1/2 要动的具体位置。列在这里是为了让"改了什么"可以对着看。

### 2.1 出厂默认值（`src/constants.hpp`）

| 设置键 | 当前默认 | 范围 | 行号 |
|---|---|---|---|
| `orbitEnabled` | `true` | — | 121 |
| `orbitEntryDelay` | `8.0` s | 0–60 | 122–124 |
| `orbitPeriod` | `60.0` s | 10–600 | 125–127 |
| `orbitRadius` | `4.0` m | 2–12 | 128–130 |
| `orbitHeight` | `0.724` m | 0.3–3.0 | 135–137 |
| `orbitLookAtHeight` | `0.671` m | 0.3–3.0 | 144–146 |
| `orbitWeave` | `1.0` m | 0–3 | 147–149 |
| `orbitFov` | `45.0`° | 20–70 | 151–153 |
| `orbitShakeStrength` | `1.0` | 0–2 | 156–158 |
| `orbitShakeFrequency` | `5.0` Hz | 2–8 | 159, 163–164 |
| `orbitHideUi` | `true` | — | 162 |

计划要把前四项改成 5.05 / 1.83 / 1.23 / 28，频率改成 2.0，并新增
`orbitTargetSway`（默认 1.0，范围 0–2）。

注意 `src/constants.hpp` 现在有三段注释与计划直接冲突，任务组 2 必须一起改写，否则
注释会继续告诉后来的人"默认值是照着车调的"：

- `kDefaultOrbitHeight` 上方写着 "The shipped default is tuned on the car rather
  than copied from the guide: the guide's 5.05 m / 1.83 m / 1.23 m reference
  geometry stays in orbit_shot.hpp as that model's fallback."
- `kDefaultOrbitLookAtHeight` 上方用 `0.724 - 0.671 = 0.053 m` 举例算仰角。
- `kDefaultOrbitFov` 上方写着 "45 degrees is about 72 degrees horizontal at
  16:9"（计划要求改成垂直 FOV 标签，水平值动态计算）。

还有 `kDefaultOrbitShakeFrequency = 5.0` 上方的注释把 2..8 Hz 说成 "useful
handheld vs too fast" 带，改默认到 2.0 之后要把这段重新表述（2 Hz 现在是默认，
不再是下限的例子）。

### 2.2 模型层回退值（`orbit_shot.hpp` 的 `OrbitShotConfig`）

`OrbitShotConfig` 的 fallback 已经是参考几何的一部分：

| 字段 | 当前 fallback | 说明 |
|---|---|---|
| `radius_m` | `5.05` | 已经是参考半径 |
| `elevation_degrees` | `6.8`（`kOrbitElevationDegrees`） | 已经对齐 |
| `look_at_height_m` | `1.23`（`kOrbitLookAtHeightM`） | 已经对齐 |
| `pivot_forward_m` | `0.772`（`kOrbitPivotForwardM`） | A 级，已实现 |
| `weave_amplitude_m` | `1.0` | 现有近似（`Instance74`） |
| `vertical_fov_degrees` | `45.0` | **计划要改成 28.0** |
| `handheld_strength` | `1.0` | 保留 |
| `handheld_frequency_hz` | `kOrbitHandheldFrequencyHz` | presenter 会用设置值覆盖 |
| `entry_delay / entry_time / exit_time` | `8.0 / 2.0 / 1.0` | 保留 |
| `entry_speed_mps / hold_speed_mps` | `0.14 / 0.14` | 4 布尔门控的近似，保留 |
| `position_tau` | `0.0` | 阻尼关闭，保留 |

关键点：**模型 fallback 已经是 5.05 / 1.23，而出厂设置默认还是 4.0 / 0.671**。
这两个不是一回事，任务组 4 的断言失败要按这个区分（计划 §3 第 5 条异常条件）。

另外，`camera_height_m` 的 fallback 是 `0.0`，含义是"用 `elevation_degrees`
决定仰角"。presenter 每次都会用 `context.orbit_height`（出厂 0.724）覆盖它，所以
运行时的实际仰角来自设置值，不是 6.8°。改成 1.83 之后沿半径 5.05、注视点 1.23
就会得到 6.8° 仰角（`asin((1.83-1.23)/5.05) ≈ 6.83°`）。

### 2.3 roll 相关

- `orbit_shot.hpp:529` 无条件写 `view.roll_degrees = 0.0;`，环绕期间地平线绝对水平。
- `shot_blend.hpp:164` 会 `result.roll = Lerp(...)` 混合，`shot_blend.hpp` 头部注释
  写的是 "parked orbit Roll 0.0"。任务组 1 加 roll 噪声后这两处注释要同步更新。
- 追车相机的 `max_roll_degrees`（`chase_cam.hpp`）与环绕无关，不要动。

### 2.4 诊断行

- 面板行（`src/ui.cpp:208` 的 `snprintf`）：`<state>  blend %.2f  hold %.1fs  t
  %.1fs  phase0 %.1f  in %c  v %.2f  shake=(%.3f,%.3f)`。
- 日志行（`src/presenter.cpp:107` 的 `snprintf`）：`nfs2015 chase cam: orbit state=%s
  blend=%.2f hold=%.2fs time=%.2fs phase0=%.1f in=%d speed=%.2f
  shake=(%.3f,%.3f)`。
- 复位路径 `PublishOrbitReset`（`src/presenter.cpp:118`）逐字段清零，新字段要加进来。

---

## 3. 证据等级对照表（本轮执行的判定标准）

这张表是计划 §1.3 的原样落地版。**每个改动都必须能对到自己那一行**；标 B/C 级的
一定要做成可调参数并在注释里写清"未闭环"，不能写成"已证实"。

| 项目 | 本轮取值 | 证据等级 | 代码/文档要求 | 状态 |
|---|---|---|---|---|
| 枢轴前移 0.772 m | 保持 | A 级：反编译 `s=(0,1.2258,0.7720)` + 同帧数值 | 已有，不改 | 已完成（历史） |
| 默认半径 5.05 m、仰角 6.8° | 改为 5.05 / 1.83 / 1.23 | A 级：资产 + 运行时几何 | 直接写死为默认 | 已完成（TG-1 模型回退值 / TG-2 出厂默认） |
| 运行时 FOV 28° | 改为 28° | A 级：45→43→42.9957→控制器 −15 | 直接写死为默认；注释写"43−15" | 已完成（TG-1 / TG-2） |
| `Instance18` 注视点噪声 | 保持 0.02 / 0.07 m、2 Hz、2 octave | A 级：资产 + 反编译 + 运行时 | 已有；只修设置默认频率 | 已完成（TG-2 频率 5→2 Hz） |
| `Instance16` roll 噪声 | 新增 ±0.7°、2 Hz、2 octave | A 级：资产 + 反编译 | 新增，和手持强度共用 | 已完成（TG-1，测试见 §11.5） |
| 注视点低频偏移 | 新增，按 `Instance192/188` 轨道 + 1 秒阻尼 | B 级：资产图 + 实测小圈，`Instance279` 实际输出幅度未直接读到 | 必须可调，注释写"B 级，幅度未闭环" | 已完成（TG-1 模型 / TG-2 设置与滑杆） |
| 机位切向 ±1 m 摆动 | 保留 | B/C 级：`Instance74` 是序列 transform，是否移动相机臂未闭环 | 保留但可调，注释写"现有近似，未被证实也未被推翻" | 已完成（TG-1 注释） |
| 30 Hz 噪声层 | 不加 | A 级负结果：60 Hz 采样无 0.1 m 签名 | 代码注释写"已测，不加" | 已完成（TG-1 注释） |
| 入场 2 秒 / 出场 1 秒 smoothstep | 保持 | A 级形状、consumer 未闭环 | 不要重复平滑 | 保持 |
| 4 布尔门控 | 保持 0.14 m/s + 无输入 + 8 秒 | C/B 级：只确认 `CanSurrender` | 文档标"近似" | 保持 |
| 阻尼系数 / 过渡 consumer | 保持现状 | C 级 | 文档标"未解"，不写成已证实 | 保持 |

---

## 4. 默认值变更表（旧 → 新）

| 项目 | 旧默认 | 新默认 | 依据 |
|---|---|---|---|
| 环绕半径 `orbitRadius` | 4.0 m | **5.05 m** | 资产 + 运行时几何（A 级） |
| 相机高度 `orbitHeight` | 0.724 m | **1.83 m** | 资产 + 运行时几何（A 级） |
| 注视点高度 `orbitLookAtHeight` | 0.671 m | **1.23 m** | 资产 + 运行时几何（A 级） |
| 垂直 FOV `orbitFov` | 45.0° | **28.0°**（= 43 − 15） | 运行时 FOV 链（A 级） |
| 手持/注视点噪声频率 `orbitShakeFrequency` | 5.0 Hz | **2.0 Hz** | 资产 + 反编译（A 级） |
| 注视点低频偏移 `orbitTargetSway`（新键） | — | **1.0**（范围 0–2） | B 级，幅度未闭环 |
| 机位切向摆动 `orbitWeave` | 1.0 m | 1.0 m（不变） | B/C 级，保留 |

旧值 4.0 / 0.724 / 0.671 / 45° 仍然可以手动填回自定义滑杆，只是不再是默认。

---

## 5. 两层摆动的语义（写清楚，避免下次又混在一起）

| | 机位切向摆动（`orbitWeave`） | 注视点低频偏移（`orbitTargetSway`，新） |
|---|---|---|
| 动谁 | 相机位置沿环绕轨迹的切线方向 | 只动注视点，相机位置不动 |
| 来源 | `Instance74` 的序列 transform（60 秒 `TranslationX` 轨道） | `Instance192`(X) / `Instance188`(Y) → `Instance279` → `Instance25` 的 TARGET 反应 |
| 时间形状 | 15 s / 45 s 各一次 ±1 m 峰值 | 60 秒轨道；X 在 22.5 / 37.5 s 到 ±1 m，Y 在 0 / 30 / 60 s 到 −0.3 m |
| 证据等级 | B/C：`Instance74` 是否真的移动相机臂未闭环 | B：轨道形状有资产图 + 实测小圈，`Instance279` 输出幅度未直接读到 |
| 观感（预期） | 相机左右平移，注视点距离保持 5.05 m | 车在画面里缓慢游移，30 秒附近上下偏移最明显 |
| 归零方式 | `orbitWeave = 0` | `orbitTargetSway = 0` |

两个默认都是 1.0。如果实机观感过头，**先单独把一层调到 0 确认是哪一层**，不要直接
改证据常量（计划 §3 第 7 条）。这一节只讲这两层横向摆动；地平线横滚是第三层，单独见 §6。

---

## 6. roll 噪声的语义

- 幅度 ±0.7°（`Instance16`），2 Hz 基频 + 2 octave（第二层 4 Hz，权重 0.1），
  A 级。
- 和现有"手持抖动强度"共用 `orbitShakeStrength`：强度 0 时 `roll_degrees`
  精确为 0，地平线绝对水平。
- 只写 `view.roll` / `view.roll_degrees`，**保持 `view.up = (0,0,1)`**，不用歪 up
  向量伪造 roll。如果写进去画面不倾斜，说明 presenter / route B 没消费该字段，
  要汇报而不是改 up。
- 入场/出场由 `shot_blend.hpp` 的 `Lerp` 淡入淡出。

---

## 7. 任务组进度

- [x] **TG-0 基线、护栏和现状快照**（本文件 §1–§4）
- [x] TG-1 模型层：注视点低频偏移 + roll 噪声（`orbit_shot.hpp`，见 §11.2）
- [x] TG-2 默认值、设置、UI 与本地化（`src/*`, `locales/zh-CN.json`，见 §11.3）
- [x] TG-3 Presenter、混合与诊断（见 §11.4）
- [x] TG-4 测试与回归（见 §11.5）
- [x] TG-5 文档、证据表和实机验收清单（见 §11.6；§8 的实机项仍保持"待测"）

每个任务组完成后在对应的 `- [ ]` 打勾，并在 §11 追加一小节记录实际改动与测试数字。

---

## 8. 实机 A/B 验收清单（交给用户实测，"结果"列保持待测）

按下面顺序看，**出问题时先归零对应层，不要同时改多个旋钮**。

| # | 项目 | 期望 | 结果 | 备注 |
|---|---|---|---|---|
| 1 | 默认构图（5.05 / 1.83 / 1.23） | 相机在车顶上方略微俯视，车身画面占比接近 `../nfs2015-idle-cam/contact_open.png` | 待测 | 车被裁切或相机贴地 → 先汇报，不要改回 4.0/0.724 |
| 2 | FOV 28° | 侧视角车身接近占满画面宽度但不裁切 | 待测 | 太紧先试 32–35°，不要一次跳回 45° |
| 3 | 机位切向摆动（`orbitTargetSway=0`, `handheld=0`，只看 `orbitWeave`） | 15/45 秒有横向位移；机位到注视点距离保持 5.05 m | 待测 | 若忽远忽近说明摆动加到了半径上 |
| 4 | 注视点低频偏移（`orbitWeave=0`, `handheld=0`，只看 `orbitTargetSway`） | 0–60 秒车在画面里缓慢游移，30 秒附近上下偏移最明显；没有 15/45 秒左右甩 | 待测 | 30 或 60 秒出现跳变 → 汇报（异常条件 2） |
| 5 | 三层同时开（默认 1.0 / 1.0 / 1.0） | 观感可接受 | 待测 | 过头就先单层归零 |
| 6 | roll 噪声（强度 1.0） | 地平线约 ±0.7° 轻微晃动；强度 0 时绝对水平 | 待测 | 写了不平 → 汇报（异常条件 3） |
| 7 | 手持噪声频率 | 默认 2 Hz（不再是 5 Hz） | 待测 | 太飘先减半强度，再考虑降频率 |
| 8 | 入场/出场 | 2 秒进、1 秒出，没有双重平滑或新甩动 | 待测 | 出现 pop → 汇报（异常条件 4） |
| 9 | 设置兼容 | 旧配置能加载；新滑杆能持久化；`orbitMode=默认` 用新参考值，`自定义` 保留用户值 | 待测 | 加载报错 → 汇报（异常条件 6） |

**排查顺序（先归零，再逐层加回）**

1. 先只看构图：把 `orbitWeave`、`orbitTargetSway`、`orbitShakeStrength` 三个都设为 0。
   这时相机是干净的正圆，15/45 秒不甩、30 秒不上下飘、地平线绝对水平。如果这一步就不对，
   问题在几何或 FOV，与三层摆动无关。
2. 只看机位横向摆动：`orbitWeave = 1.0`，另两个仍为 0。期望 15/45 秒左右平移、机位到注视点
   的距离始终 5.05 m。
3. 只看注视点低频摆动：`orbitTargetSway = 1.0`，`orbitWeave = 0`。期望车在画面里缓慢游移，
   30 秒附近上下偏移最明显，**没有** 15/45 秒那种左右甩。
4. 只看手持/横滚：`orbitShakeStrength = 1.0`，前两个为 0。期望高频细碎抖动 + 地平线约
   ±0.7° 的轻微晃动。
5. 三层全开（默认 1.0 / 1.0 / 1.0）。观感过头时回到第 2–4 步单独减一层，不要直接改代码里的
   证据常量。
6. 最后才看 FOV：构图太紧先试 32–35°，不要一次跳回 45°。

**异常条件编号**（对应计划 §3，汇报时指明编号；不要自己先把常量改回去）

| # | 触发条件 | 处理 |
|---|---|---|
| 1 | 构图被裁切或相机穿地 | 截图 + 诊断行汇报，不要改回 4.0 / 0.724 / 0.671 / 45，也不要偷偷改 FOV |
| 2 | 注视点低频偏移在 30 秒或 60 秒处跳变 | 汇报，不要用额外平滑把问题盖住 |
| 3 | `roll_degrees` 写了但画面不倾斜 | 汇报（说明 presenter / route B 没消费），不要改歪 `up` 假装 roll |
| 4 | 入场/出场出现新的甩动或双重平滑 | 停下来汇报 |
| 5 | 现有几何单测变红 | 先分清"模型 fallback"和"出厂默认"，不要放宽容差或改回默认值 |
| 6 | 旧设置文件加载失败或面板测试大面积变红 | 先查 `orbitTargetSway` 的向后兼容，汇报 |
| 7 | 三层同时 1.0 观感明显过头 | 让用户单层归零 A/B，不要直接改证据常量 |

---

## 9. 仍未闭环的清单（六项，一行都不许写成已证实）

| 项目 | 现状 | 证据等级 | 处理方式 |
|---|---|---|---|
| `Instance279` 的实际输出幅度 | 只读到轨道形状和实测小圈，幅度没有直接读数 | B | `orbitTargetSway` 可调，默认 1.0，注释标"幅度未闭环" |
| 4 布尔门控 | 只解出 `Instance78` 的 `CanSurrender`，另外两个字段未命名 | C/B | 继续用 0.14 m/s + 无输入 + 8 秒近似，文档标"近似" |
| FOV 阻尼 | 运行时 45→43→42.9957 的收敛过程未完全解 | B | 不实现阻尼，直接给目标 FOV |
| 过渡 consumer | 入场/出场权重的下游消费者未闭环 | C | 保持现状，`shot_blend.hpp` 只消费权重 |
| `Instance74` 是否移动相机臂 | 是序列 transform，未见它写入相机位置 | B/C | 保留现有近似，注释写"未被证实也未被推翻" |
| 30 Hz 高频噪声层 | 60 Hz 采样证明被压到约 0.013 m，与资产 0.1 m 签名不符 | A（负结果） | 不加，代码注释写"已测，不加" |

---

## 10. 给后续任务组的提醒（从基线里读出来的坑）

1. **两套默认值不要混**：`OrbitShotConfig` 的 fallback 已经是 5.05 / 1.23，但
   `src/constants.hpp` 的出厂设置是 4.0 / 0.671。任务组 4 的断言失败要按
   "模型 fallback" 和 "出厂默认" 分开判断，不要放宽容差。
2. **`camera_height_m` 会覆盖仰角**：presenter 总是用设置值覆盖它，所以 `6.8°`
   只在 `camera_height_m == 0` 时才生效。要保证"1.83 高度 + 1.23 注视点 + 5.05
   半径"三者同时成立，不要只改其中一个。
3. **`orbitFov` 是持久化的**：`REPORT-parking-orbit-cam.md` 记录过"已有配置文件里
   的 `orbitFov` 会保留旧值"。所以改默认 28° 之后，**已有配置的用户仍然看到旧的
   45°**，需要删键或恢复默认才会切到新值。这属于预期行为，但验收时要先确认自己的
   配置里 `orbitFov` 是多少，否则会误判为"默认值没生效"。
4. **诊断行有两个地方**：`src/ui.cpp` 的面板行和 `src/presenter.cpp` 的日志行。
   加字段两边都要加，`PublishOrbitReset` 的逐字段清零也要加，否则关闭环绕后残留
   旧值。
5. **断言条数基线 1441**（`ok` 行，不含 `wheel_steer_test` 单行汇总）。回归后
   数字只应上升。
6. **本文件 §8 只做 "待测"**：实机项需要用户在游戏里逐条确认，不要在没实测的情况
   下打勾。

---

## 11. 任务组执行记录

### 11.1 TG-0（2026-11-21）

- 基线构建和 9 个 ctest 通过，断言条数 1441（详见 §1.2）。
- 实机基线截图按授权跳过（详见 §1.4）：`NTEGame.exe` 在跑，但这个进程没有注入
  Anomaly，不是环绕画面。
- 工作区干净，`HEAD = 6778bf9`。
- 本报告建立，没有代码改动。

### 11.2 TG-1（2026-11-21，模型层）

改动文件：

| 文件 | 改动 |
|---|---|
| `orbit_shot.hpp` | 新增注视点低频偏移层、roll 噪声层、`target_sway` 配置、FOV 默认 28°、头部注释按证据等级重写 |
| `tests/orbit_shot_test.cpp` | 夹具显式 `target_sway = 0`；`TestSlopeFollow` 显式 `handheld_strength = 0` |
| `tests/orbit_handheld_test.cpp` | 夹具显式 `target_sway = 0`；`TestTargetOnly` 的 "up/roll/fov" 拆开 |

没有改 `src/*`、`locales`、`shot_blend.hpp`、`presenter.cpp`（分别是 TG-2、TG-3 的范围）。

#### 新增的常量

| 常量 | 值 | 用途 |
|---|---|---|
| `kOrbitTargetSwayDampSeconds` | 1.0 | `Instance127` 的 DampTime |
| `kOrbitTargetSwayPeriodSeconds` | 60.0 | 轨道周期，和机位摆动同一个环绕时钟 |
| `kOrbitTargetSwayRightM` | 1.0 | `Instance192` 的幅度（米） |
| `kOrbitTargetSwayUpM` | 0.3 | `Instance188` 的幅度（米） |
| `kOrbitTargetSwayXTimes/Values` | 0/15/22.5/30/37.5/45 s → 0/0/+1/0/−1/0 | 归一化轨道 |
| `kOrbitTargetSwayYTimes/Values` | 0/15/30/45 s → −1/0/−1/0 | 归一化轨道 |
| `kOrbitVerticalFovDegrees` | 28.0 | 43 − 15 |
| `kOrbitRollNoiseDegrees` | 0.7 | `Instance16` ROLL magnitude |
| `kOrbitRollNoiseFrequencyHz` | 2.0 | 与手持频率共用的编译期检查 |
| `kOrbitRollNoiseRoughness` | 0.1 | 第二层（4 Hz）权重 |
| `kOrbitRollSeed` / `kOrbitRollSeed2` | `0xA5A5A5A5` / `0x5A5A5A5A` | 两层独立 LCG 序列 |

#### 微观决定（计划 §2 授权范围内）

1. **CatmullRom 约定**：用非均匀（时间归一化的中心差分）形式，切线 =
   `(p[i+1] − p[i−1]) / (t[i+1] − t[i−1])`。原因是 `Instance192` 的键间距是
   15 / 7.5 / 7.5 / 7.5 / 7.5 / 15 秒混排，均匀参数化会在 7.5 秒段产生更大的过冲。
2. **端点 wrap**：键表只存 `[0, 60)` 内的键（X 6 个、Y 4 个），60 秒端点由 wrap
   提供（X 回到 0，Y 回到 −1）。这样每个键都有左右邻居，不需要重复键。
3. **归一化存储**：Y 轨道存 −1..0 再乘 0.3 m，X 轨道存 ±1 再乘 1.0 m。这样计划里
   "X clamp 到 ±1、Y clamp 到 −0.3..0" 在米制下成立，不会变成 −0.09 m。
4. **阻尼**：一阶指数 `alpha = 1 − exp(−dt / 1.0)`；进入环绕的第一帧直接把阻尼值
   播种到 `t = 0` 的轨道值（X=0、Y=−0.3），不从 0 爬。
5. **零分支**：`target_sway == 0` 时状态里写精确的 `0.0`，`target` 直接等于
   `pivot`；`target_sway == 0` 但噪声开着时 `0.0 + shake` 在 IEEE‑754 下仍然精确，
   所以两条路径都保持"关掉即回到改动前".
6. **roll 噪声**：两个 octave（2 Hz + 4 Hz）、独立种子、第二层权重固定 0.1（不用
   `handheld_roughness`，那是注视点噪声的 0.5）。强度和频率跟手持滑杆共用；改频率时
   roll 两个 band 的相位也会按比例缩放，避免拖滑杆时跳相。

#### 验证结果

| 项目 | 结果 |
|---|---|
| `./build-mingw.sh --fresh --run-tests` | 通过（0 warning），夹具输出 1442 条 `ok` |
| `ctest --test-dir build.mingw --output-on-failure` | 9/9 通过 |
| 断言条数（`ok` 行） | 1441 → **1442**（handheld 55 → 56，其余不变） |
| `target_sway = 0` 且 `strength = 0` | `view.target == pivot` 逐位相等 |
| `target_sway` 1↔0 对比 | `view.position` 逐位相等，只有 `view.target` 变化 |
| `strength = 0` | `view.roll == 0.0`、`view.roll_degrees == 0.0` 逐位 |
| 轨道键（strength 1） | t=0/15/22.5/30/37.5/45 s 命中 0/0/±1、−0.3/0/−0.15/−0.3；60 秒 wrap 连续 |
| 轨道包络（60 秒，240 Hz） | X −0.968..0.965 m，Y −0.300..−0.004 m |
| roll 噪声（120 秒，60 Hz） | 峰值 0.7306°、RMS 0.3292°，无 NaN，界内（0.77°） |
| 默认几何（`OrbitShotConfig{}`，750 帧后） | 半径 5.0501 m、相机高度 1.8279 m、仰角 6.7998°、FOV 28.0°、枢轴前移 0.772 m |

上面这些"逐位/包络"检查是用一个临时夹具（`/tmp/tg1_check.cpp`，没有提交）跑的，
TG-4 会把它们写成正式的 `TestTargetSwayZero` / `TestTargetSwayDoesNotMoveEye` /
`TestRollNoiseBounds` / `TestRollNoiseOff` / `TestReferenceDefaults`。

#### 为什么改了两个现有测试

这次改动会让两个旧断言变红，原因是**新层默认开启**，不是模型算错：

1. `orbit_shot_test` 的 `6.1-7 target is the shifted pivot` 断言 `target == pivot`。
   夹具原来就把 `weave_amplitude_m` 设成 0 来隔离几何，现在同样显式
   `target_sway = 0`。没有放宽任何容差。
2. `orbit_handheld_test` 的 `6.1-7 noise never moves up/roll/fov` 断言 roll 在
   强度 1 和强度 0 下完全相同。roll 现在就是跟强度滑杆走的噪声层，所以拆成两条：
   "never moves up/fov"（仍然逐位比较）和 "roll noise follows the strength slider"
   （强度 0 时精确为 0，强度 1 时 `roll == roll_degrees × π/180` 且不为 0）。

`TestSlopeFollow` 的 `6.1-13 slope keeps roll at 0` 也是同一类：这个断言的意图是
"坡度不引入 roll"，所以夹具里把噪声关掉，roll 仍然要求精确 0；roll 噪声本身由
handheld 夹具覆盖。

#### 过程中发现并修掉的一个隐患

采样器第一版用下标判断（`j == 0 ? times[0] + span : times[j]`、`n <= j ?
... : ...`）把相邻键换算到未卷绕的时间轴上。对当前的 X/Y 轨道它凑巧看不出问题，
因为 wrap 段的两个相邻键值刚好都是 0，分子为 0 把错误分母掩盖了。但 wrap 段里
"再下一个键"的时间被算成 `times[1] = 15` 而不是 `15 + 60 = 75`，分母变成负数，
再由 `max(1e-9, ...)` 夹成极大切线。用一个合成轨道（`{0,10,20}` 秒 / `{0, 0.5, 0}` /
30 秒周期，在 t=25 应为 −0.0625）就把它验证出来了：旧写法会得到爆掉的值。
改为把每个邻居用循环推到参考键同侧（`while (t <= t_ref) t += span`），合成轨道
输出 −0.062500000；真实轨道的包络与修正前完全一致。

#### 这一组刻意没做的事

- 没把 `target_sway` / `roll` 接到设置、UI、诊断（TG-2 / TG-3）。
- 没改 `shot_blend.hpp` 的 "parked orbit Roll 0.0" 注释（TG-3）。
- 没加 TG-4 的正式新测试。
- 没动 4 布尔门控、入场/出场曲线、碰撞、景深。

### 11.3 TG-2（2026-11-21，默认值、设置、UI、本地化）

改动文件：

| 文件 | 改动 |
|---|---|
| `src/constants.hpp` | 默认值切到 NFS 参考（5.05 / 1.83 / 1.23 / 28° / 2 Hz），新增 `kDefaultOrbitTargetSway`（1.0）与 0–2 范围；三段与计划冲突的旧注释重写 |
| `src/settings_groups.hpp` | `CustomSettings` 新增 `orbit_target_sway` |
| `src/state.hpp` | `DisplayState` 新增 `orbit_target_sway` / `orbit_target_sway_x_m` / `orbit_target_sway_y_m` / `orbit_roll_noise_degrees`；`Context` 新增 `orbit_target_sway` 原子量与三个诊断原子量 |
| `src/settings.cpp` | `OrbitSettingsValid` 增加 sway 参数与 0–2 校验；JSON 编码/解码新增 `orbitTargetSway`（追加在 orbit 块末尾）；`ApplyGroup` 写入生效值 |
| `src/display.cpp` | 把四个新量拷进显示快照 |
| `src/presenter.cpp` | 把 `context.orbit_target_sway` 传进 `orbit_config.target_sway`；诊断发布/复位增加 sway 与 roll；日志行加 `sway=` / `roll=` |
| `src/ui.cpp` | Orbit 组新增 sway 滑杆；weave 文案改成 "Camera lateral weave (m)"；FOV 文案改成动态计算的水平等效；诊断行加 `sway=` / `roll=`；`DrawCustomSlider` 拆出 `DrawCustomSliderCore` |
| `localization.hpp` | `Label()` 增加带 `{0}` 参数的版本，供动态 FOV 文案使用 |
| `locales/zh-CN.json` | 新增 `setting.orbit_target_sway`；更新 weave / FOV / 手持强度 / 摘要 / `diag.orbit` 文案 |

#### 设置键与默认值

| 键 | 默认 | 范围 | 说明 |
|---|---|---|---|
| `orbitRadius` | **5.05** | 2–12 | 旧默认 4.0 现在只能手填 |
| `orbitHeight` | **1.83** | 0.3–3.0 | 旧默认 0.724 |
| `orbitLookAtHeight` | **1.23** | 0.3–3.0 | 旧默认 0.671 |
| `orbitFov` | **28**（= 43 − 15） | 20–70 | 旧默认 45 |
| `orbitShakeFrequency` | **2.0** Hz | 2–8 | 旧默认 5.0；注视点噪声和 roll 噪声共用 |
| `orbitTargetSway`（新） | **1.0** | 0–2 | 注视点低频偏移；0 关掉该层 |
| `orbitWeave` | 1.0 | 0–3 | 机位切向摆动，不变 |

#### 兼容性

- `orbitTargetSway` 追加在 orbit 块的**最末尾**（`orbitHideUi` 之后），现有键的位置和相对顺序完全不变；
  持久化测试用一条顺序断言钉住这一点。
- 缺键回退 1.0；2.0.5 文档（连 orbit 数字都没有）加载后 sway = 1.0，`orbitMode` 落在 Default。
- 超出 0–2 的手改文档会被整份拒绝（和其余数值的既有行为一致），不会被猜成某个值。
- `orbitFov` 仍然是持久化的：以前存过配置的用户会继续看到自己存的 45，需要删键或"恢复默认"才会
  切到 28。这是预期行为，实机验收时要先确认自己的配置文件。

#### 测试结果

| 项目 | 结果 |
|---|---|
| `./build-mingw.sh --fresh --run-tests` | 通过，0 warning，夹具输出 **1476** 条 `ok` |
| `ctest` | 9/9 通过 |
| 断言条数 | 1442 → **1476**（groups 109 → 120、persistence 221 → 232、panel 116 → 128） |
| `./verify-release.sh --allow-missing-host-tools` | ctest 通过；剩余 5 条失败与本组无关，见下 |

新增的断言覆盖：

- `settings_groups_test`：五个参考默认值按数字钉住（5.05 / 1.83 / 1.23 / 28 / 2.0）、sway 默认 1.0 与
  0–2 范围、Custom 生效 0.25、Opposite 回落到 1.0。
- `settings_persistence_test`：`"orbitTargetSway":0.25` 写入、键顺序、"自定义值往返"、"缺键回退 1.0"、
  "超范围文档被拒绝"。
- `settings_panel_test`：Orbit/Custom 从 9 个滑杆变成 10 个；两层摆动的标签互不混淆；sway 滑杆范围
  0–2；FOV 标签在 28° 时显示"48° 水平"、改到 40° 后显示"66° 水平"，且不再出现旧的 72。

#### 关于 `verify-release.sh` 的 5 条失败（预先存在，不是本组引入）

1. `pluginmaster Version is '2.0.5', expected '2.0.6'`，以及两条 DownloadLink 不一致。这是
   `HEAD` 里提交 `e6a2e45 "serve the 2.0.5 index entry until the 2.0.6 release exists"` 的**有意**
   状态：2.0.6 还没发布，索引条目继续指向 2.0.5。
2. `ZIP entry differs from package: locales/zh-CN.json` / `plugin.dll`。`dist/Nfs2015ChaseCam-2.0.6.zip`
   是 11 月 18 日 01:30 的旧包，而 `build.mingw/package` 是刚重建的。`dist/` 在 `.gitignore` 里，
   本计划也不要求出发布包，所以这里不重新打包。

正式发版前需要按 `RELEASE-CHECKLIST-2.0.0.md` 重新打包并更新 `pluginmaster.json`。

#### 这一组刻意没做的事

- 没动 presenter 的混合逻辑和 `shot_blend.hpp`（TG-3）。
- 没改版本号、没出发布包。
- 没在实机里点过新滑杆（当前游戏会话没有注入 Anomaly，见 §1.4）。

### 11.4 TG-3（2026-11-21，Presenter、混合与诊断）

这一组以检查为主，只改了两处注释、补了两组测试。

#### 检查结果（逐条对应计划里的行动项）

| 检查项 | 结果 |
|---|---|
| `AdvancePresenter` 把 `Context` 的新字段读进 `orbit_config` | 通过（TG-2 已接 `target_sway`，其余字段原本就有） |
| `orbit_shot.Update` 的输出先与 `chase_view` 混合，再交给 burnout | 通过：`view = chase_view` → `BlendShotAroundCarPivot(chase_view, orbit.view, …)` → `BlendBurnoutCamera(view, burnout.view, …)`，burnout 是外层 |
| `blend == 1` 时返回 shot 的 `roll_degrees`（新的 roll 噪声） | 通过，并由新测试逐位钉住 |
| `blend == 0` 时返回 base 逐位 | 通过（原 `TestBlendEdges` 用合成位姿，新测试又用真实环绕位姿验了一遍） |
| `BlendShotAroundCarPivot` 的 `pivot_height_m` 仍传 `look_at_height` | 通过；注释里标明混合枢轴没有 0.772 m 前移，属于既有近似 |
| `PublishOrbitDiagnostics` 写入三个新原子量 | 通过（TG-2 已加） |
| 诊断读取不分配、不加锁 | 通过：`display.cpp` 只 load 原子量，`ui.cpp` 用栈上 `line[192]` 做 `snprintf`；唯一的分配是状态切换时的 INFO 日志（既有行为） |
| 关闭路径 1：`orbit_enabled == false` | 通过：`view` 保持 `chase_view` 不变，并 `PublishOrbitReset` |
| 关闭路径 2：`!sample.driving` | 通过（`src/presenter.cpp:293`） |
| 关闭路径 3：视角切换（`view_epoch` 变化） | 通过（`src/presenter.cpp:477`） |
| 关闭路径 4：`epoch` 变化（世界/传送/预设） | 通过（`src/presenter.cpp:487`） |
| 复位清零新字段 | 通过，新测试钉住 |
| roll 真的被消费 | 通过：`rotation[2] = view.roll_degrees` 写进游戏相机，`PublishRouteBModelPose` 也写进 route B 探针。代码层面不是"写了没人读"，实机倾斜仍要 TG-5 目视 |

#### 改动

1. `shot_blend.hpp` 文件头：删掉"parked orbit Roll 0.0"，改成"burnout 特写把 roll 钉在 0，
   环绕带着 `Instance16` 的 ±0.7° 噪声、靠这个 lerp 淡入淡出；要倾斜地平线必须设 shot 的
   `roll`，不能歪 `up`"。同时补上"混合枢轴只有高度、没有 0.772 前移"的说明，并写明这是
   有意为之：加上去会连带改掉 burnout 特写的入场。
2. `src/presenter.cpp` 的 `BlendShotAroundCarPivot` 调用点：补一条注释说明入场绕的是车原点、
   稳态绕的是前移 0.772 m 的枢轴，这个差是既有近似；如果实机看到入场最后一帧有横向台阶，
   先查这里。

#### 新增测试

- `TestRollNoiseFadesWithTheBlend`：真实环绕跑到有 roll 噪声的帧，验证
  `shot.roll == shot.roll_degrees × π/180`、blend 0 逐位去掉噪声、blend 1 逐位保留、
  blend 0.5 等于两端的线性插值（容差 1e-9）。
- `TestReset` 扩展：先确认两层偏移和 roll 噪声都非零，再 `Reset()`，验证状态机、`shake_x/y`、
  `target_sway_x/y`、`roll_noise_degrees` 全部清零（`PublishOrbitReset` 依赖这一点）。

写测试时发现的一个小坑：`SyntheticBaseView()` 里的 `roll = 0.01` 弧度和
`roll_degrees = 0.57` 并不严格对应（0.01 rad 其实是 0.5729°）。混合函数按弧度 lerp 再换算回
角度，直接用这两个数算期望会差 0.003°。新测试里把这一对改成一致的值，并在注释里写明了原因，
没有放宽容差。

#### 结果

| 项目 | 结果 |
|---|---|
| `./build-mingw.sh --fresh --run-tests` | 通过，0 warning，夹具输出 **1483** 条 `ok` |
| `ctest` | 9/9 通过 |
| 断言条数 | 1476 → **1483**（orbit_shot 92 → 99） |

#### 这一组刻意没做的事

- 没加 0.772 前移到混合枢轴（计划明确本轮不改）。
- 没做相机碰撞、防穿插、景深。
- 实机目视项（roll 是否真的让地平线倾斜、入场有没有横向台阶）留给 TG-5 的 A/B 清单。

### 11.5 TG-4（2026-11-21，测试与回归）

把任务组 1 用 `/tmp` 临时夹具验过的东西写成了正式测试，一条断言都没删、一个容差都没放宽。
设置类测试在任务组 2 已经补齐，这一组只补模型行为。

#### `tests/orbit_shot_test.cpp`（+48 条）

| 测试 | 钉住的内容 |
|---|---|
| `TestReferenceDefaults` | `OrbitShotConfig{}` 的六个默认值（半径 5.05、注视点高 1.23、枢轴前移 0.772、垂直 FOV 28、sway 1.0、噪声频率 2.0）逐位钉住；再跑一次默认进环绕，量出 `|eye−pivot| = 5.05`、枢轴高于车原点 1.23、前移 0.772、相机高于车原点 1.828、仰角 6.8°、FOV 28 |
| `TestTargetSwayKeys` | 两个层次分开测：① `OrbitTargetSwayX/Y` 这两条公开曲线在 0/15/22.5/30/37.5/45 s 精确等于资产键（容差 1e-12）；② 经过 1 s 阻尼后的状态值在这些时刻落在键附近（X 0.18 m、Y 0.05 m）；③ 60 s 一圈回到 t=0 的键；④ 全程不超出键的包络（|X| ≤ 1、Y ∈ [−0.3, 0]）、左右两侧都用满 |
| `TestTargetSwayZero` | sway=0 且噪声=0 时，60 秒里每帧 `view.target` 与枢轴逐位相等，状态量精确为 0 |
| `TestTargetSwayDoesNotMoveEye` | 开关 sway 两套模型并跑 60 秒：`view.position`、`view.up`、FOV 逐位一致，只有 `view.target` 不同 |

`TestTargetSwayKeys` 的容差是按阻尼器的物理含义算出来的，不是凑出来的：一阶阻尼的滞后上限是
`τ × max|d/dt|`，X 轨道最快约 0.15 m/s、Y 轨道约 0.03 m/s，τ = 1 s，所以 0.18 m / 0.05 m 就是
这条曲线的最大滞后。真出现形状错误会在 ±1 m 的键上差出整数米，这个容差拦得住。

#### `tests/orbit_handheld_test.cpp`（+19 条）

| 测试 | 钉住的内容 |
|---|---|
| `TestRollNoiseBounds` | 2 分钟：`|roll| ≤ 0.7 × 1.1`、峰值确实用掉大部分幅度（> 0.3°）、随时间变化、无 NaN、`view.roll == view.roll_degrees × π/180`、`view.up` 始终是 (0,0,1)（roll 不歪 up） |
| `TestRollNoiseOffAndLinear` | 强度 0 时 600 帧里 `roll` / `roll_degrees` / 状态量都逐位为 0；强度 0.5 恰好是强度 1 的一半（差值 ≤ 1e-12），说明滑杆是缩放输出而不是重新播种 |
| `TestRollOctaves` | 三个资产常数（0.7° / 2 Hz / 0.1）；用 `kOrbitRollSeed`/`kOrbitRollSeed2` 重放两个频段，逐帧复现 `0.7 × (band1 + 0.1 × band2)`（≤ 1e-12）；再用同一 seed 验证 2 Hz 频段在 1.0 s 的值恰好等于 4 Hz 频段在 0.5 s 的值（都是 r1），把"第二层是两倍频率"钉死 |
| `TestSwayAndNoiseAdditive` | 两层同时开：每帧 `view.target` 等于"枢轴 + 同一 look-local 右/上基底上 (sway+noise) 之和"（≤ 1e-9），且 `view.position` 与两层全关时逐位一致 |

重放频段这条测试要求"进入环绕那一步只前进 1e-3 秒"，所以顺手加了一条断言把这个前提也钉住：
`EnterOrbitFast()` 的第二次更新仍在噪声第一个周期内，返回初始 0。

#### 两个测试设计上的坑（都已改掉，没有放宽容差）

1. 最初把"阻尼后的 sway"直接和资产键比，t=30 s 差了 0.153 m —— 不是模型错，是 1 s 阻尼在
   最陡的那一段本来就该滞后这么多。改成上面"精确曲线 + 有物理依据的滞后上限"两层测法。
2. 60 s 包络扫描一开始接在键循环后面跑，实际结束时轨道时钟是 105 s（相位 45 s），不是 60 s。
   改成用新夹具从 t=0 扫一整圈。

#### 结果

| 项目 | 结果 |
|---|---|
| `./build-mingw.sh --fresh --run-tests` | 通过，0 warning，夹具输出 **1550** 条 `ok` |
| `ctest` | 9/9 通过 |
| 断言条数 | 1483 → **1550**（orbit_shot 99 → 147，orbit_handheld 56 → 75） |
| 相对计划基线 | 1441 → 1550，只增不减 |

设置类回归（任务组 2 已交付，这里复核确认覆盖）：`kDefaultOrbit*` 新值、`orbitTargetSway` 默认
与范围、编码/解码往返、缺键回退 1.0、未知键忽略、orbit 键顺序不变、面板滑杆数量/标签/范围、
FOV 标签不再写死 72°。

### 11.6 TG-5（2026-11-21，文档、证据表和实机验收清单）

这一组只改文档，不动代码、不动版本号、不出发布包。

#### 报告本身（本文件）

- §3 证据等级对照表的"状态"列全部从"待做（TG-x）"改成实际完成位置，一行一条能对到代码。
- §4 默认值变更表、§5 两层摆动的语义、§6 roll 噪声的语义在任务组 0 已经按计划写好，这一组
  复核后只补了两处：§6 写明"不歪 up 向量""写了不倾斜就是没被消费"，§5 末尾补一句"这一节只
  讲两层横向摆动，roll 是第三层，见 §6"。
- §8 新增**排查顺序**：先三层全零确认几何，再逐层加回（weave → sway → shake/roll），最后才
  动 FOV。每一步都写清该看到什么、看着不对该怀疑哪一层。
- §9 未闭环清单保持原样，六项一条没删。

#### `README.md` 的停车环绕章节

- 开头的默认构图改成 5.05 m / 1.83 m / 1.23 m / 6.8° / 28° / 60 秒一圈，并注明这套数字来自
  NFS 2015 的资产和运行时几何，不是照车调的。
- 滑杆表 10 行改成 11 行：新增 `orbitTargetSway`（0–2，默认 1.0），`orbitWeave` 改叫"机位横向
  摆动"，`orbitFov` 默认 28°（16:9 下约 48° 水平），`orbitShakeFrequency` 默认 2.0 Hz，手持
  强度改叫"手持抖动/横滚强度"。
- 补了一段"旧版默认 4.0 / 0.724 / 0.671 / 45° 仍然可用"的说明，以及"已保存配置里的
  `orbitFov` 会保留旧值"。
- 原来那段"5 Hz 手持噪声"扩写成三层：机位横向摆动、注视点低频摆动、手持抖动 + 地平线横滚，
  每层写清动什么、什么时间形状、怎么归零，并给出"15/45 秒左右甩 = 第 1 层、30 秒上下飘 =
  第 2 层、高频抖动和地平线微倾 = 第 3 层"的快速判断。
- 诊断行字段更新成 `… / shake / sway / roll`；设置分组总表里这一组的可调项也补齐了。

#### `REPORT-parking-orbit-cam.md` 的验收清单

- 顶部加了一条指向本报告的说明。
- §1 的设置项列表补上 `orbitLookAtHeight`、`orbitTargetSway`、`orbitMode`；诊断字段同步。
- §3 验收表把旧默认值换成新的（FOV 28°、手持 2 Hz），"地平线倾斜"从"≈0"改成"默认 ±0.7°
  晃动，强度 0 时绝对水平"，并新增三行：30 秒看低频摆动、**两层摆动分不清时一次只归零一层**、
  以及原有的 30 秒相位行改名以免和新行混淆。
- §4 字段表补 `orbitLookAtHeight` / `orbitTargetSway`，FOV 28°、频率 2.0；"完全稳定的轨道"
  建议改成三层全零。
- §5 的 `orbitFov` 说明改成"旧值保留，新的 28° 只对缺键配置生效"。

#### 父目录的调查报告

在 `../NFS2015-停车环绕-剩余未探查逻辑.md` 末尾加了第 13 节"落地情况"，指向本报告，说明哪些
结论已经变成代码、哪些仍标未闭环。历史章节一字未改。

#### 代码注释

`shot_blend.hpp` 的文件头在任务组 3 已经改过（删掉"parked orbit Roll 0.0"，改成 burnout 特写
钉 0、环绕带 ±0.7° 噪声、靠 lerp 淡入淡出，并写明"要倾斜地平线必须设 shot 的 roll，不能歪
up"）。这一组只做复核，没有重复改动。

#### 顺带改的 `RELEASE-NOTES-2.0.6.md`

2.0.6 还没发布（`pluginmaster.json` 仍指 2.0.5），但它的停车环绕默认值表还是旧的
4.0 / 0.724 / 0.671 / 45° / 5 Hz，和代码已经矛盾。这一组把那张表、俯仰角算例、"自定义"可调
项列表、两个夹具的覆盖说明，以及"仍需 NTE 实机确认"那一条都同步成新值，并加了一句"旧值仍可
在自定义里填回"。没有改版本号、没有动发布流程。

#### 刻意没做的事

- 没 bump 版本号、没重打 `dist/` 发布包、没改 `pluginmaster.json`。
- 没在 §8 的"结果"列填任何东西，全部保持"待测"，等用户实机跑。

#### 完成判据自查

- "为什么默认是 5.05 / 1.83 / 1.23 / 28"：README 开头 + 本文件 §4 + 证据表 §3 三处能对上。
- "两层摆动各是什么"：README 三层说明 + 本文件 §5（含分辨方法和归零方式）。
- "哪些还没证实"：本文件 §9 六项 + README 段落末句 + 代码注释里的 B/C 标注。
