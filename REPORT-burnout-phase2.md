# Phase 2 报告：烧胎检测与 sample 管线

- 日期：2026-11-10
- 状态：**PASS（管线与单测）**；实机 `active=1` 仍待一次手动 Shift+W 验证
- 范围：Game tick 读取 Phase 0 信号，生成 `CarSample::burnout`，进入 `SampleRing`；相机仍未消费
- 产物：`car_sample.hpp`、`plugin.cpp`、`tests/burnout_shot_test.cpp`、诊断面板

## 1. 数据流

```text
UpdateBurnoutProbe()            20 Hz，反射读取
  BurnoutSignals                紧凑、只读的事实
        |
        v
UpdateBurnoutDetection()        Game tick 策略
  BurnoutDetectionState          地面/速度门、0.28 s 保持、强度斜坡
        |
        v
CarSample::burnout              随 SampleRing 发布
        |
        v
BlendCarSample()                render 侧插值：布尔取新样本，连续值 lerp
```

Phase 2 不改变相机路径。`UpdateCamera()` 仍然只把 `sample.burnout` 写进 sample；
`AdvancePresenter()` 尚未读取它，留给 Phase 3。

## 2. `BurnoutSignals`

`UpdateBurnoutProbe()` 在完成原有 12 项反射查询后，把结果复制到一个稳定的小结构：

| 字段 | 来源 |
|---|---|
| `direct` | `GetVehicleIsBurnout` |
| `handbrake` | `GetInputInfo` + offset 12 |
| `throttle` | `HTVehicleInputInfo::Throttle` @0 |
| `player_throttle` | 玩家输入层油门（如果可用） |
| `brake` / `steering` | `HTVehicleInputInfo` @8 / @4 |
| `left` / `right` | `GetWheelIsBurnout(2)` / `(3)` |

说明：

- Phase 0 已确认后轮 index 为 2/3，当前车两轮会同时为 true；左右映射没有额外断言。
- 两个后轮标志一致时，Phase 3 的 shot 用 steering 符号选边，默认右后。
- 无载具、载具对象变化、反射未就绪时，signals 与 detection 一起清空。

## 3. 检测策略

`UpdateBurnoutDetection()` 是 Game tick 唯一写策略的地方：

| 参数 | 值 | 说明 |
|---|---:|---|
| 进入速度 | 1.0 m/s | `abs(speed) < 1.0` 才算静止 |
| 保持速度 | 3.0 m/s | 超过则强制退出 |
| 短保持 | 0.28 s | 桥接 20 Hz 轮询 |
| 强度上升 | 12 /s | 向 throttle 目标逼近 |
| 强度回落 | 8 /s | 退出时平滑 |
| 回退油门阈值 | 0.15 | 手刹 + 油门足够时，即使 `direct` 不可用也可触发 |

条件：

```text
direct   = GetVehicleIsBurnout == true
fallback = handbrake && throttle > 0.15
safe     = on_ground && forward_speed > -0.5 && finite(speed)
active   = safe && (direct || fallback) && abs(speed) < entry
           或 active 延续 && abs(speed) < hold
           或 0.28 s 短保持
```

输出 `BurnoutSample`：

- `active`：最终检测；
- `left` / `right`：后轮烧胎标志；
- `intensity`：0..1 平滑强度；
- `throttle` / `handbrake` / `steering` / `speed_mps`：诊断与后续震动/侧别使用。

状态变化时打印：

```text
nfs2015 chase cam: burnout sample active=... direct=... fallback=...
left=... right=... intensity=... throttle=... HB=... steer=... speed=...
```

## 4. `CarSample` / `SampleRing`

`CarSample` 新增 `BurnoutSample burnout{}`。

`BlendCarSample()` 的规则：

- `active` / `left` / `right` **取较新样本**，不插值；
- `intensity` / `throttle` / `handbrake` / `steering` / `speed_mps` 线性插值；
- 这样 render 侧不会出现“半个烧胎”的布尔值，也不会因为 20 Hz 采样与 render 时钟不同步而跳变。

## 5. 诊断面板

新增两行：

```text
烧胎检测（已知/直接/回退/激活/保持）
  known 1  direct 0  fallback 1  active 1  hold 0.21s
烧胎样本（左/右/强度/油门/手刹/速度）
  L 1  R 1  int 0.84  thr 0.82  HB 1  spd +0.00
```

原有 Phase 0 的 12 项 probe 行保留。`zh-CN` 文案已同步。

## 6. 验证

构建：

```text
./build-mingw.sh --sdk ../Download/Anomaly-2.7.0-sdk --run-tests
plugin.dll 3.0M，PE32+ x86-64
```

单测：

```text
ctest --test-dir build.mingw --output-on-failure
1/2 nfs2015_chase_cam_model ... Passed
2/2 nfs2015_burnout_shot_model ... Passed
100% tests passed, 0 tests failed out of 2
```

新增 `TestCarSampleBurnoutBlend` 覆盖：

- 布尔值取较新样本；
- intensity/throttle/handbrake/steering/speed 的中点插值；
- 与既有 chase 模型测试一起通过。

## 7. 实机状态与限制

- 真实 probe 日志已验证：`12/12` 函数解析、`HB12` 手刹、`wheels=0c`、后轮 angular 1718.9。
- 当前 PowerShell 为 medium integrity，游戏为 high integrity；普通 `SendInput` / `keybd_event` 受 UIPI 限制，测试注入不可靠。
- 因此本阶段**没有抓到 `burnout sample active=1` 的实机日志**。管线本身已编译、单测和诊断面板验证；`active=1` 需要一次手动 Shift+W（或 Phase 3 安装后由用户手动触发）确认。
- 左右后轮映射沿用 index 2/3；如果将来发现左右反了，只需改 `BurnoutSignals` 的 `left/right` 赋值，不影响 shot 状态机。
- 强度目标使用油门（有油门时），没有油门时 `direct` 目标为 1；这比固定强度更接近 NFS 的“油门越大越抖”。

## 8. 结论

Phase 2 的检测与 sample 管线已完成，并通过构建、CTest 和诊断面板检查。相机仍未消费 burnout 数据，符合本阶段范围。下一阶段在 `AdvancePresenter` 中把 `chase_view` 与 `BurnoutShot` 混合，并确保 route B / presenter pose / lens 使用同一个混合 view。
