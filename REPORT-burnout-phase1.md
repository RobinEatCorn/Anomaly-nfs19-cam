# Phase 1 报告：纯烧胎特写相机模型

- 日期：2026-11-10
- 状态：**PASS**
- 范围：只实现纯头文件相机模型和单测，不接插件、不读取游戏、不写 route B
- 产物：`burnout_shot.hpp`、`BurnoutSample`、`tests/burnout_shot_test.cpp`

## 1. API

```cpp
BurnoutShot shot(config);
BurnoutShotResult result =
    shot.Update(dt, car, burnout, aspect_ratio);
CameraState view =
    BlendBurnoutCamera(chase_view, result.view, car, result.blend);
```

- `BurnoutShot::Update()` 跑状态机，返回：
  - `view`：纯特写 pose；
  - `phase`：线性 0..1；
  - `blend`：eased 0..1；
  - `side`：`+1` 右后、`-1` 左后、`0` 居中；
  - `latched` / `force_exit`：诊断用。
- `BlendBurnoutCamera()` 是纯函数；`blend=0` 原样返回 chase，`blend=1` 原样返回 shot，中间走 orbit。
- `BurnoutShot::Reset()` 用于 epoch/world/teleport；`SetConfig()` 不重置进行中的 shot，避免改设置时硬切。

## 2. 状态机

| 参数 | 默认 | 行为 |
|---|---:|---|
| `entry_delay` | 0.5 s | 条件持续满 0.5 s 才 latch |
| `entry_time` | 1.0 s | phase 从 0 到 1 |
| `exit_delay` | 0.2 s | 条件消失后先保持 |
| `exit_time` | 2.5 s | phase 从 1 到 0 |
| `fast_exit_time` | 0.5 s | 超速/腾空/倒车立即退出 |
| `entry_speed_mps` | 1.0 | 只限制进入 |
| `hold_speed_mps` | 3.0 | 保持上限，超过走 fast exit |

- 进入前条件短暂丢失会清零 `hold_time`，所以轻点不会触发。
- 退出中再次触发：`phase` 从当前值回升，`latched` 和侧别保留，不重置。
- `phase` 到 0 才清 `latched`，下一次烧胎才允许重新选边。
- 侧别选择优先级：强制模式 → 单侧 per-wheel flag → steering 符号 → 默认右后。
- 进入后侧别 latch；烧胎中打方向不会换镜头。

## 3. 机位、FOV、震动

- 车体局部机位：`(-2.2, side * 1.9, 0.4)`。
- 目标点：`(-0.9, side * 0.75, 0.10)`。
- 默认 `slope_follow=true`，使用 `MakeCarBasis(yaw, pitch)`；关闭时 yaw-only。
- 水平 FOV 45°，按 viewport aspect 转垂直 FOV；16:9 约 26.25°。
- 震动：
  - `gate = SmoothStep(0.55, 1.0, blend)`，先拉近再抖；
  - 位置 `0.010 m * intensity * gate * shake_scale`，上限 0.03 m；
  - 旋转 `0.20° * intensity * gate * shake_scale`，上限 0.5°；
  - 多频 `Noise1`，约 12–20 Hz 分量，确定性。
- 安全：
  - eye Z 不低于 `car.position.z + 0.25 m`，且震动后再钳一次；
  - 目标距离小于 0.5 m 时回退到车体 forward，最终 target 固定为 `eye + forward`；
  - `camera_scale` 对 back/outboard 应用，并加 1.3 m 安全下限；
  - 输出全部经过有限值检查/回退。

## 4. Orbit blend

`BlendBurnoutCamera()`：

1. 以 `car.position + car.up * 0.8 m` 为 pivot；
2. 把 chase eye / shot eye 转成车体局部柱坐标 `(r, theta, z)`；
3. 对 `r`、`z` 线性插值，对 `theta` 走最短弧；
4. 转回世界；
5. 视线方向用 slerp，up 重新正交化；
6. FOV / roll 线性插值。

这样不是穿过车尾的直线推镜，而是绕后翼子板的弧线。`blend=0/1` 走 early return，保证与输入 pose 完全一致。

## 5. 单测

`tests/burnout_shot_test.cpp` + CTest 目标 `nfs2015_burnout_shot_model`，覆盖：

1. 0.5 s 前 phase 为 0；
2. 进入后 phase / blend 单调升到 1；
3. 退出先保持 0.2 s，再单调降到 0；
4. 退出中重新触发，phase 从当前值回升且无跳变；
5. 侧别 latch，退出后下一次可重新选边；
6. 超过 hold speed 走 0.5 s fast exit；
7. blend=0/1 与 chase/shot 完全一致，中间 orbit 不是直线；
8. 45° 水平 FOV 在 16:9 约 26.25° 垂直，并能反算回 45°；
9. 多 yaw/pitch/side/scale 组合输出有限，target 距离 > 0.5 m；
10. 上坡/下坡 eye Z 不低于车体原点 + 0.25 m；
11. 非法 aspect ratio 回退到 16:9。

验证：

```text
ctest --test-dir build.mingw -R burnout --output-on-failure
100% tests passed, 0 tests failed out of 1
```

另外用 `g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror` 单独编译测试通过。

## 6. 已知限制（留给后续 phase）

- 未接 `CarSample` / `SampleRing`，未在 `UpdateCamera` 填 burnout（Phase 2）。
- 未在 `AdvancePresenter` 混合 chase/shot，也未写 route B（Phase 3）。
- 未加设置、UI、本地化（Phase 4）。
- 没有 wall raycast；只有 eye Z 下限和最小半径，贴墙穿模仍是已知风险。
- 未接入 NTE 实机；Phase 2/3 后需要实机矩阵验证。

## 7. 结论

Phase 1 **PASS**。模型接口、状态机、orbit、FOV、震动和安全约束都已可单测，可以进入 Phase 2 的检测与 sample 管线。
