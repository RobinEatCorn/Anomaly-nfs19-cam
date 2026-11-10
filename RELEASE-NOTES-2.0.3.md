# NFS 2015 Chase Cam 2.0.3

适用：Anomaly 2.7.0+ Runtime，NTE。

## 变更

- 前轮反打改为 NFS 2015 式 drift 状态机，修复两个观感问题：
  - **低速起漂/原地掉头不再反打**：进入门槛改为 NFS 2015 的 **30 mph（约 48 km/h /
    13.4 m/s）**，低于该速度时保持原厂转向，前轮往弯内转。
  - **漂移/非漂移切换不再跳角**：进入前读取原厂前轮实际角度，进入/退出使用连续 blend，
    退出时手动角收敛到原厂角附近才恢复游戏车轮动画和 `RelativeRotation`。
- 新增 drift 状态：`grip → entering → drifting → releasing → grip`。
  - 进入滑移角默认 `6.5°`，退出 `3°`；
  - 进入后最短保持 `0.5s`，退出条件需持续 `0.25s`；
  - 进入 blend `0.2s`，退出 blend `0.3s`。
- 反打目标随滑移角连续缩放：
  - `wheelSteerFullSlipDegrees` 默认从 `25°` 改为 `35°`；
  - `wheelSteerRateDegreesPerSecond` 默认从 `360°/s` 改为 `120°/s`；
  - 低速/低滑移时反打自然很小，不再刚过阈值就接近满舵；
  - 只有反打目标达到一定幅度（6° authority）才停用原厂动画，普通高速转弯不会因为
    越过滑移阈值而冻结车轮。
- 原厂姿态交接：
  - 首次进入漂移时，在停用车轮动画前读取两个前轮的世界朝向，取平均作为实际原厂角；
  - 退出时以 `GetRawSteeringInput × 35°` 估算原厂角；
  - 手动角与原厂角差小于 `1.5°` 后才恢复动画，避免退出时一帧跳回。
- 保留滚动修复：
  - UE `FQuat::Rotator()` 反解；
  - 冻结轮座移除轮子局部 Y 轴滚动 twist；
  - `relative = steer * roll`，滚动不改变已经转向后的轮轴。
- 只改视觉，不修改车辆物理、输入、底盘或移动组件；所有退出路径仍恢复原厂姿态和动画开关。

## 参数

窗口仍只暴露四个滑条：

| 字段 | 默认 | 范围 |
|---|---:|---:|
| `wheelSteerMaxDegrees` | `40` | 20–50° |
| `wheelSteerGain` | `1.0` | 0.5–1.5 |
| `wheelSteerFullSlipDegrees` | `35` | 10–60° |
| `wheelSteerRateDegreesPerSecond` | `120` | 60–1200°/s |

内部常量：

```text
最低起漂速度      30 mph / 13.4 m/s
进入滑移角        6.5°
退出滑移角        3.0°
最短保持          0.5 s
退出确认          0.25 s
进入 blend        0.20 s
退出 blend        0.30 s
原厂前轮最大角    35°
反打接管门槛      6° counter 目标
恢复容差          1.5°
```

旧配置缺少新字段时继续使用默认值；已有的 `wheelSteerFullSlipDegrees` /
`wheelSteerRateDegreesPerSecond` 保存值不会被强制覆盖。

## 测试

```sh
./build-mingw.sh --sdk <Anomaly-2.7.0-sdk>
ctest --test-dir build.mingw --output-on-failure
anomaly-plugin validate build.mingw/package/robineatcorn.nfs2015-chase-cam
anomaly-test-host --plugin build.mingw/package/robineatcorn.nfs2015-chase-cam --reload 2 --ticks 10
```

`nfs2015_wheel_steer_model` 新增覆盖：

- 低速 30° 滑移不激活；
- 6.5° 进入 / 3° 退出；
- 0.5s 漂移锁定与 4° 中间区保持；
- 进入目标从原厂角开始，不从 0° 开始；
- 25° 滑移为部分反打，60° 才到 40° 上限；
- 低速释放保留最后一次反打目标，再平滑回到原厂角；
- 零反打目标保留原厂角，不把方向盘回正；
- 前轮世界朝向到原厂角测量，包括镜像 180° 轮座。

实机日志新增状态转换：

```text
nfs2015 chase cam: wheel drift grip -> entering speed=... stock=... counter=... blend=...
nfs2015 chase cam: wheel drift entering -> releasing ...
nfs2015 chase cam: visual wheel override active (... phase=... stock=... counter=...)
```

## 验证建议

1. 低速原地掉头/低速手刹掉头：前轮往弯内，不反打。
2. 30 mph 以下正常转弯：不接管原厂动画。
3. 高速起漂：前轮从原厂角平滑过渡到反打，没有先回正再反打。
4. 漂移中松油/回正：反打逐渐收回。
5. 漂移退出时继续打方向：前轮平滑交回原厂角，没有一跳。
6. 连续快速进出漂移：0.5s 锁定生效，不闪烁。
7. 离开载具/关闭功能：原厂 `RelativeRotation` 和动画开关恢复。
