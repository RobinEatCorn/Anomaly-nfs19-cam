# NFS 2015 Chase Cam 2.0.3

适用：Anomaly 2.7.0+ Runtime，NTE。

## 变更

- 前轮反打改为 NFS 2015 式 drift 状态机，修复实机反馈：
  - **低速起漂/原地掉头不再反打**：进入门槛为 **40 km/h（约 11.1 m/s）**，低于该速度
    保持原厂转向；进入后**不再有维持速度下限**，大漂移角导致前向速度接近 0 时仍继续反打。
  - **高速 U 型弯先向弯内**：新增 `turn_in`，方向盘仍指向弯内时先保持入弯角约 `0.18s`，
    再过渡到反打；已经在反打时跳过。
  - **下坡 U 型弯不再闪断**：速度只用于进入；退出由后轴滑移角（3° 持续 0.25s）决定，
    真正退出后 0.5s 内不再重新进入。倒车只阻止进入，不会因为大角度漂移的前向速度为负
    而释放已经进入的反打。
  - **反打角不再偏大**：目标角改用前轴中心速度，默认最大角 `32°`，全滑移角 `40°`。
- 新增 drift 状态：`grip → turn_in → entering → drifting → releasing → grip`。
  - 进入/退出滑移角默认 `6.5°` / `3°`，使用后轴滑移；
  - 进入后最短保持 `0.5s`，退出条件需持续 `0.25s`；
  - 进入 blend `0.2s`，退出 blend `0.3s`，反打速率 `120°/s`。
- 反打目标随滑移角连续缩放：
  - `wheelSteerFullSlipDegrees` 默认 `40°`；
  - `wheelSteerMaxDegrees` 默认 `32°`；
  - `wheelSteerRateDegreesPerSecond` 默认 `120°/s`；
  - 只有反打目标达到 `5°` authority 才停用原厂动画，普通高速转弯不会因为越过滑移
    阈值而冻结车轮。
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
| `wheelSteerMaxDegrees` | `32` | 20–50° |
| `wheelSteerGain` | `1.0` | 0.5–1.5 |
| `wheelSteerFullSlipDegrees` | `40` | 10–60° |
| `wheelSteerRateDegreesPerSecond` | `120` | 60–1200°/s |

内部常量：

```text
起漂速度（仅进入） 40 km/h / 11.1 m/s
进入滑移角        6.5°
退出滑移角        3.0°
最短保持          0.5 s
退出确认          0.25 s
进入 blend        0.20 s
退出 blend        0.30 s
入弯保持          0.18 s（最少 0.10 / 最多 0.30）
再进入冷却        0.5 s
原厂前轮最大角    35°
反打接管门槛      5° counter 目标
恢复容差          1.5°
```

旧配置缺少新字段时继续使用默认值。旧版三项默认值（`wheelSteerMaxDegrees=40`、
`wheelSteerFullSlipDegrees=25`、`wheelSteerRateDegreesPerSecond=360`）如果同时存在，
加载时会迁移为 `32` / `40` / `120`；只改过其中一项的配置保留用户保存值。

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
- 入弯阶段先保持向弯内，再向反打过渡；高滑移也至少保持 0.10s；
- 进入目标从原厂角开始，不从 0° 开始；
- 25° 滑移为部分反打，60° 才到 40° 上限；
- 低速、前向速度接近 0 均不退出已进入的 drift；
- 真正静止 0.5s 后释放；释放保留最后一次反打目标，再平滑回到原厂角；
- 前轴 `omega × r` 修正与前后轴滑移差；
- 零反打目标保留原厂角，不把方向盘回正；
- 前轮世界朝向到原厂角测量，包括镜像 180° 轮座。

实机日志新增状态转换：

```text
nfs2015 chase cam: wheel drift grip -> turn_in speed=... steer=... front=... rear=... yaw=...
nfs2015 chase cam: wheel drift turn_in -> entering ...
nfs2015 chase cam: wheel drift drifting -> releasing ...
nfs2015 chase cam: visual wheel override active (... phase=... stock=... counter=... front_slip=... rear_slip=...)
```

## 验证建议

1. 低速原地掉头/低速手刹掉头：前轮往弯内，不反打。
2. 40 km/h 以下正常转弯：不接管原厂动画。
3. 高速手刹 U 型弯：前轮先向弯内约 0.1–0.2s，再平滑反打。
4. 下坡 U 型弯速度下降、前向速度接近 0：反打不消失；速度恢复后继续。
5. 大角度漂移：反打角不超过默认 32°。
6. 漂移中松油/回正：反打逐渐收回。
7. 漂移退出时继续打方向：前轮平滑交回原厂角，没有一跳。
8. 连续快速进出漂移：0.5s 锁定 + 0.5s 再进入冷却，不闪烁。
9. 离开载具/关闭功能：原厂 `RelativeRotation` 和动画开关恢复。
