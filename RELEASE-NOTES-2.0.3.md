# NFS 2015 Chase Cam 2.0.3

适用：Anomaly 2.7.0+ Runtime，NTE。

## 变更

- 新增 **“NFS 漂移前轮反打”**：漂移时前轮在视觉上指向车辆实际运动方向，形成 NFS 2015
  风格的 countersteer；不修改物理、输入、底盘或车辆移动组件。
- 目标角来自速度方向与车身前向的夹角，并受独立的视觉最大角和角速度限制：
  - `wheelSteerMaxDegrees`：默认 `40°`，范围 `20–50°`；
  - `wheelSteerGain`：默认 `1.0`，范围 `0.5–1.5`；
  - `wheelSteerFullSlipDegrees`：默认 `25°`，范围 `10–60°`；
  - `wheelSteerRateDegreesPerSecond`：默认 `360°/s`，范围 `60–1200°/s`。
- 低速（< 3 m/s）、倒车、低侧滑角时保持原厂转向；进入阈值默认为全角滑移角的 32%
  （25° 全角时为 8°），并有回差，避免普通转弯频繁覆盖。
- 漂移期间暂时停用游戏的车轮动画控制器，由插件写入四个车轮的 `RelativeRotation`：
  - 前轮：车辆 up 轴上的反打角 + 绕轮轴的视觉滚动；
  - 后轮：绕轮轴的视觉滚动；
  - 退出漂移、离开载具、关闭功能、切换载具或停止插件时，恢复首次写入前捕获的
    `RelativeRotation` 和动画开关。
- 修复反打角在滚动相位下“崴脚”/万向轴的问题：
  - 使用 UE `FQuat::Rotator()` 作为 `FRotator` 反解，保证写入前后四元数一致；
  - 从冻结的轮座中移除轮子局部 Y 轴的滚动 twist，避免绕倾斜轴转向；
  - 视觉旋转按 `steer * roll` 组合，滚动不会改变已经转向后的轮轴。
- 窗口新增启用开关和四个滑条；“前轮转向”状态行显示实际角 / 目标角、轮数和
  `active/stock`。
- 新增离线几何测试 `nfs2015_wheel_steer_model`，覆盖目标角、最大角限制、低侧滑不激活、
  轮座旋转轴、滚动组合以及 `FRotator` 往返。
- 设置向后兼容：旧配置缺少新字段时使用上述默认值。

## 验证

```sh
./build-mingw.sh --sdk <Anomaly-2.7.0-sdk>
ctest --test-dir build.mingw --output-on-failure
anomaly-plugin validate build.mingw/package/robineatcorn.nfs2015-chase-cam
anomaly-test-host --plugin build.mingw/package/robineatcorn.nfs2015-chase-cam --reload 2 --ticks 10
```

实机日志在首次解析到车轮几何时输出：

```text
nfs2015 chase cam: wheel steering geometry ready (front=2 rear=2 mesh=... anim=...)
```

首次真正写入反打角时输出：

```text
nfs2015 chase cam: visual wheel override active (...)
```

发布包根目录：

```text
manifest.json
plugin.dll
locales/zh-CN.json
```
