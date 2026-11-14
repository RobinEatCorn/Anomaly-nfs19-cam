# NFS 2015 Chase Cam 2.0.5

适用：Anomaly 2.7.0 / 3.0.0 Runtime，NTE。

## 变更

- **相机模型的车头朝向来源改为渲染车辆的 proxy 朝向**，不再优先使用游戏相机 yaw。
  - `SelectCarYaw` 优先读取 `render_x.transform_forward`，用 `atan2(y, x)` 得到车体
    heading；沿用 route B 的 seqlock / proxy 匹配 / 方向有效性检查，并要求快照不超过
    50 ms。
  - 车体朝向有效时直接作为 `car.yaw` 返回，不再套 80° 相机保护；90° 漂移不会在 80°
    边界再翻到速度方向。
  - 车体朝向缺失、过期，或 forward 接近垂直时，完整回退到原来的游戏相机 yaw、80°、
    倒车和低速逻辑。
  - 倒车：车体朝向本身就是车头，exact speed 仍然照旧设置 `reversing`，轮子和显示逻辑
    不受影响；没有 exact speed 时用 velocity 与 body yaw 的夹角推断倒车。
- **诊断**：新增 `kYawSourceBody`；`yaw source switches` 可以看到 body / camera 之间的
  切换。
- 版本号同步到 2.0.5（`manifest.json`、`plugin.cpp`、`pluginmaster.json`、ZIP 文件名）。

## 没有改的

- 没有改任何弹簧、预设、look-ahead、FOV、roll 或其它镜头参数。
- 因此 GamePreset 自身的机位滞后仍在：60°/s 的稳态弯里，模型本身约落后 27°；这次只
  去掉相机源滞后和假打滑。
- `presenter.cpp` 仍在 `forward_speed < 0` 时把 `slip_angle` 清零；超过 90° 的横滑不会
  触发 drift 推近/下压/侧倾，这是下一层。

## 测试

- MinGW 构建通过，无 warning。
- `ctest` 6/6 通过。
- `verify-release.sh` 离线检查通过（manifest / package / ZIP / DLL 导入导出；宿主工具
  按环境标记为 blocked）。
- 实机待验收：正常走线转弯镜头跟随车头；90° 漂移不再 80° 翻向；`slip pos deg` 不再
  包含相机滞后；`yaw source switches` 正常不涨。

## 注意

- 如果 proxy 快照频繁过期，会回退到相机 yaw；此时行为与 2.0.4 相同，诊断里的
  `yaw source switches` 会增加。
