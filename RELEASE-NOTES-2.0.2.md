# NFS 2015 Chase Cam 2.0.2

适用：Anomaly 2.7.0+ Runtime，NTE。

## 变更

- 新增 **“镜头弹簧”** 面板分组，所有滑块只在 **“NFS 2015（实机数据）”** 预设下显示：
  - 镜头位置弹簧：`0.5 – 30`，默认 `7.5`；
  - 正常偏航弹簧：`0.5 – 30`，默认 `5.4`；
  - 漂移偏航弹簧（镜头位置）：`0.5 – 30`，默认 `2.7`；
  - 漂移看向弹簧（越大越偏一侧）：`0.5 – 30`，默认 **`15`**；
  - 漂移回正偏航弹簧：`0.5 – 30`，默认 `4.5`；
  - 俯仰弹簧：`1 – 120`，默认 `33`；
  - 侧倾弹簧：`1 – 200`，默认 `60`。
- 角度类弹簧的最大值从 `10` 提高到 `30`；俯仰和侧倾的上限分别为 `120` 和 `200`。
- **漂移看向弹簧的默认值从 `2.7` 改为 `15`**。原版漂移等效值约为 `2.7`；`15` 是当前模型下能让车留在画面一侧、同时不完全丢失道路的构图默认值。
- 面板新增 **“恢复默认值”** 按钮：按下后启用状态、预设、锚点来源、机位、FOV、所有弹簧、坡度、侧倾/俯仰/震动和诊断开关都会恢复默认，并重新稳定相机。
- **“仅在漂移时倾斜”默认开启**。普通转弯不再让地平线跟着侧倾，只有漂移时才有侧倾；旧配置里保存的开关状态会保留，按“恢复默认值”会回到开启。
- **锚点来源默认改为“车锚定”**（VehicleMesh ComponentToWorld）。旧配置里已经保存的锚点来源会保留；按“恢复默认值”会回到车锚定。
- 新增设置项均向后兼容：旧配置文件缺少这些字段时使用默认值；已有的 `lookYawSpring` 用户值会保留。

## 验证

```sh
./build-mingw.sh --sdk <Anomaly-2.7.0-sdk> --run-tests
ctest --test-dir build.mingw --output-on-failure
anomaly-plugin validate build.mingw/package/robineatcorn.nfs2015-chase-cam
anomaly-test-host --plugin build.mingw/package/robineatcorn.nfs2015-chase-cam --reload 2 --ticks 10
```

发布包根目录：

```text
manifest.json
plugin.dll
locales/zh-CN.json
```
