# NFS 2015 Chase Cam 2.1.0 发布检查

- 日期：2026-11-26
- 版本：2.1.0
- 分支：`feat/game-thread-camera`
- 宿主：Anomaly 3.1.1 Runtime，NTE
- 结论：**发布候选通过离线门禁、宿主工具和实机矩阵；帧生成开/关一项受本机硬件限制未执行。**

## 1. 产物

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `build.mingw/package/robineatcorn.nfs2015-chase-cam/plugin.dll` | 6,615,384 | `6ee0cc0f14aedef1b06ed953003e9f9a6e99e99aa713bf7ab7292289ea232577` |
| `dist/Nfs2015ChaseCam-2.1.0.zip` | 1,978,794 | `bd6850a69c769edac212d2440a64060e880b7097feabdf7c62f1e82bfc22966a` |

ZIP 根目录：

```text
manifest.json
plugin.dll
locales/zh-CN.json
```

## 2. 离线门禁

| 项目 | 状态 | 结果 |
|---|---|---|
| MinGW-w64 + Anomaly 3.1.1 SDK 构建 | PASS | `build.mingw` |
| `ctest` | PASS | 9/9 |
| `anomaly-plugin validate` | PASS | `valid id=robineatcorn.nfs2015-chase-cam version=2.1.0 entry=plugin.dll` |
| `anomaly-test-host` | PASS | `reloads=10 ticks=660 windows=11 text=187` |
| manifest / ID / 版本 / pluginmaster | PASS | `verify-release.sh` |
| 包内容 / ZIP 根目录 / 字节一致 | PASS | `verify-release.sh` |
| DLL 导入/导出 | PASS | 仅 `KERNEL32.dll`、`msvcrt.dll`；导出 `AnomalyPluginEntryV1` |
| 离线发布门禁 | PASS | `verify-release.sh --allow-missing-host-tools`，0 failed |

## 3. 实机矩阵（Anomaly 3.1.1）

| 场景 | 状态 | 检查点 |
|---|---|---|
| 插件加载、profile 门禁 | PASS | `08a3feb998c74f99375cebd6bd8866e409663680b9fe3da229274f4c9c2e249a` |
| 直行 / 加速 / 转弯 | PASS | 车辆不抖，无相机-车距离振荡 |
| 漂移 | PASS | 偏航弹簧、侧倾、推近正常 |
| 停车环绕 / HUD 隐藏 | PASS | 进入、退出、HUD 恢复 |
| 上下车 | PASS | 下车回游戏相机，上车恢复追车 |
| Z 三档循环 | PASS | 追车 / 第三人称 / 第一人称 |
| 重置 / 切预设 | PASS | 无旧姿态残留 |
| 传送 | PASS | cut 生效，无黑屏/卡死 |
| 拍照模式 | PASS | 不接管，退出后恢复 |
| 过场 | PASS | 不接管，结束后恢复 |
| HUD 世界标记（相机距离变化） | PASS | 用户目视未发现错位 |
| fail-open（GetViewPoint pattern 缺失） | PASS | 两个钩子均不安装，回游戏原相机 |
| 车辆变换 1/2/4 帧失败注入 | PASS | 外推 2 帧；4 帧时 timeout/cut/pose 作废各 1 次，随后恢复 |
| 帧生成开 / 关 | BLOCKED | 本机 RTX 3080 不支持帧生成 |
| 10 分钟 FSR3 连续驾驶 | BLOCKED | 同上 |

## 4. E4b 数据

300 帧高速直行，速度范围 50–275 km/h，因此原始偏移方差包含有意的速度相关机位变化；
高频残差和相邻帧偏移变化比用于判断抖动：

```text
               hp_sd    dmean   dp95   ratio_mean  ratio_p95
along_cm        6.12     2.97   8.81      0.035      0.120
lateral_cm      8.55     3.21   8.74      0.038      0.122
vertical_cm     3.57     1.28   8.22      0.012      0.063
```

相邻帧偏移变化只有车辆位移的约 1%–4%（均值），95% 分位约 6%–12%；没有一帧滞后。

## 5. 发布步骤

1. 提交 `feat/game-thread-camera`，推送分支。
2. 打 tag `v2.1.0`，上传 `dist/Nfs2015ChaseCam-2.1.0.zip`。
3. Release 说明使用 `RELEASE-NOTES-2.1.0.md`。
4. 合并到 `main` 并推送，使 `pluginmaster.json` 指向已存在的 v2.1.0 资源。

## 6. 回滚

- 关闭游戏和 Anomaly；
- 删除或改名 `Anomaly/plugins/robineatcorn.nfs2015-chase-cam`；
- 重新安装 2.0.6 或禁用插件，游戏使用原相机；
- 不修改 profile 门禁来绕过问题。
