# 起始点（2026-10-05）

## 本仓库

- 分支 `feat/full_vr` 从 `main` 的 `d4c7bdd`（"feat: AstrBot platform for a VRChat desktop client"）开出。
- 当时的形态：VRChat **桌面模式**（`--no-vr`）。Python bridge 负责的几何部分：
  - 画面来自 X11 截图；
  - 深度用单目 Depth Anything，配合地面拟合和启发式尺度；
  - 转向靠鼠标位移，标定值 3600 px/圈、13.1 px/度；
  - 位置靠 OSCQuery `VelocityX/Z` 积分；
  - 地图是 0.25 m 栅格 + A*；
  - 跟随用 OCR 读名牌，再用卡尔曼滤波平滑轨迹。
- 桌面模式反复出现的问题：
  - 单目深度尺度靠猜，会把沙发看成斜坡；
  - 俯仰慢慢漂移，只好加了 `vrchat_camera_y`；
  - 鼠标转向需要标定；
  - 四向截图拼全景慢；
  - 第三人称还要额外识别自己的身体。

## 上游项目

| 项目 | 版本 | 用途 |
|---|---|---|
| Monado | `cfa6078b8d7f368fa5296f540a436d59833e496c`（main，25.1.0） | OpenXR 运行时：remote 驱动 + null 合成器 |
| xrizer | `0989a7fac2d1efb7ea82f5fe1a8ed30c3eeb9596`（main，2026-09-03） | OpenVR → OpenXR |
| OpenXR loader | Arch `openxr` 1.1.60 | xrizer 加载 Monado |
| Proton | Proton - Experimental（Steam 自带） | 运行 VRChat |

## 服务器（脱敏）

- Arch Linux，RTX 3060 12GB，NVIDIA 开源内核模块 615.71。没有显示器：Xorg 用伪造的 EDID 输出 1280x720。
- bot 用户的 Steam 和 VRChat 跑在独立的网络命名空间里，游戏画面用 Sunshine 远程查看。
- AstrBot 上 `vrchat` 平台处于启用状态，插件和 bridge 正在运行（桌面模式）。
- VRChat 当时的启动参数：`DXVK_FRAME_RATE=20 %command% --no-vr --fps=20 -screen-width 1280 -screen-height 720 -screen-fullscreen 1`。
- VRChat 未运行时 GPU 占用 713 MiB（Sunshine 104 MiB，推理 worker 450 MiB）。
- 系统里没有任何 VR 运行时（SteamVR、Monado 都没有），也没有 cmake/ninja/eigen/vulkan-headers。

## 选型依据

外部候选方案文档（ALVR + cuVSLAM + VPI SGM + nvblox + Nav2）的评估结论是：方向对，但太重。先用最小代价验证"Linux 上无头显的 VRChat VR 模式 + 程序控制头部 + 拿到双目帧"这条主干，再按需加组件。详见 [decisions.md](decisions.md)。
