# 全 VR 模式（feat/full_vr）

目标：让 bot 的 VRChat 以 **VR 模式** 运行在一个不存在的头显上。头部、双眼、手柄全部由程序设定；每一帧拿到渲染好的左右眼图像，以及渲染时用的精确位姿和视场角。以此替换桌面模式下"截图 + 单目深度 + 鼠标转向 + 速度积分"这一套几何层。

| 文档 | 内容 |
|---|---|
| [starting-point.md](starting-point.md) | 起始点：本仓库、上游项目、服务器各自在什么版本和状态 |
| [changes.md](changes.md) | 修改点：本仓库、其他项目（补丁）、服务器上改了什么 |
| [decisions.md](decisions.md) | 决策：选了什么、没选什么、为什么 |
| [deploy-log.md](deploy-log.md) | 部署记录（脱敏）：按时间顺序 |
| [fbt-research.md](fbt-research.md) | 全身动作接管调研：OSC 追踪器、全身追踪校准 |
| [motion.md](motion.md) | 动作系统：片段库、步态、姿势、跟随里的身体和头 |
| [agent-vr-use.md](agent-vr-use.md) | 代理怎样操作 VRChat 的 VR 菜单（像素到手柄射线、自动校准、全景的调试接口、用户相机和绕着 bot 转的镜头） |
| [avatar-position-beacon.md](avatar-position-beacon.md) | 化身位置角标：着色器把世界坐标画进 bot 自己眼睛的角落，bridge 读出来 |
| [avatar-panorama.md](avatar-panorama.md) | 化身全景：6 台本地相机，左眼彩色立方体全景、右眼逐像素对齐的 E1C 深度 |
| [speaker.md](speaker.md) | 谁在说话：双耳声音的方向 + 名牌发光；转向说话的人；校准步骤 |

## 链路

```text
VRChat.exe (Proton, D3D11 -> DXVK)
   │  OpenVR
Proton vrclient ──> xrizer (OpenVR on OpenXR, 打了补丁)
   │  OpenXR (XR_RUNTIME_JSON)
monado-service
   ├─ remote 驱动 <── TCP :4242 ── vrc-vr::remote   (设定头、眼、手柄)
   └─ null 合成器 ──> /dev/shm/vrc-eyes ──> vrc-vr::tap (左右眼 + 位姿 + 视场角)
移动 / 转向：OSC /input/*（和桌面模式一样，VR 模式下照样有效）
```

## 现状

2026-10-05 已在服务器上实测通过：

- VRChat 用虚拟头显进入 VR 模式。没有 SteamVR，也没有显示器（null 合成器，现在跑 30 fps）。
- 通过 remote 驱动转头、低头，渲染出的画面随之改变，抓到的每帧位姿与设定一致。
- 抓帧：每只眼 1280x1280，`R8G8B8A8_SRGB`，视场 100°x100°，像素为正方形（fx = fy = 537.0，cx = cy = 640），没有遮罩，基线 0.063 m；抓帧频率可设，现在每帧都抓。
- OSC 的 `/input/Vertical`、`/input/LookHorizontal` 在 VR 模式下仍然可以移动和转向。
- 显存：monado-service 84 MiB，VRChat 约 1.2 GB（VRChat Home，房间里只有自己），整卡约 2.1 GB。
- `vr-probe sweep` 端到端可用：转头、等到按新姿态渲染出的帧、存 PNG。

2026-10-09：化身的全景（6 台相机，E1C 深度）默认开（`--pano auto`），环视、跟随、目击、说话人视觉、截图都从全景读；名字来自用户相机的镜头（默认静止朝前）和 VRChat 画在眼睛上的名牌，深度给距离（D36）。跟随：只按位置接着认有期限、由镜头确认（D40）；跟丢 1 s 后镜头快拍 6 个固定方向，读到名字就切过去（D41）；闲时每分钟快拍一圈，更新身边的人（D39）。说话人标签（含把握）每句都交给大模型（D32 起）。深度只来自全景，双目和转头扫描已删除（D42）。

还没做（见 [decisions.md](decisions.md) 末尾"待定"）：把高度图接入规划器、尺度校准、镜子识别、手柄交互；服务器上 Monado 仍是临时单元（`install_vr.sh` 会把它和 bridge 装成开机自启的用户单元；插件已部署，见 [deploy-log.md](deploy-log.md)）。

## 本分支的仓库结构

```text
Cargo.toml              Rust workspace（主语言）
crates/vrc-vr/          虚拟头显：remote 驱动协议、抓帧读取、位姿/内参
crates/vrc-scene/       多层 2.5 维高度图、候选点（地点和玩家）、编号标注
crates/vrc-players/     名牌 OCR、物品检测、房间玩家名单、名字匹配（跟随和工具共用）
crates/vrc-nav/         一帧全景的环视（survey_pano）和走到某点（goto）
crates/vrc-pano/        化身全景：一帧里 6 个面的彩色和深度（条码、E1C 标定和校验、解码、射线、equirect、透视视图），深度里的人（people）
crates/vrc-audio/       游戏双耳输出的方向估计（HRTF 模板）、语音检测和分段、单声道混音（speaker.md）
crates/vrc-bridge/      bridge：HTTP/WebSocket 给 AstrBot 插件（音频、状态、输入、Web API、跟随、VR 能力）
crates/vr-probe/        手动调试工具：info / grab / look / sweep / walk / hands
astrbot_plugin/         AstrBot 插件（Python，AstrBot 只认 Python 插件；本分支只支持 VR）
tools/motion/           动作片段的离线生成：下载开源动捕、重定向到规范骨架、关键帧姿势（motion.md）
tools/agent-vr/         操作 VR 菜单和自动校准的调试脚本（agent-vr-use.md）；方向扫描（speaker.md）
tools/hrtf-render/      用 Steam Audio SDK 渲染默认 HRTF 的 HRIR 表
assets/hrtf/            渲染好的 HRIR 表（steam-default-48k.bin）
third_party/monado/     上游基础提交 + 补丁
third_party/xrizer/     上游基础提交 + 补丁
ops/vr/                 构建、安装脚本和配置（已脱敏，路径和用户用参数传入）
docs/full-vr/           本目录
```
