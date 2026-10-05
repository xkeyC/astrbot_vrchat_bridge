# 全 VR 模式（feat/full_vr）

目标：让 bot 的 VRChat 以 **VR 模式** 运行在一个不存在的头显上。头部、双眼、手柄全部由程序设定；每一帧拿到渲染好的左右眼图像，以及渲染时用的精确位姿和视场角。以此替换桌面模式下"截图 + 单目深度 + 鼠标转向 + 速度积分"这一套几何层。

| 文档 | 内容 |
|---|---|
| [starting-point.md](starting-point.md) | 起始点：本仓库、上游项目、服务器各自在什么版本和状态 |
| [changes.md](changes.md) | 修改点：本仓库、其他项目（补丁）、服务器上改了什么 |
| [decisions.md](decisions.md) | 决策：选了什么、没选什么、为什么 |
| [deploy-log.md](deploy-log.md) | 部署记录（脱敏）：按时间顺序 |

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

## 现状（2026-10-05）

已在服务器上实测通过：

- VRChat 用虚拟头显进入 VR 模式。没有 SteamVR，也没有显示器（null 合成器 30 fps）。
- 通过 remote 驱动转头、低头，渲染出的画面随之改变，抓到的每帧位姿与设定一致。
- 抓帧：每只眼 1280x1280，`R8G8B8A8_SRGB`，视场 100°x100°，像素为正方形（fx = fy = 537.0，cx = cy = 640），没有遮罩，基线 0.063 m，最多每秒 10 帧。
- OSC 的 `/input/Vertical`、`/input/LookHorizontal` 在 VR 模式下仍然可以移动和转向。
- 显存：monado-service 84 MiB，VRChat 约 1.2 GB（VRChat Home，房间里只有自己），整卡约 2.1 GB。
- `vr-probe sweep` 端到端可用：转头、等到按新姿态渲染出的帧、存 PNG。

还没做（见 [decisions.md](decisions.md) 末尾"待定"）：双目深度、手柄姿态与交互、在 VR 下接入 AstrBot 插件、开机自启。

## 本分支的仓库结构

```text
Cargo.toml              Rust workspace（主语言）
crates/vrc-vr/          虚拟头显：remote 驱动协议、抓帧读取、位姿/内参
crates/vr-probe/        手动调试工具：info / grab / look / sweep
astrbot_plugin/         AstrBot 插件（Python，AstrBot 只认 Python 插件）
legacy/bridge/          桌面模式的 Python bridge（替换完成前继续使用）
third_party/monado/     上游基础提交 + 补丁
third_party/xrizer/     上游基础提交 + 补丁
ops/vr/                 构建、安装脚本和配置（已脱敏，路径和用户用参数传入）
docs/full-vr/           本目录
```
