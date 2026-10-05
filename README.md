# astrbot_vrchat_bridge

让一个 VRChat 客户端成为 AstrBot 的一个平台：bot 以自己的 VRChat 账号待在房间里，用实时语音和房间里的人对话，能环视四周、走到指定的地方、认出并跟随白名单好友、做动作、写 Chatbox，还能按白名单接受邀请、跨房间跟随好友。

VRChat 以 **VR 模式** 运行在一个虚拟头显上（Monado + xrizer，没有 SteamVR，也不需要显示器）。头部、双眼和手柄的位置都由程序设定；每一帧都能拿到渲染好的左右眼画面，以及渲染时用的精确位姿。主语言是 Rust。起始点、修改点、决策和部署记录见 [docs/full-vr/](docs/full-vr/README.md)。

> 旧版实现（VRChat 桌面客户端 + Python bridge）在 [`legacy`](../../tree/legacy) 分支，不再维护。

## 组成

- **AstrBot 插件**（`astrbot_plugin/`）：注册 `vrchat` 平台。
  - 房间语音交给 AstrBot 的实时语音会话；
  - 提供语音工具和 LLM 工具，以及管理员命令 `/vrc status|start|stop|restart`；
  - 决策由 AstrBot 里的 Codex agent 做，插件只负责平台适配和工具。
- **bridge**（`crates/vrc-bridge`，Rust）：和 VRChat 客户端运行在同一台机器上，通过 HTTP/WebSocket 提供能力：
  - 游戏声音和 bot 语音双向传输（按住按键说话）；
  - 房间状态（跟读 VRChat 日志）、看门狗（显存上限）、启动和停止游戏；
  - 聊天框、表情、跳、小步移动；
  - VRChat Web API：只保存 cookie；接受白名单好友的邀请，跨房间跟随；
  - 环视：只转头，约 0.2 秒扫一圈，然后算双目深度、生成高度图，用 OCR 认名牌，返回编号的地点和玩家，以及全景图和俯视地图；
  - 走到某个编号的地点：沿规划的路径分段走，每段后重新环视；
  - 在房间里跟随一位玩家，以及白名单好友"最后一次看到"的画面。
- **头显和视觉**（`crates/vrc-vr`、`vrc-stereo`、`vrc-scene`、`vrc-players`、`vrc-nav`）：
  - 虚拟头显的控制协议和抓帧读取；
  - 纯 Rust 实现的双目匹配（SGM）；
  - 多层 2.5 维高度图和候选点；
  - 名牌识别与定位；
  - 环视和行走。
- **对其他项目的修改**（`third_party/`）：Monado 和 xrizer 的上游基础提交，加上本项目的补丁：
  - Monado：抓帧（环形缓冲区）、不再有遮罩的正方形视场、运行中调整帧率；
  - xrizer：给交换链加上 `TRANSFER_SRC` 用法，让抓帧能把画面拷出来。
- **部署**（`ops/vr/`）：Monado 和 xrizer 的构建脚本、安装脚本、systemd 用户单元。机器特有的配置不入库。

## 工具

| 工具 | 作用 |
|---|---|
| `vrchat_look_around` | 原地转头环视一圈，返回编号的全景图和俯视地图，以及每个地点和玩家的距离、方位 |
| `vrchat_walk_to` | 走到上一次环视里的某个编号，或者按方位走一段距离 |
| `vrchat_step` | 小而精确的动作：转多少度、朝某个方向走几米、跳 |
| `vrchat_follow_player` / `vrchat_follow_adjust` | 在房间里跟随某人；靠近、离远、原地别动、继续跟 |
| `vrchat_last_seen` | 最后一次看到某位白名单好友时的画面 |
| `vrchat_emote`、`vrchat_jump`、`vrchat_stop`、`vrchat_chatbox`、`vrchat_who` | 表情、跳、停下、头顶文字、房间里有谁 |
| `vrchat_status`、`vrchat_social`、`vrchat_follow_rooms`、`vrchat_join` | 文字会话用：状态、白名单好友情况、跨房间跟随开关、去白名单好友所在的房间 |
| `vrchat_height`、`vrchat_vr_reset` | 调整头显高度（身高）、VR 重置（重新居中） |

## 依赖

- **AstrBot**：需要带 Codex 运行器和实时语音核心（`astrbot.core.voice`）的分支，例如 [xkeyC/AstrBot](https://github.com/xkeyC/AstrBot) 的 `codex_agent_runtime` 分支。
- **[local-multimodal-infra](https://github.com/mercallureAI/local-multimodal-infra)**：
  - 名牌识别用它的 PP-OCRv5（`POST /v1/ocr/lines`）；
  - 本地语音（`local_infra` 后端）用它的语音级联。
- **游戏机器**：Linux 加 NVIDIA 显卡，并具备：
  - 通过 Steam（Proton）运行的 VRChat；
  - Monado 和 xrizer，用 `ops/vr/build_monado.sh` 和 `build_xrizer.sh` 构建，会自动打上 `third_party/` 里的补丁；
  - PipeWire/PulseAudio 的 `parec` / `pacat`：游戏输出接到 `vrc_out`，虚拟麦克风的输入是 `vrc_mic_in`；
  - Rust 工具链，用来编译本仓库。

## 安装

1. 构建 Monado 和 xrizer：运行 `ops/vr/build_monado.sh` 和 `build_xrizer.sh`。
2. 以 root 运行 `ops/vr/install_vr.sh <bot 用户> <VR_ROOT> [仓库路径]`：写入 Monado 配置和 xrizer 的路径文件，安装启动包装脚本和 `vrc-monado`、`vrc-bridge` 两个用户单元。bridge 单元运行 `<仓库路径>/target/release/vrc-bridge`，仓库路径默认是脚本所在的仓库，也可以在 `~/.config/vrc-bridge/env` 里用 `VRC_BRIDGE_BIN=` 改。然后把 VRChat 在 Steam 里的启动参数设为 `/usr/local/lib/vrc/vrc-launch.sh %command%`，再执行 `echo vr > ~/.config/vrc-mode`。
3. 以 bot 用户编译 bridge：`cargo build --release -p vrc-bridge`。
4. 生成至少 16 个字符的访问 token，写入 `~/.config/vrc-bridge/token`。
5. 本机特有的参数写在 `~/.config/vrc-bridge/env` 的 `VRC_BRIDGE_ARGS` 里，比如监听地址、infra 的 OCR 地址、音频设备。可用参数见 `vrc-bridge --help`，默认都是本机地址。
6. 启用两个用户单元（第 2 步已安装）。bot 用户需要开启 lingering：`loginctl enable-linger <用户>`。
7. 登录 VRChat Web API。会依次询问账号、密码和两步验证码，只保存 cookie：

   ```sh
   vrc-bridge login
   ```

## 配置插件

把 `astrbot_plugin/` 目录放进 AstrBot 的 `data/plugins/`，例如命名为 `astrbot_plugin_vrchat`。然后在 WebUI 里添加 `vrchat` 平台，需要填写：

- bridge 地址和 token（就是上面 token 文件的内容）；
- 语音唤醒名、别名，以及语音提示词；
- 好友白名单（显示名或 `usr_` id，按优先级排列），以及是否自动接受邀请、是否跨房间跟随。

## 安全规则

- bot 只会进入**好友**、**好友+**和**仅邀请**的实例，绝不进入公开实例或群组实例。如果被带进了这类实例（比如跟着别人穿过传送门），会立即关闭游戏，并发出告警。
- 只认 VRChat 好友列表里的白名单好友，显示名相同的陌生人不算。
- 房间里的任何人都只能让 bot 去某位白名单好友此刻所在的房间；去别的房间，需要管理员用 `/vrc start <链接>`。

## 注意

VRChat 的服务条款对自动化或机器人账号有限制，请自行评估后再使用。

## 许可证

[GNU Affero General Public License v3.0](LICENSE)。`third_party/` 里的补丁分别遵循 Monado（BSL-1.0）和 xrizer（GPL-3.0）的许可证。
