# astrbot_vrchat_bridge

让一个 VRChat 桌面客户端成为 AstrBot 的一个平台：bot 以自己的 VRChat 账号待在房间里，用实时语音和房间里的人对话，能做动作、写 Chatbox，能按头顶名牌找到并跟随某位玩家，能看画面（当前截图、最后一次看到某位白名单好友时的截图），还能按白名单接受邀请、跨房间跟随好友。

> **分支 `feat/full_vr`**：正在改为 VR 模式运行（虚拟头显：Monado + xrizer，程序设定头部与双眼，取得双目画面与精确位姿），主语言改为 Rust。目录结构、起始点、修改点、决策和部署记录见 [docs/full-vr/](docs/full-vr/README.md)。本分支里插件在 `astrbot_plugin/`，桌面模式的 Python bridge 在 `legacy/bridge/`；下文描述的是桌面模式。

本仓库包含两部分：

- **AstrBot 插件**（`astrbot_plugin/`：`main.py`、`vrchat_adapter.py`、`metadata.yaml`）：注册 `vrchat` 平台。房间语音交给 AstrBot 的实时语音会话；插件提供语音工具（动作、跟随、看画面）、LLM 工具和管理员命令 `/vrc status|start|stop|restart`。
- **bridge**（`legacy/bridge/`）：与 VRChat 客户端运行在同一台机器上的 Python 服务（aiohttp）。它负责：
  - 采集游戏声音、把 bot 的语音写进虚拟麦克风（按住按键说话）；
  - 通过 OSC 控制移动、转身、表情和 Chatbox；
  - 跟踪 VRChat 日志，得到所在房间和其中的玩家；
  - 通过 VRChat Web API 处理好友、邀请和白名单（只保存登录 cookie，不保存密码）；
  - 跟随：用 OCR 读名牌确定对方身份和方位，用单目深度估距，卡尔曼滤波平滑轨迹，鼠标转向；
  - 导航（模型接管）：给画面标出可走位置，按编号或网格格子走过去、绕开障碍、跳上台面，期间暂停自动跟随。

## 模型接管导航

自动跟随走不通的地方（门、楼梯、台阶、绕开障碍、去画面里的某个东西），模型自己操控角色，过程是"看画面 → 走一步 → 再看画面"的循环。画面来自单目深度估计：拟合地面，标出可走的位置（编号和距离），再叠一层网格方便指认物体。

- `vrchat_view`：看画面。`around`（四向全景）、`ahead`（正前方，带网格）、`map`（边走边建的俯视地图）、`last_seen`（最后一次看到某位白名单好友时的画面）；`camera: third` 临时切到第三人称从身后看一眼，看完自动切回。
- `vrchat_goto`：走到画面里的编号位置、网格格子、某个方向一段距离，或地图上记下的地标、可站立的平台；边走边用深度刹车，被挡住会绕行，`climb` 时贴近后跳上去。走完返回新画面。
- `vrchat_step`：小而精确的动作：按角度转身、按米数朝某个方向走（按角色自身速度计量，被挡住就停）、跳。
- `vrchat_camera_y`：上下视角。`view` 拍一张从抬头到低头的长图（每 10 度一条刻度），`level` 按地平线所在刻度校准水平，`set` 抬头或低头看东西。
- `vrchat_note`：把画面里某个格子中的东西按名字记到地图上，之后能按地图找回去（看不见也行）。
- 位置：转身按校准过的鼠标位移，行走按角色自身的速度参数（OSCQuery `VelocityX/Z`）积分；第三人称时会识别并排除画面里自己的身体。
- 第一次接管时自动暂停跟随，并记住在跟谁、隔多远；停止操作 10 秒后自动恢复跟随，`vrchat_autopilot` 立即交还。`vrchat_stop` 会停下一切。
- 动作工具都不结束回合：模型看到结果后再决定下一步，最后用自己的话回复。

## 依赖

- **AstrBot**：需要带 Codex 运行器和实时语音核心（`astrbot.core.voice`）的分支，例如 [xkeyC/AstrBot](https://github.com/xkeyC/AstrBot) 的 `codex_agent_runtime` 分支（语音工具、看画面、房间上下文需要它 2026-10 以后的版本）。
- **[local-multimodal-infra](https://github.com/mercallureAI/local-multimodal-infra)**：
  - 跟随要用到 PP-OCRv5（`POST /v1/ocr/lines`）和 Depth Anything V2（`POST /v1/depth`）；
  - 本地语音（`local_infra` 后端）用它的语音级联。
- **游戏机器**：Linux，并具备：
  - 通过 Steam（Proton）运行的 VRChat，以及一个 X 显示（默认 `:1`，1280×720）；
  - PipeWire/PulseAudio：`parec` / `pacat`，游戏输出接到 `vrc_out`，虚拟麦克风输入为 `vrc_mic_in`；
  - `ffmpeg`、`xdotool`，Python 3 及 `aiohttp`、`numpy`。

## 安装 bridge

在游戏机器上，以 root 身份在 `legacy/bridge/` 目录下运行 `sh install.sh`（bot 用户默认是 `vrcbot`，可用 `VRC_USER` 指定）。它会：

- 把 bridge 安装到 `~vrcbot/vrc-bridge/`；
- 生成访问 token：`~vrcbot/.config/vrc-bridge/token`；
- 安装并启动 systemd 用户服务 `vrc-bridge`（bot 用户需开启 lingering：`loginctl enable-linger vrcbot`，用户服务管理器才会常驻）。

本机特有的参数写在 `~vrcbot/.config/vrc-bridge/env` 的 `VRC_BRIDGE_ARGS` 里，比如监听地址、infra 地址、显示器、音频设备。可用参数见 `vrc_bridge.py --help`，默认都是本机地址。

登录 VRChat Web API（会询问账号、密码和两步验证码，只保存 cookie）：

```sh
sudo -u vrcbot -H python3 ~vrcbot/vrc-bridge/vrc_bridge.py login
```

## 配置插件

把仓库根目录作为插件放进 AstrBot 的 `data/plugins/`，然后在 WebUI 中添加 `vrchat` 平台：

- bridge 地址和 token（即上面那个 token 文件的内容）；
- 语音唤醒名和别名、语音提示词；
- 好友白名单（显示名或 `usr_` id，按优先级排列），以及是否自动接受邀请、是否跨房间跟随。

## 安全规则

- bot 只会进入**好友**、**好友+**和**仅邀请**的实例，绝不进入公开实例或群组实例。如果它被带进了这类实例（比如跟着别人穿过传送门），会立即关闭游戏，并在 AstrBot 日志里记一条告警。
- 只认 VRChat 好友列表里的白名单好友，显示名相同的陌生人不算。
- 房间里的任何人都只能让 bot 去某位白名单好友此刻所在的房间；去别的房间要管理员用 `/vrc start <链接>`。

## 注意

VRChat 的服务条款对自动化或机器人账号有限制，请自行评估后再使用。

## 许可证

[GNU Affero General Public License v3.0](LICENSE)。
