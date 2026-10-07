# astrbot_vrchat_bridge

让一个 VRChat 客户端成为 AstrBot 的一个平台：bot 以自己的 VRChat 账号待在房间里，用实时语音和房间里的人对话。它能：

- 环视四周，认出白名单好友并跟着走；
- 用全身追踪做动作、坐下、躺下，走路时迈腿；
- 一边走一边给世界建地图，记住走得通和走不通的路，按名字走到地图上的地点或物品旁；
- 写 Chatbox，按白名单接受邀请、跨房间跟随好友。

VRChat 以 **VR 模式** 运行在一个虚拟头显上（Monado + xrizer，没有 SteamVR，也不需要显示器）。头部、双眼、手柄和全身追踪器的位置都由程序设定；每一帧都能拿到渲染好的左右眼画面，以及渲染时用的精确位姿。主语言是 Rust。起始点、修改点、决策和部署记录见 [docs/full-vr/](docs/full-vr/README.md)。

> 旧版实现（VRChat 桌面客户端 + Python bridge）在 [`legacy`](../../tree/legacy) 分支，不再维护。

## 主要能力

### 全身动作（FBT，OSC 追踪器）

不需要真实的追踪硬件：bridge 通过 VRChat 原生的 OSC 追踪器接口，45 Hz 发送髋和双脚的位姿，让化身用全身追踪（FBT）来动。详见 [motion.md](docs/full-vr/motion.md)、[fbt-research.md](docs/full-vr/fbt-research.md)。

- **自动校准**：游戏启动后，bot 用虚拟手柄的射线点开 VR 菜单，自己完成全身校准，再用 `TrackingType` 确认已进入全身模式（[agent-vr-use.md](docs/full-vr/agent-vr-use.md)）。
- **动作片段库**：开源动作捕捉数据（CMU、万代南梦宫 Research Motion Dataset、Quaternius）重定向到一套统一骨架，没有现成数据的用关键帧补上。包括打招呼、点头摇头、思考、拒绝、空翻、跳舞，以及坐、四种躺姿和站起来。生成脚本在 `tools/motion/`，许可证随片段记录。
- **腿由 bridge 自己驱动**（关掉了 VRChat 的"全身追踪时启用运动动画"）：
  - 走跑循环按实际移动速度推进，步幅和速度一致；
  - 站着转身时脚留在原地，转完再一步一步跟上，大角度转身时腿不会交叉；
  - 跳跃时收腿。
- **跟随时看人**：身体正对目标，小幅偏离只转头，头的俯仰对准名牌下方的脸。

### 场景地图

每个世界一张地图，bot 一边走一边建，存在磁盘上，下次进同一个世界接着用（`crates/vrc-map`，设计和实测见 [decisions.md](docs/full-vr/decisions.md) D31）。

- **化身位置角标**：化身上的一个着色器，把相机的世界坐标和朝向编码成黑白方块，画在 bot 自己眼睛画面的角落里，只有 bot 自己看得到。bridge 从画面里读出坐标，地图就建在世界自己的坐标系里，不会漂移。没有角标就不建地图。协议和化身端的做法见 [avatar-position-beacon.md](docs/full-vr/avatar-position-beacon.md)。
- **几何**：双目深度（CUDA 上的 SGM，没有显卡时用 CPU）写进 10 cm 的体素柱，每柱按 5 cm 分层，近处看到的权重高，视线穿过的地方会被清掉（走开的人）。每柱里的占用段给出"能站的面"：地面、台阶、桌面、二楼和它下面的一楼，所以楼梯和多层空间都能表示。
- **经验**比画面更可信：
  - 走过的路记下来，规划时优先走（"走通过就知道怎么走"）；
  - 推着摇杆却走不动的地方（玻璃、看不见的墙）记成"不通"，沿玻璃延伸到门框或柱子为止；之后从那里走通了，标记会撤掉；
  - 没有碰撞的东西（比如有些沙发）只要走过，就算能走。
- **物品**：用 YOLO 检测环视画面和跟随画面里的家具（沙发、椅子、桌子、电视、盆栽……），用双目定出位置，多次看到的合并。实测同一物品几次定位的离散在 0.05–0.21 m。
- **命名地点**：可以把当前位置和朝向记成一个名字（比如"舞台"），下次按名字走回来。

### 自动导航

- **走路工具**在地图上用 A* 规划：能上下台阶，绕开墙和标记过的玻璃，优先走走过的路；每走一段重新看一眼。地图上没路时，退回单次环视的高度图规划。
- **按名字去**：`vrchat_walk_to` 的 `to` 可以是命名地点或物品（`沙发`、`couch 2`），再远、看不见都行。到命名地点附近后，原地平移对准到 12 cm 以内，再转回记录时的朝向。
- **跟随**也用地图：
  - 走向目标的直线如果穿过标记过的玻璃，就按地图绕过去；
  - 两次取景之间，前方有标记时只转身不前进；
  - 跟丢后，从最后看到人的方向开始，朝他离开的一侧每转 60° 看一眼，第 2、5、8 圈抬头看。
- **认人**：OCR 读名牌，只认房间里的玩家；写在墙上的名字（四周和字在同一深度）不算名牌。

## 组成

- **AstrBot 插件**（`astrbot_plugin/`）：注册 `vrchat` 平台。
  - 房间语音交给 AstrBot 的实时语音会话，游戏一启动（或进入房间）就预先建好，上线后第一句话不用等它启动；
  - 提供语音工具和 LLM 工具，以及管理员命令 `/vrc status|start|stop|restart`；
  - 决策由 AstrBot 里的 Codex agent 做，插件只负责平台适配和工具。
- **bridge**（`crates/vrc-bridge`，Rust）：和 VRChat 客户端运行在同一台机器上，通过 HTTP/WebSocket 提供能力：
  - 游戏声音和 bot 语音双向传输（按住按键说话）；
  - 房间状态（跟读 VRChat 日志）、看门狗（显存上限）、启动和停止游戏；
  - 聊天框、表情、跳、小步移动；
  - VRChat Web API：只保存 cookie；接受白名单好友的邀请，跨房间跟随；
  - 环视：只转头，约 0.2 秒扫一圈，然后算双目深度、生成高度图，用 OCR 认名牌、用 YOLO 认物品，返回编号的地点和玩家、地图上已知的地点和物品，以及全景图和俯视地图；
  - 走到某个编号的地点或地图上的已知点：沿规划的路径分段走，每段后重新看；
  - 在房间里跟随一位玩家，以及白名单好友"最后一次看到"的画面；
  - 全身动作：OSC 追踪器、动作片段和姿势、步态和脚步、自动全身校准（`/v1/motion`、`/v1/vr/trackers`、`/v1/vr/calibrate`）；
  - 持久地图：里程、读角标、建图、按世界存盘（`/v1/map`、`/v1/map.png`、`/v1/map/place`、`/v1/vr/beacon`、`/v1/vr/detect`）。
- **头显、视觉和导航**（`crates/vrc-vr`、`vrc-stereo`、`vrc-scene`、`vrc-players`、`vrc-map`、`vrc-nav`）：
  - 虚拟头显的控制协议和抓帧读取，OSC 追踪器，化身位置角标的读码；
  - 双目匹配（SGM，CUDA 内核运行时编译，没有显卡时用 CPU 版，结果逐位相同）；
  - 单次环视的多层 2.5 维高度图和候选点；
  - 名牌识别与定位，物品检测与定位；
  - 持久地图：体素和多层表面、经验标记、物品和命名地点、配准、多层 A*、存盘；
  - 环视和行走。
- **离线工具**（`tools/`）：动作片段的生成（`motion/`）、步态的动作捕捉统计（`mocap/`）、操作 VR 菜单和校准的调试脚本（`agent-vr/`）。
- **对其他项目的修改**（`third_party/`）：Monado 和 xrizer 的上游基础提交，加上本项目的补丁：
  - Monado：抓帧（环形缓冲区，没人读时不拷贝）、不再有遮罩的正方形视场、运行中调整帧率；
  - xrizer：给交换链加上 `TRANSFER_SRC` 用法，让抓帧能把画面拷出来。
- **部署**（`ops/vr/`）：Monado 和 xrizer 的构建脚本、安装脚本、systemd 用户单元。机器特有的配置不入库。

## 工具

| 工具 | 作用 |
|---|---|
| `vrchat_view` | 原地环视一圈（默认），返回带编号的全景图和俯视地图，每个地点、玩家的距离、方位，以及地图上记得的附近地点和物品（"On your map"）；`around: false` 只看正前方。每次移动前先用它看一圈 |
| `vrchat_walk_to` | 走到上一次 `vrchat_view` 里的某个编号；或用 `to` 按名字走到地图上的地点或物品（再远也行，到命名地点会对准并转回原朝向）；或按方位走一段距离。走完返回正前方画面（可选 `around`） |
| `vrchat_remember_place` | 把当前位置和朝向记成一个名字，之后用 `vrchat_walk_to` 的 `to` 走回来；要先正常站立 |
| `vrchat_step` | 小而精确的动作：左转、右转或掉头，朝前、后（原地后退）、左右（横移）走几米，跳；转身或走了之后返回正前方画面 |
| `vrchat_follow_player` / `vrchat_follow_adjust` | 在房间里跟随某人；靠近、离远、原地别动、继续跟 |
| `vrchat_last_seen` | 最后一次看到某位白名单好友时的画面 |
| `vrchat_motion`、`vrchat_posture` | 全身动作（打招呼、点头、思考、空翻、跳舞……），坐下、躺下（四种躺姿）、站起来 |
| `vrchat_jump`、`vrchat_stop`、`vrchat_chatbox`、`vrchat_who` | 跳、停下、头顶文字、房间里有谁 |
| `vrchat_status`、`vrchat_social`、`vrchat_follow_rooms`、`vrchat_join` | 文字会话用：状态、白名单好友情况、跨房间跟随开关、去白名单好友所在的房间 |
| `vrchat_height`、`vrchat_vr_reset` | 调整头显高度（身高）、VR 重置（重新居中） |

## 依赖

- **AstrBot**：需要带 Codex 运行器和实时语音核心（`astrbot.core.voice`）的分支，例如 [xkeyC/AstrBot](https://github.com/xkeyC/AstrBot) 的 `codex_agent_runtime` 分支。
- **[local-multimodal-infra](https://github.com/mercallureAI/local-multimodal-infra)**：
  - 名牌识别用它的 PP-OCRv5（`POST /v1/ocr/lines`）；
  - 物品检测用它的 YOLO（`POST /v1/detect/objects`，和 OCR 同一个服务）；
  - 本地语音（`local_infra` 后端）用它的语音级联。
- **bot 的化身**：要建持久地图，化身上需要加位置角标（一个着色器和一个四边形，见 [avatar-position-beacon.md](docs/full-vr/avatar-position-beacon.md)）。没有角标时其他功能照常可用，只是不建地图。全身动作要求化身支持全身追踪。
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
7. 全身动作（可选）：
   - 在 VRChat 设置里关掉"全身追踪时启用运动动画"，腿交给 bridge 驱动；
   - 把 `tools/motion/` 生成的片段放进 bridge 配置目录的 `motions/`；
   - 用 `POST /v1/vr/trackers {"on": true}` 打开追踪器。游戏每次启动后，bridge 会自动完成全身校准。
8. 登录 VRChat Web API。会依次询问账号、密码和两步验证码，只保存 cookie：

   ```sh
   vrc-bridge login
   ```

## 配置插件

把 `astrbot_plugin/` 目录放进 AstrBot 的 `data/plugins/`，例如命名为 `astrbot_plugin_vrchat`。然后在 WebUI 里添加 `vrchat` 平台，需要填写：

- bridge 地址和 token（就是上面 token 文件的内容）；
- 语音唤醒名、别名，以及语音提示词；
- 唤醒词检测：自动（除 bot 外有两人及以上时，只回应叫到名字的话）、强制开启（无论几个人都要叫名字）或关闭（所有话都交给模型判断）；
- 好友白名单（显示名或 `usr_` id，按优先级排列），以及是否自动接受邀请、是否跨房间跟随。

文字会话里能用哪些工具，由人格的工具白名单决定，新加的工具（比如 `vrchat_remember_place`）要手动加进去。地图存在 bridge 配置目录的 `maps/` 下，每个世界一个文件。

## 安全规则

- bot 只会进入**好友**、**好友+**和**仅邀请**的实例，绝不进入公开实例或群组实例。如果被带进了这类实例（比如跟着别人穿过传送门），会立即关闭游戏，并发出告警。
- 只认 VRChat 好友列表里的白名单好友，显示名相同的陌生人不算。
- 房间里的任何人都只能让 bot 去某位白名单好友此刻所在的房间；去别的房间，需要管理员用 `/vrc start <链接>`。

## 注意

VRChat 的服务条款对自动化或机器人账号有限制，请自行评估后再使用。

## 许可证

[GNU Affero General Public License v3.0](LICENSE)。`third_party/` 里的补丁分别遵循 Monado（BSL-1.0）和 xrizer（GPL-3.0）的许可证。

## 致谢与参考

本项目建立在下面这些项目、数据和研究之上，感谢它们的作者。调研过、但最终没有采用的方案，记在 [docs/full-vr/](docs/full-vr/README.md) 的几份调研文档里。

> 标有 **fork** 的项目，本项目实际用的是修改过的 fork，而不是上游原版；上游版本不带这些修改，直接替换不能用。Monado 和 xrizer 没有 fork，而是在上游提交上打补丁（`third_party/`，构建脚本会自动打上）。

### 运行环境与上游

- [VRChat](https://hello.vrchat.com/)：[OSC](https://docs.vrchat.com/docs/osc-overview)、[OSC 追踪器](https://docs.vrchat.com/docs/osc-trackers)、[化身参数](https://docs.vrchat.com/docs/osc-avatar-parameters)、[化身缩放](https://docs.vrchat.com/docs/osc-avatar-scaling) 和 OSCQuery 接口
- [Monado](https://gitlab.freedesktop.org/monado/monado)：OpenXR 运行时（remote 驱动、null 合成器）；补丁见 `third_party/`（抓帧、按需拷贝、正方形视场、运行中调整帧率）
- [xrizer](https://github.com/Supreeeme/xrizer)：OpenVR 到 OpenXR 的转换层；补丁见 `third_party/`（交换链加 `TRANSFER_SRC` 用法）
- [OpenXR](https://www.khronos.org/openxr/)、Steam 和 Proton、PipeWire

### AstrBot 生态与推理服务

- [AstrBot](https://github.com/AstrBotDevs/AstrBot)：本项目作为它的平台插件运行
  - **fork**：[xkeyC/AstrBot](https://github.com/xkeyC/AstrBot) 的 `codex_agent_runtime` 分支（Codex 运行器、实时语音核心 `astrbot.core.voice`、语音工具）
- [OpenAI Codex](https://github.com/openai/codex)：AstrBot 里做决策的 agent 运行时
  - **fork**：[xkeyC/codex_for_astrbot](https://github.com/xkeyC/codex_for_astrbot) 的 `astrbot` 分支（AstrBot 的 Python 绑定、本地语音后端、Chat Completions 线路等；随上游版本合并更新）
- [local-multimodal-infra](https://github.com/mercallureAI/local-multimodal-infra)：OCR、物品检测和本地语音的推理服务，其中用到：
  - [PaddleOCR](https://github.com/PaddlePaddle/PaddleOCR) 的 PP-OCRv5（名牌识别）
  - [Ultralytics](https://github.com/ultralytics/ultralytics) 的 YOLO11（物品检测）
  - [ONNX Runtime](https://github.com/microsoft/onnxruntime)
- [VRCX](https://github.com/vrcx-team/VRCX)：VRChat Web API 的用法参考（登录、好友、邀请）

### 动作数据

- [CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu)，BVH 版本取自 [una-dinosauria/cmu-mocap](https://github.com/una-dinosauria/cmu-mocap)：走跑动作、步态统计
- [Bandai Namco Research Motion Dataset](https://github.com/BandaiNamcoResearchInc/Bandai-Namco-Research-Motiondataset)（CC BY-NC 4.0）：带风格的日常动作
- [Quaternius](https://quaternius.com) 的 Universal Animation Library（CC0）：游戏化身常用动作
- Drillis R., Contini R. *Body Segment Parameters*（1966）：重定向用的人体比例

### 算法与思路参考

- Hirschmüller H. *Stereo Processing by Semiglobal Matching and Mutual Information*（TPAMI 2008）：双目匹配（SGM）
- Zabih R., Woodfill J. *Non-parametric Local Transforms for Computing Visual Correspondence*（ECCV 1994）：census 代价
- [KISS-ICP](https://github.com/PRBonn/kiss-icp)：只估平移的配准、以里程作先验的思路
- [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)：按距离加权融合、用视线清除走开的物体
- [OctoMap](https://octomap.github.io)：体素的命中和穿过计数
- Triebel R., Pfaff P., Burgard W. *Multi-Level Surface Maps for Outdoor Terrain Mapping and Loop Closing*（IROS 2006）：一个格子里存多层表面
- Lumelsky V., Stepanov A. 的 Bug 算法（Algorithmica 1987）：跟随时沿墙绕行
- [VLMnav](https://jirl-upenn.github.io/VLMnav/)、[Set-of-Mark](https://github.com/microsoft/SoM)：在画面上给候选点编号，让大模型选

### 软件库

- Rust：[tokio](https://tokio.rs)、[axum](https://github.com/tokio-rs/axum)、[reqwest](https://github.com/seanmonstar/reqwest)、[tokio-tungstenite](https://github.com/snapview/tokio-tungstenite)、[rayon](https://github.com/rayon-rs/rayon)、[cudarc](https://github.com/coreylowman/cudarc)、[serde](https://serde.rs)、[clap](https://github.com/clap-rs/clap)、[tracing](https://github.com/tokio-rs/tracing)、[anyhow](https://github.com/dtolnay/anyhow)、[jpeg-encoder](https://github.com/vstroebel/jpeg-encoder)、[png](https://github.com/image-rs/image-png)、[memmap2](https://github.com/RazrFalcon/memmap2-rs)、[rpassword](https://github.com/conradkleinespel/rpassword)
- Python（`tools/` 里的离线脚本）：[NumPy](https://numpy.org)、[SciPy](https://scipy.org)、[Matplotlib](https://matplotlib.org)
