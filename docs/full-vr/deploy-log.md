# 部署记录（脱敏）

主机名、地址、端口映射、凭据、用户名一律略去；路径里的构建根目录记作 `<VR_ROOT>`。

## 2026-10-04

日期按备份目录名（`pre-vr-deploy-20261005-2151`）推断：正式部署在 10-05 21:51，这一节的试验在它之前。

| 时间 | 操作 | 结果 |
|---|---|---|
| 19:21 | 只读检查 GPU | 713 MiB / 12 GB，没有运行 VRChat |
| 19:29 | 停用 AstrBot `vrchat` 平台和插件（先备份配置和数据库），停止 `vrc-bridge` 并禁用开机启动，重启 AstrBot | AstrBot 正常运行，日志里显示 "Plugin astrbot_plugin_vrchat is disabled" |
| 19:3x | 第一次装构建依赖：本地包数据库过期，镜像返回 404 | 失败。改用户目录装工具和 Docker 构建两个方案都被否决 |
| 19:4x | `pacman -Syu` 并安装 cmake、ninja、eigen、vulkan-headers、openxr | 成功，36 个包，内核和驱动未变，没有重启 |
| 19:45 | 编译 Monado（上游快照 cfa6078b，未打补丁） | 成功：`monado-service`、`openxr_monado.json` |
| 19:4x | rustup 下载工具链时连接卡住（镜像走 IPv6 没有数据），中断后改走代理 | 工具链状态损坏（"missing manifest"），卸载后重装成功 |
| 19:50 | 编译 xrizer 0989a7fa：缺 libclang，补装 clang | 成功：`vrclient.so` |
| 19:53 | 写 Monado 配置（remote 驱动）、`openvrpaths.vrpath`；停 Steam，改 VRChat 启动参数为包装脚本（`localconfig.vdf` 备份为 `.pre-vr`）；用 systemd-run 启动 `monado-service` | 第一次失败：`epoll_ctl(stdin) failed`，因为 systemd 下没有可轮询的标准输入。加 `XRT_NO_STDIN=1` 后正常 |
| 19:55 | 启动 VRChat（VR 模式） | Monado 日志：会话建立；xrizer：SYNCHRONIZED → VISIBLE → FOCUSED。显存：VRChat 1090 MiB，monado 72 MiB |
| 19:56 | 截桌面镜像窗口 | 能看到 VR 视角（镜子本身被设成了低清晰度，所以模糊） |
| 20:00 | Monado 打抓帧补丁，xrizer 加 `TRANSFER_SRC`，重新编译，重启 Monado（`XRT_NULL_TAP=/dev/shm/vrc-eyes`，10 fps）和 VRChat | 加载期间抓到黑帧，进入世界后正常：每只眼 960x1080 RGBA8 sRGB，基线 0.063 m |
| 20:02 | 用 remote 驱动转头：0°、右 60°、低头 35°、左 90° 抬头 10° | 画面和抓到的位姿都与设定一致 |
| 20:03 | OSC 移动和转向测试 | VR 模式下有效 |
| 20:03 | 显存 | VRChat 1122 MiB，monado 72 MiB，整卡 1980 MiB，GPU 利用率约 12% |
| 20:07 | 在服务器上编译本仓库的 `vr-probe`，执行 `sweep --yaws=-90,0,90 --pitch=-15` | 三个方向都等到了按新姿态渲染的帧并存图；内参 fx 523.8、fy 547.2、cx 480、cy 540 |
| 20:4x | Monado 打补丁 0002（不再有隐藏区域，`XRT_REMOTE_FOV_DEG`），配置改为每只眼 1280x1280 正方形，以 100° 视场重启 Monado 和 VRChat | 进入世界后 `vr-probe sweep` 正常：1280x1280，fx = fy = 537.0，cx = cy = 640，四角不再有遮罩。显存：VRChat 1206 MiB，monado 84 MiB，整卡 2078 MiB，GPU 利用率约 13% |
| 21:xx | 在服务器上编译 `vrc-stereo` 和 `vr-probe`；`look --pitch -20` 后执行 `depth` | 640x640 用时 157 ms（16 核），69% 的像素有深度；中心 5.66 m，下方中心 2.15 m；地面在眼睛下方 1.918 m，倾斜 0.04°。镜面显示的是镜中世界的深度 |
| 21:xx | 通过 OSCQuery 读取眼高（`/avatar/eyeheight` 为 1.405，缩放未被禁止），再通过 OSC 写入 1.6 和 1.0，最后恢复为 1.405；每次都用双目重新量地面 | 读写都生效（`EyeHeightAsMeters`、`ScaleFactor` 跟着变）；双目测得的眼睛到地面距离始终在 1.913–1.929 之间，说明世界缩放时两眼间距也一起缩放。已恢复原眼高 |
| 21:xx | Monado 改为每帧都抓（`XRT_NULL_TAP_FPS=0`），合成器先后用 30 fps 和 60 fps 重启（VRChat 跟着重启）；执行 `vr-probe scan --count 5 --pitch -10 --down` | 30 fps：6 个方向 480 ms；60 fps：6 个方向 283 ms，GPU 利用率约 28%。全景覆盖 71%，360° 高度图约 190 万个点，耗时约 1 s。刚重启时 VRChat 还在加载，抓到的帧不随头部姿态变化，扫描会超时，等进入世界后再扫就正常 |
| 22:xx | Monado 打补丁 0003（运行中改帧率，`XRT_NULL_FPS_FILE=/dev/shm/vrc-fps`），抓帧改为 8 槽环形缓冲区；对比逐个方向和流水线扫描、30–120 fps | 改帧率生效，30→90 用时 4–32 ms，VRChat 提交帧率跟随到 119。扫描最快的仍是 30 fps 逐个方向：6 个方向 199–209 ms；90 fps 下约 290–310 ms；流水线方式会漏帧。启动时上报 90 Hz、运行时压到 30 也没有改善。最后恢复为启动 30 fps |
| 22:xx | 部署 vrc-bridge 的 HTTP 接口（临时端口 6121），环视、编号全景图和地图接口可用；全景和地图转到朝向 | 正常；无 token 返回 401 |
| 23:xx | Rust bridge 替换 Python bridge：以临时单元 `vrc-rs-bridge` 监听 <容器网关>:6120（Python 的 `vrc-bridge` 单元一直是停用状态） | Web API 登录成功（cookie 沿用），pipeline 已连上；status、social、聊天框、跳、小步移动（转 30° 后走 0.8 m，实走 1.0 m）、截图、环视均正常；WebSocket 先收到房间状态，1 秒的测试音播进麦克风，游戏音频持续流入；表情返回"当前化身没有表情参数"（和 Python 版的行为一致） |

## 2026-10-05

| 时间 | 操作 | 结果 |
|---|---|---|
| 21:52 | 正式部署（用户同意）：先备份插件目录、`cmd_config.json`、数据库和旧的 bridge 单元及环境文件；Rust bridge 装成 `vrc-bridge` 用户单元（`ops/vr/vrc-bridge.service`）并设为开机启动，环境文件去掉 Python 版独有的 `--depth-url`；插件换成 VR 版，重新启用 vrchat 平台和插件；Mon3tr 人格的文字工具白名单里，`vrchat_view` 换成 `vrchat_look_around`、`vrchat_last_seen`（`vrchat_who` 保留）；重置房间语音线程，让它用上新提示词 | AstrBot 正常运行，插件已加载，平台和 bridge 的音频流已连上；日志里只有原有的两个无关报错（知识库的 `get_dim`、GitHub MCP） |
| 22:0x | 排查"语音不通" | 链路正常：识别出了语音，Codex 也做了判断，但用户没叫名字，所以按规则保持沉默 |
| 22:1x | 跟随重写、静止姿势跟着身体朝向（D21、D22）；实测摇杆速度：0.3 → 0.9 m/s，0.6 → 2.2，1.0 → 4.0 | 部署后 bridge 重启，插件自动重连 |
| 22:2x–22:3x | 动画层上线（D23）；对照用户截图和镜子在线校准静止姿势：握持姿态 [85, 0, −90]，双手 ±0.155E | 手心朝内、手指朝下 |
| 22:3x | 头显高度：1.6 m 踮脚，1.45 m 屈膝，`monado-ctl -c` 重新居中无效果，最后定为 1.56 m | 保存在 bridge 配置目录的 `anim.json` |
| 22:44 | 部署 `vrchat_height`、`vrchat_vr_reset`：先备份插件、数据库和 bridge 环境文件，环境文件加上 `--monado-ctl`；更新插件，Mon3tr 人格的文字工具白名单加入这两个工具，重启 AstrBot | 两个接口实测正常（`monado-ctl -c` 返回成功，但如 22:3x 所记，重新居中看不出效果）；AstrBot 日志里只有原有的无关报错 |

## 2026-10-06

| 时间 | 操作 | 结果 |
|---|---|---|
| 白天 | OSC 追踪器和全身自动校准（fbt-research.md）；每只眼分辨率 1280 → 1920，镜子分辨率、抗锯齿、细节层次调高 | 冷重启 VRChat 4 次，自动校准 4 次都成功 |
| 白天 | 动作片段生成后复制到 bridge 配置目录的 `motions/`，`POST /v1/motion/reload`；VRChat 设置里关掉"全身追踪时启用运动动画" | 每次改完 bridge 都重新部署，并重新开启对用户的跟随 |
| 19:01–19:18 | bridge：跟随时身体正对、头看人、找人每 3 圈抬头、楼梯；`vrchat_view` / `around`。插件：先备份插件目录，换成新版，重启 AstrBot | AstrBot 正常运行，插件已加载，bridge 音频流已连上；日志里只有原有的无关报错（`get_dim`） |
| 待办 | Mon3tr 人格的文字工具白名单里，`vrchat_look_around` 要换回 `vrchat_view` | 未做（需要用户在 WebUI 改或授权） |

## 2026-10-06（持久地图）

| 时间 | 操作 | 结果 |
|---|---|---|
| 21:27–21:53 | bridge 换成 `feat/persistent-map` 分支的构建（还没合进 main），多次重新部署；地图存在 bot 配置目录的 `maps/` | 在 SuRroom 实测环视、前进、按地图走回命名点；跟随没有开着（之前用户用语音停掉了） |
| 21:34、21:47 | 用户上传了带位置角标的化身，两次重启游戏（`/v1/game/start` 带 `restart`，回到原来的私人房间） | 角标两只眼都读得出；第二次重启后 bot 回到出生点 |

### 当前服务器状态

- VRChat 以 VR 模式运行，`monado-service` 是临时单元（systemd-run）：合成器 30 fps（可通过 `/dev/shm/vrc-fps` 临时调整），每帧都抓（8 槽环形缓冲区），视场 100°。
- AstrBot 的 `vrchat` 平台和插件已启用（VR 版插件）；Rust bridge 作为 `vrc-bridge` 用户单元运行在 <容器网关>:6120（开机自启）。`monado-service` 仍是临时单元。
- 回退：部署前的插件、配置、数据库和 bridge 单元都备份在 AstrBot 的备份目录（`pre-vr-deploy-20261005-2151`、`pre-vr-tools-20261005-2244`）；桌面模式的 Python bridge 已从仓库删除，需要时从 git 历史取回。
