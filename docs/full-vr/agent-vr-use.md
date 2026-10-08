# agent-vr-use：代理怎样操作 VRChat 的 VR 界面（2026-10-06）

> 给以后的代理（和人）用：怎样通过 bridge 的调试接口打开快捷菜单、进完整设置、用射线点按钮、做全身校准，打开用户相机（第 8 节，2026-10-08），以及全景的调试接口（第 7 节）。数字都是 2026-10-06 在服务器上实测的（每只眼 1920×1920、视场 100°，xrizer + Monado remote 驱动模拟 Index 手柄）。

## 1. 接口

bridge 在 vrc netns 里（`10.88.0.2:6120`），带 token。服务器上用 `tools/agent-vr/bapi.sh` 调用（要 root 才能读 token）：

```sh
sudo sh bapi.sh METHOD PATH [JSON] [OUTFILE]   # BRIDGE=http://host:port 可改地址
```

| 接口 | 作用 |
|---|---|
| `GET /v1/screenshot?width=0&pitch=-20` | 左眼一帧（`width=0` 为原尺寸）。带 `pitch` 时先把头抬到/低到该角度，**并保持在那里**，之后的瞄准都按这个俯仰算 |
| `POST /v1/vr/input {"name":"QuickMenuToggleLeft"}` | 按一下 VRChat 的 OSC 按钮（`/input/<name>`，1 后 0）。打开/关闭快捷菜单 |
| `POST /v1/vr/hand {...}` | 手动摆一只手：`hand`（left/right），`offset` [右, 上, 前]（米，从双眼中点出发，按身体朝向），`turn` [偏航, 俯仰, 横滚]（度，相对身体朝向；握持姿态的 -Z 是指向），`trigger` 0..1（≥0.95 算按下），`buttons`（a/b/system/thumbstick），`squeeze` 0..1（握把，VRChat 用它抓可拾取物；跨调用保持，直到 `"squeeze":0`、`press_ms` 松开或 `release`），`press_ms`（按住这么久后松开两只手的扳机、握把和按键）。调用后动画层不再管手 |
| `POST /v1/vr/hand {"release":true}` | 手交还动画层，回到身侧 |
| `POST /v1/vr/head {"bend":35}` | 以髋关节为轴弯腰（头前移并下降，双手背到身后），用来低头看自己的脚；`0` 恢复站直。`{"lean":[右,上,前]}` 只移头 |
| `GET/POST /v1/vr/trackers` | OSC 追踪器：`on`、`parts`（hip/feet/chest/knees）、`head`（once/always/off）、`shift`（按部位偏移，米）、`scale`（默认 1）、`auto_calibrate`（默认开）。设置保存在 `~/.config/vrc-bridge/trackers.json`，bridge 重启后照旧 |
| `GET /v1/vr/calibrate` | VRChat 的 `TrackingType`（OSCQuery）：6 = 头、手、髋、脚（全身），3 = 只有头和手 |
| `POST /v1/vr/calibrate [{"force":true}]` | 自动全身校准（见第 5 节）；已经是 6 时跳过，除非 `force` |
| `POST /v1/vr/attend {"pause_s":8,"since_ms":4000,"name":"xkeyC"}` | 转向正在说话（或刚说过话）的人，三项都可省；`name` 先找这个人的话，`since_ms` 先找刚说完的；暂停跟随 `pause_s` 秒；中央名牌亮着才 `confirmed`（[speaker.md](speaker.md) 2.7） |
| `GET /v1/speakers` | 说话人追踪：已定位的玩家、最近的语音段和归属 |
| `POST /v1/speakers/record {"seconds":60}` | 录制校准数据（两耳音频、每 10 ms 的检测和投票、名牌截图），目录在 token 旁的 `speakers/` |
| `POST /v1/vr/usercam {"open":true}` | 打开用户相机并设好（第 8 节）：`stow`（默认开：把取景器拖进身体）、`check`（默认开：验证桌面显示相机）、`keep_open`（默认开：游戏重启或换世界后自己再打开）、`stow_delta`；`{"close":true}` 关掉；`"orbit"`：true、false 或要改的镜头设置（第 8 节"镜头绕圈"），可以单独发 |
| `POST /v1/vr/usercam {"stow":true}` | 相机已经开着时把取景器收进身体：取景器停在追踪空间里上次的位置，只有打开时才出现在右手边已知的位置，所以先关掉相机（`/usercamera/Close`），再按上面的步骤重新打开并拖进身体（游戏会保存相机的设置）。回复同打开，多一个 `reopened` |
| `GET /v1/vr/usercam` | 相机状态（OSCQuery 读到的 `Mode`、飞行模式、UI 遮罩）、设置、上次打开的报告和位姿；`orbit`：镜头在绕圈还是为什么没有、移动输入、最近的 Pose、计数、读名牌的情况 |
| `GET /v1/vr/usercam/names?since_ms=10000` | 镜头最近读到的名牌（最多 60 s）：名字、相对头的方位、镜头（orbit / travel / front / attend / look）、光环分数（[speaker.md](speaker.md) 2.4） |
| `GET /v1/vr/people` | 身边的人（D39）：名字、位置（世界、追踪空间）、相对头的方位和距离、最后看到、来源（`overlay` / `lens` / `asked` / `sweep` / `kept`），和闲时镜头一圈的安排（`idle_sweep`） |
| `POST /v1/vr/usercam/look {"bearing_deg": 40}` | 镜头看一眼那个方向读名字，然后回到朝前（第 8 节） |
| `POST /v1/vr/usercam/shot` | 放好相机并返回它看到的画面（JPEG，位姿在 `X-Usercam-Pose`）：`{"pose":[x,y,z,俯仰,偏航,横滚]}`（世界坐标），或绕头 `{"bearing_deg":0,"distance_m":0,"height_m":0.3,"look":"out"/"back","pitch_deg":0}` |
| `POST /v1/vr/usercam/sweep {"views":6}` | 绕头一圈，拼成一张图（每行 3 张、每张 640×360，从正前方顺时针，方位在 `X-Usercam-Bearings`） |

## 2. 打开快捷菜单

1. 头俯仰 -20°：`GET /v1/screenshot?width=320&pitch=-20`。
2. 左手举到身前、手腕上翻，让菜单正对眼睛：`{"hand":"left","offset":[-0.1,-0.3,0.3],"turn":[0,70,0]}`。手腕朝前（`turn` 俯仰为 0）时，菜单平躺在手上方，看不清。
3. `POST /v1/vr/input {"name":"QuickMenuToggleLeft"}`。
4. 菜单挂在左手上，**点菜单期间左手不要动**。

## 3. 像素 → 射线 → 手的姿势

### 相机

- 左眼图像 W×W，视场 100°（左右上下各 50°）：fx = (W/2) / tan 50°。1920 时为 805.5，1280 时为 537。
- 左眼在双眼中点左侧 0.0315 m。
- 像素 (u, v) 的方向（头坐标系，右/上/前）：d = ((u − W/2)/fx, −(v − W/2)/fx, 1)，再归一化。
- 再绕"右"轴转头的俯仰 p（-20° 时 p = −20°）：
  - up' = up·cos p + ahead·sin p
  - ahead' = ahead·cos p − up·sin p
- 射线偏航 = atan2(right, ahead')，射线俯仰 = asin(up')。

### VRChat 的指针偏移（实测）

VRChat 画出来的射线不沿握持姿态的 -Z，而是：

- 俯仰低约 **28°**：把手柄 -Z 对准目标，射线会打到目标下方很远；
- 偏航差约 **3°**。

所以手的 `turn` = (射线偏航 − 3.2, 射线俯仰 + 28, 0)。

### 手放在视线上

射线起点在手上。手离视线远时，角度误差会被放大，偏离校准点就打不中。做法：把手放在"眼睛 → 目标"这条视线上，离眼睛 D 米：

- `offset` = (−0.0315 + D·right, D·up', D·ahead')；
- 快捷菜单（挂在左手上，约 0.3–0.4 m 远）：**D = 0.25**；
- 完整设置的大面板（悬在世界里，约 1 m 远）：**D = 0.65**（手伸到靠近面板处）。D = 0.25 时，射线常常只是画到固定长度，停在按钮旁边，并没有打中。

`tools/agent-vr/aim.sh` 把以上算好并执行：

```sh
HOST=user@server D=0.65 OUT=/tmp/x sh tools/agent-vr/aim.sh U V          # 悬停，抓一帧
HOST=user@server D=0.65 OUT=/tmp/x sh tools/agent-vr/aim.sh U V click    # 悬停并扣扳机（150 ms）
```

### 残余误差和闭环

模型不完美，必须**先悬停，看图，再点**：

- **打中**：射线末端有一个**圆形光标点**，按钮会**高亮**，有的还会弹出说明（例如"校准全身追踪"）。
- **没打中**：只有一条线、没有光标点。这时线的末端位置没有意义。
- 用光标点和目标的像素差修正下一次瞄准。增益不是 1，常需要 2–3 次：
  - 大面板、D = 0.65 时，落点通常比瞄准点**低约 90–115 px、左右差 0–30 px**（1920 像素）。
  - 快捷菜单、D = 0.25 时，落点也比瞄准点低约 90 px。
- **点击**：`trigger:1, press_ms:150`。滑条一类控件会被点击改值，光标停在滑条上时不要扣扳机。
- **双击**：连发两次 `trigger:1, press_ms:80`。
- **滚动**（列表、设置内容区）：没有摇杆接口，用拖动代替。
  - 光标先停在**标签文字**上（不要停在开关或滑条上），扣住扳机（`trigger:1`，不带 `press_ms`）；
  - 分 8–10 步把瞄准点往上移（每步约 40–50 px，间隔约 60 ms），最后 `trigger:0` 松开；
  - 一下子跳到终点只能滚动约 10 px，必须分步。
  - `aim.sh` 加 `DRY=1` 只打印每一步手的 JSON，方便拼成一串命令一次发出。

### 安全

- 完整设置左栏顶部有"切换账号"和"退出 VRChat"，在这两项附近一律先悬停确认。
- 另一只手的射线也会落在面板上，容易混淆：点大面板前先把左手放下，`{"hand":"left","offset":[-0.25,-0.85,0],"turn":[0,-80,0]}`。
- 结束时：`QuickMenuToggleLeft` 关菜单，再 `{"release":true}` 把手交还动画层。

## 4. 已知位置（1920 像素，头俯仰 -20°，2026-10-06 的界面布局）

| 目标 | 瞄准（像素 / 手的姿势） | 备注 |
|---|---|---|
| 快捷菜单"校准"（有追踪器时才出现） | 手 `offset [0.095,-0.121,0.244]`，`turn [24.3,4.3,0]` | 按钮约在 (1328, 1050) |
| 快捷菜单底栏齿轮（设置） | `aim.sh 1545 960`（D=0.25）；旧方法：手同上，`turn [58.7,12.4,0]` | 单击打开快捷菜单里的设置页（会停在上次的子页，例如"音频"）；**双击直接打开完整设置** |
| 设置页右上角"展开"图标 → 完整设置 | `aim.sh 1300 422`（D=0.25） | 按钮约在 (1264, 635) |
| 完整设置左栏"图形选项" | `D=0.65 aim.sh 612 676` | |
| 左栏"镜子" | `D=0.65 aim.sh 615 772` | 个人镜子；"校准全身时开启个人镜子"默认开 |
| 图形：镜子渲染分辨率 右 / 左箭头 | `D=0.65 aim.sh 1399 500` / `1134 537` | 档位：25% → … → 100% → 无限制 |
| 图形：抗锯齿 + | `D=0.65 aim.sh 1410 430` | |
| 图形：细节层次质量 右箭头 | `D=0.65 aim.sh 1383 606` | |
| 左栏往下滚：在 (615, 830) 按住，分步拉到 (615, 510) | 拖动 | 之后能看到"控制""动捕与 IK""无障碍""调试" |
| 滚动后左栏"动捕与 IK" | `D=0.65 aim.sh 625 712` | |
| 动捕与 IK 内容区往下滚：在 (865, 810) 按住，拉到 (865, 360) | 拖动，约 3 次到底 | 起点在标签文字上，避开开关 |
| 动捕与 IK 最下面："追踪器显示外观"右箭头 | `D=0.65 aim.sh 1351 789` | 循环：球体 → 系统 → 方块 → **方向轴**（十字）→ 球体 |

面板位置跟打开菜单时的头和手有关，换了站位就要重新悬停核对。

### "动捕与 IK"页（2026-10-06 的值）

- **全身追踪**：玩家真实身高 1.68 m；使用旧版 IK 关；禁用肩部追踪 关；追踪器断开时冻结追踪 关；肩宽补偿 开；**全身追踪时启用运动动画 关**（2026-10-06 用菜单自动化关掉：腿由 bridge 的步态驱动，见 motion.md）；追踪器运动预测 0%；臂展-身高比例 45.37%。
- **校准选项**：使用旧版全身校准 关；允许单手确认校准 关；显示全身校准视觉反馈 关；为模型单独保存校准调整信息 关；通过 OSC 共享头显和手柄信息 关；追踪器吸附范围 0.40 m；进入校准前提示确认 关；**追踪器显示外观 方向轴**（原来是球体）。

## 5. 全身校准（OSC 追踪器）

### 自动：`POST /v1/vr/calibrate`（`crates/vrc-bridge/src/calibrate.rs`）

整个过程约 10 s，每一步都有确认，报告里列出每一步：

1. **追踪器**：没开就打开，等它们连续发出 1.5 s，VRChat 看到追踪器才会显示"校准"按钮。
2. **打开菜单**：头俯仰 -20°，左手举起，右手放下。
   - 先 OCR 一帧，看有没有文字恰好是"校准"（或"Calibrate"）的行；
   - 没有就切换一次菜单再看，最多两次。菜单可能本来就开着，所以不能盲按切换。
3. **瞄准**：取"校准"文字框的中心像素，用**这一帧自带的左眼位姿和视场**算射线，右手放在视线上 0.25 m 处，加上指针偏移（俯仰 +28°、偏航 −3.2°）。
4. **二次确认**：每次瞄准后再 OCR 一次，等悬停提示"校准全身…"（前缀匹配，OCR 常把"追踪"认错）。
   - 依次尝试相对文字的偏移：(0,−90)、(0,−45)、(0,−135)、(0,0)、(±30,−90)、(0,−180) 等（1920 宽时的像素）；
   - 实测第一次 (0,−90) 就能出现提示；
   - 没出现提示就**不按**。这样不会误点旁边的"回出生点"。
5. **按下**扳机 150 ms，然后**等菜单自己关**：进入校准模式后，VRChat 约 3.4 s 才关菜单（实测）。每 0.8 s 用 OCR 查一次，最多等 8 s。
   - 没关：重新瞄准再按一次；两次都不行就报错。
   - 校准模式下手动关菜单不会退出校准模式。
6. **站直**：头平视，双手放到身侧，等 2 s，两只手同时扣扳机 400 ms。
7. **结果**：5 s 内 `TrackingType` 变成 6 就算成功。只有原来是 3 的时候，这一步才真正证明校准成功（报告里 `verified`）。
8. **收尾**（不管成败）：松开扳机；如果菜单还开着就关掉；转回原来的朝向；手交还动画层。

### 自动触发（`auto_calibrate`）

动画线程约每 5 s 检查一次，以下条件都满足就自动校准：

- 追踪器开着，并且已经连续发了 10 s；
- `TrackingType` 是 3；
- 游戏在运行；
- 没有在跟随；
- 距上次自动尝试超过 120 s。

**VRChat 的校准在游戏运行期间一直有效**：停发追踪器后 5–10 s，`TrackingType` 会降到 3；恢复发送后 3 s 内回到 6，抬脚等动作照样生效（停发 30 s 实测）。所以只有 VRChat 重启后才需要重新校准，这正是自动触发要处理的情况。

### 冷重启实测（2026-10-06，`tools/agent-vr/fbt_cold.sh`）

脚本做的事：停 bridge → 停 VRChat 和 Steam → 启动 VRChat → 等进入世界 → 启动 bridge，然后观察自动校准，最后截图验证。

- 连续 4 轮全部成功，每一轮的过程都一样：
  - bridge 启动后约 30 s，VRChat 载入化身，`TrackingType` 为 3；
  - 自动触发校准，第一次瞄准就出现悬停提示；
  - 按下后约 3.5 s 菜单关闭；
  - 约 10 s 后变为 6。
- 第 3、4 轮做了截图验证：弯腰往下看，站立、抬左脚、右脚侧跨，三种姿态都跟着动。
- `wait_world.sh` 在 Monado 没有重启时会直接通过：抓帧里还留着上一局的画面。所以 bridge 可能在载入世界之前就启动了；自动校准会等到 `TrackingType` 变成 3 才动手，不受影响。
- 每次重启都会进入 bot 自己新建的私人房间（VRChat Home），房间里原来的人不会跟过来。

### 手动（调试时用）

1. `POST /v1/vr/trackers {"on":true}`。VRChat 收到 OSC 追踪器后，快捷菜单底部才会出现"校准"。
2. 打开快捷菜单，点"校准"（见上表），进入校准模式。`tools/agent-vr/fbt_calibrate.sh mode` 会做到这一步为止。
3. **校准模式下等一下**：对面会出现一面**校准镜子**，里面是自己的身体（T 字姿势）和追踪器标记（白色小球）。
   - 标记与髋、脚踝重合时，会被身体挡住看不见，这就说明对上了。外观改成"方向轴"后，三根轴会穿出身体，更容易看清位置和朝向；
   - 想确认的话，用 `shift` 把某个追踪器往外移，看它的小球从哪里冒出来，看完再恢复。
4. 姿势要对：双手放回身侧，`{"hand":"left","offset":[-0.25,-0.85,0],"turn":[0,-80,0]}`（右手镜像），头平视（`pitch=0`）。
5. 两只手同时扣扳机：左手 `trigger:1`，右手 `trigger:1, press_ms:400`。
6. 校准后，用世界里的镜子（例如 VRChat Home 那面）对照 `shift` 验证：抬脚、侧跨都能看到身体跟着动。

### 坑

- **脚的追踪器位置**：用方向轴样式在校准镜子里对过，放在离地 0.085×身高、眼睛铅垂线后方 0.05×身高处（脚背到脚踝之间，在靴子里）。原来的 0.045 / 0 位置会落在鞋尖、贴着鞋底。
- **服务器 `/tmp` 的同名旧文件**：`/tmp` 带粘滞位，root 不能覆盖 vrcbot 留下的同名文件（例如 10 月 3 日的 `/tmp/v0.jpg`）。`curl -o` 会静默失败，拿回来的是旧截图。截图文件名要用不常见的前缀。
- **缩放是 1**：VRChat 按追踪空间的米接收 OSC 追踪器，和头显一致。曾经误以为要按化身眼高缩放到 0.63，结果脚的标记在小腿上。
- **（更正）追踪器停发不会掉校准**：之前以为停发后要重新校准，其实是当时缩放错了（0.63）造成的误判。见上文。
- 镜子渲染分辨率原来是 25%，校准镜子看不清。已调到 100%（另外抗锯齿 X4、细节层次"高"）。
- 低头看不到自己的脚（化身前面挂着东西）：用 `{"bend":35}` 弯腰，再 `pitch=-80`。只往前伸头会悬空，必须同时降低高度。

## 6. 当前画质设置（2026-10-06）

- 每只眼 **1920×1920**（Monado `config_v0.json` 的 `w_pixels` 3840、`h_pixels` 1920，原来是 1280，备份为 `.pre-res-*`）。双目匹配按约 640 宽进行（`vrc_stereo::match_scale`），换分辨率不影响避障。
- 桌面窗口 640×360（注册表 `Screenmanager Resolution*`，备份为 `user.reg.bak-*`）。
- 游戏内图形设置：镜子渲染分辨率 100%（原 25%），抗锯齿 X4（原 X2），细节层次 高（原 低），阴影 低。追踪器显示外观 方向轴。
- GPU 占用约 34%，显存约 4.6 GB（原来约 10%、3.6 GB）。

## 7. 全景（化身的 6 台相机，2026-10-08；默认开，2026-10-09）

化身的 `Pano` 参数打开后，两只眼整个被全景占满：左眼是 6 个面的彩色，右眼同位置是深度（E1C），左下角保留区里有 PosBeacon 和全景条码 0x5B（协议见 avatar-panorama.md，决策见 [decisions.md](decisions.md) D33、D36）。`--pano` 默认 `auto`：在世界里就开，化身 3 s 内没有 0x5B 就回退到普通视角。看眼睛的功能都已移植：有全景帧时，环视、跟随、目击、说话人视觉、截图都从全景读，名字来自用户相机的镜头（第 8 节）和 VRChat 画在眼睛上的名牌；没有时照旧。

有全景时各功能的样子：

- **环视**（`/v1/vr/survey`、`goto`）：回复 `source: "pano"`，不转头；玩家带 `distance_m`、`bearing_deg`；另有 `named_bearings`（读到名字、名牌下面没找到人：只有方位）。没读到名字的人形不列出，镜头也不去看（"也许有人"已关，见 D36）。
- **跟随**：`GET /v1/follow` 的 `seen_by`：`named`（这次眼睛上的名牌读到了名字）、`lens`（镜头读到了名字，D40）、`kept`（按位置接着认）、`stereo`（没有全景时）。跟丢时先用镜头找、身体不动（D37、D41）：镜头快拍一圈（`search_stage: "lens_ring"`，6 个固定视角，第一个对着最后读到名字的地方、没有就最后定位的地方，其余左右交替往外 ±60°、±120°、180°，每拍到一张就切下一个，约 1 s；读到他的名字镜头立刻切过去，日志 "usercam lens: the sweep read whom it looked for: the lens to them"），再不行身体转向最后读到名字的方向、再转向最后的位置（`body_turn`，日志 "follow: lost them (panorama)"），第二次找起再转 4 个方向（`scan`）。全景时 1 s 没看到（站着、走着都一样）就算跟丢、腿站住、开始快拍。
- **按位置接着认的期限和确认**（D40）：只在最后一次读到名字 `kept_s`（默认 7 s）以内按位置接着认；1 s 没读到名字（D41，原来 2.5 s），镜头看一眼被认的那个（停 1.5 s，至少隔 3 s，日志 "follow: kept by position, no name a while: the lens looks"）：读到他的名字算确认，读到别人的名字、或读了 3 次都没有名字，就丢掉、立刻找（日志 "follow: the one kept by position is not them: lost"）。镜头（任何时候、任何方向）读到他的名字，跟随马上用那个位置（深度里的人，或方位加上次的距离，`seen_by: "lens"`，日志 "follow: the lens read their name: there"）。`GET /v1/follow` 的 `confirm`：`unnamed_s`（多久没读到名字）、`asking`（正在看的方向）、`looks`、`confirmed`、`drops`、`last`（`named` / `other name` / `no name` / `expired` / `unanswered`）、`last_ago_s`；`head_pitch_deg`（看他的俯仰：按名牌高度，平滑，±30°）；`settings.kept_s`。改：`POST /v1/follow {"settings": {"kept_s": 6}}`（3–30，不存盘，跟随照常）。
- **跟随时的镜头**（D40）：站着（或刚停下）时朝前的镜头朝目标（`GET /v1/vr/usercam` 的 `orbit.following`、`orbit.aim`），目标偏 30° 重瞄（`counts.target_poses`），20 s 没放过就重放（`follow_stale_s`，`counts.stale_poses`）；走路时照旧是行进镜头。
- **跟随绕障**（D37）：`GET /v1/follow` 的 `avoid`：
  - `why`：这一段在绕什么（`wall` 沿墙、`map` 按地图、`jump` 要跳、`detour` 卡住后转向；不在绕是 null），`age_s` 绕了多久，`progress_age_s` 上次离目标近 0.5 m 是多久以前；
  - `timeout_s`（30）、`stall_s`（15）、`unseen_s`（6）：整段超时、没进展、看不见的门限；
  - `replans`（跟着目标现在的位置重新规划的次数）、`relocates`（重新定位的次数）、`in_row`（连续几次，到 3 就站 8 s，`resting_s` 还剩多久）、`last_relocate`（`stalled` / `timeout` / `lost`）、`lens_looks`（绕的时候镜头看目标的次数）。
  - `avoiding` 是这一帧的动作，多了 `relocate`（停下重新找）、`rest`（站着等）。日志："follow: going round got nowhere: re-locating"、"follow: going round, not seen: lost"、"follow: going round again and again: standing a while"。
- **截图**：`GET /v1/screenshot` 是全景里朝头的方向的 100° 视图（`pitch` 抬低视线，头不动）；`normal=1` 借普通视角拍眼睛。
- **普通视角**：校准、打开用户相机借租约（关全景、等到一帧普通画面，结束 3 s 后开回去）；`/v1/vr/hand`、菜单按钮（`/v1/vr/input`，名字带 Menu）、`screenshot?normal=1` 让普通视角保持 20 s，所以 `tools/agent-vr/` 里一步步操作菜单的脚本照常能用（截图要加 `normal=1`）。

| 接口 | 作用 |
|---|---|
| `GET /v1/vr/pano` | 状态：`setting`（启动参数）、`wanted`（想不想开）、`effective`（现在该不该开：有普通视角租约、租约结束 3 s 内、`auto` 回退期间都不开）、`sent`（最后发出的 OSC 和多久以前）、`observed`（最近一帧是 `pano`、`unusable` 还是 `normal`，`unusable` 时 `why` 说原因）、`last_code`（最后一个条码：相机中心的世界坐标、`rig_yaw`、seq、age、layout、route、code、深度范围，和同一帧 PosBeacon 的头）、`fallback`（开了 0.5 s 还没看到 0x5B：化身没有全景装置）、`frame`（最后解码的一帧：解码耗时、标定残差、`calibration_cells_out`（被界面盖住、被剔除的标定格数）、`vignette`、`check`（E1C 校验在深度像素上的两项残差的 50%、99% 分位，单位是"一个字节值多少级"，和被当成界面遮掉的比例；用来调 `check_rg`、`check_b`）、`depth_ordinal`、眼睛尺寸）。默认先读一帧再报；`?peek=1` 不读帧 |
| `POST /v1/vr/pano {"on":true}` | 想开（`false` 想关），发 OSC `/avatar/parameters/Pano`，然后最多等 1.5 s，看到眼睛变成全景（或变回普通画面）就返回状态。之前判过 `fallback` 的，这里会重新问一次 |
| `GET /v1/vr/pano.jpg?kind=color&width=2048` | 最新一帧全景：`color` 是以头的朝向居中的 equirect（前方在中间，向右为正，宽 256–4096，高是宽的一半）；`depth` 是同一张的深度（近红远蓝，没有深度是黑）；`tiles` 是 6 个面在眼睛里的样子，彩色在左、深度在右（默认宽 1920）。不是全景时 409，报原因 |
| `GET /v1/vr/pano/points?step=16` | 每隔 `step` 个像素取一个点（每个面），世界坐标换成地图坐标 `[x, y, -z, r, g, b]`（米），最多 20 万个；`&format=ply` 返回 ASCII PLY（Unity 世界坐标原样），可以拿 MeshLab 之类打开看 |

用法和注意：

- 看一眼：`GET /v1/vr/pano.jpg?kind=color` 和 `kind=depth`（`auto` 时本来就开着）。VRChat 的界面（名牌、取景器）盖在全景上：深度里那块是空的，颜色里那块（右移最多 50 px 的视差）是别的面补的或黑的。
- 条码检查不过的帧（刚打开 age < 2、两只眼不一致、seq 和 PosBeacon 对不上、layout 不认识）一律不用，状态里 `observed.mode` 是 `unusable`。
- 校准（`POST /v1/vr/calibrate`）会先借用普通视角：关掉全景、等到一帧普通画面，结束 3 s 后再开回去。全景从没开过时这一步不读帧，没有代价。
- 启动参数：`--pano auto`（默认：在世界里就开，每 5 s 重发；化身 3 s 内没出 0x5B 就回退，60 s 后或下次进世界时再问）、`--pano on`（一直开）、`--pano off`（只有上面的接口能开）。状态里 `in_world`、`held_normal_ms`（菜单操作保持普通视角还剩多久）。
- 实测（VRChat Home，第一版 rig）：向下那面中心 1.18 m（眼高 1.176 m），标定残差 0.13–0.17 级；本机解一帧 4 线程约 10 ms。
- 深度只读 E1C（decisions.md D33 补充）：条码 code 不是 2 的帧是 `unusable`（`why`："the depth is not E1C"），说明化身上还是旧的全景装置，要重新上传。VRChat 自己的界面（名牌、取景器）盖在全景上：右眼校验不过的像素没有深度、左眼对应的位置（往右最多 50 px 的视差）没有颜色；`pano.jpg?kind=color` 里那里是别的面补的颜色或黑色，`/v1/vr/pano/points` 里颜色是 null（PLY 里是品红）。解一帧本机 4 线程约 22 ms。

### 待测（全景默认，D36；每项 10 分钟以内）

1. **默认开**：重启 bridge、进世界，30 s 内 `GET /v1/vr/pano` 的 `effective` 为 true、`observed.mode` 为 `pano`、`last_code.depth_code` 为 2、`frame.check` 的 99% 分位在 6 和 10 以内。换一个化身（没有全景装置）：3 s 后 `fallback` 为 true，`/v1/vr/survey` 回复 `source: "head scan"`。
2. **环视**：请一位好友站在 bot 前方 2–3 m，再到右侧、身后各一次，每次 `POST /v1/vr/survey`：`players` 里有他、`distance_m` 和实际差 0.3 m 以内、`bearing_deg` 对；编号全景图以朝向居中；俯视图里地面、墙对。好友站在 bot 身后、镜头和眼睛都没读到他的名字时，他不出现在 `players`（只在房间名单里）。
3. **跟随**：`POST /v1/follow {"name": "<好友>"}`，好友慢走、转弯、绕过家具，看 `GET /v1/follow` 的 `seen_by` 在 `named`、`kept` 之间、`distance_m` 合理；好友躲到墙后：bot 转向他最后读到名字的方向，镜头也转过去。两个人交叉走过时，看会不会跟错人（`seen_by` 为 `kept` 时）。
4. **说话的人**：好友在 bot 前方说话，`GET /v1/speakers` 里他的 `placed_by` 是 `stereo`（全景定位的也记为位置）、光环在眼睛上的名牌上量到（`glow_onset`）；在身后说话，镜头转过去（第 8 节）。
5. **截图和菜单**：`GET /v1/screenshot` 是前方 100° 的视图；`tools/agent-vr/` 的菜单脚本（截图加 `normal=1`）照常能点；`POST /v1/vr/calibrate` 照常。
6. **目击**：好友在房间里、不跟随时等 10 s，`GET /v1/sightings` 有他、`/v1/sightings/image` 是朝他那边的视图。
7. **跟随绕障和找人**（D37，10 分钟以内）：
   1. `POST /v1/follow {"name": "<好友>"}`，好友走到一面墙或一排家具后面（bot 和他之间被挡住），在后面慢慢走动。每 2 s `GET /v1/follow`：`avoid.why` 是 `wall` 或 `map`，`replans` 随他走动增加，`seen_by` 在 `named` / `kept` 之间；他 3 s 没读到名字时 `lens_looks` 加一（镜头看他一眼，走路停一下）。
   2. 好友站在 bot 绕不过去的地方（例如玻璃后面、围栏另一边）：15 s 左右 `relocates` 加一、`last_relocate` 为 `stalled`，bot 停下、镜头看他，然后换一边绕；连续 3 次后 `avoiding` 为 `rest`、`avoid.resting_s` 倒数 8 s，bot 面向他站着，不会一直转圈。
   3. 好友躲到 bot 看不见的地方：6 s 内 `last_relocate` 为 `lost`、`state` 为 `searching`；`search_stage` 先是 `lens_ring`（镜头快拍一圈，身体不动，`GET /v1/vr/usercam` 的 `orbit.sweeping`），读到他的名字就停（`orbit.sweep_end.how` 为 `found`），跟随接着走；读不到才 `body_turn`（身体转）。
   4. 找人的镜头一圈中途推一下摇杆或说句话：`orbit.sweep_end.how` 为 `move` / `voice`，镜头一圈立刻停。
8. **按位置接着认的期限、镜头确认、镜头找到就用上**（D40，10 分钟以内；部署后先 `POST /v1/follow {"name": "xkeyC"}`）：
   1. 好友站在 bot 前方 1.5–2 m，`GET /v1/follow` 的 `seen_by` 为 `named` 或 `lens`、`confirm.unnamed_s` 小；`GET /v1/vr/usercam` 的 `orbit.following` 为 true、`orbit.aim.faced` 为 true，`orbit.last_pose.pose.yaw` 和 `orbit.aim.world_yaw` 差 30° 以内。
   2. 好友绕到 bot 侧面 60° 左右站住（bot 不动时）：几秒内镜头转过去（`counts.target_poses` 加一，直播画面里是他）。
   3. **广告牌**：好友站到一块广告牌、柜子之类人形大小的东西旁边，然后快步躲到墙后。`seen_by` 变 `kept` 后约 1 s（D41），`confirm.asking` 出现、镜头看向被认的东西；约 1–2 s 后 `confirm.last` 为 `no name`（或 `other name`）、`drops` 加一，`state` 变 `searching`、`search_stage` 是 `lens_ring`（快拍的第一张对着他最后读到名字的地方，D41）。全程 `seen_by: "kept"` 不超过 `kept_s`（7 s）。
   4. **镜头找到就用上**：好友从墙后走出来、站到 bot 侧后方：镜头读到名字后（`GET /v1/vr/usercam/names?since_ms=3000` 有他），`GET /v1/follow` 马上是 `seen_by: "lens"`、`state: "following"`，bot 转身朝他。
   5. **俯仰**：好友站到台子上或楼梯上面、或坐下：`head_pitch_deg` 相应变正、变负（±30° 以内），`GET /v1/screenshot` 里他的身子和眼睛上的名牌都在画面里。
   6. 好友停着不动 20 s 以上：`counts.stale_poses` 会涨（每 20 s 一次）；推一下摇杆后行进镜头照常。

## 8. 用户相机：远程的眼睛（2026-10-08）

化身上的相机看不到名牌（只有 VRChat 自己的 UI 相机画名牌），用户相机打开 UI 遮罩后能看到。它可以用 OSC 放到世界里任何位置；直播模式下桌面窗口显示相机画面。以后全景常开（第 7 节），眼睛里只有全景拼块，所以**用户相机的直播画面就是名牌的来源**：bot 站着时镜头绕着它转一圈圈地看，走路时镜头朝前进方向看。决定见 decisions.md D34、D35。

### 自动：`POST /v1/vr/usercam {"open":true}`（`crates/vrc-bridge/src/usercam.rs`）

1. 读 OSCQuery 的 `/usercamera/Mode`（0 关闭、1 拍照、2 直播）。已经打开就跳到第 4 步。
2. **打开**：用全身校准的姿势（头俯仰 -20°，左手举菜单，右手放下），OCR 找快捷菜单里的"回出生点"，按 (+80, +48) 像素推出相机图标的位置（图标没有文字）。右手瞄准后要等到悬停提示"查看相机选项…"出现才双击，尝试的偏移和校准一样。双击后 3 s 内 `Mode` 不为 0 才算打开；不行就再试一次。
3. **取景器拖进身体**：取景器出现在右手旁边，这时右手正好在它的抓取范围内（面板会发蓝光）。先关掉 `FlingToClose`（甩出去会关相机），左手放下；握把 `squeeze` 1，等 0.3 s；保持手的朝向，分 12 步、每步 50 ms 移动 `STOW_DELTA`（右 -0.055、上 -0.08、前 -0.32 m），也就是从取景器中心约 (0.055, -0.30, 0.32) 移到胸口约 (0, -0.38, 0)；等 0.3 s 松开，再等 0.3 s。拖之前和之后各用 OCR 看一次视野里有没有取景器的按钮文字，报告里的 `stowed` 就是据此判断的。**还没有实测**，见下文"待测"。
4. **OSC**：`Mode` 2（直播）、`ShowUIInCamera` true（UI 遮罩，名牌），关飞行模式；用 OSCQuery 检查。
5. **验证桌面**：读位置角标得到头的位置和朝向，把相机放到头顶 0.3 m 处，先朝后、再朝前各抓一帧，两帧平均差 ≥ 10 才算桌面显示的是相机。之后相机留在头顶朝前。
6. **收尾**（不管成败）：松开握把和扳机；用开菜单的姿势看一眼，快捷菜单还开着就关掉；头和手恢复原样。

`keep_open`（存在 `usercam.json`）：动画线程每 5 s 检查一次，进入世界 30 s 后相机还关着，就自动执行一次打开，两次之间至少隔 120 s；有待校准的全身追踪时让校准先做。

### 拍照：`shot`、`sweep`

- 每张：`/usercamera/Pose` → 等 150 ms → `Flying` false（OSCQuery 确认，最多 3 次）→ 等 100 ms → 抓桌面。约 0.5 s 一张。
- 绕头放置时，位置和朝向取自位置角标（两只眼的平均位置、头的偏航）。`look: "out"` 从头往外看，`"back"` 在 `distance_m` 处回头看头（俯仰对准头）。`height_m` 默认 0.3，在头顶上方，避免拍到自己的头。
- 抓桌面用 ffmpeg 的 x11grab，`--display`（默认 `:1`），区域由 `--desktop-grab` 指定（默认 `1280x720+0,0`）。
- 拍照、扫一圈期间镜头绕圈让开（它们持有同一把锁），完了再接着绕。

### 镜头：站着时朝前，走路时朝前进方向（`crates/vrc-bridge/src/orbit.rs`，D35）

起初用户要求镜头一直绕着 bot 转（下面"站着：绕圈"）。2026-10-08 晚用户改主意：绕圈太引人注意，**默认改成站着时镜头静止朝前**（`idle: "front"`），绕圈留作选项（`idle: "orbit"`）。

- **站着：朝前**（`idle` 默认 `front`）。镜头停在行进镜头的位置（两眼后方 `travel.back_m`、上方 `travel.up_m`，朝头的朝向，向下 `travel.pitch_deg`）。只在头（或身体）的朝向偏离镜头超过 `travel.reaim_deg`（30°）、离上次放置至少 `travel.reaim_every_ms`（1 s）时重放：一个 `Pose`，150 ms 后关飞行。没有连续的 `Pose`，别人看不出什么。转向说话的人、看一眼某个方向、拍照之后，回到朝前。朝前时照样读名牌；朝前就和行进镜头一样，站着时往前走不用重放镜头（只等飞行关掉）。**跟随时**（D40）朝前的镜头朝目标（跟随每次定位给的方向，3 s 内有效），目标偏 30° 重瞄，20 s（`follow_stale_s`）没放过就重放；所有输入都松开就可以放（不用站够 `resume_after_s`，绕圈仍要等）。
- **快拍一圈**（`Orbit::sweep`，D37，D41 起不再连续转）：跟随跟丢、闲时一圈时，镜头依次放到 `snap.views`（6）个固定视角（看一眼的位置 `look`，各一个 `Pose`，不管 `idle`），每个视角**拍到第一张显示它的画面**就放下一个：抓帧时刻离 `Pose` 至少 `snap.min_ms`（100 ms），而且和放 `Pose` 前的最后一帧不一样（灰度缩略图平均差 ≥ `snap.diff`），或者已经 `snap.sure_ms`（350 ms）；`snap.max_ms`（700 ms）还没拍到就跳过。OCR 在单独的线程里读，不等它就放下一个视角，最多 `snap.in_flight`（3）个同时在读；每张按它自己那个视角的 `Pose` 定位。跟随找人时第一个视角对着最后看到他的方向，其余左右交替往外（0、+60、−60、+120、−120、180）；闲时一圈从头的朝向顺时针。找人时某张读到他的名字：这一圈立刻停（`found`），镜头 `look_at` 他那个方向 2.5 s，深度沿读名的射线把他放好。一圈约 1 s（每个视角约 150 ms）。`sweep_end.how`：`done` 拍完、`found` 找到了、`stopped` 跟随放弃了、`move` 被一推打断、`voice` 有人开口、`not begun` 2 s 内没开始。只在所有移动输入都松开时开始（不等 `resume_after_s`；跟随会等腿松开最多 0.5 s）；一推（移动闸门）立刻停，所以不会在飞行模式可能开着时推。不动的镜头（看一眼、朝前）换了 `Pose` 后第一张稳了的画面立刻读，不等 OCR 的节拍。
- **绕障时少重瞄**（`Orbit::calm_travel`，D37）：跟随绕障时行进镜头偏离超过 60°、至少隔 3 s 才重瞄（平时 30°、1 s），`calm_ms` 是还剩多久。
- **看一眼某个方向读名字**（`look`）：`POST /v1/vr/usercam/look {"bearing_deg": 40}`（相对头的朝向，向右为正；`tol_deg` 默认 15，`wait_ms` 默认 1500、最多 5000）。那个方向 1 s 内读到过名字就直接回答；否则镜头放到头外 0.3 m 朝那边看（同转向说话的人的镜头），等到读到名字或超时，然后回到朝前。走路时和重瞄一样，两个轴先松开一下。代码里是 `Orbit::look_at`、`Orbit::name_toward`：跟随、环视给全景里的人认名字时用。
- **站着：绕圈**（`idle: "orbit"`）。每秒 `rate_hz`（30）个 `/usercamera/Pose`：以两眼中点（位置角标，每秒读一次）为圆心，半径 `radius_m`（0.7 m，在头外面）、高 `height_m`（0，和眼睛一样高）加上一圈两次的起伏 `bob_m`（0.04 m），朝外、向下 `pitch_deg`（8°），`period_s`（3 s）一圈：每个 `Pose` 转 4°（镜头移 5 cm），差不多一帧一个，看起来是连续的（20 个/秒时是 6°）。从头的朝向开始，停下再接着上次的角度转。飞行模式开着，bot 站着不碍事。竖直视场 60° 的 16:9 画面横向约 91°，一个名牌每圈在画面里约 0.75 s，每秒读 4 次就有约 3 次机会。
- **移动前的互锁**：所有移动输入（跟随的腿、`/v1/step`、`goto`、摇杆、跳、`/v1/vr/input` 的 Move*）都经过同一个关口，`vrc_vr::osc` 的移动闸门（`set_move_gate`，所有 `Osc` 发 `/input/Vertical`、`Horizontal`、`Jump`、`MoveForward/Backward/Left/Right` 之前都先过它）：
  1. 站着时来了一次推（不为 0 的值）：停止绕圈；把镜头放到**行进镜头**的位置（`travel`：两眼后方 `back_m` 0.35 m、上方 `up_m` 0.35 m，朝这次推的方向 = 头的朝向 + 摇杆的方向，向下 `pitch_deg` 12°），和镜头现在的朝向差不到 `leg_deg`（10°）就不重放；
  2. 最后一个 `Pose` 之后 150 ms 发 `Flying` false（早了会被 `Pose` 带来的开启覆盖）；
  3. 再等 `settle_ms`（50 ms）才放这次推出去。所以每一段走路的开头停顿约 200 ms。等的时候不持有任何锁；别的推在这期间也等着；松开（值为 0）从不等。
- **走路时**：跟随方式是"玩家位置"时，关了飞行模式后镜头跟着 bot 的**位置**走（保持相对位置），但**不跟着转**：bot 转了 90°，镜头还朝原来的方向（2026-10-08 实测）。所以绕圈线程在走路时盯着：前进方向和镜头的朝向差超过 `reaim_deg`（30°），并且离上次重瞄至少 `reaim_every_ms`（1 s），就**重瞄**：两个轴先发 0（直接发，不经过闸门，请求的值不变），放一个新的行进镜头，150 ms 后关飞行，再等 `settle_ms`，把两个轴按现在请求的值发回去。走路会停顿约 200 ms。按着别的移动按钮（跳、Move*）时不重瞄。前进方向用最近发给头显的头朝向（`Anim::head`，换到世界里），没有就用位置角标的。
- **恢复绕圈**：所有移动输入都松开、并且 bot 自己的速度（里程计）也停了 `resume_after_s`（1.5 s）之后。某个移动输入 30 s 都没松开、bot 又一动不动（发送的任务死了），也当成停了。
- **相机关着**（每 2 s 读一次 `Mode`）：什么都不做；自动重开（`keep_open`）或手动打开之后自己接着绕。设置里 `on: false` 也停。
- **不绕的时候**：最后一个 `Pose` 之后 150 ms 自己关一次飞行模式，300 ms 后用 OSCQuery 看一眼，还开着就再发（最多 3 次）。
- **有人开口：镜头转过去**（`attend_onset`，**2026-10-09 起默认关**：每段话一开口镜头就甩过去太突兀。全景眼睛上 VRChat 照样画出前方的名牌；bot 闲着、被叫到时它自己转过去（`POST /v1/vr/attend`，跟随、走动、菜单时不转，[speaker.md](speaker.md) 2.7）。打开这项就恢复下面的做法）。说话人追踪开了一段新的话（不是 bot 自己的回声），并且有了方位（归属的玩家的方位，或者投票的峰：20 个有效频点就够先看一眼）：
  1. 停止绕圈，镜头放到头外 `attend.out_m`（0.3 m）处、和眼睛一样高，沿那个方位看、向下 5°；150 ms 后关飞行模式（随时可能要走路）。
  2. 这期间每秒读 `attend.ocr_hz`（5）次名牌，两次读之间的每一帧（10 帧/秒）都在上次读到的文字框上重新量光环：光环的起亮时刻能精确到一帧，名字和方位也一起交给说话人追踪（[speaker.md](speaker.md) 2.4）。
  3. 0.4 s（`mirror_after_ms`）内那边没有名牌亮、而投票的前后镜像差不多强：转去看镜像那边（只转一次）。说话人追踪后来把这段话归给了某个已定位的玩家、方位又不同：转去看他。
  4. 一直看到这段话结束后 `hold_s`（1 s，光环的尾巴是 0.9 s）；中途同一方向（30° 以内）又有人开口就接着看；另一个方向有人开口就转过去（最多每 0.5 s 一次）。之后接着绕圈。
  5. 走路时：前进方向和那个方位差不到 `travel_off_deg`（60°）就不管，行进镜头不动；差得多才转过去（和重瞄一样，走路停顿约 200 ms），这期间行进镜头不重瞄；看完再回到行进镜头。
  - 拍照、扫一圈时也让开。`GET /v1/vr/usercam` 的 `orbit.attending`：段号、方位（世界、相对头）、`why`（`direction` / `candidate`）、还没看的镜像、看了多久、亮过没有、还要看多久。
- **读名牌**：镜头绕圈、在行进位置或转向说话的人时，抓桌面窗口读名牌，交给说话人追踪（[speaker.md](speaker.md) 2.4）。定位（D38）：镜头的位置是 `Pose` 给的位置加上从那以后 bot（头）走过的位移（"玩家位置"跟随只平移不转）；不动的镜头等 `Pose` 稳了 `settle_ms` 并且画面不再变才读；读到名字时沿名牌射线在全景深度里找人，方位和距离（`GET /v1/vr/usercam/names` 的 `distance_m`、`feet`）按人相对头的位置算，深度里没有人才按射线上离头 2.5 m 的点算。
- **CPU**：x11grab 抓 1280×720、15 帧/秒（D41 起，快拍每个视角少等一帧）、原始 RGB 走管道（约 41 MB/s），ffmpeg 不编码，估计占一个核的几个百分点；bridge 这边每帧只拷一次、OCR 在 infra 的 GPU 上。服务器上还没量（见"待测"）。

设置存在 `usercam.json` 的 `orbit` 里，`POST /v1/vr/usercam {"orbit": {...}}` 改哪项给哪项（会检查范围），`{"orbit": false}` 关掉：

| 设置 | 默认 | 作用 |
|---|---|---|
| `on` | true | 镜头（站着时 `idle`，走路时行进镜头）；false 时什么都不放 |
| `idle` | `front` | 站着时：`front` 静止朝前（用行进镜头的 `back_m`、`up_m`、`pitch_deg`、`reaim_deg`、`reaim_every_ms`），`orbit` 绕圈 |
| `radius_m`、`period_s`、`height_m`、`bob_m`、`pitch_deg`、`rate_hz` | 0.7、3、0、0.04、8、30 | 绕圈（0.3–3 m、2–60 s、5–60 Hz） |
| `resume_after_s` | 1.5 | 停稳多久后接着绕 |
| `travel` | `{"on":true,"back_m":0.35,"up_m":0.35,"pitch_deg":12,"leg_deg":10,"reaim_deg":30,"reaim_every_ms":1000,"settle_ms":50}` | 行进镜头 |
| `sightings`、`grab_fps`、`ocr_hz`、`ocr_hz_hot`、`fov_deg`、`lag_ms` | true、15、4、5、null、70 | 读名牌：抓帧率、OCR 频率（有人说话时）、竖直视场（null：读 `/usercamera/Zoom`，读不到用 60°）、画面比 `Pose` 晚多少 |
| `idle_sweep_s` | 60 | 闲着时镜头每隔这么久快拍一圈读四周的名牌（0 关，10–3600；D39，`GET /v1/vr/people`） |
| `snap` | `{"views":6,"min_ms":100,"diff":6,"sure_ms":350,"max_ms":700,"in_flight":3}` | 快拍一圈（D41）：视角数（3–12）、拍一张至少离 `Pose` 多久、和放 `Pose` 前那帧至少差多少、多久以后不比画面、一个视角最多等多久、最多几个 OCR 同时在读（1–4） |
| `settle_ms`、`settle_diff` | 150、8 | 不动的镜头（不含绕圈）稳了才读：`Pose` 至少这么久没换（到抓帧时），且和上一帧的灰度缩略图平均差不超过这个值（0 不比，D38） |
| `settle_sure_ms` | 600 | `Pose` 放了这么久以后不再比画面（早已生效）：跟着 bot 走的行进镜头、视频广告牌前也照常读（0–5000，D40） |
| `follow_stale_s` | 20 | 跟随时镜头这么久没放过就重放一次（5–300，D40） |
| `look` | `{"back_m":0.15,"up_m":0.25,"pitch_deg":2}` | 看一眼某个方向时镜头的位置：离开那个方向 `back_m`、比眼睛高 `up_m`、向下 `pitch_deg`（D38） |
| `attend_onset`、`attend` | false、`{"out_m":0.3,"height_m":0,"pitch_deg":5,"hold_s":1,"mirror_after_ms":400,"reaim_deg":30,"reaim_every_ms":500,"travel_off_deg":60,"ocr_hz":5}` | 有人开口时镜头转过去 |

`GET /v1/vr/usercam` 的 `orbit`：`active`（站着时的镜头在位：朝前或在绕）、`idle_lens`（`front` / `orbit`）、`idle`（不在位的原因：`moving`、`camera closed`、`busy`、`off`、`no head (the position beacon)`、`not in a world`、`attending`、`looking`）、`angle_deg`、`axes`、`flying_maybe_on`、`last_pose`（`lens`：orbit / travel / front / attend / look / snap / placed）、`counts`（绕圈的 Pose、朝前的 Pose、互锁、行进镜头、重瞄、转向说话的人、看一眼、转一圈、关飞行的次数）、`sweeping`（快拍一圈：`why`：`follow` / `idle`、`seek` 找谁、`from` 第一个视角的世界偏航、`out` 是否左右交替、`view` / `views` 拍到第几个、拍了多久）、`reading`（正在读的快拍张数）、`counts.snap_poses`、`sweep_end`（上一圈怎么结束：`done` / `found` / `stopped` 跟随放弃 / `move` / `voice` / `not begun`，多久以前）、`calm_ms`、`following`（有跟随在跑）、`aim`（跟随最后给的目标方向：`world_yaw`、`bearing_deg`、`age_ms`、`faced` 镜头是否朝它，D40）、`counts.target_poses`、`counts.stale_poses`、`attending`、`looking`（看一眼的方位、还要看多久）、`sightings`（抓帧、读了几次、两次读之间量的光环、没对上位姿的帧、错误、最近读到的名字）。`idle` 多了 `attending`。

### 手动（实测，2026-10-08，VRChat Home）

**打开**：

1. 按第 2 节打开快捷菜单（头 -20°，左手 `{"hand":"left","offset":[-0.1,-0.3,0.3],"turn":[0,70,0]}`）。
2. 相机图标在底栏，约 (1302, 1134)；"回出生点"文字约在 (1222, 1086)。悬停：`aim.sh 1302 1044`（D=0.25），即 `{"hand":"right","offset":[0.066,-0.101,0.207],"turn":[21.96,4.22,0]}`，提示"查看相机选项，双击可唤出相机。"
3. **双击**：同样的 JSON 加 `"trigger":1,"press_ms":80`，发两次，间隔 150 ms。相机以拍照模式打开（`Mode` 1），快捷菜单自己关掉。
   - 只单击的话，快捷菜单会切到相机页（拍照相机、直播相机、拍立得相机（VRC+）、无人机（VRC+）、截图、多图层相机、浏览本地保存目录）。下次打开快捷菜单时又回到"导航"页。
4. 取景器出现在视野右下（中心约在 (1130, 1300)，大约 0.45 m 远）。右手在上面那个姿势、或 D=0.3 指向它时，面板会发蓝光，旁边显示"Equip"，这说明手在抓取范围内。

**取景器上的按钮**（头 -20°；用 D=0.15，手离面板远一些才会出射线，D=0.25–0.3 时手已经在面板的抓取范围里）。D=0.15 时，光标落点比瞄准点**低约 190–225 px、偏左 0–50 px**：

| 目标 | 瞄准（D=0.15 的 `aim.sh`）/ 手的姿势 | 悬停提示 / 备注 |
|---|---|---|
| 底排上方的"^"（收起或展开按钮排） | `1132 1253` / `{"offset":[-0.002,-0.095,0.113],"turn":[11.48,-11.06,0]}` | "展开/收起相机菜单。"只是隐藏或显示这一排 |
| 左上"⇔"（翻转镜头） | `981 968` / `[-0.028,-0.053,0.140]`、`[-1.61,7.44,0]` | "翻转相机镜头。跟随方式为默认时，将翻转镜头朝向；否则将镜像相机画面。" |
| 按钮排右侧"›"（下一页） | `1585 1345` / `[0.055,-0.088,0.086]`、`[41.79,-7.79,0]` | 第 1 页 → 第 2 页 → 第 3 页 |
| 按钮排左侧"‹"（上一页） | `772 1262` / `[-0.064,-0.095,0.111]`、`[-19.25,-11.43,0]` | |
| 第 3 页"镜头可见度" | `1195 1325` / `[0.007,-0.101,0.104]`、`[17.19,-14.52,0]` | "设置相机镜头的外观。"点开后下面一排是：关闭、半透明（原来的值）、实体 |
| 镜头可见度 → "关闭" | `1030 1410` / `[-0.020,-0.113,0.098]`、`[3.42,-21.00,0]` | 提示是"开启后，相机镜头将对您不可见。"；**2026-10-08 已设为关闭**，游戏会保存 |
| 第 2 页"跟随方式" | `1201 1323` / `[0.008,-0.101,0.104]`、`[17.65,-14.32,0]` | "设置相机的跟随方式。"选项：默认、**玩家位置**（2026-10-08 起的值）、世界。步骤见下 |
| 跟随方式 → "玩家位置" | `1136 1423` / `[-0.004,-0.113,0.095]`、`[13.18,-20.71,0]` | "开启后，相机镜头将跟随您的位置。" |
| 第 2 页"辅助功能" | `1310 1340` / `[0.023,-0.099,0.098]`、`[25.97,-13.38,0]` | 防抖、对准我、滚转水平、俯仰水平、联动镜头旋转（开）、按下扳机拍照、Fling to close（开）……这些都有对应的 OSC |

- 按钮排三页的内容：第 1 页 拍摄照片、倒计时（5 秒）、拍照模式、Spout 串流、相机作为录音源、飞行模式；第 2 页 Spout 串流、相机作为录音源、飞行模式、跟随方式、辅助功能、对焦；第 3 页 机位、图层、运镜（VRC+）、镜头可见度、滤镜、分辨率。
- 也可以拖动按钮排（扳机按住，从 `1368 1360` 分 9 步拉到 `1048 1360`，每步 60 ms），大约移动三个按钮。
- 点开的子菜单会留在下面，再点一次同一个按钮（变成返回箭头）就收起来。

**跟随方式改成"玩家位置"**（一次性设置，游戏会保存，bridge 不再每次设；2026-10-08 测试时这样改的）。头 -20°，D=0.15，取景器翻到第 2 页：

1. 悬停"跟随方式"：`aim.sh 1201 1323`（手 `{"hand":"right","offset":[0.008,-0.101,0.104],"turn":[17.65,-14.32,0]}`），提示"设置相机的跟随方式。"；
2. 点它：下面出现一排 默认 / 玩家位置 / 世界；
3. 悬停"玩家位置"：`aim.sh 1136 1423`（手 `[-0.004,-0.113,0.095]`、`[13.18,-20.71,0]`），提示"开启后，相机镜头将跟随您的位置。"；
4. 点它；
5. 再点一次"跟随方式"收起来。

- 光标落点比瞄准点低约 217 px、偏左约 21 px。**坑**：第一次瞄 `1144 1345` 悬停到的是"飞行模式"，所以每次都先悬停、看提示，再点。
- 检查：展开"跟随方式"后当前的选项有白框；或者看悬停提示。实际效果：关了飞行模式后走一步，镜头跟着 bot 平移就是"玩家位置"，留在原地就是"世界"。

**OSC**（VRChat 在 vrc 网络命名空间里监听 127.0.0.1:9000，用 vrcbot 身份发：`sudo -n ip netns exec vrc sudo -n -u vrcbot python3 ...`）：

| 地址 | 实测 |
|---|---|
| `/usercamera/Mode` i | 0 关闭、1 拍照、2 直播。直播模式下桌面窗口显示相机画面 |
| `/usercamera/ShowUIInCamera` T/F | UI 遮罩：开了才能拍到名牌 |
| `/usercamera/Pose` ffffff | 世界坐标 x、y、z，俯仰（**正值向下**）、偏航（和位置角标一样，从 +Z 顺时针）、横滚。1–3 帧内生效，没有平滑。**会打开飞行模式**。跟随方式"玩家位置"下也是世界坐标；关飞行后镜头跟着 bot 平移（走 1 m 后相对位置不变），不跟着转身（2026-10-08 实测）。每个 `Pose` 后 0.3 s 发 F，连续 6 个都对，之后走路不受影响 |
| `/usercamera/Flying` T/F | 开着时移动输入飞的是相机，bot 走不了（`/v1/step` 报 `blocked`）。`Pose` 之后约 150 ms 再发 F，同时发会被覆盖 |
| `/usercamera/Close` T | 关闭相机（`Mode` 变 0）。镜头的位置、跟随方式、镜头可见度下次打开时都还在 |
| `/usercamera/FlingToClose` T/F | 甩出取景器就关相机，默认开 |

- OSCQuery（端口写在 VRChat 日志里，`of type OSCQuery on <port>`）下的 `/usercamera` 能读到所有当前值，例如 `curl http://127.0.0.1:<port>/usercamera/Mode`。`Pose` 读出来一直是 0，不能拿来确认位姿。
- 抓桌面：`DISPLAY=:1 ffmpeg -f x11grab -video_size 1280x720 -i :1+0,0 -frames:v 1 x.png`（vrcbot 身份）。

### 坑

- **取景器固定在追踪空间里**：走路时跟着移动，原地转身时不动（转 90° 后在视野左下）。所以不管 bot 朝哪边，都有可能看到它，只能把它放到身体里。
- 取景器只能用握把抓，扳机抓不起来（实测）。
- 取景器出现在打开时快捷菜单附近，和右手离得多远无关（D=0.12 和 0.25 位置一样）。把菜单举低再打开，取景器也只到视野右下角，见 D34 的"没选"。
- 相机在 bot 身前、飞行模式开着时走路，相机会飞到 bot 脸前。飞行模式会吃掉所有移动输入（包括 OSC 的 `/input/Vertical`），所以移动闸门保证飞行模式可能开着时不发任何推。
- 镜头绕圈的半径要够大，别让镜头落在 bot 的头里（默认 0.7 m）。

### 待测（部署新 bridge 后）

1. 打开时 `stowed` 是否为 true；不是的话，用 `/v1/vr/hand` 的 `squeeze` 手动拖一次：右手放到上面的图标姿势，`"squeeze":1`，再分几步移动（每次调用都会保持握把），最后 `"squeeze":0`。看取景器跟不跟手、松手后留不留在原处，再据此改 `STOW_DELTA`。
2. 换世界后相机会不会自己关掉（`GET /v1/vr/usercam` 的 `state`）。
3. 镜头可见度"关闭"后别人看到的样子（请用户在自己那边看）。
4. 镜头朝前（默认）：站着时 `orbit.last_pose.lens` 是 `front`、`counts.front_poses` 只在转身超过 30° 时涨；`POST /v1/vr/usercam/look {"bearing_deg": 90}` 后直播画面转向右边、约 1.5 s 后回到朝前，`orbit.counts.looks` 涨。绕圈（`{"orbit": {"idle": "orbit"}}`）：`GET /v1/vr/usercam` 的 `orbit.active` 为 true、直播画面一圈圈扫过；`/v1/step` 走一步不报 `blocked`、开头停约 200 ms、镜头到了后上方朝前；跟随时转弯后镜头重瞄（`counts.reaims`），走路停顿可以接受；停下 1.5 s 后接着绕。拍照、扫一圈时镜头让开。
5. 读名牌：房间里有别人时 `orbit.sightings.grabbing` 为 true、`reads` 在涨；`GET /v1/vr/usercam/names` 里方位和人实际的方向对得上（对不上先查 `/usercamera/Zoom` 是不是竖直视场，必要时设 `fov_deg`，再查 `lag_ms`）；名牌在直播画面里多大、光环分数分不分得开（[speaker.md](speaker.md) 4.1）。
6. 服务器上 ffmpeg x11grab 连续抓 15 帧/秒的 CPU 占用（`top` 看 ffmpeg 和 vrc-bridge）。
7. 转向说话的人：有人开口后镜头多快转过去（`orbit.attending`）、方位对不对、镜像那一步、光环的起亮时刻在 `GET /v1/speakers` 的候选里是不是 `glow_onset`；说完 1 s 后接着绕。3 s 一圈看起来是否连续（不连续就把 `rate_hz` 调到 60）。
8. **身后的人**（D38，10 分钟以内）：好友站在 bot 正后方 1 m，再到右后方 1.5 m。`POST /v1/vr/usercam/look {"bearing_deg": 180}`（再试 150）：镜头在头后上方（`orbit.last_pose.pose`：比眼睛高约 0.25 m），名牌在直播画面中间；`GET /v1/vr/usercam/names?since_ms=5000` 里他的 `bearing_deg` 和实际差 10° 以内、`distance_m` 差 0.3 m 以内。bot 走动时（例如跟随）看 `orbit.sightings.unsettled` 在涨（刚转的帧被扔掉），`bearing_deg` 不再跳。
9. **闲时一圈和身边的人**（D39，10 分钟以内）：bot 站着不动、不跟随，房间里有两位好友站在 bot 四周。等 60 s 多一点：`GET /v1/vr/usercam` 的 `orbit.sweeping.why` 为 `idle`（快拍一圈约 1 s，D41）、之后 `orbit.last_pose.lens` 回到 `front`；`GET /v1/vr/people` 的 `idle_sweep.sweeps` 加一、`last_names` 大于 0，`people` 里有两人，`distance_m`、`bearing_deg` 和实际对得上（0.3 m、10° 以内），`source` 为 `sweep`。之后好友走动：`source` 变成 `kept`、位置跟着变。一圈转的时候推一下摇杆或说句话：`orbit.sweep_end.how` 为 `move` / `voice`，镜头立刻停。说话时（`GET /v1/speakers` 有进行中的一段）到点：`idle_sweep.skipped` 是 "someone is speaking"，不转。
10. **快拍一圈**（D41，10 分钟以内；部署后先 `POST /v1/vr/usercam {"orbit": {"grab_fps": 15}}`（`usercam.json` 里存的还是 10），再 `POST /v1/follow {"name": "xkeyC"}`）：好友在 bot 前面被跟着，然后快步躲到 bot 身后或侧面的柱子后面再露出来。约 1 s 后 `GET /v1/follow` 的 `state` 为 `searching`、`search_stage` 为 `lens_ring`；`GET /v1/vr/usercam` 的 `orbit.sweeping` 的 `from` 接近他最后的方向、`view` 很快往上涨（每个视角约 150 ms），`reading` 不超过 3；读到他的名字时 `orbit.sweep_end.how` 为 `found`、`last_pose.lens` 为 `look`（镜头切到他那边）、`state` 回到 `following`，从跟丢到找到 3 s 以内。他不在任何视角里时一圈约 1 s 拍完（`done`），然后 `body_turn`。闲时一圈（第 9 项）也是这样快拍。
