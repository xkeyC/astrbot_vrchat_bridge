# 修改点

## 本仓库（feat/full_vr）

| 位置 | 改动 |
|---|---|
| 根目录 | 新增 Rust workspace（`Cargo.toml`），`.gitignore` 加 `/target/` |
| `crates/vrc-vr` | 新增。`pose`：位姿、视场角、由视场角求内参（fx fy cx cy）、按偏航/俯仰构造朝向。`remote`：Monado remote 驱动的 376 字节协议编解码，以及 TCP 客户端 `RemoteHmd`。`tap`：读取抓帧共享内存（seqlock、文件尺寸变化时重新映射、RGBA/BGRA 转 RGB）。6 个单元测试。 |
| `crates/vr-probe` | 新增。命令行工具：`info` / `grab` / `look` / `sweep`（转头后等到按新姿态渲染的帧再存图）/ `walk` / `hands` |
| `crates/vrc-vr` 的 `scan` 模块 | 新增。按给定的一组偏航/俯仰依次转头，每个方向等到按该姿态渲染出的第一帧，结束后回到原来的姿态；`EyeTap::peek` 只读帧头，`Pose::unrotate` |
| `crates/vrc-scene` | 新增。`Panorama::stitch`：用每帧的位姿把多张左眼图拼成等距柱状全景。`HeightMap`：2.5 维栅格，以拟合出的地面为基准，把每个格子分为地面、障碍、高台、未知，并能画成俯视图。 |
| `crates/vrc-vr` 的 `fps` 模块、`scan_pipelined` | 新增。`FpsControl` 读写 Monado 的帧率控制文件；`scan_pipelined` 不等每个方向的画面回来就转到下一个方向，再按位姿从环形缓冲区里认领画面，漏掉的方向最后逐个补拍 |
| `crates/vrc-players` | 新增。名牌 OCR 客户端（infra 的 `/v1/ocr/lines`）、房间玩家名单（VRChat 日志）、名字匹配（和桌面 bridge 同一规则）；自动跟随和 LLM 工具共用 |
| `crates/vrc-nav` | 新增。`survey_pano`（一帧全景的环视：高度图、玩家、物品、候选点、换算成世界米、编号全景图和地图）、`goto`（分段走、每段后重新环视和规划、被挡住时标记障碍） |
| `crates/vrc-bridge` | 新增。Python bridge 的 Rust 移植，见 decisions D20 |
| `crates/vrc-vr` 的 `osc`、`walk` | 新增。OSC 发送（任意参数）和 OSCQuery 读取（眼高、速度）；按角色自身速度计量的一段行走，推着却走不动时判为被挡住 |
| `ops/vr/vrc-bridge.service` | 新增。Rust bridge 的用户单元 |
| `astrbot_plugin/` | 改为只支持 VR：删掉桌面导航工具，新增 look_around、walk_to、step、last_seen；VR 版提示词 |
| `astrbot_plugin/` | 由根目录移入：`main.py`、`vrchat_adapter.py`、`metadata.yaml`、`logo.svg`，内容未改 |
| `legacy/bridge/` | 由 `bridge/` 移入，后来整个删除（已由 `crates/vrc-bridge` 取代，代码可在 git 历史中查到） |
| `third_party/` | Monado、xrizer 的基础提交说明和补丁 |
| `ops/vr/` | 构建脚本（Monado、xrizer）、安装脚本、`vrc-launch.sh`（按 `~/.config/vrc-mode` 切换桌面/VR）、Monado 配置、`openvrpaths.vrpath` 模板、`vrc-monado.service` 用户单元 |
| `crates/vrc-vr` 的 `remote` | 静止时双手的姿势跟着身体朝向放，握持姿态按 OpenXR 规范，手指放松（D21） |
| `crates/vrc-bridge` 的 `follow` | 重写：看和走拆成两个线程，靠自身速度推算距离，刹车平滑；找人时逐个方向看（D22）。`/v1/screenshot` 加 `pitch` 参数 |
| `crates/vrc-vr` 的 `anim`、`remote` | 程序化动画（待机、走路摆臂、说话手势）；`HmdLink` 叠加动画、`hold_still` 扫描时保持静止（D23） |
| `crates/vrc-bridge` 的 `anim` | 动画线程（45 Hz）、bot 语音响度、`/v1/anim` 调参接口；`/v1/vr/height`、`/v1/vr/reset` |
| `astrbot_plugin/` | 新工具 `vrchat_height`、`vrchat_vr_reset`（语音和文字） |
| `tools/mocap/` | 从 CMU 动作捕捉数据统计头和双腕运动的脚本，以及按相位平均的曲线（D25） |
| `crates/vrc-bridge` 的 `follow` | 绕障、跳过矮障碍、卡住时脱困（D25） |
| `crates/vrc-vr` 的 `trackers` | OSC 追踪器（髋、双脚）和头部对齐，Unity 坐标（fbt-research.md） |
| `crates/vrc-bridge` 的 `calibrate` | 全身自动校准：OCR 找按钮、悬停提示二次确认、`TrackingType` 验证；跟随中也会自动校准（agent-vr-use.md 第 5 节） |
| `crates/vrc-vr` 的 `motion`、`crates/vrc-bridge` 的 `motion` | 动作片段格式和片段库、动作程序（编排、淡入淡出、坐和躺的姿势及退出）；`/v1/motion`、`/v1/motion/stop`、`/v1/motion/reload`（motion.md、D30） |
| `crates/vrc-bridge` 的 `anim` | 步态层（走跑循环按速度推进、跳跃收腿、手臂随步态）、脚步层（脚踩原地、转身后踏步跟上）；待机摇晃在跟随移动时收住 |
| `crates/vrc-vr` 的 `scan` | 环视时身体跟着头转，结束后转回 |
| `crates/vrc-bridge` 的 `follow` | 身体正对目标、头看目标（含俯仰）；找人动作重做（左、扫到右、转到身后，每 3 圈抬头一圈）；楼梯当地面跟、读目标脚下高度、不同层时走到身边（D30） |
| `crates/vrc-vr` 的 `walk` | `leg_facing`：朝向不变地后退和横移（`/v1/step` 的 back、left、right） |
| `crates/vrc-nav` | `SurveyOptions::ahead`：只看正前方（加低头一眼）；`/v1/vr/survey` 和 `/v1/vr/goto` 的 `around` 参数 |
| `tools/motion/` | 动作片段的离线生成：`fetch.py`（下载源）、`library.py`（片段表）、`retarget.py`、`postures.py`、`keyframes.py`、`poses/` |
| `tools/agent-vr/` | 操作 VR 菜单、校准和冷重启测试的脚本 |
| `astrbot_plugin/` | `vrchat_emote` 换成 `vrchat_motion`、`vrchat_posture`；走路工具加 `pace`（walk / run）；`vrchat_look_around` 改回 `vrchat_view`（默认环顾四周，`around: false` 只看前方；提示词要求每次移动前先看一圈），`vrchat_walk_to` 走完默认只返回前方画面，`vrchat_step` 转身或走了之后也返回前方画面；转身和方向改成文字参数，返回的方位写成左右；提示词补上探索、上楼梯、坐座位和“不说只做”的规则 |
| `crates/vrc-map` | 持久地图（D31）：体素柱和多层表面、经验层（走过、被挡、跳失败的标记）、物体和命名地点、只估平移的配准、多层 A*、按世界存盘、再次进房时的定位、位置角标的世界坐标系 |
| `crates/vrc-vr` 的 `beacon` | 读化身位置角标（avatar-position-beacon.md） |
| `crates/vrc-players` 的 `objects` | infra 的 YOLO 检测（`/v1/detect/objects`），按全景深度定位（`vrc_nav::pano`）；OCR 和检测共用 HTTP 请求代码（`ocr::post`） |
| `crates/vrc-nav` | `goto` 在持久地图上规划（`GotoOptions::map`、`target_up`），环视写进地图，被挡的一段在地图上记标记 |
| `crates/vrc-bridge` 的 `mapping` | 里程线程（OSCQuery 速度）、建图线程、按世界存取（配置目录下 `maps/`）；`/v1/map`、`/v1/map.png`、`/v1/map/save`、`/v1/map/forget`、`/v1/map/place`、`/v1/vr/beacon`、`/v1/vr/detect`；`/v1/vr/goto` 可传 `place`（地图上的命名地点） |
| 根目录 `Cargo.toml` | workspace 加入 `crates/vrc-audio`、`tools/hrtf-render` |
| `crates/vrc-audio` | 新增（D32）。STFT、语音检测、分段（客户端样本时钟）、按 HRTF 模板投票的方向估计（全圈直方图，前后镜像留给调用方）、按两耳时差对齐的单声道混音、HRIR 表格式 `VRCHRTF1`；球形头模型和 Steam Audio 默认 HRTF 两种模板。18 个单元测试（合成双耳信号） |
| `tools/hrtf-render` | 新增（D32）。运行时加载 Steam Audio SDK 的 `phonon`，渲染默认 HRTF 的 HRIR 表；`assets/hrtf/steam-default-48k.bin`（4.8.1，bilinear）由它生成 |
| `crates/vrc-bridge` 的 `bridge` | 采集改为 float32 双声道，发给客户端的是对齐后的单声道；客户端样本时钟（只数进了队列的样本）；说话人分析在自己的线程上（块带着时钟位置和读到的时刻经有界队列交过去），`speaker` 事件走同一个队列（满时 final 事件重试）；`bot_voice_until`；`idle_later_for` |
| `crates/vrc-bridge` 的 `speaker` | 新增（D32、speaker.md）：每段话的方向投票（按头朝向转到固定坐标系）、视觉线程（自己的抓帧，名牌定位和发光；说话时 OCR 每 1 s 最多一次，只读全景帧）、融合、bot 回声按帧剔除（过半才整段不归属）、`/v1/vr/attend`（`name`、`since_ms` 决定先选哪段；确认失败不算错）、`/v1/speakers`、`/v1/speakers/record`（录制自己的写文件线程，`{"stop": true}`） |
| `crates/vrc-vr` 的 `tap` | `read_unasked`（不续期 `.want` 地读最新帧）、`tapping`（有没有人在让 Monado 抓帧） |
| `assets/hrtf/` | Steam Audio 默认 HRTF 的表，附 `README.md`（NOTICE：来历、修改、版权、CIPIC 声明）和 `LICENSE-Apache-2.0` |
| `crates/vrc-bridge` 的 `anim` | 记下每次实际发给头显的头位姿（`Anim::head`），给说话人追踪算方向用 |
| `crates/vrc-bridge` 的 `vr` | `turn_head_gently`（只转头）、`body_yaw` |
| `crates/vrc-bridge` 的 `follow`、`main` | 跟随、环视、`goto` 读到的名牌送给说话人追踪；环视返回的玩家加 `talking`；新参数 `--speakers`、`--hrtf-table`、`--swap-ears`、`--audio-latency-ms`、`--echo-tail-ms`、`--glow-on`、`--glow-off`、`--glow-release-ms` |
| `tools/agent-vr/` | `doa_sweep.sh`（转一圈的方向扫描，转完明确结束录制）、`doa_sweep_report.py`（扫描结果：声道顺序、nearest/bilinear、和表的误差） |
| 根目录 `Cargo.toml` | workspace 加入 `crates/vrc-pano`（dev 下也按 opt-level 3 编译） |
| `crates/vrc-pano` | 新增（D33）。化身全景的解码：条码 0x5B 和分类（`classify`：全景 / 不能用 / 普通画面）、layout 1 的块和相机（`layout`）、右眼标定格的拟合（`calib`）、`decode` 成 `PanoFrame`（6 个视图：彩色、米制深度、遮罩、内参、旋转；射线、相对头的方位、世界点、地图坐标）、以航向居中的 equirect 和深度全景、拼块图（`render`）。15 个单元测试（合成帧）+ 4 个实测帧测试（`tests/data/`，三张小图约 136 KB；完整截帧用 `VRC_PANO_CAPTURES`，没有就跳过）；`examples/pano_bench.rs` 计时 |
| `crates/vrc-vr` 的 `beacon` | 格子读法抽成 `read_grid`、`block_centre_at`、`field`，PosBeacon 和全景条码共用 |
| `crates/vrc-bridge` 的 `pano` | 新增（D33）。全景的开关（OSC `/avatar/parameters/Pano`，想开时每 5 s 重发）、想要和看到的状态、回退（0.5 s 内没有 0x5B）、普通视角租约（`normal_view`，结束 3 s 后恢复）、最新全景帧（按抓帧 seq 缓存）、后台线程；4 个单元测试 |
| `crates/vrc-bridge` 的 `main`、`bridge`、`calibrate` | 新参数 `--pano`（off、on、auto，默认 off）；`GET/POST /v1/vr/pano`、`/v1/vr/pano.jpg`、`/v1/vr/pano/points`；校准时借用普通视角 |
| `crates/vrc-bridge` 的 `usercam` | 新增（D34、agent-vr-use.md 第 8 节）：VRChat 用户相机当作远程的眼睛。`POST /v1/vr/usercam {"open":true}`（快捷菜单、OCR 找"回出生点"推出相机图标、悬停提示确认后双击、OSCQuery 的 `/usercamera/Mode` 验证；取景器用握把拖进身体；直播模式、UI 遮罩、关飞行模式；抓桌面窗口验证；收尾松手、关菜单、恢复头）、`{"close":true}`、`GET /v1/vr/usercam`、`/v1/vr/usercam/shot`（世界位姿或绕头的方位、距离、高度）、`/v1/vr/usercam/sweep`（一圈拼图）；`keep_open` 时游戏重启或换世界后自己重新打开（`usercam.json`）。参数 `--desktop-grab` |
| `crates/vrc-bridge` 的 `vr`、`main` | `set_squeeze`（握把和后三指），`/v1/vr/hand` 加 `squeeze`（跨调用保持，用来拖动可拾取物）；`release_hands` 也松开握把 |
| `crates/vrc-bridge` 的 `calibrate` | 菜单操作的公用部分（`Log`、`open_menu` 按任意按钮文字、`hover_until` 按任意悬停提示）给 `usercam` 用 |
| `crates/vrc-bridge` 的 `usercam`（合并全景后） | 打开相机时借用普通视角（`pano.normal_view`，要 OCR 菜单） |
| `crates/vrc-vr` 的 `osc` | 移动闸门（D35）：`set_move_gate`，所有 `Osc` 发移动输入（`MOVE_INPUTS`：`/input/Vertical`、`Horizontal`、`Jump`、`Move*`）之前先调它；1 个单元测试 |
| `crates/vrc-bridge` 的 `orbit` | 新增（D35、agent-vr-use.md 第 8 节）。镜头绕圈（站着时每秒 30 个 `Pose`、3 s 一圈，朝外）；有人开口时镜头转向他（`decide`；镜像、保持到说完后 1 s、走路时只看偏离 60° 以上的）；移动闸门里的互锁（站着时的推：行进镜头朝前进方向、150 ms 后关飞行、再 50 ms 放推）；走路时偏离 30° 重瞄（轴先发 0、放镜头、关飞行、轴发回去）；不绕时自己关飞行并检查；镜头读名牌（ffmpeg x11grab 连续抓桌面、按 `Pose` 的历史给帧标镜头、OCR、名字匹配、相对头的方位、光环；交给说话人追踪，存 60 s；转向说话的人时每帧重新量光环）。14 个单元测试 |
| `crates/vrc-bridge` 的 `usercam`、`main`、`bridge`、`mapping` | `usercam.json` 加 `orbit`（`POST /v1/vr/usercam {"orbit": ...}`，`GET` 的 `orbit` 状态）；`GET /v1/vr/usercam/names`；`place` 告诉 orbit 放了什么；`x11grab` 抽出来给连续抓帧用；`Mapping::moved_within`；`/v1/jump`、`/v1/vr/input` 的发送放到阻塞线程（闸门可能等一会儿） |
| `crates/vrc-bridge` 的 `orbit` | 站着时镜头默认静止朝前（`orbit.idle`：`front` / `orbit`，D35 的修改）：行进镜头的位置、头的朝向，偏离 30° 才重放；`look_at`、`name_toward` 看一眼某个方向读名字；`POST /v1/vr/usercam/look`；状态加 `idle_lens`、`looking`、`front_poses`、`looks`；站着时从朝前或行进镜头出发往前走不重放镜头。4 个新单元测试 |
| `crates/vrc-bridge` 的 `usercam` | `POST /v1/vr/usercam {"stow":true}`（不带 `open`）：相机开着时关掉再打开，把取景器拖进身体（`restow`）；全景帧里看得到取景器挡在条带上 |
| `crates/vrc-pano` | 深度只读 E1C（D33 补充）：条码 code（138–139 位）不是 2 就 `Unusable`；标定三个通道各一条曲线、共用暗角，三次插值，被界面盖住的格子不用，残差大时逐格试着剔除（`Calibration::channels`、`covered`、`dropped`）；逐像素校验（容差按"一个字节值多少级"缩放，`PanoParams::check_rg`、`check_b`），不过的和 q = 0 当界面（`MASK_OVERLAY`，外扩 2 px），左眼按视差遮颜色（`MASK_COLOUR`、`PanoFrame::mask_colour`），`PanoFrame::check` 统计；equirect 和点云不用被遮的颜色。删掉 E1、E2 和旧实测帧的测试与小图；合成渲染改成 E1C、每通道调色，新测试：各种界面颜色、被盖的标定格、调色后的校验、非 E1C 被拒。`GET /v1/vr/pano` 的 `frame` 加 `calibration_cells_out`、`vignette`、`check` |
| `crates/vrc-pano` 的 `people` | 全景深度里的人（不用检测器，D36）：`Cloud`（每 4 个像素一个点、脚下的地面）、`person_along`（名牌的射线下面、离地、上沿在名牌下面一点的最近直立物体，旁边没有同深度的面）、`bodies`（像人大小的直立簇，"也许有人"）、`nearest`（按位置接着认）、`Tracking`（世界和追踪空间互换）；合成渲染加盒子（人、家具），`synth` 特性给别的 crate 的测试用。5 个测试 |
| `crates/vrc-nav` 的 `pano` | 一帧全景的环视（`survey_pano`）：高度图和候选点来自深度（每 2 个像素一个点，换到追踪空间），玩家来自调用方给的有名字的人，只有方位的名字和没名字的人另列；检测器在 4 个水平面上找东西、按框里的深度定位；`Survey::pano`、`observations`、`pano_fix`（全景帧里的位置角标）、`panorama` 用全景；`goto` 每段之后怎么看由调用方给（`Look`）。1 个测试 |
| `crates/vrc-bridge` 的 `panolook` | 全景加名字（D36）：镜头读到的名字（`NameSighting` 加了射线 `ray_from`、`ray_dir`）、眼睛上 VRChat 画的名牌（OCR 左眼，按右眼的界面遮罩认出是界面、按眼睛自己的投影成射线）、没名字的人让镜头看一眼（`name_toward`，每次环视最多 2 个）；`surveyor`：有全景帧就用全景环视，否则头部扫描。`/v1/vr/survey` 和 `/v1/vr/goto` 都走它；回复加 `source`、玩家的距离和方位、`named_bearings`、`maybe_people`。2 个测试 |
| `crates/vrc-pano` 的 `people`（实测后） | 人按"前景"认：左右各 0.45 m（绕眼睛转，跨面）、同高和低 0.15 m 处的深度都明显更远（0.15 m 或距离的 5%）；墙、地面、家具侧面都不是。实测帧里用户站在一块板前 0.26 m（2.88 m 对 3.14 m），E1C 在 3 m 处一级 6.6 cm，中间的级把两者连起来，单看深度分不开。"也许有人"还要求前景横向至少 0.18 m 宽（斜看的柜子端头只有一条）。2026-10-09 的 E1C 实测帧剪成 4 块小图（`tests/data/e1c_*.png`，约 140 KB）：条码、校验、名牌整块被遮、地面在眼睛下 1.18 m、用户 2.9 m；bridge 里用真实的眼睛位姿把眼睛上的名牌换成射线，找到下面的用户 |
| `crates/vrc-bridge` 的 `follow` | 全景时（D36）：一次取景就是最新一帧全景（`look_pano`，不转头、不做双目）；目标是读到名字的人，否则是离上次位置最近的像人的东西（门限 0.6 m 加每秒 1.5 m，4 s 内，不是别人名字的）；眼睛上的名牌每 0.8 s（有人说话时 0.4 s）OCR 一次；看路（走廊、地图绕行、沿墙、跳）抽成 `steer`，全景时每个方向都在视野里、不扫视；跟丢时先转向最后读到名字的方向、再转向最后的位置，每次让镜头也看那边（`find_pano`、`lost_ways`），不再一圈圈找。状态加 `seen_by`（stereo / named / kept）。3 个新测试（含两帧合成全景：先读名字，再按位置接着认） |
| `crates/vrc-bridge` 的 `sightings`、`speaker` | 全景时（D36）：好友目击用镜头和眼睛上的名牌读名字、全景深度定位，存的画面是全景里朝他那边的透视视图（只有方位的也朝那边）；说话人追踪的视觉线程在全景帧上不做双目，改用 `panolook` 读名牌、定位（`read_plates_pano`），两次 OCR 之间仍按位置把名牌投影到眼睛里量光环（眼睛上的名牌就画在正常视角的位置） |
| `crates/vrc-bridge` 的 `main`（看到什么） | 全景时 `GET /v1/screenshot` 是全景里朝头的方向的透视视图（100°、正方形，`pitch` 抬低视线，头不动）；`normal=1` 借普通视角租约拍眼睛 |
| `crates/vrc-bridge` 的 `pano`、`main` | `--pano` 默认 `auto`（D36）：在世界里才开（`run` 读游戏状态，`set_in_world`），化身 3 s 内没出 0x5B 就回退、进下一个世界时马上再问；`hold_normal`：`/v1/vr/hand`、菜单按钮、`screenshot?normal=1` 让普通视角保持 20 s；状态加 `in_world`、`held_normal_ms`。2 个新测试 |
| `crates/vrc-pano` 的 `render` | `perspective`：从全景里取一个透视视图（任意偏航、俯仰、视场），界面的颜色用别的面补；1 个测试。`vr::view_jpeg` |
| `astrbot_plugin/` | 环视的文字加"认出名字但不知道距离"和"也许有人" |
| `crates/vrc-bridge` 的 `speaker` | 名牌光环按 2026-10-08 的录像重做（D35、speaker.md 2.3）：沿胶囊找深色底的边缘、量边缘外的带子（和颜色无关）、两侧波纹；`View`（眼睛的帧或 RGB 图）；一帧就亮，眼睛和镜头各一份安静样子；融合按起亮时刻对齐、去掉 0.9 s 的尾巴（`cues.glow_onset`）；只有方位的候选（`saw_bearing`）；给镜头用的 `voice`、`saw_glow`、`plate_lit`；`--glow-on/off/release-ms` 默认改成 0.30 / 0.15 / 900。合成胶囊（不亮、金色、青色）、两张实测截图（`tests/data/`，约 124 KB）、起亮对齐、给镜头的方位等 7 个新测试，完整录像用 `VRC_GLOW_RECORDING` |
| `crates/vrc-bridge` 的 `follow`（D37） | 绕障的看守 `Avoid`：一次绕障算一段（断开不到 2 s 不算结束），目标每帧照样定位，挪 0.75 m 或每 1.5 s 按目标现在的位置重新规划（沿墙时问地图有没有新路）；15 s 没进展或整段 30 s 就重新定位（腿站住，全景读名牌、镜头看最后读到名字的方向和最后的位置），下一帧换边绕或走地图；连续 3 次就面向目标站 8 s；绕的时候 6 s 看不见算跟丢。去掉会自己续上的 `WALL_FOR` 和 `WALL_UNSEEN`。找人时（seek）也看路。绕的时候镜头少重瞄、3 s 没读到名字就看目标一眼。跟丢后先用镜头：看最近读到的名字、镜头转一圈，再转身体、最后转 4 个方向（`search_stage`）。`GET /v1/follow` 加 `avoid`、`search_stage`。3 个测试（看守的时间线、合成全景的整段、找法顺序） |
| `crates/vrc-bridge` 的 `orbit`（D37） | `sweep`：镜头绕头转一圈（不管 `idle`），一推或有人开口立刻停，`end_sweep`、`sweep_end`；`calm_travel`：一段时间内行进镜头 60°、3 s 才重瞄；状态加 `sweeping`、`sweep_end`、`calm_ms`、`counts.sweeps`。2 个测试 |
| `crates/vrc-bridge` 的 `bridge` | 测试用的 `test_bridge()` |
| `crates/vrc-bridge` 的 `orbit`（D38） | 镜头读名牌的定位：头的读数留历史（走路时 100 ms 一次），镜头的真实位置 = `Pose` 的位置 + 之后头的位移；不动的镜头 `Pose` 稳了 `settle_ms`、画面和上一帧相像才读（`sightings.unsettled`）；读到名字时沿射线在全景深度里找人，方位和距离相对头（`NameSighting.feet`、`distance_m`），没有深度才按 2.5 m；`look_at` 的镜头在头后上方（`orbit.look`）。4 个测试 |
| `crates/vrc-bridge` 的 `people`、`panolook`、`orbit`、`main`、`bridge`（D39） | 身边的人的缓存：每次看全景按名牌射线和深度放有名字的人、8 s 内按位置接着认没读到名字的（交给说话人追踪），150 s 删；闲着时每 `orbit.idle_sweep_s`（60）镜头转一圈读四周的名牌（一推、有人开口立刻停），之后看一帧全景放进缓存；`GET /v1/vr/people`；`LookOptions.source`；`Bridge::follow_paused`。2 个测试 |
| `crates/vrc-bridge` 的 `follow`、`orbit`、`people`、`main`（D40） | 按位置接着认有期限：最后读到名字 `kept_s`（默认 7 s，`POST /v1/follow {"settings": {"kept_s": 7}}`，3–30）以后不再认，2.5 s 没读到名字就让镜头 `look_at` 被认的那个（至少隔 3 s），读到他的名字算确认，读到别人的名字或读了 3 次都没有名字（8 m 以内）就丢掉、立刻算跟丢（`Confirm`、`Track::drop_kept`）；跟丢先让镜头看最后读到名字的地方（`lens_look`），全景时站着 2 s 没看到就算跟丢；镜头读到目标的名字（任何镜头）而这一帧深度里没找到人、或者人在别处：按深度里的人、读名时的落脚点、或方位加上次的距离直接采用（`Track::place`、`lens_place`，`seen_by: "lens"`）；俯仰按名牌高度（深度或镜头射线）平滑、限幅（`smooth_pitch_step`）。镜头：跟随时站着的镜头朝目标（`Orbit::set_following`、`aim_at`），偏 30° 重瞄、20 s 没放过就重放（`follow_stale_s`）；刚停下（都松开了）也可以放镜头（`may_front`）；`settle_sure_ms`（600 ms）以后不再要求画面和上一帧相像；`end_sweep(why)`（`stopped`）；`reads_since`。身边的人：15 s 没读到名字不再按位置接着认。`GET /v1/follow` 加 `confirm`、`head_pitch_deg`、`settings`；`GET /v1/vr/usercam` 的 `orbit` 加 `following`、`aim`、`counts.target_poses`、`counts.stale_poses`。9 个新测试 |
| `crates/vrc-bridge` 的 `orbit`、`follow`、`people`（D41） | 镜头找人改成固定视角快拍：`Orbit::sweep(why, from, seek)` 依次放 `snap.views`（6）个视角（`Lens::Snap`），读名线程拿到第一张显示这个视角的画面（离 `Pose` ≥ 100 ms 且和放 `Pose` 前那帧不同，或 350 ms）就放下一个，OCR 在单独线程里读（最多 3 个同时）；找人时读到他的名字立刻结束、镜头 `look_at` 过去（`snap_hit`）；跟随时第一个视角对着最后看到他的方向，其余左右交替往外。跟丢 1 s（全景站着、走着都一样；原来 1.5 / 2 s），找法 `lens_ring` → `body_turn` → `scan`；确认 1 s（原来 2.5 s）。不动的镜头换了 `Pose` 后第一张稳了的画面不等 OCR 节拍；`grab_fps` 默认 15；`name_toward` 30 ms 看一次。设置 `orbit.snap`；状态 `orbit.sweeping` 的 `seek`、`from`、`view`，`orbit.reading`，`counts.snap_poses`。6 个新测试 |
| `crates/vrc-stereo` 删除；`vrc-nav`、`vrc-players`、`vrc-scene`、`vrc-bridge`、`vr-probe`、插件（D42） | 删掉双目和转头扫描：`vrc_nav::survey`、`vrc_players::sightings` / `objects::place`、`vrc_scene::mirror` / `Panorama::stitch`、跟随的 `look` / `search` / `glance`、说话人的普通视角 OCR + 双目、"最后看到"的普通视角 OCR、`POST /v1/vr/detect`、`vr-probe depth/scan/goto`、`cudarc`。环视、跟随、认人只用全景，没有全景时报错或原地等；跟丢一律 1 s；`/v1/vr/corridor` 改用全景深度；全景物体定位补上 `Detection::kept` 过滤；插件去掉 `around`。服务器 `bridge_deploy.sh` 按 `.deployed-files` 删掉不再部署的文件 |
| `crates/vrc-map` 的 `nav`、`examples/map_info`；`vrc-nav`、`vrc-bridge` 的 `panolook`、`follow`（D43） | 轨迹一段超过 0.6 m（位置跳变：重生、传送、角标读错、修正）不再删掉玻璃标记；全景的一眼带取帧时刻（`taken`），跟随和环视按它给角标定位、放地图和物体；`map_info` 列出地图文件的内容。1 个新测试 |
| `crates/vrc-bridge` 的 `vr`、`anim`（D44） | 走到命名地点的最后一段：离得远、偏得多时先转向再走（不再侧移平移过去）；步态速度用地面速度（`VelocityZ`、`VelocityX`），侧移也迈腿。1 个新测试 |
| `docs/full-vr/` | 本文档集 |

## 其他项目

### Monado：`third_party/monado/patches/0001-null-compositor-frame-tap.patch`

- 新增 `src/xrt/compositor/null/null_tap.c`、`null_tap.h`。
- `null_compositor.c`：初始化时调用 `null_tap_create`（只有设置了 `XRT_NULL_TAP` 才启用）；`layer_commit` 里调用 `null_tap_layers`；销毁时释放。
- `null_compositor.h`：`struct null_compositor` 增加 `tap` 成员。
- `CMakeLists.txt`：加入新文件。
- 共享内存格式（小端）：
  - 0：`"MNDTAP01"`
  - 8：seq（写入中为奇数）
  - 16：frame_id
  - 24：display_time_ns
  - 32：capture_ns
  - 40：每只眼的宽、高
  - 48：VkFormat、每像素字节数
  - 56：两只眼各 11 个 f32（视场角 4 个；位姿 7 个：朝向 xyzw、位置 xyz）
  - 144：每只眼在数组中的下标
  - 152：层类型
  - 512 起：像素，先左眼后右眼

### Monado：`third_party/monado/patches/0002-remote-hmd-full-view-fov.patch`

- `src/xrt/drivers/remote/r_hmd.c`：
  - 新增 `r_hmd_get_visibility_mask`：隐藏网格为空，可见网格是整个视野的四边形，轮廓是矩形；
  - 水平视场改为读 `XRT_REMOTE_FOV_DEG`（默认 85）。
- 配置：`ops/vr/config_v0.json` 改为 2560x1280 像素、0.12x0.06 米，也就是每只眼 1280x1280 的正方形；`vrc-monado.service` 加上 `XRT_REMOTE_FOV_DEG=100`。

### Monado：`third_party/monado/patches/0003-null-compositor-runtime-fps.patch`

- `u_pacing.h` / `u_pacing_compositor_fake.c`：新增 `u_pc_fake_set_frame_period`，运行中修改假节拍器的帧间隔，合成时间按原规则重新计算。
- `null_compositor.c/.h`：如果设置了 `XRT_NULL_FPS_FILE`，就映射一个 8 字节的控制文件（[0] 请求的帧率，0 表示回到默认值；[1] 当前帧率）；每次预测帧之前检查一下，请求变了就改节拍（上限 240）。

### Monado 补丁 0001 第二版：抓帧改为环形缓冲区

- 文件头 `MNDTAP02`：槽数（`XRT_NULL_TAP_SLOTS`，默认 8）、槽大小、已写帧数；每个槽是原来的 512 字节帧头（`MNDSLOT1`）加像素。帧头的 seq 等于 2n，表示第 n 帧，写入中为奇数。
- Rust 端：`EyeTap::written`、`peek_all`（所有槽的帧头）、`read_seq`（按 seq 读取，帧已被覆盖时返回 None）。

### xrizer：`third_party/xrizer/patches/0001-swapchain-transfer-src.patch`

- `src/graphics_backends/vulkan.rs`：眼睛交换链的用法加上 `TRANSFER_SRC`。

## 服务器（脱敏）

| 改动 | 回退方式 |
|---|---|
| AstrBot：`vrchat` 平台 `enable=false`，插件 `astrbot_plugin_vrchat` 加入停用列表 | 改动前已备份 `cmd_config.json` 和数据库，恢复备份或在 WebUI 重新启用 |
| bot 用户的 `vrc-bridge` 用户单元：停止并禁用开机启动 | `systemctl --user enable --now vrc-bridge` |
| 整机 `pacman -Syu`（36 个包，内核和 NVIDIA 驱动未变），新装 cmake、ninja、eigen、vulkan-headers、openxr、clang | 不需要回退 |
| bot 用户：rustup stable 工具链 | 不需要回退 |
| 构建目录 `<VR_ROOT>`：monado（含补丁）、prefix（安装产物）、xrizer（含补丁）、fullvr（本仓库的 Rust 代码） | 删除该目录即可 |
| `~/.config/monado/config_v0.json`（remote 驱动）、`~/.config/openvr/openvrpaths.vrpath`（指向 xrizer）、`~/.config/vrc-mode` = `vr` | 删除，或把 vrc-mode 改成 `desktop` |
| `/usr/local/lib/vrc/vrc-launch.sh`；VRChat 的 Steam 启动参数改为 `/usr/local/lib/vrc/vrc-launch.sh %command%`（改的是 `localconfig.vdf`，改前已停 Steam 并备份为 `.pre-vr`） | 恢复 `.pre-vr` 备份（Steam 停止状态下） |
| `monado-service` 目前以 `systemd-run` 临时单元运行（正式用户单元见 `ops/vr/vrc-monado.service`，尚未安装） | `systemctl --user stop vrc-monado` |
| `panolook`、环视、插件（实测后） | 关掉"也许有人"（全是误报）：环视不再输出 `maybe_people`，插件不再说"Maybe someone"，镜头不再去看没名字的人形（`LookOptions.ask` 去掉）。 |
