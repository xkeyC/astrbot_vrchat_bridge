# 修改点

## 本仓库（feat/full_vr）

| 位置 | 改动 |
|---|---|
| 根目录 | 新增 Rust workspace（`Cargo.toml`），`.gitignore` 加 `/target/` |
| `crates/vrc-vr` | 新增。`pose`：位姿、视场角、由视场角求内参（fx fy cx cy）、按偏航/俯仰构造朝向。`remote`：Monado remote 驱动的 376 字节协议编解码，以及 TCP 客户端 `RemoteHmd`。`tap`：读取抓帧共享内存（seqlock、文件尺寸变化时重新映射、RGBA/BGRA 转 RGB）。6 个单元测试。 |
| `crates/vr-probe` | 新增。命令行工具：`info` / `grab` / `look` / `sweep`（转头后等到按新姿态渲染的帧再存图）/ `depth`（双目深度：左眼和深度并排的 PNG、点云 PLY、几个位置的距离、地面拟合） |
| `crates/vrc-stereo` | 新增。纯 Rust 实现的 SGM：7x7 census、8 方向聚合、亚像素、唯一性检查、左右一致性检查（rayon 并行）。`Stereo` 从抓帧构造（可缩小分辨率），输出深度和可追踪空间里的点云；`fit_floor` 用高度直方图找地面再做平面拟合。3 个单元测试，包括一个合成的双平面场景。 |
| `crates/vrc-vr` 的 `scan` 模块 | 新增。按给定的一组偏航/俯仰依次转头，每个方向等到按该姿态渲染出的第一帧，结束后回到原来的姿态；`EyeTap::peek` 只读帧头，`Pose::unrotate` |
| `crates/vrc-scene` | 新增。`Panorama::stitch`：用每帧的位姿把多张左眼图拼成等距柱状全景。`HeightMap`：2.5 维栅格，以拟合出的地面为基准，把每个格子分为地面、障碍、高台、未知，并能画成俯视图。 |
| `vr-probe scan` | 新增。头部扫描后输出全景图和高度图；`--boost N` 扫描期间临时提高帧率，扫完恢复原帧率；`--hold-ms` 改用流水线扫描 |
| `crates/vrc-vr` 的 `fps` 模块、`scan_pipelined` | 新增。`FpsControl` 读写 Monado 的帧率控制文件；`scan_pipelined` 不等每个方向的画面回来就转到下一个方向，再按位姿从环形缓冲区里认领画面，漏掉的方向最后逐个补拍 |
| `crates/vrc-players` | 新增。名牌 OCR 客户端（infra 的 `/v1/ocr/lines`）、房间玩家名单（VRChat 日志）、名字匹配（和桌面 bridge 同一规则）、名牌的双目定位；自动跟随和 LLM 工具共用 |
| `crates/vrc-nav` | 新增。`survey`（环视、双目、高度图、玩家、候选点、换算成世界米、编号全景图和地图）、`goto`（分段走、每段后重新环视和规划、被挡住时标记障碍） |
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
| `crates/vrc-bridge` 的 `follow` | 绕障、跳过矮障碍、卡住时脱困；跟随的双目匹配限 6 线程（D25） |
| `crates/vrc-vr` 的 `trackers` | OSC 追踪器（髋、双脚）和头部对齐，Unity 坐标（fbt-research.md） |
| `crates/vrc-bridge` 的 `calibrate` | 全身自动校准：OCR 找按钮、悬停提示二次确认、`TrackingType` 验证；跟随中也会自动校准（agent-vr-use.md 第 5 节） |
| `crates/vrc-vr` 的 `motion`、`crates/vrc-bridge` 的 `motion` | 动作片段格式和片段库、动作程序（编排、淡入淡出、坐和躺的姿势及退出）；`/v1/motion`、`/v1/motion/stop`、`/v1/motion/reload`（motion.md、D30） |
| `crates/vrc-bridge` 的 `anim` | 步态层（走跑循环按速度推进、跳跃收腿、手臂随步态）、脚步层（脚踩原地、转身后踏步跟上）；待机摇晃在跟随移动时收住 |
| `crates/vrc-vr` 的 `scan` | 环视时身体跟着头转，结束后转回 |
| `crates/vrc-bridge` 的 `follow` | 身体正对目标、头看目标（含俯仰）；找人动作重做（左、扫到右、转到身后，每 3 圈抬头一圈）；楼梯当地面跟、读目标脚下高度、不同层时走到身边（D30） |
| `crates/vrc-nav` | `SurveyOptions::ahead`：只看正前方（加低头一眼）；`/v1/vr/survey` 和 `/v1/vr/goto` 的 `around` 参数 |
| `tools/motion/` | 动作片段的离线生成：`fetch.py`（下载源）、`library.py`（片段表）、`retarget.py`、`postures.py`、`keyframes.py`、`poses/` |
| `tools/agent-vr/` | 操作 VR 菜单、校准和冷重启测试的脚本 |
| `astrbot_plugin/` | `vrchat_emote` 换成 `vrchat_motion`、`vrchat_posture`；走路工具加 `pace`（walk / run）；`vrchat_look_around` 改回 `vrchat_view`（默认只看前方，`around` 可选并提示少用），`vrchat_walk_to` 走完默认只返回前方画面 |
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
