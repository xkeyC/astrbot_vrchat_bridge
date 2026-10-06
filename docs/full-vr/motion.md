# 动作系统：用 OSC 追踪器接管全身（2026-10-06）

bot 的全身动作全部由 bridge 通过 VRChat 原生 OSC 追踪器（髋、双脚，以及头部对齐）驱动：动作片段库、走跑步态、坐和躺等姿势、站着转身时的踏步，以及跟随里的看人和找人。追踪器接入和自动校准见 [fbt-research.md](fbt-research.md)、[agent-vr-use.md](agent-vr-use.md) 第 5 节；为什么这样做见 [decisions.md](decisions.md) D30。

## 1. 整体结构

```text
tools/motion/   (Python，离线)   开源动捕 / 关键帧  ──重定向──>  片段 JSON
                                                                │ 复制到 bridge 配置目录的 motions/
crates/vrc-vr/src/motion.rs        片段格式、采样、混合、按头显位姿放置
crates/vrc-bridge/src/motion.rs    片段库、动作程序（编排、淡入淡出、姿势和退出）
crates/vrc-bridge/src/anim.rs      动画线程（45 Hz）：动作程序 / 步态 / 脚步层 → 追踪器
crates/vrc-bridge/src/follow.rs    跟随：身体朝向、头部看人、找人动作、楼梯
```

每一拍（22 ms）动画线程按优先级选一种身体来源：

1. **动作程序**（`/v1/motion` 或 LLM 工具在播）：片段给出全部 11 个关节。
2. **步态**（bot 在移动）：走、跑循环按 VRChat 报的速度推进，手臂也取自同一个循环。
3. **站立**：待机动画 + 脚步层（脚踩在原地，身体转动后再踏步跟上）。

结果按头显位姿放进追踪空间，发出髋、双脚三个追踪器。头部对齐用的是头显的真实位姿（发别的值，VRChat 会让追踪器跟着头部晃动）。

## 2. 片段格式

`crates/vrc-vr/src/motion.rs` 读 JSON：

- `fps`、`frames`：每帧 11 个关节（`hips, chest, head, l_leg, r_leg, l_foot, r_foot, l_forearm, r_forearm, l_hand, r_hand`），每个关节 7 个数：位置（以身高为单位）和四元数，都是相对**标准站姿**（`standing`）的。身体坐标：x 向左，y 向上，z 向前。
- 可选字段：
  - `curls`：手指弯曲的增量（每帧 10 个）；
  - `root`：`travel`（片段自己会移动）、`in_place`（原地播）、`hold`（停在最后的位置，接下一个片段）；
  - `exit`：离开这个姿势时播的片段（例如 `sit_down` 的退出片段是 `stand_up`）；
  - `posture`：`sitting` 或 `lying`，同一姿势的片段之间直接衔接；
  - `speed`：步态循环里着地脚相对髋向后的速度（身高/秒），用来让步幅和移动速度一致；
  - `loop`、`license`。

## 3. 生成片段：`tools/motion`

```text
python tools/motion/fetch.py DATA_DIR            # 下载 CMU、万代南梦宫的源文件（Quaternius 要手动下载）
python tools/motion/library.py DATA_DIR OUT_DIR [--preview] [--only wave,turn]
```

源数据**不进仓库**，生成的片段带 `license` 字段：

| 来源 | 许可 | 用在 |
|---|---|---|
| CMU Graphics Lab Motion Capture（una-dinosauria/cmu-mocap 的 BVH） | 免费使用和修改，不得转售数据本身 | 挥手、转身、后空翻、躺下/起身、小鸡舞 |
| Bandai Namco Research Motion Dataset 1 | CC BY-NC 4.0（非商用） | 再见、走跑步态循环、短舞 |
| Quaternius Universal Animation Library 1、2（Standard） | CC0 | 待机、说话、抱臂、点头、摇头、坐下/起立、舞蹈 |
| 关键帧（`tools/motion/poses/*.json`、`postures.py`） | 本仓库 | 拒绝、思考、东张西望、跳跃收腿、二郎腿坐姿、四种躺姿 |

处理步骤：

- **规范骨架**（`skeleton.py`）：按 Drillis & Contini 的人体比例；源动作先归一化（面朝 +z、识别镜像），再按骨骼方向对齐（带扭转参考），参考姿势是 T-pose 或片段里的一帧站姿。
- **脚**（`retarget.py`）：俯仰按站立帧里脚放平时的角度算相对值；朝向跟髋走，去掉源动作的外八，只保留一半再加 2° 内扣；横滚跟小腿。VRChat 会按脚追踪器的轴向决定脚的朝向和膝盖的弯曲，所以这一步决定了腿看起来对不对。
- **站距**：整条腿绕髋收窄到 0.2；步态循环里两脚踝至少相距 0.15 个身高（脚追踪器在踝内侧，太近会交叉）。
- **不穿地**：每个关节按半径抬到地面以上；地面取整段动作里站立帧的最低点。
- **步态循环**（`convert.py` 的 cycle）：从动作稳定的中段取一个完整步幅（左脚两次落地之间），速度用着地脚相对髋的后移速度（中位数），不用髋的位移（规范骨架的腿长和源不同，按髋位移算会让步幅和速度对不上）。
- CMU 有一部分是 60 fps 但文件头写 120，名单在 `cmu_60fps.txt`；CMU 第 0 帧常是位置错乱的 T-pose，截取时从它后面开始。

部署：把 `OUT_DIR/*.json` 复制到 bridge 配置目录下的 `motions/`，再 `POST /v1/motion/reload`。

## 4. 播放：`/v1/motion`

| 接口 | 作用 |
|---|---|
| `GET /v1/motion` | 片段列表，以及正在播的程序 |
| `POST /v1/motion` | 播一个片段 `{"clip":"wave","mirror":false,"speed":1,"seconds":5,"in_place":false,"fade":0.6}`，或一串 `{"steps":[...]}`，或一个姿势 `{"posture":"lie","way":"left"}`（stand / sit / lie：back、left、right、front）。默认立即返回，`"wait":true` 时播完再返回；新程序会停掉正在播的 |
| `POST /v1/motion/stop` | 停下；保持中的姿势先播退出片段 |
| `POST /v1/motion/reload` | 重新读 `motions/` |

- 片段之间交叉淡入淡出 0.6 s；回到站立淡出 3 s（缓入缓出）；退出片段 1.5 s。循环片段默认播 5 s，最长 600 s。
- 姿势：`sit_down → sitting`（保持）`→ stand_up`；躺：`lie_down → lying_back/left/right/front`，侧躺和趴着先翻回仰躺再起身。已经躺着时换方向直接翻身。
- 其他会动身体的接口（走、转、环视、跟随、手、头）会先停掉动作程序，等它松开头显再动（`stop_and_wait`）。
- LLM 工具：`vrchat_motion`（wave、bye、byebye、nod、shake_head、refuse、think、look_around、turn、backflip、dance、dance_short、chicken_dance、idle_talk、fold_arms；可选另一只手、次数、时长），`vrchat_posture`（坐、躺、站）。

### 二郎腿坐姿（`postures.py` 的 `sit_cross`）

按用户要求逐条调过：

- 双手撑在两侧的座面上；
- 身体左右轻晃：3 s 晃到最右，停 1 s，3 s 晃到最左，停 1 s；
- 上面那只脚轻轻晃，3 s 一个周期；
- 每 30–120 s（随机）换一次腿，换腿 2 s 完成，腿是抬起伸展着跨过去的（不穿模），交叉后的姿势在原姿势基础上只做了微调。

### 躺姿

仰躺、左侧、右侧、趴着四种，都有呼吸起伏，趴着时小腿会晃；全部抬高一点，不陷进地里。

## 5. 步态和脚步（`crates/vrc-bridge/src/anim.rs`）

- **走跑**：移动速度（VRChat 报的速度，世界米/秒）超过 0.25 m/s 时淡入步态（0.35 s，变速同样）。走循环和跑循环按速度在 1.1–1.4 m/s 之间混合；相位按 速度 ÷ 循环步长 推进，所以脚不会在地上滑。腿按原幅度播，髋的摆动和扭转只保留 45%（头显不跟着晃）。
- **跳**：`Grounded` 为 false 时混入 `jump_air` 收腿（0.12 s 收、0.25 s 放）。
- **站着转身**：脚踩在原地，先原地拧着跟上髋的朝向（最多差 40°，脚尖向内拧最多 15°，免得交叉）；身体转过 15° 以上（或脚偏离 6 cm 以上）后，转向那一侧的脚先迈，0.2 s 一步，抬脚 5 cm，绕身体中心画弧，一步最多 60°。落脚离另一只脚不小于站姿间距的 70%，不会重叠；髋最多落后身体 15°。
- **待机**：跟随在动（不是空闲）时，摇晃和重心转移逐渐停下；摇晃幅度为原来的 75%。待机晃头时脚不动。
- VRChat 设置里"全身追踪时启用运动动画"已**关闭**（用菜单自动化关的），腿完全由这里驱动。

## 6. 跟随里的身体和头（`crates/vrc-bridge/src/follow.rs`）

- **身体正对目标**：走路时身体朝前进方向；站着时目标偏离超过 20° 才转身，转到 3° 以内。
- **头看人**：小偏离只转头，最多偏离身体 35°，150°/s。俯仰对准名牌下方 0.3 m（大约是脸），限制在 −30°～30°，走路时也保持；离得近时会抬头，名牌不会跑出视野。贴墙或绕障时头的偏航看前进方向，俯仰仍对着目标的高度。
- **找人**（丢失 1.5 s，站着时 4 s）：以最后看到的方向为中心，0.35 s 转到左 70°、看；0.6 s 一口气扫到右 70°，经过中间时微微抬头 6°、看；0.7 s 转到身后、看；再以身后为中心重复。头随转向倾斜 8°，每次看之前停 80 ms；双手比静止位置稍靠后，减少遮挡视野。每一组算一圈，第 2、5、8…圈抬头 20° 看（找站在高处的人），结束时 0.3 s 内放平。环视扫描时身体跟着头转。
- **楼梯**：
  - 目标脚下的高度取名牌正下方 0.25 m 半径内的双目点（至少 15 个）的 5% 分位，相对 bot 脚下；
  - 目标和 bot 不在同一层（差超过 0.3 m）时，跟随距离缩到 0.8 m，走到目标身边，而不是在楼梯口停下；
  - 走廊检测里的地面每 10 cm 最多跟着升降 0.22 m（一级台阶），障碍按"高出该处地面一个台阶以上"判断，所以楼梯被当成地面，楼梯顶的墙仍会被认出来。

## 7. 待做

- MMD（VMD）舞蹈导入：调研和测试素材已有，导入器还没写。
- 跑步循环的右脚平均仍有约 14° 外八。
- 跟随时偶尔跳得太频繁（障碍判定），和步态无关（开关步态对比过，都是约 3 次/分钟）。
- 楼梯跟随还需要在真实楼梯上实测。
