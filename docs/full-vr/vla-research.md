# 低层学习策略调研（2026-10-05）

> 由调研代理整理，标"估"的数字都没有在 3060 上实测。结论已经收进 [decisions.md](decisions.md) 的 D12。


## 1. 结论先行

- **你们对 SmolVLA / RDT / Octo 的判断基本成立。** SmolVLA 预训练数据是 487 个 LeRobot 社区数据集（以 SO-100 机械臂为主），输入包含机器人本体状态，输出为关节动作块（https://huggingface.co/blog/smolvla）。Octo-Small/Base（27M/93M）用 Open X-Embodiment 中 25 个数据集、80 万条轨迹训练，定位是“机器人操作的通用策略”（https://arxiv.org/abs/2405.12213）。RDT-1B/170M 的 46 个预训练数据集（RT-1、DROID、Bridge 等）也都是操作数据（https://huggingface.co/robotics-diffusion-transformer/rdt-170m）。需要补充一点：RDT 的统一动作空间里**留有轮式底盘速度的维度**（https://arxiv.org/html/2410.07864v2），但数据以机械臂为主，没有可用的导航先验。所以这几个模型做导航只能当作“预训练骨干”，要大量微调，不值得。
- **唯一的例外是 CrossFormer（130M，MIT，有权重）**：训练数据里混入了导航、四足和无人机（https://github.com/rail-berkeley/crossformer）。不过 2026 年一项真机零样本评测发现，CrossFormer、GNM、ViNT、NoMaD、NaviBridger 都存在**频繁碰撞、几何理解弱、外观相似场景下认错目标、对分布变化敏感**的问题（https://arxiv.org/abs/2603.25937）。这条证据直接支持“几何为主，学习策略为辅”的路线。
- **适合我们的学习模块只有两类：** ① 小型导航基础模型（NoMaD、ViNT、OmniVLA-edge、S2E），可以作为“候选轨迹/方向提议器”；② 小型跟随 VLA（OmTrackVLA-0.6B）。7B 以上的 VLN-VLA（NaVILA、Uni-NaVid、StreamVLN、InternVLA-N1、OmniVLA-7B）在 3060 上放不下，或者只能跑到约 1 Hz，而且其中多个是 CC BY-NC-SA 许可。游戏智能体方面，NitroGen 的权重已开放，但没有目标条件，零样本跑 Quake 完全失败，不建议采用。
- **目前没有任何模型直接接收双目输入。** 能用深度的有 NavDP（RGB-D）和 ViPlanner/iPlanner（深度＋语义）。这类模型的输出本质上就是“局部路径”，和我们自己的高度图规划器重叠，零样本价值不大。

## 2. 候选对比表

VRAM 和速度凡标“估”的，都是按参数量和公开延迟推算的，**没有在 3060 上实测**。

| 模型（机构，发布） | 参数 | 输入 / 目标 | 输出 | 训练域 | 许可 / 权重 | 3060 估计 | 风格化世界零样本 / 微调 | 适配分 |
|---|---|---|---|---|---|---|---|---|
| **NoMaD**（UC Berkeley，2023-10）[1][2] | 约 19M | 单目 96×96，3 帧上下文；目标为图像，也可屏蔽目标做探索 | 8 个相对路点（扩散 10 步） | 真机：RECON、SCAND、GoStanford、SACSoN 等 | MIT，有权重 | <0.5 GB，>10 Hz（估） | 差到中：真实相机域，碰撞多 [3]；可用自录数据按 GNM 方式“未来帧当目标”自监督微调 | **3** |
| ViNT / GNM（同上，2023/2022）[1] | 约 16M / 更小 | 单目＋图像目标 | 路点 | 同上 | MIT，有权重 | 同上 | 同上，且没有探索模式 | 2 |
| **OmniVLA-edge**（Berkeley/Toyota，2025-09，ICRA'26）[4][5] | 50M（EfficientNet-B0 + CLIP） | 单目；目标可以是语言、图像或 2D 位姿，并可组合 | 速度或轨迹 | 9500 小时、10 种平台的真机数据 | MIT，HF 有 omnivla-edge | <1 GB，>10 Hz（估） | 中：能直接用语言目标（如 “go to the red door”），但属真实域 | **3** |
| OmniVLA（同上） | 8.27B（OpenVLA） | 同上 | 同上 | 同上 | MIT，有权重 | 4-bit 约 5–6 GB，1–2 Hz（估） | 与 VRChat 抢显存 | 1 |
| S2E（UCLA，ICLR'26）[6] | 未公开（DINOv3 编码器） | 11 帧 RGB，点目标 (x, y) | 10 个路点或 (v, ω) | 网络视频 BC，论文里另做了 URBAN-SIM 强化学习 | 有 ONNX 权重，但只放了 BC 版；许可未核实 | 约 1 GB（估） | 中：网络视频的多样性较好；模型库里统一了 GNM/ViNT/NoMaD/CityWalker/MBRA 的接口 | 3 |
| CityWalker（NYU，CVPR'25）[7] / LogoNav-MBRA（Berkeley，2025）[8] | 未核实 | 单目，点目标 / 图像或 GPS 目标 | 路点 | 城市行走视频、众包遥操作数据 | Apache-2.0，有权重 / 有权重 | 小（估） | 偏室外城市场景 | 2 |
| CrossFormer（Berkeley，2024）[9] | 130M | 多模态 | 按本体分头输出，含导航路点 | 跨本体（含导航） | MIT，有权重 | 约 1 GB（估） | 零样本碰撞多 [3] | 2 |
| **OmTrackVLA-0.6B**（Om AI Lab，2025H2，确切日期未核实）[10] | 609M（Qwen3-0.6B + DINO/SigLIP） | 单目视频＋语言（“跟随某人”） | 短程路点 | EVT-Bench（Habitat 仿真人物跟随） | MIT，HF 有权重，并开放训练栈 | 约 2–3 GB，3–5 Hz（估） | 中：Habitat 合成人物与卡通 avatar 有差距，但仿真域反而可能比真机域更接近游戏画面；可配合名牌 OCR 做目标锁定 | **3.5** |
| TrackVLA / TrackVLA++（北大/Galbot，CoRL'25）[11][12] | 7B 级 | 单目＋语言 | 路点（扩散头） | EVT-Bench 1.7M 样本 | CC BY-NC-SA；仓库 TODO 显示权重**未确认发布** | 放不下 | 无法测试 | 1 |
| Uni-NaVid（北大，RSS'25）[13] | 7B（Vicuna） | 视频 1 fps、224 分辨率＋语言 | 离散动作 | VLN、跟随、ObjectNav 仿真 | MIT，有权重 | 4-bit 约 5 GB，约 1 Hz（估；A100 上约 5 Hz） | 能力全面但太重 | 2 |
| NaVILA（UCSD/NVIDIA，RSS'25）[14] | 8B（Llama3 + SigLIP） | 8 帧＋语言 | **语言形式的中层动作**（如“前进 75cm”） | VLN＋真人视频 | Apache-2.0，有权重 | 放不下 / 低于 1 Hz | 中层动作接口值得借鉴，模型本身不适用 | 1.5 |
| StreamVLN / InternVLA-N1（上海 AI Lab，2025）[15] | 7B＋NavDP 小模型 | RGB(-D)＋语言 | 像素目标→轨迹 | 仿真 VLN | CC BY-NC-SA，有权重 | 放不下 | — | 1 |
| NavDP（上海 AI Lab 等，ICRA'26）[16][17] | 未公开 | **RGB-D**（深度 0.1–5 m，8 帧） | 24 个路点＋critic 评分 | IsaacSim 3000 个场景 | CC BY-NC-SA，权重需填表 | 约 1–2 GB（估） | 中：深度输入几乎不受画风影响，但不会上台阶或跳跃，与我们的规划器功能重叠 | 2.5 |
| ViPlanner（ETH，2023–24）[18] | 小 | 深度＋语义分割 | 路径路点 | Carla/Matterport/Isaac | “All rights reserved”，权重在 Drive | 小 | 思路可借鉴（可微代价图），许可不友好 | 1.5 |
| NWM（Meta，CVPR'25）[19] | 50M–1B CDiT | 4 帧＋动作 | 预测未来帧，用于规划打分 | 机器人视频 | CC BY-NC，权重需申请 | 每帧 50 步 ODE，规划太慢 | 不适合实时 | 1 |
| NavFoM（北大，ICLR'26）[20] / ABot-N0/N1（高德，2026-02/07）[21][22] | 大模型级 | 多相机＋语言；任务含人物跟随 | 轨迹 | 8M / 16.9M 样本 | **权重未核实**；NavFoM 实机部署跑在 RTX 5090 服务器上 | 放不下 | 值得关注后续是否出小模型 | 1 |
| **NitroGen**（NVIDIA，2025-12-19）[23][24] | 493M（SigLIP2 + DiT） | 单帧 256×256，**无目标条件** | 手柄动作：2 个摇杆＋17 个按钮 | 4 万小时、1000+ 款游戏视频 | NVIDIA License（研究用），有权重 | 约 2 GB；RTX 5080 上每 18 步动作块 51 ms [25]，3060 约 150 ms（估） | 动作空间与我们**完全同构**，但零样本跑 Quake 0/5 完成 [25]，只能当反应式 System-1 | 2 |
| SIMA 2（DeepMind）/ Lumine（字节，7B，4×H20）/ P2P0.1（Player2）[26][27][28] | — | — | 键鼠 / 手柄 | 3D 游戏 | 闭源，或权重未确认 | — | 无法使用 | 0–1 |

## 3. 建议：两个低成本零样本实验（各 1–2 天）

**实验 A：NoMaD（或 OmniVLA-edge）作为“方向提议器”，由几何层把关。**
- 输入：取左眼中心区域，裁到约 90° 并缩放到 96×96，按 4 Hz 左右缓存 3 帧上下文（帧率需与训练数据大致匹配，开工前先读一下配置里的采样间隔）。
- 目标：图像目标直接截取云端 LLM 圈定目标的画面；探索模式把目标屏蔽即可。
- 输出转换：8 个路点（机器人坐标系，按步长归一化）乘以 VRChat 默认步行速度 2 m/s（https://creators.vrchat.com/worlds/udon/players/player-forces），换算成米。然后取前 2–3 个路点的方位角作为头部 yaw 增量，前进摇杆按距离映射，平移摇杆置 0（这些模型都按非完整约束车辆训练）。
- 安全：每条路点轨迹都投影到我们的 2.5D 高度图上检查碰撞和高差，不通过就退回几何规划器。NoMaD 扩散模型可以一次采样多条轨迹，正好用高度图打分（这就是 NavDP 的 critic 思路）。
- 评估：在若干个 VRChat 世界里对比“纯几何”和“几何＋NoMaD 提议”的到达率与卡住次数。

**实验 B：OmTrackVLA-0.6B 做人物跟随。**
- 输入语言指令，例如 “follow the person in the blue jacket”，外观描述由云端 LLM 根据 OCR 锁定的目标生成；输出短程路点，转换方式同实验 A。
- 用名牌 OCR 的检测框做一致性校验，发现跟错人就终止。
- 显存约 2–3 GB（估），在 4–6 GB 预算内。

**跳跃和台阶：** 现有导航模型都没有跳跃动作，一律交给几何层，即用高度图检测台阶或平台边缘后触发 jump。

## 4. 微调需要记录的数据

每帧记录：
- 时间戳；
- 左右目图像（可降采样存储）；
- Monado 给出的 6DoF 头部位姿；
- 下发的摇杆、yaw、jump 指令，以及由位姿差分得到的**实际位移**；
- 高度图 / 深度；
- 当前目标（图像、语言、目标人物名与检测框）；
- 事件标签：卡住（有前进指令但位移约为 0）、碰撞、坠落、成功到达。

用法：
- 按 GNM/ViNT 的做法，**事后把未来帧重标为图像目标**，几何规划器自己跑出来的轨迹就能直接当监督数据，不需要人工标注（[1][8] 的 MBRA 也是这种“重标注”思路）。
- 数据量：按论文经验，小模型适配新域需要数小时到数十小时（这是估计，没有在游戏域验证过）。NitroGen 的报告是在新游戏上微调后任务完成率相对提升最多 52% [23]。
- 动作标签建议统一存成“未来 N 步机器人坐标系下的路点”，与具体输入设备解耦。

## 5. 主要风险

1. **域差距**：所有导航模型都在真实相机或照片级仿真上训练，卡通渲染、透明或发光材质、镜面世界都可能让它们误判。真机零样本评测已经显示碰撞频发 [3]。
2. **尺度与运动学不匹配**：VRChat 世界可以改移动速度，avatar 高度也影响相机高度（训练数据的相机高度多在 0.3–1.5 m 的机器人上），需要按世界标定。
3. **许可**：NavDP、TrackVLA、StreamVLN、InternVLA-N1 是 CC BY-NC-SA，NWM 是 CC BY-NC，NitroGen 只限研究用途；MIT/Apache 的只有 NoMaD/ViNT/GNM、OmniVLA、CrossFormer、OmTrackVLA、CityWalker、NaVILA、Uni-NaVid。
4. **显存与时延**：要和 VRChat、语音模型共用 12 GB，7B 级模型即使量化后也会挤占显存并引入抖动。
5. **未核实项**：TrackVLA、NavFoM、ABot-N0/N1、Lumine、P2P0.1 的权重是否公开；NavDP、S2E、CityWalker 的参数量；OmTrackVLA 的确切发布日期；表中所有 3060 上的速度和显存数字。

## 来源
[1] https://github.com/robodhruv/visualnav-transformer ；https://raw.githubusercontent.com/robodhruv/visualnav-transformer/main/train/config/nomad.yaml
[2] https://arxiv.org/html/2310.07896
[3] https://arxiv.org/abs/2603.25937
[4] https://github.com/NHirose/OmniVLA
[5] https://arxiv.org/abs/2509.19480 ；https://huggingface.co/NHirose/omnivla-edge
[6] https://github.com/VAIL-UCLA/S2E
[7] https://github.com/ai4ce/CityWalker
[8] https://arxiv.org/abs/2505.05592
[9] https://github.com/rail-berkeley/crossformer ；https://arxiv.org/abs/2408.11812
[10] https://huggingface.co/omlab/OmTrackVLA-0.6B
[11] https://github.com/wsakobe/TrackVLA ；https://arxiv.org/abs/2505.23189
[12] https://arxiv.org/abs/2510.07134
[13] https://github.com/jzhzhang/Uni-NaVid
[14] https://github.com/AnjieCheng/NaVILA ；https://arxiv.org/abs/2412.04453
[15] https://streamvln.github.io/ ；https://huggingface.co/InternRobotics/InternVLA-N1
[16] https://github.com/InternRobotics/NavDP
[17] https://arxiv.org/html/2505.08712v3
[18] https://github.com/leggedrobotics/viplanner
[19] https://arxiv.org/abs/2412.03572 ；https://huggingface.co/facebook/nwm
[20] https://arxiv.org/abs/2509.12129 ；https://pku-epic.github.io/NavFoM-Web/
[21] https://arxiv.org/abs/2602.11598
[22] https://arxiv.org/abs/2607.10383
[23] https://arxiv.org/abs/2601.02427 ；https://huggingface.co/nvidia/NitroGen
[24] https://github.com/MineDojo/NitroGen
[25] https://arxiv.org/html/2607.22739
[26] https://www.infoq.com/news/2025/12/sima-2-gemini-agent
[27] https://huggingface.co/papers/2511.08892
[28] https://arxiv.org/abs/2508.14295
其他：SmolVLA https://huggingface.co/blog/smolvla ；Octo https://arxiv.org/abs/2405.12213 ；RDT https://arxiv.org/html/2410.07864v2 ；VLFM https://arxiv.org/abs/2312.03275 ；LeLaN https://arxiv.org/abs/2410.03603
