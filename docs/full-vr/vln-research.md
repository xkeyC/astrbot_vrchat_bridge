# VLN（视觉语言导航）模型调研（2026-10-05）

> 由调研代理整理，标"估"的数字都没有在 3060 上实测。结论已收进 [decisions.md](decisions.md) 的 D16。


> 本文是对 vla_research.md 的补充，不重复其中关于 NoMaD/ViNT/OmniVLA、操作类 VLA、游戏智能体的内容。
> 标注说明：“估”表示按参数量推算，未在 3060 上实测；“未核实”表示没有找到一手来源。
> 不同论文的评测子集不同，尤其是零样本方法常用 100 或 50 条 episode 的子集，所以表中数字只能粗略横向比较。

## 1. 结论先行

1. **2026 年训练式 VLN 的 SOTA 已经到 R2R-CE val-unseen SR 70–80%。** 代表有 StereoNav、Robostral Navigate、Efficient-VLN、Qwen-RobotNav、LightNav-0。不过真正**开放权重、并且能放进 4–6 GB 显存**的，目前只有 **LightNav-0**（Qwen3-VL-4B，Apache-2.0）一个。其余 7–8B 模型，即使只把 FFN 量化到 4-bit 也要约 8 GB（EdgeVLN 对 StreamVLN 的实测）[17]，放不下。
2. **与我们最同构的是 StereoNav [1]。** 它是 3B 模型，输入双目 RGB，并且把“目标位置先验”画成图上的红点，在 R2R-CE 上达到 81.1% SR；真机（G1 人形机器人＋ZED Mini 双目相机）成功率 60.6%，同场景下 StreamVLN 只有 24.3%。可惜目前没有找到代码或权重。它的设计思路可以直接借鉴：我们已经有度量深度，云端 LLM 也能给出目标点。
3. **零样本路线在 2026 年显著变强。** MIP [12] 用前沿模型的 agent，只给 observe 和 step 两个工具，在 R2R-CE 的 100 条子集上就达到 70–78% SR，超过绝大多数训练式模型。代价是慢：每个 episode 中位数 206 秒，上下文 3.3 万到 16.9 万 token。用 GPT-4o-mini 做“深度→障碍图→候选路点＋编号”选择的方案能到 41% SR [13]。
4. **建议：先走“云端 LLM＋全景图上的高度图候选点编号”（训练零成本，和现有栈最契合）；再用 LightNav-0 做本地快速提议器或兜底；同时记录数据，之后把云端决策蒸馏到本地 2–4B 选择器。**

## 2. 训练式 VLN-VLA 一览（R2R-CE / RxR-CE 均为 val-unseen，单位 %）

| 模型（发布） | 参数 / 骨干 | 输入 | 输出 | R2R SR/SPL | RxR SR/SPL | 真机 | 许可 / 权重 | 3060 适配（估） |
|---|---|---|---|---|---|---|---|---|
| **LightNav-0**（2026-08）[2][3] | 4B（标称 5B），Qwen3-VL-4B | 单目 120° RGB，256×448，4 fps，SlowFast 历史压缩 | 两路像素指点 token＋3 个 RVQ token → 10 个 SE(2) 路点 | 68.5 / 62.8 | 73.6 / 64.5 | 有，零样本（Go2、TRON1） | **Apache-2.0，HF 有权重** | 4-bit 约 3.5–5 GB，0.3–1 s/步（估）；4090 上约 4 ms/token [3]；官方走 vLLM 部署，量化与自定义 token 的兼容性未核实 |
| **StereoNav**（2026-05）[1] | 约 3B（InternVL3.5-2B＋DINOv2＋FoundationStereo） | **双目** 448²，8 帧历史，加上目标位置先验（图上画红点） | 离散动作（0.25 m / 15° / stop），一次预测 4 步，带深度辅助任务 | 81.1 / 68.3（用额外数据）；72.8 / 56.4（标准数据） | 67.5 / 52.0 | G1 人形，60.6% | 论文 CC BY 4.0，**代码和权重未找到** | 去掉 FoundationStereo、改用我们自己的深度后约 3–4 GB（估） |
| Robostral Navigate（2026-07）[4] | 8B | 单目 | 指点 (u, v, Δθ) → 扩散策略 | 77.4 / 74.2 | 75.1 / 68.7 | 有（VLM 跑在 0.5 Hz） | 权重未找到 | 放不下 |
| Qwen-RobotNav（2026-06）[5][6] | 2B / 4B / 8B，Qwen3-VL | 单目或多相机（可配置） | 8 个路点（含朝向） | 8B：RGB 65.7 / 59.6；深度设置 72.1 / 66.6 [4] | 76.5 / 65.7 | Go2＋Jetson Thor，196 ms | **官方明确不发布权重** [6] | 不可用 |
| Efficient-VLN（2025-12）[7] | 骨干未核实 | 单目，渐进式记忆 | 离散动作 | v1 版 64.2；最新摘要 73.2 | v1 版 67.0；最新 75.6 | — | 权重未找到 | — |
| InternVLA-N1 DualVLN（2025–26）[8] | Qwen2.5-VL-7B＋System1 小模型 | RGB（另有 RGB-D 版本） | 像素目标 → 轨迹 | 64.3 / 58.5 | 61.4 / 51.8 | 有 | 代码 MIT；权重 CC BY-NC-SA（见上一份报告） | 放不下 |
| JanusVLN（ICLR'26）[9] | Qwen2.5-VL-7B＋VGGT | 单目 | 离散动作 | 60.5 / 56.8 | — | Go2 | GitHub 有代码，许可未核实 | 放不下 |
| NavFoM（ICLR'26）[10] | Qwen2-7B＋DINOv2/SigLIP | **1–N 路相机**（TVI token 编码视角） | 路点 | — | 单视角 57.4；4 视角 64.4（仅 SR） | 有 | 权重未核实 | 放不下 |
| StreamVLN（2025）[4][17] | 8B（Qwen2） | 单目流式输入 | 离散动作 | 56.9 / 51.9 | 52.9 / 46.0 | 有 | CC BY-NC-SA | IQ4 量化后仍需 7.9 GB，Orin NX 上 0.67 s/步 [17] |
| MonoDream（AAAI'26）[11] | **2B**，NVILA-lite | 单目 8 帧；训练时“想象”全景 RGB-D 的潜特征 | 语言化中层动作（25/50/75 cm、15/30/45°） | 55.8 / 49.1 | 49.4 / 40.9 | 无 | 代码和权重未找到 | 约 2–3 GB（估）；论文报告 0.8 s/步 |
| NaVILA（RSS'25）[4] | 8B | 单目 8 帧 | 语言化中层动作 | 54.0 / 49.0 | 49.3 / 44.0 | 有 | Apache-2.0 | 放不下 |
| Uni-NaVid（RSS'25）[14] | 7B（Vicuna） | 单目视频 | 离散动作 | 51.8 / 47.7 | 56.1 / 44.5 | 有 | MIT，HF 有权重 | 4-bit 约 5 GB，约 1 Hz（估） |
| Nav-R1（2025-09）[15] | 基于 3D-R1，LoRA 微调 | RGB-D＋**场景点云** | 先输出思维链，再给动作 | 72.5 / 68.8（自报） | 71.3 / 66.3 | Jetson 平台，云端推理 | 代码 CC BY-NC-SA | 依赖全局点云，不适合我们 |
| VLN-R1（2025-06）[16] | Qwen2-VL-2B / 7B | 单目视频 | 离散动作，一次 6 步 | 2B：25.6 / 20.5；7B：30.2 / 21.8 | 2B：21.4 / 15.5 | 无 | 未核实 | 太弱 |
| Goal2Pixel（2026-06）[18] | 未核实 | 单目＋关键帧记忆 | 可通行像素 → 反投影成 3D 路点 | 54.1 / 52.5（每个 episode 只调用 VLM 7.75 次） | — | — | 有项目页，权重未核实 | — |
| ETPNav（经典全景方法）[13] | 小 | **全景 RGB-D**＋路点预测器 | 在拓扑图上选点 | 57 / 49 | 54.8 / 44.9 | — | 开源 | 很小 |

补充说明：
- **NaVid-4D** 没有找到对应论文（未核实，可能并不存在这个名称）。NaVid（RSS'24）在 R2R-CE 上是 37 / 36 [13]。
- **ObjectNav（HM3D v2）**：LightNav-0 的 SR 为 79.5 [2]，Qwen-RobotNav 为 75.6 [5]。
- **人物跟随（EVT-Bench）**：Qwen-RobotNav 为 90.0 [5]，NavFoM 四视角为 88.4 [10]。LightNav-0 也支持跟随任务。

## 3. 零样本 / 视觉提示路线

| 方法 | VLM | 做法 | 结果 |
|---|---|---|---|
| MIP（2026-07）[12][19] | Claude Opus-5 / Fable-5、GPT-5.x、Qwen 等 | 极简工具：observe 返回 512² 单帧，step 执行动作；另有可选的“路点工具”：12 视角 RGB-D 全景经训练好的路点预测器，生成最多 5 个**编号圆圈**画在四视角拼图上 | R2R-CE 100 条子集 SR 70.7–78，混合模式 76.7；RxR 只有 26–39。混合模式比纯原语少一半步数，耗时不到 1/4；MIT 许可 |
| 障碍图路点＋拓扑提示（2025-09）[13] | GPT-4o-mini | 深度 → 点云 → 120×12 极坐标网格，按**径向高程梯度**判可通行，用 2 层 Transformer 预测路点，提示词带拓扑图和访问记录 | R2R 41 / 25.4，RxR 35.7 / 21.7 |
| GPT-6-Astra 工作流（2026-09）[20] | GPT-6-Astra | “观察 → 决策 → 执行”，直接调 API | 50 条子集 SR 52 / SPL 48.9；问题主要出在**停止判断** |
| C2Nav（2026-09）[21] | GPT-5.5；Qwen3-VL-8B | 在物理上校验过的候选视角中做序数选择，再加上回看和撤回机制 | GPT-5.5：44 / 29；8B 本地模型：31 / 16.7 |
| SmartWay / Fast-SmartWay [22] | GPT-4o | 全景＋路点；Fast 版只用前方 3 个视角 | 约 28–29 SR；每步 12–29 s |
| VLMnav（2024）[23] | Gemini Flash | 用深度算每个方向的最大可走距离，把候选箭头编号后画在图上，含“转身”选项；连续两次回答 stop 才真正停下 | ObjectNav 50.4 SR（PIVOT 只有 24.6）；用估计深度会掉约 10 个点；CC BY 4.0 |
| P2DNav（2026-05）[24] | 未核实 | 先在 360° 全景里选方向，再在俯视局部图上指定像素点 | 相对同类方法提升明显（具体数字未取到） |
| PIGEON（2025-11）[25] | 多种 | 把前沿、可疑目标、楼梯等 PoI 作为编号候选；用 RLVR 训练本地小 VLM | ObjectNav 零样本 SOTA（自报） |

## 4. 哪类设计最适合我们

**(a) 云端 LLM＋全景＋高度图候选点（首选）。**
- 我们的条件几乎就是论文里的“特权设定”：真实度量深度、6DoF 位姿、廉价的 360° 全景。上述零样本论文（VLMnav、[13]、MIP 的混合模式）都证明了“由几何生成候选、让 VLM 只做语义选择”这条路有效，而且 VLMnav 显示**深度质量是关键**。
- 优点：
  - 不用训练，也不在本地 GPU 上增加负载；
  - 卡通画风对 GPT 级模型影响最小（训练式 VLN 全部只在 Matterport 照片级场景中训练，风格化世界的零样本表现未知）；
  - 可以直接复用 OCR 的人名作为候选标签。
- 缺点：每次决策 2–10 s（估），所以只能做低频“选航点”，连续运动交给几何规划器。

**(b) 本地 VLN-VLA。**
- 唯一现实的选择是 LightNav-0：它的“像素指点＋路点”输出能映射成我们的“去点 k”，Apache 许可，显存在预算内（估）。
- 风险：
  - 只在仿真和真实室内训练过，到卡通世界的效果未知；
  - 训练时用 120° 视场，我们是 100°；
  - 部署依赖 vLLM，在 Windows 上要走 WSL 或 Linux；
  - 4-bit 量化后性能未验证。
- StereoNav 的双目设计最契合我们，但没有权重；MonoDream 是 2B，但同样没有权重。

**(c) 不建议**：7–8B 的 StreamVLN、InternVLA-N1、JanusVLN、NaVILA、Uni-NaVid（显存不够，且多为非商用许可）；依赖全局点云的 Nav-R1；以及偏弱的 VLN-R1-2B。

## 5. 推荐实验（各 1–2 天）

**实验 1：全景 SoM 选点（云端）。**
1. 用头部 yaw 转一圈，采集 6 张 60° 间隔的左眼图像，各裁成 100° 视场，也可以拼成 2×3 网格。
2. 在 2.5D 高度图上按 VLMnav 和 [13] 的做法生成候选：在 24–36 个径向方向上，求高程梯度可通行范围内的最远点，取其 2/3 处；再加上前沿点、OCR 识别到的玩家位置、门口或台阶这类“PoI”。经规划器可达性过滤后，保留 6–10 个。
3. 把候选的 3D 点投影到全景各视图上，画**编号圆圈**，并标注距离（如 “3 · 6.2 m”）。另附一张同样编号的俯视高度图。
4. 让 LLM 输出 JSON：`{choice: k | "turn_around" | "stop" | "none", target_visible, reason, confidence}`。停止判定借鉴 VLMnav：连续两次 stop 才执行。
5. 重新查询的时机：到达所选点、走过 5 m 左右、或者目标进入视野。
6. 记录：指令完成率、调用次数、总耗时、卡住次数。

**实验 2：本地 LightNav-0 试跑。**
1. 在 WSL 或服务器上用 vLLM 跑，先试 bf16（约 9 GB，估），再试 AWQ / 4-bit。
2. 输入单目左眼视频（4 fps）和语言指令。
3. 把输出的 10 个 SE(2) 路点投影到高度图上做碰撞校验，取通过校验的最远点交给规划器执行。
4. 在相同的任务上与实验 1 对比成功率和延迟。
5. 如果显存或画风不过关，改用通用 Qwen3-VL-2B/4B（4-bit）跑实验 1 的同一套提示，作为本地兜底选择器（C2Nav 的结果表明，8B 本地模型约能达到 GPT 级模型 70% 的效果 [21]）。

**接口约定。** VLN 层只输出“去点 k、转身、停止、未找到”这类中层动作，不碰摇杆；跳跃和台阶仍由几何层负责。

**成本与时延（估）。**
- 每次决策大约是 6 张小图加 1 张俯视图，约 3–6k 输入 token。
- 每个任务 5–15 次调用，单次 3–10 s；与 MIP 的 206 s/episode 相比，我们把候选和执行都交给几何层，预计能快得多。
- 本地 LightNav-0 约 1–3 Hz（估），可以在两次云端决策之间做反应式修正。

**许可风险。**
- 可放心使用：LightNav-0（Apache-2.0，但评测用的 EVT-Bench 数据是 CC BY-NC-SA）、MIP 代码（MIT）、VLMnav（CC BY 4.0）。
- InternVLA-N1、StreamVLN、Nav-R1 为 NC-SA，不适合商用或再分发。
- StereoNav 用到的 FoundationStereo 许可未核实（疑似非商用）。我们已经有自己的双目深度，不需要它。

## 6. 为后续微调或蒸馏要记录的数据

每次决策记录一条，形成 R2R 风格的 episode：
- 指令原文，以及云端 LLM 拆解出的子目标；
- 全景原图和带编号的标注图；
- 候选集：每个候选的世界坐标、高度图特征、规划路径长度、类型（前沿 / PoI / 玩家）；
- LLM 的选择、理由、置信度、延迟和 token 数；
- 整条位姿轨迹，以及执行结果（到达、卡住、跌落）；
- 最终成功与否，以及人工或 LLM 的事后纠正；
- **事后标签**：哪个候选确实位于通向最终目标的路径上。

用途：
- 用 SFT 加 RLVR 把云端选择蒸馏到本地 2–4B 选择器（PIGEON 的思路）；
- 把同一批数据转成路点监督，用来微调 LightNav 类模型；
- 双目和深度一并保存，为将来复现 StereoNav 式的“双目＋目标先验”留余地。

## 来源
[1] https://arxiv.org/abs/2605.13328 ；https://arxiv.org/html/2605.13328
[2] https://arxiv.org/abs/2608.30935 ；https://arxiv.org/html/2608.30935v1
[3] https://huggingface.co/LightOriginsHQ/LightNav-0 ；https://github.com/lightorigins/LightNav-0
[4] https://arxiv.org/html/2607.20785v1
[5] https://arxiv.org/abs/2606.18112 ；https://www.alibabacloud.com/blog/qwen-robotnav-a-scalable-navigation-model-designed-for-an-agentic-navigation-system_603266
[6] https://github.com/QwenLM/Qwen-RobotNav
[7] https://arxiv.org/abs/2512.10310
[8] https://github.com/InternRobotics/InternNav
[9] https://arxiv.org/html/2509.22548v2
[10] https://arxiv.org/html/2509.12129
[11] https://arxiv.org/html/2508.02549v4
[12] https://arxiv.org/html/2607.26148
[13] https://arxiv.org/html/2509.20499
[14] https://github.com/jzhzhang/Uni-NaVid
[15] https://arxiv.org/html/2509.10884v1 ；https://github.com/AIGeeksGroup/Nav-R1
[16] https://arxiv.org/html/2506.17221v2
[17] https://arxiv.org/html/2609.35570
[18] https://arxiv.org/abs/2606.01621
[19] https://github.com/jianzhou0420/MIP
[20] https://arxiv.org/abs/2609.20116
[21] https://arxiv.org/abs/2609.15142
[22] https://arxiv.org/html/2511.00933
[23] https://arxiv.org/html/2411.05755v1
[24] https://arxiv.org/abs/2605.19634
[25] https://arxiv.org/abs/2511.13207
未核实或存疑：NaVid-4D 是否存在；StereoNav、MonoDream、Efficient-VLN、Robostral、Goal2Pixel 的代码和权重；NavFoM 的权重；Efficient-VLN 两个版本的数字差异；Nav-R1 的成绩依赖全局点云；LightNav-0 的量化可行性；所有 3060 上的显存和时延。
