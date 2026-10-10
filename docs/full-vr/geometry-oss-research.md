# 几何管线开源项目调研（2026-10-05）

> 由调研代理整理，标 [估] 的是估算，标 [未核] 的是没有核实。结论已收进 [decisions.md](decisions.md) 的 D16。


标注：**[估]** 是我的估算，**[未核]** 是没有在原始页面上核实过的信息。优先级：P0 立即采用，P1 近期，P2 备选，Ref 只作参考。

## 0. 结论先行

- 我们的输入是"无噪声、已校正、位姿已知"的立体图，难点不在噪声，而在**无纹理、镜子、透明物体和隐形碰撞体**。最划算的改动是把 SGM 搬到 GPU（libSGM），再加一个低频运行的学习式立体（Fast-FoundationStereo）补平涂区域。
- 旋转已知，又能从 OSC 读到 `VelocityX/Y/Z` 和 `Grounded`（https://docs.vrchat.com/docs/osc-avatar-parameters ），所以定位可以退化为**只估平移**：先用速度积分做先验，再做"固定旋转的 point-to-plane ICP"。不需要上完整的 SLAM。
- 规划最值得借鉴的是游戏界的 **Recast 管线**。把 2.5D 高度图转成三角网格交给 Recast/rerecast，用跳跃链接（off-mesh link）表达台阶和跳跃，比自己从头写 hybrid A* 省事得多。
- 给云端 LLM 的候选点由几何生成：前沿点、可达点、物体簇。借鉴 VLFM、VLMnav 和 SoM 的标号方式，不必依赖开放词汇检测器。
- **用速度当碰撞传感器**：摇杆推了但实测速度约为 0，就把对应格子标成阻挡。这一条能同时处理透明墙和隐形碰撞体。

## 1. 立体匹配

| 项目 | 作用 | 语言/许可 | 成熟度 | 成本 | 用法 | 优先级 |
|---|---|---|---|---|---|---|
| libSGM https://github.com/fixstars/libSGM | CUDA SGM，census，4/8 路径，带亚像素 | C++/CUDA，Apache-2.0 | 成熟，但 GitHub 上没有正式 release | RTX 3080 上 1024×440、128 视差约 1.5 ms（README 数据）。3060 上 1280²/128 视差 [估] 约 10–15 ms | 写一层薄的 C 包装，用 bindgen/cxx 接入，替换现有约 150 ms 的 CPU SGM | **P0** |
| OpenCV `cuda::StereoSGM` / CPU SGBM | 同类算法，可交叉验证；WLS 滤波可做后处理 | C++，Apache-2.0 | 成熟 | 与 libSGM 相近 [未核] | 通过 `opencv` crate 调用，只作对照 | P2 |
| NVIDIA VPI Stereo https://docs.nvidia.com/vpi/algo_stereo_disparity.html | SGM（CUDA 后端），自带置信度图 | 闭源 SDK | 成熟 | CUDA 后端显存占用较高 | 只参考它的置信度输出设计 | Ref |
| Fast-FoundationStereo https://github.com/NVlabs/Fast-FoundationStereo | 零样本学习式立体，CVPR 2026 | PyTorch + TensorRT/ONNX 导出。代码许可未单独核实；权重为 NVIDIA Open Model Agreement（允许商用） | 2026-02 发布，活跃 | RTX 3090 上 640×480 用 TRT 为 14–23 ms，峰值显存约 650 MB（README 数据）。3060 [估] 约 35–60 ms | 导出 TRT engine，以 ort/TensorRT 在 Rust 里调用。只在全景扫描或 SGM 低置信区域时以 2–5 Hz 运行 | **P0/P1** |
| Lite Any Stereo（LAS，以及 2026 年的 V2）https://arxiv.org/abs/2511.16555 | 前馈式零样本立体，计算量不到 FoundationStereo 的 1% | 未找到代码仓库链接 [未核] | 论文 CVPR 2026 | RTX 4090 上 KITTI 分辨率约 19 ms | 等代码和权重放出后再评估 | P2 |
| Stereo Anywhere https://stereoanywhere.github.io/ | 立体加单目先验，论文里唯一能正确处理镜子和透明栏杆的方法 | PyTorch，许可 [未核] | CVPR 2025 | 依赖 Depth Anything，较重 [估] | 离线研究镜子和透明物体时参考 | Ref |
| OpenStereo / LightStereo https://github.com/XiandaGuo/OpenStereo | 12 种以上模型的统一代码库，已集成 TRT | **仅限学术用途** | 活跃 | LightStereo 约 17 ms | 只用来做算法对比 | Ref |
| HITNet ONNX https://github.com/ibaiGorordo/ONNX-Split-HITNET-Stereo-Depth-Estimation | 实时网络，不构建代价体 | ONNX | 老旧 | 低 | 已被 FFS 超越 | Ref |
| sgm-rs https://lib.rs/crates/sgm-rs | Rust 版 SGM | Rust | 2021 年以后无更新 | — | 不如我们现有实现 | — |

我没有找到成熟的 wgpu 立体匹配 crate。如果坚持纯 Rust，可以把现有 census SGM 用 wgpu/WGSL 重写（逐路径扫描并行化），但工作量明显大于接入 libSGM。

**置信度与困难区域**（都比较容易实现，可直接移植）：
- 左右一致性检查、唯一性比（best/second best）、代价曲率、局部纹理方差门限。
- 低置信像素不写入地图，而不是写成"障碍"。
- 同一格子在多帧、多朝向下观测一致才提升置信度。
- 平涂区域：优先用 FFS 的结果填补，否则依赖地图的时间累积。

## 2. 建图

| 项目 | 作用 | 语言/许可 | 成熟度 | 用法 | 优先级 |
|---|---|---|---|---|---|
| elevation_mapping_cupy https://github.com/leggedrobotics/elevation_mapping_cupy | GPU 高程图：逐格 Kalman 高度和方差更新、可见性清理（射线清除动态物体）、漂移补偿、可学习可通行性、多模态语义层 | Python/CuPy，有 ROS1/2 分支，MIT | 活跃，IROS 2022/2023 论文 | **移植算法**：高度方差融合、可见性清理和语义层的设计都直接适用于我们的 2.5D 格子 | **P0（算法）** |
| FastDEM https://github.com/Ikhyeon-Cho/FastDEM | 轻量高程图，只依赖 Eigen，不需要 ROS；CPU 每帧约 10 ms | C++17，BSD-3 | 活跃 | 作为 CPU 实现的对照和移植参考 | P1 |
| grid_map（ANYbotics）https://github.com/ANYbotics/grid_map | 多层 2.5D 格子，含坡度、粗糙度、台阶滤波器 | C++，BSD-3 [未核] | 成熟 | 参考它的可通行性滤波器链（slope、step、roughness 加权） | Ref |
| Bonxai https://github.com/facontidavide/Bonxai | 稀疏分层体素，比 OctoMap 快 20 倍以上；概率占据还没做完 | C++ 单头文件，MPL-2.0 | 维护中 | 需要 3D（桥下空间、二层楼）时，在 Rust 里仿写"哈希块 + 稠密叶子"的结构 | P1 |
| nvblox https://github.com/nvidia-isaac/nvblox | GPU TSDF/ESDF 和网格重建，可屏蔽人体等动态物体 | C++/CUDA（可不依赖 ROS），许可见 LICENSE.md [未核] | 活跃 | 可选：TSDF 网格直接喂给 Recast。会与游戏争抢显存 | P2 |
| VDBFusion / OctoMap / Voxblox | TSDF / 概率八叉树 | C++，MIT/BSD | 成熟但较老 | 只作参考 | Ref |
| Rust 生态：kiddo（KD 树）、nalgebra/glam、rerun（可视化）https://rerun.io | 基础设施 | MIT/Apache | 活跃 | rerun 可以同时记录点云、高程图、路径和 LLM 标号图，调试价值很高 | **P0（rerun）** |

**建议**：每个格子存 `min_h / max_h / 方差 / 观测次数 / 最后观测时间 / 可通行代价`，把楼层下方的空间（桌子下面、二楼）留给"多层"扩展。用可见性清理去掉移动玩家留下的残影。

## 3. 定位 / 里程计

我们的情况：头显姿态精确，但摇杆移动会改变世界位置，所以未知量只有**平移**（加上传送）。

| 项目 | 语言/许可 | 说明 | 用法 | 优先级 |
|---|---|---|---|---|
| KISS-ICP https://github.com/PRBonn/kiss-icp | C++/Python，MIT，活跃（2026-04 仍有提交） | 体素哈希局部地图、自适应阈值、恒速先验 | **移植思想**：旋转固定，只解 3×3 平移；速度先验换成 OSC 速度积分 | **P0** |
| small_gicp https://github.com/koide3/small_gicp | 单头文件 C++，MIT | 并行 ICP/GICP/VGICP | 需要更强配准时通过 FFI 调用，或作为对照 | P1 |
| kiss-icp-rs https://github.com/ulagbulag/kiss-icp-rs | Rust，MIT | 非官方移植，2024 年以后没有更新 | 可读代码，不建议依赖 | Ref |
| cuVSLAM / PyCuVSLAM https://github.com/nvidia-isaac/cuVSLAM | C++/CUDA，NVIDIA Community License（可商用，但仅限 NVIDIA 硬件） | 支持多目、RGBD、IMU；Ubuntu 22/24 | 漂移评测的交叉验证基线；不建议上主链路，因为卡通纹理下特征点少 [估] | P2 |
| Basalt https://gitlab.com/VladyslavUsenko/basalt | C++，BSD-3 | VIO/VO，有 Monado 集成版本 | 参考 | Ref |
| ORB-SLAM3 / OpenVINS（GPLv3）、stella_vslam（BSD-2 [未核]） | C++ | 特征法，平涂纹理风险大，GPL 有传染性 | 不建议 | — |

**做法**：
- 每帧做一次 `位置 += R·v·dt` 的预测。
- 用 SGM 高置信点和局部体素图做 point-to-plane ICP，只解 tx、ty、tz，并可以把 z 锁定在 `Grounded` 时的地面高度。
- 回环和传送（门户、重生）时，用存档的全景关键帧做位置识别：粗匹配用全景描述子，精配准用 ICP。

## 4. 规划

| 项目 | 语言/许可 | 说明 | 用法 | 优先级 |
|---|---|---|---|---|
| Recast/Detour https://github.com/recastnavigation/recastnavigation | C++，Zlib，7.9k star，Unity/Unreal/Godot 都在用 | 体素化 → 可行走区域 → 多边形网格；参数有 agent 半径、高度、可攀高度、最大坡度；支持 off-mesh 连接；Detour 提供寻路和拉直（funnel） | 通过 recastnavigation-sys（含 Detour 的原始绑定）https://github.com/andriyDev/recastnavigation-rs-sys 接入 | **P0** |
| rerecast https://github.com/janhohenheim/rerecast | 纯 Rust，MIT/Apache | Recast 的忠实移植；核心 crate 可脱离 Bevy；输入三角网格；不含 Detour | 纯 Rust 生成 navmesh，寻路配 landmass 或 polyanya | P1 |
| landmass https://github.com/andriyDev/landmass | Rust，MIT/Apache | navmesh 上的寻路、路径简化、转向避障；不负责生成 | 配合 rerecast 使用 | P1 |
| oxidized_navigation | Rust | **2025-11 已归档**，作者推荐改用 rerecast | 不采用 | — |
| pathfinding crate（A*、Dijkstra、fringe）https://crates.io/crates/pathfinding | Rust，MIT/Apache [未核] | 通用图搜索 | 2.5D 格子上的 A*：边代价包括台阶、跳跃、下落和贴墙惩罚 | **P0（兜底）** |
| 自动跳跃链接：Unity 的 Drop-Down / Jump-Across 链接 https://docs.unity3d.com/Manual/nav-BuildingOffMeshLinksAutomatically.html 、Utrecht 硕士论文（Recast 扩展）https://studenttheses.uu.nl/handle/20.500.12932/22818 | — | 沿 navmesh 边界采样，检查落点和抛物线是否通畅，生成链接 | 移植算法 | P1 |
| m-explore 前沿探索、VLFM 前沿和价值图 | — | 前沿检测 | 前沿点作为 LLM 候选 | P0（算法） |

**关键做法**：
- 把高程图每个格子转成两个三角形；相邻格子高差大于可攀高度的地方补竖墙。这样就能直接喂给 Recast。
- 跳跃参数要逐个世界实测。VRChat 文档里 Udon 的默认 `JumpImpulse` 是 0，也就是不能跳（https://creators.vrchat.com/worlds/udon/players/player-forces ）。默认步行速度 2 m/s，奔跑速度 4 m/s。
- 实测方法：执行一次跳跃，从 `VelocityY` 和 `Grounded` 推算跳跃高度，再用这个值生成跳跃链接。

## 5. 感知辅助

| 项目 | 许可 | 成本 | 用法 | 优先级 |
|---|---|---|---|---|
| Set-of-Mark https://github.com/microsoft/SoM | MIT | — | 参考它的标号渲染（防重叠、对比色）；标号来源换成我们的几何候选点 | **P0** |
| VLMnav https://jirl-upenn.github.io/VLMnav/ | 代码许可 [未核] | — | 在图上画编号箭头，让 VLM 选方向；与我们的设计几乎一致 | P0（参考） |
| YOLOE https://github.com/THU-MIG/yoloe | AGPL-3.0 | 实时，比 YOLO-World-S 快 1.4 倍 | 文本或视觉提示的开放词汇检测；独立进程运行，避开许可传染 | P1 |
| YOLO-World https://github.com/AILab-CVC/YOLO-World | GPL-3.0 | 实时 | YOLOE 的替代 | P2 |
| Grounding DINO 1.5 Edge | **只有云 API**，权重不开放 https://github.com/IDEA-Research/Grounding-DINO-1.5-API | — | 不可本地部署 | — |
| SAM 2.1 tiny https://github.com/facebookresearch/sam2 | Apache-2.0/BSD-3 | 38.9M 参数，A100 上 91 FPS | 点选后分割和跟踪物体 | P2 |
| SAM 3 https://github.com/facebookresearch/sam3 | SAM License [未核] | 840M 参数，3060 上与游戏共用显存吃紧 [估] | 暂不采用 | — |
| HetNet 镜面检测 https://github.com/Catherine-R-He/HetNet | [未核] | 轻量 | 训练数据是真实照片，在卡通渲染上的效果未知 | P2 |
| MirrorSAM2（2025，带深度）https://arxiv.org/abs/2509.17220 | — | — | 参考 | Ref |

**镜子和人的几何线索（推荐优先实现）**：
1. **自身镜像测试**：画面里出现与我们动作同步的"自己"，就判定为镜子。可以主动晃动头或手来验证。
2. **镜子平面**：镜中区域的深度落在已建墙面"背后"，并且边界是规则矩形框，就把该平面标成墙。
3. **玩家**：深度和地图不一致、而且在移动的点簇，判定为动态物体；名牌文字可作为辅助锚点 [未核]。跟踪器用卡尔曼滤波加匈牙利匹配（参照 ByteTrack，MIT），在 Rust 里几百行就能写完。COCO 预训练的"person"类在非人形 avatar 上可能会漏检 [估]。
4. **透明墙和隐形碰撞体**：靠前面说的"速度当碰撞传感器"，把撞到的格子写成阻挡。

## 6. 端到端参考

| 项目 | 要点 | 许可 | 用法 |
|---|---|---|---|
| VLFM https://github.com/bdaiinstitute/vlfm | 深度生成占据图、前沿检测、BLIP-2 价值图、PointNav；已部署到 Spot 真机 | MIT，已不再维护 | **管线照抄**：把 BLIP-2 价值图换成"LLM 在全景标号图上打分"（P0 参考） |
| VLMnav（见上） | 编号动作，零样本 | — | 提示词设计参考 |
| NavAI https://arxiv.org/abs/2601.03251 | 在 Unity VR 场景里用立体截图和 LLM 函数调用导航；目标导航成功率 89%，探索效率低；**没有深度、没有地图** | 未见代码 | 反面参考：说明加入几何层的必要性 |
| Cradle https://github.com/BAAI-Agents/Cradle | 截图到键鼠的通用游戏 agent，带反思和技能库 | MIT，2024-06 以后基本停更 | 高层技能管理参考 |
| SemExp、OpenFMNav [未核] | Habitat 中的语义地图加前沿 ObjectNav | — | Ref |

## 7. 优先计划（Top 5）

1. **GPU 立体匹配**：用 libSGM（C 包装加 bindgen）接入 GPU SGM，同时补齐左右检查和唯一性置信度。目标是 1280² 在 20 ms 内 [估]。CPU 实现保留作回退。
2. **只估平移的里程计**：OSC 速度积分加上 KISS-ICP 风格、固定旋转的 point-to-plane ICP；同时实现"速度碰撞传感器"，并接入 rerun 可视化。
3. **高程图升级**：按 elevation_mapping_cupy 的思路加入方差融合、可见性清理和 min/max 双层。
4. **navmesh 路线**：高程图转三角网格，交给 Recast（recastnavigation-sys，或纯 Rust 的 rerecast）；实测跳跃参数后自动生成跳跃和下落链接。格子 A* 留作兜底。
5. **LLM 候选点**：前沿点、navmesh 可达点和深度簇投影到全景图，按 SoM 和 VLMnav 的方式编号后交给云端 LLM，即 VLFM 管线去掉 BLIP-2 的版本。之后再按需评估 Fast-FoundationStereo（TRT）和 YOLOE。

## 8. 风险

- **显存和算力**：3060 同时承担游戏渲染和语音模型。学习式模型只能低频运行，要设置显存上限并监控帧率。
- **许可**：OpenStereo 仅限学术；YOLO-World 是 GPL，YOLOE 是 AGPL，要隔离成独立服务；cuVSLAM 绑定 NVIDIA 硬件。
- **游戏特性**：镜子、透明物体、隐形碰撞体、传送门和移动平台都会破坏"静态世界"假设；各世界的移动参数和跳跃可用性也不一样。
- **数据域差异**：学习式立体、检测和镜面模型都是在真实照片上训练的，卡通渲染下的效果全部 [未核]，需要在自采的 VRChat 数据上做小规模评测。
- 3060 上的性能数字都是按 3080、3090、A100 的公开数据外推 [估]，接入前要实测。
