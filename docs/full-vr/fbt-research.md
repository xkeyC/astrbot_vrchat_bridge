# 全身动作接管调研：OSC Trackers / 全身追踪（2026-10-06）

> 目标（用户）：全面接管化身的动作，并让动作更自然。D24 当时评估过全身追踪并决定先不做，本文重新评估。标"待测"的地方都还没有在服务器上验证过。

## 1. 结论先行

- **走 VRChat 原生的 OSC Trackers**：用 `/tracking/trackers/1..8/position|rotation` 发送最多 8 个追踪器（髋、胸、双脚、双膝、双肘），再加上头显和两只手柄（含手指骨架，现在已经在发），基本就能控制整个身体。这条路不用改 Monado，也不用改 xrizer。[1]
- **D24 的三个顾虑，现在看有两个不成立**：
  - **"开了全身追踪就没有走路腿部动画"——不成立。** 官方文档写明：6 点以上的全身追踪下，"Locomotion Animations" 打开（默认就是打开）时，用摇杆移动会播放走路和跑步动画。[3] 所以走路可以继续用 VRChat 自带的腿部动画，我们只接管站着时的全身动作和上半身。
  - **"校准要点菜单"——成立，但做得到。** 每次启动 VRChat 校准一次，之后换化身会自动套用。[2] 校准需要：打开快捷菜单，点"校准全身"，站直看前方，同时按两个扳机。[3] 手柄全由我们控制，抓帧和 OCR 也都有，可以做一个"按文字点菜单"的通用能力。
  - **"bridge 一卡，腿就僵住"——仍然存在。** 需要看门狗，另外要测 VRChat 收不到 OSC 后的表现。
- **坐标对齐对我们很友好**：OSC 的头部消息每帧按位置把 OSC 空间对齐到化身头部，朝向的偏航角在持续发送时用 10 秒插值对齐。[1] 我们转身只转头显，playspace 从不旋转（`walk.rs`），所以 OSC 空间和 Monado 的追踪空间之间是固定变换，10 秒插值不会造成拖尾。保险起见可以只发一次头部朝向：单条消息在 300 ms 内到达时会立即对齐。[1]
- **动作来源分层**：
  - 程序化动作（现有的 `anim`，扩展到髋和脚）；
  - 动作片段库（动作捕捉数据，加上离线生成的片段），由 LLM 按名字调用；
  - 说话手势（先按响度、重音挑片段，以后再考虑流式模型）；
  - 走路交给 VRChat。
- **大模型生成动作不在 3060 上实时跑**：HY-Motion 1.0 最少要 24–26 GB 显存。[5] 它适合在别的机器上**离线**生成片段库。MotionLCM 生成约 200 帧只要 30 ms 左右，[4] 但它是基于 HumanML3D 训练的研究模型，只作为以后的可选项。

## 2. 两条接入路线对比

| | OSC Trackers（推荐） | Monado 虚拟追踪器 |
|---|---|---|
| 改动 | 只改 bridge，往 9000 端口发 UDP | 给 Monado remote 驱动打补丁，增加名字里带 "tracker" 的设备；xrizer 已经会把这类设备当作通用追踪器交给 VRChat（`create_monado_generic_trackers`，`XR_MNDX_xdev_space`），不用改 xrizer [6] |
| 坐标 | Unity 左手坐标系，+Y 朝上，单位米，欧拉角顺序为 Z-X-Y（度）；靠头部消息对齐 [1] | 和头显处在同一个追踪空间、同一个时钟 |
| 数量 | 8 个加头部对齐 [1] | VRChat 最多 8 个额外追踪器 [2] |
| 风险 | 延迟、插值、超时行为官方没写，待测 | 要维护补丁，部署时需要重编 Monado |
| 用途 | 先用它 | 只有 OSC 抖动或延迟不可接受时才换 |

## 3. 建议的结构

```
动作源 ──> 身体姿势（身体坐标系，按眼高缩放） ──┬─ 头、双手、手指 ─> RemoteHmd（现有）
  程序化 / 片段库 / 说话手势 / 混合           └─ 髋、胸、脚、膝、肘 ─> OSC trackers（新增）
```

- **`vrc_vr::body`**（新增）：一套人形关节姿势，最终输出给头显、手柄和 8 个追踪器。
  - 现有的 `Overlay` 只覆盖头和手，扩展成全身；
  - 扫描（`hold_still`）期间，追踪器也保持静止。
- **程序化动作**（改动最小、收益最快）：
  - 待机时重心转移真正落到髋上，双脚固定在地面上；
  - 呼吸时胸口起伏；
  - 说话时身体也带一点动作。
- **片段库**：
  - **来源**：
    - CMU 动作捕捉（D25 已经在用，可以随意使用）；
    - 100STYLE（CC BY 4.0，各种风格的移动动作）；
    - 在别的机器上用 HY-Motion 按文字生成的片段（许可允许月活 1 亿以下的个人和公司使用 [5]），例如挥手、鞠躬、鼓掌、耸肩、指方向、坐下、跳舞。
  - **处理**：离线重定向到我们的关节集合，存成小文件。
  - **调用**：LLM 工具 `vrchat_motion(name)`，播放时带淡入淡出。
- **说话手势**：先从片段库里截出说话手势的片段，按现有的响度和重音触发。LiveGesture 这类流式模型（每 200 ms 一段，生成小于 50 ms）可以以后再评估。[7]
- **走路**：保持 Locomotion Animations 打开。
  - 待测：起步和停步时，腿从动画切回追踪器是否会跳一下。
  - 停步时，把脚的追踪器放到髋下方的自然站姿。

## 4. 落地步骤

1. **验证（约半天）**：
   - bridge 发 3 个静止的追踪器（髋和双脚）；
   - 用 `/v1/vr/...` 调试接口驱动手柄点"校准全身"并按两个扳机；
   - 测 OSC 追踪器的延迟和平滑，以及停发之后的行为；
   - 测摇杆走路时腿部动画能否正常接管，停下以后能否切回来。
2. **按文字点菜单**：OCR 在抓帧中找到按钮，用双目测出深度，把右手柄的射线对准按钮，再按扳机。以后点别的菜单也能复用。
3. **全身姿势**：增加 `body` 模块，把现有动画扩展到 8 个追踪器。
4. **片段库和 LLM 工具。**
5. **（可选）说话手势模型；如果 OSC 不理想，再换 Monado 虚拟追踪器。**

## 5. 验证结果（2026-10-06）

第 1 步完成，过程和接口细节见 [agent-vr-use.md](agent-vr-use.md)。

- **OSC 追踪器可用**：bridge 发 3 个（髋和双脚），VRChat 的快捷菜单马上出现"校准"。bot 用射线点"校准"，站直后扣两个扳机就能完成校准，全程不用人手操作。
- **坐标**：追踪空间的米（缩放 1），头部对齐用同一空间的头显位姿。按化身眼高缩放是错的。
- **跟随效果**：在世界镜子里看，抬脚时膝盖跟着弯，侧跨时髋跟着移，站姿自然。
- **停发不会掉校准**（更正）：停发后 `TrackingType` 从 6 降到 3，恢复发送后 3 s 内回到 6，动作照样生效。只有 VRChat 重启后才要重新校准。
- **自动校准已实现**：`POST /v1/vr/calibrate`，加上 `auto_calibrate` 自动触发（TrackingType 一直是 3 时）。用 OCR 找按钮、用悬停提示二次确认、等菜单关闭，最后用 `TrackingType` 验证，约 10 s。见 [agent-vr-use.md](agent-vr-use.md) 第 5 节。
- **追踪器设置持久化**：保存在 `trackers.json`，bridge 重启后自动恢复发送。
- **"全身追踪时启用运动动画"是开着的**：摇杆走路时腿用 VRChat 的动画，和调研结论一致。起步、停步时的过渡还没仔细看。
- **画质**：每只眼从 1280 提到 1920，镜子分辨率 25% → 100%，抗锯齿 X4，细节层次 高；GPU 约 34%。双目匹配固定在约 640 宽，避障不受影响。

## 6. 待确认 / 待测

- 摇杆移动时，髋和胸的追踪器会不会被动画覆盖。
- OSC 停发以后，追踪器会冻结还是会被移除。
- 化身方面：当前化身的 Locomotion 设置（化身的动画控制器可以强制关掉全身追踪下的移动动画 [3]），以及化身是否有膝和肘的骨骼。

## 来源

1. VRChat 文档，OSC Trackers：https://docs.vrchat.com/docs/osc-trackers
2. VRChat 文档，IK 2.0 及校准保存：https://docs.vrchat.com/docs/vrchat-202221 、https://docs.vrchat.com/docs/ik-20-features-and-options
3. VRChat 文档，Full-Body Tracking（校准步骤、Locomotion Animations）：https://docs.vrchat.com/docs/full-body-tracking ；快捷菜单开关：https://docs.vrchat.com/docs/vrchat-202221p4
4. MotionLCM：https://arxiv.org/abs/2404.19759 ，https://github.com/Dai-Wenxun/MotionLCM
5. HY-Motion 1.0：https://arxiv.org/abs/2512.23464 ，许可与显存：https://www.sourcepulse.org/projects/21729235
6. xrizer `src/input/devices.rs`（`create_monado_generic_trackers`）：https://github.com/Supreeeme/xrizer
7. LiveGesture（CVPR 2026）：https://arxiv.org/abs/2604.10927
8. 用 OSC 追踪器做移动动画的先例（hai-vr）：https://hai-vr.notion.site/Animating-locomotion-using-OSC-Trackers-Retrospective-449847c5407d4173b2c3562523c087ee
