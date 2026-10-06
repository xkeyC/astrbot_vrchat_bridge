# 化身位置角标：让 bot 从自己的眼睛画面里读出世界坐标

VRChat 的 OSC 不提供世界绝对坐标。这里的办法是：在 bot 的化身上加一个小网格，用自定义着色器把**相机的世界坐标和朝向**编码成黑白方块，画在每只眼画面的左下角。bridge 本来就在截取两只眼的画面（Monado 抓帧），读出这块方块就得到坐标。

- 只有 bot 自己的眼睛看得到；别的玩家、镜子、拍照相机都看不到。
- 坐标是世界自己的，不会漂。同一个世界下次再进，坐标系一致，地图可以直接接着用。
- 有了它，里程（速度积分）和画面配准的误差也能直接量出来。

本文是**协议和实现说明**。化身端在 Avatar 项目里实现，读码端在 bridge 里实现，两边都按第 3 节的格式。

## 1. 效果

- bot 每只眼的画面左下角有一块 22×10 的黑白方格，约占画面宽的 11%、高的 5%。
- 每一帧都按那只眼当时的相机位置重画。左眼画左眼的位置，右眼画右眼的位置，两者之差就是两眼间距，可以用来自检。
- 不画的时候：
  - 别的玩家的客户端：渲染器根本不打开（只有 `IsLocal` 为真才开，见 2.1）；
  - bot 自己的客户端里，相机不在这个化身的头附近（`_OwnerRadius`）；
  - 镜子（`_VRChatMirrorMode` 不为 0）；
  - 拍照、录像相机（`_VRChatCameraMode` 不为 0）；
  - 别人屏蔽了动画时，渲染器停在默认的关；屏蔽了着色器时，回退规则设为隐藏（`VRCFallback = Hidden`），不会露出一块白片。

## 2. 化身端怎么做

### 2.1 物体

化身端已经实现。做法是写一个编辑器安装脚本，对化身的预制体资产生成下面这些东西（用 `PrefabUtility.LoadPrefabContents` 打开、改完 `SaveAsPrefabAsset`）。材质、网格、动画控制器和两个动画片段也由脚本生成；重复运行会更新已有的角标。脚本做的事：

1. 在 Head 骨骼下新建物体 `PosBeacon`，加 `MeshFilter` 和 `MeshRenderer`。
   - 位置按化身的视点（Avatar Descriptor 的 View Position）算：和眼睛同高，正前方 15 cm，朝向和化身根一致。不用固定的本地坐标，因为每个模型 Head 骨骼的位置和缩放都不一样。
   - 世界缩放 0.05。
   - 网格是 Unity 自带 Quad 的复制品（复制顶点、UV、法线、三角形，存成网格资产），**包围盒放大到约 1 m**，避免被视锥剔除（见 7.1）。四边形最后画在哪里由着色器决定，网格本身的位置只影响剔除。
   - `MeshRenderer`：
     - Cast Shadows Off，Receive Shadows 关；
     - Light Probes、Reflection Probes 都 Off；
     - Motion Vectors 选 Force No Motion；
     - Dynamic Occlusion 关；
     - **默认关闭（`enabled = false`）**，由动画器打开。
2. **第一人称头部缩小**：本地玩家的 Head 骨骼在第一人称下会被 VRChat 缩到几乎为 0，它下面的物体也跟着缩没。在 `PosBeacon` 上加 `VRC Head Chop`，目标是它自己，缩放系数 1，Apply Condition 选 Always Apply。化身上已有别的 Head Chop 也没关系，每个化身最多 16 个。
3. 材质用下面 2.2 的着色器，属性用默认值，开 GPU Instancing。
4. **只在 bot 自己的客户端里打开**：用 Modular Avatar 合进 FX：
   - `MA Parameters`：bool `PosBeacon`，默认开、保存、只在本地（不同步，不占同步位，OSC 照样能改）。
   - `MA Merge Animator`：FX 层，路径相对 `PosBeacon`，跟随化身的 Write Defaults。层里两个状态，动画控制的是 `MeshRenderer.m_Enabled`：
     - `Off`（默认）→ `On`：`IsLocal` 且 `PosBeacon`；
     - `On` → `Off`：`IsLocal` 为假，或 `PosBeacon` 为假。
   - 别人的客户端里 `IsLocal` 恒为假，所以渲染器不打开；对方屏蔽了动画时，渲染器停在默认的关。
   - 开关的是渲染器，不是整个物体，这样 Head Chop 和 MA 组件始终挂在启用的物体上。
   - bridge 需要时可以用 OSC 发 `/avatar/parameters/PosBeacon` 打开或关掉它。
5. 化身上如果有构建时给所有渲染器自动加动画曲线的工具（比如统一限制亮度的灯光控制类工具），把 `PosBeacon` 加进它的排除列表。曲线对这个着色器没有作用，但每次构建都会改动生成的动画文件。

### 2.2 着色器

内置渲染管线，单通道立体实例化（Single Pass Instanced，VRChat 在 VR 下用的就是这种）。所有数据在顶点着色器里算好，用 `nointerpolation` 传给片元着色器，片元只负责按格子取比特。

```hlsl
Shader "xkeyC/PosBeacon"
{
    Properties
    {
        // The beacon draws only for a camera this near (metres, world): the
        // avatar's own eyes, not other players'.
        _OwnerRadius ("Owner radius", Float) = 0.35
    }
    SubShader
    {
        Tags { "Queue"="Overlay+100" "RenderType"="Overlay" "IgnoreProjector"="True" "DisableBatching"="True" "VRCFallback"="Hidden" }
        Pass
        {
            ZTest Always
            ZWrite Off
            Cull Off
            Blend Off

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            float _OwnerRadius;
            float _VRChatMirrorMode;
            float _VRChatCameraMode;

            // The marker: 22 x 10 blocks, each BLOCK wide in NDC, MARGIN in
            // from the eye's bottom-left corner.
            #define COLS 22
            #define ROWS 10
            #define BLOCK 0.01
            #define MARGIN 0.04
            #define MAGIC 0x5Au

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
                // x, y, z bits; magic
                nointerpolation uint4 a : TEXCOORD1;
                // yaw, pitch, seq, crc
                nointerpolation uint4 b : TEXCOORD2;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            // Bit i (0 first) of the 144 payload bits, most significant
            // bit of each field first.
            uint payload_bit(uint4 a, uint4 b, uint i)
            {
                if (i < 8)   return (a.w >> (7 - i)) & 1u;
                if (i < 40)  return (a.x >> (31 - (i - 8))) & 1u;
                if (i < 72)  return (a.y >> (31 - (i - 40))) & 1u;
                if (i < 104) return (a.z >> (31 - (i - 72))) & 1u;
                if (i < 120) return (b.x >> (15 - (i - 104))) & 1u;
                if (i < 136) return (b.y >> (15 - (i - 120))) & 1u;
                return (b.z >> (7 - (i - 136))) & 1u;
            }

            // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the bits.
            uint crc16(uint4 a, uint4 b)
            {
                uint crc = 0xFFFFu;
                [loop]
                for (uint i = 0; i < 144; i++)
                {
                    uint top = (crc >> 15) & 1u;
                    crc = (crc << 1) & 0xFFFFu;
                    if ((top ^ payload_bit(a, b, i)) != 0u) crc ^= 0x1021u;
                }
                return crc;
            }

            v2f vert(appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

                float3 cam = _WorldSpaceCameraPos; // this eye's camera
                float3 origin = mul(unity_ObjectToWorld, float4(0, 0, 0, 1)).xyz;
                bool own = distance(cam, origin) < _OwnerRadius;
                if (!own || _VRChatMirrorMode != 0 || _VRChatCameraMode != 0)
                {
                    o.pos = float4(2, 2, 2, 1); // outside the view: nothing drawn
                    return o;
                }

                // Where the camera looks (world): the view matrix's -Z row.
                float3 fwd = -normalize(UNITY_MATRIX_V[2].xyz);
                float yaw = degrees(atan2(fwd.x, fwd.z));           // clockwise from +Z, seen from above
                if (yaw < 0) yaw += 360.0;
                float pitch = degrees(asin(clamp(fwd.y, -1, 1)));   // + up

                o.a = uint4(asuint(cam.x), asuint(cam.y), asuint(cam.z), MAGIC);
                o.b = uint4((uint)round(yaw / 360.0 * 65536.0) & 0xFFFFu,
                            ((uint)(int)round(pitch * 100.0)) & 0xFFFFu,
                            ((uint)floor(_Time.y * 30.0)) & 0xFFu,
                            0u);
                o.b.w = crc16(o.a, o.b);

                // The quad's corners (uv 0..1) onto the marker's rectangle
                // at the eye's bottom-left, in clip space.
                float2 size = float2(COLS, ROWS) * BLOCK;
                float2 ndc = float2(-1.0 + MARGIN, -1.0 + MARGIN) + v.uv * size;
                o.pos = float4(ndc.x, ndc.y * _ProjectionParams.x, 0.0, 1.0);
                #if defined(UNITY_REVERSED_Z)
                    o.pos.z = 0.999;
                #else
                    o.pos.z = -0.999;
                #endif
                o.uv = v.uv;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                int col = (int)floor(i.uv.x * COLS);
                int row = (int)floor((1.0 - i.uv.y) * ROWS); // 0 at the top
                col = clamp(col, 0, COLS - 1);
                row = clamp(row, 0, ROWS - 1);
                uint bit;
                if (row == 0)
                    bit = (col & 1) == 0 ? 1u : 0u;          // top row: white, black, white...
                else if (row == ROWS - 1 || col == 0 || col == COLS - 1)
                    bit = 1u;                                  // the rest of the border: white
                else
                {
                    uint k = (uint)((row - 1) * (COLS - 2) + (col - 1)); // 0..159
                    bit = k < 144u ? payload_bit(i.a, i.b, k) : ((i.b.w >> (15u - (k - 144u))) & 1u);
                }
                return bit != 0u ? fixed4(1, 1, 1, 1) : fixed4(0, 0, 0, 1);
            }
            ENDCG
        }
    }
}
```

说明：

- `o.pos.z` 只要落在深度范围内就行（`ZTest Always`），上面按是否反向 Z 取一个靠近近裁剪面的值。
- `ndc.y * _ProjectionParams.x`：渲染到纹理时 Unity 会翻转 y，这样方格总在最终画面的左下角。万一实际出现在左上角也没关系，读码端两个角都会找。
- 如果你的 Unity 版本对 `UNITY_MATRIX_V` 在实例化立体下的取值有疑问，可以改用 `unity_StereoMatrixV[unity_StereoEyeIndex]`（需要先 `UNITY_SETUP_INSTANCE_ID`）。按 Unity 2022.3 的 `UnityShaderVariables.cginc`，立体实例化时 `UNITY_MATRIX_V` 和 `_WorldSpaceCameraPos` 本来就按眼取值，所以没改；还没在 VR 下实测。
- CRC 循环加 `[loop]`，避免编译器把 144 次循环展开；第 0 行用 `col & 1` 而不是 `col % 2`，后者有整数取模的性能警告。
- 上面就是实际用的着色器，原样可用。

## 3. 格式（两端的约定）

**位置**：每只眼画面（NDC，y 向上）左下角，左边界和下边界都内缩 0.04，宽 22 × 0.01，高 10 × 0.01。在 1920 像素宽的眼睛画面上，每格约 9.6 像素。

**格子**：22 列 × 10 行，行号从上往下数，白为 1、黑为 0。

- 第 0 行：白、黑、白、黑……（偶数列白）。读码端用它定位，并取白和黑的参考亮度。
- 第 9 行、第 0 列、第 21 列：全白。
- 第 1–8 行、第 1–20 列：数据，按行从左到右，共 160 位。

**数据（160 位，每个字段高位在前）**：

| 位 | 字段 | 说明 |
|---|---|---|
| 0–7 | magic | 0x5A（第 1 版） |
| 8–39 | x | 相机世界坐标，float32 的原始位 |
| 40–71 | y | 同上 |
| 72–103 | z | 同上 |
| 104–119 | yaw | 相机朝向，从世界 +Z 顺时针（俯视），`round(yaw/360×65536) mod 65536` |
| 120–135 | pitch | 抬头为正，`round(pitch×100)`，16 位补码 |
| 136–143 | seq | `floor(_Time.y×30) mod 256`，用来发现画面卡住 |
| 144–159 | crc | CRC-16/CCITT-FALSE（多项式 0x1021，初值 0xFFFF，不反转，不异或），按位计算，覆盖第 0–143 位 |

**坐标换算**（bridge 端做）：

- Unity 世界是左手系：+X 右、+Y 上、+Z 前。bot 的地图用右手系、-Z 为前，换算是 `x_map = x`、`y_map = y`、`z_map = -z`。
- 朝向：Unity 的 yaw（从 +Z 顺时针）正好等于 bot 的航向（从 -Z 往右为正），不用换算。
- 两只眼各自的位置取平均，就是头的位置。脚的位置是头的位置减去眼高（`/avatar/eyeheight`，世界米）。

## 4. 读码端（bridge）

1. 每次拿到眼睛画面，在左眼左下角（按 NDC 算出的矩形，以及上下翻转后的左上角）取每格中心 3×3 像素的平均亮度。
2. 第 0 行偶数列的平均值是白、奇数列是黑，阈值取两者中点；白和黑相差不到 80（0–255）就算没有角标。
3. 读出 160 位，校验 magic 和 CRC。不对就丢掉这一帧，不猜。
4. 右眼同样读一次，两眼距离应该接近两眼间距（随化身缩放）。差得太多就丢掉。
5. 读出的矩形区域要从双目匹配、文字识别和发给模型的画面里遮掉。

读到坐标后，地图改用世界坐标（`vrc_map::Nav::set`），不再依赖速度积分。读不到时（角标被关、换了化身）自动退回里程加配准。

## 5. 怎么验证

1. **Unity 里**（已做，全部通过）：在编辑模式下做，不进 Play 模式（Play 模式会触发 NDMF 处理，见 7.3）。渲染器默认是关的，测试前临时打开。
   - 着色器：建一台临时相机（视场 100°，渲染到 1280×1280 的纹理），放到化身视点上渲染，读回像素，按第 3、4 节的格式和步骤解码。

     | 相机 | 结果 |
     |---|---|
     | 视点处，朝向 yaw 0°/45°/180°/270°/300.5°、pitch −30°～+60° | magic 0x5A，CRC 对，读出的坐标、yaw、pitch 和相机一致，方格在左下角 |
     | 原地向左转 90° 并低头（头骨骼不动） | 同上（放大包围盒之前这里读不到，见 7.1） |
     | 离四边形 0.55 m、1.65 m，或站在 bot 对面 0.45 m 处 | 不画 |
     | 全局设 `_VRChatMirrorMode = 1` 或 `_VRChatCameraMode = 1` | 不画 |

   - 动画器：给一个只有 `MeshRenderer` 和 `Animator` 的临时物体挂上这个控制器，设参数后手动 `Animator.Update`。只有 `IsLocal` 和 `PosBeacon` 都为真时渲染器才开；本地运行中把 `PosBeacon` 关了再开，渲染器跟着关、开。
   - 合入化身：NDMF 烘焙后，参数表里有 `PosBeacon`（bool、默认 1、保存、不同步），FX 里有这一层，路径已改写成角标的完整路径，`VRC Head Chop` 保留。烘焙测试的注意事项见 7.3。

下面 2–4 还没做（要上传后在 VRChat 里测），另外要确认两件编辑器里测不了的事：单通道立体实例化时左右眼各画各的位置，以及第一人称下 Head Chop 生效、方格不会时有时无。

2. **VRChat 里，bot 自己**：bridge 的读码接口（实现后为 `GET /v1/vr/beacon`）返回坐标和朝向。原地转头时位置基本不变、yaw 跟着变；往前走 1 m，位置变 1 m。
3. **别人看**：请一位好友看 bot 的脸，**凑到面对面几十厘米内**也不应该看到任何方块（见 7.2）；在镜子前、用拍照相机拍 bot，也都看不到。
4. **跨次**：退出世界再进来，站到同一个地方，读出的坐标应该和上次一致（误差在厘米级）。

## 6. 注意

- 第一人称头部缩小处理不好时，方格会时有时无。先查 2.1 第 2 步。
- 方格在每只眼画面左下角占约 11% × 5%，bot 视野的这块就看不到了。左下角离视线中心远，影响不大。
- 世界的后处理（泛光、调色）会让黑白不够纯，但黑白对比还在。读码端用第 0 行的参考亮度定阈值，不用固定值。
- 这是纯着色器方案，不依赖世界，也不需要 Udon；对化身性能的影响只有一个材质、一个四边形。
- 化身上传期间不要做测试烘焙或进 Play 模式，见 7.3。

## 7. 实现时踩过的坑

### 7.1 相机转开后，四边形被视锥剔除

- 现象：相机在视点处原地向左转 90° 并低头、头骨骼不动时，方格不画了。
- 原因：Unity 按网格包围盒做视锥剔除。自带 Quad 的包围盒只有 5 cm 见方，在眼前 15 cm，相机一转开它就出了视锥，着色器根本没运行。方格画在哪里由着色器决定，和网格位置无关，所以这次剔除是多余的。
- 实际 VR 里头骨骼跟着头显转，四边形大多留在视锥里，但不该依赖这一点（头部 IK 跟不上、视角转动超过颈部限制时都可能出视锥）。
- 解决：复制一份 Quad，把包围盒放大到约 1 m 见方（网格本地尺寸 1 / 0.05 = 20）。相机在头附近时总在包围盒里，不会被剔除。

### 7.2 别的玩家凑近能看到方格

- 原方案只靠 `_OwnerRadius` 判断"是不是自己的眼睛"：相机离四边形不到 0.35 m 就画。四边形在 bot 眼前 15 cm，另一个玩家和 bot 面对面、两人眼睛相距约 45 cm 时，他的相机离四边形只有约 30 cm，就会画。挨近说话、摸头、贴脸都在这个距离内。
- 开关参数 `PosBeacon` 只在本地，别人的客户端里一直是默认值"开"，挡不住。
- 解决：渲染器默认关，FX 只在 `IsLocal` 且 `PosBeacon` 时打开（2.1 第 4 步）。别人的客户端里 `IsLocal` 恒为假；屏蔽了动画时停在默认的关。
- 开关渲染器而不开关整个物体：`VRC Head Chop` 和 MA 组件挂在这个物体上，物体在加载时是关着的时，这些组件是否照常生效没有验证过，不去赌。
- `_OwnerRadius` 留着，在 bot 自己的客户端里挡住头附近以外的相机。

### 7.3 测试烘焙不能和上传同时做

- NDMF 把生成的资源（合并后的动画控制器、菜单、参数等）写到 `__Generated/<化身物体名>/`。用 `Object.Instantiate` 复制出来的化身叫 `<名字>(Clone)`，和 SDK 构建用的副本同名，写的是同一个目录。
- SDK 构建是异步的，编辑器脚本可以插在构建中间运行。上传时跑测试烘焙，会把构建刚生成的资源换掉，上传的包里就没有合并后的 FX 和菜单。
- 所以：上传期间不做测试烘焙，也不进 Play 模式（同样会触发 NDMF）；要做烘焙，副本换一个不和化身重名的名字。验证尽量用第 5 节第 1 步那样不触发 NDMF 的办法。
