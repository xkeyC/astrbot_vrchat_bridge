# 步态参数：从 CMU 动作捕捉数据拟合

`crates/vrc-vr/src/anim.rs` 里走路和跑步的参数从这里来（见 `docs/full-vr/decisions.md` D25）。我们只能控制头和双手，所以只统计这三处的运动。

## 数据

- 来源：[CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu)（任何用途免费，不得转售数据本身）。BVH 格式取自 [una-dinosauria/cmu-mocap](https://github.com/una-dinosauria/cmu-mocap)，帧率 120 fps。
- 用到的片段：受试者 07、08、09、16、35 的直线行走和跑步，共 70 个，按 CMU 的片段说明挑选。
- 数据本身不入库。复现时：
  1. 把 BVH 放到 `bvh/` 目录；
  2. 把 CMU 的片段索引（制表符分隔：片段名、说明）存成 `index.txt`；
  3. 在本目录下运行脚本。

## 脚本

1. `bvhfk.py`：BVH 解析和正向运动学。
2. `analyze.py`：读取 `bvh/*.bvh`，按 Zeni 法检测足跟触地并切出步幅周期，然后转到体坐标系（只保留 yaw；x 右、y 上、z 后，原点为周期内的平均眼点），按眼高 1.6 m 缩放，输出 `cycles.npy`。
3. `stats.py`：按速度分为四档（慢走、正常、快走、跑），对受试者等权平均，打印统计表，输出 `gait_curves.csv` 和 `bins.npy`。
4. `fourier.py`：按 m + A1·cos 2π(φ−p1) + A2·cos 4π(φ−p2) 拟合各项曲线。

## 结果

`gait_curves.csv` 是 4 档速度、每档 20 个相位采样的平均曲线，包括：
- 头的位置和三个角度；
- 髋的位置；
- 左右腕的位置、pitch、twist，以及手轴和掌心法向。

相位 0 为左脚触地，0.5 为右脚触地。注意 `head_pitch` 含 CMU 头骨的静息偏置（约 −15°），只能用它的波动部分。
