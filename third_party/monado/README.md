# Monado（OpenXR 运行时）· 本仓库的补丁

上游：https://gitlab.freedesktop.org/monado/monado （BSL-1.0）

Base commit: `cfa6078b8d7f368fa5296f540a436d59833e496c` （main，项目版本 25.1.0；2026-10-05 从 GitLab 下载的 `main` zip 快照，提交号取自 zip 注释）

bot 以无头方式运行 `monado-service`：

- **remote 驱动**（上游 `src/xrt/drivers/remote`）：头显、两只眼、两个手柄的状态全部由一个 TCP 客户端设定（`crates/vrc-vr/src/remote.rs`）。
- **null 合成器**（上游 `src/xrt/compositor/null`）：接收应用提交的帧但不显示（没有显示器也没有窗口），帧率由 `XRT_COMPOSITOR_NULL_FPS` 控制。

## 补丁

| 补丁 | 内容 | 原因 |
|---|---|---|
| `patches/0001-null-compositor-frame-tap.patch` | 新增 `null_tap.c/.h`：每次提交时（每秒最多 `XRT_NULL_TAP_FPS` 次，默认 10）把第一个投影层的左右眼从应用的交换链拷进主机可见缓冲区，再写入文件 `XRT_NULL_TAP`（512 字节头：帧号、显示时间、每只眼的位姿和视场角；后接像素），用 seqlock 保证读到完整帧。 | null 合成器会丢掉画面，而 bot 需要渲染出的双眼图像以及渲染时的精确位姿（双目深度、OCR、给模型看画面）。 |

抓帧要求应用的交换链带 `TRANSFER_SRC` 用法，见 `../xrizer/patches/0001-swapchain-transfer-src.patch`。拷贝用 `vkCmdCopyImageToBuffer` 同步等待完成（960x1080 双眼每帧 8 MB，GPU 上远不到 1 毫秒）。

应用补丁：`cd <monado> && patch -p1 < 0001-null-compositor-frame-tap.patch`（`ops/vr/build_monado.sh` 会自动做）。
