# xrizer（基于 OpenXR 的 OpenVR 实现）· 本仓库的补丁

上游：https://github.com/Supreeeme/xrizer （GPL-3.0）

Base commit: `0989a7fac2d1efb7ea82f5fe1a8ed30c3eeb9596` （main，2026-09-03，"implement GetOverlayTransformAbsolute"）

VRChat 使用 OpenVR。在 Proton 下，它的 `openvr_api.dll` 交给 Proton 的 `vrclient`，后者加载 `~/.config/openvr/openvrpaths.vrpath`（或 `VR_OVERRIDE`）指定的 Linux 运行时——这里是 xrizer 而不是 SteamVR。xrizer 再通过 OpenXR 连到 Monado（`XR_RUNTIME_JSON`）。

## 补丁

| 补丁 | 内容 | 原因 |
|---|---|---|
| `patches/0001-swapchain-transfer-src.patch` | Vulkan 后端：眼睛交换链在 `COLOR_ATTACHMENT`、`TRANSFER_DST` 之外再加 `TRANSFER_SRC` 用法。 | Monado 的抓帧要从这个交换链里拷出图像；用法必须在创建交换链时声明（Monado 按应用给的创建参数分配图像）。 |

编译：`cargo xbuild --release`（bindgen 需要 clang）；运行时目录是 `target/release`（`bin/linux64/vrclient.so`）。`ops/vr/build_xrizer.sh` 会检出上面的基础提交并打补丁。
