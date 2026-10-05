# 修改点

## 本仓库（feat/full_vr）

| 位置 | 改动 |
|---|---|
| 根目录 | 新增 Rust workspace（`Cargo.toml`），`.gitignore` 加 `/target/` |
| `crates/vrc-vr` | 新增。`pose`：位姿、视场角、由视场角求内参（fx fy cx cy）、按偏航/俯仰构造朝向。`remote`：Monado remote 驱动的 376 字节协议编解码，以及 TCP 客户端 `RemoteHmd`。`tap`：读取抓帧共享内存（seqlock、文件尺寸变化时重新映射、RGBA/BGRA 转 RGB）。6 个单元测试。 |
| `crates/vr-probe` | 新增。命令行工具：`info` / `grab` / `look` / `sweep`（转头后等到按新姿态渲染的帧再存图） |
| `astrbot_plugin/` | 由根目录移入：`main.py`、`vrchat_adapter.py`、`metadata.yaml`、`logo.svg`，内容未改 |
| `legacy/bridge/` | 由 `bridge/` 移入，内容未改（桌面模式仍可用） |
| `third_party/` | Monado、xrizer 的基础提交说明和补丁 |
| `ops/vr/` | 构建脚本（Monado、xrizer）、安装脚本、`vrc-launch.sh`（按 `~/.config/vrc-mode` 切换桌面/VR）、Monado 配置、`openvrpaths.vrpath` 模板、`vrc-monado.service` 用户单元 |
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
