# 构建

使用 Linux 或 WSL2，在 Linux 原生文件系统解压。项目和 BUILD_ROOT 路径不能含空格；
BUILD_ROOT 必须位于项目目录之外，并使用全新的空目录。

安装 Python 3.11+、GNU make、GCC、Git、rsync、jq、flex、bison、bc、cpio、kmod、
libssl-dev、libelf-dev、zlib1g-dev 和 zstd。构建限流器需要 root 或已授权的 cgroup v2。
默认 4 个编译任务、18 GiB 内存上限；完整构建建议准备 64 GiB 交换空间。
本项目的 Windows/WSL 构建与交换空间使用 C 盘承载；E 盘可保存交付压缩包。

工具链仓库 URL 与提交见 `provenance/toolchains.json`。PREBUILTS 下的布局：

```text
clang/host/linux-x86/clang-r584948c/
rust-toolchain/linux-x86/1.91.1.p3/
clang-tools/linux-x86/
kernel-build-tools/linux_musl-x86/
build-tools/linux-x86/
```

ELFUTILS 指向带 zstd 支持的 elfutils 0.193 安装前缀，包含 include/ 和 lib/。

```sh
export BUILD_ROOT=/root/sheng-6.18.39-r2-build
export PREBUILTS=/path/to/prebuilts
export ELFUTILS=/path/to/elfutils-0.193
export PYTHON=python3.11
export JOBS=4
bash build.sh
```

`build.sh` 顺序准备全部源码、编译核心 Image、编译全部模块、验证模块。
也可分别执行 `bash scripts/prepare.sh`、`bash buildkernel.sh`、
`bash buildmodules.sh`、`bash verify.sh`。配置预检使用 `bash buildkernel.sh --config-only`。
源码准备会核对所有文件哈希，并拒绝复用其他源码版本的目录。

核心产物位于 `$BUILD_ROOT/k618/out-gki/`：kernel-Image、kernel.config、vmlinux、
Module.symvers 与模块 BTF 基准。模块在各自 Kbuild 目录中生成；完整收集和验证报告位于
`$BUILD_ROOT/qualified/`。参考映射有 345 个模块，Android 收集策略还包含核心及新增模块。
Android 产品排除独立批量加载器 k618_load，驱动由 init 和 DLKM 正常加载。

该源码构建生成 Image 与模块。Android boot、vendor_boot、recovery、vendor_dlkm 等
分区需要匹配的 ROM 构建树、模块签名和 AVB 配置；`android/` 提供当前设备侧集成资料。
构建入口不会刷写设备。
