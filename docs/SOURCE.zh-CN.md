# 来源与目录

官方 Android LTS 提交：`44c4d19047cd3c9c51c6fa3930c35844b10b5f9a`，
仓库 `https://android.googlesource.com/kernel/common`，分支 `android17-6.18-lts`。
原参考项目与固定输入记录保存在 provenance/；历史记录中的 6.18.32 表示补丁基线。
当前可直接构建的完整输入位于 sources/，无需重新下载历史基线或重复应用补丁。

- `sources/k618/ws/common/`：Android 通用内核与已适配的板级核心改动。
- `sources/k618/soc/`：Qualcomm SoC 驱动，含 R2 SMMU 同步修复。
- `sources/k618/vendor/`：相机、显示、音频、网络等厂商源码。
- `sources/k618/sheng-ports/`：平板外设驱动。
- `patches/` 与 `provenance/`：补丁、来源及兼容性校验记录。
- `android/`：当前 Android 配置、设备集成补丁和模块收集工具。
- `validation/`：构建、校验和实机观察状态。

sources/ 保留构建所需生成头文件，并移除编译输出、模块二进制和缓存。
SOURCE-SHA256.json 记录全部输入文件和符号链接。源码保留原有许可证和归属。
