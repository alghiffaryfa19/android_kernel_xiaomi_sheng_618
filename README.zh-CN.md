# 小米平板 6S Pro 内核源码

Android 官方 `android17-6.18-lts`，Linux 6.18.39，设备 sheng。
内核版本名：`6.18.39-android17-0-peiyuesheng-sheng-kernel`。

`sources/k618/` 是 R2 已适配的完整内核、Qualcomm SoC、厂商模块与平板驱动源码。
其中包含 SMMU IOTLB gather 修复、直接霍尔中断，以及当前 Android 硬件适配。
构建脚本从这些源码建立独立目录；默认使用 release 配置。

[构建说明](docs/BUILD.zh-CN.md) · [来源与目录](docs/SOURCE.zh-CN.md)

构建与验证结果保存在 `validation/` 的 JSON 文件。
工具链是外部依赖，固定版本见 `provenance/toolchains.json`。
各上游目录的许可证、版权说明与 MAINTAINERS 文件随源码保留。
