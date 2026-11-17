# 6.18.39 KSU 兼容性

内核保留已验证的 6.18.32 task/thread/mm 布局，并恢复 SP_EL0 的 current 与每任务栈保护约定。
启动、异常与溢出栈、任务切换、兼容 TLS、KVM、休眠恢复和 BPF 的相关路径同步保持一致；其余 6.18.39 更新保留。
板级模块必须与该内核成套重建，不能混用先前 TPIDRRO_EL0 版本的模块。

KernelSU 管理器 3.3.1（32680）生成的原始 init_boot 与提取的 6.18.32 LKM 已验证加载及 Android 启动。
测试未对 KSU 模块做寄存器指令替换，包括正式启动参数下的重启回归。
匹配模块通过 ABI、BTF、依赖与命名空间检查；源码清单和 Android LTS 校验链同步更新。
完整技术校验范围与镜像、模块哈希见 provenance/ksu-sp-el0-compatibility.json。

默认 init_boot 不内置 KSU，不预装 root。正式启动的诊断自动返回时限为 0，不应进入限时 Fastboot。
需要 root 时可再修补 init_boot；这里只验证上述版本和模块，不保证任意其他内核 ABI 的预编译模块。
禁止仅改 vermagic 或跳过类型布局校验。
