# 6.18.39 KSU 兼容修复

2026-11-17 已合入本地 6.18.32 的 task/thread/mm ABI 兼容布局及已测试的 rawdump 记录器改进。
保留 6.18.39 内核版本；板级模块必须与此布局成套重建，禁止混用旧 .39 模块。

实机已正常进入 Android，用户确认 KSU 管理器显示工作中。独立 namespace 的原 ksuinit 加载、配套模块 ABI/BTF 检查均通过。
源码校验清单和 Android LTS 校验链同步更新，不跳过校验。

默认交付的 init_boot 不内置 KSU。用户需要 root 时再选用匹配的 LKM。
此前提取的 .32 预编译模块使用 SP_EL0，而 .39 的 current 使用 TPIDRRO_EL0；仅恢复字段偏移并不能让任意旧模块兼容。
实机成功路径使用了经过精确哈希、280 条 MRS 指令校验的模块适配；该二进制适配不是通用补丁器。
后续可从本内核的源码、配置重编 LKM。不得只改 vermagic 或把任意官方预编译模块标记为已验收。

默认启动不启用诊断自动返回时限。需要诊断时单独启用 bootlog 并按每轮参数设置 60 秒；不把超时带入普通启动。
完整 ROM 必须同时更新 boot、vendor_boot、recovery、vendor_dlkm、system_dlkm 及对应 AVB 描述符，不能仅更换内核或混用旧模块。
未修补的 init_boot 与 KSU 兼容内核可以配套交付；兼容性修复不代表系统已预置 Root。
