// SPDX-License-Identifier: GPL-2.0-only
/* Derived from pinned sheng-u-oss drivers/xiaomi/dump_display.c.
 * Copyright (C) 2022 YangHui.
 * Keep the shipping crash record, with bounded writes and device lifetime.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/panic_notifier.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/stacktrace.h>
#include <linux/sched.h>

#define CRASH_RECORD_MAGIC 0x12345678
#define TRACE_INFO_SIZE 2044
struct crash_info { u32 magic; char back_trace[TRACE_INFO_SIZE]; };
struct dump_display {
	void __iomem *region;
	struct notifier_block panic;
	raw_spinlock_t lock;
	char trace[TRACE_INFO_SIZE];
};

static size_t trace_snprint(char *buf, size_t size,
		const unsigned long *entries, unsigned int count)
{
	size_t written = 0;
	unsigned int i;

	if (!size)
		return 0;
	buf[0] = '\0';
	for (i = 0; entries && i < count && written < size - 1; i++)
		written += scnprintf(buf + written, size - written,
				"\n\r%pS", (void *)entries[i]);
	written += scnprintf(buf + written, size - written,
			"\n\rcomm:%s", current->comm);
	return written;
}

static int set_backtrace_msg(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct dump_display *info = container_of(nb, struct dump_display, panic);
	unsigned long entries[30], flags;
	unsigned int count;

	/* A nested panic must never deadlock on the first panic's record. */
	if (!raw_spin_trylock_irqsave(&info->lock, flags))
		return NOTIFY_DONE;
	count = stack_trace_save(entries, ARRAY_SIZE(entries), 2);
	if (count) {
		memset(info->trace, 0, sizeof(info->trace));
		trace_snprint(info->trace, sizeof(info->trace), entries, count);
		memcpy_toio(info->region + offsetof(struct crash_info, back_trace),
				info->trace, sizeof(info->trace));
		wmb();
	}
	raw_spin_unlock_irqrestore(&info->lock, flags);
	return NOTIFY_DONE;
}

static int dump_display_probe(struct platform_device *pdev)
{
	struct dump_display *info;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < sizeof(struct crash_info))
		return -EINVAL;
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	/* This is the firmware's shared reserved record, as in the stock driver. */
	info->region = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!info->region)
		return -ENOMEM;
	raw_spin_lock_init(&info->lock);
	info->panic.notifier_call = set_backtrace_msg;
	memset_io(info->region, 0, resource_size(res));
	iowrite32(CRASH_RECORD_MAGIC, info->region);
	memcpy_toio(info->region + offsetof(struct crash_info, back_trace),
			"UNKNOWN", sizeof("UNKNOWN"));
	wmb();
	platform_set_drvdata(pdev, info);
	return atomic_notifier_chain_register(&panic_notifier_list, &info->panic);
}

static void dump_display_remove(struct platform_device *pdev)
{
	struct dump_display *info = platform_get_drvdata(pdev);
	atomic_notifier_chain_unregister(&panic_notifier_list, &info->panic);
}

static const struct of_device_id ddp_match[] = {
	{ .compatible = "xiaomi,dump_display" }, {}
};
MODULE_DEVICE_TABLE(of, ddp_match);
static struct platform_driver dump_display_driver = {
	.probe = dump_display_probe, .remove = dump_display_remove,
	.driver = { .name = "dump_display", .of_match_table = ddp_match },
};
module_platform_driver(dump_display_driver);
MODULE_LICENSE("GPL v2");
