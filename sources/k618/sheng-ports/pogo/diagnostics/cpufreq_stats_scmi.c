// SPDX-License-Identifier: GPL-2.0-only
/* Derived from pinned sheng-u-oss drivers/soc/qcom/dcvs/cpufreq_stats_scmi.c.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Keep the actual firmware statistics and original debugfs interface.
 */
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/scmi_protocol.h>
#include <linux/slab.h>
#include "scmi_cpufreq_stats.h"
#include "scmi_stats_layout.h"

struct stats_info;
struct stats_entry {
	struct stats_info *info;
	unsigned int levels;
	size_t offset;
	u32 *frequencies;
	unsigned int field;
};
struct stats_info {
	void __iomem *memory;
	size_t size;
	struct dentry *directory;
	struct stats_entry *entries;
};

static int stats_snapshot(struct stats_entry *entry, u64 *values)
{
	struct stats_info *info = entry->info;
	unsigned int attempt, i;
	u32 before, after;
	void __iomem *domain = info->memory + entry->offset;

	/* The descriptor was bounds-checked before it was exposed to debugfs. */
	if ((readl(domain) & 255) != entry->levels)
		return -EAGAIN;
	for (attempt = 0; attempt < 128; attempt++) {
		before = readl(info->memory + 12);
		if (before & 1) {
			cpu_relax();
			continue;
		}
		for (i = 0; i < entry->levels; i++) {
			void __iomem *value = domain + 16 + i * 24 + entry->field;
			values[i] = readl(value) | (u64)readl(value + 4) << 32;
		}
		rmb();
		after = readl(info->memory + 12);
		if (before == after && !(after & 1))
			return 0;
		cpu_relax();
	}
	return -EAGAIN;
}

static ssize_t stats_get(struct file *file, char __user *buffer,
		size_t count, loff_t *position)
{
	struct stats_entry *entry = file->private_data;
	u64 *values = NULL;
	char *text = NULL;
	ssize_t rc;
	size_t length = 0;
	unsigned int i;

	rc = debugfs_file_get(file->f_path.dentry);
	if (rc)
		return rc;
	if (!entry) {
		rc = -ENOENT;
		goto out;
	}
	if (!entry->levels) {
		rc = 0;
		goto out;
	}
	values = kcalloc(entry->levels, sizeof(*values), GFP_KERNEL);
	text = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!values || !text) {
		rc = -ENOMEM;
		goto out;
	}
	rc = stats_snapshot(entry, values);
	if (rc)
		goto out;
	for (i = 0; i < entry->levels && length < PAGE_SIZE - 1; i++)
		length += scnprintf(text + length, PAGE_SIZE - length, "%u %llu\n",
				entry->frequencies[i], values[i]);
	rc = simple_read_from_buffer(buffer, count, position, text, length);
out:
	kfree(text);
	kfree(values);
	debugfs_file_put(file->f_path.dentry);
	return rc;
}
static const struct file_operations stats_ops = {
	.owner = THIS_MODULE, .read = stats_get, .open = simple_open,
	.llseek = default_llseek,
};

static int scmi_cpufreq_stats_probe(struct scmi_device *sdev)
{
	const struct scmi_cpufreq_stats_vendor_ops *ops;
	struct scmi_protocol_handle *protocol;
	struct cpufreq_stats_prot_attr response;
	struct stats_info *info;
	struct dentry *domain, *file;
	unsigned int domains, d, i, field, levels;
	size_t offset;
	u64 address;
	u32 word_offset;
	char name[16];
	int rc;

	if (!sdev || !sdev->handle)
		return -ENODEV;
	ops = sdev->handle->devm_protocol_get(sdev, SCMI_CPUFREQ_STATS_PROTOCOL, &protocol);
	if (IS_ERR(ops))
		return PTR_ERR(ops);
	rc = ops->cpufreq_stats_info_get(protocol, &response);
	if (rc)
		return rc;
	info = devm_kzalloc(&sdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->size = le32_to_cpu(response.statistics_len);
	address = le32_to_cpu(response.statistics_address_low) |
			(u64)le32_to_cpu(response.statistics_address_high) << 32;
	if (info->size < 16 || !address)
		return -EPROTO;
	info->memory = devm_ioremap(&sdev->dev, address, info->size);
	if (!info->memory)
		return -ENOMEM;
	if (readl(info->memory) != 0x50455246 || (readl(info->memory + 4) & 255) != 1)
		return -EPROTO;
	domains = readl(info->memory + 8) & 255;
	if (!domains || domains * 4 > info->size - 16)
		return -EPROTO;
	info->entries = devm_kcalloc(&sdev->dev, domains * 2, sizeof(*info->entries), GFP_KERNEL);
	if (!info->entries)
		return -ENOMEM;
	/* Prepare every descriptor and cache before exposing any file. */
	for (d = 0; d < domains; d++) {
		u32 *frequencies;
		word_offset = readl(info->memory + 16 + d * 4);
		if (!scmi_stats_domain_range(info->size, word_offset, 0, &offset) ||
		    offset < 16 + domains * 4)
			return -EPROTO;
		levels = readl(info->memory + offset) & 255;
		if (!scmi_stats_domain_range(info->size, word_offset, levels, &offset))
			return -EPROTO;
		frequencies = devm_kcalloc(&sdev->dev, levels, sizeof(*frequencies), GFP_KERNEL);
		if (!frequencies && levels)
			return -ENOMEM;
		for (i = 0; i < levels; i++)
			frequencies[i] = readl(info->memory + offset + 16 + i * 24);
		for (field = 0; field < 2; field++) {
			struct stats_entry *entry = &info->entries[d * 2 + field];
			entry->info = info;
			entry->levels = levels;
			entry->offset = offset;
			entry->frequencies = frequencies;
			entry->field = field ? 16 : 8;
		}
	}
	info->directory = debugfs_create_dir("cpufreq_stats", NULL);
	if (IS_ERR(info->directory))
		return PTR_ERR(info->directory);
	for (d = 0; d < domains; d++) {
		snprintf(name, sizeof(name), "clkdom%u", d);
		domain = debugfs_create_dir(name, info->directory);
		if (IS_ERR(domain)) { rc = PTR_ERR(domain); goto error; }
		for (field = 0; field < 2; field++) {
			file = debugfs_create_file(field ? "time_in_state" : "usage", 0400,
					domain, &info->entries[d * 2 + field], &stats_ops);
			if (IS_ERR(file)) { rc = PTR_ERR(file); goto error; }
		}
	}
	dev_set_drvdata(&sdev->dev, info);
	return 0;
error:
	debugfs_remove(info->directory);
	return rc;
}
static void scmi_cpufreq_stats_remove(struct scmi_device *sdev)
{
	struct stats_info *info = dev_get_drvdata(&sdev->dev);
	debugfs_remove(info->directory);
}
static const struct scmi_device_id scmi_id_table[] = {
	{ .protocol_id = SCMI_CPUFREQ_STATS_PROTOCOL, .name = "scmi_cpufreq_stats_protocol" }, {}
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);
static struct scmi_driver scmi_cpufreq_stats_drv = {
	.name = "scmi-cpufreq-stats-driver", .probe = scmi_cpufreq_stats_probe,
	.remove = scmi_cpufreq_stats_remove, .id_table = scmi_id_table,
};
module_scmi_driver(scmi_cpufreq_stats_drv);
MODULE_SOFTDEP("pre: cpufreq_stats_vendor");
MODULE_DESCRIPTION("ARM SCMI CPUFREQ STATS driver");
MODULE_LICENSE("GPL v2");
