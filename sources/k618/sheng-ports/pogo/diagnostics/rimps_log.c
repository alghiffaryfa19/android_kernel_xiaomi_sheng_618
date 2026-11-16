// SPDX-License-Identifier: GPL-2.0-only
/* Derived from pinned sheng-u-oss drivers/soc/qcom/rimps_log.c.
 * Copyright (c) 2020 The Linux Foundation.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Real firmware mailbox/MMIO logs with bounded payloads and paired cleanup.
 */
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "rimps_payload.h"
#define CREATE_TRACE_POINTS
#include "trace_cpucp.h"

#define MAX_PRINT_SIZE 1024
#define MAX_BUF_NUM 4
struct rimps_buf {
	struct list_head node;
	char *bytes;
	size_t length;
};
struct rimps_log_info {
	void __iomem *regions[2];
	size_t region_size;
	struct mbox_client client;
	struct mbox_chan *channel;
	struct work_struct work;
	struct workqueue_struct *queue;
	spinlock_t lock;
	struct list_head free, full;
	struct rimps_buf buffers[MAX_BUF_NUM];
	bool stopping;
	size_t carry_length;
	char carry[MAX_PRINT_SIZE];
};

static void rimps_log_work(struct work_struct *work)
{
	struct rimps_log_info *info = container_of(work, struct rimps_log_info, work);
	struct rimps_buf *buffer;
	unsigned long flags;
	char text[MAX_PRINT_SIZE + 1], *source;
	size_t size, chunk, count;
	bool newline;

	for (;;) {
		spin_lock_irqsave(&info->lock, flags);
		if (list_empty(&info->full)) {
			spin_unlock_irqrestore(&info->lock, flags);
			return;
		}
		buffer = list_first_entry(&info->full, struct rimps_buf, node);
		list_del(&buffer->node);
		spin_unlock_irqrestore(&info->lock, flags);
		source = buffer->bytes + MAX_PRINT_SIZE - info->carry_length;
		size = buffer->length + info->carry_length;
		memcpy(source, info->carry, info->carry_length);
		info->carry_length = 0;
		while (size) {
			chunk = min_t(size_t, size, MAX_PRINT_SIZE);
			newline = get_last_newline(source, chunk, &count);
			if (!newline && size < MAX_PRINT_SIZE) {
				memcpy(info->carry, source, size);
				info->carry_length = size;
				break;
			}
			/* Trace never scans beyond the received fragment for a NUL. */
			memcpy(text, source, count);
			text[count] = '\0';
			trace_cpucp_log(text);
			source += count;
			size -= count;
		}
		spin_lock_irqsave(&info->lock, flags);
		list_add_tail(&buffer->node, &info->free);
		spin_unlock_irqrestore(&info->lock, flags);
	}
}

static void rimps_log_rx(struct mbox_client *client, void *message)
{
	struct rimps_log_info *info = container_of(client, struct rimps_log_info, client);
	struct rimps_payload payload;
	struct rimps_buf *buffer;
	unsigned long flags;
	u32 marker;

	spin_lock_irqsave(&info->lock, flags);
	if (info->stopping || list_empty(&info->free))
		goto unlock;
	marker = readl(info->regions[0]);
	if (!rimps_payload_decode(marker, info->region_size, &payload))
		goto unlock;
	buffer = list_first_entry(&info->free, struct rimps_buf, node);
	list_del(&buffer->node);
	memcpy_fromio(buffer->bytes + MAX_PRINT_SIZE,
			info->regions[payload.region] + payload.offset, payload.length);
	buffer->length = payload.length;
	list_add_tail(&buffer->node, &info->full);
	/* Queue under the same lock as stopping; remove cannot race a new queue. */
	queue_work(info->queue, &info->work);
unlock:
	spin_unlock_irqrestore(&info->lock, flags);
}

static int rimps_log_probe(struct platform_device *pdev)
{
	struct rimps_log_info *info;
	struct resource *res[2];
	unsigned int i;

	res[0] = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res[1] = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res[0] || !res[1] || platform_get_resource(pdev, IORESOURCE_MEM, 2) ||
	    resource_size(res[0]) != resource_size(res[1]) ||
	    resource_size(res[0]) < 4 || resource_size(res[0]) > U32_MAX / 2)
		return -EINVAL;
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->region_size = resource_size(res[0]);
	spin_lock_init(&info->lock);
	INIT_LIST_HEAD(&info->free);
	INIT_LIST_HEAD(&info->full);
	INIT_WORK(&info->work, rimps_log_work);
	for (i = 0; i < 2; i++) {
		info->regions[i] = devm_ioremap(&pdev->dev, res[i]->start, resource_size(res[i]));
		if (!info->regions[i])
			return -ENOMEM;
	}
	for (i = 0; i < MAX_BUF_NUM; i++) {
		info->buffers[i].bytes = devm_kmalloc(&pdev->dev,
				info->region_size + MAX_PRINT_SIZE, GFP_KERNEL);
		if (!info->buffers[i].bytes)
			return -ENOMEM;
		list_add_tail(&info->buffers[i].node, &info->free);
	}
	info->client.dev = &pdev->dev;
	info->client.tx_block = false;
	info->client.knows_txdone = true;
	info->client.rx_callback = rimps_log_rx;
	/* Preserve the stock logger's freezable queue, with a per-device owner. */
	info->queue = alloc_workqueue("rimps_wq", WQ_FREEZABLE | WQ_UNBOUND, 1);
	if (!info->queue)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);
	info->channel = mbox_request_channel(&info->client, 0);
	if (IS_ERR(info->channel)) {
		WRITE_ONCE(info->stopping, true);
		destroy_workqueue(info->queue);
		return PTR_ERR(info->channel);
	}
	return 0;
}

static void rimps_log_remove(struct platform_device *pdev)
{
	struct rimps_log_info *info = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	info->stopping = true;
	spin_unlock_irqrestore(&info->lock, flags);
	mbox_free_channel(info->channel);
	cancel_work_sync(&info->work);
	destroy_workqueue(info->queue);
}
static const struct of_device_id rimps_log_match[] = {
	{ .compatible = "qcom,rimps-log" }, {}
};
MODULE_DEVICE_TABLE(of, rimps_log_match);
static struct platform_driver rimps_log_driver = {
	.probe = rimps_log_probe, .remove = rimps_log_remove,
	.driver = { .name = "rimps-log", .of_match_table = rimps_log_match },
};
module_platform_driver(rimps_log_driver);
MODULE_LICENSE("GPL v2");
