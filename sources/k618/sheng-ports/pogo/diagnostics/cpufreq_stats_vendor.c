// SPDX-License-Identifier: GPL-2.0-only
/* Derived from pinned sheng-u-oss SCMI protocol 0x84 implementation.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include "common.h"
#include "scmi_cpufreq_stats.h"
#define SCMI_MAX_RX_SIZE 128

static int scmi_get_cpufreq_stats_info(const struct scmi_protocol_handle *ph, void *buf)
{
	struct scmi_xfer *transfer;
	struct cpufreq_stats_prot_attr *info = buf;
	int rc;

	if (!buf)
		return -EINVAL;
	/* Preserve stock message 16 and its 16-byte request/response layout. */
	rc = ph->xops->xfer_get_init(ph, 16, sizeof(*info), SCMI_MAX_RX_SIZE, &transfer);
	if (rc)
		return rc;
	memset(transfer->tx.buf, 0, sizeof(*info));
	rc = ph->xops->do_xfer(ph, transfer);
	if (!rc) {
		if (transfer->rx.len != sizeof(*info))
			rc = -EPROTO;
		else
			memcpy(info, transfer->rx.buf, sizeof(*info));
	}
	ph->xops->xfer_put(ph, transfer);
	return rc;
}

static int scmi_set_log_level(const struct scmi_protocol_handle *ph, u32 level)
{
	struct scmi_xfer *transfer;
	int rc;

	rc = ph->xops->xfer_get_init(ph, 3, sizeof(u32), sizeof(u32), &transfer);
	if (rc)
		return rc;
	put_unaligned_le32(level, transfer->tx.buf);
	rc = ph->xops->do_xfer(ph, transfer);
	ph->xops->xfer_put(ph, transfer);
	return rc;
}
static const struct scmi_cpufreq_stats_vendor_ops cpufreq_stats_config_ops = {
	.cpufreq_stats_info_get = scmi_get_cpufreq_stats_info,
	.set_log_level = scmi_set_log_level,
};
static int scmi_cpufreq_stats_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;
	return ph->xops->version_get(ph, &version);
}
static const struct scmi_protocol scmi_cpufreq_stats = {
	.id = SCMI_CPUFREQ_STATS_PROTOCOL, .owner = THIS_MODULE,
	.instance_init = scmi_cpufreq_stats_protocol_init,
	.ops = &cpufreq_stats_config_ops,
};
module_scmi_protocol(scmi_cpufreq_stats);
MODULE_DESCRIPTION("SCMI CPUFREQ_STATS vendor Protocol");
MODULE_LICENSE("GPL v2");
