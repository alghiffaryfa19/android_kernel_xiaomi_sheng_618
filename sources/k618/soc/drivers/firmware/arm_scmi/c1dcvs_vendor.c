// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/scmi_c1dcvs.h>
/*
 * k618: the SCMI core is the kernel's own (built in), so a protocol is built against the core's
 * own definitions -- drivers/firmware/arm_scmi/protocols.h, found through -I$(srctree) -- not
 * soc's 5.15-era common.h, whose struct scmi_protocol_handle has no hops (6.x put it before
 * set_priv/get_priv) and whose struct scmi_xfer predates 6.x's flags. These drivers only use
 * ph->dev, ph->xops and the xfer's hdr/tx/rx, which sit alike in both, but the core's header is
 * the one the core was compiled with.
 */
#include "drivers/firmware/arm_scmi/protocols.h"

/*
 * k618: 6.18's mailbox transport carries at most 104 bytes of payload
 * (SCMI_SHMEM_MAX_PAYLOAD_SIZE) unless the DT gives arm,max-msg-size, which this phone's
 * qcom,scmi node does not; 5.15's allowed 128. xfer_get_init() refuses a larger rx_size with
 * -ERANGE, so every read here failed. 0 asks for the transport's own maximum, and the core
 * then trims rx.len to what the firmware actually sent.
 */
#define SCMI_MAX_RX_SIZE	0

enum scmi_c1dcvs_protocol_cmd {
	SET_ENABLE_C1DCVS = 11,
	GET_ENABLE_C1DCVS,
	SET_ENABLE_TRACE,
	GET_ENABLE_TRACE,
	SET_IPC_THRESH,
	GET_IPC_THRESH,
	SET_EFREQ_THRESH,
	GET_EFREQ_THRESH,
	SET_HYSTERESIS,
	GET_HYSTERESIS,
};

struct c1dcvs_thresh {
	unsigned int cluster;
	unsigned int thresh;
};

static int scmi_send_tunable_c1dcvs(const struct scmi_protocol_handle *ph,
				    void *buf, u32 msg_id)
{
	int ret;
	struct scmi_xfer *t;
	unsigned int *msg;
	unsigned int *src = buf;

	ret = ph->xops->xfer_get_init(ph, msg_id, sizeof(*msg), sizeof(*msg),
				      &t);
	if (ret)
		return ret;

	msg = t->tx.buf;
	*msg = cpu_to_le32(*src);
	ret = ph->xops->do_xfer(ph, t);
	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_get_tunable_c1dcvs(const struct scmi_protocol_handle *ph,
				    void *buf, u32 msg_id)
{
	int ret;
	struct scmi_xfer *t;
	struct c1dcvs_thresh *msg;
	/* k618: what the caller's buffer holds -- one u32, or one per CPU for the thresholds */
	size_t max = (msg_id == GET_IPC_THRESH || msg_id == GET_EFREQ_THRESH) ?
		     num_possible_cpus() * sizeof(u32) : sizeof(u32);

	ret = ph->xops->xfer_get_init(ph, msg_id, sizeof(*msg), SCMI_MAX_RX_SIZE,
				      &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		memcpy(buf, t->rx.buf, min_t(size_t, t->rx.len, max));
	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_send_thresh_c1dcvs(const struct scmi_protocol_handle *ph,
				    void *buf, u32 msg_id)
{
	int ret;
	struct scmi_xfer *t;
	struct c1dcvs_thresh *msg;
	unsigned int *src = buf;

	ret = ph->xops->xfer_get_init(ph, msg_id, sizeof(*msg), sizeof(*msg),
				      &t);
	if (ret)
		return ret;

	msg = t->tx.buf;
	msg->cluster = cpu_to_le32(src[0]);
	msg->thresh = cpu_to_le32(src[1]);
	ret = ph->xops->do_xfer(ph, t);
	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_set_enable_c1dcvs(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_send_tunable_c1dcvs(ph, buf, SET_ENABLE_C1DCVS);
}
static int scmi_get_enable_c1dcvs(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_get_tunable_c1dcvs(ph, buf, GET_ENABLE_C1DCVS);
}
static int scmi_set_enable_trace(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_send_tunable_c1dcvs(ph, buf, SET_ENABLE_TRACE);
}
static int scmi_get_enable_trace(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_get_tunable_c1dcvs(ph, buf, GET_ENABLE_TRACE);
}
static int scmi_set_ipc_thresh(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_send_thresh_c1dcvs(ph, buf, SET_IPC_THRESH);
}
static int scmi_get_ipc_thresh(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_get_tunable_c1dcvs(ph, buf, GET_IPC_THRESH);
}
static int scmi_set_efreq_thresh(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_send_thresh_c1dcvs(ph, buf, SET_EFREQ_THRESH);
}
static int scmi_get_efreq_thresh(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_get_tunable_c1dcvs(ph, buf, GET_EFREQ_THRESH);
}
static int scmi_set_hysteresis(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_send_tunable_c1dcvs(ph, buf, SET_HYSTERESIS);
}
static int scmi_get_hysteresis(const struct scmi_protocol_handle *ph, void *buf)
{
	return scmi_get_tunable_c1dcvs(ph, buf, GET_HYSTERESIS);
}

static struct scmi_c1dcvs_vendor_ops c1dcvs_config_ops = {
	.set_enable_c1dcvs	= scmi_set_enable_c1dcvs,
	.get_enable_c1dcvs	= scmi_get_enable_c1dcvs,
	.set_enable_trace	= scmi_set_enable_trace,
	.get_enable_trace	= scmi_get_enable_trace,
	.set_ipc_thresh		= scmi_set_ipc_thresh,
	.get_ipc_thresh		= scmi_get_ipc_thresh,
	.set_efreq_thresh	= scmi_set_efreq_thresh,
	.get_efreq_thresh	= scmi_get_efreq_thresh,
	.set_hysteresis		= scmi_set_hysteresis,
	.get_hysteresis		= scmi_get_hysteresis,
};

static int scmi_c1dcvs_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;

	ph->xops->version_get(ph, &version);

	dev_dbg(ph->dev, "version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	return 0;
}

static const struct scmi_protocol scmi_c1dcvs = {
	.id = SCMI_C1DCVS_PROTOCOL,
	.vendor_id = "Qualcomm",
	.owner = THIS_MODULE,
	.instance_init = &scmi_c1dcvs_protocol_init,
	.ops = &c1dcvs_config_ops,
};
module_scmi_protocol(scmi_c1dcvs);

MODULE_DESCRIPTION("SCMI C1DCVS vendor Protocol");
MODULE_ALIAS("scmi-protocol-" __stringify(SCMI_C1DCVS_PROTOCOL) "-Qualcomm");
MODULE_LICENSE("GPL");
