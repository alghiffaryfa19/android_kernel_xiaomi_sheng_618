// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/unaligned.h>
#include <dt-bindings/soc/qcom,ipcc.h>

#include "qcom_ipc_lite.h"

#define IPCLITE_GLINK_GUID      "ipcliteglink-apps-dsp"
#define IPCC_CLIENT_CDSP        6

static struct ipc_lite_channel channel_info;
static struct ipc_lite_client ipc_client;
static DEFINE_MUTEX(channel_lock);
static DEFINE_MUTEX(client_lock);

int ipc_lite_register_client(IPCC_Client cb_func_ptr, void *priv)
{
	int err = 0;

	if (!cb_func_ptr)
		return -EINVAL;

	mutex_lock(&client_lock);
	ipc_client.callback = cb_func_ptr;
	ipc_client.priv_data = priv;
	mutex_unlock(&client_lock);
	pr_info("client callback registered with ipc_lite\n");
	return err;
}
EXPORT_SYMBOL(ipc_lite_register_client);

int ipc_lite_msg_send(int32_t client_id, uint64_t data, int allow_wait)
{
	int err = 0;

	mutex_lock(&channel_lock);
	VERIFY(err, !IS_ERR_OR_NULL(channel_info.rpdev));
	if (err) {
		err = -ENODEV;
		goto bail;
	}

	if (allow_wait)
		err = rpmsg_send(channel_info.rpdev->ept,
				(void *)&data, sizeof(data));
	else
		err = rpmsg_trysend(channel_info.rpdev->ept,
				(void *)&data, sizeof(data));
bail:
	mutex_unlock(&channel_lock);
	return err;
}
EXPORT_SYMBOL(ipc_lite_msg_send);

static int ipc_lite_rpmsg_callback(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 addr)
{
	int err = 0;

	if (!data || len != sizeof(uint64_t)) return -EINVAL;
	mutex_lock(&client_lock);
	if (ipc_client.callback)
		err = ipc_client.callback(IPCC_CLIENT_CDSP,
				get_unaligned((uint64_t *)data), ipc_client.priv_data);
	mutex_unlock(&client_lock);

	return err;
}

static int ipc_lite_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int err = 0;

	VERIFY(err, !IS_ERR_OR_NULL(rpdev));
	if (err)
		return -ENODEV;

	mutex_lock(&channel_lock);
	if (channel_info.rpdev) { mutex_unlock(&channel_lock); return -EBUSY; }
	channel_info.rpdev = rpdev;
	mutex_unlock(&channel_lock);
	dev_info(&rpdev->dev, "rpmsg_probe for ipc_lite done\n");

	return err;
}

static void ipc_lite_rpmsg_remove(struct rpmsg_device *rpdev)
{
	int err = 0;

	VERIFY(err, !IS_ERR_OR_NULL(rpdev));
	if (err) {
		err = -ENODEV;
		return;
	}
	mutex_lock(&channel_lock);
	if (channel_info.rpdev == rpdev) channel_info.rpdev = NULL;
	mutex_unlock(&channel_lock);
}

static const struct rpmsg_device_id ipc_lite_rpmsg_id_table[] = {
	{ IPCLITE_GLINK_GUID },
	{ },
};

MODULE_DEVICE_TABLE(rpmsg, ipc_lite_rpmsg_id_table);

static struct rpmsg_driver ipc_lite_rpmsg_client = {
	.id_table = ipc_lite_rpmsg_id_table,
	.probe = ipc_lite_rpmsg_probe,
	.remove = ipc_lite_rpmsg_remove,
	.callback = ipc_lite_rpmsg_callback,
	.drv = {
		.name = "qcom,msm_ipclite_rpmsg",
	},
};

module_rpmsg_driver(ipc_lite_rpmsg_client);

MODULE_LICENSE("GPL v2");
