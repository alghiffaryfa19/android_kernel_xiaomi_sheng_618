// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>	/* k618: ROHM OIS firmware */
#include "cam_sensor_cmn_header.h"
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_mem_mgr_api.h"
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include "oplus_cam_ois.h"
#endif

#define CAM_OIS_FW_VERSION_CHECK_MASK 0x1

/* k618 (camera 24): the 13 Ultra's OIS -- defined at the end of this file */
#include <linux/of.h>
#define K618_ULTRA_SLAVE_INFO_LEN	0x41	/* stock 0x162714: cmp x2, #0x40; b.ls -> -EINVAL */
#define K618_ULTRA_INIT_DONE		0x10000	/* k618_ultra_init(): the chip function ran */
#define K618_OIS_OPCODE_APERTURE	4	/* stock pkt opcode jump table .rodata 0x89588 */
#define K618_OIS_OPCODE_REDOWNLOAD	5
#define K618_OIS_OPCODE_PARKLENS	6
#define K618_OIS_OPCODE_SKIPFRAME	0x7e
#define K618_SEM1217S_CHIP_ID		0x4c1	/* stock 0x162d84 */
static int k618_ultra_slave_info(struct cam_ois_ctrl_t *o_ctrl, const uint8_t *info, size_t len);
static int k618_ultra_init(struct cam_ois_ctrl_t *o_ctrl);
static int k618_ultra_fw_download(struct cam_ois_ctrl_t *o_ctrl);
static int k618_ultra_flush_check(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet);
static int k618_ultra_pkt(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet);
static int k618_ultra_read_data(struct cam_ois_ctrl_t *o_ctrl, struct i2c_settings_array *set);
static void k618_ultra_start(struct cam_ois_ctrl_t *o_ctrl);
static void k618_ultra_stop(struct cam_ois_ctrl_t *o_ctrl);
static void k618_ultra_release(struct cam_ois_ctrl_t *o_ctrl);

/* k618 (camera 24, park lens): the 13 Ultra's OIS park-lens thread -- defined at the end of this file */
#include <linux/kthread.h>
static int k618_park_pkt(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet);
static void k618_park_on_init(struct cam_ois_ctrl_t *o_ctrl);
static bool k618_park_take_power(struct cam_ois_ctrl_t *o_ctrl);
static void k618_park_init_end(struct cam_ois_ctrl_t *o_ctrl);
static bool k618_park_release(struct cam_ois_ctrl_t *o_ctrl);
static bool k618_park_shutdown(struct cam_ois_ctrl_t *o_ctrl);

static inline uint64_t swap_high_byte_and_low_byte(uint8_t *src,
	uint8_t size_bytes)
{
	uint64_t ret_value = 0x00;
	uint8_t  cycle = 0;

	for (cycle = 0; cycle < size_bytes; cycle++)
		ret_value = ((ret_value<<8) | ((*(src+cycle))&0xff));

	return ret_value;
}

static inline uint64_t swap_high_word_and_low_word(uint16_t *src,
	uint8_t size_words)
{
	uint64_t ret_value = 0x00;
	uint8_t  cycle = 0;

	for (cycle = 0; cycle < size_words; cycle++)
		ret_value = ((ret_value<<16) | ((*(src+cycle))&0xffff));

	return ret_value;
}


int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		CAM_MEM_ZALLOC(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		CAM_MEM_ZALLOC(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	CAM_MEM_FREE(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;
	bridge_params.dev_id = CAM_OIS;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	if (ois_acq_dev.device_handle <= 0) {
		CAM_ERR(CAM_OIS, "Can not create device handle");
		return -EFAULT;
	}
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                                     rc = 0;
	struct cam_hw_soc_info                 *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private             *soc_private;
	struct cam_sensor_power_ctrl_t         *power_info;
	struct completion                      *i3c_probe_completion = NULL;

	soc_private = (struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	if (o_ctrl->io_master_info.master_type == I3C_MASTER)
		i3c_probe_completion = cam_ois_get_i3c_completion(o_ctrl->soc_info.index);

	rc = cam_sensor_core_power_up(power_info, soc_info, i3c_probe_completion);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_OIS, "Power Down failed");

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
#ifdef OPLUS_FEATURE_CAMERA_COMMON
int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
#else
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
#endif
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);
	o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;

	return rc;
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
EXPORT_SYMBOL(cam_ois_power_down);
#endif

static int cam_ois_update_time(struct i2c_settings_array *i2c_set,
	enum cam_endianness_type endianness)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t size = 0;
	uint32_t i = 0;
	uint64_t qtime_ns = 0;

	if (i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
	if (rc < 0) {
		CAM_ERR(CAM_OIS,
			"Failed to get current qtimer value: %d",
			rc);
		return rc;
	}

	if (endianness == CAM_ENDIANNESS_BIG)
		qtime_ns = swap_high_word_and_low_word((uint16_t *)(&qtime_ns),
					sizeof(qtime_ns) / sizeof(uint16_t));

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_SEQ) {
			size = i2c_list->i2c_settings.size;
			/* qtimer is 8 bytes so validate here*/
			if (size * (uint32_t)(i2c_list->i2c_settings.data_type) != 8) {
				CAM_ERR(CAM_OIS, "Invalid write time settings");
				return -EINVAL;
			}
			switch (i2c_list->i2c_settings.data_type) {
			case CAMERA_SENSOR_I2C_TYPE_BYTE:
				for (i = 0; i < size; i++) {
					CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
						i, (qtime_ns & 0xFF));
					i2c_list->i2c_settings.reg_setting[i].reg_data =
						(qtime_ns & 0xFF);
					qtime_ns >>= 8;
				}

				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				for (i = 0; i < size; i++) {
					uint16_t  data = (qtime_ns & 0xFFFF);
					i2c_list->i2c_settings.reg_setting[i].reg_data =
						data;

					qtime_ns >>= 16;

					CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
							i, data);
				}

				break;
			default:
				CAM_ERR(CAM_OIS, "Unsupported reg data type");
				return -EINVAL;
			}
		}
	}

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(
				&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				CAM_SENSOR_I2C_WRITE_SEQ);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed to seq write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				} else if (rc ==  I2C_COMPARE_MISMATCH) {
					/* Device compatibility. */
					if (o_ctrl->k618_poll_go_on > 0) {
						o_ctrl->k618_poll_go_on--;
						CAM_WARN(CAM_OIS,
							"k618: OIS %s: poll 0x%x == 0x%x timed out at bring-up, going on (%d more)",
							o_ctrl->ois_name,
							i2c_list->i2c_settings.reg_setting[i].reg_addr,
							i2c_list->i2c_settings.reg_setting[i].reg_data,
							o_ctrl->k618_poll_go_on);
						continue;
					}
					CAM_ERR(CAM_OIS, "i2c poll mismatch");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		/* k618: Xiaomi's three bytes after opcode (cam_ois_dev.h) */
		o_ctrl->k618_xiaomi_info = len >= sizeof(struct cam_cmd_ois_info) + 3;
		if (o_ctrl->k618_xiaomi_info) {
			const uint8_t *ext = (const uint8_t *)ois_info +
				sizeof(struct cam_cmd_ois_info);

			o_ctrl->k618_fw_addr_type = ext[0];
			o_ctrl->k618_fw_page_inc = ext[1];
			o_ctrl->k618_ois_flag = ext[2];
		} else {
			o_ctrl->k618_fw_addr_type = 0;
			o_ctrl->k618_fw_page_inc = 0;
			o_ctrl->k618_ois_flag = 0;
		}
		CAM_INFO(CAM_OIS,
			"k618: OIS %s slave 0x%x mode %u fw %u calib %u prog 0x%x coeff 0x%x mem 0x%x extra %u/%u/%u",
			o_ctrl->ois_name, ois_info->slave_addr, ois_info->i2c_freq_mode,
			o_ctrl->ois_fw_flag, o_ctrl->is_ois_calib, o_ctrl->opcode.prog,
			o_ctrl->opcode.coeff, o_ctrl->opcode.memory, o_ctrl->k618_fw_addr_type,
			o_ctrl->k618_fw_page_inc, o_ctrl->k618_ois_flag);
		/* k618: camera 24 -- the 13 Ultra's 65-byte slave info */
		rc = k618_ultra_slave_info(o_ctrl, (const uint8_t *)ois_info, len);
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		if (!o_ctrl->io_master_info.qup_client) {
			CAM_ERR(CAM_OIS, "io_master_info.qup_client is NULL");
			return -EINVAL;
		}
		o_ctrl->io_master_info.qup_client->i2c_client->addr =
			ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_parse_fw_setting(uint8_t *cmd_buf, uint32_t size,
	struct i2c_settings_array *reg_settings)
{
	int32_t                 rc = 0;
	uint32_t                byte_cnt = 0;
	struct common_header   *cmm_hdr;
	uint16_t                op_code;
	uint32_t                j = 0;
	struct list_head       *list = NULL;
	uint32_t                payload_count = 0;
	size_t                  tot_size = 0;

	while (byte_cnt < size) {
		if ((size - byte_cnt) < sizeof(struct common_header)) {
			CAM_ERR(CAM_OIS, "Not enough buffer");
			rc = -EINVAL;
			goto end;
		}
		cmm_hdr = (struct common_header *)cmd_buf;
		op_code = cmm_hdr->fifth_byte;
		CAM_DBG(CAM_OIS, "Command Type:%d, Op code:%d",
				 cmm_hdr->cmd_type, op_code);

		switch (cmm_hdr->cmd_type) {
		case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR: {
			uint32_t cmd_length_in_bytes = 0;
			struct cam_cmd_i2c_random_wr
			*cam_cmd_i2c_random_wr =
			(struct cam_cmd_i2c_random_wr *)cmd_buf;
			payload_count = cam_cmd_i2c_random_wr->header.count;

			if ((size - byte_cnt) < sizeof(struct cam_cmd_i2c_random_wr)) {
				CAM_ERR(CAM_OIS,
					"Not enough buffer provided,size %d,byte_cnt %d",
					size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}
			tot_size = sizeof(struct i2c_rdwr_header) +
			(sizeof(struct i2c_random_wr_payload) *
			payload_count);

			if (tot_size > (size - byte_cnt)) {
				CAM_ERR(CAM_SENSOR_UTIL,
				"Not enough buffer provided %d, %d, %d",
				tot_size, size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}

			rc = cam_sensor_handle_random_write(
				cam_cmd_i2c_random_wr,
				reg_settings,
				&cmd_length_in_bytes, &j, &list, payload_count);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
				"Failed in random write %d", rc);
				goto end;
			}

			byte_cnt += sizeof(struct cam_cmd_i2c_random_wr);
			cmd_buf += sizeof(struct cam_cmd_i2c_random_wr);

			break;
		}
		case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_WR: {
			uint32_t cmd_length_in_bytes = 0;
			struct cam_cmd_i2c_continuous_wr
			*cam_cmd_i2c_continuous_wr =
			(struct cam_cmd_i2c_continuous_wr *)cmd_buf;
			payload_count = cam_cmd_i2c_continuous_wr->header.count;
			if ((size - byte_cnt) < sizeof(struct cam_cmd_i2c_continuous_wr)) {
				CAM_ERR(CAM_OIS,
					"Not enough buffer provided,size %d,byte_cnt %d",
					size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}
			tot_size = sizeof(struct i2c_rdwr_header) +
			sizeof(cam_cmd_i2c_continuous_wr->reg_addr) +
			(sizeof(struct cam_cmd_read) *
			payload_count);

			if (tot_size > (size - byte_cnt)) {
				CAM_ERR(CAM_SENSOR_UTIL,
				"Not enough buffer provided %d, %d, %d",
				tot_size, size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}
			rc = cam_sensor_handle_continuous_write(
				cam_cmd_i2c_continuous_wr,
				reg_settings,
				&cmd_length_in_bytes, &j, &list, payload_count);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
				"Failed in continuous write %d", rc);
				goto end;
			}

			byte_cnt += sizeof(struct cam_cmd_i2c_continuous_wr);
			cmd_buf += sizeof(struct cam_cmd_i2c_continuous_wr);

			break;
		}
		case CAMERA_SENSOR_CMD_TYPE_WAIT: {
			if (op_code == CAMERA_SENSOR_WAIT_OP_HW_UCND ||
				op_code == CAMERA_SENSOR_WAIT_OP_SW_UCND) {
				if ((size - byte_cnt) <
					sizeof(struct cam_cmd_unconditional_wait)) {
					CAM_ERR(CAM_OIS,
						"Not enough buffer provided,size %d,byte_cnt %d",
						size, byte_cnt);
					rc = -EINVAL;
					goto end;
				}

				rc = cam_sensor_handle_delay(
					(uint32_t **)(&cmd_buf), op_code,
					reg_settings, j, &byte_cnt,
					list);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"delay hdl failed: %d",
						rc);
					goto end;
				}
			} else if (op_code == CAMERA_SENSOR_WAIT_OP_COND) {
				if ((size - byte_cnt) <
					sizeof(struct cam_cmd_conditional_wait)) {
					CAM_ERR(CAM_OIS,
						"Not enough buffer provided,size %d,byte_cnt %d",
						size, byte_cnt);
					rc = -EINVAL;
					goto end;
				}
				rc = cam_sensor_handle_poll(
					(uint32_t **)(&cmd_buf), reg_settings,
					&byte_cnt, &j, &list);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"parsing POLL fail: %d",
						rc);
					goto end;
				}
			} else {
				CAM_ERR(CAM_OIS,
					"Wrong Wait Command: %d",
					op_code);
				rc = -EINVAL;
				goto end;
			}
			break;
		}
		case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_RD: {
			uint16_t cmd_length_in_bytes = 0;
			struct cam_cmd_i2c_random_rd *i2c_random_rd =
			(struct cam_cmd_i2c_random_rd *)cmd_buf;
			payload_count = i2c_random_rd->header.count;
			if ((size - byte_cnt) < sizeof(struct cam_cmd_i2c_random_rd)) {
				CAM_ERR(CAM_OIS,
					"Not enough buffer provided,size %d,byte_cnt %d",
					size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}

			tot_size = sizeof(struct i2c_rdwr_header) +
			(sizeof(struct cam_cmd_read) *
			payload_count);
			if (tot_size > (size - byte_cnt)) {
				CAM_ERR(CAM_SENSOR_UTIL,
				"Not enough buffer provided %d, %d, %d",
				tot_size, size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}
			rc = cam_sensor_handle_random_read(
				i2c_random_rd,
				reg_settings,
				&cmd_length_in_bytes, &j, &list,
				NULL, payload_count);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
				"Failed in random read %d", rc);
				goto end;
			}

			byte_cnt += sizeof(struct cam_cmd_i2c_random_rd);
			cmd_buf += sizeof(struct cam_cmd_i2c_random_rd);

			break;
		}
		case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD: {
			uint16_t cmd_length_in_bytes = 0;
			struct cam_cmd_i2c_continuous_rd
			*i2c_continuous_rd =
			(struct cam_cmd_i2c_continuous_rd *)cmd_buf;

			if ((size - byte_cnt) < sizeof(struct cam_cmd_i2c_continuous_rd)) {
				CAM_ERR(CAM_OIS,
					"Not enough buffer provided,size %d,byte_cnt %d",
					size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}

			rc = cam_sensor_handle_continuous_read(
				i2c_continuous_rd,
				reg_settings,
				&cmd_length_in_bytes, &j, &list,
				NULL);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
				"Failed in continuous read %d", rc);
				goto end;
			}

			byte_cnt += sizeof(struct cam_cmd_i2c_continuous_rd);
			cmd_buf += sizeof(struct cam_cmd_i2c_continuous_rd);

			break;
		}
		default:
			CAM_ERR(CAM_OIS, "Invalid Command Type:%d",
				 cmm_hdr->cmd_type);
			rc = -EINVAL;
			goto end;
		}
	}

end:
	return rc;
}

static int cam_ois_fw_info_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t                         rc = 0;
	struct cam_cmd_ois_fw_info     *ois_fw_info;
	uint8_t                        *pSettingData = NULL;
	uint32_t                        size = 0;
	uint32_t                        version_size = 0;
	struct i2c_settings_array      *reg_settings = NULL;
	uint8_t                         count = 0;
	uint32_t                        idx;
	uint8_t                         ois_fw_count = 0;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_fw_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args,o_ctrl %p,cmd_buf %p,len %d",
			o_ctrl, cmd_buf, len);
		return -EINVAL;
	}

	ois_fw_info = (struct cam_cmd_ois_fw_info *)cmd_buf;
	ois_fw_count = ois_fw_info->fw_count;
	CAM_DBG(CAM_OIS, "endianness %d, fw_count %d",
		ois_fw_info->endianness, ois_fw_count);

	if (ois_fw_count <= MAX_OIS_FW_COUNT) {
		o_ctrl->fw_info.version     = ois_fw_info->version;
		o_ctrl->fw_info.cmd_type    = ois_fw_info->cmd_type;
		o_ctrl->fw_info.fw_count    = ois_fw_info->fw_count;
		o_ctrl->fw_info.endianness  = ois_fw_info->endianness;
		o_ctrl->fw_info.fw_param    = CAM_MEM_ZALLOC(sizeof(struct cam_cmd_ois_fw_param) * ois_fw_info->fw_count, GFP_KERNEL);
		if (!o_ctrl->fw_info.fw_param) {
			return -ENOMEM;
		}

		memcpy(o_ctrl->fw_info.fw_param, &ois_fw_info->fw_param[0], sizeof(struct cam_cmd_ois_fw_param) * ois_fw_info->fw_count);

		pSettingData = (uint8_t *)cmd_buf + sizeof(struct cam_cmd_ois_fw_info);

		if ((ois_fw_info->param_mask & CAM_OIS_FW_VERSION_CHECK_MASK) == 0x1) {
			version_size = ois_fw_info->params[0];
			CAM_DBG(CAM_OIS, "versionSize: %d", version_size);
		}

		if ((version_size != 0) && (o_ctrl->i2c_fw_version_data.is_settings_valid == 0)) {
			reg_settings = &o_ctrl->i2c_fw_version_data;
			reg_settings->is_settings_valid = 1;
			rc = cam_ois_parse_fw_setting(pSettingData, version_size, reg_settings);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed to parse fw version settings");
				goto release;
			}

			pSettingData += version_size;
		}

		o_ctrl->i2c_fw_init_data     = CAM_MEM_ZALLOC(sizeof(struct i2c_settings_array) * ois_fw_info->fw_count, GFP_KERNEL);
		o_ctrl->i2c_fw_finalize_data = CAM_MEM_ZALLOC(sizeof(struct i2c_settings_array) * ois_fw_info->fw_count, GFP_KERNEL);
		if (!o_ctrl->i2c_fw_init_data || !o_ctrl->i2c_fw_finalize_data){
			rc= -ENOMEM;
			goto release;
		}

		for (count = 0; count < o_ctrl->fw_info.fw_count; count++){
			INIT_LIST_HEAD(&(o_ctrl->i2c_fw_init_data[count].list_head));
			INIT_LIST_HEAD(&(o_ctrl->i2c_fw_finalize_data[count].list_head));
		}

		for (count = 0; count < o_ctrl->fw_info.fw_count*2; count++) {
			idx = count / 2;
			/* init settings */
			if ((count & 0x1) == 0) {
				size =  o_ctrl->fw_info.fw_param[idx].fw_init_size;
				reg_settings = &o_ctrl->i2c_fw_init_data[idx];
				CAM_DBG(CAM_OIS, "init size %d", size);
			/* finalize settings */
			} else if ((count & 0x1) == 1) {
				size = o_ctrl->fw_info.fw_param[idx].fw_finalize_size;
				reg_settings = &o_ctrl->i2c_fw_finalize_data[idx];
				CAM_DBG(CAM_OIS, "finalize size %d", size);
			} else {
				size = 0;
				CAM_DBG(CAM_OIS, "Unsupported case");
				rc= -EINVAL;
				goto release;
			}

			if (size != 0) {
				reg_settings->is_settings_valid = 1;
				rc = cam_ois_parse_fw_setting(pSettingData, size, reg_settings);
			}

			if (rc) {
				CAM_ERR(CAM_OIS, "Failed to parse fw setting");
				goto release;
			}

			pSettingData += size;
		}
	} else {
		CAM_ERR(CAM_OIS, "Exceed max fw count");
		return -EINVAL;
	}

	return rc;

release:
	if (o_ctrl->fw_info.fw_param){
		CAM_MEM_FREE(o_ctrl->fw_info.fw_param);
		o_ctrl->fw_info.fw_param = NULL;
	}
	if (o_ctrl->i2c_fw_init_data){
		CAM_MEM_FREE(o_ctrl->i2c_fw_init_data);
		o_ctrl->i2c_fw_init_data = NULL;
	}
	if (o_ctrl->i2c_fw_finalize_data){
		CAM_MEM_FREE(o_ctrl->i2c_fw_finalize_data);
		o_ctrl->i2c_fw_finalize_data = NULL;
	}
	return rc;
}


int cam_ois_fw_info_input_check(uint32_t *cmd_buf, size_t len)
{
	int32_t     rc           = 0;
	uint32_t    version_size = 0;
	size_t      cmdsize      = 0;
	uint8_t     count        = 0;
	struct cam_cmd_ois_fw_info_v2  *ois_fw_info;

	ois_fw_info  = (struct cam_cmd_ois_fw_info_v2 *)cmd_buf;
	if ((ois_fw_info->param_mask & CAM_OIS_FW_VERSION_CHECK_MASK) == 0x1) {
		version_size = ois_fw_info->params[0];
		CAM_DBG(CAM_OIS, "versionSize: %d", version_size);
	}

	cmdsize += sizeof(struct cam_cmd_ois_fw_info_v2);
	cmdsize += sizeof(struct cam_cmd_ois_fw_param) * ois_fw_info->fw_count;
	cmdsize += version_size;
	for (count = 0; count < ois_fw_info->fw_count; count++){
		if (ois_fw_info->fw_param[count].fw_init_size > 0){
			cmdsize += ois_fw_info->fw_param[count].fw_init_size;
			CAM_DBG(CAM_OIS, "count:%d, fw init size:%d", count, ois_fw_info->fw_param[count].fw_init_size);
		}
		if (ois_fw_info->fw_param[count].fw_finalize_size > 0){
			cmdsize += ois_fw_info->fw_param[count].fw_finalize_size;
			CAM_DBG(CAM_OIS, "count:%d, fw finalize size:%d", count, ois_fw_info->fw_param[count].fw_finalize_size);
		}
	}
	if (cmdsize != len){
		CAM_ERR(CAM_OIS, "cmd size validate error! cmdsize:%d, expect:%d", cmdsize, len);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_info_pkt_parser_v2(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t                         rc = 0;
	struct cam_cmd_ois_fw_info_v2  *ois_fw_info;
	uint8_t                        *pSettingData = NULL;
	uint32_t                        size = 0;
	uint32_t                        version_size = 0;
	struct i2c_settings_array      *reg_settings = NULL;
	uint8_t                         count = 0;
	uint8_t                        *ois_fw_param = NULL;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_fw_info_v2)) {
		CAM_ERR(CAM_OIS, "Invalid Args,o_ctrl %p,cmd_buf %p,len %d",
			o_ctrl, cmd_buf, len);
		return -EINVAL;
	}

	if(cam_ois_fw_info_input_check(cmd_buf, len) < 0) {
		return -EINVAL;
	}

	ois_fw_info  = (struct cam_cmd_ois_fw_info_v2 *)cmd_buf;
	ois_fw_param = (uint8_t*)ois_fw_info->fw_param;

	CAM_DBG(CAM_OIS, "endianness %d, fw_count %d",
		ois_fw_info->endianness, ois_fw_info->fw_count);

	if (ois_fw_info->fw_count > 0) {
		o_ctrl->fw_info.version     = ois_fw_info->version;
		o_ctrl->fw_info.cmd_type    = ois_fw_info->cmd_type;
		o_ctrl->fw_info.fw_count    = ois_fw_info->fw_count;
		o_ctrl->fw_info.endianness  = ois_fw_info->endianness;
		o_ctrl->fw_info.fw_param    = CAM_MEM_ZALLOC(sizeof(struct cam_cmd_ois_fw_param) * ois_fw_info->fw_count, GFP_KERNEL);
		if (!o_ctrl->fw_info.fw_param) {
			return -ENOMEM;
		}
		memcpy(o_ctrl->fw_info.fw_param, ois_fw_param, sizeof(struct cam_cmd_ois_fw_param) * ois_fw_info->fw_count);

		pSettingData = (uint8_t *)cmd_buf + sizeof(struct cam_cmd_ois_fw_info_v2) + sizeof(struct cam_cmd_ois_fw_param)*ois_fw_info->fw_count;

		if ((ois_fw_info->param_mask & CAM_OIS_FW_VERSION_CHECK_MASK) == 0x1) {
			version_size = ois_fw_info->params[0];
			CAM_DBG(CAM_OIS, "versionSize: %d", version_size);
		}
		if ((version_size != 0) && (o_ctrl->i2c_fw_version_data.is_settings_valid == 0)) {
			reg_settings = &o_ctrl->i2c_fw_version_data;
			reg_settings->is_settings_valid = 1;
			rc = cam_ois_parse_fw_setting(pSettingData, version_size, reg_settings);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed to parse fw version settings");
				goto release;
			}
			pSettingData += version_size;
		}

		o_ctrl->i2c_fw_init_data     = CAM_MEM_ZALLOC(sizeof(struct i2c_settings_array) * ois_fw_info->fw_count, GFP_KERNEL);
		o_ctrl->i2c_fw_finalize_data = CAM_MEM_ZALLOC(sizeof(struct i2c_settings_array) * ois_fw_info->fw_count, GFP_KERNEL);
		if (!o_ctrl->i2c_fw_init_data || !o_ctrl->i2c_fw_finalize_data) {
			goto release;
		}

		for (count = 0; count < ois_fw_info->fw_count; count++) {
			INIT_LIST_HEAD(&(o_ctrl->i2c_fw_init_data[count].list_head));
			INIT_LIST_HEAD(&(o_ctrl->i2c_fw_finalize_data[count].list_head));

			if (o_ctrl->fw_info.fw_param[count].fw_init_size > 0)
			{
				reg_settings = &o_ctrl->i2c_fw_init_data[count];
				reg_settings->is_settings_valid = 1;
				size = o_ctrl->fw_info.fw_param[count].fw_init_size;
				rc = cam_ois_parse_fw_setting(pSettingData, size, reg_settings);
				if (rc) {
					CAM_ERR(CAM_OIS, "Failed to parse fw setting");
					goto release;
				}
				pSettingData += size;
				CAM_DBG(CAM_OIS, "count:%d, init size %d", count, size);
			}

			if (o_ctrl->fw_info.fw_param[count].fw_finalize_size > 0)
			{
				reg_settings = &o_ctrl->i2c_fw_finalize_data[count];
				reg_settings->is_settings_valid = 1;
				size = o_ctrl->fw_info.fw_param[count].fw_finalize_size;
				rc = cam_ois_parse_fw_setting(pSettingData, size, reg_settings);
				if (rc) {
					CAM_ERR(CAM_OIS, "Failed to parse fw setting");
					goto release;
				}
				pSettingData += size;
				CAM_DBG(CAM_OIS, "count:%d, finalize size %d", count, size);
			}
		}
	}

	return rc;

release:
	if (o_ctrl->fw_info.fw_param){
		CAM_MEM_FREE(o_ctrl->fw_info.fw_param);
		o_ctrl->fw_info.fw_param = NULL;
	}
	if (o_ctrl->i2c_fw_init_data){
		CAM_MEM_FREE(o_ctrl->i2c_fw_init_data);
		o_ctrl->i2c_fw_init_data = NULL;
	}
	if (o_ctrl->i2c_fw_finalize_data){
		CAM_MEM_FREE(o_ctrl->i2c_fw_finalize_data);
		o_ctrl->i2c_fw_finalize_data = NULL;
	}
	return rc;
}

/* Device compatibility. */
static int k618_xiaomi_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	static const char * const sfx[] = { "prog", "coeff", "mem" };
	uint32_t base[] = { o_ctrl->opcode.prog, o_ctrl->opcode.coeff, o_ctrl->opcode.memory };
	struct cam_sensor_i2c_reg_setting setting = {0};
	struct cam_sensor_i2c_reg_array *regs;
	const struct firmware *fw = NULL;
	char name[MAX_OIS_NAME_SIZE + 8];
	uint32_t total, done, page, n, i;
	int f, rc = 0;

	setting.addr_type = o_ctrl->k618_fw_addr_type;
	if (setting.addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID ||
		setting.addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
		CAM_ERR(CAM_OIS, "k618: OIS %s: firmware address type %u",
			o_ctrl->ois_name, o_ctrl->k618_fw_addr_type);
		return -EINVAL;
	}
	setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	setting.delay = 0;
	regs = vzalloc(256 * sizeof(*regs));
	if (!regs)
		return -ENOMEM;
	setting.reg_setting = regs;

	for (f = 0; f < ARRAY_SIZE(sfx); f++) {
		snprintf(name, sizeof(name), "%s.%s", o_ctrl->ois_name, sfx[f]);
		rc = request_firmware(&fw, name, &(o_ctrl->pdev->dev));
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to locate %s", name);
			break;
		}
		total = fw->size & 0xffff;	/* 5.15 kept the size in 16 bits */
		for (done = 0, page = 0; done < total; done += n) {
			n = min_t(uint32_t, 256, total - done);
			for (i = 0; i < n; i++) {
				regs[i].reg_addr = base[f] + (page << 8);
				regs[i].reg_data = fw->data[done + i];
				regs[i].delay = 0;
				regs[i].data_mask = 0;
			}
			setting.size = n;
			if (o_ctrl->k618_fw_page_inc)
				page++;
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&setting, CAM_SENSOR_I2C_WRITE_BURST);
			if (rc < 0)
				break;
		}
		release_firmware(fw);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS FW(%s) size(%u) download failed rc: %d",
				sfx[f], total, rc);
			break;
		}
		CAM_INFO(CAM_OIS, "k618: OIS %s: %s, %u bytes from 0x%x, address type %u, page step %u",
			o_ctrl->ois_name, name, total, base[f], o_ctrl->k618_fw_addr_type,
			o_ctrl->k618_fw_page_inc);
	}
	vfree(regs);
	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void                              *vaddr = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	dev = &(o_ctrl->pdev->dev);

	/* k618: camera 16 -- Xiaomi's slave info, Xiaomi's download */
	if (o_ctrl->k618_xiaomi_info)
		return k618_xiaomi_fw_download(o_ctrl);

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW prog size:%d.", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW(prog) size(%d) download failed. %d",
			total_bytes, rc);
		goto release_firmware;
	}
	vfree(vaddr);
	vaddr = NULL;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW coeff size:%d", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);

	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW(coeff) size(%d) download failed rc: %d",
			total_bytes, rc);

release_firmware:
	vfree(vaddr);
	vaddr = NULL;
	release_firmware(fw);
	return rc;
}

static int write_ois_fw(uint8_t *fw_data, enum cam_endianness_type endianness,
	struct cam_cmd_ois_fw_param *fw_param, struct camera_io_master io_master_info,
	uint8_t i2c_operation)
{
	int32_t                             rc = 0;
	struct cam_sensor_i2c_reg_setting   setting;
	uint8_t                            *ptr = fw_data;
	int32_t                             cnt = 0, wcnt = 0;
	void                               *vaddr = NULL;
	uint16_t                            data_type = fw_param->fw_data_type;
	uint16_t                            len_per_write = fw_param->fw_len_per_write /
								fw_param->fw_data_type;

	vaddr = vmalloc((sizeof(struct cam_sensor_i2c_reg_array) * len_per_write));
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: size: %u",
			(sizeof(struct cam_sensor_i2c_reg_array) * len_per_write));
		return -ENOMEM;
	}

	setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (vaddr);
	setting.addr_type   = fw_param->fw_addr_type;
	setting.data_type   = fw_param->fw_data_type;
	setting.size        = len_per_write;
	setting.delay       = fw_param->fw_delayUs;

	for (wcnt = 0; wcnt < (fw_param->fw_size/data_type); wcnt += len_per_write) {
		for (cnt = 0; cnt < len_per_write; cnt++, ptr += data_type) {
			setting.reg_setting[cnt].reg_addr =
				fw_param->fw_reg_addr + wcnt + cnt;
			/* Big */
			if (endianness == CAM_ENDIANNESS_BIG) {
				setting.reg_setting[cnt].reg_data =
					(uint32_t)swap_high_byte_and_low_byte(ptr, data_type);
			/* Little */
			} else if (endianness == CAM_ENDIANNESS_LITTLE) {
				switch (data_type) {
				case CAMERA_SENSOR_I2C_TYPE_BYTE:
					setting.reg_setting[cnt].reg_data = *((uint8_t *)ptr);
					break;
				case CAMERA_SENSOR_I2C_TYPE_WORD:
					setting.reg_setting[cnt].reg_data = *((uint16_t *)ptr);
					break;
				default:
					CAM_ERR(CAM_OIS,
						"Unsupported data type");
					rc = -EINVAL;
					goto End;
				}
			}

			setting.reg_setting[cnt].delay = fw_param->fw_delayUs;
			setting.reg_setting[cnt].data_mask = 0;
		}

		if (i2c_operation == CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(io_master_info),
				&setting);
		} else if (i2c_operation == CAM_SENSOR_I2C_WRITE_BURST ||
			i2c_operation == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(&io_master_info,
				&setting, i2c_operation);
		}

		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in Applying i2c wrt settings");
			break;
		}
	}

End:
	vfree(vaddr);
	vaddr = NULL;

	return rc;
}

static int cam_ois_fw_download_v2(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                             rc = 0;
	struct cam_cmd_ois_fw_param        *fw_param = NULL;
	uint32_t                            fw_size;
	uint16_t                            len_per_write = 0;
	uint8_t                            *ptr = NULL;
	const struct firmware              *fw = NULL;
	struct device                      *dev;
	uint8_t                             count = 0;
	uint8_t                             cont_wr_flag = 0;
	uint8_t                             fwcnt = 0;
	uint8_t                             endianness = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (o_ctrl->i2c_fw_version_data.is_settings_valid == 1) {
		CAM_DBG(CAM_OIS, "check version to decide FW download");
		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_fw_version_data);
		if ((rc == -EAGAIN) &&
			(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
			CAM_WARN(CAM_OIS,
			"CCI HW is resetting: Reapplying FW init settings");
			usleep_range(1000, 1010);
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_fw_version_data);
		}

		if (delete_request(&o_ctrl->i2c_fw_version_data) < 0)
			CAM_WARN(CAM_OIS,
				"Fail deleting i2c_fw_version_data: rc: %d", rc);

		if (rc == I2C_COMPARE_MATCH) {
			CAM_INFO(CAM_OIS,
				"OIS FW version matched, skipping FW download");
			return rc;
		} else if (rc == I2C_COMPARE_MISMATCH) {
			CAM_INFO(CAM_OIS, "OIS FW version not matched, load FW");
		} else {
			CAM_WARN(CAM_OIS, "OIS FW version check failed,rc=%d", rc);
		}
	}

	if (o_ctrl->fw_info.fw_count > 0) {
		fwcnt      = o_ctrl->fw_info.fw_count;
		fw_param   = &o_ctrl->fw_info.fw_param[0];
		endianness = o_ctrl->fw_info.endianness;
	}

	for (count = 0; count < fwcnt; count++, fw_param++) {
		fw_size       = fw_param->fw_size;
		len_per_write = fw_param->fw_len_per_write / fw_param->fw_data_type;

		CAM_DBG(CAM_OIS, "count: %d, fw_size: %d, data_type: %d, len_per_write: %d",
			count, fw_size, fw_param->fw_data_type, len_per_write);

		/* Load FW */
		dev = &(o_ctrl->pdev->dev);
		rc = request_firmware(&fw, fw_param->fw_name, dev);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to locate %s", fw_param->fw_name);
			return rc;
		}

		if (0 == rc && NULL != fw &&
			(fw_size <= fw->size - fw_param->fw_start_pos)) {

			/* fw init */
			CAM_DBG(CAM_OIS, "fw init");
			if (o_ctrl->i2c_fw_init_data[count].is_settings_valid == 1) {
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_fw_init_data[count]);
				if ((rc == -EAGAIN) &&
					(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_OIS,
					"CCI HW is resetting: Reapplying FW init settings");
					usleep_range(1000, 1010);
					rc = cam_ois_apply_settings(o_ctrl,
						&o_ctrl->i2c_fw_init_data[count]);
				}
				if (rc) {
					CAM_ERR(CAM_OIS,
						"Cannot apply FW init settings %d",
						rc);
					goto release_firmware;
				} else {
					CAM_DBG(CAM_OIS, "OIS FW init settings success");
				}
			}

			/* send fw */
			CAM_DBG(CAM_OIS, "send fw, operation %d", fw_param->fw_operation);

			ptr = (uint8_t *)(fw->data + fw_param->fw_start_pos);
			if (fw_param->fw_operation == CAMERA_SENSOR_I2C_OP_RNDM_WR)
				cont_wr_flag = CAM_SENSOR_I2C_WRITE_RANDOM;
			else if (fw_param->fw_operation == CAMERA_SENSOR_I2C_OP_CONT_WR_BRST)
				cont_wr_flag = CAM_SENSOR_I2C_WRITE_BURST;
			else if (fw_param->fw_operation == CAMERA_SENSOR_I2C_OP_CONT_WR_SEQN)
				cont_wr_flag = CAM_SENSOR_I2C_WRITE_SEQ;

			write_ois_fw(ptr, (endianness & OIS_ENDIANNESS_MASK_FW),
					fw_param, o_ctrl->io_master_info, cont_wr_flag);

			/* fw finalize */
			CAM_DBG(CAM_OIS, "fw finalize");
			if (o_ctrl->i2c_fw_finalize_data[count].is_settings_valid == 1) {
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_fw_finalize_data[count]);
				if ((rc == -EAGAIN) &&
					(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_OIS,
					"CCI HW is resetting: Reapplying FW finalize settings");
					usleep_range(1000, 1010);
					rc = cam_ois_apply_settings(o_ctrl,
						&o_ctrl->i2c_fw_finalize_data[count]);
				}
				if (rc) {
					CAM_ERR(CAM_OIS,
						"Cannot apply FW finalize settings %d",
						rc);
					goto release_firmware;
				} else {
					CAM_DBG(CAM_OIS, "OIS FW finalize settings success");
				}
			}
		}

		if (fw != NULL) {
			release_firmware(fw);
			fw = NULL;
		}
	}

release_firmware:
	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}

	return rc;
}

/*
 * k618 (camera 16): Xiaomi's ROHM OIS path. The phone's own 5.15 camera.ko has it as bu24721_ois_*
 * (pkt_download, update_i2c_info, apply_settings, apply_calib_settings) and runs it instead of the
 * generic init when the third extra byte of the slave info (k618_ois_flag) is 1..4; both OIS of this
 * phone (CamX nuwa_wide/tele_bu24618_ois, firmware /odm/firmware/<name>.prog/.coeff/.mem) are
 * driven so. 6.12's generic path instead burst-writes .prog/.coeff byte by byte to the run-time
 * slave, never loads .mem and never restarts the chip: its status register then reads 0xff.
 * Reconstructed from the stock disassembly:
 *  1. F050 = 0 (out of standby), wait for status F024 == 1, read the 32-bit program id F01C;
 *  2. if it is not the id the flag expects: CCI client -> download slave 0xA0 at 1 MHz, apply the
 *     packet's init settings there (a failure is logged, not fatal), write .prog/.coeff/.mem as
 *     big-endian dwords, one register per dword from opcode.prog/.coeff/.memory on (address type
 *     k618_fw_addr_type), client -> run-time slave 0x7C at 1 MHz, F097 = 0 (+1 ms), F058 = 0
 *     (+8 ms), wait, F050 = 0, wait, read F01C again;
 *  3. when is_ois_calib: gyro offsets = the values the calib settings write to F09D, sent as
 *     F09C = 0 / F09D = x and F09C = 1 / F09D = y, each followed by the status wait;
 *  4. the packet's fwinit settings -- the only step whose failure fails the init.
 * Writes, reads and status waits of 1-3 are not checked (5.15 did not check them either).
 */
#define K618_ROHM_STANDBY	0xF050
#define K618_ROHM_STATUS	0xF024
#define K618_ROHM_PROG_ID	0xF01C
#define K618_ROHM_GYRO_GAIN_X	0xF07A
#define K618_ROHM_GYRO_GAIN_Y	0xF07C
#define K618_ROHM_RESTART_1	0xF097
#define K618_ROHM_RESTART_2	0xF058
#define K618_ROHM_GOFS_SEL	0xF09C
#define K618_ROHM_GOFS_VAL	0xF09D
#define K618_ROHM_DL_SLAVE	0xA0	/* 8-bit addresses, as the slave info carries them */
#define K618_ROHM_RUN_SLAVE	0x7C
#define K618_ROHM_WAIT_MS	100

/* the program id each flag expects (the stock module's table, indexed by flag - 1) */
static const uint32_t k618_rohm_prog_id[] = {
	0x1a8f025b, 0x1a930253, 0x1a8f024b, 0x1a940043,
};

static void k618_rohm_set_slave(struct cam_ois_ctrl_t *o_ctrl, uint16_t addr)
{
	struct cam_sensor_cci_client *cci = o_ctrl->io_master_info.cci_client;

	cci->sid = addr >> 1;
	cci->retries = 3;
	cci->id_map = 0;
	cci->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	CAM_DBG(CAM_OIS, "k618: ROHM OIS slave 0x%x", addr);
}

static int k618_rohm_write(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr,
	uint32_t data, enum camera_sensor_i2c_type data_type, unsigned short delay_ms)
{
	struct cam_sensor_i2c_reg_array reg = {
		.reg_addr = addr,
		.reg_data = data,
	};
	struct cam_sensor_i2c_reg_setting setting = {
		.reg_setting = &reg,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = data_type,
		.delay = delay_ms,
	};
	int rc = camera_io_dev_write(&(o_ctrl->io_master_info), &setting);

	if (rc < 0)
		CAM_ERR(CAM_OIS, "k618: ROHM OIS write 0x%x = 0x%x failed: %d",
			addr, data, rc);
	return rc;
}

static void k618_rohm_wait_ready(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = camera_io_dev_poll(&(o_ctrl->io_master_info), K618_ROHM_STATUS,
		0x01, 0, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE,
		K618_ROHM_WAIT_MS);

	if (rc)
		CAM_WARN(CAM_OIS, "k618: ROHM OIS not ready (status wait %d)", rc);
}

static uint32_t k618_rohm_read(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr,
	enum camera_sensor_i2c_type data_type, bool quiet)
{
	uint32_t val = 0;
	int rc = camera_io_dev_read(&(o_ctrl->io_master_info), addr, &val,
		CAMERA_SENSOR_I2C_TYPE_WORD, data_type, quiet);

	if (rc < 0)
		CAM_WARN(CAM_OIS, "k618: ROHM OIS read 0x%x failed: %d", addr, rc);
	return val;
}

/* stock bu24721_ois_apply_settings: no burst writes, a poll that times out goes on */
static int k618_rohm_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i;

	list_for_each_entry(i2c_list, &(i2c_set->list_head), list) {
		switch (i2c_list->op_code) {
		case CAM_SENSOR_I2C_WRITE_RANDOM:
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "k618: ROHM OIS settings write failed: %d", rc);
				return rc;
			}
			break;
		case CAM_SENSOR_I2C_WRITE_SEQ:
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings), CAM_SENSOR_I2C_WRITE_SEQ);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "k618: ROHM OIS settings seq write failed: %d", rc);
				return rc;
			}
			break;
		case CAM_SENSOR_I2C_POLL:
			for (i = 0; i < i2c_list->i2c_settings.size; i++) {
				rc = camera_io_dev_poll(&(o_ctrl->io_master_info),
					i2c_list->i2c_settings.reg_setting[i].reg_addr,
					i2c_list->i2c_settings.reg_setting[i].reg_data,
					i2c_list->i2c_settings.reg_setting[i].data_mask,
					i2c_list->i2c_settings.addr_type,
					i2c_list->i2c_settings.data_type,
					i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS, "k618: ROHM OIS settings poll failed: %d", rc);
					return rc;
				}
			}
			break;
		default:
			break;
		}
	}
	return rc;	/* as 5.15: the last operation's result, 1 if a last poll timed out */
}

static int k618_rohm_apply_retry(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	int rc = k618_rohm_apply_settings(o_ctrl, i2c_set);

	if (rc == -EAGAIN) {
		CAM_WARN(CAM_OIS, "k618: ROHM OIS: CCI resetting, applying again");
		usleep_range(1000, 1010);
		rc = k618_rohm_apply_settings(o_ctrl, i2c_set);
	}
	return rc;
}

/* <ois_name>.<suffix>: one register per big-endian dword from base on; *write_rc = the CCI result */
static int k618_rohm_load_fw(struct cam_ois_ctrl_t *o_ctrl, const char *suffix,
	uint32_t base, int *write_rc)
{
	struct cam_sensor_i2c_reg_setting setting = {0};
	struct cam_sensor_i2c_reg_array *regs;
	const struct firmware *fw = NULL;
	char name[MAX_OIS_NAME_SIZE + 8];
	uint32_t total, count, i;
	int rc;

	snprintf(name, sizeof(name), "%s.%s", o_ctrl->ois_name, suffix);
	rc = request_firmware(&fw, name, &(o_ctrl->pdev->dev));
	if (rc) {
		CAM_ERR(CAM_OIS, "k618: ROHM OIS: failed to locate %s: %d", name, rc);
		return rc;
	}
	if (fw->size > 0xffff)
		CAM_WARN(CAM_OIS, "k618: ROHM OIS: %s is %zu bytes, sending the first %zu as 5.15",
			name, fw->size, fw->size & 0xffff);
	total = fw->size & 0xffff;	/* 5.15 kept the size in 16 bits */
	count = DIV_ROUND_UP(total, 4);
	regs = vzalloc((count ? count : 1) * sizeof(*regs));
	if (!regs) {
		release_firmware(fw);
		return -ENOMEM;
	}
	for (i = 0; i < count; i++) {
		uint8_t b[4] = {0};

		memcpy(b, fw->data + 4 * i, min_t(uint32_t, 4, total - 4 * i));
		regs[i].reg_addr = base + i;
		regs[i].reg_data = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			((uint32_t)b[2] << 8) | b[3];
	}
	setting.reg_setting = regs;
	setting.size = count;
	setting.addr_type = o_ctrl->k618_fw_addr_type;
	setting.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	if (setting.addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID ||
		setting.addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
		CAM_WARN(CAM_OIS, "k618: ROHM OIS: firmware address type %u, using word",
			o_ctrl->k618_fw_addr_type);
		setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	}
	*write_rc = count ? camera_io_dev_write(&(o_ctrl->io_master_info), &setting) : 0;
	if (*write_rc < 0)
		CAM_ERR(CAM_OIS, "k618: ROHM OIS: %s (%u bytes) download failed: %d",
			name, total, *write_rc);
	else
		CAM_DBG(CAM_OIS, "k618: ROHM OIS: %s, %u dwords from 0x%x", name, count, base);
	vfree(regs);
	release_firmware(fw);
	return 0;
}

/* stock bu24721_ois_apply_calib_settings */
static void k618_rohm_apply_calib(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	uint32_t ofs[4] = {0}, n = 0, i;

	list_for_each_entry(i2c_list, &(i2c_set->list_head), list) {
		if (i2c_list->op_code != CAM_SENSOR_I2C_WRITE_RANDOM)
			continue;
		for (i = 0; i < i2c_list->i2c_settings.size && n < ARRAY_SIZE(ofs); i++)
			if (i2c_list->i2c_settings.reg_setting[i].reg_addr == K618_ROHM_GOFS_VAL)
				ofs[n++] = i2c_list->i2c_settings.reg_setting[i].reg_data & 0xffff;
	}
	k618_rohm_write(o_ctrl, K618_ROHM_GOFS_SEL, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_rohm_wait_ready(o_ctrl);
	k618_rohm_write(o_ctrl, K618_ROHM_GOFS_VAL, ofs[0], CAMERA_SENSOR_I2C_TYPE_WORD, 0);
	k618_rohm_wait_ready(o_ctrl);
	k618_rohm_write(o_ctrl, K618_ROHM_GOFS_SEL, 1, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_rohm_wait_ready(o_ctrl);
	k618_rohm_write(o_ctrl, K618_ROHM_GOFS_VAL, ofs[1], CAMERA_SENSOR_I2C_TYPE_WORD, 0);
	k618_rohm_wait_ready(o_ctrl);
	CAM_DBG(CAM_OIS, "k618: ROHM OIS gyro offsets x 0x%x y 0x%x (%u in calib)",
		ofs[0], ofs[1], n);
}

/* stock bu24721_ois_pkt_download; the caller has powered the OIS up */
static int k618_rohm_ois_init(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t want = k618_rohm_prog_id[o_ctrl->k618_ois_flag - 1];
	uint32_t id, gyro_x, gyro_y;
	int rc = 0, write_rc = 0;

	if (o_ctrl->io_master_info.master_type != CCI_MASTER ||
		!o_ctrl->io_master_info.cci_client) {
		CAM_ERR(CAM_OIS, "k618: ROHM OIS needs a CCI master (type %d)",
			o_ctrl->io_master_info.master_type);
		return -EINVAL;
	}

	k618_rohm_write(o_ctrl, K618_ROHM_STANDBY, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
	k618_rohm_wait_ready(o_ctrl);
	id = k618_rohm_read(o_ctrl, K618_ROHM_PROG_ID, CAMERA_SENSOR_I2C_TYPE_DWORD, true);
	k618_rohm_wait_ready(o_ctrl);
	CAM_INFO(CAM_OIS, "k618: ROHM OIS %s flag %u: program id 0x%08x, wants 0x%08x",
		o_ctrl->ois_name, o_ctrl->k618_ois_flag, id, want);

	if (id != want) {
		k618_rohm_set_slave(o_ctrl, K618_ROHM_DL_SLAVE);
		if (o_ctrl->i2c_init_data.is_settings_valid == 1) {
			rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_init_data);
			if (rc)	/* 5.15 logged this and downloaded all the same */
				CAM_ERR(CAM_OIS, "k618: ROHM OIS: download-mode settings: %d", rc);
		}
		rc = k618_rohm_load_fw(o_ctrl, "prog", o_ctrl->opcode.prog, &write_rc);
		if (!rc && write_rc >= 0)
			rc = k618_rohm_load_fw(o_ctrl, "coeff", o_ctrl->opcode.coeff, &write_rc);
		if (!rc && write_rc >= 0)
			rc = k618_rohm_load_fw(o_ctrl, "mem", o_ctrl->opcode.memory, &write_rc);
		k618_rohm_set_slave(o_ctrl, K618_ROHM_RUN_SLAVE);
		if (rc)		/* no firmware file / no memory: 5.15 gave up here as well */
			return rc;

		k618_rohm_write(o_ctrl, K618_ROHM_RESTART_1, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
		k618_rohm_write(o_ctrl, K618_ROHM_RESTART_2, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 8);
		k618_rohm_wait_ready(o_ctrl);
		k618_rohm_write(o_ctrl, K618_ROHM_STANDBY, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
		k618_rohm_wait_ready(o_ctrl);
		id = k618_rohm_read(o_ctrl, K618_ROHM_PROG_ID, CAMERA_SENSOR_I2C_TYPE_DWORD, true);
		k618_rohm_wait_ready(o_ctrl);
		if (id == want && write_rc >= 0)
			CAM_INFO(CAM_OIS, "k618: ROHM OIS %s: firmware loaded, program id 0x%08x",
				o_ctrl->ois_name, id);
		else
			CAM_ERR(CAM_OIS,
				"k618: ROHM OIS %s: program id 0x%08x after the download, wants 0x%08x (write %d)",
				o_ctrl->ois_name, id, want, write_rc);
	}

	if (o_ctrl->is_ois_calib)
		k618_rohm_apply_calib(o_ctrl, &o_ctrl->i2c_calib_data);

	gyro_x = k618_rohm_read(o_ctrl, K618_ROHM_GYRO_GAIN_X, CAMERA_SENSOR_I2C_TYPE_WORD, false);
	gyro_y = k618_rohm_read(o_ctrl, K618_ROHM_GYRO_GAIN_Y, CAMERA_SENSOR_I2C_TYPE_WORD, false);
	CAM_DBG(CAM_OIS, "k618: ROHM OIS gyro gain x 0x%x y 0x%x", gyro_x, gyro_y);

	if (o_ctrl->i2c_fwinit_data.is_settings_valid != 1)
		return 0;
	rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_fwinit_data);
	if (rc)
		CAM_ERR(CAM_OIS, "k618: ROHM OIS %s: fwinit settings: %d (flag %u)",
			o_ctrl->ois_name, rc, o_ctrl->k618_ois_flag);
	return rc;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
#ifdef OPLUS_FEATURE_CAMERA_COMMON
int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
#else
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
#endif
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	struct cam_packet              *csl_packet_u = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
	uint32_t                        fwversion = 0;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto put_ref;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet_u = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);
	rc = cam_packet_util_copy_pkt_to_kmd(csl_packet_u, &csl_packet, remain_len);
	if (rc) {
		CAM_ERR(CAM_OIS, "Copying packet to KMD failed");
		goto put_ref;
	}

	/* k618: camera 24, park lens -- packet 6 on the 13 Ultra (k618_park_pkt) */
	if (o_ctrl->k618_ultra &&
		((csl_packet->header.op_code & 0xFFFFFF) == K618_OIS_OPCODE_PARKLENS)) {
		rc = k618_ultra_flush_check(o_ctrl, csl_packet);
		if (!rc)
			rc = k618_park_pkt(o_ctrl, csl_packet);
		goto end;
	}

	/* k618: camera 24 -- stock refuses packets of a flushed request (see k618_ultra_flush_check) */
	if (o_ctrl->k618_ultra) {
		rc = k618_ultra_flush_check(o_ctrl, csl_packet);
		if (rc)
			goto end;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		CAM_DBG(CAM_OIS, "CAM_OIS_PACKET_OPCODE_INIT,num_cmd_buf %d",
			csl_packet->num_cmd_buf);
		k618_park_on_init(o_ctrl);	/* k618: camera 24, park lens */

		offset = (uint32_t *)&csl_packet->payload_flex;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc) {
				CAM_ERR(CAM_OIS, "Invalid cmd desc");
				goto end;
			}

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				goto end;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				rc = -EINVAL;
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				goto end;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				goto end;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			CAM_DBG(CAM_OIS,
					"cmm_hdr->cmd_type: %d", cmm_hdr->cmd_type);
			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					break;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				if (o_ctrl->k618_pl_powered) {
					/* k618: camera 24, park lens -- the powered OIS keeps its settings */
					CAM_DBG(CAM_OIS, "k618: no need repower up again");
					break;
				}
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					break;
				}
				break;
			case CAMERA_SENSOR_OIS_CMD_TYPE_FW_INFO:
				fwversion = *cmd_buf;
				CAM_DBG(CAM_OIS,
					"Received fwInfo buffer,total_cmd_buf_in_bytes: %d, fwversion: %d",
					total_cmd_buf_in_bytes, fwversion);
				if (fwversion == CAM_OIS_FWINFO_VERSION_2) {
					rc = cam_ois_fw_info_pkt_parser_v2(
						o_ctrl, cmd_buf, total_cmd_buf_in_bytes);
					if (rc) {
						CAM_ERR(CAM_OIS,
						"Failed: parse fw info2 settings");
						break;
					}
				}
				else {
					rc = cam_ois_fw_info_pkt_parser(
						o_ctrl, cmd_buf, total_cmd_buf_in_bytes);
					if (rc) {
						CAM_ERR(CAM_OIS,
						"Failed: parse fw info settings");
						break;
					}
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init/config settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					break;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					break;
				}
			} else if (o_ctrl->i2c_fwinit_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS, "received fwinit settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_fwinit_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_DBG(CAM_OIS,
					"fw init parsing failed: %d", rc);
					break;
				}
			} else if (o_ctrl->k618_xiaomi_info &&
				o_ctrl->k618_postinit_data.is_settings_valid == 0) {
				/* k618: camera 16 -- Xiaomi's fourth buffer, applied after calib as 5.15 did */
				i2c_reg_settings = &(o_ctrl->k618_postinit_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS, "k618: postinit parsing failed: %d", rc);
					break;
				}
				CAM_INFO(CAM_OIS, "k618: OIS %s: postinit settings received",
					o_ctrl->ois_name);
			}
			break;
			}
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);

			if (rc < 0)
				goto end;
		}

		/* k618: camera 24, park lens -- not when a stopped park left the OIS powered */
		if (!k618_park_take_power(o_ctrl) &&
			(o_ctrl->cam_ois_state != CAM_OIS_CONFIG)) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				goto end;
			}
		}

		/* k618: camera 16, third round -- see cam_ois_apply_settings() */
		o_ctrl->k618_poll_go_on = o_ctrl->k618_xiaomi_info ? 4 : 0;
		/* k618: Xiaomi's ROHM OIS path, as the phone's 5.15 camera.ko (camera 16) */
		if (o_ctrl->k618_xiaomi_info && o_ctrl->k618_ois_flag >= 1 &&
			o_ctrl->k618_ois_flag <= ARRAY_SIZE(k618_rohm_prog_id)) {
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
			rc = k618_rohm_ois_init(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "k618: ROHM OIS init failed: %d (flag %u)",
					rc, o_ctrl->k618_ois_flag);
				goto pwr_dwn;
			}
			if (rc)	/* 5.15 returned this 1 to CamX and kept the OIS powered */
				CAM_WARN(CAM_OIS, "k618: ROHM OIS: a poll at the end of fwinit timed out");
			rc = 0;
			goto k618_init_done;
		}

		CAM_DBG(CAM_OIS, "ois_fw_flag: %d", o_ctrl->ois_fw_flag);
		/* k618: camera 24 -- the 13 Ultra's chip types, and its fwinit with ois_fw_flag clear */
		if (o_ctrl->k618_ultra_info) {
			rc = k618_ultra_init(o_ctrl);
			if (rc < 0)
				goto pwr_dwn;
			if (rc == K618_ULTRA_INIT_DONE) {
				rc = 0;
				goto k618_init_done;
			}
		}
		if (o_ctrl->ois_fw_flag) {
			CAM_DBG(CAM_OIS, "fw_count: %d", o_ctrl->fw_info.fw_count);
			if (o_ctrl->fw_info.fw_count != 0) {
				rc = cam_ois_fw_download_v2(o_ctrl);
				if (rc) {
					CAM_ERR(CAM_OIS, "Failed OIS FW Download v2");
					goto pwr_dwn;
				}
			} else {
				if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1) {
					rc = cam_ois_apply_settings(o_ctrl,
						&o_ctrl->i2c_fwinit_data);
					if ((rc == -EAGAIN) &&
						(o_ctrl->io_master_info.master_type ==
							CCI_MASTER)) {
						CAM_WARN(CAM_OIS,
							"Reapplying fwinit settings");
						usleep_range(1000, 1010);
						rc = cam_ois_apply_settings(o_ctrl,
							&o_ctrl->i2c_fwinit_data);
					}
					if (rc) {
						CAM_ERR(CAM_OIS,
							"Cannot apply fwinit data %d",
							rc);
						goto pwr_dwn;
					} else {
						CAM_DBG(CAM_OIS, "OIS fwinit settings success");
					}

					rc = k618_ultra_fw_download(o_ctrl);	/* k618: camera 24 (SEM1217S) */
					if (rc) {
						CAM_ERR(CAM_OIS, "Failed OIS FW Download");
						goto pwr_dwn;
					}
				}
			}
		}
		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		{
			rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
			if ((rc == -EAGAIN) &&
				(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_OIS,
					"CCI HW is restting: Reapplying INIT settings");
				usleep_range(1000, 1010);
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_init_data);
			}

			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Cannot apply Init settings: rc = %d",
					rc);
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_OIS, "apply Init settings success");
			}
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if ((rc == -EAGAIN) &&
				(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_OIS,
					"CCI HW is restting: Reapplying calib settings");
				usleep_range(1000, 1010);
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_calib_data);
			}
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_OIS, "apply calib data settings success");
			}
		}
		/* k618: camera 16 -- Xiaomi's postinit, last, as 5.15 (a poll that timed out: logged) */
		if (o_ctrl->k618_postinit_data.is_settings_valid == 1) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->k618_postinit_data);
			if ((rc == -EAGAIN) &&
				(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
				usleep_range(1000, 1010);
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->k618_postinit_data);
			}
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "k618: cannot apply postinit data: %d", rc);
				goto pwr_dwn;
			}
			if (rc)
				CAM_WARN(CAM_OIS, "k618: OIS %s: a postinit poll timed out",
					o_ctrl->ois_name);
			rc = 0;
		}

k618_init_done:
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;

		rc = delete_request(&o_ctrl->i2c_fwinit_data);
		if (rc < 0)
			CAM_WARN(CAM_OIS,
				"Fail deleting fwinit data: rc: %d", rc);

		for (i = 0; i < o_ctrl->fw_info.fw_count; i++) {
			if (o_ctrl->i2c_fw_init_data &&
				(o_ctrl->i2c_fw_init_data[i].is_settings_valid == 1)) {
				rc = delete_request(&o_ctrl->i2c_fw_init_data[i]);
				if (rc < 0)
					CAM_WARN(CAM_OIS,
						"Fail deleting i2c_fw_init_data: rc: %d", rc);
			}
			if (o_ctrl->i2c_fw_finalize_data &&
				(o_ctrl->i2c_fw_finalize_data[i].is_settings_valid == 1)) {
				rc = delete_request(&o_ctrl->i2c_fw_finalize_data[i]);
				if (rc < 0)
					CAM_WARN(CAM_OIS,
						"Fail deleting i2c_fw_finalize_data: rc: %d", rc);
			}
		}
		if (o_ctrl->i2c_fw_init_data){
			CAM_MEM_FREE(o_ctrl->i2c_fw_init_data);
			o_ctrl->i2c_fw_init_data = NULL;
		}
		if (o_ctrl->i2c_fw_finalize_data){
			CAM_MEM_FREE(o_ctrl->i2c_fw_finalize_data);
			o_ctrl->i2c_fw_finalize_data = NULL;
		}
		if (o_ctrl->fw_info.fw_param) {
			CAM_MEM_FREE(o_ctrl->fw_info.fw_param);
			o_ctrl->fw_info.fw_param = NULL;
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0)
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);

		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		if (o_ctrl->k618_postinit_data.is_settings_valid == 1)	/* k618 */
			delete_request(&o_ctrl->k618_postinit_data);
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		CAM_DBG(CAM_OIS, "CAM_OIS_PACKET_OPCODE_OIS_CONTROL");
		o_ctrl->k618_poll_go_on = 0;	/* k618: per-frame packets fail fast */
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			goto end;
		}
		offset = (uint32_t *)&csl_packet->payload_flex;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto end;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto end;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto end;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_READ: {
		uint64_t qtime_ns;
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		CAM_DBG(CAM_OIS, "CAM_OIS_PACKET_OPCODE_READ");

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			goto end;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_OIS, "No I/O configs to process");
			rc = -EINVAL;
			goto end;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload_flex +
			csl_packet->io_configs_offset);

		/* validate read data io config */
		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			goto end;
		}

		offset = (uint32_t *)&csl_packet->payload_flex;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, &io_cfg[0]);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			goto end;
		}

		mutex_lock(&(o_ctrl->read_buf_lock));
		rc = cam_sensor_util_add_read_buf_to_list(&(o_ctrl->read_buf_list),
			io_cfg->mem_handle[0]);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Add read buf to list failed rc:%d", rc);
			mutex_unlock(&(o_ctrl->read_buf_lock));
			goto end;
		}
		mutex_unlock(&(o_ctrl->read_buf_lock));

		rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "failed to get qtimer rc:%d");
			goto end;
		}

		rc = k618_ultra_read_data(o_ctrl,	/* k618: camera 24; elsewhere cam_sensor_i2c_read_data */
			&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			goto end;
		}

		if (csl_packet->num_io_configs > 1) {
			rc = cam_sensor_util_write_qtimer_to_io_buffer(
				qtime_ns, &io_cfg[1]);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"write qtimer failed rc: %d", rc);
				delete_request(&i2c_read_settings);
				goto end;
			}
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in deleting the read settings");
			goto end;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_WRITE_TIME: {
		CAM_DBG(CAM_OIS,
				"CAM_OIS_PACKET_OPCODE_WRITE_TIME");
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_ERR(CAM_OIS,
				"Not in right state to write time to OIS: %d",
				o_ctrl->cam_ois_state);
			goto end;
		}
		offset = (uint32_t *)&csl_packet->payload_flex;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_time_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto end;
		}

		if (o_ctrl->fw_info.fw_count > 0) {
			uint8_t ois_endianness =
				(o_ctrl->fw_info.endianness & OIS_ENDIANNESS_MASK_INPUTPARAM) >> 4;
			rc = cam_ois_update_time(i2c_reg_settings, ois_endianness);
		} else
			rc = cam_ois_update_time(i2c_reg_settings, CAM_ENDIANNESS_LITTLE);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot update time");
			goto end;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto end;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto end;
		}
		break;
	}
	case K618_OIS_OPCODE_APERTURE:
	case K618_OIS_OPCODE_REDOWNLOAD:
	case K618_OIS_OPCODE_PARKLENS:
	case K618_OIS_OPCODE_SKIPFRAME:
	case CAM_PKT_NOP_OPCODE:
		/* k618: camera 24 -- the 13 Ultra's packets; elsewhere as before */
		if (o_ctrl->k618_ultra) {
			rc = k618_ultra_pkt(o_ctrl, csl_packet);
			goto end;
		}
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto end;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto end;
	}

	if (!rc)
		goto end;

pwr_dwn:
	o_ctrl->k618_poll_go_on = 0;	/* k618 */
	cam_ois_power_down(o_ctrl);
end:
	k618_park_init_end(o_ctrl);	/* k618: camera 24, park lens */
	cam_common_mem_free(csl_packet);
put_ref:
	cam_mem_put_cpu_buf(dev_config.packet_handle);
	return rc;
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
EXPORT_SYMBOL(cam_ois_pkt_parse);
#endif

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0, i = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
	bool k618_park_down = k618_park_shutdown(o_ctrl);	/* k618: camera 24, park lens */

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	k618_ultra_release(o_ctrl);	/* k618: camera 24 */

	if ((o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) && !k618_park_down) {	/* k618: camera 24, park lens */
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_fwinit_data);

	for (i = 0; i < o_ctrl->fw_info.fw_count; i++) {
		if (o_ctrl->i2c_fw_init_data &&
			(o_ctrl->i2c_fw_init_data[i].is_settings_valid == 1)) {
			rc = delete_request(&o_ctrl->i2c_fw_init_data[i]);
			if (rc < 0)
				CAM_WARN(CAM_OIS,
					"Fail deleting i2c_fw_init_data: rc: %d", rc);
		}
		if (o_ctrl->i2c_fw_finalize_data &&
			(o_ctrl->i2c_fw_finalize_data[i].is_settings_valid == 1)) {
			rc = delete_request(&o_ctrl->i2c_fw_finalize_data[i]);
			if (rc < 0)
				CAM_WARN(CAM_OIS,
					"Fail deleting i2c_fw_finalize_data: rc: %d", rc);
		}
	}

	if (o_ctrl->i2c_fw_version_data.is_settings_valid == 1) {
		rc = delete_request(&o_ctrl->i2c_fw_version_data);
		if (rc < 0)
			CAM_WARN(CAM_OIS,
				"Fail deleting i2c_fw_version_data: rc: %d", rc);
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);
	if (o_ctrl->k618_postinit_data.is_settings_valid == 1)	/* k618 */
		delete_request(&o_ctrl->k618_postinit_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

	if (o_ctrl->i2c_fw_init_data){
		CAM_MEM_FREE(o_ctrl->i2c_fw_init_data);
		o_ctrl->i2c_fw_init_data = NULL;
	}
	if (o_ctrl->i2c_fw_finalize_data){
		CAM_MEM_FREE(o_ctrl->i2c_fw_finalize_data);
		o_ctrl->i2c_fw_finalize_data = NULL;
	}
	if (o_ctrl->fw_info.fw_param) {
		CAM_MEM_FREE(o_ctrl->fw_info.fw_param);
		o_ctrl->fw_info.fw_param = NULL;
	}
	CAM_MEM_FREE(power_info->power_setting);
	CAM_MEM_FREE(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
EXPORT_SYMBOL(cam_ois_shutdown);
#endif

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0, i = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		k618_ultra_start(o_ctrl);	/* k618: camera 24 */
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		/* k618: camera 24, park lens -- a parklens thread may own the power-down */
		o_ctrl->k618_pl_keep_power = k618_park_release(o_ctrl);
		if ((o_ctrl->cam_ois_state == CAM_OIS_CONFIG) &&
			!o_ctrl->k618_pl_keep_power) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;
		k618_ultra_release(o_ctrl);	/* k618: camera 24 */

		if (!o_ctrl->k618_pl_keep_power) {	/* k618: camera 24, park lens -- else the thread's */
			CAM_MEM_FREE(power_info->power_setting);
			CAM_MEM_FREE(power_info->power_down_setting);
			power_info->power_setting = NULL;
			power_info->power_down_setting = NULL;
			power_info->power_down_setting_size = 0;
			power_info->power_setting_size = 0;
		}
		o_ctrl->k618_pl_keep_power = false;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);
		if (o_ctrl->k618_postinit_data.is_settings_valid == 1)	/* k618 */
			delete_request(&o_ctrl->k618_postinit_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_fwinit_data);

		for (i = 0; i < o_ctrl->fw_info.fw_count; i++) {
			if (o_ctrl->i2c_fw_init_data &&
				(o_ctrl->i2c_fw_init_data[i].is_settings_valid == 1)) {
				rc = delete_request(&o_ctrl->i2c_fw_init_data[i]);
				if (rc < 0) {
					CAM_WARN(CAM_OIS,
						"Fail deleting i2c_fw_init_data: rc: %d", rc);
					rc = 0;
				}
			}
			if (o_ctrl->i2c_fw_finalize_data &&
				(o_ctrl->i2c_fw_finalize_data[i].is_settings_valid == 1)) {
				rc = delete_request(&o_ctrl->i2c_fw_finalize_data[i]);
				if (rc < 0) {
					CAM_WARN(CAM_OIS,
						"Fail deleting i2c_fw_finalize_data: rc: %d", rc);
					rc = 0;
				}
			}
		}

		if (o_ctrl->i2c_fw_init_data){
			CAM_MEM_FREE(o_ctrl->i2c_fw_init_data);
			o_ctrl->i2c_fw_init_data = NULL;
		}
		if (o_ctrl->i2c_fw_finalize_data){
			CAM_MEM_FREE(o_ctrl->i2c_fw_finalize_data);
			o_ctrl->i2c_fw_finalize_data = NULL;
		}
		if (o_ctrl->fw_info.fw_param) {
			CAM_MEM_FREE(o_ctrl->fw_info.fw_param);
			o_ctrl->fw_info.fw_param = NULL;
		}
		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state for stop : %d",
				o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		k618_ultra_stop(o_ctrl);	/* k618: camera 24 */
		break;
	case CAM_FLUSH_REQ:
		CAM_DBG(CAM_OIS, "Flush recveived");
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode: %d", cmd->op_code);
		goto release_mutex;
	}
release_mutex:
	mutex_lock(&(o_ctrl->read_buf_lock));
	cam_sensor_util_release_read_buf(&(o_ctrl->read_buf_list));
	mutex_unlock(&(o_ctrl->read_buf_lock));
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
EXPORT_SYMBOL(cam_ois_driver_cmd);
#endif

/*
 * k618 (camera 24): the 13 Ultra's OIS, rebuilt from that phone's own 5.15 camera.ko (ishtar
 * OS3.0.308.0.WMACNXM, symbols kept; the addresses are offsets in its .text). Every entry point
 * returns early unless o_ctrl->k618_ultra is set, which k618_ois_ultra_setup() does at probe when
 * the root node's "model" names ishtar; on the 13 Pro camera 16's paths run as before.
 *  - slave info: 65 bytes (0x162714 refuses 64 or fewer); 0x38 address type and 0x39 page step as
 *    on the 13 Pro, 0x3a chip id (u16), 0x3c expected program id (u32; RUMBA-S4H: checksum),
 *    0x40 OIS type (0x1626fc copies 0x28..0x40 to o_ctrl+0xe72..0xe8a).
 *  - INIT by type (0x1603e4; table pflash_ois at .rodata 0x895e0 = 1, 2, 4, 3, 5, 6, 7): 1-4 and
 *    7 BU24721, 5 BU24532, 6 RUMBA-S4H; 0 the generic path, where chip id 0x4c1 is a SEM1217S
 *    (cam_ois_fw_download 0x162d74).
 *  - a request-manager device (cam_ois_component_bind 0x157b9c-0x157bdc: get_dev_info,
 *    link_setup, apply_req; only the I2C variant 0x1583b4-0x1583e0 adds flush_req and the
 *    frame-skip query): packets 4 (settings by request id, "aperture mode"), 0x7e (skip frames),
 *    0x7f (NOP). Packet 5 runs the chip function again with new calib/postinit buffers; 6 (park
 *    lens) is accepted and not acted on.
 *  - RUMBA-S4H and SEM1217S: stock rewrites the chip's flash when its version differs from the
 *    packaged image; here only with camera.k618_ois_flash_update=1, otherwise both versions are
 *    logged and the flash is left as it is.
 */
static bool k618_ois_flash_update;
module_param(k618_ois_flash_update, bool, 0644);
MODULE_PARM_DESC(k618_ois_flash_update,
	"k618: 13 Ultra OIS -- rewrite the RUMBA-S4H / SEM1217S flash when its version differs from the packaged image (off: log both versions)");

static uint k618_bu24532_fw_update_op;
module_param(k618_bu24532_fw_update_op, uint, 0644);
MODULE_PARM_DESC(k618_bu24532_fw_update_op,
	"k618: 13 Ultra BU24532 -- stock bu24532_ois_fw_update_op: 0 download if the program id differs after auto boot, 1 download unless the expected id is 0, 2 always");

#define K618_BU24532_STATUS	0x6024	/* SETTING_6024: == 1 within 100 ms */
#define K618_BU24532_PROG_ID	0x6010	/* SETTING_PROGRAM_ID, dword */
#define K618_BU24532_DL_SID	0x50	/* 7-bit; stock sets cci_client->sid to it around each burst */
#define K618_RUMBAS4H_FW_SIZE	0x7000
#define K618_RUMBAS4H_VER_OFS	0x6ff4
#define K618_SEM1217S_FW_SIZE	0xc000
#define K618_SEM1217S_VER_OFS	0xbff4

static uint32_t k618_le32(const uint8_t *p)
{
	return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int k618_ultra_cci(struct cam_ois_ctrl_t *o_ctrl, const char *chip)
{
	if (o_ctrl->io_master_info.master_type != CCI_MASTER ||
		!o_ctrl->io_master_info.cci_client) {
		CAM_ERR(CAM_OIS, "k618: %s OIS %s needs a CCI master (type %d)", chip,
			o_ctrl->ois_name, o_ctrl->io_master_info.master_type);
		return -EINVAL;
	}
	return 0;
}

/* len bytes from addr, word address (stock rumbas4h_i2c_read_data 0x15c320, i2c_read_data 0x16448c) */
static int k618_ois_rd(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint8_t *buf, int32_t len)
{
	int rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, buf,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, len);

	if (rc < 0)
		CAM_ERR(CAM_OIS, "k618: OIS %s: read 0x%04x (%d bytes) failed: %d",
			o_ctrl->ois_name, addr, len, rc);
	return rc;
}

/*
 * len (1..256) bytes from addr on, word address, byte data, then delay_ms: as one burst (the
 * RUMBA-S4H data buffer and checksum) or sequential (stock i2c_write_data 0x1645c0, SEM1217S)
 */
static int k618_ois_wr(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, const uint8_t *buf,
	uint32_t len, uint8_t mode, unsigned short delay_ms)
{
	struct cam_sensor_i2c_reg_setting setting = {0};
	struct cam_sensor_i2c_reg_array *regs;
	uint32_t i;
	int rc;

	if (!len || len > 256)
		return -EINVAL;
	regs = kcalloc(len, sizeof(*regs), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;
	for (i = 0; i < len; i++) {
		regs[i].reg_addr = (mode == CAM_SENSOR_I2C_WRITE_SEQ) ? addr + i : addr;
		regs[i].reg_data = buf[i];
	}
	setting.reg_setting = regs;
	setting.size = len;
	setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	setting.delay = delay_ms;
	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info), &setting, mode);
	kfree(regs);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "k618: OIS %s: write 0x%04x (%u bytes) failed: %d",
			o_ctrl->ois_name, addr, len, rc);
	return rc;
}

/*
 * stock bu24721_ois_pkt_download, the 13 Ultra's (0x158980): camera 16's k618_rohm_ois_init()
 * with three differences -- a restart ahead of the first program-id read (F050, wait, F097 +1 ms,
 * F058 +8 ms, F050, wait: 0x158a20-0x158aa8), the expected program id from the slave info
 * (0x158af8, o_ctrl+0xe86) where the 13 Pro's module indexes a table by type (0x156a3c), and no
 * gyro-gain reads. A failed download goes back to the run-time slave before returning, as
 * camera 16 does (stock returns with the download slave set).
 */
static int k618_ultra_bu24721_init(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t want = o_ctrl->k618_ultra_expect_id;
	uint32_t id;
	int rc, write_rc = 0;

	rc = k618_ultra_cci(o_ctrl, "BU24721");
	if (rc)
		return rc;

	k618_rohm_write(o_ctrl, K618_ROHM_STANDBY, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
	k618_rohm_wait_ready(o_ctrl);
	k618_rohm_write(o_ctrl, K618_ROHM_RESTART_1, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
	k618_rohm_write(o_ctrl, K618_ROHM_RESTART_2, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 8);
	k618_rohm_write(o_ctrl, K618_ROHM_STANDBY, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
	k618_rohm_wait_ready(o_ctrl);
	id = k618_rohm_read(o_ctrl, K618_ROHM_PROG_ID, CAMERA_SENSOR_I2C_TYPE_DWORD, true);
	k618_rohm_wait_ready(o_ctrl);
	CAM_INFO(CAM_OIS, "k618: BU24721 OIS %s type %u: program id 0x%08x, slave info wants 0x%08x",
		o_ctrl->ois_name, o_ctrl->k618_ultra_type, id, want);

	if (id != want) {
		k618_rohm_set_slave(o_ctrl, K618_ROHM_DL_SLAVE);
		if (o_ctrl->i2c_init_data.is_settings_valid == 1) {
			rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_init_data);
			if (rc)	/* stock logs this and downloads all the same (0x158d08) */
				CAM_ERR(CAM_OIS, "k618: BU24721 OIS %s: download-mode settings: %d",
					o_ctrl->ois_name, rc);
		}
		rc = k618_rohm_load_fw(o_ctrl, "prog", o_ctrl->opcode.prog, &write_rc);
		if (!rc && write_rc >= 0)
			rc = k618_rohm_load_fw(o_ctrl, "coeff", o_ctrl->opcode.coeff, &write_rc);
		if (!rc && write_rc >= 0)
			rc = k618_rohm_load_fw(o_ctrl, "mem", o_ctrl->opcode.memory, &write_rc);
		k618_rohm_set_slave(o_ctrl, K618_ROHM_RUN_SLAVE);
		if (rc)
			return rc;

		k618_rohm_write(o_ctrl, K618_ROHM_RESTART_1, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
		k618_rohm_write(o_ctrl, K618_ROHM_RESTART_2, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 8);
		k618_rohm_wait_ready(o_ctrl);
		k618_rohm_write(o_ctrl, K618_ROHM_STANDBY, 0, CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
		k618_rohm_wait_ready(o_ctrl);
		id = k618_rohm_read(o_ctrl, K618_ROHM_PROG_ID, CAMERA_SENSOR_I2C_TYPE_DWORD, true);
		k618_rohm_wait_ready(o_ctrl);
		if (id == want && write_rc >= 0)
			CAM_INFO(CAM_OIS, "k618: BU24721 OIS %s: firmware loaded, program id 0x%08x",
				o_ctrl->ois_name, id);
		else
			CAM_ERR(CAM_OIS,
				"k618: BU24721 OIS %s: program id 0x%08x after the download, wants 0x%08x (write %d)",
				o_ctrl->ois_name, id, want, write_rc);
	}

	if (o_ctrl->is_ois_calib)
		k618_rohm_apply_calib(o_ctrl, &o_ctrl->i2c_calib_data);
	if (o_ctrl->i2c_fwinit_data.is_settings_valid != 1)
		return 0;
	rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_fwinit_data);
	if (rc)
		CAM_ERR(CAM_OIS, "k618: BU24721 OIS %s: fwinit settings: %d", o_ctrl->ois_name, rc);
	return rc;
}

static void k618_bu24532_wait(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = camera_io_dev_poll(&(o_ctrl->io_master_info), K618_BU24532_STATUS, 0x01, 0,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, 100);

	if (rc)
		CAM_WARN(CAM_OIS, "k618: BU24532 OIS %s: status 0x%04x wait %d",
			o_ctrl->ois_name, K618_BU24532_STATUS, rc);
}

/* stock apply_fw_setting (0x1622bc), the "auto boot" */
static void k618_bu24532_boot(struct cam_ois_ctrl_t *o_ctrl)
{
	CAM_DBG(CAM_OIS, "k618: [BU24532] %s: apply kernel firmware setting", o_ctrl->ois_name);
	k618_rohm_write(o_ctrl, 0x7010, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0x6019, 0x87, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_rohm_write(o_ctrl, 0x6018, 0x10, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0x6080, 0xc800b4e1, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_rohm_write(o_ctrl, 0x6084, 0x00000000, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0x6080, 0xc800b0e1, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_rohm_write(o_ctrl, 0x6084, 0x00000001, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0xF011, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	msleep(40);
}

/* stock 0x15a008-0x15a114, ahead of a download */
static void k618_bu24532_dl_setup(struct cam_ois_ctrl_t *o_ctrl)
{
	k618_rohm_write(o_ctrl, 0x7010, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0x6019, 0x87, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_rohm_write(o_ctrl, 0x6018, 0x10, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0x6080, 0x10868ce1, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_rohm_write(o_ctrl, 0x6084, 0x08068180, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0x6080, 0x108690e1, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_rohm_write(o_ctrl, 0x6084, 0x08068100, CAMERA_SENSOR_I2C_TYPE_DWORD, 0);
	k618_bu24532_wait(o_ctrl);
	k618_rohm_write(o_ctrl, 0xF013, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 0);
	usleep_range(2000, 2010);
}

/*
 * <ois_name>.<suffix> (stock 0x15a4f4-0x15a708 for .prog; .coeff, .mem and .pher alike): size / 4
 * big-endian dwords, 16 per burst to the download slave at base + 64 * page, the page counted up
 * per burst when the slave info's page-step byte is set, 10 ms after each burst
 */
static int k618_bu24532_load(struct cam_ois_ctrl_t *o_ctrl, const char *suffix, uint32_t base)
{
	struct cam_sensor_cci_client *cci = o_ctrl->io_master_info.cci_client;
	struct cam_sensor_i2c_reg_array regs[16];
	struct cam_sensor_i2c_reg_setting setting = {0};
	const struct firmware *fw = NULL;
	char name[MAX_OIS_NAME_SIZE + 8];
	uint32_t count, done, page = 0, n = 0, i;
	uint16_t sid;
	int rc;

	snprintf(name, sizeof(name), "%s.%s", o_ctrl->ois_name, suffix);
	rc = request_firmware(&fw, name, &(o_ctrl->pdev->dev));
	if (rc) {
		CAM_ERR(CAM_OIS, "k618: [BU24532]Failed to locate %s: %d", name, rc);
		return rc;
	}
	count = (fw->size >> 2) & 0xffff;	/* stock: ubfx w24, w8, #2, #16 */
	setting.reg_setting = regs;
	setting.addr_type = o_ctrl->k618_fw_addr_type;
	if (setting.addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID ||
		setting.addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
		CAM_WARN(CAM_OIS, "k618: [BU24532] firmware address type %u, using word",
			o_ctrl->k618_fw_addr_type);
		setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	}
	setting.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	setting.delay = 10;
	sid = cci->sid;
	for (done = 0; done < count; done += n) {
		n = min_t(uint32_t, ARRAY_SIZE(regs), count - done);
		for (i = 0; i < n; i++) {
			const uint8_t *p = fw->data + 4 * (done + i);

			regs[i].reg_addr = base + (page << 6);
			regs[i].reg_data = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
				((uint32_t)p[2] << 8) | p[3];
			regs[i].delay = 0;
			regs[i].data_mask = 0;
		}
		setting.size = n;
		if (o_ctrl->k618_fw_page_inc)
			page++;
		cci->sid = K618_BU24532_DL_SID;
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info), &setting,
			CAM_SENSOR_I2C_WRITE_BURST);
		cci->sid = sid;
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: [BU24532]OIS FW(%s) size(%u) download failed rc: %d",
				suffix, count, rc);
			break;
		}
	}
	if (rc >= 0)
		CAM_INFO(CAM_OIS, "k618: [BU24532] %s: %s, %u dwords from 0x%x, page step %u",
			o_ctrl->ois_name, name, count, base, o_ctrl->k618_fw_page_inc);
	release_firmware(fw);
	return rc < 0 ? rc : 0;
}

/*
 * stock bu24532_ois_pkt_download (0x159d74). A fwinit buffer is applied instead of all the rest
 * (0x159dac). Expected id 0xff: no boot, no download (0x159e8c). With update op 0 (stock's
 * default): auto boot (skipped in packet 5), read the program id; in packet 5 a mismatch gets up
 * to three more boots, then -EINVAL (0x15a234-0x15a4ec). A download (decided at 0x159f88: op 2,
 * or op 0/1 with a mismatch) sends .prog .coeff .mem .pher, boots and reads the id again; with
 * op 0 a mismatch then fails (0x15bce0). Then postinit and calib. Stock's fallback of a download
 * read from the module's EEPROM (bu24532_ois_enable_read_eeprom, default 0) is not ported.
 */
static int k618_bu24532_init(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t want = o_ctrl->k618_ultra_expect_id, id = 0;
	uint32_t op = k618_bu24532_fw_update_op;
	int rc, retry;

	rc = k618_ultra_cci(o_ctrl, "BU24532");
	if (rc)
		return rc;

	if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1) {
		rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_fwinit_data);
		if (rc)
			CAM_ERR(CAM_OIS, "k618: [BU24532] %s: Cannot apply fwinit data %d",
				o_ctrl->ois_name, rc);
		return rc;
	}

	if (want != 0xff) {
		if (op == 0) {
			if (!o_ctrl->k618_ultra_redl)
				k618_bu24532_boot(o_ctrl);
			k618_bu24532_wait(o_ctrl);
			k618_bu24532_wait(o_ctrl);
			id = k618_rohm_read(o_ctrl, K618_BU24532_PROG_ID,
				CAMERA_SENSOR_I2C_TYPE_DWORD, true);
			CAM_INFO(CAM_OIS, "k618: [BU24532] %s: read program ID:0x%x, current ID:0x%x",
				o_ctrl->ois_name, id, want);
			for (retry = 0; o_ctrl->k618_ultra_redl && id != want && retry < 3; retry++) {
				CAM_INFO(CAM_OIS,
					"k618: [BU24532] %s: retry[%d:max%d] auto boot when open camera",
					o_ctrl->ois_name, retry, 3);
				k618_bu24532_boot(o_ctrl);
				k618_bu24532_wait(o_ctrl);
				k618_bu24532_wait(o_ctrl);
				id = k618_rohm_read(o_ctrl, K618_BU24532_PROG_ID,
					CAMERA_SENSOR_I2C_TYPE_DWORD, true);
				CAM_INFO(CAM_OIS, "k618: [BU24532] %s: read program ID:0x%x, current ID:0x%x",
					o_ctrl->ois_name, id, want);
			}
			if (o_ctrl->k618_ultra_redl && id != want) {
				CAM_ERR(CAM_OIS,
					"k618: [BU24532] %s: read program ID:0x%x, current ID:0x%x, AUTO BOOT failed",
					o_ctrl->ois_name, id, want);
				return -EINVAL;
			}
		}
		if (op == 2 || ((op == 0 || op == 1) && id != want)) {
			k618_bu24532_dl_setup(o_ctrl);
			rc = k618_bu24532_load(o_ctrl, "prog", o_ctrl->opcode.prog);
			if (!rc)
				rc = k618_bu24532_load(o_ctrl, "coeff", o_ctrl->opcode.coeff);
			if (!rc)
				rc = k618_bu24532_load(o_ctrl, "mem", o_ctrl->opcode.memory);
			if (!rc)
				rc = k618_bu24532_load(o_ctrl, "pher", o_ctrl->opcode.pheripheral);
			if (!rc) {
				k618_bu24532_boot(o_ctrl);
				k618_bu24532_wait(o_ctrl);
				k618_bu24532_wait(o_ctrl);
				id = k618_rohm_read(o_ctrl, K618_BU24532_PROG_ID,
					CAMERA_SENSOR_I2C_TYPE_DWORD, true);
				CAM_INFO(CAM_OIS,
					"k618: [BU24532] %s: after DL, read program ID:0x%x, current ID:0x%x",
					o_ctrl->ois_name, id, want);
				if (op == 0 && id != want)
					rc = -EINVAL;
			}
			if (rc) {
				CAM_ERR(CAM_OIS, "k618: [BU24532] %s: fw update failed: %d",
					o_ctrl->ois_name, rc);
				return rc;
			}
			CAM_INFO(CAM_OIS, "k618: [BU24532] %s: FW update success", o_ctrl->ois_name);
		}
	}

	if (o_ctrl->k618_postinit_data.is_settings_valid == 1) {
		rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->k618_postinit_data);
		if (rc) {
			CAM_ERR(CAM_OIS, "k618: [BU24532] %s: Cannot apply postinit data %d",
				o_ctrl->ois_name, rc);
			return rc;
		}
	}
	if (o_ctrl->is_ois_calib) {
		rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_calib_data);
		if (rc) {
			CAM_ERR(CAM_OIS, "k618: [BU24532] %s: Cannot apply calib data %d",
				o_ctrl->ois_name, rc);
			return rc;
		}
	}
	return 0;
}

/*
 * stock RUMBA-S4H flash rewrite (0x15cb68-0x15d1f0), at FAST mode: 0x0001 must read 1; 0x000C =
 * 0x75 (+65 ms); the image as 112 bursts of 256 bytes to 0x0100 (+13 ms each); 0x0006 must read
 * 0; 0x0008..0x000B = checksum low, checksum high, 0x00, 0x80 (+210 ms), the checksum being the
 * slave info's 0x3c (0x15cd98); 0x0006 must read 0; 0x0036 = 0x41 (+190 ms), 0x000D = 0x01
 * (+20 ms); 0x0001 must reach 9 within ten reads 1 ms apart; 0x000E = 0x06 (+10 ms)
 */
static int k618_rumbas4h_flash(struct cam_ois_ctrl_t *o_ctrl, const uint8_t *img)
{
	uint16_t cksum = o_ctrl->k618_ultra_expect_id & 0xffff;
	uint8_t b[4] = {0};
	uint32_t blk;
	int rc, i;

	rc = k618_ois_rd(o_ctrl, 0x0001, b, 1);
	if (rc < 0)
		return rc;
	if (b[0] != 1) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: not in idle status (0x%02x)",
			o_ctrl->ois_name, b[0]);
		return -EINVAL;
	}
	CAM_INFO(CAM_OIS, "k618: [RUMBAS4H] %s: in idle status, begin FW update", o_ctrl->ois_name);
	k618_rohm_write(o_ctrl, 0x000C, 0x75, CAMERA_SENSOR_I2C_TYPE_BYTE, 65);
	for (blk = 0; blk < K618_RUMBAS4H_FW_SIZE / 256; blk++) {
		rc = k618_ois_wr(o_ctrl, 0x0100, img + 256 * blk, 256,
			CAM_SENSOR_I2C_WRITE_BURST, 13);
		if (rc < 0)
			return rc;
	}
	rc = k618_ois_rd(o_ctrl, 0x0006, b, 2);
	if (rc < 0)
		return rc;
	if (b[0] | b[1]) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: FW update fail, 0x0006 = 0x%04x",
			o_ctrl->ois_name, b[0] | (b[1] << 8));
		return -EINVAL;
	}
	b[0] = cksum & 0xff;
	b[1] = cksum >> 8;
	b[2] = 0x00;
	b[3] = 0x80;
	rc = k618_ois_wr(o_ctrl, 0x0008, b, 4, CAM_SENSOR_I2C_WRITE_BURST, 210);
	if (rc < 0)
		return rc;
	rc = k618_ois_rd(o_ctrl, 0x0006, b, 2);
	if (rc < 0)
		return rc;
	CAM_INFO(CAM_OIS, "k618: [RUMBAS4H] %s: CHKSUM status = 0x%x", o_ctrl->ois_name,
		b[0] | (b[1] << 8));
	if (b[0] | b[1]) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: FW update fail", o_ctrl->ois_name);
		return -EINVAL;
	}
	k618_rohm_write(o_ctrl, 0x0036, 0x41, CAMERA_SENSOR_I2C_TYPE_BYTE, 190);
	k618_rohm_write(o_ctrl, 0x000D, 0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 20);
	for (i = 0; i < 10; i++) {
		b[0] = 0;
		k618_ois_rd(o_ctrl, 0x0001, b, 1);
		msleep(1);
		if (b[0] == 9)
			break;
	}
	if (b[0] != 9) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: software reset fail, addr 0x1 rxdata 0x%x",
			o_ctrl->ois_name, b[0]);
		return -EINVAL;
	}
	CAM_INFO(CAM_OIS, "k618: [RUMBAS4H] %s: software reset success", o_ctrl->ois_name);
	k618_rohm_write(o_ctrl, 0x000E, 0x06, CAMERA_SENSOR_I2C_TYPE_BYTE, 10);
	CAM_INFO(CAM_OIS, "k618: [RUMBAS4H] %s: FW update success", o_ctrl->ois_name);
	return 0;
}

/*
 * stock rumbas4h_ois_pkt_download (0x15c840): the packaged version is the u32 at 0x6ff4 of
 * <ois_name>.prog, the chip's 0x00FC (4 bytes; three tries 1 ms apart, then -EINVAL); a
 * difference rewrites the flash (here only with k618_ois_flash_update); then calib (a failure is
 * logged and the init settings still go out, 0x15ce58) and the init settings.
 */
static int k618_rumbas4h_init(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_sensor_cci_client *cci;
	const struct firmware *fw = NULL;
	char name[MAX_OIS_NAME_SIZE + 8];
	enum i2c_freq_mode freq;
	uint8_t v[4] = {0};
	uint32_t cur, pkg = 0;
	bool have_pkg = false;
	int rc, i;

	rc = k618_ultra_cci(o_ctrl, "RUMBAS4H");
	if (rc)
		return rc;
	cci = o_ctrl->io_master_info.cci_client;
	freq = cci->i2c_freq_mode;

	snprintf(name, sizeof(name), "%s.prog", o_ctrl->ois_name);
	if (request_firmware(&fw, name, &(o_ctrl->pdev->dev))) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] Failed to load %s", name);
		fw = NULL;
	} else if (fw->size < K618_RUMBAS4H_FW_SIZE) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s is %zu bytes, the image has %d",
			name, fw->size, K618_RUMBAS4H_FW_SIZE);
	} else {
		pkg = k618_le32(fw->data + K618_RUMBAS4H_VER_OFS);
		have_pkg = true;
	}

	for (i = 0; i < 3; i++) {
		rc = k618_ois_rd(o_ctrl, 0x00FC, v, 4);
		if (rc >= 0)
			break;
		msleep(1);
	}
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: version read failed %d times",
			o_ctrl->ois_name, i);
		rc = -EINVAL;
		goto out;
	}
	cur = k618_le32(v);
	CAM_INFO(CAM_OIS, "k618: [RUMBAS4H] %s: current firmware version = %u, packaged = %u%s",
		o_ctrl->ois_name, cur, pkg, have_pkg ? "" : " (no image)");
	if (have_pkg && cur != pkg) {
		if (!k618_ois_flash_update) {
			CAM_INFO(CAM_OIS,
				"k618: [RUMBAS4H] %s: versions differ, flash left as it is (camera.k618_ois_flash_update=0)",
				o_ctrl->ois_name);
		} else {
			cci->i2c_freq_mode = I2C_FAST_MODE;
			rc = k618_rumbas4h_flash(o_ctrl, fw->data);
			cci->i2c_freq_mode = freq;
			if (rc)
				goto out;
		}
	}

	rc = 0;
	if (o_ctrl->is_ois_calib) {
		rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_calib_data);
		if (rc)
			CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: Cannot apply calib data: %d",
				o_ctrl->ois_name, rc);
	}
	if (o_ctrl->i2c_init_data.is_settings_valid == 1) {
		rc = k618_rohm_apply_retry(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc)
			CAM_ERR(CAM_OIS, "k618: [RUMBAS4H] %s: Cannot apply Init settings: rc = %d",
				o_ctrl->ois_name, rc);
	}
out:
	release_firmware(fw);
	return rc;
}

/*
 * stock cam_sem1217s_ois_fw_download, inlined in cam_ois_fw_download (0x163274-0x163d58): with a
 * firmware already there, OIS and AF stopped unless idle (REG_OIS_STS 0x0001 / REG_OIS_CTRL
 * 0x0000, REG_AF_STS 0x0201 / REG_AF_CTRL 0x0200); REG_FWUP_CTRL 0x1000 = 7, 60 ms, 0x0001 must
 * read 2; 192 blocks of 256 bytes to REG_DATA_BUF 0x1100, 20 ms each; the 16-bit sum of the
 * image's little-endian halfwords to REG_FWUP_CHKSUM 0x1002, 200 ms; REG_FWUP_ERR 0x1001 must
 * read 0; REG_FWUP_CTRL = 0x80, 200 ms; REG_APP_VER 0x1008 must then read the packaged version
 */
static int k618_sem1217s_flash(struct cam_ois_ctrl_t *o_ctrl, const uint8_t *img, uint32_t cur,
	uint32_t pkg)
{
	uint8_t b[4] = {0};
	uint32_t sum = 0, ver, i, j;
	int rc;

	if (cur) {
		rc = k618_ois_rd(o_ctrl, 0x0001, b, 1);
		if (rc < 0)
			return rc;
		if (b[0] != 1) {
			b[0] = 0;
			rc = k618_ois_wr(o_ctrl, 0x0000, b, 1, CAM_SENSOR_I2C_WRITE_SEQ, 0);
			if (rc < 0)
				return rc;
		}
		rc = k618_ois_rd(o_ctrl, 0x0201, b, 1);
		if (rc < 0)
			return rc;
		if (b[0] != 1) {
			b[0] = 0;
			rc = k618_ois_wr(o_ctrl, 0x0200, b, 1, CAM_SENSOR_I2C_WRITE_SEQ, 0);
			if (rc < 0)
				return rc;
		}
	}
	b[0] = 0x07;
	rc = k618_ois_wr(o_ctrl, 0x1000, b, 1, CAM_SENSOR_I2C_WRITE_SEQ, 0);
	if (rc < 0)
		return rc;
	msleep(60);
	rc = k618_ois_rd(o_ctrl, 0x0001, b, 1);
	if (rc < 0)
		return rc;
	if (b[0] != 2) {
		CAM_ERR(CAM_OIS, "k618: [SEM1217S] %s: OIS firmware upgrade status check failed (0x%02x)",
			o_ctrl->ois_name, b[0]);
		return -EINVAL;
	}
	for (i = 0; i < K618_SEM1217S_FW_SIZE / 256; i++) {
		const uint8_t *blk = img + 256 * i;

		for (j = 0; j < 256; j += 2)
			sum += blk[j] | (blk[j + 1] << 8);
		rc = k618_ois_wr(o_ctrl, 0x1100, blk, 256, CAM_SENSOR_I2C_WRITE_SEQ, 0);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: [SEM1217S] %s: Failed to write REG_DATA_BUF, index = %u",
				o_ctrl->ois_name, i);
			return rc;
		}
		msleep(20);
	}
	b[0] = sum & 0xff;
	b[1] = (sum >> 8) & 0xff;
	rc = k618_ois_wr(o_ctrl, 0x1002, b, 2, CAM_SENSOR_I2C_WRITE_SEQ, 0);
	if (rc < 0)
		return rc;
	msleep(200);
	rc = k618_ois_rd(o_ctrl, 0x1002, b, 2);
	if (rc < 0)
		return rc;
	CAM_INFO(CAM_OIS, "k618: [SEM1217S] %s: REG_FWUP_CHKSUM written 0x%x, read 0x%x",
		o_ctrl->ois_name, sum & 0xffff, b[0] | (b[1] << 8));
	rc = k618_ois_rd(o_ctrl, 0x1001, b, 1);
	if (rc < 0)
		return rc;
	if (b[0]) {
		CAM_ERR(CAM_OIS, "k618: [SEM1217S] %s: Failed to update firmware, REG_FWUP_ERR = 0x%x",
			o_ctrl->ois_name, b[0]);
		return -EINVAL;
	}
	b[0] = 0x80;
	rc = k618_ois_wr(o_ctrl, 0x1000, b, 1, CAM_SENSOR_I2C_WRITE_SEQ, 0);
	if (rc < 0)
		return rc;
	msleep(200);
	rc = k618_ois_rd(o_ctrl, 0x1008, b, 4);
	if (rc < 0)
		return rc;
	ver = k618_le32(b);
	if (ver != pkg) {
		CAM_ERR(CAM_OIS, "k618: [SEM1217S] %s: firmware version %u after the update, wants %u",
			o_ctrl->ois_name, ver, pkg);
		return -EINVAL;
	}
	CAM_INFO(CAM_OIS, "k618: [SEM1217S] %s: Firmware update success, version %u",
		o_ctrl->ois_name, ver);
	return 0;
}

/*
 * stock cam_ois_fw_download, chip id 0x4c1 (0x162d84): stands in for the generic download. The
 * packaged version is the u32 at 0xbff4 of <ois_name>.prog (0xc000 bytes), the chip's REG_APP_VER
 * 0x1008; a failed version read fails the INIT as on stock. Stock's semco_ois_fw_update_op
 * (default 1) rewrites on any difference; here k618_ois_flash_update does, off by default.
 */
static int k618_sem1217s_fw(struct cam_ois_ctrl_t *o_ctrl)
{
	const struct firmware *fw = NULL;
	char name[MAX_OIS_NAME_SIZE + 8];
	uint8_t b[4] = {0};
	uint32_t cur, pkg = 0;
	bool have_pkg = false;
	int rc;

	snprintf(name, sizeof(name), "%s.prog", o_ctrl->ois_name);
	rc = request_firmware(&fw, name, &(o_ctrl->pdev->dev));
	if (rc) {
		CAM_ERR(CAM_OIS, "k618: [SEM1217S] Failed to load firmware: %s (%d)", name, rc);
		fw = NULL;
	} else if (fw->size < K618_SEM1217S_FW_SIZE) {
		CAM_ERR(CAM_OIS, "k618: [SEM1217S] %s is %zu bytes, the image has %d",
			name, fw->size, K618_SEM1217S_FW_SIZE);
	} else {
		pkg = k618_le32(fw->data + K618_SEM1217S_VER_OFS);
		have_pkg = true;
	}
	if (!have_pkg && k618_ois_flash_update) {
		rc = rc ? rc : -EINVAL;	/* stock: no image, the INIT fails */
		goto out;
	}

	rc = k618_ois_rd(o_ctrl, 0x1008, b, 4);
	if (rc < 0)
		goto out;
	cur = k618_le32(b);
	CAM_INFO(CAM_OIS, "k618: [SEM1217S] %s: current firmware version = %u, packaged = %u%s",
		o_ctrl->ois_name, cur, pkg, have_pkg ? "" : " (no image)");
	rc = 0;
	if (have_pkg && cur != pkg) {
		if (!k618_ois_flash_update)
			CAM_INFO(CAM_OIS,
				"k618: [SEM1217S] %s: versions differ, flash left as it is (camera.k618_ois_flash_update=0)",
				o_ctrl->ois_name);
		else
			rc = k618_sem1217s_flash(o_ctrl, fw->data, cur, pkg);
	}
out:
	release_firmware(fw);
	return rc;
}

/* stock pflash_ois (.rodata 0x895e0) as the compare chains 0x1603e4-0x1605c0 / 0x15fd00-0x160328 */
static int k618_ultra_chip_init(struct cam_ois_ctrl_t *o_ctrl)
{
	switch (o_ctrl->k618_ultra_type) {
	case 1:
	case 2:
	case 3:
	case 4:
	case 7:
		return k618_ultra_bu24721_init(o_ctrl);
	case 5:
		return k618_bu24532_init(o_ctrl);
	case 6:
		return k618_rumbas4h_init(o_ctrl);
	default:	/* stock logs "need pkt function" (0x160d6c) and returns 0 */
		CAM_ERR(CAM_OIS, "k618: OIS %s: type %u has no chip function",
			o_ctrl->ois_name, o_ctrl->k618_ultra_type);
		return 0;
	}
}

/* called from cam_ois_slaveInfo_pkt_parser() after camera 16's 59-byte fields */
static int k618_ultra_slave_info(struct cam_ois_ctrl_t *o_ctrl, const uint8_t *info, size_t len)
{
	const uint8_t *x = info + 0x38;

	o_ctrl->k618_ultra_info = false;
	o_ctrl->k618_ultra_type = 0;
	o_ctrl->k618_ultra_chip_id = 0;
	o_ctrl->k618_ultra_expect_id = 0;
	if (!o_ctrl->k618_ultra)
		return 0;
	if (len < K618_ULTRA_SLAVE_INFO_LEN) {
		CAM_ERR(CAM_OIS, "k618: OIS %s: slave info %zu bytes, the 13 Ultra's has %d",
			o_ctrl->ois_name, len, K618_ULTRA_SLAVE_INFO_LEN);
		return -EINVAL;
	}
	o_ctrl->k618_ultra_info = true;
	o_ctrl->k618_ois_flag = 0;	/* camera 16's ROHM branch is the 13 Pro's; the type is below */
	o_ctrl->k618_ultra_chip_id = x[2] | ((uint16_t)x[3] << 8);
	o_ctrl->k618_ultra_expect_id = k618_le32(x + 4);
	o_ctrl->k618_ultra_type = x[8];
	CAM_INFO(CAM_OIS,
		"k618: OIS %s (13 Ultra) slave info 0x38-0x40: %02x %02x %02x %02x %02x %02x %02x %02x %02x: address type %u, page step %u, chip id 0x%04x, expected id 0x%08x, type %u",
		o_ctrl->ois_name, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8],
		x[0], x[1], o_ctrl->k618_ultra_chip_id, o_ctrl->k618_ultra_expect_id,
		o_ctrl->k618_ultra_type);
	return 0;
}

/*
 * INIT, after power-up: type != 0 runs the chip function (stock 0x1603e4; a nonzero result is
 * "Failed OIS Customer Pkt Download", 0x1605cc). Type 0 goes on to the generic path; stock applies
 * fwinit there whether or not ois_fw_flag is set (0x160410; the download alone is under the flag,
 * 0x1604a8), 6.x only under the flag -- so with the flag clear it is applied here.
 */
static int k618_ultra_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc;

	if (o_ctrl->k618_ultra_type) {
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		rc = k618_ultra_chip_init(o_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: OIS %s: Failed OIS Customer Pkt Download: %d (type %u)",
				o_ctrl->ois_name, rc, o_ctrl->k618_ultra_type);
			return rc;
		}
		if (rc)	/* a poll at the end timed out: logged, as camera 16 does */
			CAM_WARN(CAM_OIS, "k618: OIS %s: type %u: a poll timed out (%d)",
				o_ctrl->ois_name, o_ctrl->k618_ultra_type, rc);
		return K618_ULTRA_INIT_DONE;
	}
	if (!o_ctrl->ois_fw_flag && o_ctrl->i2c_fwinit_data.is_settings_valid == 1) {
		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_fwinit_data);
		if ((rc == -EAGAIN) && (o_ctrl->io_master_info.master_type == CCI_MASTER)) {
			CAM_WARN(CAM_OIS, "k618: Reapplying fwinit settings");
			usleep_range(1000, 1010);
			rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_fwinit_data);
		}
		if (rc) {
			CAM_ERR(CAM_OIS, "k618: OIS %s: Cannot apply fwinit data %d",
				o_ctrl->ois_name, rc);
			return rc < 0 ? rc : -EINVAL;
		}
	}
	return 0;
}

/* the generic path's download: a SEM1217S (chip id 0x4c1) gets its own, as stock 0x162d84 */
static int k618_ultra_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl->k618_ultra_info && o_ctrl->k618_ultra_chip_id == K618_SEM1217S_CHIP_ID)
		return k618_sem1217s_fw(o_ctrl);
	return cam_ois_fw_download(o_ctrl);
}

/* stock cam_ois_publish_dev_info (0x1613b8) */
static int k618_ois_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	if (!info) {
		CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Invalid Args");
		return -EINVAL;
	}
	info->dev_id = CAM_REQ_MGR_DEVICE_OIS;
	strscpy(info->name, "cam-ois", sizeof(info->name));
	info->p_delay = CAM_PIPELINE_DELAY_1;
	info->m_delay = CAM_MODESWITCH_DELAY_1;	/* 6.x field (not in 5.15); as actuator and flash */
	info->trigger = CAM_TRIGGER_POINT_SOF;
	return 0;
}

/* stock cam_ois_establish_link (0x161444) */
static int k618_ois_establish_link(struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_ois_ctrl_t *o_ctrl;

	if (!link) {
		CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Invalid Args");
		return -EINVAL;
	}
	o_ctrl = (struct cam_ois_ctrl_t *)cam_get_device_priv(link->dev_hdl);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Device data is NULL");
		return -EINVAL;
	}
	mutex_lock(&(o_ctrl->ois_mutex));
	if (link->link_enable) {
		o_ctrl->bridge_intf.link_hdl = link->link_hdl;
		o_ctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&(o_ctrl->ois_mutex));
	return 0;
}

/*
 * stock cam_ois_apply_request (0x161974): the slot of this request id is applied and marked
 * spent; a request in the skip table returns -EAGAIN while its count lasts, counting down (6.x's
 * request manager does not count -EAGAIN as a failed retry); otherwise the slot two requests back
 * is cleared.
 */
static int k618_ois_apply_request(struct cam_req_mgr_apply_request *apply)
{
	struct cam_ois_ctrl_t *o_ctrl;
	struct i2c_settings_array *slot;
	uint32_t n, idx, del;
	int rc = 0;

	if (!apply) {
		CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Invalid Input Args");
		return -EINVAL;
	}
	o_ctrl = (struct cam_ois_ctrl_t *)cam_get_device_priv(apply->dev_hdl);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Device data is NULL");
		return -EINVAL;
	}
	n = ARRAY_SIZE(o_ctrl->k618_req_slot);
	idx = apply->request_id % n;
	CAM_DBG(CAM_OIS, "k618: [CRM-OIS] OIS apply request:%llu", apply->request_id);

	mutex_lock(&(o_ctrl->ois_mutex));
	slot = &(o_ctrl->k618_req_slot[idx]);
	if ((slot->request_id == apply->request_id) && (slot->is_settings_valid == 1)) {
		o_ctrl->k618_poll_go_on = 0;	/* per-frame settings fail fast */
		rc = cam_ois_apply_settings(o_ctrl, slot);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Failed in applying the request: %llu.",
				apply->request_id);
			goto unlock;
		}
		slot->is_settings_valid = 0;
		CAM_DBG(CAM_OIS, "k618: Apply setting at req %llu, rc = %d", apply->request_id, rc);
	}

	if ((o_ctrl->k618_skip_req[idx] == apply->request_id) && o_ctrl->k618_skip_cnt[idx]) {
		CAM_INFO(CAM_OIS, "k618: [CRM-OIS] Skipframe number from %llu to %llu at request:%llu",
			o_ctrl->k618_skip_cnt[idx], o_ctrl->k618_skip_cnt[idx] - 1,
			apply->request_id);
		if (!--o_ctrl->k618_skip_cnt[idx])
			o_ctrl->k618_skip_req[idx] = 0;
		rc = -EAGAIN;
		goto unlock;
	}

	del = (idx + n - 2) % n;
	slot = &(o_ctrl->k618_req_slot[del]);
	if (apply->request_id > slot->request_id) {
		slot->request_id = 0;
		rc = delete_request(slot);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Fail deleting the req: %u err: %d.", del, rc);
	} else {
		CAM_DBG(CAM_OIS, "k618: [CRM-OIS] No Valid Req to clean Up");
	}
unlock:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}

/* stock cam_ois_update_req_mgr (0x16152c); trigger_eof for packet 4 (0x16156c) */
static int k618_ois_add_req(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet)
{
	struct cam_req_mgr_add_request add_req = {0};
	int rc;

	add_req.link_hdl = o_ctrl->bridge_intf.link_hdl;
	add_req.dev_hdl = o_ctrl->bridge_intf.device_hdl;
	add_req.req_id = csl_packet->header.request_id;
	if ((csl_packet->header.op_code & 0xFFFFFF) == K618_OIS_OPCODE_APERTURE)
		add_req.trigger_eof = true;
	if (!o_ctrl->bridge_intf.crm_cb || !o_ctrl->bridge_intf.crm_cb->add_req) {
		CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Can't add Request ID: %llu to CRM",
			add_req.req_id);
		return -EINVAL;
	}
	rc = o_ctrl->bridge_intf.crm_cb->add_req(&add_req);
	if (rc) {
		if (rc == -EBADR)
			CAM_INFO(CAM_OIS,
				"k618: [CRM-OIS] Adding request: %llu failed: rc: %d, it has been flushed",
				add_req.req_id, rc);
		else
			CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Adding request: %llu failed: rc: %d",
				add_req.req_id, rc);
		return rc;
	}
	CAM_DBG(CAM_OIS, "k618: [CRM-OIS] Request Id: %llu added to CRM", add_req.req_id);
	return 0;
}

/*
 * stock packet 0x7e (0x15fffc-0x160a68): the first buffer holds u64 request id, u64 count; the
 * table keeps the count less one (0x160a44), apply_request holds that request so many frames
 */
static int k618_ois_skip_frame(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet,
	struct cam_cmd_buf_desc *cmd_desc)
{
	const uint64_t *p;
	uintptr_t cpu = 0;
	size_t len = 0;
	uint32_t idx;
	int rc;

	CAM_DBG(CAM_OIS, "k618: [SkipFrame] num_cmd_buf %u", csl_packet->num_cmd_buf);
	if (!csl_packet->num_cmd_buf) {
		CAM_ERR(CAM_OIS, "k618: [SkipFrame] Recvied skip frame packet is empty.");
		return -EINVAL;
	}
	if (cmd_desc->length < 2 * sizeof(uint64_t)) {
		CAM_ERR(CAM_OIS, "k618: [SkipFrame] Insufficient number of packet bytes received: %u.",
			cmd_desc->length);
		return -EINVAL;
	}
	rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle, &cpu, &len);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "k618: [SkipFrame] Failed to get cpu buf: 0x%x", cmd_desc->mem_handle);
		return rc;
	}
	if (!cpu || len < 2 * sizeof(uint64_t) || len - 2 * sizeof(uint64_t) < cmd_desc->offset) {
		CAM_ERR(CAM_OIS, "k618: [SkipFrame] Invalid length for skip frame packet");
		rc = -EINVAL;
		goto put;
	}
	p = (const uint64_t *)(cpu + (cmd_desc->offset & ~3U));
	if (p[1]) {
		idx = p[0] % ARRAY_SIZE(o_ctrl->k618_skip_req);
		o_ctrl->k618_skip_req[idx] = p[0];
		o_ctrl->k618_skip_cnt[idx] = p[1] - 1;
		CAM_INFO(CAM_OIS, "k618: [SkipFrame] skip reqId: %llu, skip num: %llu queue index:%u",
			o_ctrl->k618_skip_req[idx], o_ctrl->k618_skip_cnt[idx], idx);
	}
put:
	cam_mem_put_cpu_buf(cmd_desc->mem_handle);
	return rc;
}

/*
 * stock packet 5 (0x15f510-0x15f6a4, 0x15fcfc, 0x160d8c): its buffers fill calib (when
 * is_ois_calib) and then postinit; with o_ctrl+0x11d8 set (k618_ultra_redl) the type's chip
 * function runs again; the two buffers are deleted. No state check, as stock.
 */
static int k618_ultra_redownload(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet,
	struct cam_cmd_buf_desc *cmd_desc)
{
	struct i2c_settings_array *set;
	uintptr_t ptr = 0;
	size_t len = 0;
	uint32_t i;
	int rc = 0;

	CAM_DBG(CAM_OIS, "k618: OIS %s: num_cmd_buf %u", o_ctrl->ois_name, csl_packet->num_cmd_buf);
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		if (!cmd_desc[i].length)
			continue;
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc) {
			CAM_ERR(CAM_OIS, "k618: OIS %s: invalid cmd desc %u", o_ctrl->ois_name, i);
			goto out;
		}
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle, &ptr, &len);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: OIS %s: Failed to get cpu buf: 0x%x",
				o_ctrl->ois_name, cmd_desc[i].mem_handle);
			goto out;
		}
		cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
		if (!ptr || len < 8 || len - 8 < cmd_desc[i].offset) {	/* stock 0x15f6d0 */
			CAM_ERR(CAM_OIS, "k618: OIS %s: invalid length of cmd buf %u",
				o_ctrl->ois_name, i);
			rc = -EINVAL;
			goto out;
		}
		if (o_ctrl->is_ois_calib && (o_ctrl->i2c_calib_data.is_settings_valid == 0))
			set = &(o_ctrl->i2c_calib_data);
		else if (o_ctrl->k618_postinit_data.is_settings_valid == 0)
			set = &(o_ctrl->k618_postinit_data);
		else
			continue;
		set->is_settings_valid = 1;
		set->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&(o_ctrl->io_master_info), set, &cmd_desc[i], 1,
			NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: OIS %s: re-download settings parsing failed: %d",
				o_ctrl->ois_name, rc);
			goto out;
		}
	}
	if (o_ctrl->k618_ultra_info && o_ctrl->k618_ultra_type) {
		o_ctrl->k618_ultra_redl = true;
		rc = k618_ultra_chip_init(o_ctrl);
		o_ctrl->k618_ultra_redl = false;
		if (rc < 0)
			CAM_ERR(CAM_OIS, "k618: OIS %s: Failed OIS Customer Pkt Download: %d",
				o_ctrl->ois_name, rc);
		else
			rc = 0;
	}
out:
	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&(o_ctrl->i2c_calib_data));
	if (o_ctrl->k618_postinit_data.is_settings_valid == 1)
		delete_request(&(o_ctrl->k618_postinit_data));
	return rc;
}

/* cam_ois_pkt_parse(): stock packets 4, 5, 6, 0x7e and 0x7f (jump table 0x15f308, 0x15f3a8) */
static int k618_ultra_pkt(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet)
{
	uint32_t op = csl_packet->header.op_code & 0xFFFFFF;
	uint32_t *offset = (uint32_t *)&csl_packet->payload_flex;
	struct cam_cmd_buf_desc *cmd_desc;
	struct i2c_settings_array *slot;
	uint64_t req = csl_packet->header.request_id;
	int rc;

	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	switch (op) {
	case K618_OIS_OPCODE_APERTURE:	/* 0x15f4c0, 0x15ff64 */
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			CAM_WARN(CAM_OIS, "k618: [CRM-OIS] Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return -EINVAL;
		}
		CAM_DBG(CAM_OIS, "k618: [CRM-OIS] Received aperture mode packets");
		slot = &(o_ctrl->k618_req_slot[req % ARRAY_SIZE(o_ctrl->k618_req_slot)]);
		/* a slot left from a request 32 back is emptied first (stock appends to it) */
		if (slot->request_id != req && !list_empty(&(slot->list_head)))
			delete_request(slot);
		slot->request_id = req;
		slot->is_settings_valid = 1;
		rc = cam_sensor_i2c_command_parser(&(o_ctrl->io_master_info), slot, cmd_desc, 1,
			NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: [CRM-OIS] CRM OIS parsing failed: %d", rc);
			return rc;
		}
		rc = k618_ois_add_req(o_ctrl, csl_packet);
		if (rc)
			CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Failed in adding request to request manager");
		return rc;
	case CAM_PKT_NOP_OPCODE:	/* 0x15f3b0, 0x15fd30 */
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			CAM_DBG(CAM_OIS, "k618: [CRM-OIS] Received NOP packets in invalid state: %d",
				o_ctrl->cam_ois_state);
			return -EINVAL;
		}
		rc = k618_ois_add_req(o_ctrl, csl_packet);
		if (rc)
			CAM_ERR(CAM_OIS, "k618: [CRM-OIS] Failed in adding request to request manager");
		return rc;
	case K618_OIS_OPCODE_SKIPFRAME:	/* 0x15f604 */
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			CAM_WARN(CAM_OIS,
				"k618: [SkipFrame] Recvied skip frame packet in invalid state: %d",
				o_ctrl->cam_ois_state);
			return -EINVAL;
		}
		return k618_ois_skip_frame(o_ctrl, csl_packet, cmd_desc);
	case K618_OIS_OPCODE_REDOWNLOAD:	/* 0x15f510 */
		return k618_ultra_redownload(o_ctrl, csl_packet, cmd_desc);
	case K618_OIS_OPCODE_PARKLENS:	/* 0x15f560 */
		/*
		 * Stock parses the packet's park settings into o_ctrl+0xf88 and starts
		 * "parklens-thread", which walks them after release and then powers the OIS
		 * down; release skips its own power-down while the thread runs (0x15ee6c). Not
		 * ported: the packet is accepted, nothing is applied, and release powers the OIS
		 * down at once, as 6.x does and as stock does when no park thread was started.
		 */
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG)
			return -EINVAL;
		CAM_DBG(CAM_OIS, "k618: OIS %s: park-lens packet (not ported) for request %llu",
			o_ctrl->ois_name, req);
		return 0;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d", op);
		return -EINVAL;
	}
}

/*
 * stock 0x15f280-0x15f2f4: a packet other than INIT for a request at or below the last flushed
 * one is refused, -EBADR. Only flush_req sets that id (0x161768), and stock registers flush_req
 * for the I2C variant only (0x1583c4) -- these OIS are platform devices -- so on both phones the
 * id stays 0 and nothing is refused.
 */
static int k618_ultra_flush_check(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet)
{
	uint64_t req = csl_packet->header.request_id;

	if ((csl_packet->header.op_code & 0xFFFFFF) && o_ctrl->k618_last_flush_req &&
		(req <= o_ctrl->k618_last_flush_req)) {
		CAM_INFO(CAM_OIS, "k618: reject request %llu, last request to flush %llu",
			req, o_ctrl->k618_last_flush_req);
		return -EBADR;
	}
	if (req > o_ctrl->k618_last_flush_req)
		o_ctrl->k618_last_flush_req = 0;
	return 0;
}

/*
 * stock cam_sensor_i2c_read_write_ois_data (0x173ac8), which the 13 Ultra's OIS read packet uses
 * (0x15fe74) in place of cam_sensor_i2c_read_data: a random write in the list is sent where it
 * stands (6.x stops at it, -EINVAL: it has no read buffer), and a read lands after the bytes of
 * the read before it in the shared output buffer (6.x puts every read at the buffer's start).
 * The bound check against the buffer's end, offset included, is added here.
 */
static int k618_ultra_read_data(struct cam_ois_ctrl_t *o_ctrl, struct i2c_settings_array *set)
{
	struct i2c_settings_list *i2c_list;
	uint32_t prev = 0, len, cnt;
	uint8_t *buf;
	int rc = 0;

	if (!o_ctrl->k618_ultra)
		return cam_sensor_i2c_read_data(set, &(o_ctrl->io_master_info));

	list_for_each_entry(i2c_list, &(set->list_head), list) {
		struct cam_sensor_i2c_reg_setting *s = &(i2c_list->i2c_settings);

		if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info), s);
			if (rc) {
				CAM_ERR(CAM_OIS, "k618: OIS %s: write failed rc %d", o_ctrl->ois_name, rc);
				return rc;
			}
			continue;
		}
		if (!s->read_buff || !s->read_buff_len) {
			CAM_ERR(CAM_OIS, "k618: Invalid input buffer, buffer: %pK, length: %d",
				s->read_buff, s->read_buff_len);
			return -EINVAL;
		}
		if (i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ) {
			len = s->size;
			if (len > s->read_buff_len || prev > s->read_buff_len - len) {
				CAM_ERR(CAM_OIS, "k618: Invalid size, readLen:%u, offset:%u, bufLen:%u",
					len, prev, s->read_buff_len);
				return -EINVAL;
			}
			rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info),
				s->reg_setting[0].reg_addr, s->read_buff + prev, s->addr_type,
				s->data_type, len);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "k618: failed: seq read I2C settings: %d", rc);
				return rc;
			}
			prev = len;
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_READ_RANDOM) {
			len = s->data_type * s->size;
			if (len > s->read_buff_len || len < s->size || prev > s->read_buff_len - len) {
				CAM_ERR(CAM_OIS,
					"k618: Invalid size, readLen:%u, offset:%u, bufLen:%u, size: %u",
					len, prev, s->read_buff_len, s->size);
				return -EINVAL;
			}
			buf = s->read_buff + prev;
			for (cnt = 0; cnt < s->size; cnt++) {
				rc = camera_io_dev_read(&(o_ctrl->io_master_info),
					s->reg_setting[cnt].reg_addr, &(s->reg_setting[cnt].reg_data),
					s->addr_type, s->data_type, false);
				if (rc < 0) {
					CAM_ERR(CAM_OIS, "k618: Failed: random read I2C settings: %d", rc);
					return rc;
				}
				if (s->data_type <= CAMERA_SENSOR_I2C_TYPE_DWORD) {
					memcpy(buf, &(s->reg_setting[cnt].reg_data), s->data_type);
					buf += s->data_type;
				}
			}
			prev = len;
		}
	}
	return rc;
}

/* stock CAM_START_DEV (0x15eca8) */
static void k618_ultra_start(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl->k618_ultra)
		o_ctrl->k618_last_flush_req = 0;
}

/* stock CAM_STOP_DEV (0x15ed68-0x15ed80) */
static void k618_ultra_stop(struct cam_ois_ctrl_t *o_ctrl)
{
	if (!o_ctrl->k618_ultra)
		return;
	o_ctrl->k618_last_flush_req = 0;
	memset(o_ctrl->k618_skip_req, 0, sizeof(o_ctrl->k618_skip_req));
	memset(o_ctrl->k618_skip_cnt, 0, sizeof(o_ctrl->k618_skip_cnt));
}

/*
 * stock CAM_RELEASE_DEV (0x15eec0-0x15efb8): skip table and flush id cleared, the request slots
 * deleted. Stock deletes the slots still marked valid; here also a slot already applied whose
 * list apply_request had not cleared yet. Called from cam_ois_shutdown() as well.
 */
static void k618_ultra_release(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t i;

	if (!o_ctrl->k618_ultra)
		return;
	o_ctrl->k618_last_flush_req = 0;
	memset(o_ctrl->k618_skip_req, 0, sizeof(o_ctrl->k618_skip_req));
	memset(o_ctrl->k618_skip_cnt, 0, sizeof(o_ctrl->k618_skip_cnt));
	for (i = 0; i < ARRAY_SIZE(o_ctrl->k618_req_slot); i++) {
		if ((o_ctrl->k618_req_slot[i].is_settings_valid == 1) ||
			!list_empty(&(o_ctrl->k618_req_slot[i].list_head)))
			delete_request(&(o_ctrl->k618_req_slot[i]));
		o_ctrl->k618_req_slot[i].request_id = 0;
	}
}

/*
 * The live device tree's root "model": "Qualcomm Technologies, Inc. 8550 nuwa" on the 13 Pro,
 * "... 8550 ishtar" in the 13 Ultra's dtbo overlay; compatible and qcom,board-id are the same.
 */
static bool k618_ois_is_ishtar(void)
{
	struct device_node *root = of_find_node_by_path("/");
	const char *model = NULL;
	bool ishtar = false;

	if (!root)
		return false;
	if (!of_property_read_string(root, "model", &model) && model)
		ishtar = strstr(model, "ishtar") != NULL;
	of_node_put(root);
	return ishtar;
}

/* from cam_ois_driver_soc_init(): stock cam_ois_component_bind 0x157970-0x157bdc */
void k618_ois_ultra_setup(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(o_ctrl->k618_req_slot); i++)
		INIT_LIST_HEAD(&(o_ctrl->k618_req_slot[i].list_head));
	o_ctrl->k618_ultra = k618_ois_is_ishtar();
	if (!o_ctrl->k618_ultra)
		return;
	o_ctrl->bridge_intf.link_hdl = -1;
	o_ctrl->bridge_intf.ops.get_dev_info = k618_ois_publish_dev_info;
	o_ctrl->bridge_intf.ops.link_setup = k618_ois_establish_link;
	o_ctrl->bridge_intf.ops.apply_req = k618_ois_apply_request;
	CAM_INFO(CAM_OIS, "k618: OIS %u: 13 Ultra -- request-manager device, 65-byte slave info",
		o_ctrl->soc_info.index);
}


/*
 * k618 (camera 24, park lens): the 13 Ultra's OIS park-lens thread, rebuilt from its 5.15
 * camera.ko -- packet 6 (cam_ois_driver_cmd 0x15f560, 0x16110c), init_ois_parklens_info 0x15d3cc,
 * ois_deinit_parklens_info 0x15d528, ois_parklens_power_down 0x15d728,
 * ois_parklens_thread_trigger 0x15d8dc, the thread 0x15db30, ois_parklens_thread_stop 0x15e35c,
 * cam_ois_i2c_modes_util 0x162464. Packet 6 carries settings that move the lens to rest step by
 * step; a kernel thread applies them, each entry's delay as a countdown of 1 ms sleeps, and then
 * powers the OIS down itself, so RELEASE can return while the lens is still moving.
 *  k618_pl_state    (+0x1240) 0 none; 1 set up, no thread (not started, or ended and collected);
 *                             2 thread started (it may have ended on its own since)
 *  k618_pl_step     (+0x123c) 0 while the thread walks; set once as it ends: 2 stopped, power kept;
 *                             3 done, or stopped with power-down; 4 settings invalid; 5 a step
 *                             failed (3-5 power the OIS down); 6 that power-down failed
 *  k618_pl_cmd      (+0x1238) 0; 2 stop and power down; 3 stop and keep the power
 *  k618_pl_start    (+0x11e8) completed by the thread as it starts, waited for by the trigger
 *  k618_pl_exit     (+0x1210) completed by the thread as it ends, waited for by the trigger (when
 *                             the thread ended at once) and by a stop
 *  k618_pl_settings (+0xf88)  packet 6's settings: the thread walks them, deletes them at its end
 *  k618_pl_list     (+0x1248) the thread's copy of the last two entries, made as it starts, applied
 *                             by a new INIT that stops the thread with the OIS still powered, freed
 *                             by deinit
 * The thread takes no mutex; trigger, stop and deinit run under ois_mutex (driver_cmd, shutdown).
 * The power-down belongs to the thread from packet 6 on: RELEASE skips its own and leaves the power
 * settings to the thread, which frees them after powering down. A new INIT stops the thread keeping
 * the power; if the thread had not powered down yet, the INIT applies the copied entries and goes
 * on without reading the power settings again or powering up; otherwise it powers up as usual.
 * Where this differs from stock:
 *  - the thread powers down with cam_sensor_util_power_down() and camera_io_release(), the body of
 *    stock's cam_ois_power_down(); 6.x's cam_ois_power_down() also sets cam_ois_state, which the
 *    thread may not touch outside ois_mutex
 *  - shutdown (subdev close, CAM_SD_SHUTDOWN, unbind) first stops the thread with power-down and
 *    waits for it, and powers down itself only when the thread did not; stock powers down and
 *    frees the power settings while the thread may still be running, and its deinit refuses a
 *    running thread
 *  - a new INIT stops the thread as the INIT starts, not at its power-settings buffer; if the INIT
 *    then fails before its power-up decision with the OIS still powered from the park, the OIS is
 *    powered down at the INIT's exit; if the thread had powered down while cam_ois_state still
 *    reads CONFIG (a second INIT in one session), cam_ois_state goes back to ACQUIRE so that this
 *    INIT powers up
 *  - packet 6 in the wrong state stops the thread with power-down, as stock; when that power-down
 *    worked, cam_ois_state goes back to ACQUIRE so that RELEASE does not power down again
 *  - RELEASE leaves the power settings to the thread whenever one may still use them, in any state
 *    (6.x frees them always, stock only after its own power-down)
 *  - a stop command other than 2 or 3 ends the thread as 2 (stock loops on "Invalid opcode for
 *    parklens"); an empty settings list ends it as done (stock reads the list head as an entry)
 *  - the copy takes the last two entries that exist, all of their registers (stock reads past the
 *    head with fewer than two entries, and copies one register while keeping the size)
 *  - poll timeouts in the thread are capped at K618_PARK_POLL_MAX_MS, so that a stop is reached in
 *    bounded time
 */
#define K618_PARK_STOP_POWER_DOWN	2
#define K618_PARK_STOP_KEEP_POWER	3
#define K618_PARK_POLL_MAX_MS		1000

/* from cam_ois_driver_soc_init(), both phones: stock cam_ois_component_bind 0x157c14-0x157c34 */
void k618_ois_park_setup(struct cam_ois_ctrl_t *o_ctrl)
{
	INIT_LIST_HEAD(&(o_ctrl->k618_pl_settings.list_head));
	INIT_LIST_HEAD(&(o_ctrl->k618_pl_list.list_head));
	init_completion(&o_ctrl->k618_pl_start);
	init_completion(&o_ctrl->k618_pl_exit);
	atomic_set(&o_ctrl->k618_pl_cmd, 0);
	atomic_set(&o_ctrl->k618_pl_step, 0);
	atomic_set(&o_ctrl->k618_pl_state, 0);
	o_ctrl->k618_pl_task = NULL;
	o_ctrl->k618_pl_keep_power = false;
	o_ctrl->k618_pl_powered = false;
}

/* stock cam_ois_i2c_modes_util (0x162464): random write, sequential write, polls; others ignored */
static int k618_park_modes_util(struct camera_io_master *io, struct i2c_settings_list *l)
{
	struct cam_sensor_i2c_reg_setting *s = &(l->i2c_settings);
	uint32_t i;
	int rc = 0;

	switch (l->op_code) {
	case CAM_SENSOR_I2C_WRITE_RANDOM:
		rc = camera_io_dev_write(io, s);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "k618: park: Failed in Applying i2c wrt settings: %d", rc);
		return rc;
	case CAM_SENSOR_I2C_WRITE_SEQ:
		rc = camera_io_dev_write_continuous(io, s, CAM_SENSOR_I2C_WRITE_SEQ);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "k618: park: Failed to seq write I2C settings: %d", rc);
		return rc;
	case CAM_SENSOR_I2C_POLL:
		for (i = 0; i < s->size; i++) {
			rc = camera_io_dev_poll(io, s->reg_setting[i].reg_addr,
				s->reg_setting[i].reg_data, s->reg_setting[i].data_mask,
				s->addr_type, s->data_type,
				min_t(uint32_t, s->reg_setting[i].delay, K618_PARK_POLL_MAX_MS));
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "k618: park: i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
		return rc;	/* a poll that timed out (1) is no failure, as stock */
	default:
		return 0;
	}
}

/* one entry with all of its registers (stock 0x15db94-0x15dccc copies the first register only) */
static struct i2c_settings_list *k618_park_copy(const struct i2c_settings_list *src)
{
	struct i2c_settings_list *dst;
	uint32_t n = src->i2c_settings.size;

	dst = CAM_MEM_ZALLOC(sizeof(*dst), GFP_KERNEL);
	if (!dst)
		return NULL;
	*dst = *src;
	dst->i2c_settings.reg_setting = NULL;
	if (n && src->i2c_settings.reg_setting) {
		dst->i2c_settings.reg_setting = CAM_MEM_ZALLOC(
			n * sizeof(struct cam_sensor_i2c_reg_array), GFP_KERNEL);
		if (!dst->i2c_settings.reg_setting) {
			CAM_MEM_FREE(dst);
			return NULL;
		}
		memcpy(dst->i2c_settings.reg_setting, src->i2c_settings.reg_setting,
			n * sizeof(struct cam_sensor_i2c_reg_array));
	} else {
		dst->i2c_settings.size = 0;
	}
	INIT_LIST_HEAD(&(dst->list));
	return dst;
}

/* stock cam_ois_power_down (0x15e79c), which leaves cam_ois_state as it is */
static int k618_park_power_off(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	int rc;

	rc = cam_sensor_util_power_down(&(soc_private->power_info), &(o_ctrl->soc_info));
	if (rc) {
		CAM_ERR(CAM_OIS, "k618: park: power down the core is failed:%d", rc);
		return rc < 0 ? rc : -EIO;
	}
	camera_io_release(&(o_ctrl->io_master_info));
	return 0;
}

/* stock thread 0x15e078-0x15e094: what RELEASE left to the thread */
static void k618_park_free_power_settings(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &(soc_private->power_info);

	CAM_MEM_FREE(power_info->power_setting);
	CAM_MEM_FREE(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;
}

/*
 * stock "parklens-thread" (0x15db30): tells the trigger it runs; copies the last two entries;
 * then per pass reads the stop command (2: power down, step 3; 3: keep the power, step 2), checks
 * the settings are valid (else step 4), sleeps the current entry's delay away 1 ms at a time
 * (990-1010 us, the command checked between), and applies the next entry with that entry's delay
 * zeroed (a failure: step 5); after the last entry (and its delay) step 3. Steps 3-5 power the OIS
 * down (failing that: step 6) and free the power settings. Last, the settings are deleted and
 * k618_pl_exit completed.
 */
static int k618_park_thread(void *data)
{
	struct cam_ois_ctrl_t *o_ctrl = data;
	struct i2c_settings_array *set = &(o_ctrl->k618_pl_settings);
	struct i2c_settings_list *cur = NULL, *copy;
	struct list_head *pos;
	uint32_t delay = 0, steps = 0;
	int cmd, rc, step = 3, i;

	complete(&o_ctrl->k618_pl_start);
	CAM_INFO(CAM_OIS, "k618: OIS %s: parklens thread start up", o_ctrl->ois_name);

	o_ctrl->k618_pl_list.is_settings_valid = 1;
	o_ctrl->k618_pl_list.request_id = 0;
	pos = set->list_head.prev;
	for (i = 0; (i < 2) && (pos != &(set->list_head)); i++, pos = pos->prev) {
		copy = k618_park_copy(list_entry(pos, struct i2c_settings_list, list));
		if (copy)
			list_add(&(copy->list), &(o_ctrl->k618_pl_list.list_head));
	}

	for (;;) {
		cmd = atomic_read(&o_ctrl->k618_pl_cmd);
		if (cmd == K618_PARK_STOP_KEEP_POWER) {
			CAM_DBG(CAM_OIS, "k618: exit parklens thread no need power off");
			atomic_set(&o_ctrl->k618_pl_step, 2);
			goto out;
		}
		if (cmd >= 2) {
			if (cmd != K618_PARK_STOP_POWER_DOWN)
				CAM_ERR(CAM_OIS, "k618: Invalid opcode for parklens: %d, stopping with power down",
					cmd);
			step = 3;
			goto power_off;
		}
		if (set->is_settings_valid != 1) {
			CAM_ERR(CAM_OIS, "k618: parklens settings is not valid");
			step = 4;
			goto power_off;
		}
		if (delay) {
			delay--;
			usleep_range(990, 1010);
			continue;
		}
		pos = cur ? cur->list.next : set->list_head.next;
		if (pos == &(set->list_head)) {
			CAM_DBG(CAM_OIS, "k618: parklens settings execute done");
			step = 3;
			goto power_off;
		}
		cur = list_entry(pos, struct i2c_settings_list, list);
		delay = cur->i2c_settings.delay;
		cur->i2c_settings.delay = 0;
		CAM_DBG(CAM_OIS, "k618: Parklens total delay time %ums", delay);
		rc = k618_park_modes_util(&(o_ctrl->io_master_info), cur);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "k618: parklens step failed, %d", rc);
			step = 5;
			goto power_off;
		}
		steps++;
		CAM_DBG(CAM_OIS, "k618: ois_parklens_thread_func arg step/state %u/%d", steps,
			atomic_read(&o_ctrl->k618_pl_state));
	}

power_off:
	atomic_set(&o_ctrl->k618_pl_step, step);
	rc = k618_park_power_off(o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "k618: parklens power down failed rc: %d", rc);
		atomic_set(&o_ctrl->k618_pl_step, 6);
	} else {
		CAM_DBG(CAM_OIS, "k618: parklens thread power down success");
	}
	k618_park_free_power_settings(o_ctrl);
out:
	CAM_INFO(CAM_OIS, "k618: OIS %s: parklens thread exit, %u steps, step/result %d",
		o_ctrl->ois_name, steps, atomic_read(&o_ctrl->k618_pl_step));
	rc = delete_request(set);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "k618: delete parklens request failed rc: %d", rc);
	kthread_complete_and_exit(&o_ctrl->k618_pl_exit, 0);
}

/* stock ois_deinit_parklens_info (0x15d528): only with no thread running (state 1) */
static int k618_park_deinit(struct cam_ois_ctrl_t *o_ctrl)
{
	if (atomic_read(&o_ctrl->k618_pl_state) != 1) {
		CAM_DBG(CAM_OIS, "k618: deinit parklens in wrong state: %d",
			atomic_read(&o_ctrl->k618_pl_state));
		return -EINVAL;
	}
	delete_request(&(o_ctrl->k618_pl_list));
	complete_all(&o_ctrl->k618_pl_start);
	complete_all(&o_ctrl->k618_pl_exit);
	atomic_set(&o_ctrl->k618_pl_cmd, 0);
	atomic_set(&o_ctrl->k618_pl_step, 0);
	atomic_set(&o_ctrl->k618_pl_state, 0);
	o_ctrl->k618_pl_task = NULL;
	return 0;
}

/*
 * stock init_ois_parklens_info (0x15d3cc) and ois_parklens_thread_trigger (0x15d8dc): state 1,
 * start the thread, wait for it to run; a thread that has not ended by then makes state 2, one
 * that has is collected (state 1) and its step returned
 */
static int k618_park_trigger(struct cam_ois_ctrl_t *o_ctrl)
{
	struct task_struct *task;
	int step;

	delete_request(&(o_ctrl->k618_pl_list));
	init_completion(&o_ctrl->k618_pl_start);
	init_completion(&o_ctrl->k618_pl_exit);
	atomic_set(&o_ctrl->k618_pl_cmd, 0);
	atomic_set(&o_ctrl->k618_pl_step, 0);
	o_ctrl->k618_pl_task = NULL;
	atomic_set(&o_ctrl->k618_pl_state, 1);

	task = kthread_run(k618_park_thread, o_ctrl, "parklens-thread");
	if (IS_ERR(task)) {
		CAM_ERR(CAM_OIS, "k618: parklens_thread create failed: %ld", PTR_ERR(task));
		delete_request(&(o_ctrl->k618_pl_settings));
		k618_park_deinit(o_ctrl);
		return -ENOMEM;
	}
	o_ctrl->k618_pl_task = task;
	CAM_INFO(CAM_OIS, "k618: OIS %s: trigger parklens thread", o_ctrl->ois_name);
	wait_for_completion(&o_ctrl->k618_pl_start);
	step = atomic_read(&o_ctrl->k618_pl_step);
	if (step < 1) {
		atomic_set(&o_ctrl->k618_pl_state, 2);
		return step;
	}
	CAM_DBG(CAM_OIS, "k618: parklens thread execute failed %d", step);
	wait_for_completion(&o_ctrl->k618_pl_exit);
	step = atomic_read(&o_ctrl->k618_pl_step);
	CAM_DBG(CAM_OIS, "k618: parklens thread exit: %d", step);
	atomic_set(&o_ctrl->k618_pl_state, 1);
	o_ctrl->k618_pl_task = NULL;
	return step;
}

/*
 * stock ois_parklens_thread_stop (0x15e35c): only with state 2; a thread still walking gets the
 * command (anything but 2 or 3 becomes 2); then wait for its end, state 1, return its step
 */
static int k618_park_stop(struct cam_ois_ctrl_t *o_ctrl, int cmd)
{
	int step;

	if (atomic_read(&o_ctrl->k618_pl_state) != 2) {
		CAM_DBG(CAM_OIS, "k618: parklens thread is in other state: %d",
			atomic_read(&o_ctrl->k618_pl_state));
		return -EINVAL;
	}
	step = atomic_read(&o_ctrl->k618_pl_step);
	if (step) {
		CAM_DBG(CAM_OIS, "k618: parklens thread is already end: %d", step);
	} else {
		if ((cmd != K618_PARK_STOP_POWER_DOWN) && (cmd != K618_PARK_STOP_KEEP_POWER)) {
			CAM_DBG(CAM_OIS, "k618: Invalid stop opcode, stop parklens with power down");
			cmd = K618_PARK_STOP_POWER_DOWN;
		}
		atomic_set(&o_ctrl->k618_pl_cmd, cmd);
		CAM_DBG(CAM_OIS, "k618: wake up parklens thread to stop it (%d)", cmd);
	}
	wait_for_completion(&o_ctrl->k618_pl_exit);
	atomic_set(&o_ctrl->k618_pl_state, 1);
	o_ctrl->k618_pl_task = NULL;
	step = atomic_read(&o_ctrl->k618_pl_step);
	if (step)
		CAM_DBG(CAM_OIS, "k618: parklens thread exit status: %d", step);
	else
		CAM_ERR(CAM_OIS, "k618: parklens thread exit failed, no status");
	return step;
}

/*
 * stock ois_parklens_power_down (0x15d728): 1 when the power-down is the thread's (it runs, or it
 * ended having powered down), 0 when the caller powers down
 */
static int k618_park_owns_power(struct cam_ois_ctrl_t *o_ctrl)
{
	int state = atomic_read(&o_ctrl->k618_pl_state);

	switch (state) {
	case 0:
		CAM_DBG(CAM_OIS, "k618: parklens is not created, power down");
		return 0;
	case 1:
		if (atomic_read(&o_ctrl->k618_pl_step) > 2)
			return 1;
		CAM_DBG(CAM_OIS, "k618: parklens stop without power down");
		return 0;
	case 2:
		CAM_DBG(CAM_OIS, "k618: parklens is running, no need powerdown");
		return 1;
	default:
		CAM_ERR(CAM_OIS, "k618: Invalid parklens_state %d", state);
		return 0;
	}
}

/*
 * packet 6 (stock 0x15f560-0x15f600): with no park set up and cam_ois_state CONFIG or START, the
 * first buffer's settings become k618_pl_settings and the thread starts (its result is not
 * returned, as stock); otherwise (0x16110c) a thread is stopped with power-down, the park torn
 * down, -EINVAL. A parse failure deletes the settings (0x1601dc).
 */
static int k618_park_pkt(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet)
{
	uint32_t *offset = (uint32_t *)&csl_packet->payload_flex;
	struct cam_cmd_buf_desc *cmd_desc;
	int pl_state = atomic_read(&o_ctrl->k618_pl_state);
	int step = atomic_read(&o_ctrl->k618_pl_step);
	int rc;

	CAM_DBG(CAM_OIS, "k618: Received parklens buffer, parklens_state %d", pl_state);
	if (pl_state || (o_ctrl->cam_ois_state < CAM_OIS_CONFIG)) {
		CAM_WARN(CAM_OIS,
			"k618: [OIS]Not in right state to do parklens: %d/%d, exit result %d release power control right",
			o_ctrl->cam_ois_state, pl_state, step);
		step = k618_park_stop(o_ctrl, K618_PARK_STOP_POWER_DOWN);
		if ((step >= 3) && (step <= 5) && (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG))
			o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;	/* the thread powered it down */
		k618_park_deinit(o_ctrl);
		return -EINVAL;
	}
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);
	o_ctrl->k618_pl_settings.request_id = 0;
	o_ctrl->k618_pl_settings.is_settings_valid = 1;
	rc = cam_sensor_i2c_command_parser(&(o_ctrl->io_master_info), &(o_ctrl->k618_pl_settings),
		cmd_desc, 1, NULL);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "k618: parklens settings parsing failed: %d", rc);
		delete_request(&(o_ctrl->k618_pl_settings));
		return rc;
	}
	k618_park_trigger(o_ctrl);
	return 0;
}

/*
 * a new INIT (stock 0x15f87c-0x15f8e8 at its power-settings buffer): stop the thread keeping the
 * power; if it had not powered down, apply the copied entries and mark the OIS as still powered
 * (the INIT then neither reads the power settings again nor powers up, stock 0x15fbbc, 0x1603d4);
 * then tear the park down
 */
static void k618_park_on_init(struct cam_ois_ctrl_t *o_ctrl)
{
	struct i2c_settings_list *l;
	int pd;

	o_ctrl->k618_pl_powered = false;
	if (!o_ctrl->k618_ultra || !atomic_read(&o_ctrl->k618_pl_state))
		return;
	k618_park_stop(o_ctrl, K618_PARK_STOP_KEEP_POWER);
	pd = k618_park_owns_power(o_ctrl);
	CAM_INFO(CAM_OIS, "k618: OIS %s: stop parklens thread, powerdown:%d", o_ctrl->ois_name, pd);
	if (!pd) {
		list_for_each_entry(l, &(o_ctrl->k618_pl_list.list_head), list)
			k618_park_modes_util(&(o_ctrl->io_master_info), l);
		o_ctrl->k618_pl_powered = true;
	} else if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;	/* the thread powered it down */
	}
	k618_park_deinit(o_ctrl);
}

/* the INIT's power-up decision: false (power up as usual) unless the park left the OIS powered */
static bool k618_park_take_power(struct cam_ois_ctrl_t *o_ctrl)
{
	bool powered = o_ctrl->k618_pl_powered;

	o_ctrl->k618_pl_powered = false;
	if (powered)
		CAM_INFO(CAM_OIS, "k618: OIS %s: no need repower up again", o_ctrl->ois_name);
	return powered;
}

/* cam_ois_pkt_parse() exit: an INIT that failed before its power-up decision powers the park's OIS down */
static void k618_park_init_end(struct cam_ois_ctrl_t *o_ctrl)
{
	if (!o_ctrl->k618_pl_powered)
		return;
	o_ctrl->k618_pl_powered = false;
	CAM_WARN(CAM_OIS, "k618: OIS %s: INIT ended before its power-up; powering the OIS down",
		o_ctrl->ois_name);
	cam_ois_power_down(o_ctrl);
}

/*
 * RELEASE (stock 0x15ebac-0x15ee74, 0x15eee8): true when a park thread runs or has powered down
 * -- RELEASE then neither powers down nor frees the power settings
 */
static bool k618_park_release(struct cam_ois_ctrl_t *o_ctrl)
{
	if (!o_ctrl->k618_ultra || !atomic_read(&o_ctrl->k618_pl_state))
		return false;
	if (k618_park_owns_power(o_ctrl) != 1)
		return false;
	CAM_INFO(CAM_OIS, "k618: OIS %s: CAM_RELEASE_DEV leaves the power-down to the parklens thread",
		o_ctrl->ois_name);
	return true;
}

/* cam_ois_shutdown(): stop a park thread with power-down; true when the thread powered down */
static bool k618_park_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	bool down;

	if (!o_ctrl->k618_ultra || !atomic_read(&o_ctrl->k618_pl_state))
		return false;
	k618_park_stop(o_ctrl, K618_PARK_STOP_POWER_DOWN);
	down = (k618_park_owns_power(o_ctrl) == 1);
	k618_park_deinit(o_ctrl);
	CAM_INFO(CAM_OIS, "k618: OIS %s: shutdown stopped the parklens thread, powered down: %d",
		o_ctrl->ois_name, down);
	return down;
}
