/** ***************************************************************************
 * @file nanosic_platform.c
 *
 * @brief provided interface of initialza and release nanosic driver .
 *
 * <em>Copyright (C) 2010, Nanosic, Inc.  All rights reserved.</em>
 * Author : Bin.yuan bin.yuan@nanosic.com
 * */

/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/kernel.h>
#include <asm/processor.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include "sheng_nano_charge_protocol.h"
#include "sheng_nano_charge_supply.h"
#include "inc/nano_macro.h"
int Nanosic_ADSP_Get_Object(u8 *object);


static ADSP_read_cb g_ADSP_read_cb = NULL;

static DEFINE_MUTEX(rwlock);
static DEFINE_MUTEX(nano_api_lock);

static u8 buf_tmp[I2C_DATA_LENGTH_READ];
static int length;

#define DEV_TYPE_KEYBOARD       (0x1)
#define DEV_TYPE_ZHIJIA         (0x2)

/*0x38:???  */
#define DEV_KEYBOARD_OBJECT  (0x38)
#define DEV_ZHIJIA_OBJECT    (0x3B)
#define DEV_UNKOWN_OBJECT     (0x0)
#define PROBE_CNT_MAX	100

static u8 gObject = DEV_UNKOWN_OBJECT;

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_FLOAT = 4,
	BQ2589X_VBUS_USB_SDP = 6,
	BQ2589X_VBUS_USB_CDP = 9, /*CDP for bq25890, Adapter for bq25892*/
	BQ2589X_VBUS_USB_DCP = 10,
	BQ2589X_VBUS_USB_QC_5v = 16,
	BQ2589X_VBUS_USB_QC2,
	BQ2589X_VBUS_USB_QC2_12v,
    BQ2589X_VBUS_USB_QC3 = 20,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

#define USER_VOTER			"USER_VOTER"
struct nanochg* g_nodev;

/* The exported charge struct keeps the original field order. Private queue and
 * device lifetime state live outside that ABI. No global packet overwrite. */
#define NANO_PENDING_SLOTS 16
struct nano_platform {
	struct nanochg charge;
	struct battmngr_device *battmgr;
	spinlock_t pending_lock;
	unsigned int head, tail, count;
	bool stopping;
	struct { size_t len; u8 data[I2C_DATA_LENGTH_READ]; } packets[NANO_PENDING_SLOTS];
};
EXPORT_SYMBOL(g_nodev);

/** ************************************************************************
 *  @func rawdata_show
 */
static inline void rawdata_show(const char* descp, const u8* buf , size_t size)
{
    int i=0;
    char display[300]={0};

    if(!descp) {
        dbgprint(DEBUG_LEVEL,"descp is null.\n");
        return;
    }

    for (i = 0; i < (size > 35 ? 35 : size); i++) {
        char str[4]={0};
        char str1[5] = {0};

        snprintf(str1,sizeof(str1),"%s%d%s", "[", i, "]");
        strcat(display, str1);

        snprintf(str,sizeof(str),"%02X%s",buf[i], " ");
        strcat(display,str);
    }

    dbgprint(DEBUG_LEVEL,"%s -> %s\n",descp,display);
}

int Nanosic_vendor_raw_data_inject(void *buf, size_t len)
{
	struct nano_platform *platform;
	unsigned long flags;
	int ret = sheng_nano_charge_validate(buf, len);

	if (ret)
		return ret;
	mutex_lock(&nano_api_lock);
	if (!g_nodev) {
		ret = -ENODEV;
		goto out;
	}
	platform = container_of(g_nodev, struct nano_platform, charge);
	if (platform->stopping) {
		ret = -ENODEV;
		goto out;
	}
	spin_lock_irqsave(&platform->pending_lock, flags);
	if (platform->count == NANO_PENDING_SLOTS) {
		ret = -ENOSPC;
	} else {
		memcpy(platform->packets[platform->tail].data, buf, len);
		platform->packets[platform->tail].len = len;
		platform->tail = (platform->tail + 1) % NANO_PENDING_SLOTS;
		platform->count++;
	}
	spin_unlock_irqrestore(&platform->pending_lock, flags);
	if (!ret)
		schedule_delayed_work(&g_nodev->analyse_rawdata_work, 0);
out:
	mutex_unlock(&nano_api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(Nanosic_vendor_raw_data_inject);

static int Nanosic_charge_i2c_write_callback_register_locked(i2c_write_cb cb)
{
    struct nanochg *nodev = g_nodev;
    int ret = 0;

    if (IS_ERR_OR_NULL(nodev))
    {
        dbgprint(ERROR_LEVEL, "nodev ERROR");
        ret = -ENODEV;
        return ret;
    }

    if (IS_ERR_OR_NULL(cb))
    {
        dbgprint(ERROR_LEVEL, "cb ERROR");
        ret = -EINVAL;
        return ret;
    }

    mutex_lock(&nodev->nano_i2c_write_mutex);

    if (!nodev->pogo_charge_i2c_write)
    {
        nodev->pogo_charge_i2c_write = cb;
    }
    else
    {
        dbgprint(ERROR_LEVEL, "charge_i2c_write already registered");
        ret = -EINVAL;
        goto _out;
    }

    dbgprint(INFO_LEVEL, "charge_i2c_write registered");

_out:
    mutex_unlock(&nodev->nano_i2c_write_mutex);

    return ret;
}
EXPORT_SYMBOL_GPL(Nanosic_charge_i2c_write_callback_register);

static int Nanosic_charge_i2c_write_callback_unregister_locked(void)
{
    struct nanochg *nodev = g_nodev;
    int ret = 0;

    if (IS_ERR_OR_NULL(nodev))
    {
        dbgprint(ERROR_LEVEL, "nodev ERROR");
        ret = -ENODEV;
        return ret;
    }

    mutex_lock(&nodev->nano_i2c_write_mutex);
    if (nodev->pogo_charge_i2c_write)
    {
        nodev->pogo_charge_i2c_write = NULL;
    }
    else
    {
        dbgprint(ERROR_LEVEL, "pogo_charge_i2c_write already unregistered");
        ret = -ENODEV;
        goto _out;
    }

    dbgprint(INFO_LEVEL, "charge_i2c_write unregistered");

_out:
    mutex_unlock(&nodev->nano_i2c_write_mutex);

    return ret;
}
EXPORT_SYMBOL_GPL(Nanosic_charge_i2c_write_callback_unregister);


/**
 * @brief This function will call Nanosic_charge_i2c_write_callback
 *        in nanosic_driver.c
 *
 * @param buf Data buf
 * @param len Data length
 *
 * @return Result num of bytes success or failure mode
 */
__attribute__((unused)) static int Nanosic_charge_i2c_write(void *buf, size_t len)
{
    struct nanochg *nodev = g_nodev;
    int ret = 0;

    if (IS_ERR_OR_NULL(nodev))
    {
        dbgprint(ERROR_LEVEL, "nodev ERROR");
        ret = -ENODEV;
        return ret;
    }

    mutex_lock(&nodev->nano_i2c_write_mutex);
    if (!IS_ERR_OR_NULL(nodev->pogo_charge_i2c_write))
    {
        ret = nodev->pogo_charge_i2c_write(buf, len);
    }
    else
    {
        dbgprint(ERROR_LEVEL, "pogo_charge_i2c_write ERROR");
        ret = -ENODEV;
        goto _out;
    }

_out:
    mutex_unlock(&nodev->nano_i2c_write_mutex);

    return ret;
}


static int nano_send_charge(u8 cmd, u8 addr, u8 count, const u8 *data,
			    size_t len, u8 power)
{
	u8 frame[I2C_DATA_LENGTH_WRITE];
	int ret;

	mutex_lock(&nano_api_lock);
	if (!g_nodev || container_of(g_nodev, struct nano_platform, charge)->stopping) {
		ret = -ENODEV;
	} else {
		mutex_lock(&rwlock);
		ret = sheng_nano_charge_command(frame, gObject, cmd, addr, count, data, len, power);
		mutex_unlock(&rwlock);
	}
	if (!ret) {
		ret = Nanosic_charge_i2c_write(frame, sizeof(frame));
		if (ret >= 0 && ret != sizeof(frame))
			ret = -EIO;
	}
	mutex_unlock(&nano_api_lock);
	return ret;
}
static void assign_mcu_data(struct read_mcu_data* mcuData, const u8* buf)
{
    if (mcuData) {
        mcuData->input_volt = (int)(buf[11] * 100);
        mcuData->input_watt = buf[12];
        mcuData->vbat_set = buf[13];
        mcuData->ibat_set = buf[14];
        mcuData->work_state = buf[15];
        mcuData->typec_state = buf[16];
        mcuData->dpdm_in_state = buf[17];
        mcuData->protocol_volt = (int)(buf[18] * 100);
        mcuData->protocol_curr = (int)(buf[19] * 50);
        mcuData->vbus_pwr = (int)(buf[20] * 100);
        mcuData->vbus_mon = (int)(buf[21] * 100);
        mcuData->typec_curr = (int)(buf[22] * 50);
        mcuData->ntc_l = buf[23];
        mcuData->ntc_h = buf[24];
        mcuData->dpdm0_state = buf[25];
        mcuData->sink_comm = buf[26];
        dbgprint(DEBUG_LEVEL,"Address:%02x, RegLength:%02x, input_volt:%d, input_watt:%02x, vbat_set:%02x, ibat_set:%02x\n",
                                buf[9], buf[10], mcuData->input_volt, mcuData->input_watt, mcuData->vbat_set, mcuData->ibat_set);

        dbgprint(DEBUG_LEVEL, "work_state:%02x, typec_state:%02x, dpdm_in_state:%02x, protocol_volt:%d\n",
                                mcuData->work_state, mcuData->typec_state, mcuData->dpdm_in_state, mcuData->protocol_volt);
        dbgprint(DEBUG_LEVEL, "protocol_curr:%d, vbus_pwr:%d, vbus_mon:%d, typec_curr:%d\n",
                                mcuData->protocol_curr, mcuData->vbus_pwr, mcuData->vbus_mon, mcuData->typec_curr);
        dbgprint(DEBUG_LEVEL, "ntc_l:%02x, ntc_h:%02x, dpdm0_state:%02x, sink_comm:%02x\n",
                                mcuData->ntc_l, mcuData->ntc_h, mcuData->dpdm0_state, mcuData->sink_comm);
    }
}

static u8 vin_low;
static u8 vin_high;
static u8 i_low;
static u8 i_high;
static void assign_holder_mcu_data(struct read_mcu_data* mcuData, const u8* buf)
{
    int vin = 0, curr = 0;
    if (mcuData) {
        mcuData->addr = buf[9];
        switch (mcuData->addr)
        {
        case 0x01:
        dbgprint(DEBUG_LEVEL,"ctrl-1 Address:%02x, RegLength:%02x, data:%02x\n",
                                 buf[9], buf[10], buf[11]);
        break;
        case 0x10:
            mcuData->typec_state = buf[11];
            break;
        case 0x11:
            mcuData->code = buf[11];
            break;
        case 0x13:
            mcuData->vout_low = buf[11];
            vin_low = buf[11];
            break;
        case 0x14:
            mcuData->vout_high = buf[11];
            vin_high = buf[11];
            vin = (int)((vin_high << 8) | vin_low);
            dbgprint(DEBUG_LEVEL,"Volt-Curr Address:%02x, vin:%d\n", buf[9], vin);
            break;
        case 0x1e:
            mcuData->vout_low = buf[11];
            i_low = buf[11];
            break;
        case 0x1f:
            mcuData->vout_high = buf[11];
            i_high = buf[11];
            curr = (int)((i_high << 8) | i_low);
            dbgprint(DEBUG_LEVEL,"Volt-Curr Address:%02x, curr_in:%d\n", buf[9], curr);
            break;
        default:
            break;
        }

        dbgprint(DEBUG_LEVEL,"Address:%02x, RegLength:%02x, data:%02x\n",
                                 buf[9], buf[10], buf[11]);
    }
}

static int nano_get_chg_real_type_locked(int* real_type, struct battmngr_notify *data)
{
	int type;

    if (!g_nodev || !data) {
        dbgprint(ERROR_LEVEL, "%s g_nodev or data is null\n", __func__);
        return -ENODEV;
    }

    if (gObject != DEV_KEYBOARD_OBJECT) {

        if (READ_ONCE(data->irq_msg.value)) {
            WRITE_ONCE(data->pd_msg.pd_active, 1);
            dbgprint(ERROR_LEVEL, "%s holder detect pd type. \n", __func__);
            *real_type = POWER_SUPPLY_USB_TYPE_PD;
        } else
            *real_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

        goto result;
    }

	type = (int)g_nodev->dpdm;

	dbgprint(ERROR_LEVEL, "%s type=%d, old_type=%d \n", __func__, type, g_nodev->old_type);

	if (g_nodev->pd_flag) {
        WRITE_ONCE(data->pd_msg.pd_active, 1);
		dbgprint(ERROR_LEVEL, "%s detect pd type. \n", __func__);
		*real_type = POWER_SUPPLY_USB_TYPE_PD;
		goto result;
	}

	switch (type)
	{
	case BQ2589X_VBUS_NONSTAND:
	case BQ2589X_VBUS_UNKNOWN:
	case BQ2589X_VBUS_USB_FLOAT:
		*real_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	case BQ2589X_VBUS_USB_SDP:
		*real_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case BQ2589X_VBUS_USB_CDP:
		*real_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case BQ2589X_VBUS_USB_DCP:
		*real_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	case BQ2589X_VBUS_USB_QC2:
		*real_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		break;
	 case BQ2589X_VBUS_USB_QC3:
	 	*real_type = POWER_SUPPLY_TYPE_USB_HVDCP_3;
	 	break;
	default:
		*real_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	}

result:
	g_nodev->charge_type = *real_type;
	dbgprint(ERROR_LEVEL, "%s bc1.2 type=%d\n", __func__, *real_type);

	return 0;
}

static int nano_get_chg_usb_type_locked(int* usb_type, struct battmngr_notify *data)
{
	int type;

    if (!g_nodev || !data) {
        dbgprint(ERROR_LEVEL, "%s g_nodev or data is null\n", __func__);
        return -ENODEV;
    }

    if (gObject != DEV_KEYBOARD_OBJECT) {

        if (READ_ONCE(data->irq_msg.value)) {
            WRITE_ONCE(data->pd_msg.pd_active, 1);
            dbgprint(ERROR_LEVEL, "%s holder detect pd type. \n", __func__);
            *usb_type = POWER_SUPPLY_USB_TYPE_PD;
        } else
            *usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

        goto result;
    }

	type = (int)g_nodev->dpdm;

	dbgprint(ERROR_LEVEL, "%s type=%d, old_type=%d \n", __func__, type, g_nodev->old_type);

	if (g_nodev->pd_flag) {
        WRITE_ONCE(data->pd_msg.pd_active, 1);
		dbgprint(ERROR_LEVEL, "%s detect pd type. \n", __func__);
		*usb_type = POWER_SUPPLY_USB_TYPE_PD;
		goto result;
	}

	switch (type)
	{
	case BQ2589X_VBUS_NONSTAND:
	case BQ2589X_VBUS_UNKNOWN:
	case BQ2589X_VBUS_USB_FLOAT:
		*usb_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	case BQ2589X_VBUS_USB_SDP:
		*usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case BQ2589X_VBUS_USB_CDP:
		*usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case BQ2589X_VBUS_USB_DCP:
    case BQ2589X_VBUS_USB_QC2:
    case BQ2589X_VBUS_USB_QC3:
		*usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		*usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	}

result:
	dbgprint(ERROR_LEVEL, "%s bc1.2 type=%d\n", __func__, *usb_type);

	return 0;
}

static int nano_is_charge_online_locked(u8* online, struct battmngr_notify *data)
{
    if (g_nodev) {
        g_nodev->usb_icl_votable = find_votable("ICL");

        if (!g_nodev->usb_icl_votable || !data)
            return -ENODEV;

        if (get_client_vote_locked(g_nodev->usb_icl_votable, USER_VOTER) == 0) {
            *online = false;
            return 1;
        }
		 if (data)
            *online = READ_ONCE(data->irq_msg.value);
        return 1;
    } else {
        dbgprint(ERROR_LEVEL,"%s g_nodev is null\n", __func__);
    }

	return -ENODEV;
}

static int nano_input_curr_limit_locked(int* curr_limit, struct battmngr_notify *data)
{
    if (!data) return -ENODEV;
    if (g_nodev) {
        g_nodev->usb_icl_votable = find_votable("ICL");
        if (!g_nodev->usb_icl_votable)
            return -ENODEV;
        *curr_limit = get_effective_result(g_nodev->usb_icl_votable);
        if (*curr_limit >= 0)
            return 1;

    } else {
        dbgprint(ERROR_LEVEL,"%s g_nodev is null\n", __func__);
    }

	return -1;
}

static int nano_get_power_max_locked(int* powr_max)
{
    if (g_nodev) {
        if (gObject == DEV_KEYBOARD_OBJECT)
            *powr_max = ((g_nodev->protocol_volt / 1000) * (g_nodev->protocol_curr / 1000));
        else if (gObject == DEV_ZHIJIA_OBJECT)
            *powr_max = 22;
        else
            *powr_max = 0;

        if (*powr_max >= 0)
            return 1;

    } else {
        dbgprint(ERROR_LEVEL,"%s g_nodev is null\n", __func__);
    }

	return -1;
}

static void deal_report_event(const u8* buf, struct battmngr_notify *data)
{
    static int last_online;

    if (buf && g_nodev) {
        if (gObject == DEV_KEYBOARD_OBJECT) {
            g_nodev->online = buf[16] & 0x01;

            g_nodev->pd_flag = (buf[16] & 0x04) >> 2;

            g_nodev->dpdm_in_state = (buf[17] & 0x80);
            g_nodev->dpdm = buf[25];
            g_nodev->protocol_volt = (int)(buf[18] * 100);
            g_nodev->protocol_curr = (int)(buf[19] * 50);
        } else if (data){
            g_nodev->online = READ_ONCE(data->irq_msg.value);
        }

        dbgprint(ERROR_LEVEL, "ENTER -1 last_online:%d, %d.\n", last_online, g_nodev->online);
        // send notify
        if (data && last_online != g_nodev->online) {
            dbgprint(ERROR_LEVEL, "Send report info.\n");
            //data->mcu_msg.msg_type = BATTMNGR_MSG_MCU_TYPE;
            battmngr_notifier_call_chain(BATTMNGR_EVENT_REPORT, data);
        }
        last_online = g_nodev->online;

    }
}

#include "sheng_nano_notify_get.h"
SHENG_NANO_NOTIFY_GETTER(nano_get_chg_real_type, int, nano_get_chg_real_type_locked)

SHENG_NANO_NOTIFY_GETTER(nano_get_chg_usb_type, int, nano_get_chg_usb_type_locked)

SHENG_NANO_NOTIFY_GETTER(nano_is_charge_online, u8, nano_is_charge_online_locked)

SHENG_NANO_NOTIFY_GETTER(nano_input_curr_limit, int, nano_input_curr_limit_locked)

static int nano_get_power_max(int *powr_max)
{
	int ret;
	if (!powr_max)
		return -EINVAL;
	mutex_lock(&nano_api_lock);
	ret = g_nodev ? nano_get_power_max_locked(powr_max) : -ENODEV;
	mutex_unlock(&nano_api_lock);
	return ret;
}

static const struct battmngr_ops nano_op = {
    .online = nano_is_charge_online,
    .real_type = nano_get_chg_real_type,
    .usb_type = nano_get_chg_usb_type,
    .input_curr_limit = nano_input_curr_limit,
    .power_max = nano_get_power_max,
};

/** ************************************************************************
 *  @func Nanosic_Calculate_crc
 *
 *  @brief calculate sum
 *
 ** */
static __maybe_unused
uint8_t Nanosic_Calculate_crc(uint8_t* data,uint32_t datalen)
{
    uint8_t i=0;
    uint8_t crc = 0;

    for(i=2;i<datalen;i++) /*fixed bug in 2023.6.15*/
        crc += data[i];

    return crc;
}

/** ************************************************************************
 *  @func Nanosic_ADSP_Set_Charge_Status
 *
 *  B0     B1    B2    B3    B4    B5    B6    B7......B32     B33  B34  .......B65
 *  0x32   0x00  0x4E  0x31  0x81  0x38  0x42  DATA0...DATA25  crc   0  .......  0
 *
 *  @brief send command to i2c device driver , Called by adsp device driver
 *  write i2c fail when return < 0 or succee return 66
 ** */
int  Nanosic_ADSP_Set_Charge_Status(uint8_t* data,uint32_t datalen)
{
	if (!data)
		return -EINVAL;
	return nano_send_charge(0x42, 0, 0, data, datalen, 1);
}

/** ************************************************************************
 *  @func Nanosic_ADSP_Read_Charge_Status
 *
 *  B0     B1    B2    B3    B4    B5    B6    B7    B8    B9........B65
 *  0x32   0x00  0x4E  0x31  0x81  0x38  0x40  0x00  crc   0.......  0
 *
 *  @brief send command to i2c device driver , Called by adsp device driver
 *  write i2c fail when return < 0 or succee return 66
 ** */
int  Nanosic_ADSP_Read_Charge_Status(void)
{
	return nano_send_charge(0x40, 0, 0, NULL, 0, 0);
}
EXPORT_SYMBOL(Nanosic_ADSP_Read_Charge_Status);

/** ************************************************************************
 *  @func Nanosic_ADSP_Read_Charge_Status_Single
 *
 *  B0     B1    B2    B3    B4    B5    B6    B7    B8    B9........B65
 *  0x32   0x00  0x4E  0x31  0x81  0x38  0x40  0x00  crc   0.......  0
 *
 *  @brief send command to i2c device driver , Called by adsp device driver
 *  write i2c fail when return < 0 or succee return 66
 ** */
int Nanosic_ADSP_Read_Charge_Status_Single(uint8_t addr)
{
	return nano_send_charge(0x40, addr, 1, NULL, 0, 0);
}
EXPORT_SYMBOL(Nanosic_ADSP_Read_Charge_Status_Single);

/** ************************************************************************
 *  @func Nanosic_ADSP_Set_Charge_Status_Single
 *
 *  B0     B1    B2    B3    B4    B5    B6    B7......B32     B33  B34  .......B65
 *  0x32   0x00  0x4E  0x31  0x81  0x38  0x42  DATA0...DATA25  crc   0  .......  0
 *
 *  @brief send command to i2c device driver , Called by adsp device driver
 *  write i2c fail when return < 0 or succee return 66
 ** */
int  Nanosic_ADSP_Set_Charge_Status_Single(uint8_t addr, uint8_t addrlen, uint8_t* data, uint32_t datalen)
{
	if (!addr)
		return -EINVAL;
	return nano_send_charge(0x42, addr, addrlen, data, datalen, 1);
}

/** ************************************************************************
 *  @func Nanosic_ADSP_Control_Charge_Power
 *  
 *  B0     B1    B2    B3    B4    B5    B6    B7......B32     B33  B34  .......B65
 *  0x32   0x00  0x4E  0x31  0x81  0x38  0x42  DATA0...DATA25  crc   0  .......  0
 *
 *  @brief send command to i2c device driver , Called by adsp device driver
 *  write i2c fail when return < 0 or succee return 66
 ** */
int  Nanosic_ADSP_Control_Charge_Power(bool on)
{
	return nano_send_charge(0x42, 0, 0, NULL, 0, on);
}
EXPORT_SYMBOL(Nanosic_ADSP_Control_Charge_Power);

/** ************************************************************************
 *  @func Nanosic_ADSP_Read_803_Version
 *
 *  B0     B1    B2    B3    B4    B5    B6    B7    B8    B9.............B65
 *  0x32   0x00  0x4E  0x30  0x80  0x38  0x1   0x0   crc   0............  0
 *
 *  @brief send command to i2c device driver , Called by adsp device driver
 *  write i2c fail when return < 0 or succee return 66
 ** */
int  Nanosic_ADSP_Read_803_Version(void)
{
    int ret = -1;
    uint8_t read_803_vers_cmd[I2C_DATA_LENGTH_WRITE]={0x32,0x00,0x4F,0x30,FIELD_HOST,FIELD_803X,0x01,0x00,0x00};

    read_803_vers_cmd[8] = Nanosic_Calculate_crc(read_803_vers_cmd,sizeof(read_803_vers_cmd));

    rawdata_show("request 803 version",read_803_vers_cmd,sizeof(read_803_vers_cmd));

    ret = Nanosic_charge_i2c_write(read_803_vers_cmd, sizeof(read_803_vers_cmd));
    if (ret < 0)
    {
        dbgprint(ERROR_LEVEL, "i2c write fail, ret = %d, size = %zu", ret, sizeof(read_803_vers_cmd));
    }

    return ret;
}

/** ************************************************************************
 *  @func Nanosic_ADSP_register_cb
 *
 *  @brief register read callback function for adsp , Called by adsp device driver
 *
 ** */
int  Nanosic_ADSP_register_cb(ADSP_read_cb cb)
{
	int ret = 0;
	if (IS_ERR_OR_NULL(cb))
		return -EINVAL;
	mutex_lock(&nano_api_lock);
	if (!g_nodev)
		ret = -ENODEV;
	else if (g_ADSP_read_cb)
		ret = -EBUSY;
	else
		g_ADSP_read_cb = cb;
	mutex_unlock(&nano_api_lock);
	return ret;
}

static void Nanosic_analyse_packet_state(struct work_struct *work, struct battmngr_notify *data)
{
    struct nanochg *nodev = container_of(work, struct nanochg, analyse_rawdata_work.work);

    char    report_id;
    char    protocol;
    char    source;
    char    object;
    char    command;
    int  retval = 0;
    u8 *rawdata_buf;
    char len, subcmd, cmd_state, rev,keyboard_state, dev_type, subcmd_status;
    bool connected = false;
    char *dev_name = "unknown";
    char pswitch,reserve,ic_type,reg_addr,reg_len,reg_data[19];

    u16     mcu_version;
    u16     dev_version;
    u8      colour;
    u16     pid;

    if (IS_ERR_OR_NULL(nodev))
    {
        dbgprint(ERROR_LEVEL, "nodev ERROR");
        retval = -ENODEV;
        return;
    }
	rawdata_buf = buf_tmp;

    STREAM_TO_UINT8(report_id, rawdata_buf);
    STREAM_TO_UINT8(protocol, rawdata_buf);
    STREAM_TO_UINT8(source, rawdata_buf);
    STREAM_TO_UINT8(object, rawdata_buf);
    STREAM_TO_UINT8(command, rawdata_buf);

    rawdata_show("recv data1", buf_tmp, length);

    switch(command) {
        case CMD_KEYBOARD_CONN_STATUS_RSP:
        {
            mutex_lock(&rwlock);
            gObject = DEV_UNKOWN_OBJECT;

            STREAM_TO_UINT8(len, rawdata_buf);
            STREAM_TO_UINT8(subcmd, rawdata_buf);
            STREAM_TO_UINT8(cmd_state, rawdata_buf);
            STREAM_TO_UINT8(rev, rawdata_buf);
            STREAM_TO_UINT8(keyboard_state, rawdata_buf);
            STREAM_TO_UINT8(dev_type, rawdata_buf);  /*1:keyboard  2:zhijia*/

            if (cmd_state == 0) {
                if (((keyboard_state & 0x3) == 0x3) || ((keyboard_state & 0x3) == 0x1)) {
                    dev_name = "keyboard";
                    if ((keyboard_state & 0x60) == 0x20) {
                        connected = true;
                        gObject = DEV_KEYBOARD_OBJECT;
                    } else {
                        dbgprint(ERROR_LEVEL, "fail to connect keyboard device because no data\n");
                    }
                } else if ((keyboard_state & 0x3) == 0x0) {
                    if ((keyboard_state & 0x68) == 0x28) {
                        connected = true;
                        gObject = DEV_ZHIJIA_OBJECT;
                        dev_name = "zhijia";
                    } else {
                        dbgprint(ERROR_LEVEL, "fail to connect zhijia device\n");
                    }
                }
                g_nodev->gObject = gObject;
                dbgprint(DEBUG_LEVEL, "%s attach_detect connected %s\n", dev_name, connected ? "true" : "fasle");
            }

            mutex_unlock(&rwlock);

            if (data && (gObject == DEV_ZHIJIA_OBJECT)) {
                mutex_lock(&data->notify_lock);
                dbgprint(ERROR_LEVEL, "Send zhijia connect info.\n");

                data->mcu_msg.kb_attach = connected;
                data->mcu_msg.object_type = gObject;
                data->mcu_msg.msg_type = BATTMNGR_MSG_MCU_TYPE;
                battmngr_notifier_call_chain(BATTMNGR_EVENT_MCU, data);
                mutex_unlock(&data->notify_lock);
            } else {
                dbgprint(ERROR_LEVEL, "ADSP read cb is null\n");
            }
        }
        break;
        case CMD_KEYBOARD_READ_VERSION_RSP:
        {
            STREAM_TO_UINT8(len, rawdata_buf);
            STREAM_TO_UINT16(mcu_version, rawdata_buf);
            STREAM_TO_UINT16(dev_version, rawdata_buf);
            STREAM_TO_UINT8(colour, rawdata_buf);
            STREAM_TO_UINT16(pid, rawdata_buf);
            STREAM_TO_UINT8(dev_type, rawdata_buf);
            dbgprint(DEBUG_LEVEL,"dev_type %d dev_version %d\n",dev_type,dev_version);
            break;
        }
        case CMD_MCU_RSP:
        {
            STREAM_TO_UINT8(len, rawdata_buf);
            STREAM_TO_UINT8(subcmd, rawdata_buf);
            STREAM_TO_UINT8(subcmd_status, rawdata_buf);

            //rawdata_show("got mcu rsp event", rawdata_buf, sizeof(rawdata_buf));
        }
        break;
        case CMD_READ_CHARGE_STATUS_RSP:
        {
            STREAM_TO_UINT8(len, rawdata_buf);
            STREAM_TO_UINT8(pswitch, rawdata_buf);
            STREAM_TO_UINT8(reserve, rawdata_buf);
            STREAM_TO_UINT8(ic_type, rawdata_buf);
            STREAM_TO_UINT8(reg_addr, rawdata_buf);
            STREAM_TO_UINT8(reg_len, rawdata_buf);
            memcpy(reg_data, rawdata_buf, reg_len);

            //rawdata_show("got read charge event", rawdata_buf, sizeof(rawdata_buf));
            if (data && (gObject == DEV_ZHIJIA_OBJECT)) {
                mutex_lock(&data->notify_lock);
                dbgprint(ERROR_LEVEL, "Send charge info.\n");
                if (gObject == DEV_KEYBOARD_OBJECT)
                    assign_mcu_data(&data->mcu_msg.mcu_data, buf_tmp);
                else
                    assign_holder_mcu_data(&data->mcu_msg.mcu_data, buf_tmp);
                deal_report_event(buf_tmp, data);
                data->mcu_msg.object_type = gObject;
                data->mcu_msg.msg_type = BATTMNGR_MSG_MCU_TYPE;
                battmngr_notifier_call_chain(BATTMNGR_EVENT_MCU, data);
                mutex_unlock(&data->notify_lock);
            } else {
                dbgprint(ERROR_LEVEL, "ADSP read cb is null\n");
            }
        }
        break;
        case CMD_SET_CHARGE_REG_RSP:
        {
            STREAM_TO_UINT8(len,rawdata_buf);
            STREAM_TO_UINT8(subcmd_status,rawdata_buf);
            //rawdata_show("got set charge cmd rsp event", rawdata_buf, sizeof(rawdata_buf));
        }
        break;
        default:
            dbgprint(ERROR_LEVEL, "unkown cmd rsp code\n");
        break;
    }


    return;
}

static void Nanosic_analyse_packet(struct work_struct *work)
{
	struct battmngr_notify *data;
	int cookie;

	/* The worker's device lifetime is drained by remove. SRCU keeps its
	 * actual manager alive even if a notification requests GLINK/nano. */
	data = battmngr_notify_acquire(&cookie);
	Nanosic_analyse_packet_state(work, data);
	battmngr_notify_release(cookie);
}


static void Nanosic_Analyse_RawData_work(struct work_struct *work)
{
	struct nanochg *nodev = container_of(work, struct nanochg, analyse_rawdata_work.work);
	struct nano_platform *platform = container_of(nodev, struct nano_platform, charge);
	unsigned long flags;

	mutex_lock(&nano_api_lock);
	while (g_nodev == nodev && !platform->stopping) {
		spin_lock_irqsave(&platform->pending_lock, flags);
		if (!platform->count) {
			spin_unlock_irqrestore(&platform->pending_lock, flags);
			break;
		}
		length = platform->packets[platform->head].len;
		memset(buf_tmp, 0, sizeof(buf_tmp));
		memcpy(buf_tmp, platform->packets[platform->head].data, length);
		platform->head = (platform->head + 1) % NANO_PENDING_SLOTS;
		platform->count--;
		spin_unlock_irqrestore(&platform->pending_lock, flags);
		/* A notifier may call an ADSP command synchronously. Remove keeps
		 * nodev alive until cancel_delayed_work_sync has joined this worker. */
		mutex_unlock(&nano_api_lock);
		Nanosic_analyse_packet(work);
		mutex_lock(&nano_api_lock);
	}
	mutex_unlock(&nano_api_lock);
}
/** ************************************************************************
 *  @func Nanosic_platform_probe
 *
 *  @brief
 *
 ** */
static int
Nanosic_platform_probe(struct platform_device *pdev)
{
	struct nano_platform *platform;
	struct nanochg *nodev;
	int ret = 0;

	platform = devm_kzalloc(&pdev->dev, sizeof(*platform), GFP_KERNEL);
	if (!platform)
		return -ENOMEM;
	nodev = &platform->charge;
	nodev->dev = &pdev->dev;
	mutex_init(&nodev->nano_i2c_write_mutex);
	spin_lock_init(&platform->pending_lock);
	INIT_DELAYED_WORK(&nodev->analyse_rawdata_work, Nanosic_Analyse_RawData_work);
	platform->battmgr = battmngr_device_register("nano_ops", nodev->dev, nodev, &nano_op, NULL);
	if (IS_ERR_OR_NULL(platform->battmgr))
		return platform->battmgr ? PTR_ERR(platform->battmgr) : -ENOMEM;
	mutex_lock(&nano_api_lock);
	if (g_nodev) {
		ret = -EBUSY;
	} else {
		gObject = DEV_UNKOWN_OBJECT;
		g_nodev = nodev;
		platform_set_drvdata(pdev, platform);
	}
	mutex_unlock(&nano_api_lock);
	if (ret)
		battmngr_device_unregister(platform->battmgr);
	return ret;
}

/** ************************************************************************
 *  @func nanosic_platform_remove
 *
 *  @brief
 *
 ** */
static void Nanosic_platform_remove(struct platform_device *pdev)
{
	struct nano_platform *platform = platform_get_drvdata(pdev);
	mutex_lock(&nano_api_lock);
	platform->stopping = true;
	platform->charge.pogo_charge_i2c_write = NULL;
	mutex_unlock(&nano_api_lock);
	cancel_delayed_work_sync(&platform->charge.analyse_rawdata_work);
	mutex_lock(&nano_api_lock);
	g_nodev = NULL;
	gObject = DEV_UNKOWN_OBJECT;
	g_ADSP_read_cb = NULL;
	mutex_unlock(&nano_api_lock);
	battmngr_device_unregister(platform->battmgr);
	platform_set_drvdata(pdev, NULL);
}

static int Nanosic_platform_suspend(struct device *dev)
{
	struct nano_platform *platform = dev_get_drvdata(dev);
	flush_delayed_work(&platform->charge.analyse_rawdata_work);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(nano_platform_pm, Nanosic_platform_suspend, NULL);

static const struct of_device_id nanosic_platform_ids[] = {
	{ .compatible = "nanosic,platform" },
	{ /* Sentinel */ }
};

static struct platform_driver nanosic_platform_driver = {
	.probe = Nanosic_platform_probe,
	.remove = Nanosic_platform_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name = "nanosic,platform",
		.pm = pm_sleep_ptr(&nano_platform_pm),
		.of_match_table	= nanosic_platform_ids,
	},
};

/** ************************************************************************
 *  @func nanosic_platform_init
 *
 *  @brief
 *
 ** */
static int __init Nanosic_platform_init(void)
{
    int retval = 0;

    retval = platform_driver_register(&nanosic_platform_driver);
    if(retval < 0)
        dbgprint(ERROR_LEVEL,"platform init %d\n",retval);

    return retval;

}

/** ************************************************************************
 *  @func nanosic_platform_exit
 *
 *  @brief
 *
 ** */
static void __exit Nanosic_platform_exit(void)
{
    platform_driver_unregister(&nanosic_platform_driver);
}

module_init(Nanosic_platform_init);
module_exit(Nanosic_platform_exit);

EXPORT_SYMBOL_GPL(Nanosic_ADSP_register_cb);
EXPORT_SYMBOL_GPL(Nanosic_ADSP_Set_Charge_Status);
EXPORT_SYMBOL_GPL(Nanosic_ADSP_Set_Charge_Status_Single);

MODULE_DEVICE_TABLE(of, nanosic_platform_ids);
MODULE_DESCRIPTION("Nanosic platform Driver");
MODULE_LICENSE("GPL");


int Nanosic_charge_i2c_write_callback_register(i2c_write_cb cb)
{
	int ret;
	mutex_lock(&nano_api_lock);
	ret = Nanosic_charge_i2c_write_callback_register_locked(cb);
	mutex_unlock(&nano_api_lock);
	return ret;
}
int Nanosic_charge_i2c_write_callback_unregister(void)
{
	int ret;
	mutex_lock(&nano_api_lock);
	ret = Nanosic_charge_i2c_write_callback_unregister_locked();
	mutex_unlock(&nano_api_lock);
	return ret;
}

int Nanosic_ADSP_Get_Object(u8 *object)
{
 int ret = 0;
 if (!object) return -EINVAL;
 mutex_lock(&nano_api_lock);
 if (!g_nodev) ret = -ENODEV;
 else *object = READ_ONCE(g_nodev->gObject);
 mutex_unlock(&nano_api_lock);
 return ret;
}
EXPORT_SYMBOL(Nanosic_ADSP_Get_Object);
