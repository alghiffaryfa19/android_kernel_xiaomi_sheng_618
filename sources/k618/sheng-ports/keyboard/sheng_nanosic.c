// SPDX-License-Identifier: GPL-2.0-only
/*
 * HID driver for Nanosic WN8030 keyboard MCU
 *
 * Copyright (c) 2025 map220v <map220v300@gmail.com>
 */

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/kref.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define WN8030_HEADER_ADDR 0x7FC0
#define WN8030_CODE_ADDR 0x8000

#define XM_WN8030_I2C_READ (68)
#define XM_WN8030_I2C_WRITE (66)

#define WN8030_NUM_SUPPLIES 2

struct nanosic_wn8030 {
	struct device *dev;
	struct i2c_client *client;
	struct hid_device *hid_keyboard;
	struct hid_device *hid_touchpad;
	struct hid_device *hid_mouse;
	struct hid_device *hid_consumer;

	struct gpio_desc *status_gpio;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *sleep_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[WN8030_NUM_SUPPLIES];

	struct regmap *regmap;
	struct mutex conn_mutex;
	bool wake_irq_armed;

	bool suspended;
	bool keyboard_attached;
	bool connection_known;
	int connection_state;
	unsigned int firmware_generation, recovery_count;
	unsigned long recovery_window;

	/* Stock HAL owns authentication; kernel only transports real messages. */
	struct miscdevice raw_misc;
	struct kref refs;
	struct mutex clients_lock;
	struct list_head clients;
	struct mutex io_lock;
	struct delayed_work fw_worker;
	unsigned int fw_attempts;
	struct delayed_work plugin_worker;
	int plugin_requested, plugin_sent;
	unsigned int plugin_retries;
	bool ready, removing;
	const struct firmware *ram_firmware;

};

#include "sheng_nanosic_wake.h"
#include "sheng_nanosic_recovery.h"
#include "sheng_nano_hid_requests.h"
#include "sheng_nanosic_bridge.h"
#include "sheng_nanosic_power.h"
#include "sheng_nanosic_connection.h"
#include "sheng_nanosic_state.h"
extern int xiaomi_touchpad_edge_filter_param_init(struct device_node *node);
extern int xiaomi_touchpad_edge_filter(u8 *packet);
extern int Nanosic_vendor_raw_data_inject(void *buf, size_t len);
extern int Nanosic_charge_i2c_write_callback_register(int (*cb)(void *, size_t));
extern int Nanosic_charge_i2c_write_callback_unregister(void);
static struct nanosic_wn8030 *charge_nano;

/* Unregister waits for this callback before remove frees the client. */
static int sheng_nano_charge_write(void *buf, size_t len)
{
	struct nanosic_wn8030 *nano = READ_ONCE(charge_nano);
	int ret;
	if (!nano || !buf || len != XM_WN8030_I2C_WRITE)
		return -EINVAL;
	mutex_lock(&nano->io_lock);
	ret = nano->removing ? -ENODEV : !nano->ready ? -EAGAIN :
		sheng_nano_wake_locked(nano);
	sheng_nano_recover_after_timeout_locked(nano, ret);
	if (!ret)
		ret = regmap_bulk_write(nano->regmap, 0x5c, buf, len);
	mutex_unlock(&nano->io_lock);
	return ret ? ret : len;
}
#include "sheng_keyboard_charger.h"

static const struct regmap_config nanosic_wn8030_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#pragma pack(push, 1)
struct wn8030_bin_header {
	u8 flag[7];
	u16 chip_id;
	u32 ro_data;
	u32 rame_code; //??
	u32 rame_code_size; //??
	u32 code_entry;
	u16 pid;
	u16 vid;
	u8 major_version;
	u8 minor_version;
	char version_string[17];
	u32 ram_code_size;
	u32 ram_code_checksum;
	u32 unk;
	u8 device_type;
	u8 unk1[3];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct wn8030_header_cmd {
	u8 cmd; /* 0x06 */
	u8 flag[7];
	u16 chip_id;
	u32 ram_code_source;
	u32 ram_code_dest;
	u32 ram_code_size;
	u32 code_entry;
	u16 pid;
	u16 vid;
	u8 major_version;
	u8 minor_version;
	u8 unk[17]; /* version_string? */
	u32 image_size;
	u32 ram_code_checksum;
	u32 image_checksum;
	u8 pad[3];
	u8 header_checksum;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct wn8030_send_cmd {
	u8 cmd; /* 0x07 */
	__be32 ram_dest;
	__be32 size;
	u8 data[503];
};
#pragma pack(pop)

/* Exact original HID_KeyboardReportDescriptor; SHA256 ea3772c6141ab07f67df4cf7a7b3aa4e77180476433c7ba9dddb8f1f7d54c118 */
static u8 hid_keyboard_descriptor[] = {
	0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x85, 0x05, 0x05, 0x07, 0x19, 0xe0,
	0x29, 0xe7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x81, 0x03, 0x95, 0x05, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
	0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00,
	0x26, 0xa4, 0x00, 0x05, 0x07, 0x19, 0x00, 0x2a, 0xa4, 0x00, 0x81, 0x00,
	0xc0,
};

/* Exact original HID_TouchReportDescriptor; SHA256 6015a701529de372e6ec5c1ec0348c7a82d8825e0ad98e5ebbef02eb4c08f368 */
static u8 hid_touchpad_descriptor[] = {
	0x05, 0x0d, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x19, 0x15, 0x00, 0x25, 0x01,
	0x35, 0x00, 0x45, 0x01, 0x75, 0x01, 0x95, 0x02, 0x05, 0x09, 0x09, 0x01,
	0x09, 0x02, 0x81, 0x02, 0x95, 0x06, 0x81, 0x01, 0x05, 0x0d, 0x09, 0x22,
	0xa1, 0x02, 0x09, 0x42, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01,
	0x81, 0x02, 0x09, 0x32, 0x81, 0x02, 0x09, 0x47, 0x81, 0x02, 0x95, 0x05,
	0x81, 0x03, 0x75, 0x08, 0x09, 0x51, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01,
	0x15, 0x00, 0x26, 0xe7, 0x0b, 0x75, 0x10, 0x55, 0x0d, 0x65, 0x13, 0x09,
	0x30, 0x35, 0x00, 0x46, 0x00, 0x00, 0x81, 0x02, 0x09, 0x31, 0x26, 0xef,
	0x07, 0x46, 0x00, 0x00, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x05, 0x0d, 0x09,
	0x42, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02, 0x09,
	0x32, 0x81, 0x02, 0x09, 0x47, 0x81, 0x02, 0x95, 0x05, 0x81, 0x03, 0x75,
	0x08, 0x09, 0x51, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x15, 0x00, 0x26,
	0xe7, 0x0b, 0x75, 0x10, 0x55, 0x0d, 0x65, 0x13, 0x09, 0x30, 0x35, 0x00,
	0x46, 0x00, 0x00, 0x81, 0x02, 0x09, 0x31, 0x26, 0xef, 0x07, 0x46, 0x00,
	0x00, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x05, 0x0d, 0x09, 0x42, 0x15, 0x00,
	0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02, 0x09, 0x32, 0x81, 0x02,
	0x09, 0x47, 0x81, 0x02, 0x95, 0x05, 0x81, 0x03, 0x75, 0x08, 0x09, 0x51,
	0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x15, 0x00, 0x26, 0xe7, 0x0b, 0x75,
	0x10, 0x55, 0x0d, 0x65, 0x13, 0x09, 0x30, 0x35, 0x00, 0x46, 0x00, 0x00,
	0x81, 0x02, 0x09, 0x31, 0x26, 0xef, 0x07, 0x46, 0x00, 0x00, 0x81, 0x02,
	0xc0, 0x05, 0x0d, 0x09, 0x54, 0x95, 0x01, 0x75, 0x08, 0x15, 0x00, 0x25,
	0x08, 0x81, 0x02, 0x09, 0x55, 0xb1, 0x02, 0x06, 0x00, 0xff, 0x09, 0xc5,
	0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x96, 0x00, 0x01, 0xb1, 0x02,
	0xc0,
};

/* Exact original HID_MouseReportDescriptor; SHA256 3eef32d1f3220a35241d4f9f8ec62ff142c8f5738a90a58a0611a30c0b3d5da5 */
static u8 hid_mouse_descriptor[] = {
	0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x85, 0x02, 0x09, 0x01, 0xa1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01, 0x95, 0x05,
	0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x03, 0x81, 0x01, 0x05, 0x01,
	0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x16, 0x00, 0x80, 0x26, 0xff, 0x7f,
	0x75, 0x10, 0x95, 0x03, 0x81, 0x06, 0xc0, 0xc0,
};

/* Exact original HID_ConsumerReportDescriptor; SHA256 5921bb8886eb856cc783787d1d778ce46fe8f49856eadc7ee39e3ec971b82fd0 */
static u8 hid_consumer_descriptor[] = {
	0x05, 0x0c, 0x09, 0x01, 0xa1, 0x01, 0x85, 0x06, 0x15, 0x00, 0x26, 0x80,
	0x03, 0x19, 0x00, 0x2a, 0x80, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
	0xc0,
};

static int nanosic_wn8030_hid_parse(struct hid_device *hid)
{
	switch (hid->product) {
	case 0x00a3: return hid_parse_report(hid, hid_keyboard_descriptor, sizeof(hid_keyboard_descriptor));
	case 0x00a1: return hid_parse_report(hid, hid_touchpad_descriptor, sizeof(hid_touchpad_descriptor));
	case 0x00a2: return hid_parse_report(hid, hid_mouse_descriptor, sizeof(hid_mouse_descriptor));
	case 0x00a4: return hid_parse_report(hid, hid_consumer_descriptor, sizeof(hid_consumer_descriptor));
	default: return -ENODEV;
	}
}

static int nanosic_wn8030_hid_start(struct hid_device *hid)
{
	return 0;
}

static void nanosic_wn8030_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int nanosic_wn8030_hid_open(struct hid_device *hid)
{
	return 0;
}

static void nanosic_wn8030_hid_close(struct hid_device *hid)
{
}

static int nanosic_wn8030_raw_request(struct hid_device *hid,
				      unsigned char reportnum,
				      __u8 *buf, size_t len,
				      unsigned char rtype, int reqtype)
{
	return len;
}

static inline u32 nanosic_wn8030_checksum32(const u8 *data, int size)
{
	u32 checksum = 0;

	for (u32 i = 0; i < size; i += 4)
		checksum += get_unaligned_le32(data + i);

	return checksum;
}

static inline u8 nanosic_wn8030_checksum8(const u8 *data, int size)
{
	u8 checksum = 0;

	for (u32 i = 0; i < size; i++)
		checksum += data[i];

	return checksum;
}

static int nanosic_wn8030_set_caps_led(struct nanosic_wn8030 *nanosic, bool enable)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4E, 0x31,
					0x80, 0x38, 0x2E, 0x01 };

	int ret = sheng_nano_wake_locked(nanosic);
	sheng_nano_recover_after_timeout_locked(nanosic, ret);

	if (ret)
		return ret;
	buf[8] = enable ? 0xFD : 0xFC;
	buf[9] = nanosic_wn8030_checksum8(&buf[2], 7);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_output_report(struct hid_device *hid, u8 *buf, size_t count)
{
	struct nanosic_wn8030 *nanosic = hid->driver_data;
	int ret = sheng_nano_hid_output(buf, count);

	if (ret)
		return ret;
	if (hid->product != 0x00a3)
		return -EOPNOTSUPP;
	mutex_lock(&nanosic->io_lock);
	ret = nanosic->removing ? -ENODEV : !nanosic->ready ? -EAGAIN :
		nanosic_wn8030_set_caps_led(nanosic, buf[1]);
	mutex_unlock(&nanosic->io_lock);
	return ret ? ret : count;
}

static struct hid_ll_driver nanosic_wn8030_hid_ll_driver = {
	.parse = nanosic_wn8030_hid_parse,
	.start = nanosic_wn8030_hid_start,
	.stop = nanosic_wn8030_hid_stop,
	.open = nanosic_wn8030_hid_open,
	.close = nanosic_wn8030_hid_close,
	.raw_request = nanosic_wn8030_raw_request,
	.output_report = nanosic_wn8030_output_report,
};

static void sheng_nano_release_hids(struct nanosic_wn8030 *nano)
{
	struct hid_device **slots[] = { &nano->hid_keyboard, &nano->hid_touchpad,
		&nano->hid_mouse, &nano->hid_consumer };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(slots); i++) {
		struct hid_device *hid = *slots[i];
		*slots[i] = NULL;
		if (hid)
			hid_destroy_device(hid);
	}
	nano->keyboard_attached = false;
}

static int sheng_nano_create_hid(struct nanosic_wn8030 *nano,
				struct hid_device **slot, u32 product, const char *name)
{
	struct hid_device *hid = hid_allocate_device();
	int ret;

	if (IS_ERR(hid))
		return PTR_ERR(hid);
	hid->driver_data = nano;
	hid->ll_driver = &nanosic_wn8030_hid_ll_driver;
	/* These are the original logical HID identities over the real I2C MCU. */
	hid->bus = BUS_VIRTUAL;
	hid->dev.parent = nano->dev;
	hid->vendor = 0x15d9;
	hid->product = product;
	strscpy(hid->name, name, sizeof(hid->name));
	ret = hid_add_device(hid);
	if (ret) {
		hid_destroy_device(hid);
		return ret;
	}
	*slot = hid;
	return 0;
}

static int sheng_nano_attach_hids(struct nanosic_wn8030 *nano)
{
	int ret;
	ret = sheng_nano_create_hid(nano, &nano->hid_keyboard, 0x00a3, "Xiaomi Keyboard");
	if (ret)
		goto failed;
	ret = sheng_nano_create_hid(nano, &nano->hid_touchpad, 0x00a1, "Xiaomi Touch");
	if (ret)
		goto failed;
	ret = sheng_nano_create_hid(nano, &nano->hid_mouse, 0x00a2, "Xiaomi Mouse");
	if (ret)
		goto failed;
	ret = sheng_nano_create_hid(nano, &nano->hid_consumer, 0x00a4, "Xiaomi Consumer");
	if (ret)
		goto failed;
	nano->keyboard_attached = true;
	return 0;
failed:
	sheng_nano_release_hids(nano);
	dev_err(nano->dev, "Original HID registration failed: %d\n", ret);
	return ret;
}

static void nanosic_wn8030_handle_vendor(struct nanosic_wn8030 *nanosic,
					 u8 *buf, size_t len)
{
	int state = sheng_nano_connection_state(buf + 3, len);
	bool attached = state == SHENG_NANO_KEYBOARD;

	if (state != SHENG_NANO_NO_UPDATE) {
		mutex_lock(&nanosic->conn_mutex);
		if (nanosic->removing) {
			mutex_unlock(&nanosic->conn_mutex);
			return;
		}
		nanosic->connection_state = state;
		nanosic->connection_known = true;
		/* The shipping provider leaves keyboard_plugin unchanged for docks. */
		if (state != SHENG_NANO_DOCK && nanosic->plugin_requested != attached) {
			nanosic->plugin_requested = attached;
			nanosic->plugin_retries = 0;
			mod_delayed_work(system_wq, &nanosic->plugin_worker, 0);
		}
		if (attached && !nanosic->keyboard_attached) {
			sheng_nano_attach_hids(nanosic);
		} else if (!attached && nanosic->keyboard_attached) {
			sheng_nano_release_hids(nanosic);
		}
		mutex_unlock(&nanosic->conn_mutex);
	}
	/* Vendor reports, including auth challenges/replies, go to stock HAL. */

}

static irqreturn_t nanosic_wn8030_handler(int irq, void *data)
{
	struct nanosic_wn8030 *nano = data;
	u8 buf[XM_WN8030_I2C_READ], *p;
	size_t left, len;
	int ret;

	mutex_lock(&nano->io_lock);
	if (!nano->ready || nano->removing) {
		mutex_unlock(&nano->io_lock);
		return IRQ_HANDLED;
	}
	if (nano->suspended) {
		mutex_unlock(&nano->io_lock);
		return IRQ_HANDLED;
	}
	ret = regmap_bulk_read(nano->regmap, 0x4c, buf, sizeof(buf));
	mutex_unlock(&nano->io_lock);
	if (ret) {
		dev_warn_ratelimited(nano->dev, "MCU read failed: %d\n", ret);
		return IRQ_HANDLED;
	}
	if ((buf[0] & 0x7f) != 0x57 ||
	    (buf[2] != 0x39 && buf[2] != 0x4a && buf[2] != 0x5b && buf[2] != 0x6c))
		return IRQ_HANDLED;
	p = buf + 3;
	left = sizeof(buf) - 3;
	while (left) {
		switch (*p) {
		case 0x02: len = 8; break;
		case 0x05: len = 9; break;
		case 0x06: len = 5; break;
		case 0x19: len = 21; break;
		case 0x22: len = 16; break;
		case 0x23: len = 32; break;
		case 0x24: case 0x26: len = left; break;
		default: return IRQ_HANDLED;
		}
		if (len > left)
			return IRQ_HANDLED;
		if (*p == 0x05 && nano->hid_keyboard)
			hid_input_report(nano->hid_keyboard, HID_INPUT_REPORT, p, len, 0);
		else if (*p == 0x06 && nano->hid_consumer)
			hid_input_report(nano->hid_consumer, HID_INPUT_REPORT, p, len, 0);
		else if (*p == 0x02 && nano->hid_mouse)
			hid_input_report(nano->hid_mouse, HID_INPUT_REPORT, p, len, 0);
		else if (*p == 0x19 && nano->hid_touchpad) {
			int filtered = xiaomi_touchpad_edge_filter(p);
			/* Stock -EINVAL means the user explicitly disabled the filter. */
			if (filtered > 0 || filtered == -EINVAL)
				hid_input_report(nano->hid_touchpad, HID_INPUT_REPORT, p, len, 0);
			else if (filtered < 0)
				dev_warn_ratelimited(nano->dev, "Touchpad frame rejected: %d\n", filtered);
		} else if (*p >= 0x22) {
			int charge_ret = Nanosic_vendor_raw_data_inject((void *)p, len);
			if (charge_ret && charge_ret != -EOPNOTSUPP)
				dev_warn_ratelimited(nano->dev, "Charge MCU packet rejected: %d\n", charge_ret);
			sheng_nano_broadcast(nano, p, len);
			if (*p == 0x22 || *p == 0x23 || *p == 0x24) {
				u8 vendor[XM_WN8030_I2C_READ] = {0};
				memcpy(vendor + 3, p, len);
				nanosic_wn8030_handle_vendor(nano, vendor, len);
			}
		}
		p += len;
		left -= len;
	}
	return IRQ_HANDLED;
}

static int nanosic_wn8030_check_boot_id(struct nanosic_wn8030 *nanosic)
{
	int ret;
	unsigned int boot_id;

	ret = regmap_write(nanosic->regmap, 0x5c, 0x04);
	if (ret) {
		dev_err(nanosic->dev, "failed to set read address\n");
		return ret;
	}

	ret = regmap_read(nanosic->regmap, 0x5c, &boot_id);
	if (ret) {
		dev_err(nanosic->dev, "failed to read bootloader id\n");
		return ret;
	}

	if (boot_id != 0xc8) {
		dev_err(nanosic->dev, "unexpected bootloader id 0x%x\n", boot_id);
		return -EINVAL;
	}

	return 0;
}

static int nanosic_wn8030_get_boot_state(struct nanosic_wn8030 *nanosic)
{
	int ret;
	unsigned int boot_state;

	ret = regmap_write(nanosic->regmap, 0x5c, 0x05);
	if (ret) {
		dev_err(nanosic->dev, "failed to set read address\n");
		return ret;
	}

	ret = regmap_read(nanosic->regmap, 0x5c, &boot_state);
	if (ret) {
		dev_err(nanosic->dev, "failed to read bootloader state\n");
		return ret;
	}

	return boot_state;
}

static int nanosic_wn8030_send_header(struct nanosic_wn8030 *nanosic, const u8 *fw_buf, u32 *ram_code_size)
{
	int ret;
	struct wn8030_header_cmd hdr_cmd = {0};
	struct wn8030_bin_header *bin_hdr = (struct wn8030_bin_header *)&fw_buf[WN8030_HEADER_ADDR];

	hdr_cmd.cmd = 0x06; /* header send cmd */
	memcpy(hdr_cmd.flag, bin_hdr->flag, sizeof(hdr_cmd.flag));
	hdr_cmd.chip_id = 0x8140;
	hdr_cmd.ram_code_source = 0x2345;
	hdr_cmd.ram_code_dest = 0x40;
	hdr_cmd.ram_code_size = bin_hdr->ram_code_size;
	*ram_code_size = bin_hdr->ram_code_size;
	hdr_cmd.code_entry = bin_hdr->code_entry;
	hdr_cmd.code_entry = bin_hdr->code_entry;
	hdr_cmd.pid = bin_hdr->pid;
	hdr_cmd.vid = bin_hdr->vid;
	hdr_cmd.major_version = bin_hdr->major_version;
	hdr_cmd.minor_version = bin_hdr->minor_version;
	hdr_cmd.image_size = 0;
	hdr_cmd.ram_code_checksum = nanosic_wn8030_checksum32(&fw_buf[WN8030_CODE_ADDR], hdr_cmd.ram_code_size);
	hdr_cmd.image_checksum = 0;
	hdr_cmd.header_checksum = nanosic_wn8030_checksum8((u8 *)&hdr_cmd + 1, 64);

	ret = regmap_bulk_write(nanosic->regmap, 0x5c, &hdr_cmd, sizeof(hdr_cmd));
	if (ret) {
		dev_err(nanosic->dev, "failed to send header\n");
		return ret;
	}

	ret = nanosic_wn8030_get_boot_state(nanosic);
	if (ret < 0)
		return ret;

	if (ret != 0x1) {
		dev_err(nanosic->dev, "unexpected bootloader state after sending header %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static int nanosic_wn8030_send_data(struct nanosic_wn8030 *nanosic, const u8 *fw_buf, u32 ram_code_size)
{
	int ret;
	struct wn8030_send_cmd send_cmd;
	unsigned int offset = 0;

	while (ram_code_size > offset) {
		unsigned int send_size;

		memset(&send_cmd, 0, sizeof(send_cmd));
		send_cmd.cmd = 0x07;
		send_cmd.ram_dest = cpu_to_be32(offset);

		if (ram_code_size - offset < 503)
			send_size = ram_code_size - offset;
		else
			send_size = 503;

		send_cmd.size = cpu_to_be32(send_size);
		memcpy(send_cmd.data, &fw_buf[WN8030_CODE_ADDR+offset], send_size);

		ret = regmap_bulk_write(nanosic->regmap, 0x5c, &send_cmd, sizeof(send_cmd));
		if (ret) {
			dev_err(nanosic->dev, "failed to send data to 0x%x\n", offset);
			return ret;
		}
		offset += send_size;
	}

	return 0;
}

static int nanosic_wn8030_load_fw(struct nanosic_wn8030 *nanosic)
{
	int ret;
	const struct firmware *firmware;
	unsigned int ram_code_size;

	/* Keep the verified RAM image across suspend. Stock ueventd resolves
	 * /odm/firmware on the first load; direct-only requests cannot find it.
	 * Resume reuses this cache without entering the firmware fallback. */
	firmware = nanosic->ram_firmware;
	ret = firmware ? 0 : request_firmware(&firmware, "MCU_Upgrade.bin", nanosic->dev);
	if (!ret && !nanosic->ram_firmware)
		nanosic->ram_firmware = firmware;
	if (ret) {
		dev_err(nanosic->dev, "failed to load MCU firmware\n");
		return ret;
	}

	if (firmware->size < 0xA000) {
		dev_err(nanosic->dev, "invalid firmware size\n");
		ret = -EINVAL;
		goto exit;
	}

	if (strncmp(&firmware->data[0x7FC0], "NANO IC", 7) != 0) {
		dev_err(nanosic->dev, "invalid firmware header\n");
		ret = -EINVAL;
		goto exit;
	}

	ram_code_size = get_unaligned_le32(firmware->data + WN8030_HEADER_ADDR +
		offsetof(struct wn8030_bin_header, ram_code_size));
	if (!ram_code_size || (ram_code_size & 3) ||
	    ram_code_size > firmware->size - WN8030_CODE_ADDR) {
		ret = -EINVAL;
		goto exit;
	}
	ret = nanosic_wn8030_send_header(nanosic, firmware->data, &ram_code_size);
	if (ret)
		goto exit;

	ret = nanosic_wn8030_send_data(nanosic, firmware->data, ram_code_size);
	if (ret)
		goto exit;

	ret = regmap_write(nanosic->regmap, 0x5c, 0x08);
	if (ret) {
		dev_err(nanosic->dev, "failed to start firmware verification\n");
		goto exit;
	}

	msleep(10);

	ret = nanosic_wn8030_get_boot_state(nanosic);
	if (ret < 0)
		goto exit;

	if (ret != 0x3) {
		dev_err(nanosic->dev, "bootloader failed to verify firmware %d\n", ret);
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_write(nanosic->regmap, 0x5c, 0x09);
	if (ret) {
		dev_err(nanosic->dev, "failed to boot firmware\n");
		goto exit;
	}

	msleep(50);

exit:
	/* Released with the transport lifetime, after all workers stop. */

	return ret;
}

static int nanosic_wn8030_power_on(struct nanosic_wn8030 *nanosic)
{
	int ret;

	ret = regulator_bulk_enable(WN8030_NUM_SUPPLIES,
				    nanosic->supplies);
	if (ret) {
		dev_err(nanosic->dev, "failed to enable regulators\n");
		return ret;
	}

	gpiod_set_value_cansleep(nanosic->reset_gpio, 0);
	msleep(20); /* On Pad 6S Pro we only wait for WN8030 bootrom start */
	gpiod_set_value_cansleep(nanosic->sleep_gpio, 0);

	return 0;
}

static void nanosic_wn8030_power_off(struct nanosic_wn8030 *nanosic)
{
	gpiod_set_value_cansleep(nanosic->sleep_gpio, 1);
	gpiod_set_value_cansleep(nanosic->reset_gpio, 1);
	msleep(10);
	regulator_bulk_disable(WN8030_NUM_SUPPLIES, nanosic->supplies);
}

static void sheng_nano_plugin_worker(struct work_struct *work)
{
	struct nanosic_wn8030 *nano =
		container_of(work, struct nanosic_wn8030, plugin_worker.work);
	int wanted, ret;

	mutex_lock(&nano->conn_mutex);
	if (nano->removing || nano->plugin_requested < 0 ||
	    nano->plugin_requested == nano->plugin_sent) {
		mutex_unlock(&nano->conn_mutex);
		return;
	}
	wanted = nano->plugin_requested;
	mutex_unlock(&nano->conn_mutex);
	ret = qti_set_keyboard_plugin(wanted);
	mutex_lock(&nano->conn_mutex);
	if (!ret) {
		nano->plugin_sent = wanted;
		nano->plugin_retries = 0;
	} else {
		nano->plugin_retries++;
	}
	if (!nano->removing && nano->plugin_requested != nano->plugin_sent &&
	    nano->plugin_retries < 30)
		mod_delayed_work(system_wq, &nano->plugin_worker, ret ? HZ : 0);
	mutex_unlock(&nano->conn_mutex);
	if (!ret)
		do { } while (0);
	else
		dev_warn_ratelimited(nano->dev, "Keyboard charger event %d failed: %d\n", wanted, ret);
}

static void sheng_nano_firmware_worker(struct work_struct *work)
{
	struct nanosic_wn8030 *nano =
		container_of(work, struct nanosic_wn8030, fw_worker.work);
	int ret;

	mutex_lock(&nano->io_lock);
	if (nano->removing) {
		mutex_unlock(&nano->io_lock);
		return;
	}
	nano->fw_attempts++;
	/* Firmware is volatile; resetting the MCU does not flash its storage. */
	gpiod_set_value_cansleep(nano->reset_gpio, 1);
	msleep(10);
	gpiod_set_value_cansleep(nano->sleep_gpio, 0);
	gpiod_set_value_cansleep(nano->reset_gpio, 0);
	msleep(20);
	ret = nanosic_wn8030_check_boot_id(nano);
	if (!ret) {
		ret = nanosic_wn8030_get_boot_state(nano);
		if (ret > 0)
			ret = -EINVAL;
	}
	if (!ret)
		ret = nanosic_wn8030_load_fw(nano);
	if (!ret) {
		WRITE_ONCE(nano->firmware_generation, nano->firmware_generation + 1);
		nano->suspended = false;
		nano->ready = true;
		/* Original HAL discovery/authentication restores feature state after boot. */
		enable_irq(nano->client->irq);
		do { } while (0);
	} else if ((ret == -ENOENT || ret == -EAGAIN) && nano->fw_attempts < 60) {
		/* /odm may not be mounted, or PM may still block firmware requests. */
		schedule_delayed_work(&nano->fw_worker, HZ);
	} else {
		dev_err(nano->dev, "sheng_nanosic MCU firmware failed: %d attempt=%u\n",
			ret, nano->fw_attempts);
	}
	mutex_unlock(&nano->io_lock);
}

static int nanosic_wn8030_probe(struct i2c_client *client)
{
	struct nanosic_wn8030 *nano;
	struct device *dev = &client->dev;
	struct gpio_desc *irq_gpio;
	const char *model;
	int ret;

	if (of_property_read_string(of_root, "model", &model) ||
	    strcmp(model, "Qualcomm Technologies, Inc. 8550 sheng"))
		return -ENODEV;
	nano = kzalloc(sizeof(*nano), GFP_KERNEL);
	if (!nano)
		return -ENOMEM;
	nano->dev = dev;
	nano->client = client;
	kref_init(&nano->refs);
	mutex_init(&nano->io_lock);
	mutex_init(&nano->conn_mutex);
	mutex_init(&nano->clients_lock);
	INIT_LIST_HEAD(&nano->clients);
	ret = xiaomi_touchpad_edge_filter_param_init(dev->of_node);
	if (ret)
		goto free;
	INIT_DELAYED_WORK(&nano->fw_worker, sheng_nano_firmware_worker);
	INIT_DELAYED_WORK(&nano->plugin_worker, sheng_nano_plugin_worker);
	nano->plugin_requested = -1;
	nano->plugin_sent = -1;

	/* Reference DTS confirms reset and sleep are physically active-low. */
	nano->reset_gpio = sheng_nano_gpio(dev, "reset_pin", GPIOF_OUT_INIT_HIGH, true);
	if (IS_ERR(nano->reset_gpio)) {
		ret = PTR_ERR(nano->reset_gpio);
		goto free;
	}
	nano->sleep_gpio = sheng_nano_gpio(dev, "sleep_pin", GPIOF_OUT_INIT_HIGH, true);
	if (IS_ERR(nano->sleep_gpio)) {
		ret = PTR_ERR(nano->sleep_gpio);
		goto free;
	}
	nano->status_gpio = sheng_nano_gpio(dev, "status_pin", GPIOF_IN, false);
	if (IS_ERR(nano->status_gpio)) {
		ret = PTR_ERR(nano->status_gpio);
		goto free;
	}
	irq_gpio = sheng_nano_gpio(dev, "irq_pin", GPIOF_IN, false);
	if (IS_ERR(irq_gpio)) {
		ret = PTR_ERR(irq_gpio);
		goto free;
	}
	nano->irq_gpio = irq_gpio;
	client->irq = gpiod_to_irq(irq_gpio);
	if (client->irq < 0) {
		ret = client->irq;
		goto free;
	}
	/* Match shipping Nanosic_regulator_init's board-version selection.
	 * L6B and L12B are alternatives, not a three-rail bulk supply. */
	{
		struct device_node *chosen = of_find_node_by_path("/chosen");
		const char *bootargs, *digital_key;

		ret = chosen ? of_property_read_string(chosen, "bootargs", &bootargs) : -ENOENT;
		if (!ret)
			ret = sheng_nano_digital_supply_key(bootargs, &digital_key);
		of_node_put(chosen);
		if (ret)
			goto free;
		ret = of_property_read_string(dev->of_node, "nanosic,vdd-name",
					      &nano->supplies[0].supply);
		if (!ret)
			ret = of_property_read_string(dev->of_node, digital_key,
						      &nano->supplies[1].supply);
		if (ret)
			goto free;
		do { } while (0);
	}
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(nano->supplies), nano->supplies);
	if (ret)
		goto free;
	nano->regmap = devm_regmap_init_i2c(client, &nanosic_wn8030_regmap_config);
	if (IS_ERR(nano->regmap)) {
		ret = PTR_ERR(nano->regmap);
		goto free;
	}
	ret = nanosic_wn8030_power_on(nano);
	if (ret)
		goto free;
	i2c_set_clientdata(client, nano);
	ret = devm_request_threaded_irq(dev, client->irq, NULL,
		nanosic_wn8030_handler, IRQF_ONESHOT | IRQF_TRIGGER_RISING |
		IRQF_NO_AUTOEN, "sheng_nanosic", nano);
	if (ret)
		goto power_off;
	nano->raw_misc.minor = MISC_DYNAMIC_MINOR;
	nano->raw_misc.name = "nanodev0";
	nano->raw_misc.fops = &sheng_nano_fops;
	nano->raw_misc.parent = dev;
	nano->raw_misc.groups = sheng_nano_state_groups;
	/* Stock init assigns system ownership/mode on boot. */
	ret = misc_register(&nano->raw_misc);
	if (ret)
		goto free_irq;
	ret = device_init_wakeup(dev, true);
	if (ret) {
		misc_deregister(&nano->raw_misc);
		goto free_irq;
	}
	WRITE_ONCE(charge_nano, nano);
	ret = Nanosic_charge_i2c_write_callback_register(sheng_nano_charge_write);
	if (ret) {
		WRITE_ONCE(charge_nano, NULL);
		device_init_wakeup(dev, false);
		misc_deregister(&nano->raw_misc);
		if (ret == -ENODEV)
			ret = -EPROBE_DEFER;
		goto free_irq;
	}
	schedule_delayed_work(&nano->fw_worker, HZ);
	return 0;

free_irq:
	devm_free_irq(dev, client->irq, nano);
power_off:
	nanosic_wn8030_power_off(nano);
free:
	dev_err_probe(dev, ret, "sheng_nanosic probe failed\n");
	kref_put(&nano->refs, sheng_nano_free);
	return ret;
}

static void nanosic_wn8030_remove(struct i2c_client *client)
{
	struct nanosic_wn8030 *nano = i2c_get_clientdata(client);
	bool ready;

	mutex_lock(&nano->io_lock);
	sheng_nano_disconnect_clients(nano);
	ready = nano->ready;
	nano->ready = false;
	mutex_unlock(&nano->io_lock);
	Nanosic_charge_i2c_write_callback_unregister();
	WRITE_ONCE(charge_nano, NULL);
	misc_deregister(&nano->raw_misc);
	cancel_delayed_work_sync(&nano->fw_worker);
	if (ready && !nano->suspended)
		disable_irq(client->irq);
	/* Drop the wake reference before releasing the IRQ descriptor. */
	if (sheng_nano_set_irq_wake(nano, false))
		dev_warn(&client->dev, "Failed to disarm MCU wake IRQ\n");
	device_init_wakeup(&client->dev, false);
	devm_free_irq(&client->dev, client->irq, nano);
	cancel_delayed_work_sync(&nano->plugin_worker);
	if (nano->plugin_sent == 1)
		qti_set_keyboard_plugin(0);
	sheng_nano_release_hids(nano);
	nanosic_wn8030_power_off(nano);
	kref_put(&nano->refs, sheng_nano_free);
}

static int nanosic_wn8030_suspend(struct device *dev)
{
	struct nanosic_wn8030 *nano = dev_get_drvdata(dev);
	int ret;

	cancel_delayed_work_sync(&nano->fw_worker);
	if (nano->ready) {
		disable_irq(nano->client->irq);
		ret = sheng_nano_set_irq_wake(nano, device_may_wakeup(dev));
		if (ret) {
			enable_irq(nano->client->irq);
			return ret;
		}
	}
	mutex_lock(&nano->io_lock);
	nano->suspended = true;
	gpiod_set_value_cansleep(nano->sleep_gpio, 1);
	mutex_unlock(&nano->io_lock);
	return 0;
}

static int nanosic_wn8030_resume(struct device *dev)
{
	struct nanosic_wn8030 *nano = dev_get_drvdata(dev);
	int ret = 0, wake_ret;

	ret = sheng_nano_set_irq_wake(nano, false);
	mutex_lock(&nano->io_lock);
	nano->suspended = false;
	if (!nano->ready) {
		nano->fw_attempts = 0;
		mutex_unlock(&nano->io_lock);
		schedule_delayed_work(&nano->fw_worker, msecs_to_jiffies(100));
		return ret;
	}
	/* Keep the IRQ disabled while polling, so the wake edge cannot be consumed. */
	wake_ret = sheng_nano_wake_locked(nano);
	sheng_nano_recover_after_timeout_locked(nano, wake_ret);
	mutex_unlock(&nano->io_lock);
	/* Deliver the first real packet; never discard it as a wake acknowledgement. */
	enable_irq(nano->client->irq);
	if (wake_ret)
		dev_warn(dev, "MCU resume handshake failed: %d\n", wake_ret);
	return ret ? ret : wake_ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nanosic_wn8030_pm_ops,
	nanosic_wn8030_suspend, nanosic_wn8030_resume);

static const struct of_device_id nanosic_wn8030_of_match[] = {
	{ .compatible = "nanosic,803" },
	{ }
};
MODULE_DEVICE_TABLE(of, nanosic_wn8030_of_match);

static struct i2c_driver nanosic_wn8030_driver = {
	.driver = {
		.name = "nanosic_driver",
		.of_match_table = nanosic_wn8030_of_match,
		.pm = pm_sleep_ptr(&nanosic_wn8030_pm_ops),
	},
	.probe = nanosic_wn8030_probe,
	.remove = nanosic_wn8030_remove,
};
module_i2c_driver(nanosic_wn8030_driver);
MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("Sheng WN8030 keyboard MCU with stock HyperOS transport");
MODULE_LICENSE("GPL v2");
