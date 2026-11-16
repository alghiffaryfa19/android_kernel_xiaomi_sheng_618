

#include "ntag_common.h"

struct ntag_dev *ntag_dev = NULL;
void read_wire_data_test(void);
int ntag5_read_data(u8 *buf, int count, int flags);
int ntag5_write_data(u8 *buf, int count, int flags);
int read_register(int add, u8 reg, u8 *val);
int write_register(int add, char reg, char mask, char *val);
unsigned char ntag5_i2c_blockWrite(unsigned short addr, unsigned char *buf, unsigned char size);
unsigned char ntag5_i2c_blockRead(unsigned short addr, unsigned short count, unsigned char *buf, unsigned short size);
static ssize_t nfc_i2c_dev_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t nfc_i2c_dev_write(struct file *, const char __user *, size_t, loff_t *);
static int ntag5_irq_fasync(int, struct file *, int);
static int ntag5_fasync_release(struct inode *, struct file *);


static DEFINE_MUTEX(ntag_api_lock);
static int ntag_transfer_locked(struct ntag_dev *dev, u8 *buf, int count, int flags)
{
 struct i2c_msg msg;
 int ret;
 if (!dev || dev != ntag_dev) return -ENODEV;
 if (!buf || count <= 0 || count > MAX_BUFFER_SIZE) return -EINVAL;
 msg.addr = dev->i2c_dev.client->addr;
 msg.flags = flags;
 msg.len = count;
 msg.buf = buf;
 ret = i2c_transfer(dev->i2c_dev.client->adapter, &msg, 1);
 return ret == 1 ? count : ret < 0 ? ret : -EIO;
}

int ntag5_read_data(u8 *buf, int count, int flags)
{
 int ret;
 mutex_lock(&ntag_api_lock);
 ret = ntag_transfer_locked(ntag_dev, buf, count, flags | I2C_M_RD);
 mutex_unlock(&ntag_api_lock);
 return ret;
}

int ntag5_write_data(u8 *buf, int count, int flags)
{
 int ret;
 mutex_lock(&ntag_api_lock);
 ret = ntag_transfer_locked(ntag_dev, buf, count, flags);
 mutex_unlock(&ntag_api_lock);
 return ret;
}

static void ntag_release(struct kref *ref)
{ kfree(container_of(ref, struct ntag_dev, ref)); }

static int ntag_open(struct inode *inode, struct file *filp)
{
 int ret = -ENODEV;
 mutex_lock(&ntag_api_lock);
 if (ntag_dev && ntag_dev->ready && inode->i_cdev == &ntag_dev->c_dev) {
  kref_get(&ntag_dev->ref);
  filp->private_data = ntag_dev;
  ret = stream_open(inode, filp);
 }
 mutex_unlock(&ntag_api_lock);
 return ret;
}

int read_register(int add, u8 reg, u8 *val)
{
	u8 buffer[3];
	buffer[0] = add >> 8;
	buffer[1] = add & 0xFF;
	buffer[2] = reg;
	if (ntag5_write_data(buffer, 3, 0) != 3) {
		return 1;
	}

	if (ntag5_read_data(val, 1, 0) != 1) {
		return 1;
	}
	return 0;
}

int write_register(int add, char reg, char mask, char *val)
{
	u8 buffer[5];
	buffer[0] = add >> 8;
	buffer[1] = add & 0xFF;
	buffer[2] = reg;
	buffer[3] = mask;
	buffer[4] = val[0];
	if (ntag5_write_data(buffer, 5, 0) != 5) {
		return 1;
	}
	/* Wait for write operation completion */
	mdelay(10);
	return 0;
}

/**************************************************************************************************
** IN:     unsigned short iicBlockAddr    block number to be written
**         unsigned char *wtBuf           data to be written
**         unsigned char wtBuf_size       data buffer size
** NOTE    IMPORTANT - host shall wait for ~4ms after stop_condition to have eeprom programming finished
**         write 4 bytes to EEPROM or Configuration area
**************************************************************************************************/
unsigned char ntag5_i2c_blockWrite(unsigned short iicBlockAddr, unsigned char *wtBuf, unsigned char wtBuf_size)
{
	unsigned char status = 1;
	unsigned char buf[7];

	pr_info("%s : ntag5_i2c_blockWrite iicBlockAddr = %d!\n", __func__, iicBlockAddr);
	pr_info("%s : ntag5_i2c_blockWrite wtBuf_size = %d!\n", __func__, wtBuf_size);

	if (wtBuf_size < NTAG5_BLOCK_SIZE) {
		pr_err("%s : ntag5_i2c_blockWrite STATUS_BUFFER_SIZE_NOT_SUPPORT!\n", __func__);
		return STATUS_BUFFER_SIZE_NOT_SUPPORT;
	}

	// write one I2C Block: 1 + 4bytes
	buf[0] = iicBlockAddr >> 8;	// block address: memAddr H
	buf[1] = (unsigned char)iicBlockAddr;	// block address: memAddr L
	memcpy(&buf[2], wtBuf, 4);	// data to be written

	status = ntag5_write_data(buf, 6, 0);
	pr_info("%s : ntag5_i2c_blockWrite status = %d!\n", __func__, status);

	//host shall wait for ~4ms after stop_condition
	mdelay(4);

	return status;
}

/**************************************************************************************************
** IN:          unsigned short iicBlockAddr     starting block number to be read
**              unsigned char nrOfBlocks        number of blocks to be read, 4 bytes per block
**              unsigned char *rdBuf            buffer to store read data
**              unsigned short rdBuf_size       buffer size
** OUT:         ntag5 will continuely send back data unless receive NACK or Stop from Host
** NOTE         make sure the read buffer does not overflow
**************************************************************************************************/
unsigned char ntag5_i2c_blockRead(unsigned short iicBlockAddr, unsigned short nrOfBlocks, unsigned char *rdBuf, unsigned short rdBuf_size)
{
	unsigned char status = 1;
	unsigned char addr[2];

	pr_info("%s : ntag5_i2c_blockRead iicBlockAddr = %d!\n", __func__, iicBlockAddr);
	pr_info("%s : ntag5_i2c_blockRead nrOfBlocks = %d!\n", __func__, nrOfBlocks);
	pr_info("%s : ntag5_i2c_blockRead rdBuf_size = %d!\n", __func__, rdBuf_size);

	if (rdBuf_size < NTAG5_BLOCK_SIZE * nrOfBlocks) {
		pr_err("%s : ntag5_i2c_blockRead STATUS_BUFFER_SIZE_NOT_SUPPORT!\n", __func__);
		return STATUS_BUFFER_SIZE_NOT_SUPPORT;
	}

	addr[0] = iicBlockAddr >> 8;	// addr High
	addr[1] = (unsigned char)iicBlockAddr;

	// write Block Address: 2bytes
	status = ntag5_write_data(addr, 2, 0);
	if (status)
	{
		// read block data, total bytes = nrOfBlocks * 4
		status = ntag5_read_data(rdBuf, nrOfBlocks * 4, 0);
		if (true) {
			pr_err("%s : ntag5_i2c_blockRead status = %d!\n", __func__, status);
		}
	}

	mdelay(4);
	return status;
}

int i2c_disable_irq(void)
{
	unsigned long flags;
	pr_info("%s : i2c_disable_irq enter  !\n", __func__);
	spin_lock_irqsave(&ntag_dev->i2c_dev.irq_enabled_lock, flags);
	if (ntag_dev->i2c_dev.irq_enabled) {
		disable_irq_nosync(ntag_dev->i2c_dev.client->irq);
		ntag_dev->i2c_dev.irq_enabled = false;
		pr_err("%s : i2c_disable_irq success !\n", __func__);
	}
	spin_unlock_irqrestore(&ntag_dev->i2c_dev.irq_enabled_lock, flags);
	pr_info("%s : i2c_disable_irq exit   !\n", __func__);
	return 0;
}

/**
 * i2c_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: int
 */
int i2c_enable_irq(void)
{
	unsigned long flags;
	pr_info("%s : i2c_enable_irq enter  !\n", __func__);
	spin_lock_irqsave(&ntag_dev->i2c_dev.irq_enabled_lock, flags);
	if (!ntag_dev->i2c_dev.irq_enabled) {
		ntag_dev->i2c_dev.irq_enabled = true;
		enable_irq(ntag_dev->i2c_dev.client->irq);
		pr_err("%s : i2c_enable_irq success !\n", __func__);

	}
	spin_unlock_irqrestore(&ntag_dev->i2c_dev.irq_enabled_lock, flags);
	pr_info("%s : i2c_enable_irq exit   !\n", __func__);
	return 0;
}


int i2c_read(struct ntag_dev *dev, char *buf, size_t count, int timeout)
{
 int ret;
 if (count > MAX_BUFFER_SIZE || timeout < 0) return -EINVAL;
 mutex_lock(&ntag_api_lock);
 ret = ntag_transfer_locked(dev, (u8 *)buf, count, I2C_M_RD);
 mutex_unlock(&ntag_api_lock);
 return ret;
}

static ssize_t nfc_i2c_dev_read(struct file *filp, char __user *buf,
 size_t count, loff_t *offset)
{
 u8 kernel_buf[MAX_BUFFER_SIZE];
 int ret;
 if (!count) return 0;
 if (count > sizeof(kernel_buf)) return -EMSGSIZE;
 mutex_lock(&ntag_api_lock);
 ret = ntag_transfer_locked(filp->private_data, kernel_buf, count, I2C_M_RD);
 mutex_unlock(&ntag_api_lock);
 if (ret < 0) return ret;
 return copy_to_user(buf, kernel_buf, count) ? -EFAULT : ret;
}

static ssize_t nfc_i2c_dev_write(struct file *filp, const char __user *buf,
 size_t count, loff_t *offset)
{
 u8 kernel_buf[6];
 int ret;
 if (!count) return 0;
 if (count != 2 && count != 5 && count != 6) return -EINVAL;
 if (copy_from_user(kernel_buf, buf, count)) return -EFAULT;
 mutex_lock(&ntag_api_lock);
 ret = ntag_transfer_locked(filp->private_data, kernel_buf, count, 0);
 mutex_unlock(&ntag_api_lock);
 return ret;
}

static irqreturn_t i2c_irq_handler(int irq, void *dev_id)
{

	unsigned long flags;
	struct ntag_dev *ntag_dev = dev_id;
	spin_lock_irqsave(&ntag_dev->i2c_dev.irq_enabled_lock, flags);
	if(ntag_dev->fasync_queue) {
		/* 在中断服务函数中向应用层发送SIGIO信号-异步通知 */
		pr_info("%s kill_fasync, inform upper layer", __func__);
		kill_fasync(&(ntag_dev->fasync_queue), SIGIO, POLL_IN);
	}
	spin_unlock_irqrestore(&ntag_dev->i2c_dev.irq_enabled_lock, flags);
	wake_up(&ntag_dev->read_wq);
	return IRQ_HANDLED;
}

static int ntag5_irq_fasync(int fd, struct file *file, int on)
{
 struct ntag_dev *dev = file->private_data;
 int ret;
 mutex_lock(&ntag_api_lock);
 if (!dev || (on && dev != ntag_dev)) ret = -ENODEV;
 else ret = fasync_helper(fd, file, on, &dev->fasync_queue);
 mutex_unlock(&ntag_api_lock);
 return ret;
}

static int ntag5_fasync_release(struct inode *inode, struct file *filp)
{
 struct ntag_dev *dev = filp->private_data;
 int ret = ntag5_irq_fasync(-1, filp, 0);
 filp->private_data = NULL;
 if (dev) kref_put(&dev->ref, ntag_release);
 return ret;
}

static const struct file_operations ntag_i2c_dev_fops = {
	.owner = THIS_MODULE,
	.open = ntag_open,
	.read = nfc_i2c_dev_read,
	.write = nfc_i2c_dev_write,
	.fasync = ntag5_irq_fasync,
	.release = ntag5_fasync_release,
};

void read_wire_data_test(void)
{
	int count = 10;

	int ret = -1;

	while (count--) {
		i2c_enable_irq();
		if (gpio_get_value(ntag_dev->configs.gpio.pu)) {
			ret = wait_event_interruptible(ntag_dev->read_wq,
				!ntag_dev->i2c_dev.irq_enabled);
			if (ret) {
				pr_err("%s unexpeted wakeup of read wq, try wait again.\n", __func__);
				return;
			}
		}
		if (!gpio_get_value(ntag_dev->configs.gpio.pu)) {
			pr_err("%s : gpio_get_value = 0!\n", __func__);
			break;
		}
		i2c_disable_irq();
		pr_err("%s unexpeted wakeup of read wq, try wait again, count = %d\n", __func__, count);
	}

}

static int nfc_i2c_dev_probe(struct i2c_client *client)
{
 struct ntag_dev *dev;
 struct platform_configs configs = {0};
 bool pu = false, hpd = false, irq = false, misc = false;
 char buffer = 1;
 int ret;
 ret = nfc_parse_dt(&client->dev, &configs);
 if (ret) return ret;
 if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) return -ENODEV;
 dev = kzalloc(sizeof(*dev), GFP_KERNEL);
 if (!dev) return -ENOMEM;
 kref_init(&dev->ref);
 dev->configs = configs;
 dev->nfc_device = &client->dev;
 dev->i2c_dev.client = client;
 dev->nfc_enable_intr = i2c_enable_irq;
 dev->nfc_disable_intr = i2c_disable_irq;
 init_waitqueue_head(&dev->read_wq);
 spin_lock_init(&dev->i2c_dev.irq_enabled_lock);
 mutex_lock(&ntag_api_lock);
 if (ntag_dev) { mutex_unlock(&ntag_api_lock); ret = -EBUSY; goto cleanup; }
 ntag_dev = dev;
 mutex_unlock(&ntag_api_lock);
 ret = ntag5_gpio_init(dev);
 if (ret) goto cleanup;
 ret = configure_gpio(configs.gpio.pu, GPIO_IRQ);
 if (ret < 0) goto cleanup;
 pu = true;
 client->irq = ret;
 dev->i2c_dev.irq_enabled = true;
 ret = request_irq(client->irq, i2c_irq_handler, IRQF_TRIGGER_FALLING, client->name, dev);
 if (ret) goto cleanup;
 irq = true;
 ret = configure_gpio(configs.gpio.hpd, GPIO_HPD);
 if (ret < 0) goto cleanup;
 hpd = true;
 ret = nfc_ldo_config(&client->dev, dev);
 if (ret) goto cleanup;
 ret = write_register(0x10A8, 0, 0xFF, &buffer);
 if (ret) { ret = -EIO; goto cleanup; }
 ret = nfc_misc_register(dev, &ntag_i2c_dev_fops, DEV_COUNT, NFC_CHAR_DEV_NAME, CLASS_NAME);
 if (ret) goto cleanup;
 misc = true;
 i2c_set_clientdata(client, dev);
 mutex_lock(&ntag_api_lock);
 dev->ready = true;
 mutex_unlock(&ntag_api_lock);
 return 0;
cleanup:
 mutex_lock(&ntag_api_lock);
 if (ntag_dev == dev) ntag_dev = NULL;
 mutex_unlock(&ntag_api_lock);
 if (misc) nfc_misc_unregister(dev, DEV_COUNT);
 if (irq) free_irq(client->irq, dev);
 if (!IS_ERR_OR_NULL(dev->reg)) { if (dev->is_vreg_enabled) regulator_disable(dev->reg); regulator_put(dev->reg); }
 if (hpd) gpio_free(configs.gpio.hpd);
 if (pu) gpio_free(configs.gpio.pu);
 kref_put(&dev->ref, ntag_release);
 return ret;
}

static void nfc_i2c_dev_remove(struct i2c_client *client)
{
 struct ntag_dev *dev = i2c_get_clientdata(client);
 if (!dev) return;
 mutex_lock(&ntag_api_lock);
 dev->ready = false;
 ntag_dev = NULL;
 mutex_unlock(&ntag_api_lock);
 free_irq(client->irq, dev);
 nfc_misc_unregister(dev, DEV_COUNT);
 if (!IS_ERR_OR_NULL(dev->reg)) { if (dev->is_vreg_enabled) regulator_disable(dev->reg); regulator_put(dev->reg); }
 gpio_free(dev->configs.gpio.hpd);
 gpio_free(dev->configs.gpio.pu);
 i2c_set_clientdata(client, NULL);
 kref_put(&dev->ref, ntag_release);
}

static const struct of_device_id nfc_i2c_dev_match_table[] = {
	{.compatible = NTAG5_I2C_DRV_STR,},
	{}
};

MODULE_DEVICE_TABLE(of, nfc_i2c_dev_match_table);

static struct i2c_driver nfc_i2c_dev_driver = {
	.probe = nfc_i2c_dev_probe,
	.remove = nfc_i2c_dev_remove,
	.driver = {
		.name = NTAG5_I2C_DRV_STR,
		.of_match_table = nfc_i2c_dev_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

static int __init my_i2c_drv_init(void)
{

	int ret = 0;
	pr_info("loading NTAG5 I2C driver\n");
	ret = i2c_add_driver(&nfc_i2c_dev_driver);
	if (ret != 0) {
		pr_err("NFC I2C add driver error ret %d\n", ret);
	}
	return ret;
}

module_init(my_i2c_drv_init);

static void __exit my_i2c_drv_exit(void) {

	pr_info("my_i2c_drv_exit NTAG5 I2C driver\n");
	i2c_del_driver(&nfc_i2c_dev_driver);
	return;
}

module_exit(my_i2c_drv_exit);

MODULE_DESCRIPTION("NFC NTAG5 I2C driver");
MODULE_LICENSE("GPL v2");
