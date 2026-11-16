
#include "linux/battmngr/platform_class.h"

static struct class *battmngr_class;

static ssize_t name_show(struct device *dev,
  				    struct device_attribute *attr, char *buf)
{
  	struct battmngr_device *battmg_dev = to_battmngr_device(dev);

  	return snprintf(buf, 20, "%s\n",
  		       battmg_dev->props.alias_name ?
  		       battmg_dev->props.alias_name : "anonymous");
}

static DEVICE_ATTR_RO(name);

static struct attribute *battmngr_class_attrs[] = {
  	&dev_attr_name.attr,
  	NULL,
};

static const struct attribute_group battmngr_group = {
  	.attrs = battmngr_class_attrs,
};

static const struct attribute_group *battmngr_groups[] = {
  	&battmngr_group,
  	NULL,
};

static void battmngr_device_release(struct device *dev)
{
  	struct battmngr_device *chg_dev = to_battmngr_device(dev);

  	srcu_cleanup_notifier_head(&chg_dev->evt_nh);
	kfree(chg_dev);
}

#include "sheng_battmngr_calls.h"

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg_soc, int, fg_soc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg_soc);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg_curr, int, fg_curr)
EXPORT_SYMBOL(battmngr_qtiops_get_fg_curr);


SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg_volt, int, fg_volt)
EXPORT_SYMBOL(battmngr_qtiops_get_fg_volt);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg_temp, int, fg_temp)
EXPORT_SYMBOL(battmngr_qtiops_get_fg_temp);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_charge_status, int, charge_status)
EXPORT_SYMBOL(battmngr_qtiops_get_charge_status);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_ibatt, int, fg1_ibatt)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_ibatt);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_ibatt, int, fg2_ibatt)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_ibatt);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_volt, int, fg1_volt)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_volt);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_volt, int, fg2_volt)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_volt);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_fcc, int, fg1_fcc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_fcc);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_fcc, int, fg2_fcc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_fcc);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_rm, int, fg1_rm)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_rm);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_rm, int, fg2_rm)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_rm);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_raw_soc, int, fg1_raw_soc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_raw_soc);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_raw_soc, int, fg2_raw_soc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_raw_soc);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_soc, int, fg1_soc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_soc);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_soc, int, fg2_soc)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_soc);

SHENG_BATTMNGR_SETTER(battmngr_qtiops_set_fg1_fastcharge, set_fg1_fastcharge)
EXPORT_SYMBOL(battmngr_qtiops_set_fg1_fastcharge);

SHENG_BATTMNGR_SETTER(battmngr_qtiops_set_fg2_fastcharge, set_fg2_fastcharge)
EXPORT_SYMBOL(battmngr_qtiops_set_fg2_fastcharge);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_fastcharge, int, get_fg1_fastcharge)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_fastcharge);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg1_temp, int, fg1_temp)
EXPORT_SYMBOL(battmngr_qtiops_get_fg1_temp);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_fg2_temp, int, fg2_temp)
EXPORT_SYMBOL(battmngr_qtiops_get_fg2_temp);

SHENG_BATTMNGR_SETTER(battmngr_qtiops_set_fg_suspend, fg_suspend)
EXPORT_SYMBOL(battmngr_qtiops_set_fg_suspend);

SHENG_BATTMNGR_SETTER(battmngr_qtiops_set_term_cur, set_term_cur)
EXPORT_SYMBOL(battmngr_qtiops_set_term_cur);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_batt_auth, int, get_batt_auth)
EXPORT_SYMBOL(battmngr_qtiops_get_batt_auth);

SHENG_BATTMNGR_GETTER(battmngr_qtiops_get_chip_ok, int, get_chip_ok)
EXPORT_SYMBOL(battmngr_qtiops_get_chip_ok);

bool check_qti_ops(struct battmngr_device **battmg_dev)
{

	if (!battmg_dev)
		return false;
	if (*battmg_dev) {
		bool ready;
		mutex_lock(&(*battmg_dev)->ops_lock);
		ready = (*battmg_dev)->ops != NULL;
		mutex_unlock(&(*battmg_dev)->ops_lock);
		return ready;
	}

	*battmg_dev = get_adapter_by_name("qti_ops");
	if (!(*battmg_dev)) {
		class_err("xm charge get battmngr dev is fail");
		return false;
	}

	class_info("xm charge get battmngr dev is success");
	return true;
}
EXPORT_SYMBOL(check_qti_ops);

SHENG_BATTMNGR_GETTER(battmngr_noops_get_online, u8, online)
EXPORT_SYMBOL(battmngr_noops_get_online);

SHENG_BATTMNGR_GETTER(battmngr_noops_get_real_type, int, real_type)
EXPORT_SYMBOL(battmngr_noops_get_real_type);

SHENG_BATTMNGR_GETTER(battmngr_noops_get_usb_type, int, usb_type)
EXPORT_SYMBOL(battmngr_noops_get_usb_type);

SHENG_BATTMNGR_GETTER(battmngr_noops_get_input_curr_limit, int, input_curr_limit)
EXPORT_SYMBOL(battmngr_noops_get_input_curr_limit);

SHENG_BATTMNGR_GETTER(battmngr_noops_get_power_max, int, power_max)
EXPORT_SYMBOL(battmngr_noops_get_power_max);

struct battmngr_device* check_nano_ops(void)
{
	struct battmngr_device *battmg_dev = get_adapter_by_name("nano_ops");

	if (!battmg_dev) {
		class_err("nano get battmngr dev is fail");
		return NULL;
	}

	class_info("nano get battmngr dev is success");
	return battmg_dev;
}
EXPORT_SYMBOL(check_nano_ops);

/**
 * battmngr_device_register - create and register a new object of
 *   battmngr_device class.
 * @name: the name of the new object
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use.
 * The methods may retrieve it by using adapter_get_data(adapter_dev).
 * @ops: the charger operations structure.
 *
 * Creates and registers new charger device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct battmngr_device *battmngr_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct battmngr_ops *ops,
		const struct battmngr_properties *props)
{
	struct battmngr_device *battmg_dev = NULL;
	static struct lock_class_key key;
	struct srcu_notifier_head *head = NULL;
	int rc;

	class_info("%s: name=%s\n", __func__, name);
	battmg_dev = kzalloc(sizeof(*battmg_dev), GFP_KERNEL);
	if (!battmg_dev)
		return ERR_PTR(-ENOMEM);

	if (IS_ERR_OR_NULL(battmngr_class)) {
		kfree(battmg_dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	mutex_init(&battmg_dev->ops_lock);
	device_initialize(&battmg_dev->dev);
	battmg_dev->dev.class = battmngr_class;
	battmg_dev->dev.parent = parent;
	battmg_dev->dev.release = battmngr_device_release;
	head = &battmg_dev->evt_nh;
	srcu_init_notifier_head(head);
	lockdep_init_map(&(&head->srcu)->dep_map, name, &key, 0);
	rc = dev_set_name(&battmg_dev->dev, "%s", name);
	if (rc) {
		put_device(&battmg_dev->dev);
		return ERR_PTR(rc);
	}
	dev_set_drvdata(&battmg_dev->dev, devdata);
	battmg_dev->ops = ops;

	/* Copy properties */
	if (props) {
		memcpy(&battmg_dev->props, props,
		       sizeof(struct battmngr_properties));
	}
	rc = device_add(&battmg_dev->dev);
	if (rc) {
		put_device(&battmg_dev->dev);
		return ERR_PTR(rc);
	}
    class_info("%s: is successful\n", __func__);
	return battmg_dev;
}
EXPORT_SYMBOL(battmngr_device_register);

/**
 * battmngr_device_unregister - unregisters a switching charger device
 * object.
 * @adapter_dev: the switching charger device object to be unregistered
 * and freed.
 *
 * Unregisters a previously registered via battmngr_device_register object.
 */
void battmngr_device_unregister(struct battmngr_device *adapter_dev)
{
	if (!adapter_dev)
		return;

	sheng_battmngr_stop_ops(adapter_dev);
	device_unregister(&adapter_dev->dev);
}
EXPORT_SYMBOL(battmngr_device_unregister);

static int battmngr_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct battmngr_device *get_adapter_by_name(const char *name)
{
	struct device *dev = NULL;

	if (!name || IS_ERR_OR_NULL(battmngr_class))
		return NULL;
	dev = class_find_device(battmngr_class, NULL, name,
				battmngr_match_device_by_name);

	return dev ? to_battmngr_device(dev) : NULL;

}
EXPORT_SYMBOL(get_adapter_by_name);

static void battmngr_class_exit(void)
{
	class_destroy(battmngr_class);
}

static int battmngr_class_init(void)
{
	battmngr_class = class_create("Battmngr_class");
	if (IS_ERR(battmngr_class)) {
		class_err("Unable to create Battmngr_class; errno = %ld\n",
			PTR_ERR(battmngr_class));
		return PTR_ERR(battmngr_class);
	}
	battmngr_class->dev_groups = battmngr_groups;

	return 0;
}

module_init(battmngr_class_init);
module_exit(battmngr_class_exit);
 
MODULE_DESCRIPTION("Battmngr Class Device");
MODULE_AUTHOR("litianpeng6@xiaomi.com");
MODULE_LICENSE("GPL v2");

