
#include "inc/xm_battmngr_init.h"

struct xm_battmngr *g_battmngr;
EXPORT_SYMBOL(g_battmngr);
static DEFINE_MUTEX(manager_owner_lock);

static int battmngr_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct battmngr_notify *noti_data = data;
	struct xm_battmngr *manager = container_of(nb, struct xm_battmngr, battmngr_nb);
	int previous;
	int ret = 0;

	pr_err("%s: event %lu\n", __func__, event);

	switch (event) {
	case BATTMNGR_EVENT_FG:
		ret = battery_process_event_fg(noti_data);
		break;
	case BATTMNGR_EVENT_MAINCHG:
		ret = charger_process_event_mainchg(noti_data);
		break;
	case BATTMNGR_EVENT_MCU:
		ret = charger_process_event_mcu(noti_data);
		break;
	case BATTMNGR_EVENT_IRQ:
		ret = charger_process_event_irq(noti_data);
		break;
	case BATTMNGR_EVENT_THERMAL:
		previous = manager->charger.system_temp_level;
		manager->charger.system_temp_level = noti_data->misc_msg.thermal_level;
		ret = xm_charger_thermal(&manager->charger);
		if (ret < 0) manager->charger.system_temp_level = previous;
		break;
	default:
		break;
	}

	return notifier_from_errno(ret < 0 ? ret : 0);
}

static int xm_battmngr_probe(struct platform_device *pdev)
{
 struct xm_battmngr *manager;
 int ret;
 mutex_lock(&manager_owner_lock);
 if (g_battmngr) { ret = -EBUSY; goto unlock; }
 if (!check_g_bcdev_ops()) { ret = -EPROBE_DEFER; goto unlock; }
 manager = devm_kzalloc(&pdev->dev, sizeof(*manager), GFP_KERNEL);
 if (!manager) { ret = -ENOMEM; goto unlock; }
 manager->dev = &pdev->dev;
 manager->charger.dev = &pdev->dev;
 mutex_init(&manager->battmngr_noti.notify_lock);
 ret = xm_charger_init(&manager->charger);
 if (ret < 0) goto unlock;
 manager->battmngr_nb.notifier_call = battmngr_notifier_call;
 g_xm_charger = &manager->charger;
 g_battmngr = manager;
 ret = battmngr_notifier_register(&manager->battmngr_nb);
 if (ret < 0) goto clear;
 ret = battmngr_notify_publish(&manager->battmngr_noti);
 if (ret) goto unregister;
 platform_set_drvdata(pdev, manager);
 goto unlock;
unregister:
 battmngr_notifier_unregister(&manager->battmngr_nb);
clear:
 g_xm_charger = NULL; g_battmngr = NULL;
 xm_charger_deinit(&manager->charger);
unlock:
 mutex_unlock(&manager_owner_lock);
 return ret;
}

static void xm_battmngr_remove(struct platform_device *pdev)
{
 struct xm_battmngr *manager;
 mutex_lock(&manager_owner_lock);
 manager = platform_get_drvdata(pdev);
 if (!manager) goto unlock;
 battmngr_notify_unpublish(&manager->battmngr_noti);
 battmngr_notifier_unregister(&manager->battmngr_nb);
 g_xm_charger = NULL; g_battmngr = NULL;
 xm_charger_deinit(&manager->charger);
 platform_set_drvdata(pdev, NULL);
unlock:
 mutex_unlock(&manager_owner_lock);
}

static void xm_battmngr_shutdown(struct platform_device *pdev)
{ xm_battmngr_remove(pdev); }

static const struct of_device_id match_table[] = {
	{.compatible = "xiaomi,battmngr"},
	{},
};

MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver xm_battmngr_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "xm_battmngr",
		.of_match_table = match_table,
	},
	.probe = xm_battmngr_probe,
	.remove = xm_battmngr_remove,
	.shutdown = xm_battmngr_shutdown,
};

static int __init xm_battmngr_init(void)
{
	pr_err("%s: cxlcheck1\n", __func__);
	return platform_driver_register(&xm_battmngr_driver);
}
module_init(xm_battmngr_init);

static void __exit xm_battmngr_exit(void)
{
	platform_driver_unregister(&xm_battmngr_driver);
}
module_exit(xm_battmngr_exit);

MODULE_DESCRIPTION("Xiaomi Battery Management");
MODULE_AUTHOR("yinshunan@xiaomi.com");
MODULE_LICENSE("GPL v2");
