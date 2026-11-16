/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SHENG_CONSUMER_NOTIFY_H
#define SHENG_CONSUMER_NOTIFY_H
#include <linux/cleanup.h>
#include <linux/battmngr/battmngr_notifier.h>
struct sheng_consumer_guard { struct battmngr_notify *data; int cookie; };
static inline struct sheng_consumer_guard sheng_consumer_acquire(void)
{
 struct sheng_consumer_guard guard = { .cookie = -1 };
 guard.data = battmngr_notify_acquire(&guard.cookie);
 return guard;
}
static inline void sheng_consumer_release(struct sheng_consumer_guard *guard)
{ battmngr_notify_release(guard->cookie); }
#define SHENG_CONSUMER_STATE(name) \
 struct sheng_consumer_guard __cleanup(sheng_consumer_release) sheng_guard = sheng_consumer_acquire(); \
 struct battmngr_notify *name = sheng_guard.data
#endif
