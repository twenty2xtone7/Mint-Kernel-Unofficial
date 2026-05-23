#ifndef __LINUX_STATE_NOTIFIER_H
#define __LINUX_STATE_NOTIFIER_H

#include <linux/notifier.h>

#define STATE_NOTIFIER_ACTIVE		0x01
#define STATE_NOTIFIER_SUSPEND		0x02
#define STATE_NOTIFIER			"state_notifier"

struct state_event {
	void *data;
};
#ifdef CONFIG_STATE_NOTIFIER
extern bool state_suspended;
extern void state_suspend(void);
extern void state_resume(void);
int state_register_client(struct notifier_block *nb);
int state_unregister_client(struct notifier_block *nb);
int state_notifier_call_chain(unsigned long val, void *v);
#else
static const bool state_suspended = false;
static inline void state_suspend(void) {}
static inline void state_resume(void) {}
static inline int state_register_client(struct notifier_block *nb) { return 0; }
static inline int state_unregister_client(struct notifier_block *nb) { return 0; }
static inline int state_notifier_call_chain(unsigned long val, void *v) { return 0; }
#endif
#endif /* _LINUX_STATE_NOTIFIER_H */
