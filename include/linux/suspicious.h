#ifndef _LINUX_SUSPICIOUS_H_
#define _LINUX_SUSPICIOUS_H_

#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/cred.h>
#include <linux/uidgid.h>
#include <linux/types.h>

#define AID_APP_START	10000U

struct suspicious_entry {
	const char	*str;
	size_t		 len;
};

#define SUSP_ENTRY(s)	{ .str = (s), .len = sizeof(s) - 1 }

int is_suspicious_mount(struct vfsmount *mnt, const struct path *root);

#endif
