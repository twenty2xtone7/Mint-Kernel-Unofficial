#include <linux/string.h>
#include <linux/types.h>
#include <linux/cred.h>
#include <linux/fs.h>
#include <linux/path.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/dcache.h>
#include <linux/lsm_hooks.h>
#include <linux/init.h>
#include <linux/suspicious.h>

static uid_t susp_uid(void)
{
	const struct cred *cred = current_cred();
	return unlikely(!cred) ? 0 : cred->uid.val;
}

static bool is_app(void)
{
	return susp_uid() >= AID_APP_START;
}

static const struct suspicious_entry blocked_paths[] = {
	SUSP_ENTRY("/system/lib/libzygisk.so"),
	SUSP_ENTRY("/system/lib64/libzygisk.so"),
	SUSP_ENTRY("/dev/zygisk"),
	SUSP_ENTRY("/system/addon.d"),
	SUSP_ENTRY("/vendor/bin/install-recovery.sh"),
	SUSP_ENTRY("/system/bin/install-recovery.sh"),
};

static const struct suspicious_entry blocked_mounts[] = {
	SUSP_ENTRY("/data/adb"),
	SUSP_ENTRY("/dev/zygisk"),
	SUSP_ENTRY("/apex/com.android.art/bin/dex2oat"),
	SUSP_ENTRY("/system/apex/com.android.art/bin/dex2oat"),
	SUSP_ENTRY("/system/etc/preloaded-classes"),
	SUSP_ENTRY("/system/etc/hosts"),
};

static struct inode	*blocked_inodes[ARRAY_SIZE(blocked_paths)];
static struct inode	*blocked_mount_inodes[ARRAY_SIZE(blocked_mounts)];
static bool		 susp_ready;

static void populate_inodes(const struct suspicious_entry *tbl,
			     struct inode **inodes, size_t n)
{
	struct path p;
	size_t i;

	for (i = 0; i < n; i++) {
		if (!kern_path(tbl[i].str, LOOKUP_FOLLOW, &p)) {
			inodes[i] = d_inode(p.dentry);
			path_put(&p);
		}
	}
}

static bool inode_is_blocked(const struct inode *ino,
			     struct inode * const *inodes, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		struct inode *b = READ_ONCE(inodes[i]);

		if (b && b == ino)
			return true;
	}
	return false;
}

static int susp_inode_permission(struct inode *inode, int mask)
{
	if (!READ_ONCE(susp_ready) || !is_app())
		return 0;

	if (inode_is_blocked(inode, blocked_inodes,
			     ARRAY_SIZE(blocked_inodes))) {
		pr_info_ratelimited("suspicious-fs: permission denied uid=%u\n",
				    susp_uid());
		return -ENOENT;
	}

	return 0;
}

static struct security_hook_list susp_hooks[] __lsm_ro_after_init = {
	LSM_HOOK_INIT(inode_permission, susp_inode_permission),
};

int is_suspicious_mount(struct vfsmount *mnt, const struct path *root)
{
	struct inode *ino;

	if (!READ_ONCE(susp_ready) || !is_app())
		return 0;

	ino = d_inode(mnt->mnt_root);
	if (inode_is_blocked(ino, blocked_mount_inodes,
			     ARRAY_SIZE(blocked_mount_inodes))) {
		pr_info_ratelimited("suspicious-fs: hidden mount uid=%u\n",
				    susp_uid());
		return 1;
	}

	return 0;
}

static int __init suspicious_init(void)
{
	populate_inodes(blocked_paths, blocked_inodes,
			ARRAY_SIZE(blocked_paths));
	populate_inodes(blocked_mounts, blocked_mount_inodes,
			ARRAY_SIZE(blocked_mounts));

	security_add_hooks(susp_hooks, ARRAY_SIZE(susp_hooks), "suspicious");

	WRITE_ONCE(susp_ready, true);
	pr_info("suspicious-fs: hooks registered\n");
	return 0;
}
late_initcall(suspicious_init);
