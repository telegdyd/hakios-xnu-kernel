/*
 * HakiOS: what HFS's kext had in its C++ part (hfs_iokit.cpp), for HFS built into the kernel.
 *
 * The kext registered the filesystem from its IOService's start(); built in, it is an entry of the static table
 * (bsd/vfs/vfs_conf.c) and hfs_init() does the rest.  The IOKit questions -- is the medium ejectable, which media
 * carry a given content UUID, the platform's serial number -- have no use for a data volume on the phone's own
 * flash, and the keys of content protection are AppleKeyStore's, which this kernel does not have: a volume made
 * without content protection never asks for them.
 */
#include <sys/types.h>
#include <sys/errno.h>
#include <stdbool.h>
#include <mach/kern_return.h>
#include "hfs.h"
#include "hfs_iokit.h"

#ifndef panic_on_assert
bool panic_on_assert = false;
#endif

int
hfs_is_ejectable(__unused const char *cdev_name)
{
	return 0;
}

void
hfs_iterate_media_with_content(__unused const char *content_uuid_cstring,
    __unused int (*func)(const char *bsd_name, const char *uuid_str, void *arg), __unused void *arg)
{
}

/*
 * The snapshot namespace handler -- the user-space daemon told before a file changes, so that a snapshot of it
 * can be taken first -- left the kernel after the release HFS was written for (xnu-12377 has neither the
 * handler nor this call). The old kernel answered 0 when no handler was registered, which is always the case
 * here: so does this.
 */
int
nspace_snapshot_event(__unused vnode_t vp, __unused time_t ctime, __unused uint64_t op_type, __unused void *arg)
{
	return 0;
}

kern_return_t
hfs_get_platform_serial_number(__unused char *serial_number_str, __unused uint32_t len)
{
	return KERN_FAILURE;
}

#if CONFIG_PROTECT
int
hfs_unwrap_key(__unused aks_cred_t access, __unused const aks_wrapped_key_t wrapped_key_in,
    __unused aks_raw_key_t key_out)
{
	return ENOTSUP;
}

int
hfs_rewrap_key(__unused aks_cred_t access, __unused cp_key_class_t dp_class,
    __unused const aks_wrapped_key_t wrapped_key_in, __unused aks_wrapped_key_t wrapped_key_out)
{
	return ENOTSUP;
}

int
hfs_new_key(__unused aks_cred_t access, __unused cp_key_class_t dp_class, __unused aks_raw_key_t key_out,
    __unused aks_wrapped_key_t wrapped_key_out)
{
	return ENOTSUP;
}

int
hfs_backup_key(__unused aks_cred_t access, __unused const aks_wrapped_key_t wrapped_key_in,
    __unused aks_wrapped_key_t wrapped_key_out)
{
	return ENOTSUP;
}
#endif /* CONFIG_PROTECT */

#if !CONFIG_PROTECT
/*
 * Data protection classes as labels. Darwin gives each file it stores a class (fcntl F_SETPROTECTIONCLASS,
 * open_dprotected_np()) and checks it: keybagd's boot task sets /private/var/tmp to class D, PosterBoard's data store
 * must be class D throughout, and the container manager's personas are set up only where classes can be set. A
 * volume that answers ENOTSUP has no lock screen at all (INVALID_PERSONA: lsd and appprotectiond abort). Content
 * protection proper needs AppleKeyStore's keys; without them the class is what it is on ramfs: kept and told back,
 * nothing encrypted. It is kept here, so that it is there at the next boot, in an extended attribute under
 * com.apple.system. (hidden from listxattr, not settable from user space), read once per cnode. No attribute: class D.
 */
#include <sys/content_protection.h>
#include <sys/xattr.h>
#include <libkern/OSByteOrder.h>

#define HFS_DPCLASS_XATTR "com.apple.system.hakios.dpclass"

int
hfs_dpclass_valid(enum vtype type, uint32_t dpclass)
{
	if (dpclass == PROTECTION_CLASS_DIR_NONE) {
		return type == VDIR;
	}
	return dpclass >= PROTECTION_CLASS_MIN && dpclass <= PROTECTION_CLASS_MAX;
}

uint32_t
hfs_dpclass_get(vnode_t vp)
{
	struct cnode *cp = VTOC(vp);

	if (!cp->c_dpclass_read) {
		uint32_t stored = 0, dpclass = PROTECTION_CLASS_D;
		size_t size = sizeof(stored);

		if (hfs_xattr_read(vp, HFS_DPCLASS_XATTR, &stored, &size) == 0 && size == sizeof(stored) &&
		    hfs_dpclass_valid(vnode_vtype(vp), OSSwapLittleToHostInt32(stored))) {
			dpclass = OSSwapLittleToHostInt32(stored);
		}
		/* (under a shared lock two readers may both get here: they find the same) */
		cp->c_dpclass = dpclass;
		cp->c_dpclass_read = 1;
	}
	return cp->c_dpclass;
}

int
hfs_dpclass_set(vnode_t vp, uint32_t dpclass)
{
	struct cnode *cp = VTOC(vp);
	uint32_t stored = OSSwapHostToLittleInt32(dpclass);
	int error;

	if (!hfs_dpclass_valid(vnode_vtype(vp), dpclass)) {
		return EINVAL;
	}
	if (hfs_dpclass_get(vp) == dpclass) {
		return 0;
	}
	if (vfs_isrdonly(vnode_mount(vp))) {
		return EROFS;
	}
	error = MacToVFSError(hfs_xattr_write(vp, HFS_DPCLASS_XATTR, &stored, sizeof(stored)));
	if (error == 0) {
		cp->c_dpclass = dpclass;
	}
	return error;
}

int
hfs_dpclass_set_unlocked(vnode_t vp, uint32_t dpclass)
{
	int error = hfs_lock(VTOC(vp), HFS_EXCLUSIVE_LOCK, HFS_LOCK_DEFAULT);

	if (error == 0) {
		error = hfs_dpclass_set(vp, dpclass);
		hfs_unlock(VTOC(vp));
	}
	return error;
}

uint32_t
hfs_dpclass_get_unlocked(vnode_t vp)
{
	uint32_t dpclass = PROTECTION_CLASS_D;

	if (hfs_lock(VTOC(vp), HFS_SHARED_LOCK, HFS_LOCK_DEFAULT) == 0) {
		dpclass = hfs_dpclass_get(vp);
		hfs_unlock(VTOC(vp));
	}
	return dpclass;
}
#endif /* !CONFIG_PROTECT */
