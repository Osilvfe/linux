// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/completion.h>
#include <linux/gunyah.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include "rsc_mgr.h"

struct gunyah_rm_platform_ref {
	const struct gunyah_rm_platform_ops *ops;
	refcount_t users;
	struct completion released;
};

static struct gunyah_rm_platform_ref *rm_platform_ref;
static bool rm_platform_unregistering;
static DECLARE_RWSEM(rm_platform_ops_lock);
static DEFINE_MUTEX(rm_platform_registration_lock);

/* A VM keeps the same provider until all of its memory has been returned. */
struct gunyah_rm_platform_ref *gunyah_rm_platform_get(void)
{
	struct gunyah_rm_platform_ref *ref;

	down_read(&rm_platform_ops_lock);
	ref = rm_platform_ref;
	if (rm_platform_unregistering) {
		ref = ERR_PTR(-ENODEV);
	} else if (ref) {
		if (!try_module_get(ref->ops->owner))
			ref = ERR_PTR(-ENODEV);
		else
			refcount_inc(&ref->users);
	}
	up_read(&rm_platform_ops_lock);
	return ref;
}
EXPORT_SYMBOL_GPL(gunyah_rm_platform_get);

void gunyah_rm_platform_put(struct gunyah_rm_platform_ref *ref)
{
	struct module *owner;

	if (!ref)
		return;
	owner = ref->ops->owner;
	if (refcount_dec_and_test(&ref->users))
		complete(&ref->released);
	/* unregister may have freed ref after the completion. */
	module_put(owner);
}
EXPORT_SYMBOL_GPL(gunyah_rm_platform_put);

int gunyah_rm_platform_pre_mem_share(struct gunyah_rm *rm,
				     struct gunyah_rm_mem_parcel *mem_parcel)
{
	struct gunyah_rm_platform_ref *ref = mem_parcel->platform_ref;

	if (ref && ref->ops->pre_mem_share)
		return ref->ops->pre_mem_share(rm, mem_parcel);
	return 0;
}
EXPORT_SYMBOL_GPL(gunyah_rm_platform_pre_mem_share);

int gunyah_rm_platform_post_mem_reclaim(struct gunyah_rm *rm,
					struct gunyah_rm_mem_parcel *mem_parcel)
{
	struct gunyah_rm_platform_ref *ref = mem_parcel->platform_ref;

	if (ref && ref->ops->post_mem_reclaim)
		return ref->ops->post_mem_reclaim(rm, mem_parcel);
	return 0;
}
EXPORT_SYMBOL_GPL(gunyah_rm_platform_post_mem_reclaim);

int gunyah_rm_platform_pre_demand_page(struct gunyah_rm *rm, u16 vmid,
				       enum gunyah_pagetable_access access,
				       struct folio *folio,
				       struct gunyah_rm_platform_ref *ref)
{
	if (ref && ref->ops->pre_demand_page)
		return ref->ops->pre_demand_page(rm, vmid, access, folio);
	return 0;
}
EXPORT_SYMBOL_GPL(gunyah_rm_platform_pre_demand_page);

int gunyah_rm_platform_reclaim_demand_page(struct gunyah_rm *rm, u16 vmid,
					   enum gunyah_pagetable_access access,
					   struct folio *folio,
					   struct gunyah_rm_platform_ref *ref)
{
	if (ref && ref->ops->release_demand_page)
		return ref->ops->release_demand_page(rm, vmid, access, folio);
	return 0;
}
EXPORT_SYMBOL_GPL(gunyah_rm_platform_reclaim_demand_page);

int gunyah_rm_register_platform_ops(
	const struct gunyah_rm_platform_ops *platform_ops)
{
	struct gunyah_rm_platform_ref *ref;
	int ret = 0;

	if (!platform_ops)
		return -EINVAL;
	ref = kzalloc_obj(*ref);
	if (!ref)
		return -ENOMEM;
	ref->ops = platform_ops;
	refcount_set(&ref->users, 1);
	init_completion(&ref->released);

	mutex_lock(&rm_platform_registration_lock);
	down_write(&rm_platform_ops_lock);
	if (!rm_platform_ref)
		rm_platform_ref = ref;
	else
		ret = -EEXIST;
	up_write(&rm_platform_ops_lock);
	mutex_unlock(&rm_platform_registration_lock);
	if (ret)
		kfree(ref);
	return ret;
}
EXPORT_SYMBOL_GPL(gunyah_rm_register_platform_ops);

void gunyah_rm_unregister_platform_ops(
	const struct gunyah_rm_platform_ops *platform_ops)
{
	struct gunyah_rm_platform_ref *ref;

	mutex_lock(&rm_platform_registration_lock);
	down_write(&rm_platform_ops_lock);
	ref = rm_platform_ref;
	if (!ref || ref->ops != platform_ops) {
		up_write(&rm_platform_ops_lock);
		goto unlock;
	}
	rm_platform_ref = NULL;
	rm_platform_unregistering = true;
	up_write(&rm_platform_ops_lock);
	/* Existing VMs still need their callbacks, even while unregistering. */
	if (!refcount_dec_and_test(&ref->users))
		wait_for_completion(&ref->released);
	down_write(&rm_platform_ops_lock);
	rm_platform_unregistering = false;
	up_write(&rm_platform_ops_lock);
	kfree(ref);
unlock:
	mutex_unlock(&rm_platform_registration_lock);
}
EXPORT_SYMBOL_GPL(gunyah_rm_unregister_platform_ops);

static void _devm_gunyah_rm_unregister_platform_ops(void *data)
{
	gunyah_rm_unregister_platform_ops(
		(const struct gunyah_rm_platform_ops *)data);
}

int devm_gunyah_rm_register_platform_ops(
	struct device *dev, const struct gunyah_rm_platform_ops *ops)
{
	int ret;

	ret = gunyah_rm_register_platform_ops(ops);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, _devm_gunyah_rm_unregister_platform_ops,
				      (void *)ops);
}
EXPORT_SYMBOL_GPL(devm_gunyah_rm_register_platform_ops);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Gunyah Platform Hooks");
