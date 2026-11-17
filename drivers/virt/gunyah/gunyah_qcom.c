// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/arm-smccc.h>
#include <linux/gunyah.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/uuid.h>

#define QCOM_SCM_RM_MANAGED_VMID 0x3A
#define QCOM_SCM_MAX_MANAGED_VMID 0x3F

static int qcom_scm_gunyah_mem_entry_compare(const void *first, const void *second)
{
	const struct gunyah_rm_mem_entry *left = first, *right = second;
	u64 left_addr = le64_to_cpu(left->phys_addr);
	u64 right_addr = le64_to_cpu(right->phys_addr);

	return (left_addr > right_addr) - (left_addr < right_addr);
}

static int qcom_scm_gunyah_mem_entries(struct gunyah_rm_mem_parcel *parcel,
				       struct gunyah_rm_mem_entry **result,
				       size_t *count)
{
	struct gunyah_rm_mem_entry *entries;
	u64 address, size, end, previous_end = 0;
	size_t index, retained = 0;
	int ret = -EINVAL;

	entries = kvmalloc_array(parcel->n_mem_entries, sizeof(*entries), GFP_KERNEL);
	if (!entries)
		return -ENOMEM;

	memcpy(entries, parcel->mem_entries,
	       array_size(parcel->n_mem_entries, sizeof(*entries)));
	sort(entries, parcel->n_mem_entries, sizeof(*entries),
	     qcom_scm_gunyah_mem_entry_compare, NULL);
	for (index = 0; index < parcel->n_mem_entries; index++) {
		address = le64_to_cpu(entries[index].phys_addr);
		size = le64_to_cpu(entries[index].size);
		if (!size)
			goto free_entries;
		if (check_add_overflow(address, size, &end)) {
			ret = -EOVERFLOW;
			goto free_entries;
		}
		if (retained && address < previous_end)
			goto free_entries;
		if (retained && address == previous_end)
			entries[retained - 1].size =
				cpu_to_le64(end - le64_to_cpu(entries[retained - 1].phys_addr));
		else
			entries[retained++] = entries[index];
		previous_end = end;
	}
	*result = entries;
	*count = retained;
	return 0;

free_entries:
	kvfree(entries);
	return ret;
}

static int
qcom_scm_gunyah_rm_pre_mem_share(struct gunyah_rm *rm,
				 struct gunyah_rm_mem_parcel *mem_parcel)
{
	struct qcom_scm_vmperm *new_perms __free(kfree) = NULL;
	struct gunyah_rm_mem_entry *mem_entries __free(kvfree) = NULL;
	size_t entry, n_mem_entries;
	u64 src, src_cpy;
	int ret, n;
	u16 vmid;

	ret = qcom_scm_gunyah_mem_entries(mem_parcel, &mem_entries, &n_mem_entries);
	if (ret)
		return ret;

	new_perms = kcalloc(mem_parcel->n_acl_entries, sizeof(*new_perms),
			    GFP_KERNEL);
	if (!new_perms)
		return -ENOMEM;

	for (n = 0; n < mem_parcel->n_acl_entries; n++) {
		vmid = le16_to_cpu(mem_parcel->acl_entries[n].vmid);
		if (vmid <= QCOM_SCM_MAX_MANAGED_VMID)
			new_perms[n].vmid = vmid;
		else
			new_perms[n].vmid = QCOM_SCM_RM_MANAGED_VMID;
		if (mem_parcel->acl_entries[n].perms & GUNYAH_RM_ACL_X)
			new_perms[n].perm |= QCOM_SCM_PERM_EXEC;
		if (mem_parcel->acl_entries[n].perms & GUNYAH_RM_ACL_W)
			new_perms[n].perm |= QCOM_SCM_PERM_WRITE;
		if (mem_parcel->acl_entries[n].perms & GUNYAH_RM_ACL_R)
			new_perms[n].perm |= QCOM_SCM_PERM_READ;
	}

	src = BIT_ULL(QCOM_SCM_VMID_HLOS);

	for (entry = 0; entry < n_mem_entries; entry++) {
		src_cpy = src;
		ret = qcom_scm_assign_mem(
			le64_to_cpu(mem_entries[entry].phys_addr),
			le64_to_cpu(mem_entries[entry].size), &src_cpy,
			new_perms, mem_parcel->n_acl_entries);
		if (ret)
			return ret;
		mem_parcel->n_shared_mem_entries = entry + 1;
	}

	return 0;
}

static int
qcom_scm_gunyah_rm_post_mem_reclaim(struct gunyah_rm *rm,
				    struct gunyah_rm_mem_parcel *mem_parcel)
{
	struct qcom_scm_vmperm new_perms;
	struct gunyah_rm_mem_entry *mem_entries __free(kvfree) = NULL;
	u64 src = 0, src_cpy;
	size_t entry, n_mem_entries;
	int ret, n;
	u16 vmid;

	if (mem_parcel->n_reclaimed_mem_entries == mem_parcel->n_shared_mem_entries)
		return 0;

	ret = qcom_scm_gunyah_mem_entries(mem_parcel, &mem_entries, &n_mem_entries);
	if (ret)
		return ret;
	if (mem_parcel->n_reclaimed_mem_entries > mem_parcel->n_shared_mem_entries ||
	    mem_parcel->n_shared_mem_entries > n_mem_entries)
		return -EINVAL;

	new_perms.vmid = QCOM_SCM_VMID_HLOS;
	new_perms.perm = QCOM_SCM_PERM_EXEC | QCOM_SCM_PERM_WRITE |
			 QCOM_SCM_PERM_READ;

	for (n = 0; n < mem_parcel->n_acl_entries; n++) {
		vmid = le16_to_cpu(mem_parcel->acl_entries[n].vmid);
		if (vmid <= QCOM_SCM_MAX_MANAGED_VMID)
			src |= (1ull << vmid);
		else
			src |= (1ull << QCOM_SCM_RM_MANAGED_VMID);
	}

	for (entry = mem_parcel->n_reclaimed_mem_entries;
	     entry < mem_parcel->n_shared_mem_entries; entry++) {
		src_cpy = src;
		ret = qcom_scm_assign_mem(
			le64_to_cpu(mem_entries[entry].phys_addr),
			le64_to_cpu(mem_entries[entry].size), &src_cpy,
			&new_perms, 1);
		WARN_ON_ONCE(ret);
		if (ret)
			return ret;
		mem_parcel->n_reclaimed_mem_entries = entry + 1;
	}

	return 0;
}

static int
qcom_scm_gunyah_rm_pre_demand_page(struct gunyah_rm *rm, u16 vmid,
				   enum gunyah_pagetable_access access,
				   struct folio *folio)
{
	struct qcom_scm_vmperm new_perms[2];
	unsigned int n = 1;
	u64 src;

	new_perms[0].vmid = QCOM_SCM_RM_MANAGED_VMID;
	new_perms[0].perm = QCOM_SCM_PERM_EXEC | QCOM_SCM_PERM_WRITE |
			    QCOM_SCM_PERM_READ;
	if (access != GUNYAH_PAGETABLE_ACCESS_X &&
	    access != GUNYAH_PAGETABLE_ACCESS_RX &&
	    access != GUNYAH_PAGETABLE_ACCESS_RWX) {
		new_perms[1].vmid = QCOM_SCM_VMID_HLOS;
		new_perms[1].perm = QCOM_SCM_PERM_EXEC | QCOM_SCM_PERM_WRITE |
				    QCOM_SCM_PERM_READ;
		n++;
	}

	src = BIT_ULL(QCOM_SCM_VMID_HLOS);

	return qcom_scm_assign_mem(__pfn_to_phys(folio_pfn(folio)),
				   folio_size(folio), &src, new_perms, n);
}

static int
qcom_scm_gunyah_rm_release_demand_page(struct gunyah_rm *rm, u16 vmid,
				       enum gunyah_pagetable_access access,
				       struct folio *folio)
{
	struct qcom_scm_vmperm new_perms;
	u64 src;

	new_perms.vmid = QCOM_SCM_VMID_HLOS;
	new_perms.perm = QCOM_SCM_PERM_EXEC | QCOM_SCM_PERM_WRITE |
			 QCOM_SCM_PERM_READ;

	src = BIT_ULL(QCOM_SCM_RM_MANAGED_VMID);

	if (access != GUNYAH_PAGETABLE_ACCESS_X &&
	    access != GUNYAH_PAGETABLE_ACCESS_RX &&
	    access != GUNYAH_PAGETABLE_ACCESS_RWX)
		src |= BIT_ULL(QCOM_SCM_VMID_HLOS);

	return qcom_scm_assign_mem(__pfn_to_phys(folio_pfn(folio)),
				   folio_size(folio), &src, &new_perms, 1);
}

static struct gunyah_rm_platform_ops qcom_scm_gunyah_rm_platform_ops = {
	.owner = THIS_MODULE,
	.pre_mem_share = qcom_scm_gunyah_rm_pre_mem_share,
	.post_mem_reclaim = qcom_scm_gunyah_rm_post_mem_reclaim,
	.pre_demand_page = qcom_scm_gunyah_rm_pre_demand_page,
	.release_demand_page = qcom_scm_gunyah_rm_release_demand_page,
};

/* {19bd54bd-0b37-571b-946f-609b54539de6} */
static const uuid_t QCOM_EXT_UUID = UUID_INIT(0x19bd54bd, 0x0b37, 0x571b, 0x94,
					      0x6f, 0x60, 0x9b, 0x54, 0x53,
					      0x9d, 0xe6);

#define GUNYAH_QCOM_EXT_CALL_UUID_ID                              \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_32, \
			   ARM_SMCCC_OWNER_VENDOR_HYP, 0x3f01)

static bool gunyah_has_qcom_extensions(void)
{
	struct arm_smccc_res res;
	uuid_t uuid;
	u32 *up;

	arm_smccc_1_1_invoke(GUNYAH_QCOM_EXT_CALL_UUID_ID, &res);

	up = (u32 *)&uuid.b[0];
	up[0] = lower_32_bits(res.a0);
	up[1] = lower_32_bits(res.a1);
	up[2] = lower_32_bits(res.a2);
	up[3] = lower_32_bits(res.a3);

	return uuid_equal(&uuid, &QCOM_EXT_UUID);
}

static int __init qcom_gunyah_platform_hooks_register(void)
{
	if (!gunyah_has_qcom_extensions())
		return -ENODEV;

	pr_info("Enabling Gunyah hooks for Qualcomm platforms.\n");

	return gunyah_rm_register_platform_ops(
		&qcom_scm_gunyah_rm_platform_ops);
}

static void __exit qcom_gunyah_platform_hooks_unregister(void)
{
	gunyah_rm_unregister_platform_ops(&qcom_scm_gunyah_rm_platform_ops);
}

module_init(qcom_gunyah_platform_hooks_register);
module_exit(qcom_gunyah_platform_hooks_unregister);
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Platform Hooks for Gunyah");
MODULE_LICENSE("GPL");
