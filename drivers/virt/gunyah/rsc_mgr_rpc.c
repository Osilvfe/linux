// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/error-injection.h>

#include "rsc_mgr.h"

/* Message IDs: Memory Management */
#define GUNYAH_RM_RPC_MEM_LEND 0x51000012
#define GUNYAH_RM_RPC_MEM_SHARE 0x51000013
#define GUNYAH_RM_RPC_MEM_RECLAIM 0x51000015
#define GUNYAH_RM_RPC_MEM_APPEND 0x51000018

/* Message IDs: VM Management */
/* clang-format off */
#define GUNYAH_RM_RPC_VM_ALLOC_VMID		0x56000001
#define GUNYAH_RM_RPC_VM_DEALLOC_VMID		0x56000002
#define GUNYAH_RM_RPC_VM_START			0x56000004
#define GUNYAH_RM_RPC_VM_STOP			0x56000005
#define GUNYAH_RM_RPC_VM_RESET			0x56000006
#define GUNYAH_RM_RPC_VM_GET_STATE		0x56000017
#define GUNYAH_RM_RPC_VM_CONFIG_IMAGE		0x56000009
#define GUNYAH_RM_RPC_VM_AUTH_IMAGE		0x5600000A
#define GUNYAH_RM_RPC_VM_INIT			0x5600000B
#define GUNYAH_RM_RPC_VM_GET_HYP_RESOURCES	0x56000020
#define GUNYAH_RM_RPC_VM_GET_VMID		0x56000024
#define GUNYAH_RM_RPC_VM_SET_BOOT_CONTEXT	0x56000031
#define GUNYAH_RM_RPC_VM_SET_FIRMWARE_MEM	0x56000032
#define GUNYAH_RM_RPC_VM_SET_DEMAND_PAGING	0x56000033
#define GUNYAH_RM_RPC_VM_SET_ADDRESS_LAYOUT	0x56000034
#define GUNYAH_RM_RPC_DEVICE_ACCEPT		0x56000060
#define GUNYAH_RM_RPC_DEVICE_LEND		0x56000061
#define GUNYAH_RM_RPC_DEVICE_RELEASE		0x56000062
#define GUNYAH_RM_RPC_DEVICE_RECLAIM		0x56000063
#define GUNYAH_RM_RPC_DEVICE_FIND_HANDLE	0x56000065
#define GUNYAH_RM_RPC_DEVICE_GET_RESOURCES	0x56000066
#define GUNYAH_RM_RPC_DEVICE_DONATE		0x56000069
/* clang-format on */

#define GUNYAH_RM_DEVICE_RESET_FLAG		BIT(0)
#define GUNYAH_RM_DEVICE_BIND_FLAG		BIT(1)
#define GUNYAH_RM_DEVICE_GET_RESOURCES_PHYSICAL	BIT(0)

struct gunyah_rm_device_transfer_req {
	__le16 vmid;
	u8 flags;
	u8 reserved;
	__le32 handle;
} __packed;

struct gunyah_rm_device_accept_req {
	__le32 handle;
	u8 flags;
	u8 reserved[3];
	__le32 bus_handle;
} __packed;

struct gunyah_rm_device_release_req {
	__le32 handle;
	u8 flags;
	u8 reserved[3];
} __packed;

struct gunyah_rm_device_reclaim_req {
	__le32 handle;
	u8 flags;
	u8 reserved[3];
} __packed;

struct gunyah_rm_device_find_resp {
	__le32 handle;
} __packed;

struct gunyah_rm_device_get_resources_req {
	__le32 handle;
	u8 flags;
	u8 reserved[3];
} __packed;

struct gunyah_rm_device_get_resources_resp {
	__le16 num_entries;
	__le16 reserved;
	struct gunyah_device_resource resources[];
} __packed;

static_assert(sizeof(struct gunyah_device_resource) == 16);
static_assert(sizeof(struct gunyah_rm_device_transfer_req) == 8);
static_assert(offsetof(struct gunyah_rm_device_transfer_req, vmid) == 0);
static_assert(offsetof(struct gunyah_rm_device_transfer_req, flags) == 2);
static_assert(offsetof(struct gunyah_rm_device_transfer_req, handle) == 4);
static_assert(sizeof(struct gunyah_rm_device_accept_req) == 12);
static_assert(offsetof(struct gunyah_rm_device_accept_req, handle) == 0);
static_assert(offsetof(struct gunyah_rm_device_accept_req, flags) == 4);
static_assert(offsetof(struct gunyah_rm_device_accept_req, bus_handle) == 8);
static_assert(sizeof(struct gunyah_rm_device_release_req) == 8);
static_assert(offsetof(struct gunyah_rm_device_release_req, handle) == 0);
static_assert(offsetof(struct gunyah_rm_device_release_req, flags) == 4);
static_assert(sizeof(struct gunyah_rm_device_reclaim_req) == 8);
static_assert(offsetof(struct gunyah_rm_device_reclaim_req, handle) == 0);
static_assert(offsetof(struct gunyah_rm_device_reclaim_req, flags) == 4);
static_assert(sizeof(struct gunyah_rm_device_get_resources_req) == 8);
static_assert(offsetof(struct gunyah_rm_device_get_resources_req, handle) == 0);
static_assert(offsetof(struct gunyah_rm_device_get_resources_req, flags) == 4);

struct gunyah_rm_vm_common_vmid_req {
	__le16 vmid;
	__le16 _padding;
} __packed;

/* Call: MEM_LEND, MEM_SHARE */
#define GUNYAH_RM_MAX_MEM_ENTRIES 512

#define GUNYAH_MEM_SHARE_REQ_FLAGS_APPEND BIT(1)

struct gunyah_rm_mem_share_req_header {
	u8 mem_type;
	u8 _padding0;
	u8 flags;
	u8 _padding1;
	__le32 label;
} __packed;

struct gunyah_rm_mem_share_req_acl_section {
	__le16 n_entries;
	__le16 _padding;
	struct gunyah_rm_mem_acl_entry entries[];
} __packed;

struct gunyah_rm_mem_share_req_mem_section {
	__le16 n_entries;
	__le16 _padding;
	struct gunyah_rm_mem_entry entries[];
} __packed;

/* Call: MEM_RELEASE */
struct gunyah_rm_mem_release_req {
	__le32 mem_handle;
	u8 flags; /* currently not used */
	u8 _padding0;
	__le16 _padding1;
} __packed;

/* Call: MEM_APPEND */
#define GUNYAH_MEM_APPEND_REQ_FLAGS_END BIT(0)

struct gunyah_rm_mem_append_req_header {
	__le32 mem_handle;
	u8 flags;
	u8 _padding0;
	__le16 _padding1;
} __packed;

/* Call: VM_ALLOC */
struct gunyah_rm_vm_alloc_vmid_resp {
	__le16 vmid;
	__le16 _padding;
} __packed;

/* Call: VM_STOP */
#define GUNYAH_RM_VM_STOP_FLAG_FORCE_STOP BIT(0)

#define GUNYAH_RM_VM_STOP_REASON_FORCE_STOP 3

struct gunyah_rm_vm_stop_req {
	__le16 vmid;
	u8 flags;
	u8 _padding;
	__le32 stop_reason;
} __packed;

/* Call: VM_AUTH_IMAGE */
struct gunyah_rm_vm_authenticate_req_header {
	__le16 vmid;
	__le16 n_params;
} __packed;

/* Call: VM_CONFIG_IMAGE */
struct gunyah_rm_vm_config_image_req {
	__le16 vmid;
	__le16 auth_mech;
	__le32 mem_handle;
	__le64 image_offset;
	__le64 image_size;
	__le64 dtb_offset;
	__le64 dtb_size;
} __packed;

/* Call: VM_SET_BOOT_CONTEXT */
struct gunyah_rm_vm_set_boot_context_req {
	__le16 vmid;
	u8 reg_set;
	u8 reg_index;
	__le32 _padding;
	__le64 value;
} __packed;

/* Call: VM_SET_DEMAND_PAGING */
struct gunyah_rm_vm_set_demand_paging_req {
	__le16 vmid;
	__le16 _padding;
	__le32 range_count;
	DECLARE_FLEX_ARRAY(struct gunyah_rm_mem_entry, ranges);
} __packed;

/* Call: VM_SET_ADDRESS_LAYOUT */
struct gunyah_rm_vm_set_address_layout_req {
	__le16 vmid;
	__le16 _padding;
	__le32 range_id;
	__le64 range_base;
	__le64 range_size;
} __packed;

/* Call: VM_SET_FIRMWARE_MEM */
struct gunyah_vm_set_firmware_mem_req {
	__le16 vmid;
	__le16 reserved;
	__le32 mem_handle;
	__le64 fw_offset;
	__le64 fw_size;
} __packed;

/*
 * Several RM calls take only a VMID as a parameter and give only standard
 * response back. Deduplicate boilerplate code by using this common call.
 */
static int gunyah_rm_common_vmid_call(struct gunyah_rm *rm, u32 message_id,
				      u16 vmid)
{
	struct gunyah_rm_vm_common_vmid_req req_payload = {
		.vmid = cpu_to_le16(vmid),
	};

	return gunyah_rm_call(rm, message_id, &req_payload, sizeof(req_payload),
			      NULL, NULL);
}

static int gunyah_rm_mem_append(struct gunyah_rm *rm, u32 mem_handle,
				struct gunyah_rm_mem_entry *entries,
				size_t n_entries)
{
	struct gunyah_rm_mem_append_req_header *req __free(kfree) = NULL;
	struct gunyah_rm_mem_share_req_mem_section *mem;
	int ret = 0;
	size_t n;

	req = kzalloc(sizeof(*req) + struct_size(mem, entries, GUNYAH_RM_MAX_MEM_ENTRIES),
		      GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->mem_handle = cpu_to_le32(mem_handle);
	mem = (void *)(req + 1);

	while (n_entries) {
		req->flags = 0;
		if (n_entries > GUNYAH_RM_MAX_MEM_ENTRIES) {
			n = GUNYAH_RM_MAX_MEM_ENTRIES;
		} else {
			req->flags |= GUNYAH_MEM_APPEND_REQ_FLAGS_END;
			n = n_entries;
		}

		mem->n_entries = cpu_to_le16(n);
		memcpy(mem->entries, entries, sizeof(*entries) * n);

		ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_MEM_APPEND, req,
				     sizeof(*req) + struct_size(mem, entries, n),
				     NULL, NULL);
		if (ret)
			break;

		entries += n;
		n_entries -= n;
	}

	return ret;
}

/**
 * gunyah_rm_mem_share() - Share memory with other virtual machines.
 * @rm: Handle to a Gunyah resource manager
 * @p: Information about the memory to be shared.
 *
 * Sharing keeps Linux's access to the memory while the memory parcel is shared.
 */
int gunyah_rm_mem_share(struct gunyah_rm *rm, struct gunyah_rm_mem_parcel *p)
{
	u32 message_id = p->n_acl_entries == 1 ? GUNYAH_RM_RPC_MEM_LEND :
						 GUNYAH_RM_RPC_MEM_SHARE;
	size_t msg_size, initial_mem_entries = p->n_mem_entries, resp_size;
	struct gunyah_rm_mem_share_req_acl_section *acl;
	struct gunyah_rm_mem_share_req_mem_section *mem;
	struct gunyah_rm_mem_share_req_header *req_header;
	size_t acl_size, mem_size;
	u32 *attr_section;
	bool need_append = false;
	__le32 *resp;
	void *msg;
	int ret;

	if (!p->acl_entries || !p->n_acl_entries || !p->mem_entries ||
	    !p->n_mem_entries || p->n_acl_entries > U8_MAX ||
	    gunyah_rm_mem_reclaim_needed(p))
		return -EINVAL;
	if (gunyah_rm_is_failed(rm))
		return -EIO;

	if (initial_mem_entries > GUNYAH_RM_MAX_MEM_ENTRIES) {
		initial_mem_entries = GUNYAH_RM_MAX_MEM_ENTRIES;
		need_append = true;
	}

	acl_size = struct_size(acl, entries, p->n_acl_entries);
	mem_size = struct_size(mem, entries, initial_mem_entries);

	/* The format of the message goes:
	 * request header
	 * ACL entries (which VMs get what kind of access to this memory parcel)
	 * Memory entries (list of memory regions to share)
	 * Memory attributes (currently unused, we'll hard-code the size to 0)
	 */
	msg_size = sizeof(struct gunyah_rm_mem_share_req_header) + acl_size +
		   mem_size +
		   sizeof(u32); /* for memory attributes, currently unused */

	msg = kzalloc(msg_size, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	p->rm_reclaimed = true;
	p->rm_unknown = false;
	p->n_shared_mem_entries = 0;
	p->n_reclaimed_mem_entries = 0;
	ret = gunyah_rm_platform_pre_mem_share(rm, p);
	if (ret) {
		kfree(msg);
		gunyah_rm_mem_reclaim(rm, p);
		return ret;
	}

	req_header = msg;
	acl = (void *)req_header + sizeof(*req_header);
	mem = (void *)acl + acl_size;
	attr_section = (void *)mem + mem_size;

	req_header->mem_type = p->mem_type;
	if (need_append)
		req_header->flags |= GUNYAH_MEM_SHARE_REQ_FLAGS_APPEND;
	req_header->label = cpu_to_le32(p->label);

	acl->n_entries = cpu_to_le32(p->n_acl_entries);
	memcpy(acl->entries, p->acl_entries,
	       flex_array_size(acl, entries, p->n_acl_entries));

	mem->n_entries = cpu_to_le16(initial_mem_entries);
	memcpy(mem->entries, p->mem_entries,
	       flex_array_size(mem, entries, initial_mem_entries));

	/* Set n_entries for memory attribute section to 0 */
	*attr_section = 0;

	ret = gunyah_rm_call(rm, message_id, msg, msg_size, (void **)&resp,
			     &resp_size);
	kfree(msg);

	if (ret) {
		p->rm_unknown = gunyah_rm_is_failed(rm);
		gunyah_rm_mem_reclaim(rm, p);
		return ret;
	}

	if (resp_size != sizeof(*resp) || !resp ||
	    le32_to_cpu(*resp) == GUNYAH_MEM_HANDLE_INVAL) {
		kfree(resp);
		p->rm_unknown = true;
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	p->mem_handle = le32_to_cpu(*resp);
	p->rm_reclaimed = false;
	kfree(resp);

	if (need_append) {
		ret = gunyah_rm_mem_append(
			rm, p->mem_handle, &p->mem_entries[initial_mem_entries],
			p->n_mem_entries - initial_mem_entries);
		if (ret && !gunyah_rm_mem_reclaim(rm, p))
			p->mem_handle = GUNYAH_MEM_HANDLE_INVAL;
	}

	return ret;
}
ALLOW_ERROR_INJECTION(gunyah_rm_mem_share, ERRNO);

/**
 * gunyah_rm_mem_reclaim() - Reclaim a memory parcel
 * @rm: Handle to a Gunyah resource manager
 * @parcel: Information about the memory to be reclaimed.
 *
 * RM maps the associated memory back into the stage-2 page tables of the owner VM.
 */
int gunyah_rm_mem_reclaim(struct gunyah_rm *rm,
			  struct gunyah_rm_mem_parcel *parcel)
{
	struct gunyah_rm_mem_release_req req = {
		.mem_handle = cpu_to_le32(parcel->mem_handle),
	};
	int ret;

	if (!gunyah_rm_mem_reclaim_needed(parcel))
		return 0;
	if (parcel->rm_unknown || gunyah_rm_is_failed(rm))
		return -EIO;

	if (parcel->mem_handle != GUNYAH_MEM_HANDLE_INVAL && !parcel->rm_reclaimed) {
		ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_MEM_RECLAIM, &req,
				     sizeof(req), NULL, NULL);
		if (ret)
			return ret;
		parcel->rm_reclaimed = true;
	}

	return gunyah_rm_platform_post_mem_reclaim(rm, parcel);
}
ALLOW_ERROR_INJECTION(gunyah_rm_mem_reclaim, ERRNO);

/**
 * gunyah_rm_alloc_vmid() - Allocate a new VM in Gunyah. Returns the VM identifier.
 * @rm: Handle to a Gunyah resource manager
 * @vmid: Use 0 to dynamically allocate a VM. A reserved VMID can be supplied
 *        to request allocation of a platform-defined VM.
 *
 * Return: the allocated VMID or negative value on error
 */
int gunyah_rm_alloc_vmid(struct gunyah_rm *rm, u16 vmid)
{
	struct gunyah_rm_vm_common_vmid_req req_payload = {
		.vmid = cpu_to_le16(vmid),
	};
	struct gunyah_rm_vm_alloc_vmid_resp *resp_payload;
	size_t resp_size;
	void *resp;
	int ret;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_ALLOC_VMID, &req_payload,
			     sizeof(req_payload), &resp, &resp_size);
	if (ret)
		return ret;

	if (!vmid) {
		if (resp_size != sizeof(*resp_payload) || !resp)
			goto malformed;
		resp_payload = resp;
		ret = le16_to_cpu(resp_payload->vmid);
		if (!ret || ret == GUNYAH_VMID_INVAL)
			goto malformed;
	} else if (resp_size) {
		goto malformed;
	}

	kfree(resp);
	return ret;

malformed:
	kfree(resp);
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_alloc_vmid, ERRNO);

/**
 * gunyah_rm_dealloc_vmid() - Dispose of a VMID
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 */
int gunyah_rm_dealloc_vmid(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_common_vmid_call(rm, GUNYAH_RM_RPC_VM_DEALLOC_VMID,
					  vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_dealloc_vmid, ERRNO);

/**
 * gunyah_rm_vm_get_state() - Query the resource manager's VM state
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 * @vm_status: Set to the VM state reported by the resource manager
 *
 * Return: 0 on success; negative errno on failure.
 */
int gunyah_rm_vm_get_state(struct gunyah_rm *rm, u16 vmid,
			   enum gunyah_rm_vm_status *vm_status)
{
	struct gunyah_rm_vm_common_vmid_req req_payload = {
		.vmid = cpu_to_le16(vmid),
	};
	__le32 *resp;
	size_t resp_size;
	u8 state;
	int ret;

	if (!vm_status)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_STATE, &req_payload,
			     sizeof(req_payload), (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size != sizeof(*resp))
		goto malformed;

	state = le32_to_cpu(*resp) & 0xff;
	switch (state) {
	case GUNYAH_RM_VM_STATUS_NO_STATE:
	case GUNYAH_RM_VM_STATUS_INIT:
	case GUNYAH_RM_VM_STATUS_READY:
	case GUNYAH_RM_VM_STATUS_RUNNING:
	case GUNYAH_RM_VM_STATUS_PAUSED:
	case GUNYAH_RM_VM_STATUS_LOAD:
	case GUNYAH_RM_VM_STATUS_AUTH:
	case GUNYAH_RM_VM_STATUS_INIT_FAILED:
	case GUNYAH_RM_VM_STATUS_EXITED:
	case GUNYAH_RM_VM_STATUS_RESETTING:
	case GUNYAH_RM_VM_STATUS_RESET:
	case GUNYAH_RM_VM_STATUS_RESET_FAILED:
		break;
	default:
		goto malformed;
	}

	*vm_status = state;
	kfree(resp);
	return 0;

malformed:
	kfree(resp);
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_get_state, ERRNO);

/**
 * gunyah_rm_vm_reset() - Reset a VM's resources
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 *
 * As part of tearing down the VM, request RM to clean up all the VM resources
 * associated with the VM. Only after this, Linux can clean up all the
 * references it maintains to resources.
 */
int gunyah_rm_vm_reset(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_common_vmid_call(rm, GUNYAH_RM_RPC_VM_RESET, vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_reset, ERRNO);

/**
 * gunyah_rm_vm_start() - Move a VM into "ready to run" state
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 *
 * On VMs which use proxy scheduling, vcpu_run is needed to actually run the VM.
 * On VMs which use Gunyah's scheduling, the vCPUs start executing in accordance with Gunyah
 * scheduling policies.
 */
int gunyah_rm_vm_start(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_common_vmid_call(rm, GUNYAH_RM_RPC_VM_START, vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_start, ERRNO);

/**
 * gunyah_rm_vm_stop() - Send a request to Resource Manager VM to forcibly stop a VM.
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 */
int gunyah_rm_vm_stop(struct gunyah_rm *rm, u16 vmid)
{
	struct gunyah_rm_vm_stop_req req_payload = {
		.vmid = cpu_to_le16(vmid),
		.flags = GUNYAH_RM_VM_STOP_FLAG_FORCE_STOP,
		.stop_reason = cpu_to_le32(GUNYAH_RM_VM_STOP_REASON_FORCE_STOP),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_STOP, &req_payload,
			      sizeof(req_payload), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_stop, ERRNO);

/**
 * gunyah_rm_vm_authenticate() - Send a request to Resource Manager VM to authenticate a VM.
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 * @n_entries: Number of entries of type gunyah_rm_vm_auth_param_entry
 * @entry: Type of authentication parameters
 */
int gunyah_rm_vm_authenticate(struct gunyah_rm *rm, u16 vmid,
		ssize_t n_entries,
		struct gunyah_rm_vm_authenticate_param_entry *entry)
{
	struct gunyah_rm_vm_authenticate_req_header *req_header;
	struct gunyah_rm_vm_authenticate_param_entry *dest_entry;
	size_t resp_payload_size;
	size_t req_size;
	int err;
	void *req_buf;
	__le32 *resp;

	req_size = sizeof(*req_header) + n_entries * sizeof(*entry);

	req_buf = kzalloc(req_size, GFP_KERNEL);
	if (!req_buf)
		return -ENOMEM;

	req_header = req_buf;
	req_header->vmid = vmid;
	req_header->n_params = n_entries;

	dest_entry = req_buf + sizeof(*req_header);
	memcpy(dest_entry, entry, sizeof(*entry) * n_entries);

	err = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_AUTH_IMAGE,
					req_buf, req_size, (void **)&resp,
						&resp_payload_size);
	if (err) {
		pr_err("%s: Unable to send VM_AUTH_IMAGE to RM: %d\n", __func__, err);
		kfree(req_buf);
		return err;
	}

	if (resp_payload_size) {
		pr_err("%s: Invalid size received for VM_AUTH_IMAGE: %zu\n",
			__func__, resp_payload_size);
		kfree(resp);
		kfree(req_buf);
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	/* no need to free the resp as no payload is expected */
	kfree(req_buf);
	return 0;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_authenticate, ERRNO);
EXPORT_SYMBOL_GPL(gunyah_rm_vm_authenticate);

/**
 * gunyah_rm_vm_configure() - Prepare a VM to start and provide the common
 *			  configuration needed by RM to configure a VM
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier allocated with gunyah_rm_alloc_vmid
 * @auth_mechanism: Authentication mechanism used by resource manager to verify
 *                  the virtual machine
 * @mem_handle: Handle to a previously shared memparcel that contains all parts
 *              of the VM image subject to authentication.
 * @image_offset: Start address of VM image, relative to the start of memparcel
 * @image_size: Size of the VM image
 * @dtb_offset: Start address of the devicetree binary with VM configuration,
 *              relative to start of memparcel.
 * @dtb_size: Maximum size of devicetree binary.
 */
int gunyah_rm_vm_configure(struct gunyah_rm *rm, u16 vmid,
			   enum gunyah_rm_vm_auth_mechanism auth_mechanism,
			   u32 mem_handle, u64 image_offset, u64 image_size,
			   u64 dtb_offset, u64 dtb_size)
{
	struct gunyah_rm_vm_config_image_req req_payload = {
		.vmid = cpu_to_le16(vmid),
		.auth_mech = cpu_to_le16(auth_mechanism),
		.mem_handle = cpu_to_le32(mem_handle),
		.image_offset = cpu_to_le64(image_offset),
		.image_size = cpu_to_le64(image_size),
		.dtb_offset = cpu_to_le64(dtb_offset),
		.dtb_size = cpu_to_le64(dtb_size),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_CONFIG_IMAGE, &req_payload,
			      sizeof(req_payload), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_configure, ERRNO);

/**
 * gunyah_rm_vm_init() - Move the VM to initialized state.
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier
 *
 * RM will allocate needed resources for the VM.
 */
int gunyah_rm_vm_init(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_common_vmid_call(rm, GUNYAH_RM_RPC_VM_INIT, vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_init, ERRNO);

/**
 * gunyah_rm_vm_set_boot_context() - set the initial boot context of the primary vCPU
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM identifier
 * @reg_set: See &enum gunyah_vm_boot_context_reg
 * @reg_index: Which register to set; must be 0 for REG_SET_PC
 * @value: Value to set in the register
 */
int gunyah_rm_vm_set_boot_context(struct gunyah_rm *rm, u16 vmid, u8 reg_set,
				  u8 reg_index, u64 value)
{
	struct gunyah_rm_vm_set_boot_context_req req_payload = {
		.vmid = cpu_to_le16(vmid),
		.reg_set = reg_set,
		.reg_index = reg_index,
		.value = cpu_to_le64(value),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_SET_BOOT_CONTEXT,
			      &req_payload, sizeof(req_payload), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_set_boot_context, ERRNO);

/**
 * gunyah_rm_get_hyp_resources() - Retrieve hypervisor resources (capabilities) associated with a VM
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VMID of the other VM to get the resources of
 * @resources: Set by gunyah_rm_get_hyp_resources and contains the returned hypervisor resources.
 *             Caller must free the resources pointer if successful.
 */
int gunyah_rm_get_hyp_resources(struct gunyah_rm *rm, u16 vmid,
				struct gunyah_rm_hyp_resources **resources)
{
	struct gunyah_rm_vm_common_vmid_req req_payload = {
		.vmid = cpu_to_le16(vmid),
	};
	struct gunyah_rm_hyp_resources *resp;
	size_t resp_size;
	int ret;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_HYP_RESOURCES,
			     &req_payload, sizeof(req_payload), (void **)&resp,
			     &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size < struct_size(resp, entries, 0) ||
	    resp_size !=
		    struct_size(resp, entries, le32_to_cpu(resp->n_entries))) {
		kfree(resp);
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	*resources = resp;
	return 0;
}
ALLOW_ERROR_INJECTION(gunyah_rm_get_hyp_resources, ERRNO);

/**
 * gunyah_rm_device_find() - Locate a physical device managed by RM
 * @rm: Handle to a Gunyah resource manager
 * @resource: A resource descriptor uniquely identifying the device
 * @handle: Set to the RM device handle on success
 */
int gunyah_rm_device_find(struct gunyah_rm *rm,
			  const struct gunyah_device_resource *resource,
			  u32 *handle)
{
	struct gunyah_rm_device_find_resp *resp;
	size_t resp_size;
	int ret;

	if (!resource || !handle)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_DEVICE_FIND_HANDLE, resource,
			     sizeof(*resource), (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size != sizeof(*resp) ||
	    le32_to_cpu(resp->handle) == GUNYAH_DEVICE_HANDLE_INVALID) {
		kfree(resp);
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	*handle = le32_to_cpu(resp->handle);
	kfree(resp);
	return 0;
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_find, ERRNO);

/**
 * gunyah_rm_device_get_resources() - Return a device's RM resource descriptors
 * @rm: Handle to a Gunyah resource manager
 * @handle: Device handle returned by gunyah_rm_device_find()
 * @flags: See &enum gunyah_device_get_resources_flags
 * @resources: Set to a newly allocated resource array
 * @num_resources: Number of entries placed in @resources
 */
int gunyah_rm_device_get_resources(struct gunyah_rm *rm, u32 handle,
				   u32 flags,
				   struct gunyah_device_resource **resources,
				   u32 *num_resources)
{
	struct gunyah_rm_device_get_resources_req req = {
		.handle = cpu_to_le32(handle),
		.flags = flags,
	};
	struct gunyah_rm_device_get_resources_resp *resp;
	size_t body_size, resp_size;
	struct gunyah_device_resource *copy = NULL;
	u32 count;
	int ret;

	if (!resources || !num_resources ||
	    flags & ~GUNYAH_RM_DEVICE_GET_RESOURCES_PHYSICAL)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_DEVICE_GET_RESOURCES, &req,
			     sizeof(req), (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size < sizeof(*resp)) {
		kfree(resp);
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	count = le16_to_cpu(resp->num_entries);
	if (check_mul_overflow((size_t)count, sizeof(**resources), &body_size) ||
	    check_add_overflow((size_t)sizeof(*resp), body_size, &body_size) ||
	    body_size != resp_size) {
		kfree(resp);
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	if (count) {
		copy = kmemdup(resp->resources,
			       count * sizeof(**resources), GFP_KERNEL);
		if (!copy) {
			kfree(resp);
			return -ENOMEM;
		}
	}

	kfree(resp);
	*resources = copy;
	*num_resources = count;
	return 0;
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_get_resources, ERRNO);

static int gunyah_rm_device_transfer(struct gunyah_rm *rm, u32 message_id,
				     u16 vmid, u32 handle, bool reset)
{
	struct gunyah_rm_device_transfer_req req = {
		.vmid = cpu_to_le16(vmid),
		.flags = reset ? GUNYAH_RM_DEVICE_RESET_FLAG : 0,
		.handle = cpu_to_le32(handle),
	};

	return gunyah_rm_call(rm, message_id, &req, sizeof(req), NULL, NULL);
}

/**
 * gunyah_rm_device_accept() - Accept a device offered by another VM
 * @rm: Handle to a Gunyah resource manager
 * @handle: Device handle owned by or offered to this VM
 * @flags: RM accept flags, including reset and destination bus binding
 * @bus_handle: Destination bus when binding is requested
 */
int gunyah_rm_device_accept(struct gunyah_rm *rm, u32 handle, u32 flags, u32 bus_handle)
{
	struct gunyah_rm_device_accept_req req = {
		.handle = cpu_to_le32(handle),
		.flags = flags,
		.bus_handle = cpu_to_le32(bus_handle),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_DEVICE_ACCEPT, &req, sizeof(req),
			      NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_accept, ERRNO);

/**
 * gunyah_rm_device_release() - Return a borrowed device to its owner
 * @rm: Handle to a Gunyah resource manager
 * @handle: Device handle returned by gunyah_rm_device_find()
 * @reset: Reset the device while releasing it
 */
int gunyah_rm_device_release(struct gunyah_rm *rm, u32 handle, bool reset)
{
	struct gunyah_rm_device_release_req req = {
		.handle = cpu_to_le32(handle),
		.flags = reset ? GUNYAH_RM_DEVICE_RESET_FLAG : 0,
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_DEVICE_RELEASE, &req, sizeof(req),
			      NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_release, ERRNO);

/**
 * gunyah_rm_device_lend() - Lend a physical device to another VM
 * @rm: Handle to a Gunyah resource manager
 * @vmid: Recipient VMID
 * @handle: Device handle owned by this VM
 * @reset: Reset the device during transfer
 */
int gunyah_rm_device_lend(struct gunyah_rm *rm, u16 vmid, u32 handle,
			  bool reset)
{
	return gunyah_rm_device_transfer(rm, GUNYAH_RM_RPC_DEVICE_LEND, vmid,
					 handle, reset);
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_lend, ERRNO);

/**
 * gunyah_rm_device_donate() - Donate a physical device to another VM
 * @rm: Handle to a Gunyah resource manager
 * @vmid: Recipient VMID
 * @handle: Device handle owned by this VM
 * @reset: Reset the device during transfer
 */
int gunyah_rm_device_donate(struct gunyah_rm *rm, u16 vmid, u32 handle,
			    bool reset)
{
	return gunyah_rm_device_transfer(rm, GUNYAH_RM_RPC_DEVICE_DONATE, vmid,
					 handle, reset);
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_donate, ERRNO);

/**
 * gunyah_rm_device_reclaim() - Reclaim a device offered to another VM
 * @rm: Handle to a Gunyah resource manager
 * @handle: Device handle owned by this VM
 * @reset: Reset the device while reclaiming it
 *
 * The borrower VM must have released a lent device first.  Reclaim also
 * rolls back an offer that the borrower has not accepted yet.
 */
int gunyah_rm_device_reclaim(struct gunyah_rm *rm, u32 handle, bool reset)
{
	struct gunyah_rm_device_reclaim_req req = {
		.handle = cpu_to_le32(handle),
		.flags = reset ? GUNYAH_RM_DEVICE_RESET_FLAG : 0,
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_DEVICE_RECLAIM, &req, sizeof(req),
			      NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_device_reclaim, ERRNO);

/**
 * gunyah_rm_get_vmid() - Retrieve VMID of this virtual machine
 * @rm: Handle to a Gunyah resource manager
 * @vmid: Filled with the VMID of this VM
 */
int gunyah_rm_get_vmid(struct gunyah_rm *rm, u16 *vmid)
{
	static u16 cached_vmid = GUNYAH_VMID_INVAL;
	size_t resp_size;
	__le32 *resp;
	int ret;

	if (cached_vmid != GUNYAH_VMID_INVAL) {
		*vmid = cached_vmid;
		return 0;
	}

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_VMID, NULL, 0,
			     (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (resp_size != sizeof(*resp) || !resp ||
	    !lower_16_bits(le32_to_cpu(*resp)) ||
	    lower_16_bits(le32_to_cpu(*resp)) == GUNYAH_VMID_INVAL) {
		kfree(resp);
		gunyah_rm_mark_failed(rm);
		return -EBADMSG;
	}

	*vmid = cached_vmid = lower_16_bits(le32_to_cpu(*resp));
	kfree(resp);

	return ret;
}
EXPORT_SYMBOL_GPL(gunyah_rm_get_vmid);

/**
 * gunyah_rm_vm_set_demand_paging() - Enable demand paging of memory regions
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VMID of the other VM
 * @count: Number of demand paged memory regions
 * @entries: Array of the regions
 */
int gunyah_rm_vm_set_demand_paging(struct gunyah_rm *rm, u16 vmid, u32 count,
				   struct gunyah_rm_mem_entry *entries)
{
	struct gunyah_rm_vm_set_demand_paging_req *req __free(kfree) = NULL;
	size_t req_size;

	req_size = struct_size(req, ranges, count);
	if (req_size == SIZE_MAX)
		return -EINVAL;

	req = kzalloc(req_size, GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->vmid = cpu_to_le16(vmid);
	req->range_count = cpu_to_le32(count);
	memcpy(req->ranges, entries, sizeof(*entries) * count);

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_SET_DEMAND_PAGING, req,
			      req_size, NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_set_demand_paging, ERRNO);

/**
 * gunyah_rm_vm_set_address_layout() - Set the start address of images
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VMID of the other VM
 * @range_id: Which image to set
 * @base_address: Base address
 * @size: Size
 */
int gunyah_rm_vm_set_address_layout(struct gunyah_rm *rm, u16 vmid,
				    enum gunyah_rm_range_id range_id,
				    u64 base_address, u64 size)
{
	struct gunyah_rm_vm_set_address_layout_req req = {
		.vmid = cpu_to_le16(vmid),
		.range_id = cpu_to_le32(range_id),
		.range_base = cpu_to_le64(base_address),
		.range_size = cpu_to_le64(size),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_SET_ADDRESS_LAYOUT, &req,
			      sizeof(req), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_set_address_layout, ERRNO);

/**
 * gunyah_rm_vm_set_firmware_mem() - Set the location of firmware for GH_RM_VM_AUTH_QCOM_ANDROID_PVM VMs
 * @rm: Handle to a Gunyah resource manager.
 * @vmid: VM identifier allocated with gh_rm_alloc_vmid.
 * @parcel: Memory parcel where the firmware should be loaded.
 * @fw_offset: offset into the memory parcel where the firmware should be loaded.
 * @fw_size: Maxmimum size of the fw that can be loaded.
 */
int gunyah_rm_vm_set_firmware_mem(struct gunyah_rm *rm, u16 vmid, struct gunyah_rm_mem_parcel *parcel,
				u64 fw_offset, u64 fw_size)
{
	struct gunyah_vm_set_firmware_mem_req req = {
		.vmid = cpu_to_le16(vmid),
		.mem_handle = cpu_to_le32(parcel->mem_handle),
		.fw_offset = cpu_to_le64(fw_offset),
		.fw_size = cpu_to_le64(fw_size),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_SET_FIRMWARE_MEM, &req, sizeof(req), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_set_firmware_mem, ERRNO);

/* Message IDs: VM registry, host identity and guest services */
/* clang-format off */
#define GUNYAH_RM_RPC_VM_GET_ID			0x56000010
#define GUNYAH_RM_RPC_VM_LOOKUP_URI		0x56000011
#define GUNYAH_RM_RPC_VM_LOOKUP_GUID		0x56000012
#define GUNYAH_RM_RPC_VM_LOOKUP_NAME		0x56000013
#define GUNYAH_RM_RPC_VM_GET_OWNER		0x56000015
#define GUNYAH_RM_RPC_VM_GET_CRASH_MSG		0x56000019
#define GUNYAH_RM_RPC_VM_GET_PEERS		0x56000025
#define GUNYAH_RM_RPC_VM_SET_TIME_BASE		0x56000030
#define GUNYAH_RM_RPC_VM_HOST_GET_TYPE		0x560000A0
#define GUNYAH_RM_RPC_VM_IRQ_ACCEPT		0x56000050
#define GUNYAH_RM_RPC_VM_IRQ_LEND		0x56000051
#define GUNYAH_RM_RPC_VM_IRQ_RELEASE		0x56000052
#define GUNYAH_RM_RPC_VM_IRQ_RECLAIM		0x56000053
/* clang-format on */

struct gunyah_rm_vm_string_id_req {
	__le16 len;
	__le16 reserved;
	u8 key[];
} __packed;

struct gunyah_rm_vm_time_base_req {
	__le16 vmid;
	__le16 reserved;
	__le64 time_base;
	__le64 sys_timer_ref;
} __packed;

struct gunyah_rm_vm_irq_lend_req {
	__le16 borrower;
	__le16 reserved;
	__le32 virq;
	__le32 label;
} __packed;

struct gunyah_rm_vm_irq_accept_req {
	__le32 handle;
	__le32 virq;
} __packed;

/**
 * gunyah_rm_vm_lookup() - Find a VM identifier in the RM VM registry
 * @rm: Handle to a Gunyah resource manager
 * @type: Kind of key supplied
 * @key: Key bytes (URI or name string, or a 16 byte GUID)
 * @key_len: Length of @key
 * @vmid: Set to the first matching VM identifier
 *
 * Return: 0 on success, -ENOENT if nothing matched, negative errno otherwise.
 */
int gunyah_rm_vm_lookup(struct gunyah_rm *rm, enum gunyah_rm_vm_id_type type,
			const void *key, size_t key_len, u16 *vmid)
{
	struct gunyah_rm_vm_string_id_req *string_req;
	__le32 *resp __free(kfree) = NULL;
	u8 *req __free(kfree) = NULL;
	size_t req_size, resp_size;
	u32 message_id, count;
	int ret;

	if (!key || !vmid)
		return -EINVAL;

	switch (type) {
	case GUNYAH_RM_VM_ID_URI:
	case GUNYAH_RM_VM_ID_NAME:
		if (!key_len || key_len > GUNYAH_RM_VM_MAX_NAME_LEN)
			return -EINVAL;
		message_id = type == GUNYAH_RM_VM_ID_URI ?
				     GUNYAH_RM_RPC_VM_LOOKUP_URI :
				     GUNYAH_RM_RPC_VM_LOOKUP_NAME;
		req_size = sizeof(*string_req) + ALIGN(key_len, 4);
		req = kzalloc(req_size, GFP_KERNEL);
		if (!req)
			return -ENOMEM;
		string_req = (struct gunyah_rm_vm_string_id_req *)req;
		string_req->len = cpu_to_le16(key_len);
		memcpy(string_req->key, key, key_len);
		break;
	case GUNYAH_RM_VM_ID_GUID:
		if (key_len != GUNYAH_RM_VM_GUID_LEN)
			return -EINVAL;
		message_id = GUNYAH_RM_RPC_VM_LOOKUP_GUID;
		req_size = GUNYAH_RM_VM_GUID_LEN;
		req = kmemdup(key, req_size, GFP_KERNEL);
		if (!req)
			return -ENOMEM;
		break;
	default:
		return -EINVAL;
	}

	ret = gunyah_rm_call(rm, message_id, req, req_size, (void **)&resp,
			     &resp_size);
	if (ret)
		return ret;

	/* FIFO replies carry the RM error in the header; the payload is the
	 * match count followed by the matching VMIDs. */
	if (!resp || resp_size < sizeof(u32))
		goto malformed;

	count = le32_to_cpu(resp[0]);
	if (!count)
		return -ENOENT;
	if (resp_size < sizeof(u32) + count * sizeof(u32))
		goto malformed;

	*vmid = (u16)le32_to_cpu(resp[1]);
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_lookup, ERRNO);

/**
 * gunyah_rm_vm_get_owner() - Read the owner VMID the RM recorded for us
 * @rm: Handle to a Gunyah resource manager
 * @owner: Set to the owner VMID
 *
 * The RM answers about the calling VM, so this reports the host's owner.
 *
 * Return: 0 on success, negative errno otherwise.
 */
int gunyah_rm_vm_get_owner(struct gunyah_rm *rm, u16 *owner)
{
	__le32 *resp __free(kfree) = NULL;
	size_t resp_size;
	int ret;

	if (!owner)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_OWNER, NULL, 0,
			     (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size < sizeof(*resp))
		goto malformed;

	*owner = (u16)le32_to_cpu(*resp);
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_get_owner, ERRNO);

/**
 * gunyah_rm_vm_get_peers() - List the peer VMIDs the RM registered for us
 * @rm: Handle to a Gunyah resource manager
 * @buf: Destination for the peer VMID array
 * @buf_size: Capacity of @buf
 * @payload_size: Set to the number of bytes the RM returned
 * @n_entries: Set to the number of peer VMIDs
 *
 * Return: 0 on success, -ENOSPC if @buf is too small, negative errno otherwise.
 */
int gunyah_rm_vm_get_peers(struct gunyah_rm *rm, void *buf, size_t buf_size,
			   size_t *payload_size, u32 *n_entries)
{
	__le32 *resp __free(kfree) = NULL;
	size_t resp_size;
	u32 count;
	int ret;

	if (!buf || !payload_size)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_PEERS, NULL, 0,
			     (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size < sizeof(*resp))
		goto malformed;

	count = le32_to_cpu(resp[0]);
	resp_size -= sizeof(*resp);
	if (resp_size != count * sizeof(*resp))
		goto malformed;
	if (resp_size > buf_size)
		return -ENOSPC;

	memcpy(buf, resp + 1, resp_size);
	*payload_size = resp_size;
	if (n_entries)
		*n_entries = count;
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_get_peers, ERRNO);

/**
 * gunyah_rm_vm_get_id() - Read the RM identifier blob of a VM
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM to query, 0 for the caller
 * @buf: Destination for the identifier entries
 * @buf_size: Capacity of @buf
 * @payload_size: Set to the number of bytes the RM returned
 * @n_entries: Set to the number of identifier entries
 *
 * Return: 0 on success, -ENOSPC if @buf is too small, negative errno otherwise.
 */
int gunyah_rm_vm_get_id(struct gunyah_rm *rm, u16 vmid, void *buf,
			size_t buf_size, size_t *payload_size, u16 *n_entries)
{
	struct gunyah_rm_vm_common_vmid_req req = {
		.vmid = cpu_to_le16(vmid),
	};
	u8 *resp __free(kfree) = NULL;
	size_t resp_size;
	u32 entries;
	int ret;

	if (!buf || !payload_size)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_ID, &req, sizeof(req),
			     (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size < sizeof(u32))
		goto malformed;

	entries = le32_to_cpu(*(__le32 *)resp);
	resp_size -= sizeof(u32);
	memmove(resp, resp + sizeof(u32), resp_size);

	if (resp_size > buf_size)
		return -ENOSPC;

	memcpy(buf, resp, resp_size);
	*payload_size = resp_size;
	if (n_entries)
		*n_entries = (u16)entries;
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_get_id, ERRNO);

/**
 * gunyah_rm_vm_get_crash_msg() - Read the crash message the VM reported
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM to query
 * @buf: Destination buffer
 * @buf_size: Capacity of @buf
 * @msg_len: Set to the length of the stored message
 *
 * Return: 0 on success, -ENOSPC if @buf is too small, negative errno otherwise.
 */
int gunyah_rm_vm_get_crash_msg(struct gunyah_rm *rm, u16 vmid, void *buf,
			       size_t buf_size, size_t *msg_len)
{
	struct gunyah_rm_vm_common_vmid_req req = {
		.vmid = cpu_to_le16(vmid),
	};
	u8 *resp __free(kfree) = NULL;
	size_t resp_size, len;
	int ret;

	if (!buf || !msg_len)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_GET_CRASH_MSG, &req,
			     sizeof(req), (void **)&resp, &resp_size);
	if (ret)
		return ret;

	/* length (2) | padding (2) | message */
	if (!resp || resp_size < 4)
		goto malformed;

	len = le16_to_cpu(*(__le16 *)resp);
	if (resp_size < 4 + len)
		goto malformed;

	*msg_len = len;
	if (len > buf_size)
		return -ENOSPC;

	memcpy(buf, resp + 4, len);
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_get_crash_msg, ERRNO);

/**
 * gunyah_rm_vm_set_time_base() - Set the virtual RTC reference of a VM
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM in INIT or READY state
 * @time_base: Guest time base in nanoseconds
 * @sys_timer_ref: Host system timer value matching @time_base
 */
int gunyah_rm_vm_set_time_base(struct gunyah_rm *rm, u16 vmid, u64 time_base,
			       u64 sys_timer_ref)
{
	struct gunyah_rm_vm_time_base_req req = {
		.vmid = cpu_to_le16(vmid),
		.time_base = cpu_to_le64(time_base),
		.sys_timer_ref = cpu_to_le64(sys_timer_ref),
	};

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_SET_TIME_BASE, &req,
			      sizeof(req), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_vm_set_time_base, ERRNO);

/**
 * gunyah_rm_host_get_type() - Query the RM's view of the host
 * @rm: Handle to a Gunyah resource manager
 * @host_type: Set to the RM host type word (arch/type/version fields)
 */
int gunyah_rm_host_get_type(struct gunyah_rm *rm, u32 *host_type)
{
	__le32 *resp __free(kfree) = NULL;
	size_t resp_size;
	int ret;

	if (!host_type)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_HOST_GET_TYPE, NULL, 0,
			     (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size != sizeof(*resp))
		goto malformed;

	*host_type = le32_to_cpu(*resp);
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_host_get_type, ERRNO);

/**
 * gunyah_rm_irq_lend() - Offer an interrupt of this VM to another VM
 * @rm: Handle to a Gunyah resource manager
 * @borrower: VM that may accept the interrupt
 * @virq: Interrupt number in this VM
 * @label: Caller-defined label echoed back in notifications
 * @handle: Set to the RM handle for the lent interrupt
 */
int gunyah_rm_irq_lend(struct gunyah_rm *rm, u16 borrower, u32 virq,
		       u32 label, u32 *handle)
{
	struct gunyah_rm_vm_irq_lend_req req = {
		.borrower = cpu_to_le16(borrower),
		.virq = cpu_to_le32(virq),
		.label = cpu_to_le32(label),
	};
	__le32 *resp __free(kfree) = NULL;
	size_t resp_size;
	int ret;

	if (!handle)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_IRQ_LEND, &req, sizeof(req),
			     (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size != sizeof(*resp))
		goto malformed;

	*handle = le32_to_cpu(*resp);
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_irq_lend, ERRNO);

/**
 * gunyah_rm_irq_release() - Withdraw a lent interrupt from the borrower
 * @rm: Handle to a Gunyah resource manager
 * @handle: Handle returned by gunyah_rm_irq_lend()
 */
int gunyah_rm_irq_release(struct gunyah_rm *rm, u32 handle)
{
	__le32 req = cpu_to_le32(handle);

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_IRQ_RELEASE, &req,
			      sizeof(req), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_irq_release, ERRNO);

/**
 * gunyah_rm_irq_reclaim() - Take a lent interrupt back
 * @rm: Handle to a Gunyah resource manager
 * @handle: Handle returned by gunyah_rm_irq_lend()
 */
int gunyah_rm_irq_reclaim(struct gunyah_rm *rm, u32 handle)
{
	__le32 req = cpu_to_le32(handle);

	return gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_IRQ_RECLAIM, &req,
			      sizeof(req), NULL, NULL);
}
ALLOW_ERROR_INJECTION(gunyah_rm_irq_reclaim, ERRNO);

/**
 * gunyah_rm_irq_accept() - Accept an interrupt lent to this VM
 * @rm: Handle to a Gunyah resource manager
 * @handle: Handle received in a lend notification
 * @virq: Destination interrupt number, or %GUNYAH_VIRQ_AUTO
 * @virq_out: Set to the interrupt number the RM assigned
 */
int gunyah_rm_irq_accept(struct gunyah_rm *rm, u32 handle, u32 virq,
			 u32 *virq_out)
{
	struct gunyah_rm_vm_irq_accept_req req = {
		.handle = cpu_to_le32(handle),
		.virq = cpu_to_le32(virq),
	};
	__le32 *resp __free(kfree) = NULL;
	size_t resp_size;
	int ret;

	if (!virq_out)
		return -EINVAL;

	ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_IRQ_ACCEPT, &req,
			     sizeof(req), (void **)&resp, &resp_size);
	if (ret)
		return ret;

	if (!resp || resp_size != sizeof(*resp))
		goto malformed;

	*virq_out = le32_to_cpu(*resp);
	return 0;

malformed:
	gunyah_rm_mark_failed(rm);
	return -EBADMSG;
}
ALLOW_ERROR_INJECTION(gunyah_rm_irq_accept, ERRNO);

/* Message IDs: guest console */
/* clang-format off */
#define GUNYAH_RM_RPC_VM_CONSOLE_OPEN	0x56000081
#define GUNYAH_RM_RPC_VM_CONSOLE_CLOSE	0x56000082
#define GUNYAH_RM_RPC_VM_CONSOLE_WRITE	0x56000083
#define GUNYAH_RM_RPC_VM_CONSOLE_FLUSH	0x56000084
/* clang-format on */

struct gunyah_rm_vm_console_req {
	__le16 vmid;
	__le16 reserved;
} __packed;

struct gunyah_rm_vm_console_write_req {
	__le16 vmid;
	__le16 num_bytes;
	u8 data[];
} __packed;

/* The RM validates the payload length exactly, so write in bounded chunks. */
#define GUNYAH_RM_CONSOLE_CHUNK	1024

static int gunyah_rm_console_cmd(struct gunyah_rm *rm, u32 message_id, u16 vmid)
{
	struct gunyah_rm_vm_console_req req = {
		.vmid = cpu_to_le16(vmid),
	};

	return gunyah_rm_call(rm, message_id, &req, sizeof(req), NULL, NULL);
}

/**
 * gunyah_rm_console_open() - Start receiving a VM's console output
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM whose console to open
 */
int gunyah_rm_console_open(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_console_cmd(rm, GUNYAH_RM_RPC_VM_CONSOLE_OPEN, vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_console_open, ERRNO);

/**
 * gunyah_rm_console_close() - Stop receiving a VM's console output
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM whose console to close
 */
int gunyah_rm_console_close(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_console_cmd(rm, GUNYAH_RM_RPC_VM_CONSOLE_CLOSE, vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_console_close, ERRNO);

/**
 * gunyah_rm_console_flush() - Ask the RM to flush pending console output
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM whose console to flush
 */
int gunyah_rm_console_flush(struct gunyah_rm *rm, u16 vmid)
{
	return gunyah_rm_console_cmd(rm, GUNYAH_RM_RPC_VM_CONSOLE_FLUSH, vmid);
}
ALLOW_ERROR_INJECTION(gunyah_rm_console_flush, ERRNO);

/**
 * gunyah_rm_console_write() - Send bytes to a VM's console input
 * @rm: Handle to a Gunyah resource manager
 * @vmid: VM whose console receives the bytes
 * @data: Bytes to send
 * @len: Number of bytes
 *
 * The RM requires the console to be open on both ends and rejects any payload
 * whose length does not exactly match the declared byte count, so long writes
 * are split into chunks.
 */
int gunyah_rm_console_write(struct gunyah_rm *rm, u16 vmid, const void *data,
			    size_t len)
{
	struct gunyah_rm_vm_console_write_req *req;
	size_t chunk, sent = 0;
	int ret;

	if (!data && len)
		return -EINVAL;

	while (sent < len) {
		chunk = min_t(size_t, len - sent, GUNYAH_RM_CONSOLE_CHUNK);
		req = kmalloc(sizeof(*req) + chunk, GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		req->vmid = cpu_to_le16(vmid);
		req->num_bytes = cpu_to_le16(chunk);
		memcpy(req->data, (const u8 *)data + sent, chunk);

		ret = gunyah_rm_call(rm, GUNYAH_RM_RPC_VM_CONSOLE_WRITE, req,
				     sizeof(*req) + chunk, NULL, NULL);
		kfree(req);
		if (ret)
			return ret;

		sent += chunk;
	}

	return 0;
}
ALLOW_ERROR_INJECTION(gunyah_rm_console_write, ERRNO);
