/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __GUNYAH_RSC_MGR_PRIV_H
#define __GUNYAH_RSC_MGR_PRIV_H

#include <linux/gunyah.h>
#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/types.h>

#define GUNYAH_VMID_INVAL U16_MAX
#define GUNYAH_MEM_HANDLE_INVAL U32_MAX

struct gunyah_rm;

bool gunyah_rm_is_failed(struct gunyah_rm *rm);
void gunyah_rm_mark_failed(struct gunyah_rm *rm);

int gunyah_rm_notifier_register(struct gunyah_rm *rm,
				struct notifier_block *nb);
int gunyah_rm_notifier_unregister(struct gunyah_rm *rm,
				  struct notifier_block *nb);
struct device *gunyah_rm_get(struct gunyah_rm *rm);
bool gunyah_rm_has_demand_paging(struct gunyah_rm *rm);
bool gunyah_rm_has_device_passthrough(struct gunyah_rm *rm);
void gunyah_rm_put(struct gunyah_rm *rm);

struct gunyah_rm_vm_exited_payload {
	__le16 vmid;
	__le16 exit_type;
	__le32 exit_reason_size;
	u8 exit_reason[];
} __packed;

enum gunyah_rm_notification_id {
	/* clang-format off */
	GUNYAH_RM_NOTIFICATION_VM_EXITED		 = 0x56100001,
	GUNYAH_RM_NOTIFICATION_VM_SHUTDOWN		 = 0x56100002,
	GUNYAH_RM_NOTIFICATION_VM_STATUS		 = 0x56100008,
	GUNYAH_RM_NOTIFICATION_IRQ_LENT			 = 0x56100011,
	GUNYAH_RM_NOTIFICATION_IRQ_RELEASED		 = 0x56100012,
	GUNYAH_RM_NOTIFICATION_IRQ_ACCEPTED		 = 0x56100013,
	GUNYAH_RM_NOTIFICATION_CONSOLE_CHARS		 = 0x56100080,
	/* clang-format on */
};

enum gunyah_rm_vm_status {
	/* clang-format off */
	GUNYAH_RM_VM_STATUS_NO_STATE		= 0,
	GUNYAH_RM_VM_STATUS_INIT		= 1,
	GUNYAH_RM_VM_STATUS_READY		= 2,
	GUNYAH_RM_VM_STATUS_RUNNING		= 3,
	GUNYAH_RM_VM_STATUS_PAUSED		= 4,
	GUNYAH_RM_VM_STATUS_LOAD		= 5,
	GUNYAH_RM_VM_STATUS_AUTH		= 6,
	GUNYAH_RM_VM_STATUS_INIT_FAILED		= 8,
	GUNYAH_RM_VM_STATUS_EXITED		= 9,
	GUNYAH_RM_VM_STATUS_RESETTING		= 10,
	GUNYAH_RM_VM_STATUS_RESET		= 11,
	GUNYAH_RM_VM_STATUS_RESET_FAILED	= 12,
	/* clang-format on */
};

/* Interrupt lending notifications, one queue per resource manager */
enum gunyah_rm_irq_event_type {
	GUNYAH_RM_IRQ_LENT = 1,
	GUNYAH_RM_IRQ_RELEASED = 2,
	GUNYAH_RM_IRQ_ACCEPTED = 3,
};

struct gunyah_rm_irq_event {
	struct list_head node;
	enum gunyah_rm_irq_event_type type;
	u32 handle;
	u32 label;
	u16 vmid;
	u32 flags;
};

#define GUNYAH_RM_IRQ_EVENT_QUEUE_LEN	32

/* Payload of a console character notification from the resource manager */
struct gunyah_rm_console_chars_payload {
	__le16 from;
	__le16 num_bytes;
	u8 data[];
} __packed;

struct gunyah_rm_vm_status_payload {
	__le16 vmid;
	u16 reserved;
	u8 vm_status;
	u8 os_status;
	__le16 app_status;
} __packed;

/* RPC Calls */
int gunyah_rm_mem_share(struct gunyah_rm *rm,
			struct gunyah_rm_mem_parcel *parcel);
int gunyah_rm_mem_reclaim(struct gunyah_rm *rm,
			  struct gunyah_rm_mem_parcel *parcel);

static inline bool
gunyah_rm_mem_reclaim_needed(const struct gunyah_rm_mem_parcel *parcel)
{
	return parcel->mem_handle != GUNYAH_MEM_HANDLE_INVAL ||
		parcel->rm_unknown ||
		parcel->n_shared_mem_entries > parcel->n_reclaimed_mem_entries;
}

int gunyah_rm_alloc_vmid(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_dealloc_vmid(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_vm_get_state(struct gunyah_rm *rm, u16 vmid,
			   enum gunyah_rm_vm_status *vm_status);
int gunyah_rm_vm_reset(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_vm_start(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_vm_stop(struct gunyah_rm *rm, u16 vmid);

enum gunyah_rm_vm_auth_mechanism {
	/* clang-format off */
	GUNYAH_RM_VM_AUTH_NONE			= 0,
	GUNYAH_RM_VM_AUTH_QCOM_TRUSTED_VM	= 1,
	GUNYAH_RM_VM_AUTH_QCOM_ANDROID_PVM	= 2,
	/* clang-format on */
};

#define GUNYAH_VM_AUTH_PARAM_PAS_ID		0
struct gunyah_rm_vm_authenticate_param_entry {
	u32 param_type;
	u32 param;
} __packed;

int gunyah_rm_vm_authenticate(struct gunyah_rm *rm, u16 vmid,
			   ssize_t n_entries,
			   struct gunyah_rm_vm_authenticate_param_entry *entry);
int gunyah_rm_vm_configure(struct gunyah_rm *rm, u16 vmid,
			   enum gunyah_rm_vm_auth_mechanism auth_mechanism,
			   u32 mem_handle, u64 image_offset, u64 image_size,
			   u64 dtb_offset, u64 dtb_size);
int gunyah_rm_vm_init(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_vm_set_boot_context(struct gunyah_rm *rm, u16 vmid, u8 reg_set,
				  u8 reg_index, u64 value);

struct gunyah_rm_hyp_resource {
	u8 type;
	u8 reserved;
	__le16 partner_vmid;
	__le32 resource_handle;
	__le32 resource_label;
	__le64 cap_id;
	__le32 virq_handle;
	__le32 virq;
	__le64 base;
	__le64 size;
} __packed;

struct gunyah_rm_hyp_resources {
	__le32 n_entries;
	struct gunyah_rm_hyp_resource entries[];
} __packed;

int gunyah_rm_get_hyp_resources(struct gunyah_rm *rm, u16 vmid,
				struct gunyah_rm_hyp_resources **resources);

int gunyah_rm_device_find(struct gunyah_rm *rm,
			  const struct gunyah_device_resource *resource,
			  u32 *handle);
int gunyah_rm_device_get_resources(struct gunyah_rm *rm, u32 handle,
				   u32 flags,
				   struct gunyah_device_resource **resources,
				   u32 *num_resources);
int gunyah_rm_device_accept(struct gunyah_rm *rm, u32 handle, u32 flags, u32 bus_handle);
int gunyah_rm_device_release(struct gunyah_rm *rm, u32 handle, bool reset);
int gunyah_rm_device_lend(struct gunyah_rm *rm, u16 vmid, u32 handle,
			  bool reset);
int gunyah_rm_device_donate(struct gunyah_rm *rm, u16 vmid, u32 handle,
			    bool reset);
int gunyah_rm_device_reclaim(struct gunyah_rm *rm, u32 handle, bool reset);

int gunyah_rm_get_vmid(struct gunyah_rm *rm, u16 *vmid);

/* VM registry, host identity and guest service RPCs */
enum gunyah_rm_vm_id_type {
	GUNYAH_RM_VM_ID_URI = 0,
	GUNYAH_RM_VM_ID_GUID = 1,
	GUNYAH_RM_VM_ID_NAME = 2,
};

#define GUNYAH_RM_VM_MAX_NAME_LEN	79
#define GUNYAH_RM_VM_MAX_URI_LEN	79
#define GUNYAH_RM_VM_GUID_LEN		16
#define GUNYAH_RM_VM_MAX_CRASH_MSG_LEN	192

int gunyah_rm_vm_lookup(struct gunyah_rm *rm, enum gunyah_rm_vm_id_type type,
			const void *key, size_t key_len, u16 *vmid);
int gunyah_rm_vm_get_id(struct gunyah_rm *rm, u16 vmid, void *buf,
			size_t buf_size, size_t *payload_size, u16 *n_entries);
int gunyah_rm_vm_get_owner(struct gunyah_rm *rm, u16 *owner);
int gunyah_rm_vm_get_peers(struct gunyah_rm *rm, void *buf, size_t buf_size,
			   size_t *payload_size, u32 *n_entries);
int gunyah_rm_vm_get_crash_msg(struct gunyah_rm *rm, u16 vmid, void *buf,
			       size_t buf_size, size_t *msg_len);
int gunyah_rm_vm_set_time_base(struct gunyah_rm *rm, u16 vmid, u64 time_base,
			       u64 sys_timer_ref);
int gunyah_rm_host_get_type(struct gunyah_rm *rm, u32 *host_type);

/* Interrupt lending */
int gunyah_rm_irq_lend(struct gunyah_rm *rm, u16 borrower, u32 virq,
		       u32 label, u32 *handle);
int gunyah_rm_irq_release(struct gunyah_rm *rm, u32 handle);
int gunyah_rm_irq_reclaim(struct gunyah_rm *rm, u32 handle);
int gunyah_rm_irq_accept(struct gunyah_rm *rm, u32 handle, u32 virq,
			 u32 *virq_out);
int gunyah_rm_irq_event_get(struct gunyah_rm *rm,
			    struct gunyah_rm_irq_event *event);

/* Guest console */
int gunyah_rm_console_open(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_console_close(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_console_flush(struct gunyah_rm *rm, u16 vmid);
int gunyah_rm_console_write(struct gunyah_rm *rm, u16 vmid, const void *data,
			    size_t len);

int gunyah_rm_vm_set_demand_paging(struct gunyah_rm *rm, u16 vmid, u32 count,
				   struct gunyah_rm_mem_entry *mem_entries);

enum gunyah_rm_range_id {
	GUNYAH_RM_RANGE_ID_IMAGE = 0,
	GUNYAH_RM_RANGE_ID_FIRMWARE = 1,
};

int gunyah_rm_vm_set_firmware_mem(struct gunyah_rm *rm, u16 vmid, struct gunyah_rm_mem_parcel *parcel,
				u64 fw_offset, u64 fw_size);

int gunyah_rm_vm_set_address_layout(struct gunyah_rm *rm, u16 vmid,
				    enum gunyah_rm_range_id range_id,
				    u64 base_address, u64 size);

struct gunyah_resource *
gunyah_rm_alloc_resource(struct gunyah_rm *rm,
			 struct gunyah_rm_hyp_resource *hyp_resource);
void gunyah_rm_free_resource(struct gunyah_resource *ghrsc);

int gunyah_rm_call(struct gunyah_rm *rsc_mgr, u32 message_id,
		   const void *req_buf, size_t req_buf_size, void **resp_buf,
		   size_t *resp_buf_size);

int gunyah_rm_platform_pre_mem_share(struct gunyah_rm *rm,
				     struct gunyah_rm_mem_parcel *mem_parcel);
int gunyah_rm_platform_post_mem_reclaim(
	struct gunyah_rm *rm, struct gunyah_rm_mem_parcel *mem_parcel);

struct gunyah_rm_platform_ref *gunyah_rm_platform_get(void);
void gunyah_rm_platform_put(struct gunyah_rm_platform_ref *ref);

int gunyah_rm_platform_pre_demand_page(struct gunyah_rm *rm, u16 vmid,
				       enum gunyah_pagetable_access access,
				       struct folio *folio,
				       struct gunyah_rm_platform_ref *ref);
int gunyah_rm_platform_reclaim_demand_page(struct gunyah_rm *rm, u16 vmid,
					   enum gunyah_pagetable_access access,
					   struct folio *folio,
					   struct gunyah_rm_platform_ref *ref);

#endif
