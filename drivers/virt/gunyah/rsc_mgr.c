// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#define pr_fmt(fmt) "gunyah: " fmt

#include <linux/completion.h>
#include <linux/gunyah.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/auxiliary_bus.h>
#include <linux/libfdt.h>
#include <linux/mm.h>
#include <linux/suspend.h>
#include <linux/vmalloc.h>

#include <asm/gunyah.h>

#include "rsc_mgr.h"
#include "vm_mgr.h"

/* clang-format off */
#define RM_RPC_API_VERSION_MASK		GENMASK(3, 0)
#define RM_RPC_HEADER_WORDS_MASK	GENMASK(7, 4)
#define RM_RPC_API_VERSION		FIELD_PREP(RM_RPC_API_VERSION_MASK, 1)
#define RM_RPC_HEADER_WORDS		FIELD_PREP(RM_RPC_HEADER_WORDS_MASK, \
						(sizeof(struct gunyah_rm_rpc_hdr) / sizeof(u32)))
#define RM_RPC_API			(RM_RPC_API_VERSION | RM_RPC_HEADER_WORDS)

#define RM_RPC_TYPE_CONTINUATION	0x0
#define RM_RPC_TYPE_REQUEST		0x1
#define RM_RPC_TYPE_REPLY		0x2
#define RM_RPC_TYPE_NOTIF		0x3
#define RM_RPC_TYPE_MASK		GENMASK(1, 0)

#define GUNYAH_RM_MAX_NUM_FRAGMENTS		62
#define RM_RPC_FRAGMENTS_MASK		GENMASK(7, 2)
/* clang-format on */

struct gunyah_rm_rpc_hdr {
	u8 api;
	u8 type;
	__le16 seq;
	__le32 msg_id;
} __packed;

struct gunyah_rm_rpc_reply_hdr {
	struct gunyah_rm_rpc_hdr hdr;
	__le32 err_code; /* GUNYAH_RM_ERROR_* */
} __packed;

#define GUNYAH_RM_MSGQ_MSG_SIZE 240
#define GUNYAH_RM_RPC_TIMEOUT_MS 30000
#define GUNYAH_RM_PAYLOAD_SIZE \
	(GUNYAH_RM_MSGQ_MSG_SIZE - sizeof(struct gunyah_rm_rpc_hdr))

/* RM Error codes */
enum gunyah_rm_error {
	/* clang-format off */
	GUNYAH_RM_ERROR_OK			= 0x0,
	GUNYAH_RM_ERROR_UNIMPLEMENTED		= 0xFFFFFFFF,
	GUNYAH_RM_ERROR_NOMEM			= 0x1,
	GUNYAH_RM_ERROR_NORESOURCE		= 0x2,
	GUNYAH_RM_ERROR_DENIED			= 0x3,
	GUNYAH_RM_ERROR_INVALID			= 0x4,
	GUNYAH_RM_ERROR_BUSY			= 0x5,
	GUNYAH_RM_ERROR_ARGUMENT_INVALID	= 0x6,
	GUNYAH_RM_ERROR_HANDLE_INVALID		= 0x7,
	GUNYAH_RM_ERROR_VALIDATE_FAILED		= 0x8,
	GUNYAH_RM_ERROR_MAP_FAILED		= 0x9,
	GUNYAH_RM_ERROR_MEM_INVALID		= 0xA,
	GUNYAH_RM_ERROR_MEM_INUSE		= 0xB,
	GUNYAH_RM_ERROR_MEM_RELEASED		= 0xC,
	GUNYAH_RM_ERROR_VMID_INVALID		= 0xD,
	GUNYAH_RM_ERROR_LOOKUP_FAILED		= 0xE,
	GUNYAH_RM_ERROR_IRQ_INVALID		= 0xF,
	GUNYAH_RM_ERROR_IRQ_INUSE		= 0x10,
	GUNYAH_RM_ERROR_IRQ_RELEASED		= 0x11,
	/* clang-format on */
};

struct gunyah_rm_info {
	__le64 tx_msgq_cap;
	__le64 rx_msgq_cap;
	__le32 tx_msgq_irq;
	__le32 rx_msgq_irq;
	__le32 tx_msgq_queue_depth;
	__le32 tx_msgq_max_msg_size;
	__le32 rx_msgq_queue_depth;
	__le32 rx_msgq_max_msg_size;
};

/**
 * struct gunyah_rm_message - Represents a complete message from resource manager
 * @payload: Combined payload of all the fragments (msg headers stripped off).
 * @size: Size of the payload received so far.
 * @msg_id: Message ID from the header.
 * @type: RM_RPC_TYPE_REPLY or RM_RPC_TYPE_NOTIF.
 * @num_fragments: total number of fragments expected to be received.
 * @fragments_received: fragments received so far.
 * @reply: Fields used for request/reply sequences
 */
struct gunyah_rm_message {
	void *payload;
	size_t size;
	u32 msg_id;
	u8 type;

	u8 num_fragments;
	u8 fragments_received;

	/**
	 * @ret: Linux return code, there was an error processing message
	 * @seq: Sequence ID for the main message.
	 * @rm_error: For request/reply sequences with standard replies
	 * @seq_done: Signals caller that the RM reply has been received
	 */
	struct {
		int ret;
		u16 seq;
		enum gunyah_rm_error rm_error;
		struct completion seq_done;
	} reply;
};

/**
 * struct gunyah_rm - private data for communicating w/Gunyah resource manager
 * @tx_ghrsc: message queue resource to TX to RM
 * @rx_ghrsc: message queue resource to RX from RM
 * @active_rx_message: ongoing gunyah_rm_message for which we're receiving fragments
 * @call_xarray: xarray to allocate & lookup sequence IDs for Request/Response flows
 * @next_seq: next ID to allocate (for xa_alloc_cyclic)
 * @recv_msg: cached allocation for Rx messages
 * @send_msg: cached allocation for Tx messages. Must hold @send_lock to manipulate.
 * @send_lock: synchronization to allow only one request to be sent at a time
 * @recv_lock: synchronization of reply parsing and cancellation
 * @failed: request outcome is unknown; no further RPCs are permitted
 * @send_ready: completed when we know Tx message queue can take more messages
 * @nh: notifier chain for clients interested in RM notification messages
 * @adev: auxiliary device for RM clients
 * @adev_released: signals that no references to the embedded auxiliary device remain
 * @miscdev: /dev/gunyah
 * @parent_fwnode: Parent IRQ fwnode to translate Gunyah hwirqs to Linux irqs
 * @pm_nb: hibernation notifier for info-area resources
 * @pm_registered: @pm_nb was successfully registered
 * @cma_registered: the optional CMA driver was successfully registered
 * @ready: initialization completed; published with release/acquire ordering
 * @tx_irq_requested: TX interrupt handler is installed
 * @rx_irq_requested: RX interrupt handler is installed
 * @tx_irq_wake: TX interrupt has a wake reference
 * @rx_irq_wake: RX interrupt has a wake reference
 * @no_demand_paging: RM requires all guest memory to be shared up-front
 */
struct gunyah_rm {
	struct gunyah_resource tx_ghrsc;
	struct gunyah_resource rx_ghrsc;
	struct gunyah_rm_message *active_rx_message;

	struct xarray call_xarray;
	u32 next_seq;

	unsigned char recv_msg[GUNYAH_RM_MSGQ_MSG_SIZE];
	unsigned char send_msg[GUNYAH_RM_MSGQ_MSG_SIZE];
	struct mutex send_lock;
	struct mutex recv_lock;
	bool failed;
	struct completion send_ready;
	struct blocking_notifier_head nh;

	struct auxiliary_device adev;
	struct completion adev_released;
	struct miscdevice miscdev;
	struct fwnode_handle *parent_fwnode;

	struct notifier_block pm_nb;
	bool pm_registered;
	bool cma_registered;
	bool ready;
	bool tx_irq_requested;
	bool rx_irq_requested;
	bool tx_irq_wake;
	bool rx_irq_wake;

	/* RM predates VM_SET_DEMAND_PAGING: share all guest memory up-front */
	bool no_demand_paging;
};

/**
 * gunyah_rm_error_remap() - Remap Gunyah resource manager errors into a Linux error code
 * @rm_error: "Standard" return value from Gunyah resource manager
 */
static inline int gunyah_rm_error_remap(enum gunyah_rm_error rm_error)
{
	switch (rm_error) {
	case GUNYAH_RM_ERROR_OK:
		return 0;
	case GUNYAH_RM_ERROR_UNIMPLEMENTED:
		return -EOPNOTSUPP;
	case GUNYAH_RM_ERROR_NOMEM:
		return -ENOMEM;
	case GUNYAH_RM_ERROR_NORESOURCE:
		return -ENOSPC;
	case GUNYAH_RM_ERROR_DENIED:
		return -EPERM;
	case GUNYAH_RM_ERROR_BUSY:
		return -EBUSY;
	case GUNYAH_RM_ERROR_INVALID:
	case GUNYAH_RM_ERROR_ARGUMENT_INVALID:
	case GUNYAH_RM_ERROR_HANDLE_INVALID:
	case GUNYAH_RM_ERROR_VALIDATE_FAILED:
	case GUNYAH_RM_ERROR_MAP_FAILED:
	case GUNYAH_RM_ERROR_MEM_INVALID:
	case GUNYAH_RM_ERROR_MEM_INUSE:
	case GUNYAH_RM_ERROR_MEM_RELEASED:
	case GUNYAH_RM_ERROR_VMID_INVALID:
	case GUNYAH_RM_ERROR_LOOKUP_FAILED:
	case GUNYAH_RM_ERROR_IRQ_INVALID:
	case GUNYAH_RM_ERROR_IRQ_INUSE:
	case GUNYAH_RM_ERROR_IRQ_RELEASED:
		return -EINVAL;
	default:
		return -EBADMSG;
	}
}

static int gunyah_rm_alloc_irq(struct gunyah_rm *rm, u32 virq)
{
	struct irq_fwspec fwspec;
	int ret;

	fwspec.fwnode = rm->parent_fwnode;
	ret = arch_gunyah_fill_irq_fwspec_params(virq, &fwspec);
	if (ret) {
		pr_err("Failed to translate interrupt: %d\n", ret);
		return ret;
	}

	ret = irq_create_fwspec_mapping(&fwspec);
	if (ret < 0) {
		pr_err("Failed to allocate irq mapping: %d\n", ret);
		return ret;
	}

	return ret;
}


struct gunyah_resource *
gunyah_rm_alloc_resource(struct gunyah_rm *rm,
			 struct gunyah_rm_hyp_resource *hyp_resource)
{
	struct gunyah_resource *ghrsc;
	int ret;

	ghrsc = kzalloc(sizeof(*ghrsc), GFP_KERNEL);
	if (!ghrsc)
		return NULL;

	ghrsc->type = hyp_resource->type;
	ghrsc->capid = le64_to_cpu(hyp_resource->cap_id);
	ghrsc->irq = IRQ_NOTCONNECTED;
	ghrsc->rm_label = le32_to_cpu(hyp_resource->resource_label);
	if (hyp_resource->virq) {
		ret = gunyah_rm_alloc_irq(rm, le32_to_cpu(hyp_resource->virq));
		if (ret < 0) {
			pr_err("Failed to allocate interrupt for resource %d label: %d: %d\n",
				ghrsc->type, ghrsc->rm_label, ret);
			kfree(ghrsc);
			return NULL;
		}
		ghrsc->irq = ret;
	}

	return ghrsc;
}

void gunyah_rm_free_resource(struct gunyah_resource *ghrsc)
{
	irq_dispose_mapping(ghrsc->irq);
	kfree(ghrsc);
}

static int gunyah_rm_init_message_payload(struct gunyah_rm_message *message,
					  const void *msg, size_t hdr_size,
					  size_t msg_size)
{
	const struct gunyah_rm_rpc_hdr *hdr = msg;
	size_t max_buf_size, payload_size;

	if (msg_size < hdr_size)
		return -EINVAL;

	payload_size = msg_size - hdr_size;

	message->num_fragments = FIELD_GET(RM_RPC_FRAGMENTS_MASK, hdr->type);
	message->fragments_received = 0;

	/* There's not going to be any payload, no need to allocate buffer. */
	if (!payload_size && !message->num_fragments)
		return 0;

	if (message->num_fragments > GUNYAH_RM_MAX_NUM_FRAGMENTS)
		return -EINVAL;

	max_buf_size = payload_size +
		       (message->num_fragments * GUNYAH_RM_PAYLOAD_SIZE);

	message->payload = kzalloc(max_buf_size, GFP_KERNEL);
	if (!message->payload)
		return -ENOMEM;

	memcpy(message->payload, msg + hdr_size, payload_size);
	message->size = payload_size;
	return 0;
}

/*
 * Erase the call_xarray entry before waking the waiter: once seq_done is
 * completed, gunyah_rm_call() may return and its on-stack message can go
 * out of scope at any time, so the entry must not be reachable via
 * xa_load() after this point.
 */
static inline void gunyah_rm_complete_reply(struct gunyah_rm *rm,
					    struct gunyah_rm_message *message)
{
	xa_erase(&rm->call_xarray, message->reply.seq);
	complete(&message->reply.seq_done);
}

static void gunyah_rm_abort_message(struct gunyah_rm *rm)
{
	struct gunyah_rm_message *message = rm->active_rx_message;

	rm->active_rx_message = NULL;
	kfree(message->payload);
	message->payload = NULL;

	switch (message->type) {
	case RM_RPC_TYPE_REPLY:
		message->reply.ret = -EIO;
		gunyah_rm_complete_reply(rm, message);
		break;
	case RM_RPC_TYPE_NOTIF:
		fallthrough;
	default:
		kfree(message);
	}
}

bool gunyah_rm_is_failed(struct gunyah_rm *rm)
{
	return READ_ONCE(rm->failed);
}

static void __gunyah_rm_mark_failed(struct gunyah_rm *rm)
{
	lockdep_assert_held(&rm->recv_lock);
	if (rm->failed)
		return;

	WRITE_ONCE(rm->failed, true);
	if (rm->active_rx_message)
		gunyah_rm_abort_message(rm);
	pr_err("RM connection failed; reboot required before creating more VMs\n");
}

void gunyah_rm_mark_failed(struct gunyah_rm *rm)
{
	mutex_lock(&rm->recv_lock);
	__gunyah_rm_mark_failed(rm);
	mutex_unlock(&rm->recv_lock);
}

static bool gunyah_rm_cancel_call(struct gunyah_rm *rm,
				  struct gunyah_rm_message *message,
				  bool fail_transport)
{
	bool pending;

	mutex_lock(&rm->recv_lock);
	pending = !completion_done(&message->reply.seq_done);
	if (pending) {
		xa_erase(&rm->call_xarray, message->reply.seq);
		if (rm->active_rx_message == message)
			rm->active_rx_message = NULL;
		kfree(message->payload);
		message->payload = NULL;
		if (fail_transport)
			__gunyah_rm_mark_failed(rm);
	}
	mutex_unlock(&rm->recv_lock);
	return pending;
}

static bool gunyah_rm_validate_notif(const struct gunyah_rm_message *message)
{
	const struct gunyah_rm_vm_exited_payload *exited = message->payload;

	switch (message->msg_id) {
	case GUNYAH_RM_NOTIFICATION_VM_STATUS:
		return message->size == sizeof(struct gunyah_rm_vm_status_payload);
	case GUNYAH_RM_NOTIFICATION_VM_EXITED:
		if (message->size < sizeof(*exited))
			return false;
		return le32_to_cpu(exited->exit_reason_size) <=
		       message->size - sizeof(*exited);
	default:
		return true;
	}
}

static inline void gunyah_rm_try_complete_message(struct gunyah_rm *rm)
{
	struct gunyah_rm_message *message = rm->active_rx_message;

	if (!message || message->fragments_received != message->num_fragments)
		return;

	rm->active_rx_message = NULL;
	switch (message->type) {
	case RM_RPC_TYPE_REPLY:
		gunyah_rm_complete_reply(rm, message);
		break;
	case RM_RPC_TYPE_NOTIF:
		mutex_unlock(&rm->recv_lock);
		if (gunyah_rm_validate_notif(message))
			blocking_notifier_call_chain(&rm->nh, message->msg_id,
						     message->payload);
		else
			pr_err_ratelimited("Invalid notification %08x payload size %zu\n",
					   message->msg_id, message->size);

		kfree(message->payload);
		kfree(message);
		mutex_lock(&rm->recv_lock);
		break;
	default:
		pr_err_ratelimited("Invalid message type (%u) received\n",
				   message->type);
		rm->active_rx_message = message;
		gunyah_rm_abort_message(rm);
		break;
	}
}

static void gunyah_rm_process_notif(struct gunyah_rm *rm, const void *msg,
				    size_t msg_size)
{
	const struct gunyah_rm_rpc_hdr *hdr = msg;
	struct gunyah_rm_message *message;
	int ret;

	if (rm->active_rx_message) {
		pr_err("Unexpected new notification, still processing an active message\n");
		gunyah_rm_abort_message(rm);
	}

	message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return;

	message->type = RM_RPC_TYPE_NOTIF;
	message->msg_id = le32_to_cpu(hdr->msg_id);

	ret = gunyah_rm_init_message_payload(message, msg, sizeof(*hdr),
					     msg_size);
	if (ret) {
		pr_err("Failed to initialize message for notification: %d\n",
		       ret);
		kfree(message);
		return;
	}

	rm->active_rx_message = message;

	gunyah_rm_try_complete_message(rm);
}

static void gunyah_rm_process_reply(struct gunyah_rm *rm, const void *msg,
				    size_t msg_size)
{
	const struct gunyah_rm_rpc_reply_hdr *reply_hdr = msg;
	struct gunyah_rm_message *message;
	u16 seq_id;
	int ret;

	seq_id = le16_to_cpu(reply_hdr->hdr.seq);
	message = xa_load(&rm->call_xarray, seq_id);

	if (!message) {
		pr_err_ratelimited("Stray reply for unknown sequence %d\n",
				   seq_id);
		return;
	}

	if (message->msg_id != le32_to_cpu(reply_hdr->hdr.msg_id)) {
		pr_err_ratelimited("Reply msg_id mismatch for sequence %d: expected %u, got %u\n",
				   seq_id, message->msg_id,
				   le32_to_cpu(reply_hdr->hdr.msg_id));
		return;
	}

	if (rm->active_rx_message) {
		pr_err("Unexpected new reply, still processing an active message\n");
		if (rm->active_rx_message == message) {
			gunyah_rm_abort_message(rm);
			return;
		}
		gunyah_rm_abort_message(rm);
	}

	ret = gunyah_rm_init_message_payload(message, msg, sizeof(*reply_hdr),
					     msg_size);
	if (ret) {
		pr_err("Failed to initialize reply for sequence %d: %d\n",
		       seq_id, ret);
		/* Send message complete and error the client. */
		message->reply.ret = ret;
		gunyah_rm_complete_reply(rm, message);
		return;
	}

	message->reply.rm_error = le32_to_cpu(reply_hdr->err_code);
	rm->active_rx_message = message;

	gunyah_rm_try_complete_message(rm);
}

static void gunyah_rm_process_cont(struct gunyah_rm *rm,
				   struct gunyah_rm_message *message,
				   const void *msg, size_t msg_size)
{
	const struct gunyah_rm_rpc_hdr *hdr = msg;
	size_t payload_size = msg_size - sizeof(*hdr);

	if (!rm->active_rx_message)
		return;

	/*
	 * hdr->fragments and hdr->msg_id preserves the value from first reply
	 * or notif message. To detect mishandling, check it's still intact.
	 */
	if (message->msg_id != le32_to_cpu(hdr->msg_id) ||
	    message->num_fragments !=
		    FIELD_GET(RM_RPC_FRAGMENTS_MASK, hdr->type) ||
	    (message->type == RM_RPC_TYPE_REPLY &&
	     message->reply.seq != le16_to_cpu(hdr->seq))) {
		gunyah_rm_abort_message(rm);
		return;
	}

	memcpy(message->payload + message->size, msg + sizeof(*hdr),
	       payload_size);
	message->size += payload_size;
	message->fragments_received++;

	gunyah_rm_try_complete_message(rm);
}

static irqreturn_t gunyah_rm_rx(int irq, void *data)
{
	enum gunyah_error gunyah_error;
	struct gunyah_rm_rpc_hdr *hdr;
	struct gunyah_rm *rm = data;
	void *msg = &rm->recv_msg[0];
	size_t len;
	bool ready;

	do {
		gunyah_error = gunyah_hypercall_msgq_recv(rm->rx_ghrsc.capid,
							  msg,
							  sizeof(rm->recv_msg),
							  &len, &ready);
		if (gunyah_error != GUNYAH_ERROR_OK) {
			if (gunyah_error != GUNYAH_ERROR_MSGQUEUE_EMPTY)
				pr_warn("Failed to receive data: %d\n",
					 gunyah_error);
			return IRQ_HANDLED;
		}

		if (len < sizeof(*hdr) || len > sizeof(rm->recv_msg)) {
			pr_err_ratelimited("Invalid RM message size: %zu\n", len);
			continue;
		}

		hdr = msg;
		if (hdr->api != RM_RPC_API) {
			pr_err("Unknown RM RPC API version: %x\n",
				hdr->api);
			return IRQ_HANDLED;
		}

		mutex_lock(&rm->recv_lock);
		if (rm->failed) {
			mutex_unlock(&rm->recv_lock);
			continue;
		}
		switch (FIELD_GET(RM_RPC_TYPE_MASK, hdr->type)) {
		case RM_RPC_TYPE_NOTIF:
			gunyah_rm_process_notif(rm, msg, len);
			break;
		case RM_RPC_TYPE_REPLY:
			gunyah_rm_process_reply(rm, msg, len);
			break;
		case RM_RPC_TYPE_CONTINUATION:
			gunyah_rm_process_cont(rm, rm->active_rx_message, msg,
					       len);
			break;
		default:
			pr_err("Invalid message type (%lu) received\n",
				FIELD_GET(RM_RPC_TYPE_MASK, hdr->type));
			break;
		}
		mutex_unlock(&rm->recv_lock);
	} while (ready);

	return IRQ_HANDLED;
}

static irqreturn_t gunyah_rm_tx(int irq, void *data)
{
	struct gunyah_rm *rm = data;

	complete(&rm->send_ready);

	return IRQ_HANDLED;
}

static unsigned long gunyah_rm_time_left(unsigned long deadline)
{
	unsigned long now = jiffies;

	return time_after(deadline, now) ? deadline - now : 0;
}

static int gunyah_rm_msgq_send(struct gunyah_rm *rm, size_t size, bool push,
			       unsigned long deadline)
{
	const u64 tx_flags = push ? GUNYAH_HYPERCALL_MSGQ_TX_FLAGS_PUSH : 0;
	enum gunyah_error gunyah_error;
	void *data = &rm->send_msg[0];
	unsigned long timeout;
	bool ready;

	lockdep_assert_held(&rm->send_lock);

again:
	timeout = gunyah_rm_time_left(deadline);
	if (!timeout || !wait_for_completion_timeout(&rm->send_ready, timeout))
		return -ETIMEDOUT;
	gunyah_error = gunyah_hypercall_msgq_send(rm->tx_ghrsc.capid, size,
						  data, tx_flags, &ready);

	/* Should never happen because Linux properly tracks the ready-state of the msgq */
	if (WARN_ON(gunyah_error == GUNYAH_ERROR_MSGQUEUE_FULL))
		goto again;
	if (gunyah_error != GUNYAH_ERROR_OK)
		return gunyah_error_remap(gunyah_error);

	if (ready)
		complete(&rm->send_ready);

	return 0;
}

static int gunyah_rm_send_request(struct gunyah_rm *rm, u32 message_id,
				  const void *req_buf, size_t req_buf_size,
				  struct gunyah_rm_message *message,
				  unsigned long deadline)
{
	size_t buf_size_remaining = req_buf_size;
	const void *req_buf_curr = req_buf;
	struct gunyah_rm_rpc_hdr *hdr =
		(struct gunyah_rm_rpc_hdr *)&rm->send_msg[0];
	struct gunyah_rm_rpc_hdr hdr_template;
	void *payload = hdr + 1;
	u32 cont_fragments = 0;
	size_t payload_size;
	bool push;
	int ret;

	if (req_buf_size >
	    GUNYAH_RM_MAX_NUM_FRAGMENTS * GUNYAH_RM_PAYLOAD_SIZE) {
		pr_warn("Limit (%lu bytes) exceeded for the maximum message size: %lu\n",
			GUNYAH_RM_MAX_NUM_FRAGMENTS * GUNYAH_RM_PAYLOAD_SIZE,
			req_buf_size);
		dump_stack();
		return -E2BIG;
	}

	if (req_buf_size)
		cont_fragments = (req_buf_size - 1) / GUNYAH_RM_PAYLOAD_SIZE;

	hdr_template.api = RM_RPC_API;
	hdr_template.type = FIELD_PREP(RM_RPC_TYPE_MASK, RM_RPC_TYPE_REQUEST) |
			    FIELD_PREP(RM_RPC_FRAGMENTS_MASK, cont_fragments);
	hdr_template.seq = cpu_to_le16(message->reply.seq);
	hdr_template.msg_id = cpu_to_le32(message_id);

	do {
		*hdr = hdr_template;

		/* Copy payload */
		payload_size = min(buf_size_remaining, GUNYAH_RM_PAYLOAD_SIZE);
		memcpy(payload, req_buf_curr, payload_size);
		req_buf_curr += payload_size;
		buf_size_remaining -= payload_size;

		/* Only the last message should have push flag set */
		push = !buf_size_remaining;
		ret = gunyah_rm_msgq_send(rm, sizeof(*hdr) + payload_size,
					  push, deadline);
		if (ret)
			break;

		hdr_template.type =
			FIELD_PREP(RM_RPC_TYPE_MASK, RM_RPC_TYPE_CONTINUATION) |
			FIELD_PREP(RM_RPC_FRAGMENTS_MASK, cont_fragments);
	} while (buf_size_remaining);

	return ret;
}

/**
 * gunyah_rm_call: Achieve request-response type communication with RPC
 * @rm: Pointer to Gunyah resource manager internal data
 * @message_id: The RM RPC message-id
 * @req_buf: Request buffer that contains the payload
 * @req_buf_size: Total size of the payload
 * @resp_buf: Pointer to a response buffer
 * @resp_buf_size: Size of the response buffer
 *
 * Make a request to the Resource Manager and wait for reply back. For a successful
 * response, the function returns the payload. The size of the payload is set in
 * resp_buf_size. The resp_buf must be freed by the caller when 0 is returned
 * and resp_buf_size != 0.
 * If both response pointers are NULL, a successful reply must have no payload.
 *
 * req_buf should be not NULL for req_buf_size >0. If req_buf_size == 0,
 * req_buf *can* be NULL and no additional payload is sent.
 *
 * Context: Process context. Will sleep waiting for reply.
 * Return: 0 on success. <0 if error.
 */
int gunyah_rm_call(struct gunyah_rm *rm, u32 message_id, const void *req_buf,
		   size_t req_buf_size, void **resp_buf, size_t *resp_buf_size)
{
	struct gunyah_rm_message message = { 0 };
	unsigned long deadline;
	u32 seq_id;
	int ret;

	/* message_id 0 is reserved. req_buf_size implies req_buf is not NULL */
	if (!rm || !message_id || (!req_buf && req_buf_size))
		return -EINVAL;
	if (resp_buf)
		*resp_buf = NULL;
	if (resp_buf_size)
		*resp_buf_size = 0;

	message.type = RM_RPC_TYPE_REPLY;
	message.msg_id = message_id;

	message.reply.seq_done =
		COMPLETION_INITIALIZER_ONSTACK(message.reply.seq_done);

	mutex_lock(&rm->send_lock);
	if (gunyah_rm_is_failed(rm)) {
		ret = -EIO;
		goto out;
	}
	deadline = jiffies + msecs_to_jiffies(GUNYAH_RM_RPC_TIMEOUT_MS);

	/* Allocate a new seq number for this message */
	mutex_lock(&rm->recv_lock);
	ret = xa_alloc_cyclic(&rm->call_xarray, &seq_id, &message, xa_limit_16b,
			      &rm->next_seq, GFP_KERNEL);
	if (ret >= 0)
		message.reply.seq = lower_16_bits(seq_id);
	mutex_unlock(&rm->recv_lock);
	if (ret < 0)
		goto out;

	/* Send the request to the Resource Manager */
	ret = gunyah_rm_send_request(rm, message_id, req_buf, req_buf_size,
				     &message, deadline);
	if (ret < 0) {
		pr_warn("Failed to send request. Error: %d\n", ret);
		if (gunyah_rm_cancel_call(rm, &message, ret != -E2BIG))
			goto out;
	} else if (!wait_for_completion_timeout(&message.reply.seq_done,
					      gunyah_rm_time_left(deadline)) &&
		   gunyah_rm_cancel_call(rm, &message, true)) {
		pr_err("RM request %08x sequence %u timed out\n", message_id,
		       message.reply.seq);
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Check for internal (kernel) error waiting for the response */
	ret = 0;
	if (message.reply.ret) {
		gunyah_rm_mark_failed(rm);
		ret = message.reply.ret;
		goto out;
	}

	/* Got a response, did resource manager give us an error? */
	if (message.reply.rm_error != GUNYAH_RM_ERROR_OK) {
		pr_warn("RM rejected message %08x. Error: %d\n",
			 message_id, message.reply.rm_error);
		ret = gunyah_rm_error_remap(message.reply.rm_error);
		if (ret == -EBADMSG)
			gunyah_rm_mark_failed(rm);
		kfree(message.payload);
		goto out;
	}

	if (!resp_buf && !resp_buf_size && message.size) {
		gunyah_rm_mark_failed(rm);
		kfree(message.payload);
		ret = -EBADMSG;
		goto out;
	}

	/* Everything looks good, return the payload */
	if (resp_buf_size)
		*resp_buf_size = message.size;

	if (message.size && resp_buf) {
		*resp_buf = message.payload;
	} else {
		/* kfree in case RM sent us multiple fragments but never any data in
		 * those fragments. We would've allocated memory for it, but message.size == 0
		 */
		kfree(message.payload);
	}

out:
	mutex_unlock(&rm->send_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gunyah_rm_call);

int gunyah_rm_notifier_register(struct gunyah_rm *rm, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&rm->nh, nb);
}
EXPORT_SYMBOL_GPL(gunyah_rm_notifier_register);

int gunyah_rm_notifier_unregister(struct gunyah_rm *rm,
				  struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&rm->nh, nb);
}
EXPORT_SYMBOL_GPL(gunyah_rm_notifier_unregister);

struct device *gunyah_rm_get(struct gunyah_rm *rm)
{
	return get_device(rm->miscdev.this_device);
}
EXPORT_SYMBOL_GPL(gunyah_rm_get);

void gunyah_rm_put(struct gunyah_rm *rm)
{
	put_device(rm->miscdev.this_device);
}
EXPORT_SYMBOL_GPL(gunyah_rm_put);

static int gunyah_dev_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct gunyah_rm *rm = container_of(miscdev, struct gunyah_rm, miscdev);

	/* misc_register() precedes auxiliary device setup, which can fail. */
	return smp_load_acquire(&rm->ready) ? 0 : -ENODEV;
}

static long gunyah_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct miscdevice *miscdev = filp->private_data;
	struct gunyah_rm *rm = container_of(miscdev, struct gunyah_rm, miscdev);

	return gunyah_dev_vm_mgr_ioctl(rm, cmd, arg);
}

static const struct file_operations gunyah_dev_fops = {
	/* clang-format off */
	.owner		= THIS_MODULE,
	.open		= gunyah_dev_open,
	.unlocked_ioctl	= gunyah_dev_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.llseek		= noop_llseek,
	/* clang-format on */
};

static int gunyah_rm_probe_info_area(struct gunyah_rm *rm)
{
	struct gunyah_rm_info *info;
	size_t info_size;
	int ret;

	info = gunyah_get_info(GUNYAH_INFO_OWNER_RM, 0, &info_size);
	if (IS_ERR_OR_NULL(info))
		return -ENOENT;
	if (info_size != sizeof(*info))
		return -EINVAL;

	rm->tx_ghrsc.type = GUNYAH_RESOURCE_TYPE_MSGQ_TX;
	rm->tx_ghrsc.capid = le64_to_cpu(info->tx_msgq_cap);
	ret = gunyah_rm_alloc_irq(rm, le32_to_cpu(info->tx_msgq_irq));
	if (ret <= 0)
		return -EINVAL;
	rm->tx_ghrsc.irq = ret;

	rm->rx_ghrsc.type = GUNYAH_RESOURCE_TYPE_MSGQ_RX;
	rm->rx_ghrsc.capid = le64_to_cpu(info->rx_msgq_cap);
	ret = gunyah_rm_alloc_irq(rm, le32_to_cpu(info->rx_msgq_irq));
	if (ret <= 0)
		return -EINVAL;
	rm->rx_ghrsc.irq = ret;

	return 0;
}

/*
 * Linux is normally started by the resource manager, which overlays the
 * /hypervisor subtree (including the rm-rpc capabilities it just allocated)
 * onto the DTB handed to the VM.  When XBL starts Linux directly that overlay
 * is instead merged into XBL's own DT log buffer, so the DTB we booted with
 * only carries a placeholder.  The capability IDs are allocated fresh on every
 * boot, so they cannot be written into the board DTS.  Recover them from the
 * merged tree when it is present.
 */
static int gunyah_rm_info_from_xbl(u64 *tx_capid, u64 *rx_capid,
				   u32 *tx_virq, u32 *rx_virq)
{
	struct device_node *np __free(device_node) = NULL;
	const void *fdt = NULL;
	u64 reg_addr, reg_size;
	unsigned long size, i;
	u8 *buf;
	int off, rm_off, len, ret = -ENOENT;
	const __be32 *reg, *irq;

	np = of_find_node_by_name(NULL, "xbl-dt-log-merged");
	if (!np)
		return -ENOENT;
	if (of_property_read_reg(np, 0, &reg_addr, &reg_size) || !reg_size)
		return -EINVAL;

	/*
	 * Only the first part of the region is guaranteed to be mapped; the
	 * merged tree we need sits near its start.  Stay well inside that and
	 * copy a page at a time so an unmapped tail cannot fault the boot.
	 */
	size = min_t(u64, reg_size, SZ_128K);
	buf = kvmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < size; i += PAGE_SIZE) {
		size_t n = min_t(size_t, PAGE_SIZE, size - i);
		void *p = memremap(reg_addr + i, n, MEMREMAP_WB);

		if (!p) {
			ret = -EFAULT;
			goto out;
		}
		memcpy(buf + i, p, n);
		memunmap(p);
	}

	for (i = 0; i + sizeof(struct fdt_header) <= size; i += 4) {
		fdt = buf + i;
		if (fdt_magic(fdt) != FDT_MAGIC || fdt_check_header(fdt) ||
		    fdt_totalsize(fdt) > size - i)
			continue;

		off = fdt_path_offset(fdt, "/hypervisor");
		if (off < 0)
			continue;

		for (rm_off = fdt_first_subnode(fdt, off); rm_off >= 0;
		     rm_off = fdt_next_subnode(fdt, rm_off)) {
			const char *compat = fdt_getprop(fdt, rm_off, "compatible", &len);

			if (!compat ||
			    !fdt_stringlist_contains(compat, len, "gunyah-resource-manager"))
				continue;

			reg = fdt_getprop(fdt, rm_off, "reg", &len);
			if (!reg || len != 4 * sizeof(__be32))
				continue;

			irq = fdt_getprop(fdt, rm_off, "interrupts", &len);
			if (!irq || len != 6 * sizeof(__be32))
				continue;

			if (fdt32_to_cpu(irq[0]) != 0 || fdt32_to_cpu(irq[3]) != 0 ||
			    fdt32_to_cpu(irq[1]) > U32_MAX - 32 ||
			    fdt32_to_cpu(irq[4]) > U32_MAX - 32)
				continue;

			*tx_capid = ((u64)fdt32_to_cpu(reg[0]) << 32) | fdt32_to_cpu(reg[1]);
			*rx_capid = ((u64)fdt32_to_cpu(reg[2]) << 32) | fdt32_to_cpu(reg[3]);
			*tx_virq = fdt32_to_cpu(irq[1]) + 32;
			*rx_virq = fdt32_to_cpu(irq[4]) + 32;
			ret = 0;
			goto out;
		}
	}

out:
	kvfree(buf);
	return ret;
}

static int gunyah_rm_get_of_info(struct gunyah_rm *rm)
{
	struct device_node *gunyah_np __free(device_node) = NULL;
	struct device_node *rm_np __free(device_node) = NULL;
	int ret;

	if (!arch_is_gunyah_guest())
		return -ENODEV;

	gunyah_np = of_find_node_by_path("/hypervisor");
	if (!gunyah_np)
		return -ENODEV;

	rm_np = of_get_compatible_child(gunyah_np, "gunyah-resource-manager");
	if (!rm_np)
		return -ENODEV;

	rm->tx_ghrsc.type = GUNYAH_RESOURCE_TYPE_MSGQ_TX;
	ret = of_property_read_u64_index(rm_np, "reg", 0, &rm->tx_ghrsc.capid);
	if (ret)
		return -EINVAL;

	ret = of_irq_get(rm_np, 0);
	if (ret <= 0)
		return -EINVAL;
	rm->tx_ghrsc.irq = ret;

	rm->rx_ghrsc.type = GUNYAH_RESOURCE_TYPE_MSGQ_RX;
	ret = of_property_read_u64_index(rm_np, "reg", 1, &rm->rx_ghrsc.capid);
	if (ret)
		return -EINVAL;

	ret = of_irq_get(rm_np, 1);
	if (ret <= 0)
		return -EINVAL;
	rm->rx_ghrsc.irq = ret;

	return 0;
}

/**
 * gunyah_rm_get_xbl_info() - recover rm-rpc resources from XBL's merged DT
 *
 * The resource manager publishes the rm-rpc capabilities by overlaying its
 * DT onto the one it hands to the VM.  When XBL boots Linux directly that
 * overlay lands in XBL's own DT log buffer instead, and the capability IDs
 * are reallocated on every boot.  Read them (and the virtual IRQs) back out
 * of the merged tree.
 */
static int gunyah_rm_get_xbl_info(struct gunyah_rm *rm)
{
	u32 tx_virq, rx_virq;
	int ret;

	ret = gunyah_rm_info_from_xbl(&rm->tx_ghrsc.capid, &rm->rx_ghrsc.capid,
				      &tx_virq, &rx_virq);
	if (ret)
		return ret;

	rm->tx_ghrsc.type = GUNYAH_RESOURCE_TYPE_MSGQ_TX;
	ret = gunyah_rm_alloc_irq(rm, tx_virq);
	if (ret <= 0)
		return -EINVAL;
	rm->tx_ghrsc.irq = ret;

	rm->rx_ghrsc.type = GUNYAH_RESOURCE_TYPE_MSGQ_RX;
	ret = gunyah_rm_alloc_irq(rm, rx_virq);
	if (ret <= 0)
		return -EINVAL;
	rm->rx_ghrsc.irq = ret;

	pr_info("Using rm-rpc resources from XBL merged DT\n");
	return 0;
}

static void gunyah_adev_release(struct device *dev)
{
	struct gunyah_rm *rm = container_of(dev, struct gunyah_rm, adev.dev);

	complete(&rm->adev_released);
}

static int gunyah_adev_init(struct gunyah_rm *rm, const char *name)
{
	struct auxiliary_device *adev = &rm->adev;
	int ret = 0;

	adev->name = name;
	adev->dev.platform_data = rm;
	adev->dev.parent = rm->miscdev.this_device;
	adev->dev.release = gunyah_adev_release;
	init_completion(&rm->adev_released);
	ret = auxiliary_device_init(adev);
	if (ret)
		return ret;

	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		wait_for_completion(&rm->adev_released);
		return ret;
	}

	return ret;
}

static void gunyah_adev_uninit(struct gunyah_rm *rm)
{
	auxiliary_device_delete(&rm->adev);
	auxiliary_device_uninit(&rm->adev);
	wait_for_completion(&rm->adev_released);
}

static void gunyah_rm_free_irqs(struct gunyah_rm *rm)
{
	if (rm->rx_irq_wake) {
		disable_irq_wake(rm->rx_ghrsc.irq);
		rm->rx_irq_wake = false;
	}
	if (rm->rx_irq_requested) {
		free_irq(rm->rx_ghrsc.irq, rm);
		rm->rx_irq_requested = false;
	}
	if (rm->tx_irq_wake) {
		disable_irq_wake(rm->tx_ghrsc.irq);
		rm->tx_irq_wake = false;
	}
	if (rm->tx_irq_requested) {
		free_irq(rm->tx_ghrsc.irq, rm);
		rm->tx_irq_requested = false;
	}
	if (rm->rx_ghrsc.irq > 0) {
		irq_dispose_mapping(rm->rx_ghrsc.irq);
		rm->rx_ghrsc.irq = 0;
	}
	if (rm->tx_ghrsc.irq > 0) {
		irq_dispose_mapping(rm->tx_ghrsc.irq);
		rm->tx_ghrsc.irq = 0;
	}
}

static int gunyah_rm_request_irqs(struct gunyah_rm *rm)
{
	int ret;

	ret = request_threaded_irq(rm->tx_ghrsc.irq, NULL, gunyah_rm_tx,
				   IRQF_ONESHOT, "gunyah_rm_tx", rm);
	if (ret)
		return ret;
	rm->tx_irq_requested = true;
	rm->tx_irq_wake = !enable_irq_wake(rm->tx_ghrsc.irq);
	/* Assume RM is ready to receive messages from us. */
	complete(&rm->send_ready);

	ret = request_threaded_irq(rm->rx_ghrsc.irq, NULL, gunyah_rm_rx,
				   IRQF_ONESHOT, "gunyah_rm_rx", rm);
	if (ret)
		return ret;
	rm->rx_irq_requested = true;
	rm->rx_irq_wake = !enable_irq_wake(rm->rx_ghrsc.irq);
	return 0;
}

static int gunyah_rm_hibernation_restore(struct notifier_block *nb,
					 unsigned long action, void *data)
{
	struct gunyah_rm *rm;
	int ret;

	rm = container_of(nb, struct gunyah_rm, pm_nb);

	switch (action) {
	case PM_HIBERNATION_PREPARE:
		/* Cleanup stale data from pre-hibernation */
		gunyah_rm_free_irqs(rm);
		gunyah_unmap_addrspace_info_area();
		break;
	case PM_POST_HIBERNATION:
		/* Fetch the latest RM info from addrspace info area */
		ret = gunyah_map_addrspace_info_area();
		if (ret) {
			pr_err("Failed to map addrspace info area\n");
			return NOTIFY_BAD;
		}
		ret = gunyah_rm_probe_info_area(rm);
		if (ret) {
			pr_err("Failed to get RM info\n");
			goto err;
		}

		ret = gunyah_rm_request_irqs(rm);
		if (ret)
			goto err;
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
err:
	gunyah_rm_free_irqs(rm);
	gunyah_unmap_addrspace_info_area();
	return NOTIFY_BAD;
}

/**
 * gunyah_rm_has_demand_paging() - can this RM fault guest pages in on demand?
 * @rm: resource manager handle
 */
bool gunyah_rm_has_demand_paging(struct gunyah_rm *rm)
{
	return !rm->no_demand_paging;
}
EXPORT_SYMBOL_GPL(gunyah_rm_has_demand_paging);

/**
 * gunyah_rm_probe_features() - detect optional RM capabilities
 * @rm: resource manager handle
 *
 * RMs older than VM_SET_DEMAND_PAGING cannot fault guest pages in on
 * demand, so the VM manager must have all guest memory shared before the
 * VM starts.  Probe once at init: the query is cheap and harmless.
 */
static void gunyah_rm_probe_features(struct gunyah_rm *rm)
{
	int ret;

	/* Zero ranges: this cannot change any VM state on any RM version. */
	ret = gunyah_rm_vm_set_demand_paging(rm, 0, 0, NULL);
	rm->no_demand_paging = (ret == -EOPNOTSUPP);
	if (rm->no_demand_paging)
		pr_info("RM has no VM_SET_DEMAND_PAGING; guest memory will be shared up-front\n");
}

static struct gunyah_rm *__rm;

static int __init gunyah_rm_init(void)
{
	struct device_node *parent_irq_node __free(device_node) = NULL;
	struct gunyah_rm *rm __free(kfree) = NULL;
	int ret;

	rm = kzalloc(sizeof(*rm), GFP_KERNEL);
	if (!rm)
		return -ENOMEM;

	parent_irq_node = of_irq_find_parent(of_root);
	if (!parent_irq_node) {
		pr_err("Failed to find interrupt parent of resource manager\n");
		return -ENODEV;
	}

	rm->parent_fwnode = fwnode_handle_get(of_fwnode_handle(parent_irq_node));
	if (!rm->parent_fwnode) {
		pr_err("Failed to find interrupt parent domain of resource manager\n");
		return -ENODEV;
	}

	mutex_init(&rm->send_lock);
	mutex_init(&rm->recv_lock);
	init_completion(&rm->send_ready);
	BLOCKING_INIT_NOTIFIER_HEAD(&rm->nh);
	xa_init_flags(&rm->call_xarray, XA_FLAGS_ALLOC);

	ret = gunyah_rm_probe_info_area(rm);
	if (ret == -ENOENT) {
		ret = gunyah_rm_get_of_info(rm);
		if (ret) {
			gunyah_rm_free_irqs(rm);
			ret = gunyah_rm_get_xbl_info(rm);
		}
	} else if (!ret) {
		rm->pm_nb.notifier_call = gunyah_rm_hibernation_restore;
	}
	if (ret)
		goto free_irqs;

	ret = gunyah_rm_request_irqs(rm);
	if (ret)
		goto free_irqs;

	gunyah_rm_probe_features(rm);

	rm->miscdev.name = "gunyah";
	rm->miscdev.minor = MISC_DYNAMIC_MINOR;
	rm->miscdev.fops = &gunyah_dev_fops;

	ret = misc_register(&rm->miscdev);
	if (ret) {
		pr_err("Failed to register gunyah misc device\n");
		goto free_irqs;
	}

	ret = gunyah_adev_init(rm, "gh_rm_core");
	if (ret) {
		pr_err("Failed to add gh_rm_core device\n");
		goto deregister_misc;
	}

	ret = gunyah_cma_mem_init();
	if (ret)
		pr_err("Failed to register gunyah CMA device\n");
	else
		rm->cma_registered = true;

	if (rm->pm_nb.notifier_call) {
		ret = register_pm_notifier(&rm->pm_nb);
		if (ret) {
			pr_err("Failed to register pm notifier\n");
			goto uninit_adev;
		}
		rm->pm_registered = true;
	}

	__rm = no_free_ptr(rm);
	/* No fallible setup may follow publication to userspace. */
	smp_store_release(&__rm->ready, true);
	return 0;
uninit_adev:
	if (rm->cma_registered)
		gunyah_cma_mem_exit();
	gunyah_adev_uninit(rm);
deregister_misc:
	misc_deregister(&rm->miscdev);
free_irqs:
	gunyah_rm_free_irqs(rm);
	if (rm->active_rx_message)
		gunyah_rm_abort_message(rm);
	xa_destroy(&rm->call_xarray);
	fwnode_handle_put(rm->parent_fwnode);
	return ret;
}
module_init(gunyah_rm_init);

static void __exit gunyah_rm_exit(void)
{
	struct gunyah_rm *rm = __rm;

	__rm = NULL;

	if (!rm)
		return;

	if (rm->pm_registered)
		unregister_pm_notifier(&rm->pm_nb);
	if (rm->cma_registered)
		gunyah_cma_mem_exit();
	gunyah_adev_uninit(rm);
	misc_deregister(&rm->miscdev);
	gunyah_rm_free_irqs(rm);
	if (rm->active_rx_message)
		gunyah_rm_abort_message(rm);
	xa_destroy(&rm->call_xarray);
	fwnode_handle_put(rm->parent_fwnode);
	kfree(rm);
}
module_exit(gunyah_rm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Gunyah Resource Manager Driver");
