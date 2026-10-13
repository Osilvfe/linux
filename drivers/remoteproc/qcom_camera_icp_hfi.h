/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#ifndef __QCOM_CAMERA_ICP_HFI_H__
#define __QCOM_CAMERA_ICP_HFI_H__

#include <linux/spinlock.h>
#include <linux/sizes.h>
#include <linux/types.h>

#define QCOM_CAMERA_ICP_HFI_QUEUE_COUNT		3
#define QCOM_CAMERA_ICP_HFI_CMD_QUEUE_SIZE	SZ_8K
#define QCOM_CAMERA_ICP_HFI_MSG_QUEUE_SIZE	SZ_8K
#define QCOM_CAMERA_ICP_HFI_DBG_QUEUE_SIZE	(100 * SZ_1K)
#define QCOM_CAMERA_ICP_HFI_SFR_SIZE		SZ_4K
#define QCOM_CAMERA_ICP_HFI_SECONDARY_HEAP_SIZE	SZ_1M

enum qcom_camera_icp_hfi_queue_id {
	QCOM_CAMERA_ICP_HFI_CMD_QUEUE,
	QCOM_CAMERA_ICP_HFI_MSG_QUEUE,
	QCOM_CAMERA_ICP_HFI_DBG_QUEUE,
};

/* Every live field occupies its own 64-byte firmware cache line. */
struct qcom_camera_icp_hfi_queue_header {
	__le32 reserved0[15];
	__le32 status;
	__le32 reserved1[15];
	__le32 start_addr;
	__le32 reserved2[15];
	__le32 type;
	__le32 reserved3[15];
	__le32 size;
	__le32 reserved4[15];
	__le32 packet_size;
	__le32 reserved5[15];
	__le32 packet_drop_count;
	__le32 reserved6[15];
	__le32 rx_watermark;
	__le32 reserved7[15];
	__le32 tx_watermark;
	__le32 reserved8[15];
	__le32 rx_request;
	__le32 reserved9[15];
	__le32 tx_request;
	__le32 reserved10[15];
	__le32 rx_irq_status;
	__le32 reserved11[15];
	__le32 tx_irq_status;
	__le32 reserved12[15];
	__le32 read_index;
	__le32 reserved13[15];
	__le32 write_index;
	__le32 reserved14[15];
};

struct qcom_camera_icp_hfi_table_header {
	__le32 version;
	__le32 size;
	__le32 first_queue_offset;
	__le32 queue_header_size;
	__le32 queue_count;
	__le32 active_queue_count;
};

struct qcom_camera_icp_hfi_table {
	struct qcom_camera_icp_hfi_table_header header;
	struct qcom_camera_icp_hfi_queue_header queues[4];
};

struct qcom_camera_icp_hfi_queue {
	struct qcom_camera_icp_hfi_queue_header *header;
	__le32 *data;
	u32 word_count;
	/* Serializes updates to the shared ring indices and contents. */
	spinlock_t lock;
};

struct qcom_camera_icp_hfi {
	struct qcom_camera_icp_hfi_table *table;
	struct qcom_camera_icp_hfi_queue queues[QCOM_CAMERA_ICP_HFI_QUEUE_COUNT];
};

struct qcom_camera_icp_hfi_memory {
	struct qcom_camera_icp_hfi_table *table;
	dma_addr_t table_iova;
	void *queue[QCOM_CAMERA_ICP_HFI_QUEUE_COUNT];
	dma_addr_t queue_iova[QCOM_CAMERA_ICP_HFI_QUEUE_COUNT];
};

int qcom_camera_icp_hfi_init(struct qcom_camera_icp_hfi *hfi,
			     const struct qcom_camera_icp_hfi_memory *memory);
int qcom_camera_icp_hfi_write(struct qcom_camera_icp_hfi *hfi,
			      const void *packet, size_t packet_size);
int qcom_camera_icp_hfi_read(struct qcom_camera_icp_hfi *hfi,
			     enum qcom_camera_icp_hfi_queue_id id,
			     void *packet, size_t capacity, size_t *packet_size);

#endif
