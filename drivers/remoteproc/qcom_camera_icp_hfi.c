// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Camera Image Control Processor HFI transport
 *
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#include <linux/build_bug.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/string.h>

#include "qcom_camera_icp_hfi.h"

#define QCOM_CAMERA_ICP_HFI_TABLE_VERSION	U32_MAX
#define QCOM_CAMERA_ICP_HFI_QUEUE_ACTIVE	1
#define QCOM_CAMERA_ICP_HFI_VARIABLE_PACKET	0
#define QCOM_CAMERA_ICP_HFI_WATERMARK		1
#define QCOM_CAMERA_ICP_HFI_DEBUG_WATERMARK	1024

static const size_t qcom_camera_icp_hfi_queue_sizes[] = {
	[QCOM_CAMERA_ICP_HFI_CMD_QUEUE] = QCOM_CAMERA_ICP_HFI_CMD_QUEUE_SIZE,
	[QCOM_CAMERA_ICP_HFI_MSG_QUEUE] = QCOM_CAMERA_ICP_HFI_MSG_QUEUE_SIZE,
	[QCOM_CAMERA_ICP_HFI_DBG_QUEUE] = QCOM_CAMERA_ICP_HFI_DBG_QUEUE_SIZE,
};

static size_t hfi_allocation_size(size_t payload_size)
{
	return ALIGN(payload_size, PAGE_SIZE) + PAGE_SIZE;
}

int qcom_camera_icp_hfi_region_init(struct qcom_camera_icp_hfi *hfi,
				    struct qcom_camera_icp_hfi_region *region)
{
	size_t allocation_size;
	size_t heap_size = QCOM_CAMERA_ICP_HFI_SECONDARY_HEAP_SIZE;
	size_t offset = 0;
	int id;

	if (!region || !region->base ||
	    region->size < QCOM_CAMERA_ICP_HFI_REGION_SIZE ||
	    region->iova > U32_MAX - QCOM_CAMERA_ICP_HFI_REGION_SIZE)
		return -EINVAL;

	region->secondary_heap = region->base;
	region->secondary_heap_iova = region->iova;
	allocation_size = hfi_allocation_size(heap_size);
	region->secondary_heap_size = allocation_size;
	offset += region->secondary_heap_size;
	region->memory.table = (struct qcom_camera_icp_hfi_table *)
		((u8 *)region->base + offset);
	region->memory.table_iova = region->iova + offset;
	allocation_size = hfi_allocation_size(sizeof(*hfi->table));
	offset += allocation_size;

	for (id = 0; id < QCOM_CAMERA_ICP_HFI_QUEUE_COUNT; id++) {
		size_t queue_size = qcom_camera_icp_hfi_queue_sizes[id];

		region->memory.queue[id] = (u8 *)region->base + offset;
		region->memory.queue_iova[id] = region->iova + offset;
		allocation_size = hfi_allocation_size(queue_size);
		offset += allocation_size;
	}

	region->sfr = (u8 *)region->base + offset;
	region->sfr_iova = region->iova + offset;
	allocation_size = hfi_allocation_size(QCOM_CAMERA_ICP_HFI_SFR_SIZE);
	offset += allocation_size;
	if (offset > region->size)
		return -E2BIG;

	*(__le32 *)region->sfr = cpu_to_le32(QCOM_CAMERA_ICP_HFI_SFR_SIZE);
	return qcom_camera_icp_hfi_init(hfi, &region->memory);
}

static void hfi_copy_to_queue(__le32 *queue, u32 queue_words, u32 index,
			      const void *source, u32 words)
{
	u32 first_words = min(words, queue_words - index);

	memcpy(queue + index, source, first_words * sizeof(*queue));
	if (first_words != words)
		memcpy(queue, (const u8 *)source + first_words * sizeof(*queue),
		       (words - first_words) * sizeof(*queue));
}

static void hfi_copy_from_queue(void *destination, const __le32 *queue,
				u32 queue_words, u32 index, u32 words)
{
	u32 first_words = min(words, queue_words - index);

	memcpy(destination, queue + index, first_words * sizeof(*queue));
	if (first_words != words)
		memcpy((u8 *)destination + first_words * sizeof(*queue), queue,
		       (words - first_words) * sizeof(*queue));
}

static int qcom_camera_icp_hfi_init_queue(struct qcom_camera_icp_hfi *hfi,
					  const struct qcom_camera_icp_hfi_memory *memory,
					  enum qcom_camera_icp_hfi_queue_id id)
{
	struct qcom_camera_icp_hfi_queue_header *header;
	struct qcom_camera_icp_hfi_queue *queue;
	size_t size = qcom_camera_icp_hfi_queue_sizes[id];
	dma_addr_t iova = memory->queue_iova[id];

	if (!memory->queue[id] || !IS_ALIGNED(iova, sizeof(u32)) ||
	    iova > U32_MAX || size / sizeof(u32) > U32_MAX)
		return -EINVAL;

	header = &hfi->table->queues[id];
	queue = &hfi->queues[id];
	queue->header = header;
	queue->data = memory->queue[id];
	queue->word_count = size / sizeof(u32);
	spin_lock_init(&queue->lock);

	header->status = cpu_to_le32(QCOM_CAMERA_ICP_HFI_QUEUE_ACTIVE);
	header->start_addr = cpu_to_le32(iova);
	header->type = cpu_to_le32(id);
	header->size = cpu_to_le32(queue->word_count);
	header->packet_size = cpu_to_le32(QCOM_CAMERA_ICP_HFI_VARIABLE_PACKET);
	header->rx_watermark = cpu_to_le32(QCOM_CAMERA_ICP_HFI_WATERMARK);
	header->tx_watermark = cpu_to_le32(id == QCOM_CAMERA_ICP_HFI_DBG_QUEUE ?
					 QCOM_CAMERA_ICP_HFI_DEBUG_WATERMARK :
					 QCOM_CAMERA_ICP_HFI_WATERMARK);
	header->rx_request = cpu_to_le32(id == QCOM_CAMERA_ICP_HFI_DBG_QUEUE ? 0 : 1);

	return 0;
}

int qcom_camera_icp_hfi_init(struct qcom_camera_icp_hfi *hfi,
			     const struct qcom_camera_icp_hfi_memory *memory)
{
	struct qcom_camera_icp_hfi_table_header *header;
	int ret;
	int id;

	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_queue_header) != 956);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_table) != 3848);

	if (!hfi || !memory || !memory->table || memory->table_iova > U32_MAX ||
	    !IS_ALIGNED(memory->table_iova, sizeof(u32)))
		return -EINVAL;

	memset(hfi, 0, sizeof(*hfi));
	memset(memory->table, 0, sizeof(*memory->table));
	hfi->table = memory->table;
	header = &hfi->table->header;
	header->version = cpu_to_le32(QCOM_CAMERA_ICP_HFI_TABLE_VERSION);
	header->size = cpu_to_le32(sizeof(*hfi->table));
	header->first_queue_offset = cpu_to_le32(offsetof(struct qcom_camera_icp_hfi_table,
							  queues));
	header->queue_header_size =
		cpu_to_le32(sizeof(struct qcom_camera_icp_hfi_queue_header));
	header->queue_count = cpu_to_le32(QCOM_CAMERA_ICP_HFI_QUEUE_COUNT);
	header->active_queue_count = cpu_to_le32(QCOM_CAMERA_ICP_HFI_QUEUE_COUNT);

	for (id = 0; id < QCOM_CAMERA_ICP_HFI_QUEUE_COUNT; id++) {
		ret = qcom_camera_icp_hfi_init_queue(hfi, memory, id);
		if (ret)
			return ret;
	}

	dma_wmb();
	return 0;
}

int qcom_camera_icp_hfi_write(struct qcom_camera_icp_hfi *hfi,
			      const void *packet, size_t packet_size)
{
	struct qcom_camera_icp_hfi_queue *queue;
	struct qcom_camera_icp_hfi_queue_header *header;
	unsigned long flags;
	u32 packet_words;
	u32 empty_words;
	u32 read_index;
	u32 write_index;
	u32 new_write_index;

	if (!hfi || !hfi->table || !packet || packet_size < 2 * sizeof(u32) ||
	    !IS_ALIGNED(packet_size, sizeof(u32)) || packet_size > U32_MAX ||
	    le32_to_cpup(packet) != packet_size)
		return -EINVAL;

	queue = &hfi->queues[QCOM_CAMERA_ICP_HFI_CMD_QUEUE];
	header = queue->header;
	packet_words = packet_size / sizeof(u32);

	spin_lock_irqsave(&queue->lock, flags);
	read_index = le32_to_cpu(READ_ONCE(header->read_index));
	write_index = le32_to_cpu(READ_ONCE(header->write_index));
	if (read_index >= queue->word_count || write_index >= queue->word_count) {
		spin_unlock_irqrestore(&queue->lock, flags);
		return -EIO;
	}

	empty_words = write_index >= read_index ?
		queue->word_count - (write_index - read_index) :
		read_index - write_index;
	if (empty_words <= packet_words) {
		WRITE_ONCE(header->tx_request, cpu_to_le32(1));
		spin_unlock_irqrestore(&queue->lock, flags);
		return -ENOSPC;
	}

	hfi_copy_to_queue(queue->data, queue->word_count, write_index, packet,
			  packet_words);
	new_write_index = (write_index + packet_words) % queue->word_count;
	dma_wmb();
	WRITE_ONCE(header->write_index, cpu_to_le32(new_write_index));
	WRITE_ONCE(header->tx_request, cpu_to_le32(0));
	dma_wmb();
	spin_unlock_irqrestore(&queue->lock, flags);

	return 0;
}

int qcom_camera_icp_hfi_read(struct qcom_camera_icp_hfi *hfi,
			     enum qcom_camera_icp_hfi_queue_id id,
			     void *packet, size_t capacity, size_t *packet_size)
{
	struct qcom_camera_icp_hfi_queue_header *header;
	struct qcom_camera_icp_hfi_queue *queue;
	unsigned long flags;
	u32 available_words;
	u32 packet_bytes;
	u32 packet_words;
	u32 read_index;
	u32 write_index;
	u32 new_read_index;
	int ret = 0;

	if (!hfi || !hfi->table || !packet || !packet_size ||
	    (id != QCOM_CAMERA_ICP_HFI_MSG_QUEUE &&
	     id != QCOM_CAMERA_ICP_HFI_DBG_QUEUE))
		return -EINVAL;

	*packet_size = 0;
	queue = &hfi->queues[id];
	header = queue->header;
	spin_lock_irqsave(&queue->lock, flags);
	read_index = le32_to_cpu(READ_ONCE(header->read_index));
	write_index = le32_to_cpu(READ_ONCE(header->write_index));
	if (read_index >= queue->word_count || write_index >= queue->word_count) {
		ret = -EIO;
		goto unlock;
	}

	if (read_index == write_index) {
		if (id == QCOM_CAMERA_ICP_HFI_MSG_QUEUE)
			WRITE_ONCE(header->rx_request, cpu_to_le32(1));
		ret = -ENODATA;
		goto unlock;
	}

	dma_rmb();
	packet_bytes = le32_to_cpu(READ_ONCE(queue->data[read_index]));
	available_words = write_index >= read_index ? write_index - read_index :
		queue->word_count - (read_index - write_index);
	if (packet_bytes < 2 * sizeof(u32) ||
	    !IS_ALIGNED(packet_bytes, sizeof(u32))) {
		ret = -EBADMSG;
		goto discard;
	}

	packet_words = packet_bytes / sizeof(u32);
	if (packet_words > available_words || packet_words >= queue->word_count) {
		ret = -EBADMSG;
		goto discard;
	}
	*packet_size = packet_bytes;
	if (packet_bytes > capacity) {
		ret = -EMSGSIZE;
		goto unlock;
	}

	hfi_copy_from_queue(packet, queue->data, queue->word_count, read_index,
			    packet_words);
	new_read_index = (read_index + packet_words) % queue->word_count;
	dma_wmb();
	WRITE_ONCE(header->read_index, cpu_to_le32(new_read_index));
	WRITE_ONCE(header->rx_request,
		   cpu_to_le32(id == QCOM_CAMERA_ICP_HFI_MSG_QUEUE &&
			       new_read_index == write_index));
	dma_wmb();
	goto unlock;

discard:
	WRITE_ONCE(header->read_index, cpu_to_le32(write_index));
	if (id == QCOM_CAMERA_ICP_HFI_MSG_QUEUE)
		WRITE_ONCE(header->rx_request, cpu_to_le32(1));
	dma_wmb();
unlock:
	spin_unlock_irqrestore(&queue->lock, flags);
	return ret;
}
