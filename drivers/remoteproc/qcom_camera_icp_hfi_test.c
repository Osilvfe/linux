// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#include <kunit/test.h>
#include <linux/slab.h>

#include "qcom_camera_icp_bps.h"
#include "qcom_camera_icp_hfi.h"

struct qcom_camera_icp_hfi_test_context {
	struct qcom_camera_icp_hfi hfi;
	struct qcom_camera_icp_hfi_memory memory;
};

static void qcom_camera_icp_hfi_region_free(void *data)
{
	kvfree(data);
}

static int qcom_camera_icp_hfi_test_init(struct kunit *test)
{
	struct qcom_camera_icp_hfi_test_context *context;
	static const size_t sizes[] = {
		QCOM_CAMERA_ICP_HFI_CMD_QUEUE_SIZE,
		QCOM_CAMERA_ICP_HFI_MSG_QUEUE_SIZE,
		QCOM_CAMERA_ICP_HFI_DBG_QUEUE_SIZE,
	};
	size_t table_size = sizeof(struct qcom_camera_icp_hfi_table);
	int id;
	int ret;

	context = kunit_kzalloc(test, sizeof(*context), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, context);
	context->memory.table = kunit_kzalloc(test, table_size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, context->memory.table);
	context->memory.table_iova = 0xc0300000;
	for (id = 0; id < QCOM_CAMERA_ICP_HFI_QUEUE_COUNT; id++) {
		context->memory.queue[id] = kunit_kzalloc(test, sizes[id], GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, context->memory.queue[id]);
		context->memory.queue_iova[id] = 0xc0302000 + id * 0x20000;
	}

	ret = qcom_camera_icp_hfi_init(&context->hfi, &context->memory);
	KUNIT_ASSERT_EQ(test, ret, 0);
	test->priv = context;
	return 0;
}

static void qcom_camera_icp_hfi_layout_test(struct kunit *test)
{
	struct qcom_camera_icp_hfi_test_context *context = test->priv;
	struct qcom_camera_icp_hfi_table *table = context->memory.table;

	KUNIT_EXPECT_EQ(test, le32_to_cpu(table->header.size), (u32)sizeof(*table));
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table->header.queue_count), 3U);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table->queues[0].start_addr), 0xc0302000U);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(table->queues[2].tx_watermark), 1024U);
}

static void qcom_camera_icp_hfi_region_layout_test(struct kunit *test)
{
	struct qcom_camera_icp_hfi_region region = {
		.iova = 0xc0300000,
		.size = QCOM_CAMERA_ICP_HFI_REGION_SIZE,
	};
	struct qcom_camera_icp_hfi hfi;
	int ret;

	region.base = kvzalloc(region.size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, region.base);
	ret = kunit_add_action_or_reset(test, qcom_camera_icp_hfi_region_free,
					region.base);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ret = qcom_camera_icp_hfi_region_init(&hfi, &region);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_PTR_EQ(test, region.secondary_heap, region.base);
	KUNIT_EXPECT_EQ(test, region.secondary_heap_iova,
			(u64)0xc0300000);
	KUNIT_EXPECT_EQ(test, region.secondary_heap_size, (size_t)0x101000);
	KUNIT_EXPECT_EQ(test, region.memory.table_iova, (u64)0xc0401000);
	KUNIT_EXPECT_PTR_EQ(test, region.memory.queue[0],
			    (void *)((u8 *)region.base + 0x103000));
	KUNIT_EXPECT_EQ(test, region.memory.queue_iova[0],
			(u64)0xc0403000);
	KUNIT_EXPECT_EQ(test, region.memory.queue_iova[1],
			(u64)0xc0406000);
	KUNIT_EXPECT_EQ(test, region.memory.queue_iova[2],
			(u64)0xc0409000);
	KUNIT_EXPECT_EQ(test, region.sfr_iova, (u64)0xc0423000);
	KUNIT_EXPECT_EQ(test, le32_to_cpup(region.sfr),
			QCOM_CAMERA_ICP_HFI_SFR_SIZE);
}

static void qcom_camera_icp_hfi_write_wrap_test(struct kunit *test)
{
	struct qcom_camera_icp_hfi_test_context *context = test->priv;
	struct qcom_camera_icp_hfi_queue *queue = &context->hfi.queues[0];
	__le32 packet[] = { 0, cpu_to_le32(0x10001),
			    cpu_to_le32(0x12345678) };
	int ret;

	packet[0] = cpu_to_le32(sizeof(packet));
	queue->header->read_index = cpu_to_le32(2);
	queue->header->write_index = cpu_to_le32(queue->word_count - 1);
	ret = qcom_camera_icp_hfi_write(&context->hfi, packet, sizeof(packet));
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(queue->header->write_index), 2U);
	KUNIT_EXPECT_EQ(test, queue->data[queue->word_count - 1], packet[0]);
	KUNIT_EXPECT_EQ(test, queue->data[0], packet[1]);
	KUNIT_EXPECT_EQ(test, queue->data[1], packet[2]);
}

static void qcom_camera_icp_hfi_queue_full_test(struct kunit *test)
{
	struct qcom_camera_icp_hfi_test_context *context = test->priv;
	struct qcom_camera_icp_hfi_queue *queue = &context->hfi.queues[0];
	__le32 packet[] = { 0, cpu_to_le32(0x10001) };
	int ret;

	packet[0] = cpu_to_le32(sizeof(packet));
	queue->header->read_index = cpu_to_le32(2);
	queue->header->write_index = cpu_to_le32(0);
	ret = qcom_camera_icp_hfi_write(&context->hfi, packet, sizeof(packet));
	KUNIT_EXPECT_EQ(test, ret, -ENOSPC);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(queue->header->tx_request), 1U);
}

static void qcom_camera_icp_hfi_read_wrap_test(struct kunit *test)
{
	struct qcom_camera_icp_hfi_test_context *context = test->priv;
	struct qcom_camera_icp_hfi_queue *queue = &context->hfi.queues[1];
	__le32 expected[] = { 0, cpu_to_le32(0x20001),
			      cpu_to_le32(0xabcdef01) };
	__le32 result[ARRAY_SIZE(expected)] = {};
	size_t packet_size = 0;
	int ret;

	expected[0] = cpu_to_le32(sizeof(expected));
	queue->header->read_index = cpu_to_le32(queue->word_count - 1);
	queue->header->write_index = cpu_to_le32(2);
	queue->data[queue->word_count - 1] = expected[0];
	queue->data[0] = expected[1];
	queue->data[1] = expected[2];
	ret = qcom_camera_icp_hfi_read(&context->hfi,
				       QCOM_CAMERA_ICP_HFI_MSG_QUEUE, result,
				       sizeof(result), &packet_size);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, packet_size, sizeof(expected));
	KUNIT_EXPECT_MEMEQ(test, result, expected, sizeof(expected));
	KUNIT_EXPECT_EQ(test, le32_to_cpu(queue->header->read_index), 2U);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(queue->header->rx_request), 1U);
}

static void qcom_camera_icp_hfi_bad_packet_test(struct kunit *test)
{
	struct qcom_camera_icp_hfi_test_context *context = test->priv;
	struct qcom_camera_icp_hfi_queue *queue = &context->hfi.queues[1];
	__le32 result[4];
	size_t packet_size;
	int ret;

	queue->header->read_index = cpu_to_le32(0);
	queue->header->write_index = cpu_to_le32(1);
	queue->data[0] = cpu_to_le32(3);
	ret = qcom_camera_icp_hfi_read(&context->hfi,
				       QCOM_CAMERA_ICP_HFI_MSG_QUEUE, result,
				       sizeof(result), &packet_size);
	KUNIT_EXPECT_EQ(test, ret, -EBADMSG);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(queue->header->read_index), 1U);
}

static void qcom_camera_icp_bps_abi_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_create_handle),
			28UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_async_indirect),
			40UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_async_map), 56UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_async_destroy),
			44UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_image_info), 64UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_image_desc), 96UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_bps_config), 1248UL);
	KUNIT_EXPECT_EQ(test,
			sizeof(struct qcom_camera_icp_hfi_bps_frame_buffers), 20UL);
	KUNIT_EXPECT_EQ(test,
			sizeof(struct qcom_camera_icp_hfi_bps_frame_set), 240UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_bps_frame_data),
			5700UL);
	KUNIT_EXPECT_EQ(test, sizeof(struct qcom_camera_icp_hfi_bps_frame), 5712UL);
	KUNIT_EXPECT_EQ(test,
			sizeof(struct qcom_camera_icp_hfi_bps_cdm_program), 24UL);
	KUNIT_EXPECT_EQ(test,
			sizeof(struct qcom_camera_icp_hfi_bps_frame_payload), 6128UL);
}

static struct kunit_case qcom_camera_icp_hfi_test_cases[] = {
	KUNIT_CASE(qcom_camera_icp_hfi_layout_test),
	KUNIT_CASE(qcom_camera_icp_hfi_region_layout_test),
	KUNIT_CASE(qcom_camera_icp_hfi_write_wrap_test),
	KUNIT_CASE(qcom_camera_icp_hfi_queue_full_test),
	KUNIT_CASE(qcom_camera_icp_hfi_read_wrap_test),
	KUNIT_CASE(qcom_camera_icp_hfi_bad_packet_test),
	KUNIT_CASE(qcom_camera_icp_bps_abi_test),
	{}
};

static struct kunit_suite qcom_camera_icp_hfi_test_suite = {
	.name = "qcom-camera-icp-hfi",
	.init = qcom_camera_icp_hfi_test_init,
	.test_cases = qcom_camera_icp_hfi_test_cases,
};

kunit_test_suite(qcom_camera_icp_hfi_test_suite);
