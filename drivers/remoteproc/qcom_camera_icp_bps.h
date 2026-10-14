/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */

#ifndef __QCOM_CAMERA_ICP_BPS_H__
#define __QCOM_CAMERA_ICP_BPS_H__

#include <linux/errno.h>
#include <linux/types.h>

#define QCOM_CAMERA_ICP_HFI_BPS_NON_RT		1
#define QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO	1
#define QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS	2
#define QCOM_CAMERA_ICP_HFI_BPS_DESTROY		4
#define QCOM_CAMERA_ICP_HFI_MEM_MAP		0xe
#define QCOM_CAMERA_ICP_HFI_MEM_UNMAP		0xf

#define QCOM_CAMERA_ICP_HFI_IMAGE_FORMAT_MIPI_10	2
#define QCOM_CAMERA_ICP_HFI_IMAGE_FORMAT_LINEAR_TP_10	18
#define QCOM_CAMERA_ICP_HFI_BPS_IMAGE_COUNT		12
#define QCOM_CAMERA_ICP_HFI_BPS_INPUT_IMAGE		0
#define QCOM_CAMERA_ICP_HFI_BPS_OUTPUT_IMAGE_FULL	1
#define QCOM_CAMERA_ICP_HFI_BPS_MAX_FRAME_SETS		16
#define QCOM_CAMERA_ICP_HFI_BPS_MAX_CDM_PROGRAMS	17
#define QCOM_CAMERA_ICP_HFI_BPS_BUFFER_SHIFT		8

static inline int qcom_camera_icp_bps_validate_region(u64 iova, u64 size)
{
	if (!size)
		return -EINVAL;
	if (iova > U32_MAX || size > U32_MAX ||
	    iova > U32_MAX - (size - 1))
		return -ERANGE;

	return 0;
}

static inline bool qcom_camera_icp_bps_region_contains(u64 base, u64 size,
					       u64 iova, u64 length)
{
	return length && iova >= base && iova - base < size &&
	       length <= size - (iova - base);
}

static inline int qcom_camera_icp_bps_encode_buffer(u64 iova, __le32 *encoded)
{
	if (iova & ((1U << QCOM_CAMERA_ICP_HFI_BPS_BUFFER_SHIFT) - 1))
		return -EINVAL;
	if (iova >> QCOM_CAMERA_ICP_HFI_BPS_BUFFER_SHIFT > U32_MAX)
		return -ERANGE;

	*encoded = cpu_to_le32(iova >> QCOM_CAMERA_ICP_HFI_BPS_BUFFER_SHIFT);
	return 0;
}

struct qcom_camera_icp_hfi_create_handle {
	__le32 size;
	__le32 packet_type;
	__le32 handle_type;
	__le64 user_data1;
	__le64 user_data2;
} __packed;

struct qcom_camera_icp_hfi_async_indirect {
	__le32 size;
	__le32 packet_type;
	__le32 opcode;
	__le64 user_data1;
	__le64 user_data2;
	__le32 num_fw_handles;
	__le32 fw_handle;
	__le32 payload_iova;
} __packed;

struct qcom_camera_icp_hfi_async_map {
	__le32 size;
	__le32 packet_type;
	__le32 opcode;
	__le64 user_data1;
	__le64 user_data2;
	__le32 num_fw_handles;
	__le32 fw_handle;
	__le64 map_user_data;
	__le32 num_regions;
	__le32 region_iova;
	__le32 region_size;
} __packed;

struct qcom_camera_icp_hfi_async_destroy {
	__le32 size;
	__le32 packet_type;
	__le32 opcode;
	__le64 user_data1;
	__le64 user_data2;
	__le32 num_fw_handles;
	__le32 fw_handle;
	__le64 destroy_user_data;
} __packed;

struct qcom_camera_icp_hfi_buffer_layout {
	__le32 stride;
	__le32 height;
} __packed;

struct qcom_camera_icp_hfi_image_info {
	__le32 format;
	__le32 width;
	__le32 height;
	__le32 bayer_order;
	__le32 pixel_alignment;
	__le32 yuv422_order;
	__le32 byte_swap;
	__le32 ubwc_version;
	__le32 ubwc_lossy_mode;
	__le32 ubwc_bw_limit;
	__le32 ubwc_lossy_threshold0;
	__le32 ubwc_lossy_threshold1;
	__le32 argb_alpha;
	__le32 ubwc_lossy_offset;
	__le32 from_downscale;
	__le32 hdr_content;
} __packed;

struct qcom_camera_icp_hfi_image_desc {
	struct qcom_camera_icp_hfi_image_info info;
	struct qcom_camera_icp_hfi_buffer_layout plane[2];
	struct qcom_camera_icp_hfi_buffer_layout metadata[2];
} __packed;

struct qcom_camera_icp_hfi_cadl_pass {
	__le32 width;
	__le32 height;
	__le32 channels;
	__le32 bps_input;
	__le32 cadl_input;
} __packed;

struct qcom_camera_icp_hfi_bps_config {
	struct qcom_camera_icp_hfi_image_desc
		image[QCOM_CAMERA_ICP_HFI_BPS_IMAGE_COUNT];
	__le32 secure_mode;
	__le32 cadl_pass_count;
	struct qcom_camera_icp_hfi_cadl_pass cadl_pass[4];
	__le64 user_data;
} __packed;

struct qcom_camera_icp_hfi_bps_frame_buffers {
	__le32 buffer[2];
	__le32 metadata[2];
	__le32 synx_handle;
} __packed;

struct qcom_camera_icp_hfi_bps_frame_set {
	struct qcom_camera_icp_hfi_bps_frame_buffers
		buffer[QCOM_CAMERA_ICP_HFI_BPS_IMAGE_COUNT];
} __packed;

struct qcom_camera_icp_hfi_bps_frame_data {
	__le32 max_cores;
	__le32 target_time_ns;
	__le32 ubwc_stats_iova;
	__le32 ubwc_stats_size;
	__le32 cdm_iova;
	__le32 cdm_size;
	__le32 iq_iova;
	__le32 striping_iova;
	__le32 cdm_program_array_iova;
	__le32 request_id;
	__le64 timestamp;
	__le32 num_frame_sets;
	struct qcom_camera_icp_hfi_bps_frame_set
		frame_set[QCOM_CAMERA_ICP_HFI_BPS_MAX_FRAME_SETS];
	u8 cadl_frame_set[560];
	__le32 reconfig_valid;
	u8 reconfig_data[1240];
	__le32 domain_id;
} __packed;

struct qcom_camera_icp_hfi_bps_frame {
	struct qcom_camera_icp_hfi_bps_frame_data data;
	__le32 alignment_hole;
	__le64 user_data;
} __packed;

struct qcom_camera_icp_hfi_bps_cdm_program {
	__le64 base_length;
	__le32 type;
	__le32 uid;
	__le32 single_register;
	__le32 internally_allocated;
} __packed;

struct qcom_camera_icp_hfi_bps_frame_payload {
	struct qcom_camera_icp_hfi_bps_frame frame;
	__le32 cdm_allocator;
	__le32 cdm_num_programs;
	struct qcom_camera_icp_hfi_bps_cdm_program
		program[QCOM_CAMERA_ICP_HFI_BPS_MAX_CDM_PROGRAMS];
} __packed;

#endif
