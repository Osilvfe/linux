/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */

#ifndef __LINUX_REMOTEPROC_QCOM_CAMERA_ICP_H__
#define __LINUX_REMOTEPROC_QCOM_CAMERA_ICP_H__

#include <linux/types.h>

struct qcom_camera_icp_bps;
struct qcom_camera_icp_bps_session;
struct sg_table;

struct qcom_camera_icp_bps_job {
	dma_addr_t input_iova;
	dma_addr_t output_y_iova;
	dma_addr_t output_c_iova;
	u64 request_id;
};

struct qcom_camera_icp_bps_ops {
	int (*power_on)(struct qcom_camera_icp_bps *bps);
	void (*power_off)(struct qcom_camera_icp_bps *bps);
	struct qcom_camera_icp_bps_session *
		(*session_open)(struct qcom_camera_icp_bps *bps);
	int (*session_close)(struct qcom_camera_icp_bps_session *session);
	void (*session_abort)(struct qcom_camera_icp_bps_session *session);
	int (*session_map)(struct qcom_camera_icp_bps_session *session,
			   dma_addr_t iova, size_t size);
	int (*session_unmap)(struct qcom_camera_icp_bps_session *session,
			     dma_addr_t iova, size_t size);
	int (*session_config)(struct qcom_camera_icp_bps_session *session,
			      dma_addr_t payload_iova);
	int (*session_submit)(struct qcom_camera_icp_bps_session *session,
			      dma_addr_t payload_iova, u64 request_id);
	int (*session_prepare)(struct qcom_camera_icp_bps_session *session);
	int (*session_process)(struct qcom_camera_icp_bps_session *session,
			       const struct qcom_camera_icp_bps_job *job);
	int (*alias_map)(struct qcom_camera_icp_bps *bps, dma_addr_t source_iova,
			 dma_addr_t bps_iova, size_t size);
	int (*alias_map_sg)(struct qcom_camera_icp_bps *bps,
			    struct sg_table *sgt, dma_addr_t bps_iova,
			    size_t size);
	void (*alias_unmap)(struct qcom_camera_icp_bps *bps, dma_addr_t bps_iova,
			    size_t size);
};

struct qcom_camera_icp_bps {
	const struct qcom_camera_icp_bps_ops *ops;
};

#endif
