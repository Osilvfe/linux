// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/remoteproc/qcom_camera_icp.h>

#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define QCOM_CAMERA_ICP_BPS_NAME	"qcom-camera-icp-bps"
#define V4L2_PIX_FMT_QCOM_TP10		v4l2_fourcc('Q', 'T', '1', '0')
#define QCOM_CAMERA_ICP_BPS_INPUT_IOVA	0x08000000
#define QCOM_CAMERA_ICP_BPS_OUTPUT_Y_IOVA 0x08100000
#define QCOM_CAMERA_ICP_BPS_OUTPUT_C_IOVA 0x08200000

struct qcom_camera_icp_bps_fmt {
	u32 pixelformat;
	u32 width;
	u32 height;
	u32 bytesperline[2];
	u32 sizeimage[2];
	u8 num_planes;
};

struct qcom_camera_icp_bps_dev {
	struct device *dev;
	struct qcom_camera_icp_bps *service;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex lock; /* Serializes V4L2 device and queue operations. */
};

struct qcom_camera_icp_bps_ctx {
	struct qcom_camera_icp_bps_dev *bps;
	struct v4l2_fh fh;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct qcom_camera_icp_bps_fmt output;
	struct qcom_camera_icp_bps_fmt capture;
	struct qcom_camera_icp_bps_session *session;
	bool powered;
	u64 request_id;
};

static void
qcom_camera_icp_bps_default_formats(struct qcom_camera_icp_bps_ctx *ctx)
{
	ctx->output.pixelformat = V4L2_PIX_FMT_SBGGR10P;
	ctx->output.width = 640;
	ctx->output.height = 480;
	ctx->output.num_planes = 1;
	ctx->output.bytesperline[0] = 832;
	ctx->output.sizeimage[0] = 832 * 480;

	ctx->capture.pixelformat = V4L2_PIX_FMT_QCOM_TP10;
	ctx->capture.width = 640;
	ctx->capture.height = 480;
	ctx->capture.num_planes = 2;
	ctx->capture.bytesperline[0] = 896;
	ctx->capture.bytesperline[1] = 896;
	ctx->capture.sizeimage[0] = 896 * 480;
	ctx->capture.sizeimage[1] = 896 * 240;
}

static struct qcom_camera_icp_bps_fmt *
qcom_camera_icp_bps_get_fmt(struct qcom_camera_icp_bps_ctx *ctx, u32 type)
{
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return &ctx->output;
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return &ctx->capture;

	return NULL;
}

static int qcom_camera_icp_bps_querycap(struct file *file, void *fh,
					struct v4l2_capability *cap)
{
	strscpy(cap->driver, QCOM_CAMERA_ICP_BPS_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Camera ICP BPS", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:qcom-camera-icp",
		sizeof(cap->bus_info));

	return 0;
}

static int qcom_camera_icp_bps_enum_fmt(struct file *file, void *fh,
				       struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		f->pixelformat = V4L2_PIX_FMT_SBGGR10P;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		f->pixelformat = V4L2_PIX_FMT_QCOM_TP10;
	else
		return -EINVAL;

	return 0;
}

static void
qcom_camera_icp_bps_fill_pix_mp(struct v4l2_pix_format_mplane *pix,
				const struct qcom_camera_icp_bps_fmt *fmt)
{
	unsigned int i;

	pix->width = fmt->width;
	pix->height = fmt->height;
	pix->pixelformat = fmt->pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->num_planes = fmt->num_planes;
	for (i = 0; i < fmt->num_planes; i++) {
		pix->plane_fmt[i].bytesperline = fmt->bytesperline[i];
		pix->plane_fmt[i].sizeimage = fmt->sizeimage[i];
	}
}

static int qcom_camera_icp_bps_g_fmt(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct qcom_camera_icp_bps_ctx *ctx = fh;
	struct qcom_camera_icp_bps_fmt *fmt;

	fmt = qcom_camera_icp_bps_get_fmt(ctx, f->type);
	if (!fmt)
		return -EINVAL;
	qcom_camera_icp_bps_fill_pix_mp(&f->fmt.pix_mp, fmt);

	return 0;
}

static int qcom_camera_icp_bps_try_fmt(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct qcom_camera_icp_bps_ctx *ctx = fh;
	struct qcom_camera_icp_bps_fmt *fmt;

	fmt = qcom_camera_icp_bps_get_fmt(ctx, f->type);
	if (!fmt)
		return -EINVAL;
	qcom_camera_icp_bps_fill_pix_mp(&f->fmt.pix_mp, fmt);

	return 0;
}

static int qcom_camera_icp_bps_s_fmt(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct qcom_camera_icp_bps_ctx *ctx = fh;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;
	if (vb2_is_busy(vq))
		return -EBUSY;

	return qcom_camera_icp_bps_try_fmt(file, fh, f);
}

static int qcom_camera_icp_bps_queue_setup(struct vb2_queue *vq,
				   unsigned int *num_buffers,
				   unsigned int *num_planes,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct qcom_camera_icp_bps_ctx *ctx = vb2_get_drv_priv(vq);
	struct qcom_camera_icp_bps_fmt *fmt;
	unsigned int i;

	fmt = qcom_camera_icp_bps_get_fmt(ctx, vq->type);
	if (!fmt)
		return -EINVAL;
	if (*num_planes) {
		if (*num_planes != fmt->num_planes)
			return -EINVAL;
		for (i = 0; i < *num_planes; i++)
			if (sizes[i] < fmt->sizeimage[i])
				return -EINVAL;
		return 0;
	}

	*num_planes = fmt->num_planes;
	for (i = 0; i < *num_planes; i++)
		sizes[i] = fmt->sizeimage[i];

	return 0;
}

static int qcom_camera_icp_bps_buf_prepare(struct vb2_buffer *vb)
{
	struct qcom_camera_icp_bps_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct qcom_camera_icp_bps_fmt *fmt;
	unsigned int i;

	fmt = qcom_camera_icp_bps_get_fmt(ctx, vb->vb2_queue->type);
	for (i = 0; i < fmt->num_planes; i++) {
		if (vb2_plane_size(vb, i) < fmt->sizeimage[i])
			return -EINVAL;
		vb2_set_plane_payload(vb, i, fmt->sizeimage[i]);
	}

	return 0;
}

static void qcom_camera_icp_bps_buf_queue(struct vb2_buffer *vb)
{
	struct qcom_camera_icp_bps_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int qcom_camera_icp_bps_start_streaming(struct vb2_queue *vq,
				       unsigned int count)
{
	struct qcom_camera_icp_bps_ctx *ctx = vb2_get_drv_priv(vq);
	struct qcom_camera_icp_bps *service = ctx->bps->service;
	struct vb2_queue *other;
	int ret;

	other = V4L2_TYPE_IS_OUTPUT(vq->type) ?
		v4l2_m2m_get_dst_vq(ctx->m2m_ctx) :
		v4l2_m2m_get_src_vq(ctx->m2m_ctx);
	if (!vb2_is_streaming(other) || ctx->session)
		return 0;

	ret = service->ops->power_on(service);
	if (ret)
		return ret;
	ctx->powered = true;
	ctx->session = service->ops->session_open(service);
	if (IS_ERR(ctx->session)) {
		ret = PTR_ERR(ctx->session);
		ctx->session = NULL;
		service->ops->power_off(service);
		ctx->powered = false;
		return ret;
	}
	ret = service->ops->session_prepare(ctx->session);
	if (ret) {
		service->ops->session_close(ctx->session);
		ctx->session = NULL;
		service->ops->power_off(service);
		ctx->powered = false;
		return ret;
	}

	return 0;
}

static void qcom_camera_icp_bps_stop_streaming(struct vb2_queue *vq)
{
	struct qcom_camera_icp_bps_ctx *ctx = vb2_get_drv_priv(vq);
	struct qcom_camera_icp_bps *service = ctx->bps->service;
	struct vb2_v4l2_buffer *buf;
	int ret;

	if (ctx->session) {
		ret = service->ops->session_close(ctx->session);
		if (ret)
			dev_warn(ctx->bps->dev, "failed to close BPS session: %d\n",
				 ret);
		ctx->session = NULL;
	}
	if (ctx->powered) {
		service->ops->power_off(service);
		ctx->powered = false;
	}

	for (;;) {
		buf = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		if (!buf)
			break;
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops qcom_camera_icp_bps_vb2_ops = {
	.queue_setup = qcom_camera_icp_bps_queue_setup,
	.buf_prepare = qcom_camera_icp_bps_buf_prepare,
	.buf_queue = qcom_camera_icp_bps_buf_queue,
	.start_streaming = qcom_camera_icp_bps_start_streaming,
	.stop_streaming = qcom_camera_icp_bps_stop_streaming,
};

static int qcom_camera_icp_bps_queue_init(void *priv,
				  struct vb2_queue *src_vq,
				  struct vb2_queue *dst_vq)
{
	struct qcom_camera_icp_bps_ctx *ctx = priv;
	struct vb2_queue *queues[] = { src_vq, dst_vq };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		queues[i]->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
				       V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		queues[i]->io_modes = VB2_MMAP | VB2_DMABUF;
		queues[i]->ops = &qcom_camera_icp_bps_vb2_ops;
		queues[i]->mem_ops = &vb2_dma_contig_memops;
		queues[i]->drv_priv = ctx;
		queues[i]->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		queues[i]->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		queues[i]->lock = &ctx->bps->lock;
		queues[i]->dev = ctx->bps->dev->parent;
		ret = vb2_queue_init(queues[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static void qcom_camera_icp_bps_device_run(void *priv)
{
	struct qcom_camera_icp_bps_ctx *ctx = priv;
	struct qcom_camera_icp_bps *service = ctx->bps->service;
	struct vb2_v4l2_buffer *src;
	struct vb2_v4l2_buffer *dst;
	struct qcom_camera_icp_bps_job job = {
		.input_iova = QCOM_CAMERA_ICP_BPS_INPUT_IOVA,
		.output_y_iova = QCOM_CAMERA_ICP_BPS_OUTPUT_Y_IOVA,
		.output_c_iova = QCOM_CAMERA_ICP_BPS_OUTPUT_C_IOVA,
	};
	dma_addr_t source_iova[3];
	dma_addr_t alias_iova[3] = { job.input_iova, job.output_y_iova,
				     job.output_c_iova };
	size_t size[3] = { PAGE_ALIGN(832 * 480), PAGE_ALIGN(896 * 480),
			   PAGE_ALIGN(896 * 240) };
	enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
	unsigned int alias_count = 0;
	unsigned int map_count = 0;
	int cleanup_ret;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if (!src || !dst || !ctx->session)
		goto finish;

	source_iova[0] = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	source_iova[1] = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	source_iova[2] = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 1);
	for (alias_count = 0; alias_count < ARRAY_SIZE(alias_iova); alias_count++) {
		ret = service->ops->alias_map(service, source_iova[alias_count],
					      alias_iova[alias_count],
					      size[alias_count]);
		if (ret)
			goto unmap_aliases;
	}
	for (map_count = 0; map_count < ARRAY_SIZE(alias_iova); map_count++) {
		ret = service->ops->session_map(ctx->session, alias_iova[map_count],
						size[map_count]);
		if (ret)
			goto unmap_session;
	}
	job.request_id = ++ctx->request_id;
	ret = service->ops->session_process(ctx->session, &job);
	if (!ret)
		state = VB2_BUF_STATE_DONE;

unmap_session:
	while (map_count) {
		map_count--;
		cleanup_ret = service->ops->session_unmap(ctx->session,
						 alias_iova[map_count],
						 size[map_count]);
		if (cleanup_ret) {
			dev_warn(ctx->bps->dev,
				 "failed to unmap BPS session buffer: %d\n",
				 cleanup_ret);
			state = VB2_BUF_STATE_ERROR;
		}
	}
unmap_aliases:
	while (alias_count) {
		alias_count--;
		service->ops->alias_unmap(service, alias_iova[alias_count],
					  size[alias_count]);
	}
finish:
	if (src) {
		src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	}
	if (dst) {
		dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		if (src)
			v4l2_m2m_buf_copy_metadata(src, dst);
		v4l2_m2m_buf_done(dst, state);
	}
	if (src)
		v4l2_m2m_buf_done(src, state);

	v4l2_m2m_job_finish(ctx->bps->m2m_dev, ctx->m2m_ctx);
}

static void qcom_camera_icp_bps_job_abort(void *priv)
{
	qcom_camera_icp_bps_device_run(priv);
}

static const struct v4l2_m2m_ops qcom_camera_icp_bps_m2m_ops = {
	.device_run = qcom_camera_icp_bps_device_run,
	.job_abort = qcom_camera_icp_bps_job_abort,
};

static int qcom_camera_icp_bps_open(struct file *file)
{
	struct qcom_camera_icp_bps_dev *bps = video_drvdata(file);
	struct qcom_camera_icp_bps_ctx *ctx;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->bps = bps;
	qcom_camera_icp_bps_default_formats(ctx);
	v4l2_fh_init(&ctx->fh, &bps->vdev);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh, file);
	ctx->m2m_ctx = v4l2_m2m_ctx_init(bps->m2m_dev, ctx,
					 qcom_camera_icp_bps_queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		int ret = PTR_ERR(ctx->m2m_ctx);

		v4l2_fh_del(&ctx->fh, file);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}
	ctx->fh.m2m_ctx = ctx->m2m_ctx;

	return 0;
}

static int qcom_camera_icp_bps_release(struct file *file)
{
	struct qcom_camera_icp_bps_ctx *ctx;

	ctx = container_of(file->private_data,
			   struct qcom_camera_icp_bps_ctx, fh);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations qcom_camera_icp_bps_fops = {
	.owner = THIS_MODULE,
	.open = qcom_camera_icp_bps_open,
	.release = qcom_camera_icp_bps_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops qcom_camera_icp_bps_ioctl_ops = {
	.vidioc_querycap = qcom_camera_icp_bps_querycap,
	.vidioc_enum_fmt_vid_cap = qcom_camera_icp_bps_enum_fmt,
	.vidioc_enum_fmt_vid_out = qcom_camera_icp_bps_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = qcom_camera_icp_bps_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = qcom_camera_icp_bps_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = qcom_camera_icp_bps_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = qcom_camera_icp_bps_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = qcom_camera_icp_bps_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = qcom_camera_icp_bps_s_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int qcom_camera_icp_bps_probe(struct auxiliary_device *auxdev,
				     const struct auxiliary_device_id *id)
{
	struct qcom_camera_icp_bps_dev *bps;
	int ret;

	bps = devm_kzalloc(&auxdev->dev, sizeof(*bps), GFP_KERNEL);
	if (!bps)
		return -ENOMEM;
	bps->dev = &auxdev->dev;
	bps->service = dev_get_platdata(&auxdev->dev);
	if (!bps->service || !bps->service->ops)
		return -EINVAL;
	mutex_init(&bps->lock);

	ret = v4l2_device_register(&auxdev->dev, &bps->v4l2_dev);
	if (ret)
		return ret;
	bps->m2m_dev = v4l2_m2m_init(&qcom_camera_icp_bps_m2m_ops);
	if (IS_ERR(bps->m2m_dev)) {
		ret = PTR_ERR(bps->m2m_dev);
		goto unregister_v4l2;
	}

	strscpy(bps->vdev.name, QCOM_CAMERA_ICP_BPS_NAME,
		sizeof(bps->vdev.name));
	bps->vdev.v4l2_dev = &bps->v4l2_dev;
	bps->vdev.fops = &qcom_camera_icp_bps_fops;
	bps->vdev.ioctl_ops = &qcom_camera_icp_bps_ioctl_ops;
	bps->vdev.lock = &bps->lock;
	bps->vdev.release = video_device_release_empty;
	bps->vdev.vfl_dir = VFL_DIR_M2M;
	bps->vdev.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE |
				V4L2_CAP_STREAMING;
	video_set_drvdata(&bps->vdev, bps);

	ret = video_register_device(&bps->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto release_m2m;
	auxiliary_set_drvdata(auxdev, bps);

	return 0;

release_m2m:
	v4l2_m2m_release(bps->m2m_dev);
unregister_v4l2:
	v4l2_device_unregister(&bps->v4l2_dev);
	return ret;
}

static void qcom_camera_icp_bps_remove(struct auxiliary_device *auxdev)
{
	struct qcom_camera_icp_bps_dev *bps = auxiliary_get_drvdata(auxdev);

	video_unregister_device(&bps->vdev);
	v4l2_m2m_release(bps->m2m_dev);
	v4l2_device_unregister(&bps->v4l2_dev);
}

static const struct auxiliary_device_id qcom_camera_icp_bps_id_table[] = {
	{ .name = "qcom_camera_icp.bps" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, qcom_camera_icp_bps_id_table);

static struct auxiliary_driver qcom_camera_icp_bps_driver = {
	.probe = qcom_camera_icp_bps_probe,
	.remove = qcom_camera_icp_bps_remove,
	.id_table = qcom_camera_icp_bps_id_table,
	.driver.name = QCOM_CAMERA_ICP_BPS_NAME,
};
module_auxiliary_driver(qcom_camera_icp_bps_driver);

MODULE_DESCRIPTION("Qualcomm Camera ICP BPS V4L2 mem2mem driver");
MODULE_LICENSE("GPL");
