#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc/qcom_camera_icp.h>
#include <linux/swab.h>
#include <linux/unaligned.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-sg.h>

#include "../camss/camss.h"
#include "../camss/camss-video.h"

static bool enable;
module_param(enable, bool, 0400);
MODULE_PARM_DESC(enable, "Explicitly enable experimental native capture");

struct native_mode {
	u32 width;
	u32 height;
};

struct native_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head entry;
};

struct native_device;

struct native_camera {
	struct native_device *owner;
	struct video_device vdev;
	struct vb2_queue queue;
	struct mutex lock;
	spinlock_t pending_lock;
	struct list_head pending;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
	struct v4l2_ctrl *auto_balance;
	struct v4l2_ctrl *auto_exposure;
	struct v4l2_ctrl *rotation;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_subdev *sensor;
	struct camss_video *raw_video;
	struct camss_video_capture *capture;
	struct qcom_camera_icp_bps_client *client;
	struct task_struct *worker;
	struct native_mode modes[2];
	struct v4l2_pix_format format;
	struct sg_table *isp_buffer;
	u8 *isp_pixels;
	u32 isp_stride;
	size_t isp_size;
	unsigned int route;
	unsigned int auto_frames_until_update;
	u64 request_id;
};

struct native_device {
	struct v4l2_device v4l2;
	struct camss *camss;
	struct device *dma_dev;
	struct qcom_camera_icp_bps *bps;
	struct native_camera cameras[2];
	unsigned int initialized;
};

static void native_fill_format(struct native_camera *camera,
			       struct v4l2_pix_format *format)
{
	const struct native_mode *mode = &camera->modes[0];
	u64 distance = U64_MAX;
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(camera->modes); index++) {
		u64 candidate = abs((s64)format->width - camera->modes[index].width) +
			abs((s64)format->height - camera->modes[index].height);

		if (candidate < distance) {
			distance = candidate;
			mode = &camera->modes[index];
		}
	}
	*format = (struct v4l2_pix_format) {
		.width = mode->width,
		.height = mode->height,
		.pixelformat = V4L2_PIX_FMT_NV12,
		.field = V4L2_FIELD_NONE,
		.bytesperline = mode->width,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};
	format->sizeimage = format->bytesperline * mode->height * 3 / 2;
}

static int native_configure(struct native_camera *camera,
			    struct v4l2_format *raw_format)
{
	struct camss *camss = camera->owner->camss;
	struct csiphy_device *phy = camera->sensor->host_priv;
	struct v4l2_subdev *subdevs[] = {
		camera->sensor, &phy->subdev, &camss->csid[camera->route].subdev,
		&camss->vfe[camera->route].line[VFE_LINE_RDI0].subdev,
	};
	struct media_link *wanted[2];
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.width = camera->format.width,
			.height = camera->format.height,
			.code = MEDIA_BUS_FMT_SBGGR10_1X10,
			.field = V4L2_FIELD_NONE,
		},
	};
	unsigned int index;
	int ret = 0;

	mutex_lock(&camss->media_dev.graph_mutex);
	for (index = 0; index < ARRAY_SIZE(subdevs); index++) {
		if (media_entity_is_streaming(&subdevs[index]->entity)) {
			ret = -EBUSY;
			goto unlock;
		}
	}
	wanted[0] = media_entity_find_link(&phy->pads[MSM_CSIPHY_PAD_SRC],
		&camss->csid[camera->route].pads[MSM_CSID_PAD_SINK]);
	wanted[1] = media_entity_find_link(
		&camss->csid[camera->route].pads[MSM_CSID_PAD_SRC],
		&camss->vfe[camera->route].line[VFE_LINE_RDI0].pads[MSM_VFE_PAD_SINK]);
	if (!wanted[0] || !wanted[1]) {
		ret = -ENOLINK;
		goto unlock;
	}
	for (index = 1; index < ARRAY_SIZE(subdevs); index++) {
		struct media_link *link;

		list_for_each_entry(link, &subdevs[index]->entity.links, list) {
			struct media_link *forward = link->is_backlink ? link->reverse : link;

			if (!(forward->flags & MEDIA_LNK_FL_ENABLED) ||
			    forward->flags & MEDIA_LNK_FL_IMMUTABLE ||
			    forward == wanted[0] || forward == wanted[1])
				continue;
			ret = __media_entity_setup_link(forward,
				forward->flags & ~MEDIA_LNK_FL_ENABLED);
			if (ret)
				goto unlock;
		}
	}
	for (index = 0; index < ARRAY_SIZE(wanted); index++) {
		ret = __media_entity_setup_link(wanted[index],
			wanted[index]->flags | MEDIA_LNK_FL_ENABLED);
		if (ret)
			goto unlock;
	}
	for (index = 0; index < ARRAY_SIZE(subdevs); index++) {
		format.pad = 0;
		ret = v4l2_subdev_call(subdevs[index], pad, set_fmt, NULL, &format);
		if (ret)
			goto unlock;
		if (index) {
			format.pad = 1;
			ret = v4l2_subdev_call(subdevs[index], pad, get_fmt, NULL, &format);
			if (ret)
				goto unlock;
		}
		if (format.format.width != camera->format.width ||
		    format.format.height != camera->format.height ||
		    format.format.code != MEDIA_BUS_FMT_SBGGR10_1X10) {
			ret = -EINVAL;
			goto unlock;
		}
	}
	*raw_format = (struct v4l2_format) {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.fmt.pix_mp = {
			.width = camera->format.width,
			.height = camera->format.height,
			.pixelformat = V4L2_PIX_FMT_SBGGR10P,
		},
	};
unlock:
	mutex_unlock(&camss->media_dev.graph_mutex);
	return ret;
}

static struct native_buffer *native_take_buffer(struct native_camera *camera)
{
	struct native_buffer *buffer = NULL;
	unsigned long flags;

	spin_lock_irqsave(&camera->pending_lock, flags);
	if (!list_empty(&camera->pending)) {
		buffer = list_first_entry(&camera->pending, struct native_buffer, entry);
		list_del(&buffer->entry);
	}
	spin_unlock_irqrestore(&camera->pending_lock, flags);
	return buffer;
}

static void native_flush(struct native_camera *camera, enum vb2_buffer_state state)
{
	struct native_buffer *buffer;

	while ((buffer = native_take_buffer(camera)))
		vb2_buffer_done(&buffer->vb.vb2_buf, state);
}

static u32 native_ae_factor(u32 mean, u32 clipped, u32 samples)
{
	u32 target = 112 * 1024;
	u32 signal;
	u32 target_signal;

	if (!samples)
		return 0;
	if ((u64)clipped * 100 > (u64)samples * 6 && mean > 80 * 1024)
		target = min(target, mean * 85 / 100);
	else if ((u64)clipped * 100 > (u64)samples * 3 && mean > 80 * 1024)
		target = min(target, mean);
	if (mean >= target - 8 * 1024 && mean <= target + 8 * 1024)
		return 0;
	signal = max_t(s32, (s32)mean - 16 * 1024, 1024);
	target_signal = max_t(s32, (s32)target - 16 * 1024, 1024);
	return clamp_t(u32, div64_u64((u64)target_signal * 65536, signal), 32768, 131072);
}

static s32 native_awb_gain(s32 channel, s32 green, s32 gain)
{
	s64 step;

	if (channel <= 1024 || green <= 1024)
		return gain;
	step = 65536 + div_s64(((s64)green - channel) * 65536, 4 * channel);
	step = clamp_t(s64, step, 62259, 68813);
	return clamp_t(s32, DIV_ROUND_CLOSEST_ULL(gain * step, 65536),
		QCOM_CAMERA_ICP_BPS_BALANCE_MIN_Q10, QCOM_CAMERA_ICP_BPS_BALANCE_MAX_Q10);
}

static int native_update_exposure(struct native_camera *camera, const u8 *pixels)
{
	struct v4l2_ctrl *exposure = camera->exposure;
	struct v4l2_ctrl *gain = camera->gain;
	u64 sum = 0;
	u64 product;
	u32 clipped = 0;
	u32 samples = 0;
	u32 factor;
	u32 row;
	u32 column;
	s32 next_exposure;
	s32 next_gain;
	int ret;

	for (row = 8; row < camera->format.height; row += 16) {
		for (column = 8; column < camera->format.width; column += 16) {
			u8 value = pixels[(size_t)row * camera->isp_stride + column];

			sum += value;
			clipped += value >= 250;
			samples++;
		}
	}
	if (!samples)
		return 0;
	factor = native_ae_factor(div64_u64(sum * 1024, samples), clipped, samples);
	if (!factor)
		return 0;
	v4l2_ctrl_lock(exposure);
	product = (u64)exposure->val * gain->val * factor;
	if (factor > 65536) {
		next_exposure = clamp_t(s32,
			DIV_ROUND_CLOSEST_ULL((u64)exposure->val * factor, 65536),
			exposure->minimum, exposure->maximum);
		next_gain = clamp_t(s32,
			DIV_ROUND_CLOSEST_ULL(product, (u64)next_exposure * 65536),
			gain->minimum, gain->maximum);
	} else {
		next_gain = clamp_t(s32,
			DIV_ROUND_CLOSEST_ULL((u64)gain->val * factor, 65536),
			gain->minimum, gain->maximum);
		next_exposure = clamp_t(s32,
			DIV_ROUND_CLOSEST_ULL(product, (u64)next_gain * 65536),
			exposure->minimum, exposure->maximum);
	}
	ret = __v4l2_ctrl_s_ctrl(exposure, next_exposure);
	if (!ret)
		ret = __v4l2_ctrl_s_ctrl(gain, next_gain);
	v4l2_ctrl_unlock(exposure);
	return ret;
}

static int native_update_balance(struct native_camera *camera, const u8 *pixels)
{
	const u8 *chroma = pixels + (size_t)camera->isp_stride * camera->format.height;
	u64 luma_sum = 0;
	u64 blue_sum = 0;
	u64 red_sum = 0;
	u64 weights = 0;
	u32 samples = 0;
	u32 row;
	u32 column;
	s32 mean_luma;
	s32 blue_delta;
	s32 red_delta;
	s32 red;
	s32 green;
	s32 blue;
	int ret = 0;

	for (row = 4; row < camera->format.height / 2; row += 8) {
		for (column = 4; column < camera->format.width / 2; column += 8) {
			u32 luma = pixels[(size_t)row * 2 * camera->isp_stride + column * 2];
			u32 blue_value = chroma[(size_t)row * camera->isp_stride + column * 2];
			u32 red_value = chroma[(size_t)row * camera->isp_stride + column * 2 + 1];
			u32 distance = max(abs((s32)blue_value - 128), abs((s32)red_value - 128));
			u32 weight;

			if (luma < 32 || luma > 224 || distance > 48)
				continue;
			weight = 49 - distance;
			luma_sum += luma * weight;
			blue_sum += blue_value * weight;
			red_sum += red_value * weight;
			weights += weight;
			samples++;
		}
	}
	if (samples < 64 || !weights)
		return 0;
	mean_luma = div64_u64(luma_sum * 1024, weights);
	blue_delta = (s32)div64_u64(blue_sum * 1024, weights) - 128 * 1024;
	red_delta = (s32)div64_u64(red_sum * 1024, weights) - 128 * 1024;
	if (abs(blue_delta) <= 1536 && abs(red_delta) <= 1536)
		return 0;
	red = mean_luma + div_s64(1402LL * red_delta, 1000);
	green = mean_luma - div_s64(344136LL * blue_delta + 714136LL * red_delta, 1000000);
	blue = mean_luma + div_s64(1772LL * blue_delta, 1000);
	if (red <= 1024 || green <= 1024 || blue <= 1024)
		return 0;
	v4l2_ctrl_lock(camera->red_balance);
	if (camera->auto_balance->val) {
		ret = __v4l2_ctrl_s_ctrl(camera->red_balance,
			native_awb_gain(red, green, camera->red_balance->val));
		if (!ret)
			ret = __v4l2_ctrl_s_ctrl(camera->blue_balance,
				native_awb_gain(blue, green, camera->blue_balance->val));
	}
	v4l2_ctrl_unlock(camera->red_balance);
	return ret;
}

static u64 native_reverse_word(u64 value, u32 unit)
{
	value = swab64(value);
	if (unit == 2)
		value = ((value & 0x00ff00ff00ff00ffULL) << 8) |
			((value & 0xff00ff00ff00ff00ULL) >> 8);
	return value;
}

static void native_rotate_plane(u8 *pixels, u32 width, u32 height,
				u32 stride, u32 unit)
{
	u32 row;
	u32 column;
	u32 component;

	for (row = 0; row < height / 2; row++) {
		u8 *top = pixels + (size_t)row * stride;
		u8 *bottom = pixels + (size_t)(height - 1 - row) * stride;

		for (column = 0; column + sizeof(u64) <= width; column += sizeof(u64)) {
			u64 top_word = get_unaligned_le64(top + column);
			u64 bottom_word = get_unaligned_le64(bottom + width - sizeof(u64) - column);

			put_unaligned_le64(native_reverse_word(bottom_word, unit), top + column);
			put_unaligned_le64(native_reverse_word(top_word, unit),
					   bottom + width - sizeof(u64) - column);
		}
		for (; column < width; column += unit) {
			for (component = 0; component < unit; component++) {
				u8 saved = top[column + component];

				top[column + component] = bottom[width - unit - column + component];
				bottom[width - unit - column + component] = saved;
			}
		}
	}
	if (height & 1) {
		u8 *middle = pixels + (size_t)(height / 2) * stride;

		for (column = 0; column < width / 2; column += unit) {
			for (component = 0; component < unit; component++) {
				u8 saved = middle[column + component];

				middle[column + component] = middle[width - unit - column + component];
				middle[width - unit - column + component] = saved;
			}
		}
	}
}

static int native_process_output(struct native_camera *camera, struct vb2_buffer *buffer)
{
	struct sg_table *sgt;
	u8 *pixels = camera->isp_pixels;
	u8 *output;
	u32 row;
	bool balance = v4l2_ctrl_g_ctrl(camera->auto_balance);
	bool exposure = v4l2_ctrl_g_ctrl(camera->auto_exposure) == V4L2_EXPOSURE_AUTO;
	bool rotate = v4l2_ctrl_g_ctrl(camera->rotation) == 180;
	bool update = false;
	int ret = 0;

	if (balance || exposure) {
		if (camera->auto_frames_until_update)
			camera->auto_frames_until_update--;
		else {
			camera->auto_frames_until_update = 8;
			update = true;
		}
	}
	output = vb2_plane_vaddr(buffer, 0);
	if (!output)
		return -ENOMEM;
	sgt = vb2_dma_sg_plane_desc(buffer, 0);
	dma_sync_sgtable_for_cpu(camera->owner->dma_dev, camera->isp_buffer, DMA_BIDIRECTIONAL);
	invalidate_kernel_vmap_range(pixels, camera->isp_size);
	if (update && exposure)
		ret = native_update_exposure(camera, pixels);
	if (!ret && update && balance)
		ret = native_update_balance(camera, pixels);
	if (!ret && rotate) {
		native_rotate_plane(pixels, camera->format.width, camera->format.height,
				    camera->isp_stride, 1);
		native_rotate_plane(pixels + (size_t)camera->isp_stride * camera->format.height,
				    camera->format.width, camera->format.height / 2,
				    camera->isp_stride, 2);
	}
	if (!ret) {
		dma_sync_sgtable_for_cpu(camera->owner->dma_dev, sgt, camera->queue.dma_dir);
		invalidate_kernel_vmap_range(output, camera->format.sizeimage);
		for (row = 0; row < camera->format.height * 3 / 2; row++)
			memcpy(output + (size_t)row * camera->format.bytesperline,
			       pixels + (size_t)row * camera->isp_stride, camera->format.width);
		flush_kernel_vmap_range(output, camera->format.sizeimage);
		dma_sync_sgtable_for_device(camera->owner->dma_dev, sgt, camera->queue.dma_dir);
	}
	flush_kernel_vmap_range(pixels, camera->isp_size);
	dma_sync_sgtable_for_device(camera->owner->dma_dev, camera->isp_buffer, DMA_BIDIRECTIONAL);
	return ret;
}

static int native_capture_thread(void *data)
{
	struct native_camera *camera = data;
	struct qcom_camera_icp_bps *bps = camera->owner->bps;
	unsigned long deadline = jiffies + msecs_to_jiffies(2000);

	while (!kthread_should_stop()) {
		struct qcom_camera_icp_bps_frame frame = {};
		struct camss_capture_frame raw;
		struct native_buffer *buffer;
		int ret;
		int requeue_ret;

		ret = camss_video_capture_next(camera->capture, &raw);
		if (ret == -EAGAIN && time_before(jiffies, deadline)) {
			usleep_range(500, 1000);
			continue;
		}
		if (ret) {
			vb2_queue_error(&camera->queue);
			break;
		}
		deadline = jiffies + msecs_to_jiffies(2000);
		buffer = native_take_buffer(camera);
		if (buffer) {
			frame.planes[0] = raw.sgt;
			frame.planes[1] = camera->isp_buffer;
			frame.planes[2] = frame.planes[1];
			frame.source_devs[0] = raw.dev;
			frame.source_devs[1] = camera->owner->dma_dev;
			frame.source_devs[2] = camera->owner->dma_dev;
			frame.size[0] = PAGE_ALIGN(raw.size);
			frame.size[1] = camera->isp_size;
			frame.size[2] = frame.size[1];
			frame.offset[2] = camera->isp_stride * camera->format.height;
			frame.request_id = ++camera->request_id;
			v4l2_ctrl_lock(camera->red_balance);
			frame.red_balance_q10 = camera->red_balance->val;
			frame.blue_balance_q10 = camera->blue_balance->val;
			v4l2_ctrl_unlock(camera->red_balance);
			dma_sync_sgtable_for_device(raw.dev, raw.sgt, DMA_FROM_DEVICE);
			ret = raw.error ? -EIO : bps->ops->client_process(camera->client, &frame);
			if (!ret)
				ret = native_process_output(camera, &buffer->vb.vb2_buf);
			buffer->vb.vb2_buf.timestamp = raw.timestamp;
			buffer->vb.sequence = raw.sequence;
			buffer->vb.field = V4L2_FIELD_NONE;
		}
		requeue_ret = camss_video_capture_requeue(camera->capture, raw.index);
		if (buffer)
			vb2_buffer_done(&buffer->vb.vb2_buf,
				ret || requeue_ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
		if (requeue_ret) {
			vb2_queue_error(&camera->queue);
			break;
		}
	}
	return 0;
}

static int native_queue_setup(struct vb2_queue *queue, unsigned int *count,
			      unsigned int *planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct native_camera *camera = vb2_get_drv_priv(queue);

	if (*planes)
		return *planes == 1 && sizes[0] >= camera->format.sizeimage ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = camera->format.sizeimage;
	return 0;
}

static int native_buffer_prepare(struct vb2_buffer *buffer)
{
	struct native_camera *camera = vb2_get_drv_priv(buffer->vb2_queue);

	if (vb2_plane_size(buffer, 0) < camera->format.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(buffer, 0, camera->format.sizeimage);
	return 0;
}

static void native_buffer_queue(struct vb2_buffer *buffer)
{
	struct native_camera *camera = vb2_get_drv_priv(buffer->vb2_queue);
	struct native_buffer *pending = container_of(to_vb2_v4l2_buffer(buffer),
						    struct native_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&camera->pending_lock, flags);
	list_add_tail(&pending->entry, &camera->pending);
	spin_unlock_irqrestore(&camera->pending_lock, flags);
}

static int native_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct native_camera *camera = vb2_get_drv_priv(queue);
	struct qcom_camera_icp_bps *bps = camera->owner->bps;
	struct qcom_camera_icp_bps_config config;
	struct v4l2_format raw_format;
	int ret;

	camera->auto_frames_until_update = 0;
	ret = native_configure(camera, &raw_format);
	if (ret)
		goto flush;
	camera->capture = camss_video_capture_start(camera->raw_video, &raw_format);
	if (IS_ERR(camera->capture)) {
		ret = PTR_ERR(camera->capture);
		camera->capture = NULL;
		goto flush;
	}
	camera->isp_stride = ALIGN(camera->format.width, 128);
	camera->isp_size = PAGE_ALIGN((size_t)camera->isp_stride * camera->format.height * 3 / 2);
	camera->isp_buffer = dma_alloc_noncontiguous(camera->owner->dma_dev, camera->isp_size,
		DMA_BIDIRECTIONAL, GFP_KERNEL, 0);
	if (!camera->isp_buffer) {
		ret = -ENOMEM;
		goto stop_raw;
	}
	camera->isp_pixels = dma_vmap_noncontiguous(camera->owner->dma_dev, camera->isp_size,
		camera->isp_buffer);
	if (!camera->isp_pixels) {
		ret = -ENOMEM;
		goto free_isp;
	}
	config = (struct qcom_camera_icp_bps_config) {
		.width = camera->format.width,
		.height = camera->format.height,
		.input_stride = raw_format.fmt.pix_mp.plane_fmt[0].bytesperline,
		.output_stride = camera->isp_stride,
	};
	camera->client = bps->ops->client_open(bps, &config);
	if (IS_ERR(camera->client)) {
		ret = PTR_ERR(camera->client);
		camera->client = NULL;
		goto free_isp;
	}
	camera->worker = kthread_run(native_capture_thread, camera, "caihong-cam%u", camera->route);
	if (IS_ERR(camera->worker)) {
		ret = PTR_ERR(camera->worker);
		camera->worker = NULL;
		bps->ops->client_close(camera->client);
		camera->client = NULL;
		goto free_isp;
	}
	return 0;

free_isp:
	if (camera->isp_pixels)
		dma_vunmap_noncontiguous(camera->owner->dma_dev, camera->isp_pixels);
	camera->isp_pixels = NULL;
	dma_free_noncontiguous(camera->owner->dma_dev, camera->isp_size,
		camera->isp_buffer, DMA_BIDIRECTIONAL);
	camera->isp_buffer = NULL;
stop_raw:
	camss_video_capture_stop(camera->capture);
	camera->capture = NULL;
flush:
	native_flush(camera, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void native_stop_streaming(struct vb2_queue *queue)
{
	struct native_camera *camera = vb2_get_drv_priv(queue);

	if (camera->worker) {
		kthread_stop(camera->worker);
		camera->worker = NULL;
	}
	if (camera->capture) {
		camss_video_capture_stop(camera->capture);
		camera->capture = NULL;
	}
	if (camera->client) {
		camera->owner->bps->ops->client_close(camera->client);
		camera->client = NULL;
	}
	if (camera->isp_buffer) {
		dma_vunmap_noncontiguous(camera->owner->dma_dev, camera->isp_pixels);
		dma_free_noncontiguous(camera->owner->dma_dev, camera->isp_size,
			camera->isp_buffer, DMA_BIDIRECTIONAL);
		camera->isp_pixels = NULL;
		camera->isp_buffer = NULL;
	}
	native_flush(camera, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops native_queue_ops = {
	.queue_setup = native_queue_setup,
	.buf_prepare = native_buffer_prepare,
	.buf_queue = native_buffer_queue,
	.start_streaming = native_start_streaming,
	.stop_streaming = native_stop_streaming,
};

static int native_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct native_camera *camera = video_drvdata(file);

	strscpy(cap->driver, "caihong-native", sizeof(cap->driver));
	strscpy(cap->card, video_devdata(file)->name, sizeof(cap->card));
	strscpy(cap->bus_info, camera->route ? "platform:caihong-front" :
		"platform:caihong-rear", sizeof(cap->bus_info));
	return 0;
}

static int native_enum_format(struct file *file, void *priv, struct v4l2_fmtdesc *format)
{
	if (format->index)
		return -EINVAL;
	format->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int native_enum_size(struct file *file, void *priv, struct v4l2_frmsizeenum *size)
{
	struct native_camera *camera = video_drvdata(file);

	if (size->pixel_format != V4L2_PIX_FMT_NV12 || size->index >= ARRAY_SIZE(camera->modes))
		return -EINVAL;
	size->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	size->discrete.width = camera->modes[size->index].width;
	size->discrete.height = camera->modes[size->index].height;
	return 0;
}

static int native_enum_interval(struct file *file, void *priv, struct v4l2_frmivalenum *interval)
{
	struct native_camera *camera = video_drvdata(file);
	unsigned int index;

	if (interval->index || interval->pixel_format != V4L2_PIX_FMT_NV12)
		return -EINVAL;
	for (index = 0; index < ARRAY_SIZE(camera->modes); index++) {
		if (interval->width == camera->modes[index].width &&
		    interval->height == camera->modes[index].height) {
			interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			interval->discrete = (struct v4l2_fract) { 1, 30 };
			return 0;
		}
	}
	return -EINVAL;
}

static int native_get_format(struct file *file, void *priv, struct v4l2_format *format)
{
	struct native_camera *camera = video_drvdata(file);

	format->fmt.pix = camera->format;
	return 0;
}

static int native_try_format(struct file *file, void *priv, struct v4l2_format *format)
{
	native_fill_format(video_drvdata(file), &format->fmt.pix);
	return 0;
}

static int native_set_format(struct file *file, void *priv, struct v4l2_format *format)
{
	struct native_camera *camera = video_drvdata(file);

	if (vb2_is_busy(&camera->queue))
		return -EBUSY;
	native_fill_format(camera, &format->fmt.pix);
	camera->format = format->fmt.pix;
	return 0;
}

static int native_get_parm(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(&parm->parm.capture, 0, sizeof(parm->parm.capture));
	parm->parm.capture.timeperframe = (struct v4l2_fract) { 1, 30 };
	return 0;
}

static int native_enum_input(struct file *file, void *priv, struct v4l2_input *input)
{
	if (input->index)
		return -EINVAL;
	input->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(input->name, "Camera", sizeof(input->name));
	return 0;
}

static int native_get_input(struct file *file, void *priv, unsigned int *input)
{
	*input = 0;
	return 0;
}

static int native_set_input(struct file *file, void *priv, unsigned int input)
{
	return input ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops native_ioctl_ops = {
	.vidioc_querycap = native_querycap,
	.vidioc_enum_fmt_vid_cap = native_enum_format,
	.vidioc_enum_framesizes = native_enum_size,
	.vidioc_enum_frameintervals = native_enum_interval,
	.vidioc_g_fmt_vid_cap = native_get_format,
	.vidioc_try_fmt_vid_cap = native_try_format,
	.vidioc_s_fmt_vid_cap = native_set_format,
	.vidioc_g_parm = native_get_parm,
	.vidioc_s_parm = native_get_parm,
	.vidioc_enum_input = native_enum_input,
	.vidioc_g_input = native_get_input,
	.vidioc_s_input = native_set_input,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations native_file_ops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static void native_release(struct v4l2_device *v4l2)
{
	struct native_device *native = container_of(v4l2, struct native_device, v4l2);
	unsigned int index;

	for (index = 0; index < native->initialized; index++) {
		v4l2_ctrl_handler_free(&native->cameras[index].controls);
		mutex_destroy(&native->cameras[index].lock);
	}
	kfree(native);
}

static int native_register_camera(struct native_device *native, unsigned int index)
{
	struct native_camera *camera = &native->cameras[index];
	struct vb2_queue *queue = &camera->queue;
	struct video_device *vdev = &camera->vdev;
	int ret;

	camera->owner = native;
	camera->route = index ? 0 : 1;
	camera->raw_video = &native->camss->vfe[camera->route].line[VFE_LINE_RDI0].video_out;
	camera->modes[0] = index ? (struct native_mode) { 4208, 3120 } :
				  (struct native_mode) { 3264, 2448 };
	camera->modes[1] = index ? (struct native_mode) { 2104, 1560 } :
				  (struct native_mode) { 1632, 1224 };
	camera->format.width = camera->modes[0].width;
	camera->format.height = camera->modes[0].height;
	native_fill_format(camera, &camera->format);
	mutex_init(&camera->lock);
	spin_lock_init(&camera->pending_lock);
	INIT_LIST_HEAD(&camera->pending);
	v4l2_ctrl_handler_init(&camera->controls, 8);
	native->initialized++;
	camera->red_balance = v4l2_ctrl_new_std(&camera->controls, NULL, V4L2_CID_RED_BALANCE,
		QCOM_CAMERA_ICP_BPS_BALANCE_MIN_Q10, QCOM_CAMERA_ICP_BPS_BALANCE_MAX_Q10,
		1, QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10);
	camera->blue_balance = v4l2_ctrl_new_std(&camera->controls, NULL, V4L2_CID_BLUE_BALANCE,
		QCOM_CAMERA_ICP_BPS_BALANCE_MIN_Q10, QCOM_CAMERA_ICP_BPS_BALANCE_MAX_Q10,
		1, QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10);
	camera->auto_balance = v4l2_ctrl_new_std(&camera->controls, NULL,
		V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	camera->auto_exposure = v4l2_ctrl_new_std_menu(&camera->controls, NULL,
		V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	camera->rotation = v4l2_ctrl_new_std(&camera->controls, NULL,
		V4L2_CID_ROTATE, 0, 180, 180, 180);
	camera->exposure = v4l2_ctrl_find(camera->sensor->ctrl_handler, V4L2_CID_EXPOSURE);
	camera->gain = v4l2_ctrl_find(camera->sensor->ctrl_handler, V4L2_CID_ANALOGUE_GAIN);
	if (!camera->exposure || !camera->gain || camera->exposure->handler != camera->gain->handler)
		return -EINVAL;
	ret = v4l2_ctrl_add_handler(&camera->controls, camera->sensor->ctrl_handler, NULL, false);
	if (ret)
		return ret;
	if (camera->controls.error)
		return camera->controls.error;
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP;
	queue->drv_priv = camera;
	queue->buf_struct_size = sizeof(struct native_buffer);
	queue->ops = &native_queue_ops;
	queue->mem_ops = &vb2_dma_sg_memops;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &camera->lock;
	queue->dev = native->dma_dev;
	queue->bidirectional = true;
	queue->min_queued_buffers = 2;
	ret = vb2_queue_init(queue);
	if (ret)
		return ret;
	strscpy(vdev->name, index ? "Caihong Rear Native (experimental)" :
		"Caihong Front Native (experimental)", sizeof(vdev->name));
	vdev->v4l2_dev = &native->v4l2;
	vdev->fops = &native_file_ops;
	vdev->ioctl_ops = &native_ioctl_ops;
	vdev->lock = &camera->lock;
	vdev->release = video_device_release_empty;
	vdev->queue = queue;
	vdev->ctrl_handler = &camera->controls;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, camera);
	return video_register_device(vdev, VFL_TYPE_VIDEO, -1);
}

static int native_probe(struct auxiliary_device *auxdev, const struct auxiliary_device_id *id)
{
	static const char * const sensors[] = { "smartsens,sc820cs", "smartsens,sc1320cs" };
	struct device_node *camss_node;
	struct native_device *native;
	struct device *camss_dev;
	struct v4l2_subdev *subdev;
	unsigned int index;
	int ret;

	if (!enable || !of_machine_is_compatible("oneplus,caihong"))
		return -ENODEV;
	camss_node = of_find_compatible_node(NULL, NULL, "qcom,sm8650-camss");
	if (!camss_node)
		return -ENODEV;
	camss_dev = bus_find_device_by_of_node(&platform_bus_type, camss_node);
	of_node_put(camss_node);
	if (!camss_dev)
		return -EPROBE_DEFER;
	if (!device_link_add(&auxdev->dev, camss_dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(camss_dev);
		return -ENOMEM;
	}
	native = kzalloc_obj(*native);
	if (!native) {
		put_device(camss_dev);
		return -ENOMEM;
	}
	device_lock(camss_dev);
	native->camss = dev_get_drvdata(camss_dev);
	if (!camss_dev->driver || !native->camss ||
	    native->camss->res->version != CAMSS_8650 ||
	    native->camss->res->vfe_num < 2 || native->camss->res->csid_num < 2) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	native->bps = dev_get_platdata(&auxdev->dev);
	native->dma_dev = auxdev->dev.parent;
	if (!native->bps || !native->bps->ops) {
		ret = -EINVAL;
		goto unlock;
	}
	mutex_lock(&native->camss->media_dev.graph_mutex);
	list_for_each_entry(subdev, &native->camss->v4l2_dev.subdevs, list) {
		for (index = 0; index < ARRAY_SIZE(sensors); index++) {
			if (subdev->host_priv && subdev->dev &&
			    of_device_is_compatible(subdev->dev->of_node, sensors[index]))
				native->cameras[index].sensor = subdev;
		}
	}
	mutex_unlock(&native->camss->media_dev.graph_mutex);
	for (index = 0; index < ARRAY_SIZE(sensors); index++) {
		if (!native->cameras[index].sensor) {
			ret = -EPROBE_DEFER;
			goto unlock;
		}
		if (!device_link_add(&auxdev->dev, native->cameras[index].sensor->dev,
				     DL_FLAG_AUTOREMOVE_CONSUMER)) {
			ret = -ENOMEM;
			goto unlock;
		}
	}
	device_unlock(camss_dev);
	put_device(camss_dev);
	ret = v4l2_device_register(&auxdev->dev, &native->v4l2);
	if (ret)
		goto free_native;
	native->v4l2.release = native_release;
	for (index = 0; index < ARRAY_SIZE(sensors); index++) {
		ret = native_register_camera(native, index);
		if (ret)
			goto unregister;
	}
	auxiliary_set_drvdata(auxdev, native);
	return 0;

unregister:
	for (index = 0; index < ARRAY_SIZE(sensors); index++)
		vb2_video_unregister_device(&native->cameras[index].vdev);
	v4l2_device_unregister(&native->v4l2);
	v4l2_device_put(&native->v4l2);
	return ret;
unlock:
	device_unlock(camss_dev);
	put_device(camss_dev);
free_native:
	kfree(native);
	return ret;
}

static void native_remove(struct auxiliary_device *auxdev)
{
	struct native_device *native = auxiliary_get_drvdata(auxdev);
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(native->cameras); index++)
		vb2_video_unregister_device(&native->cameras[index].vdev);
	v4l2_device_unregister(&native->v4l2);
	v4l2_device_put(&native->v4l2);
}

static const struct auxiliary_device_id native_ids[] = {
	{ .name = "qcom_camera_icp.capture" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, native_ids);

static struct auxiliary_driver native_driver = {
	.probe = native_probe,
	.remove = native_remove,
	.id_table = native_ids,
	.driver.name = "caihong-camera-native",
};
module_auxiliary_driver(native_driver);

MODULE_DESCRIPTION("Experimental Caihong CAMSS and BPS native V4L2 capture");
MODULE_LICENSE("GPL");
