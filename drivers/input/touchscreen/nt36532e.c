// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/*
 * Novatek NT36532E no-flash SPI touchscreen/pen driver.
 * Standalone mainline-style port; no Oplus touchpanel framework dependency.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <drm/drm_panel.h>

#define NVT_MAX_TOUCHES 10
#define NVT_POINT_DATA_LEN 120
#define NVT_POINT_CSUM_INDEX 65
#define NVT_PEN_DATA_OFFSET 66
#define NVT_PEN_DATA_LEN 14
#define NVT_XFER_LEN 4096
#define NVT_MAX_PARTITIONS 64
#define NVT_SECTOR_SIZE 4096

#define NVT_EVENT_BUF_ADDR 0x125800
#define NVT_CHIP_VER_TRIM_ADDR 0x1fb104
#define NVT_SWRST_SIF_ADDR 0x1fb43e
#define NVT_ENB_CASC_ADDR 0x1fb12c
#define NVT_BOOT_RDY_ADDR 0x1fb50d
#define NVT_TX_AUTO_COPY_EN 0x1fc925
#define NVT_ILM_LENGTH_ADDR 0x1fb518
#define NVT_DLM_LENGTH_ADDR 0x1fb530
#define NVT_ILM_DES_ADDR 0x1fb528
#define NVT_DLM_DES_ADDR 0x1fb52c
#define NVT_G_ILM_CSUM_ADDR 0x1fb500
#define NVT_G_DLM_CSUM_ADDR 0x1fb504
#define NVT_EVENT_HOST_CMD 0x50
#define NVT_EVENT_RESET_COMPLETE 0x60
#define NVT_EVENT_FWINFO 0x78
#define NVT_EVENTBUF_PROT_HIGH_RESO 0xf1
#define NVT_RESET_STATE_INIT 0xa0
#define NVT_RESET_STATE_MAX 0xaf
#define NVT_CMD_SLEEP 0x11
#define NVT_EXT_CMD 0x7f

struct nvt_partition {
	u32 bin, sram, size, crc;
};

struct nt36532e {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
	struct input_dev *input, *pen;
	struct touchscreen_properties prop, pen_prop;
	struct mutex lock;
	struct drm_panel_follower panel_follower;
	struct work_struct resume_work;
	const struct firmware *fw;
	bool panel_ready, suspended, detected;
	u64 starts, start_failures, stops;
	int start_error, sleep_error;
	const char *fw_name;
	u32 max_x, max_y, max_pressure;
	u32 pen_max_pressure, pen_max_tilt, pen_x_mm, pen_y_mm;
	bool pen_support, high_res, cascade;
	bool irq_enabled;
	u8 fw_version, event_protocol;
	u8 last_event[NVT_POINT_DATA_LEN + 1];
	u64 irq_count, event_reads, touch_frames, touch_contacts;
	u64 spi_errors, checksum_errors, out_of_range, boot_events;
	u64 pen_packets, pen_reports, pen_checksum_errors, pen_out_of_range;
	u64 pen_unknown_formats;
	u16 pen_x, pen_y, pen_pressure, pen_distance;
	u8 pen_format, pen_buttons;
	bool pen_in_range, pen_contact;
	int pen_scan_type, pen_command_error;
	int last_error;
	u8 *tx, *rx;
	size_t xfer_size;
};

static void nvt_hw_reset(struct nt36532e *ts)
{
	if (!ts->reset_gpio)
		return;

	/* reset-gpios is active-low: logical 1 asserts reset. */
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(20);
}

static u32 nvt_le32(const u8 *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
	       ((u32)p[3] << 24);
}

static int nvt_spi_write(struct nt36532e *ts, const u8 *src, size_t len)
{
	if (!len || len > ts->xfer_size)
		return -EINVAL;
	memcpy(ts->tx, src, len);
	ts->tx[0] |= 0x80;
	return spi_write(ts->spi, ts->tx, len);
}

static int nvt_spi_read(struct nt36532e *ts, u8 *buf, size_t len)
{
	struct spi_transfer xfer = {};
	int ret;

	if (len < 2 || len + 1 > ts->xfer_size)
		return -EINVAL;
	memset(ts->tx, 0, len + 1);
	memset(ts->rx, 0, len + 1);
	memcpy(ts->tx, buf, len);
	ts->tx[0] &= 0x7f;
	xfer.tx_buf = ts->tx;
	xfer.rx_buf = ts->rx;
	xfer.len = len + 1;
	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	if (!ret)
		memcpy(buf + 1, ts->rx + 2, len - 1);
	return ret;
}

static int nvt_set_page(struct nt36532e *ts, u32 addr)
{
	u8 b[3] = { 0xff, (addr >> 15) & 0xff, (addr >> 7) & 0xff };

	return nvt_spi_write(ts, b, sizeof(b));
}

static int nvt_write_addr(struct nt36532e *ts, u32 addr, u8 val)
{
	u8 b[2] = { addr & 0x7f, val };
	int ret = nvt_set_page(ts, addr);

	return ret ? ret : nvt_spi_write(ts, b, sizeof(b));
}

static int nvt_read_addr(struct nt36532e *ts, u32 addr, u8 *val)
{
	u8 b[2] = { addr & 0x7f, 0 };
	int ret = nvt_set_page(ts, addr);

	if (!ret)
		ret = nvt_spi_read(ts, b, sizeof(b));
	if (!ret)
		*val = b[1];
	return ret;
}

static int nvt_detect(struct nt36532e *ts)
{
	u8 id[7] = {}, casc;
	int try, ret;

	for (try = 0; try < 5; try++) {
		ret = nvt_write_addr(ts, NVT_SWRST_SIF_ADDR, 0x69);
		if (ret)
			continue;
		usleep_range(5000, 6000);
		ret = nvt_set_page(ts, NVT_CHIP_VER_TRIM_ADDR);
		if (ret)
			continue;
		id[0] = NVT_CHIP_VER_TRIM_ADDR & 0x7f;
		ret = nvt_spi_write(ts, id, sizeof(id));
		if (ret)
			continue;
		memset(id, 0, sizeof(id));
		id[0] = NVT_CHIP_VER_TRIM_ADDR & 0x7f;
		ret = nvt_spi_read(ts, id, sizeof(id));
		if (!ret && id[4] == 0x32 && id[5] == 0x65 && id[6] == 0x03)
			break;
		usleep_range(10000, 11000);
	}
	if (try == 5)
		return dev_err_probe(&ts->spi->dev, -ENODEV,
				     "NT36532E trim ID not found\n");
	ret = nvt_read_addr(ts, NVT_ENB_CASC_ADDR, &casc);
	if (ret)
		return ret;
	ts->cascade = !(casc & BIT(0));
	dev_info(&ts->spi->dev, "NT36532E %s detected\n",
		 ts->cascade ? "cascade" : "single");
	return 0;
}

static int nvt_fw_needed_size(const struct firmware *fw, size_t *needed)
{
	int s;

	for (s = fw->size / NVT_SECTOR_SIZE; s > 0; s--) {
		size_t off = s * NVT_SECTOR_SIZE - 3;

		if (!memcmp(fw->data + off, "NVT", 3) ||
		    !memcmp(fw->data + off, "MOD", 3)) {
			*needed = s * NVT_SECTOR_SIZE;
			return 0;
		}
	}
	return -EINVAL;
}

static int nvt_parse_fw(const struct firmware *fw, struct nvt_partition **out,
			unsigned int *out_count, bool *second_header)
{
	struct nvt_partition *p;
	u32 end, pos;
	unsigned int info = 0, overlay = 0, count, i;
	bool second, found_header = false;

	if (fw->size < 0x40)
		return -EINVAL;
	end = nvt_le32(fw->data);
	if (end < 0x30 || end > fw->size)
		return -EINVAL;
	second = !!(fw->data[0x20] & BIT(1));
	*second_header = second;
	if (second) {
		for (pos = 0x30; pos < end / 2; pos += 0x10)
			info++;
		info++;
	} else {
		for (pos = 0x30; pos < end; pos += 0x10)
			info++;
	}
	if (fw->data[0x28] & BIT(4))
		overlay = fw->data[0x28] & 0x0f;
	count = 2 + info + overlay;
	if (count > NVT_MAX_PARTITIONS)
		return -EINVAL;
	p = kcalloc(count, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	for (i = 0; i < 2; i++) {
		p[i].bin = nvt_le32(fw->data + i * 12);
		p[i].sram = nvt_le32(fw->data + i * 12 + 4);
		p[i].size = nvt_le32(fw->data + i * 12 + 8);
		p[i].crc = nvt_le32(fw->data + 0x18 + i * 4);
	}
	for (i = 0; i < info; i++) {
		unsigned int n = i + 2;
		u32 off = found_header && second ? end - 0x10 : 0x30 + i * 0x10;

		if ((u64)off + 0x10 > fw->size)
			goto bad;
		p[n].sram = nvt_le32(fw->data + off);
		p[n].size = nvt_le32(fw->data + off + 4);
		p[n].bin = nvt_le32(fw->data + off + 8);
		p[n].crc = nvt_le32(fw->data + off + 12);
		if (p[n].bin < end && p[n].size)
			found_header = true;
	}
	for (i = 0; i < overlay; i++) {
		unsigned int n = 2 + info + i;
		u32 off = p[1].bin + i * 0x10;

		if ((u64)off + 0x10 > fw->size)
			goto bad;
		p[n].sram = nvt_le32(fw->data + off);
		p[n].size = nvt_le32(fw->data + off + 4);
		p[n].bin = nvt_le32(fw->data + off + 8);
		p[n].crc = nvt_le32(fw->data + off + 12);
	}
	for (i = 0; i < count; i++)
		if (p[i].size && (u64)p[i].bin + p[i].size >= fw->size)
			goto bad;
	*out = p;
	*out_count = count;
	return 0;
bad:
	kfree(p);
	return -EINVAL;
}

static int nvt_write_sram(struct nt36532e *ts, const u8 *fw, u32 sram,
			  u32 bytes, u32 bin)
{
	int ret;

	while (bytes) {
		u32 len = min_t(u32, bytes, NVT_XFER_LEN);

		ret = nvt_set_page(ts, sram);
		if (ret)
			return ret;
		ts->tx[0] = (sram & 0x7f) | 0x80;
		memcpy(ts->tx + 1, fw + bin, len);
		ret = spi_write(ts->spi, ts->tx, len + 1);
		if (ret)
			return ret;
		sram += len;
		bin += len;
		bytes -= len;
	}
	return 0;
}

static int nvt_crc_bank(struct nt36532e *ts, u32 des, u32 sram, u32 length,
			u32 size, u32 golden, u32 crc)
{
	u8 b[5];
	int ret = nvt_set_page(ts, des);

	if (ret)
		return ret;
	b[0] = des & 0x7f;
	b[1] = sram;
	b[2] = sram >> 8;
	b[3] = sram >> 16;
	ret = nvt_spi_write(ts, b, 4);
	if (ret)
		return ret;
	b[0] = length & 0x7f;
	b[1] = size;
	b[2] = size >> 8;
	b[3] = size >> 16;
	ret = nvt_spi_write(ts, b, 4);
	if (ret)
		return ret;
	b[0] = golden & 0x7f;
	b[1] = crc;
	b[2] = crc >> 8;
	b[3] = crc >> 16;
	b[4] = crc >> 24;
	return nvt_spi_write(ts, b, 5);
}

static int nvt_wait_auto_copy(struct nt36532e *ts)
{
	u8 v;
	int i, ret;

	for (i = 0; i < 200; i++) {
		ret = nvt_read_addr(ts, NVT_TX_AUTO_COPY_EN, &v);
		if (ret)
			return ret;
		if (!v)
			return 0;
		usleep_range(1000, 2000);
	}
	dev_warn(&ts->spi->dev, "cascade auto-copy timeout: status=%02x\n", v);
	return -ETIMEDOUT;
}

static int nvt_fw_crc_enable(struct nt36532e *ts)
{
	u8 b[2];
	int ret = nvt_set_page(ts, NVT_EVENT_BUF_ADDR);

	if (ret)
		return ret;
	b[0] = NVT_EVENT_RESET_COMPLETE;
	b[1] = 0;
	ret = nvt_spi_write(ts, b, 2);
	if (ret)
		return ret;
	b[0] = NVT_EVENT_HOST_CMD;
	b[1] = 0xae;
	return nvt_spi_write(ts, b, 2);
}

static int nvt_wait_reset(struct nt36532e *ts)
{
	u8 b[6];
	int i, ret = nvt_set_page(ts, NVT_EVENT_BUF_ADDR |
					      NVT_EVENT_RESET_COMPLETE);
	if (ret)
		return ret;
	for (i = 0; i <= 10; i++) {
		memset(b, 0, sizeof(b));
		b[0] = NVT_EVENT_RESET_COMPLETE;
		ret = nvt_spi_read(ts, b, sizeof(b));
		if (ret)
			return ret;
		if (b[1] >= NVT_RESET_STATE_INIT && b[1] <= NVT_RESET_STATE_MAX)
			return 0;
		usleep_range(10000, 11000);
	}
	dev_warn(&ts->spi->dev, "firmware reset timeout: status=%5ph\n", b + 1);
	return -ETIMEDOUT;
}

static int nvt_download_fw(struct nt36532e *ts)
{
	const struct firmware *fw = ts->fw;
	struct nvt_partition *p = NULL;
	unsigned int count = 0, i;
	size_t need;
	bool second;
	int ret, attempt;

	ret = nvt_fw_needed_size(fw, &need);
	if (ret)
		goto out;
	if (need < NVT_SECTOR_SIZE ||
	    fw->data[need - NVT_SECTOR_SIZE] +
			    fw->data[need - NVT_SECTOR_SIZE + 1] !=
		    0xff) {
		ret = -ENOEXEC;
		goto out;
	}
	ret = nvt_parse_fw(fw, &p, &count, &second);
	if (ret)
		goto out;
	for (attempt = 0; attempt < 3; attempt++) {
		ret = nvt_write_addr(ts, NVT_SWRST_SIF_ADDR, 0x69);
		if (ret)
			continue;
		usleep_range(5000, 6000);
		ret = nvt_crc_bank(ts, NVT_ILM_DES_ADDR, p[0].sram,
				   NVT_ILM_LENGTH_ADDR, p[0].size,
				   NVT_G_ILM_CSUM_ADDR, p[0].crc);
		if (ret)
			continue;
		ret = nvt_crc_bank(ts, NVT_DLM_DES_ADDR, p[1].sram,
				   NVT_DLM_LENGTH_ADDR, p[1].size,
				   NVT_G_DLM_CSUM_ADDR, p[1].crc);
		if (ret)
			continue;
		if (second) {
			ret = nvt_write_addr(ts, NVT_TX_AUTO_COPY_EN, 0x56);
			if (ret)
				continue;
		}
		for (i = 0; i < count; i++) {
			if (!p[i].size)
				continue;
			ret = nvt_write_sram(ts, fw->data, p[i].sram,
					     p[i].size + 1, p[i].bin);
			if (ret)
				break;
		}
		if (ret)
			continue;
		if (second) {
			ret = nvt_wait_auto_copy(ts);
			if (ret)
				continue;
		}
		ret = nvt_fw_crc_enable(ts);
		if (ret)
			continue;
		ret = nvt_write_addr(ts, NVT_BOOT_RDY_ADDR, 1);
		if (ret)
			continue;
		usleep_range(5000, 6000);
		ret = nvt_wait_reset(ts);
		if (!ret)
			break;
	}
	if (!ret)
		dev_info(&ts->spi->dev, "loaded %s (%u partitions)\n",
			 ts->fw_name, count);
	else
		dev_err(&ts->spi->dev, "firmware download failed: %d\n", ret);
	kfree(p);
out:
	return ret;
}

static bool nvt_point_checksum(const u8 *d)
{
	u8 sum = 0, want;
	int i;

	for (i = 0; i < NVT_POINT_CSUM_INDEX - 1; i++)
		sum += d[i + 1];
	want = ~sum + 1;
	return want == d[NVT_POINT_CSUM_INDEX];
}

static bool nvt_pen_checksum(const u8 *d)
{
	u8 sum = 0, want;
	int i;

	for (i = 0; i < NVT_PEN_DATA_LEN - 1; i++)
		sum += d[NVT_PEN_DATA_OFFSET + i];
	want = ~sum + 1;
	return want == d[NVT_PEN_DATA_OFFSET + NVT_PEN_DATA_LEN - 1];
}

/* The vendor driver selects the packet layout from FWINFO byte 13, not
 * from the coordinate range in DT. Reading an event after download also
 * acknowledges a boot notification that may already have asserted IRQ.
 */
static int nvt_read_event(struct nt36532e *ts)
{
	int ret;

	memset(ts->last_event, 0, sizeof(ts->last_event));
	ret = nvt_set_page(ts, NVT_EVENT_BUF_ADDR);
	if (!ret)
		ret = nvt_spi_read(ts, ts->last_event, sizeof(ts->last_event));
	ts->last_error = ret;
	if (ret) {
		ts->spi_errors++;
		if (ts->spi_errors <= 3)
			dev_warn(&ts->spi->dev, "event read failed: %d\n", ret);
		return ret;
	}

	ts->event_reads++;
	return 0;
}

static int nvt_prepare_events(struct nt36532e *ts)
{
	u8 info[39];
	int ret, attempt;

	ret = nvt_read_event(ts);
	if (ret)
		return ret;

	for (attempt = 0; attempt < 4; attempt++) {
		ret = nvt_set_page(ts, NVT_EVENT_BUF_ADDR | NVT_EVENT_FWINFO);
		if (ret)
			return ret;
		memset(info, 0, sizeof(info));
		info[0] = NVT_EVENT_FWINFO;
		ret = nvt_spi_read(ts, info, sizeof(info));
		if (ret)
			return ret;
		if (info[1] + info[2] == 0xff)
			break;
		usleep_range(10000, 11000);
	}
	if (attempt == 4)
		return dev_err_probe(&ts->spi->dev, -EIO,
				     "invalid firmware info: %02x %02x\n",
				     info[1], info[2]);

	ts->fw_version = info[1];
	ts->event_protocol = info[13];
	ts->high_res = info[13] == NVT_EVENTBUF_PROT_HIGH_RESO;
	dev_info(&ts->spi->dev,
		 "FW %02x protocol=%02x (%s coordinates) PID=%02x%02x\n",
		 ts->fw_version, ts->event_protocol,
		 ts->high_res ? "16-bit" : "12-bit", info[36], info[35]);
	return 0;
}

static void nvt_pen_release(struct nt36532e *ts)
{
	if (!ts->pen)
		return;
	ts->pen_in_range = false;
	ts->pen_contact = false;
	ts->pen_pressure = 0;
	ts->pen_distance = 0;
	ts->pen_buttons = 0;
	input_report_abs(ts->pen, ABS_PRESSURE, 0);
	input_report_abs(ts->pen, ABS_DISTANCE, 0);
	input_report_abs(ts->pen, ABS_TILT_X, 0);
	input_report_abs(ts->pen, ABS_TILT_Y, 0);
	input_report_key(ts->pen, BTN_TOUCH, 0);
	input_report_key(ts->pen, BTN_TOOL_PEN, 0);
	input_report_key(ts->pen, BTN_STYLUS, 0);
	input_report_key(ts->pen, BTN_STYLUS2, 0);
	input_sync(ts->pen);
}

static void nvt_report_pen(struct nt36532e *ts, const u8 *d)
{
	u16 x, y, pressure, distance;
	int tilt_x, tilt_y;

	if (!ts->pen)
		return;
	ts->pen_packets++;
	ts->pen_format = d[66];
	if (!nvt_pen_checksum(d)) {
		ts->pen_checksum_errors++;
		return;
	}
	if (d[66] == 0xff || d[66] == 0xf0) {
		/* No pen, or an ID packet without coordinates (vendor behavior). */
		nvt_pen_release(ts);
		return;
	}
	if (d[66] != 0x01) {
		ts->pen_unknown_formats++;
		return;
	}
	x = ((u16)d[67] << 8) | d[68];
	y = ((u16)d[69] << 8) | d[70];
	pressure = ((u16)d[71] << 8) | d[72];
	distance = ((u16)d[75] << 8) | d[76];
	if (x >= ts->max_x || y >= ts->max_y) {
		ts->pen_out_of_range++;
		return;
	}
	tilt_x = clamp_t(int, (s8)d[73], -(int)ts->pen_max_tilt,
			 ts->pen_max_tilt);
	tilt_y = clamp_t(int, (s8)d[74], -(int)ts->pen_max_tilt,
			 ts->pen_max_tilt);
	if (ts->pen_prop.invert_x)
		tilt_x = -tilt_x;
	if (ts->pen_prop.invert_y)
		tilt_y = -tilt_y;
	if (ts->pen_prop.swap_x_y)
		swap(tilt_x, tilt_y);
	ts->pen_reports++;
	ts->pen_x = x;
	ts->pen_y = y;
	ts->pen_pressure = min_t(u32, pressure, ts->pen_max_pressure);
	ts->pen_distance = distance;
	ts->pen_buttons = d[77] & 3;
	ts->pen_in_range = pressure || distance;
	ts->pen_contact = pressure != 0;
	touchscreen_report_pos(ts->pen, &ts->pen_prop, x, y, false);
	input_report_abs(ts->pen, ABS_PRESSURE, ts->pen_pressure);
	input_report_abs(ts->pen, ABS_TILT_X, tilt_x);
	input_report_abs(ts->pen, ABS_TILT_Y, tilt_y);
	input_report_abs(ts->pen, ABS_DISTANCE, distance);
	input_report_key(ts->pen, BTN_TOOL_PEN, ts->pen_in_range);
	input_report_key(ts->pen, BTN_TOUCH, ts->pen_contact);
	input_report_key(ts->pen, BTN_STYLUS, !!(d[77] & BIT(0)));
	input_report_key(ts->pen, BTN_STYLUS2, !!(d[77] & BIT(1)));
	input_sync(ts->pen);
}

/* Vendor pencil_connect types: 0 off, 1 Havon, 2 Maxeye, 3 Maxeye 2nd,
 * 4 Sunwoda, 5 Maxeye 3rd. A retail model name is not this protocol ID.
 * Called with the device mutex held so IRQ reads cannot change the page.
 */
static int nvt_set_pen_scan(struct nt36532e *ts, unsigned int type)
{
	static const u8 commands[] = { 0x11, 0x10, 0x12, 0x13, 0x15, 0x18 };
	u8 buf[3] = { 0 };
	int ret, attempt;

	if (type >= ARRAY_SIZE(commands))
		return -EINVAL;
	ret = nvt_set_page(ts, NVT_EVENT_BUF_ADDR);
	if (ret)
		return ret;
	for (attempt = 0; attempt < 5; attempt++) {
		if (buf[1] != NVT_EXT_CMD) {
			buf[0] = NVT_EVENT_HOST_CMD;
			buf[1] = NVT_EXT_CMD;
			buf[2] = commands[type];
			ret = nvt_spi_write(ts, buf, sizeof(buf));
			if (ret)
				return ret;
		}
		msleep(20);
		buf[0] = NVT_EVENT_HOST_CMD;
		buf[1] = 0xff;
		ret = nvt_spi_read(ts, buf, sizeof(buf));
		if (ret)
			return ret;
		if (!buf[1])
			return 0;
	}
	return -ETIMEDOUT;
}

static void nvt_report_touch(struct nt36532e *ts, const u8 *d)
{
	int i;

	ts->touch_frames++;
	for (i = 0; i < NVT_MAX_TOUCHES; i++) {
		int pos = 1 + 6 * i;
		int id = (d[pos] >> 3) - 1;
		u8 state = d[pos] & 0x07;
		u32 x, y, pressure;

		if (id < 0 || id >= NVT_MAX_TOUCHES ||
		    (state != 0x01 && state != 0x02))
			continue;
		if (ts->high_res) {
			x = ((u32)d[pos + 1] << 8) | d[pos + 2];
			y = ((u32)d[pos + 3] << 8) | d[pos + 4];
			pressure = d[pos + 5] ?: 1;
		} else {
			x = ((u32)d[pos + 1] << 4) | (d[pos + 3] >> 4);
			y = ((u32)d[pos + 2] << 4) | (d[pos + 3] & 0x0f);
			pressure = d[pos + 5];
			if (i < 2)
				pressure += (u32)d[i + 63] << 8;
			pressure = clamp_val(pressure, 1, 1000);
		}
		if (x >= ts->max_x || y >= ts->max_y) {
			ts->out_of_range++;
			continue;
		}
		ts->touch_contacts++;
		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_PRESSURE, pressure);
	}
	/* DROP_UNUSED retires lifted contacts before pointer/BTN_TOUCH emulation. */
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void nvt_process_event(struct nt36532e *ts)
{
	const u8 *d = ts->last_event;

	if (nvt_read_event(ts))
		return;
	if (d[1] == 0x77 && d[2] == 0x77 && d[3] == 0x77 && d[4] == 0x77 &&
	    d[5] == 0x77 && d[6] == 0x77) {
		ts->boot_events++;
		return;
	}
	if (nvt_point_checksum(d)) {
		nvt_report_touch(ts, d);
	} else {
		ts->checksum_errors++;
		if (ts->checksum_errors <= 3)
			dev_warn(&ts->spi->dev,
				 "touch checksum mismatch: %8ph ... %02x\n",
				 d + 1, d[NVT_POINT_CSUM_INDEX]);
	}
	/* The touch and pen packets have independent checksums in the vendor path. */
	if (ts->pen_support)
		nvt_report_pen(ts, d);
}

static irqreturn_t nvt_irq(int irq, void *data)
{
	struct nt36532e *ts = data;

	mutex_lock(&ts->lock);
	ts->irq_count++;
	if (ts->irq_enabled)
		nvt_process_event(ts);
	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}

static void nvt_start_events(struct nt36532e *ts)
{
	/* Caller holds the mutex. Arm first, then read once: this acknowledges an
	 * already-low IRQ without losing a falling edge between drain and enable.
	 * Any concurrent IRQ thread waits until this initial read is complete.
	 */
	enable_irq(ts->spi->irq);
	ts->irq_enabled = true;
	nvt_process_event(ts);
}

static ssize_t touch_stats_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nt36532e *ts = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ts->lock);
	len = sysfs_emit(
		buf,
		"irq=%llu reads=%llu frames=%llu contacts=%llu spi_errors=%llu "
		"checksum_errors=%llu out_of_range=%llu boot_events=%llu "
		"last_error=%d enabled=%u fw=%02x protocol=%02x high_res=%u "
		"panel_ready=%u suspended=%u starts=%llu start_failures=%llu "
		"stops=%llu start_error=%d sleep_error=%d\n",
		ts->irq_count, ts->event_reads, ts->touch_frames,
		ts->touch_contacts, ts->spi_errors, ts->checksum_errors,
		ts->out_of_range, ts->boot_events, ts->last_error,
		ts->irq_enabled, ts->fw_version, ts->event_protocol,
		ts->high_res, ts->panel_ready, ts->suspended, ts->starts,
		ts->start_failures, ts->stops, ts->start_error,
		ts->sleep_error);
	mutex_unlock(&ts->lock);
	return len;
}
static DEVICE_ATTR_RO(touch_stats);

static ssize_t pen_scan_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct nt36532e *ts = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ts->lock);
	len = sysfs_emit(buf, "%d\n", ts->pen_scan_type);
	mutex_unlock(&ts->lock);
	return len;
}

static ssize_t pen_scan_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nt36532e *ts = dev_get_drvdata(dev);
	unsigned int type;
	int ret;

	ret = kstrtouint(buf, 0, &type);
	if (ret)
		return ret;
	if (type > 5)
		return -EINVAL;
	mutex_lock(&ts->lock);
	if (!ts->pen_support)
		ret = -EOPNOTSUPP;
	else if (!ts->irq_enabled)
		ret = -EBUSY;
	else {
		ret = nvt_set_pen_scan(ts, type);
		ts->pen_command_error = ret;
		/* An absent ACK leaves the applied controller state uncertain. */
		ts->pen_scan_type = ret ? -1 : (int)type;
		nvt_pen_release(ts);
	}
	mutex_unlock(&ts->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pen_scan);

static ssize_t pen_stats_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct nt36532e *ts = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ts->lock);
	len = sysfs_emit(
		buf,
		"scan_type=%d command_error=%d packets=%llu reports=%llu "
		"checksum_errors=%llu out_of_range=%llu unknown_formats=%llu "
		"format=%02x in_range=%u contact=%u raw_x=%u raw_y=%u "
		"pressure=%u distance=%u buttons=%u max_pressure=%u\n",
		ts->pen_scan_type, ts->pen_command_error, ts->pen_packets,
		ts->pen_reports, ts->pen_checksum_errors, ts->pen_out_of_range,
		ts->pen_unknown_formats, ts->pen_format, ts->pen_in_range,
		ts->pen_contact, ts->pen_x, ts->pen_y, ts->pen_pressure,
		ts->pen_distance, ts->pen_buttons, ts->pen_max_pressure);
	mutex_unlock(&ts->lock);
	return len;
}
static DEVICE_ATTR_RO(pen_stats);

static struct attribute *nvt_attrs[] = {
	&dev_attr_touch_stats.attr,
	&dev_attr_pen_scan.attr,
	&dev_attr_pen_stats.attr,
	NULL,
};
ATTRIBUTE_GROUPS(nvt);

static int nvt_input_init(struct nt36532e *ts)
{
	struct device *dev = &ts->spi->dev;
	int ret;

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;
	ts->input->name = "Novatek NT36532E Touchscreen";
	ts->input->id.bustype = BUS_SPI;
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 1000, 0, 0);
	input_set_capability(ts->input, EV_KEY, BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, ts->input->propbit);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	ret = input_mt_init_slots(ts->input, NVT_MAX_TOUCHES,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;
	ret = input_register_device(ts->input);
	if (ret || !ts->pen_support)
		return ret;
	ts->pen = devm_input_allocate_device(dev);
	if (!ts->pen)
		return -ENOMEM;
	ts->pen->name = "Novatek NT36532E Pen";
	ts->pen->id.bustype = BUS_SPI;
	input_set_abs_params(ts->pen, ABS_X, 0, ts->max_x - 1, 0, 0);
	input_set_abs_params(ts->pen, ABS_Y, 0, ts->max_y - 1, 0, 0);
	input_set_abs_params(ts->pen, ABS_PRESSURE, 0, ts->pen_max_pressure, 0,
			     0);
	if (ts->pen_x_mm)
		input_abs_set_res(ts->pen, ABS_X,
				  DIV_ROUND_CLOSEST(ts->max_x, ts->pen_x_mm));
	if (ts->pen_y_mm)
		input_abs_set_res(ts->pen, ABS_Y,
				  DIV_ROUND_CLOSEST(ts->max_y, ts->pen_y_mm));
	/* Apply axis swap to both the advertised range/resolution and positions. */
	touchscreen_parse_properties(ts->pen, false, &ts->pen_prop);
	/* The shared touchscreen-max-pressure is for fingers, not the stylus. */
	input_set_abs_params(ts->pen, ABS_PRESSURE, 0, ts->pen_max_pressure, 0,
			     0);
	input_set_abs_params(ts->pen, ABS_TILT_X, -(int)ts->pen_max_tilt,
			     ts->pen_max_tilt, 0, 0);
	input_set_abs_params(ts->pen, ABS_TILT_Y, -(int)ts->pen_max_tilt,
			     ts->pen_max_tilt, 0, 0);
	input_abs_set_res(ts->pen, ABS_TILT_X, 1);
	input_abs_set_res(ts->pen, ABS_TILT_Y, 1);
	input_set_abs_params(ts->pen, ABS_DISTANCE, 0, 65535, 0, 0);
	input_set_capability(ts->pen, EV_KEY, BTN_TOUCH);
	input_set_capability(ts->pen, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(ts->pen, EV_KEY, BTN_STYLUS);
	input_set_capability(ts->pen, EV_KEY, BTN_STYLUS2);
	__set_bit(INPUT_PROP_DIRECT, ts->pen->propbit);
	return input_register_device(ts->pen);
}

/* All state and SPI access below is serialized with the IRQ by ts->lock. */
static void nvt_stop_events(struct nt36532e *ts)
{
	u8 cmd[2] = { NVT_EVENT_HOST_CMD, NVT_CMD_SLEEP };

	if (!ts->irq_enabled)
		return;
	/* A pending IRQ thread takes the mutex and sees irq_enabled=false. */
	disable_irq_nosync(ts->spi->irq);
	ts->irq_enabled = false;
	ts->stops++;
	ts->sleep_error = nvt_set_page(ts, NVT_EVENT_BUF_ADDR);
	if (!ts->sleep_error)
		ts->sleep_error = nvt_spi_write(ts, cmd, sizeof(cmd));
	if (ts->sleep_error)
		dev_warn(&ts->spi->dev, "sleep command failed: %d\n",
			 ts->sleep_error);
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	nvt_pen_release(ts);
}

static void nvt_resume_work(struct work_struct *work)
{
	struct nt36532e *ts = container_of(work, struct nt36532e, resume_work);
	struct device *dev = &ts->spi->dev;
	int ret = 0;

	mutex_lock(&ts->lock);
	/* Panel and SPI-parent resume may run in either order. Both must finish. */
	if (ts->suspended || !ts->panel_ready || ts->irq_enabled)
		goto out;
	ts->starts++;
	nvt_hw_reset(ts);
	if (!ts->detected) {
		ret = nvt_detect(ts);
		if (!ret)
			ts->detected = true;
	}
	if (!ret)
		ret = nvt_download_fw(ts);
	if (!ret)
		ret = nvt_prepare_events(ts);
	ts->start_error = ret;
	if (ret) {
		ts->start_failures++;
		dev_err(dev, "touch start %llu failed: %d (IRQ disabled)\n",
			ts->starts, ret);
		goto out;
	}
	if (ts->pen_scan_type >= 0) {
		ts->pen_command_error = nvt_set_pen_scan(ts, ts->pen_scan_type);
		if (ts->pen_command_error) {
			dev_warn(dev, "pen scan restore failed: %d\n",
				 ts->pen_command_error);
			ts->pen_scan_type = -1;
		}
	}
	nvt_start_events(ts);
	dev_info(dev,
		 "touch start %llu complete: firmware ready, IRQ %d armed\n",
		 ts->starts, ts->spi->irq);
out:
	mutex_unlock(&ts->lock);
}

static int nvt_panel_prepared(struct drm_panel_follower *follower)
{
	struct nt36532e *ts =
		container_of(follower, struct nt36532e, panel_follower);

	mutex_lock(&ts->lock);
	ts->panel_ready = true;
	if (!ts->suspended)
		schedule_work(&ts->resume_work);
	mutex_unlock(&ts->lock);
	return 0;
}

static int nvt_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nt36532e *ts =
		container_of(follower, struct nt36532e, panel_follower);

	mutex_lock(&ts->lock);
	ts->panel_ready = false;
	nvt_stop_events(ts);
	mutex_unlock(&ts->lock);
	cancel_work_sync(&ts->resume_work);
	return 0;
}

static const struct drm_panel_follower_funcs nvt_panel_funcs = {
	.panel_prepared = nvt_panel_prepared,
	.panel_unpreparing = nvt_panel_unpreparing,
};

static void nvt_quiesce(void *data)
{
	struct nt36532e *ts = data;

	mutex_lock(&ts->lock);
	ts->suspended = true;
	nvt_stop_events(ts);
	mutex_unlock(&ts->lock);
	cancel_work_sync(&ts->resume_work);
}

static void nvt_release_firmware(void *data)
{
	release_firmware(data);
}

static int nt36532e_probe(struct spi_device *spi)
{
	struct nt36532e *ts;
	int ret;

	ts = devm_kzalloc(&spi->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	ts->pen_scan_type = -1;
	mutex_init(&ts->lock);
	INIT_WORK(&ts->resume_work, nvt_resume_work);
	spi_set_drvdata(spi, ts);
	ts->xfer_size = NVT_XFER_LEN + 2;
	ts->tx = devm_kmalloc(&spi->dev, ts->xfer_size, GFP_KERNEL);
	ts->rx = devm_kmalloc(&spi->dev, ts->xfer_size, GFP_KERNEL);
	if (!ts->tx || !ts->rx)
		return -ENOMEM;
	ts->reset_gpio =
		devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(&spi->dev, PTR_ERR(ts->reset_gpio),
				     "reset GPIO\n");
	if (device_property_read_u32(&spi->dev, "touchscreen-size-x",
				     &ts->max_x))
		ts->max_x = 21200;
	if (device_property_read_u32(&spi->dev, "touchscreen-size-y",
				     &ts->max_y))
		ts->max_y = 30000;
	if (device_property_read_u32(&spi->dev, "touchscreen-max-pressure",
				     &ts->max_pressure))
		ts->max_pressure = 4095;
	if (device_property_read_u32(&spi->dev, "novatek,pen-max-pressure",
				     &ts->pen_max_pressure))
		ts->pen_max_pressure = 16383;
	if (device_property_read_u32(&spi->dev, "novatek,pen-max-tilt",
				     &ts->pen_max_tilt))
		ts->pen_max_tilt = 60;
	if (!ts->max_x || !ts->max_y || !ts->pen_max_pressure ||
	    ts->pen_max_pressure > 65535 || !ts->pen_max_tilt ||
	    ts->pen_max_tilt > 127)
		return -EINVAL;
	device_property_read_u32(&spi->dev, "touchscreen-x-mm", &ts->pen_x_mm);
	device_property_read_u32(&spi->dev, "touchscreen-y-mm", &ts->pen_y_mm);
	ts->pen_support =
		device_property_read_bool(&spi->dev, "novatek,pen-support");
	if (device_property_read_string(&spi->dev, "firmware-name",
					&ts->fw_name))
		ts->fw_name = "novatek/DT-novatek-nt36532.bin";
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;
	/* Keep the no-flash image for the device lifetime. Resume must not rely
	 * on a mounted rootfs or the temporary firmware-loader suspend cache.
	 */
	ret = request_firmware(&ts->fw, ts->fw_name, &spi->dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "cannot load %s\n",
				     ts->fw_name);
	ret = devm_add_action_or_reset(&spi->dev, nvt_release_firmware,
				       (void *)ts->fw);
	if (ret)
		return ret;
	ret = nvt_input_init(ts);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL, nvt_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
						IRQF_NO_AUTOEN,
					dev_name(&spi->dev), ts);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "request touch IRQ\n");
	/* Unregister the follower and drain work before devm frees IRQ/input/fw. */
	ret = devm_add_action_or_reset(&spi->dev, nvt_quiesce, ts);
	if (ret)
		return ret;
	if (drm_is_panel_follower(&spi->dev)) {
		ts->panel_follower.funcs = &nvt_panel_funcs;
		ret = devm_drm_panel_add_follower(&spi->dev,
						  &ts->panel_follower);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "register panel follower\n");
		dev_info(&spi->dev, "touch power follows panel preparation\n");
	} else {
		ts->panel_ready = true;
		schedule_work(&ts->resume_work);
	}
	return 0;
}

static int nt36532e_suspend(struct device *dev)
{
	nvt_quiesce(dev_get_drvdata(dev));
	return 0;
}
static int nt36532e_resume(struct device *dev)
{
	struct nt36532e *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->lock);
	ts->suspended = false;
	if (ts->panel_ready)
		schedule_work(&ts->resume_work);
	mutex_unlock(&ts->lock);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(nt36532e_pm, nt36532e_suspend, nt36532e_resume);
static const struct of_device_id nt36532e_of_match[] = {
	{ .compatible = "novatek,nt36532e" },
	{}
};
MODULE_DEVICE_TABLE(of, nt36532e_of_match);
static struct spi_driver nt36532e_driver = {
	.driver = { .name = "nt36532e",
		    .of_match_table = nt36532e_of_match,
		    .pm = pm_sleep_ptr(&nt36532e_pm),
		    .dev_groups = nvt_groups },
	.probe = nt36532e_probe
};
module_spi_driver(nt36532e_driver);
MODULE_DESCRIPTION("Novatek NT36532E no-flash SPI touchscreen and pen");
MODULE_LICENSE("GPL");
