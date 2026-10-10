// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/* Bounded CPS8601 pen charging controller for OnePlus Pad Pro (caihong). */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define PEN_OWNER 32785
#define PEN_SET_HBOOST 0x10007
#define PEN_ACK_MS 2500
#define PEN_NAME "caihong-pen-power"
#define PEN_I2C_PATH "/soc@0/geniqup@9c0000/i2c@98c000"
#define PEN_TLMM_PATH "/soc@0/pinctrl@f100000"
#define PEN_ATTACH_MS 15000
#define PEN_STARTUP_MS 2500
/* FW 0x0118 writes 2 to register 0x0004 on TX entry; see attachment docs. */
#define PEN_SYS_MODE_TX 2
#define PEN_INT_ATTACH BIT(9)
#define PEN_INT_REMOVE BIT(10)
#define PEN_INT_ASK BIT(5)
#define PEN_INT_FAULT (BIT(6) | BIT(7) | BIT(8) | BIT(13))
#define PEN_INT_UNDEFINED (BIT(11) | BIT(14) | BIT(15))

/* Board-local lookup for the current DT; never use global GPIO IDs. */
static struct gpiod_lookup_table pen_gpios = {
	.dev_id = PEN_NAME,
	.table = {
		GPIO_LOOKUP("f100000.pinctrl", 111, "charge-disable", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("f100000.pinctrl", 10, "supply", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("f100000.pinctrl", 15, "wake", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("f100000.pinctrl", 85, "scan", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("f100000.pinctrl", 12, "irq", GPIO_ACTIVE_HIGH),
		{ }
	},
};

/* TLMM exposes bias via pinctrl, not gpio_chip.set_config on this kernel. */
static unsigned long pen_irq_config[] = {
	PIN_CONF_PACKED(PIN_CONFIG_BIAS_PULL_UP, 1),
};

static const struct pinctrl_map pen_pinmaps[] = {
	PIN_MAP_CONFIGS_GROUP(PEN_NAME, "irq-pull-up", "f100000.pinctrl",
			      "gpio12", pen_irq_config),
};

struct pen_boost_request {
	struct pmic_glink_hdr hdr;
	u8 value;
	u8 reserved[3];
};

struct pen_exchange {
	u32 events, attaches, removes, asks, checks, addresses, invalid;
	u32 charge_samples, charge_ms, stop_packets;
	u16 flags_seen, charge_iin_min, charge_iin_max, charge_temp_max;
	u8 stop_soc;
	u16 initial_flags, last_flags, vin, iin, temp, ept;
	u16 command_before, command_after, mode_after;
	u16 defaults[4], after_cycle[4];
	u8 defaults_valid, cycle_valid, initial_raw[11];
	u8 raw[11], mac[6], check[3];
	bool have_check, have_address, mac_valid, enabled, initial_raw_valid, cycled;
	bool charge_complete;
};

struct pen_power {
	struct device *dev;
	struct pmic_glink_client *glink;
	struct i2c_client *i2c;
	struct gpio_desc *disable, *supply, *wake, *scan, *irq;
	struct mutex lock;
	struct mutex power_lock;
	spinlock_t ack_lock;
	struct completion ack, lost;
	struct completion event;
	struct delayed_work cutoff_work;
	atomic_t cutoff_fired, irq_count;
	struct pen_exchange exchange;
	bool attach_requested, tx_requested, cycle_requested, startup_requested;
	bool charge_requested;
	u32 startup_ready_reads, startup_ready_ms;
	bool up, pending, poisoned, active, suspended;
	bool hardware_ready;
	int ack_error;
	u32 rejected;
	int result, cleanup;
	const char *phase;
	u32 valid;
	u16 values[8];
};

static struct platform_device *pen_device;
static int pen_probe_result = -ENODEV;
/* One attempt per module load; unbinding/rebinding must not reset this. */
static bool pen_used;

static bool pen_reply_valid(const void *data, size_t len)
{
	const u8 *bytes = data;

	return len == 16 && get_unaligned_le32(bytes) == PEN_OWNER &&
		get_unaligned_le32(bytes + 4) == PMIC_GLINK_REQ_RESP &&
		get_unaligned_le32(bytes + 8) == PEN_SET_HBOOST;
}

static void pen_reply(const void *data, size_t len, void *priv)
{
	struct pen_power *pen = priv;
	unsigned long flags;

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (!pen_reply_valid(data, len) || !pen->pending) {
		pen->rejected++;
	} else {
		pen->ack_error = get_unaligned_le32((const u8 *)data + 12) ?
			-EREMOTEIO : 0;
		pen->pending = false;
		complete(&pen->ack);
	}
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static void pen_transport(void *priv, int state)
{
	struct pen_power *pen = priv;
	unsigned long flags;

	/* PMIC-Glink invokes callbacks under a spinlock; never sleep here. */
	spin_lock_irqsave(&pen->ack_lock, flags);
	pen->up = state == SERVREG_SERVICE_STATE_UP;
	if (!pen->up) {
		if (pen->active)
			pen->poisoned = true;
		if (pen->pending) {
			pen->ack_error = -ENOTCONN;
			pen->pending = false;
			complete(&pen->ack);
		}
		complete_all(&pen->lost);
	}
	spin_unlock_irqrestore(&pen->ack_lock, flags);
}

static int pen_set_boost(struct pen_power *pen, u8 value)
{
	struct pen_boost_request req = {
		.hdr = {
			.owner = cpu_to_le32(PEN_OWNER),
			.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
			.opcode = cpu_to_le32(PEN_SET_HBOOST),
		},
		.value = value,
	};
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (!pen->up || pen->poisoned) {
		ret = pen->poisoned ? -EPIPE : -ENOTCONN;
		spin_unlock_irqrestore(&pen->ack_lock, flags);
		return ret;
	}
	reinit_completion(&pen->ack);
	pen->pending = true;
	pen->ack_error = -EINPROGRESS;
	spin_unlock_irqrestore(&pen->ack_lock, flags);

	ret = pmic_glink_send(pen->glink, &req, sizeof(req));
	if (!ret)
		wait_for_completion_timeout(&pen->ack, msecs_to_jiffies(PEN_ACK_MS));

	spin_lock_irqsave(&pen->ack_lock, flags);
	if (ret || pen->pending) {
		/* No request cookie: a late ACK cannot identify a subsequent request. */
		pen->poisoned = true;
		pen->pending = false;
		pen->ack_error = ret ?: -ETIMEDOUT;
	}
	ret = pen->ack_error;
	if (!ret && pen->poisoned)
		ret = -ENOTCONN;
	spin_unlock_irqrestore(&pen->ack_lock, flags);
	return ret;
}

static void pen_off(struct pen_power *pen)
{
	gpiod_set_value_cansleep(pen->disable, 1);
	gpiod_set_value_cansleep(pen->wake, 0);
	gpiod_set_value_cansleep(pen->scan, 0);
	gpiod_set_value_cansleep(pen->supply, 0);
}

static int pen_delay(struct pen_power *pen, unsigned int ms)
{
	return wait_for_completion_timeout(&pen->lost, msecs_to_jiffies(ms)) ?
		-ENOTCONN : 0;
}

static int pen_read(struct pen_power *pen, u16 reg, unsigned int len, u16 *value)
{
	u16 result = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		u8 selector[2], byte;
		struct i2c_msg messages[] = {
			{ .addr = 0x41, .len = 2, .buf = selector },
			{ .addr = 0x41, .flags = I2C_M_RD, .len = 1, .buf = &byte },
		};

		if (completion_done(&pen->lost))
			return -ENOTCONN;
		put_unaligned_be16(reg + i, selector);
		ret = i2c_transfer(pen->i2c->adapter, messages, ARRAY_SIZE(messages));
		if (ret != ARRAY_SIZE(messages))
			return ret < 0 ? ret : -EIO;
		result |= (u16)byte << (8 * i);
	}
	*value = result;
	return 0;
}

static int pen_identify(struct pen_power *pen)
{
	static const u16 registers[] = { 0, 2, 4, 7, 0x34, 0x38, 0x3a, 0x3e };
	static const u8 sizes[] = { 2, 2, 1, 2, 2, 2, 1, 2 };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		ret = pen_read(pen, registers[i], sizes[i], &pen->values[i]);
		if (ret)
			return ret;
		pen->valid |= BIT(i);
		if (!i && pen->values[0] != 0x8601)
			return -ENODEV;
	}
	return 0;
}

static int pen_write(struct pen_power *pen, u16 reg, unsigned int len, u16 value)
{
	unsigned int i;
	int ret;

	/* Only stock protection/IRQ registers and the explicitly requested TX bit. */
	if (pen->startup_requested && reg != 5 && reg != 9 &&
	    !(pen->charge_requested && pen->exchange.mac_valid))
		return -EPERM;
	if (reg == 0xb) {
		if (len != 1 || value != BIT(1) || !pen->tx_requested ||
		    !pen->exchange.enabled || gpiod_get_value_cansleep(pen->supply) != 1 ||
		    gpiod_get_value_cansleep(pen->disable) != 0)
			return -EPERM;
	} else if (len != 2 || (reg != 0x28 && reg != 0x2a && reg != 0x2c &&
				reg != 0x2f && reg != 5 && reg != 9)) {
		return -EPERM;
	}
	for (i = 0; i < len; i++) {
		u8 data[3];
		struct i2c_msg msg = { .addr = 0x41, .len = 3, .buf = data };

		if (completion_done(&pen->lost) || atomic_read(&pen->cutoff_fired))
			return -ECANCELED;
		put_unaligned_be16(reg + i, data);
		data[2] = value >> (8 * i);
		ret = i2c_transfer(pen->i2c->adapter, &msg, 1);
		if (ret != 1)
			return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int pen_write_checked(struct pen_power *pen, u16 reg, u16 value)
{
	u16 actual;
	int ret;

	ret = pen_write(pen, reg, 2, value);
	if (ret) {
		dev_err(pen->dev, "attach=config reg=%#x write error=%d\n", reg, ret);
		return ret;
	}
	ret = pen_read(pen, reg, 2, &actual);
	if (!ret && actual != value)
		ret = -EIO;
	if (ret)
		dev_err(pen->dev, "attach=config reg=%#x readback error=%d\n", reg, ret);
	return ret;
}

static int pen_packet(struct pen_exchange *ex)
{
	const u8 *p = ex->raw;
	unsigned int i;

	if (p[0] == 0x48 && p[1] == 0xc1) {
		ex->checks++;
		ex->check[0] = p[4];
		ex->check[1] = p[3];
		ex->check[2] = p[2];
		ex->have_check = true;
	} else if (p[0] == 0x48 && p[1] == 0xb6 && p[5] == 0x48 && p[6] == 0xb7) {
		ex->addresses++;
		for (i = 0; i < 3; i++) {
			ex->mac[i] = (p[9 - i] << 4) | (p[4 - i] >> 4);
			ex->mac[3 + i] = (p[4 - i] << 4) | (p[9 - i] >> 4);
		}
		ex->have_address = true;
	} else {
		/* Stock treats 0x28/0x17 as an instruction to stop charging. */
		if (p[0] == 0x28 && p[1] == 0x17) {
			ex->stop_packets++;
			ex->stop_soc = p[2];
			return -ECANCELED;
		}
		return 0;
	}
	if (!ex->have_check || !ex->have_address)
		return 0;
	for (i = 0; i < 3; i++) {
		if ((ex->mac[i] ^ ex->mac[3 + i]) != ex->check[i]) {
			ex->invalid++;
			return -EBADMSG;
		}
	}
	if (!memchr_inv(ex->mac, 0, 6) || !memchr_inv(ex->mac, 0xff, 6)) {
		ex->invalid++;
		return -EBADMSG;
	}
	ex->mac_valid = true;
	return 0;
}

static int pen_sample(struct pen_power *pen)
{
	struct pen_exchange *ex = &pen->exchange;
	int ret;

	ret = pen_read(pen, 0x34, 2, &ex->vin);
	if (!ret)
		ret = pen_read(pen, 0x38, 2, &ex->iin);
	if (!ret)
		ret = pen_read(pen, 0x3a, 1, &ex->temp);
	if (!ret)
		ret = pen_read(pen, 0x3e, 2, &ex->ept);
	if (ret)
		return ret;
	/* Conservative experiment limits, in addition to the stock hardware OCP. */
	if (ex->vin < 4000 || ex->vin > 6500 || ex->iin >= 500 ||
	    ex->temp >= 50 || ex->ept)
		return -ERANGE;
	return 0;
}

static int pen_prepare_attach(struct pen_power *pen)
{
	static const u16 regs[] = { 0x28, 0x2c, 0x2a, 0x2f };
	static const u16 limits[] = { 500, 4000, 12000, 400 };
	struct pen_exchange *ex = &pen->exchange;
	u16 flags, byte;
	unsigned int i;
	int ret;

	/* Only the firmware identified on this tablet is eligible for TX testing. */
	if (pen->valid != 0xff || pen->values[0] != 0x8601 || pen->values[1] != 0x0118)
		return -EOPNOTSUPP;
	/* Preserve startup evidence separately; never use it as fresh identity. */
	ret = pen_read(pen, 7, 2, &ex->initial_flags);
	if (ret)
		return ret;
	if (ex->initial_flags & PEN_INT_ASK) {
		for (i = 0; i < ARRAY_SIZE(ex->initial_raw); i++) {
			ret = pen_read(pen, 0x40 + i, 1, &byte);
			if (ret)
				return ret;
			ex->initial_raw[i] = byte;
		}
		ex->initial_raw_valid = true;
	}
	/* A supply cycle may restore defaults; require known limits before it. */
	if (pen->cycle_requested) {
		for (i = 0; i < ARRAY_SIZE(regs); i++) {
			ret = pen_read(pen, regs[i], 2, &ex->defaults[i]);
			if (ret)
				return ret;
			ex->defaults_valid |= BIT(i);
		}
		for (i = 0; i < ARRAY_SIZE(regs); i++) {
			if (ex->defaults[i] != limits[i])
				return -EOPNOTSUPP;
		}
	}
	ret = pen_write_checked(pen, 0x28, 500);
	if (!ret)
		ret = pen_write_checked(pen, 0x2c, 4000);
	if (!ret)
		ret = pen_write_checked(pen, 0x2a, 12000);
	if (!ret)
		ret = pen_write_checked(pen, 0x2f, 400);
	if (!ret)
		ret = pen_write_checked(pen, 5, 0xffff);
	if (!ret)
		ret = pen_sample(pen);
	if (!ret && pen->tx_requested) {
		ret = pen_read(pen, 0xb, 1, &pen->exchange.command_before);
		if (!ret && pen->exchange.command_before)
			ret = -EBUSY;
	}
	if (!ret)
		ret = pen_write(pen, 9, 2, pen->exchange.initial_flags);
	if (!ret)
		ret = pen_delay(pen, 10);
	if (!ret)
		ret = pen_read(pen, 7, 2, &flags);
	if (!ret && flags)
		ret = -EBUSY;
	return ret;
}

static int pen_cycle(struct pen_power *pen)
{
	static const u16 regs[] = { 0x28, 0x2c, 0x2a, 0x2f };
	struct pen_exchange *ex = &pen->exchange;
	u16 id, firmware;
	unsigned int i;
	int ret;

	pen->phase = "attach-supply-cycle";
	/* Match the stock final 50 ms off/on cycle, within the active deadline. */
	gpiod_set_value_cansleep(pen->supply, 0);
	if (gpiod_get_value_cansleep(pen->supply) != 0)
		return -EIO;
	ret = pen_delay(pen, 50);
	if (ret)
		return ret;
	mutex_lock(&pen->power_lock);
	if (atomic_read(&pen->cutoff_fired) || completion_done(&pen->lost)) {
		ret = -ECANCELED;
	} else {
		gpiod_set_value_cansleep(pen->supply, 1);
		ex->cycled = true;
		if (gpiod_get_value_cansleep(pen->supply) != 1)
			ret = -EIO;
	}
	mutex_unlock(&pen->power_lock);
	if (!ret)
		ret = pen_delay(pen, 50);
	if (!ret)
		ret = pen_read(pen, 0, 2, &id);
	if (!ret)
		ret = pen_read(pen, 2, 2, &firmware);
	if (!ret && (id != 0x8601 || firmware != 0x0118))
		ret = -ENODEV;
	for (i = 0; !ret && i < ARRAY_SIZE(regs); i++) {
		ret = pen_read(pen, regs[i], 2, &ex->after_cycle[i]);
		if (!ret) {
			ex->cycle_valid |= BIT(i);
			if (ex->after_cycle[i] != ex->defaults[i])
				ret = -EIO;
		}
	}
	if (!ret)
		ret = pen_write_checked(pen, 5, 0xffff);
	if (!ret)
		ret = pen_read(pen, 4, 1, &ex->mode_after);
	return ret;
}

static void pen_cutoff(struct work_struct *work)
{
	struct pen_power *pen = container_of(to_delayed_work(work), struct pen_power, cutoff_work);

	/* Never wait for the I2C/experiment mutex to cut the physical supply. */
	mutex_lock(&pen->power_lock);
	atomic_set(&pen->cutoff_fired, 1);
	pen_off(pen);
	mutex_unlock(&pen->power_lock);
	complete(&pen->event);
}

static irqreturn_t pen_irq_thread(int irq, void *data)
{
	struct pen_power *pen = data;

	atomic_inc(&pen->irq_count);
	complete(&pen->event);
	return IRQ_HANDLED;
}

static int pen_receive(struct pen_power *pen)
{
	struct pen_exchange *ex = &pen->exchange;
	u16 flags, byte;
	unsigned int i;
	int ret;

	ret = pen_read(pen, 7, 2, &flags);
	if (ret || !flags)
		return ret;
	ex->events++;
	ex->last_flags = flags;
	ex->flags_seen |= flags;
	/* Consume stale/aborted identity before accepting any new packet. */
	if (flags & PEN_INT_ATTACH) {
		ex->attaches++;
		ex->have_check = ex->have_address = ex->mac_valid = false;
	}
	if (flags & PEN_INT_REMOVE) {
		ex->removes++;
		ex->have_check = ex->have_address = ex->mac_valid = false;
		return -ENOLINK;
	}
	if (flags & PEN_INT_FAULT)
		return -ECANCELED;
	/* Stock clears the latched flags before consuming the ASK mailbox. */
	ret = pen_write(pen, 9, 2, flags);
	if (ret)
		return ret;
	if (flags & PEN_INT_ASK) {
		ex->asks++;
		for (i = 0; i < ARRAY_SIZE(ex->raw); i++) {
			ret = pen_read(pen, 0x40 + i, 1, &byte);
			if (ret)
				return ret;
			ex->raw[i] = byte;
		}
		ret = pen_packet(ex);
	}
	return ret;
}

static int pen_attach(struct pen_power *pen)
{
	unsigned long deadline, sample_at;
	int ret;

	pen->phase = "attach-prepare";
	ret = pen_prepare_attach(pen);
	if (ret)
		return ret;
	reinit_completion(&pen->event);
	atomic_set(&pen->irq_count, 0);
	deadline = jiffies + msecs_to_jiffies(PEN_ATTACH_MS);
	sample_at = jiffies;
	if (!queue_delayed_work(system_unbound_wq, &pen->cutoff_work,
				msecs_to_jiffies(PEN_ATTACH_MS)))
		return -EBUSY;
	mutex_lock(&pen->power_lock);
	if (atomic_read(&pen->cutoff_fired) || completion_done(&pen->lost)) {
		ret = -ECANCELED;
	} else {
		/* Stock screen-on scan, allow charging, then enter automatic operation. */
		gpiod_set_value_cansleep(pen->scan, 1);
		gpiod_set_value_cansleep(pen->disable, 0);
		gpiod_set_value_cansleep(pen->wake, 0);
		pen->exchange.enabled = true;
		if (gpiod_get_value_cansleep(pen->disable) != 0 ||
		    gpiod_get_value_cansleep(pen->supply) != 1 ||
		    gpiod_get_value_cansleep(pen->scan) != 1 ||
		    gpiod_get_value_cansleep(pen->wake) != 0)
			ret = -EIO;
	}
	mutex_unlock(&pen->power_lock);
	if (!ret && pen->cycle_requested)
		ret = pen_cycle(pen);
	if (!ret && pen->tx_requested) {
		pen->phase = "attach-enter-tx";
		ret = pen_write(pen, 0xb, 1, BIT(1));
		if (!ret)
			ret = pen_read(pen, 0xb, 1, &pen->exchange.command_after);
		if (!ret)
			ret = pen_read(pen, 4, 1, &pen->exchange.mode_after);
	}
	if (!ret) {
		pen->phase = "attach-observe";
	}
	while (!ret && !pen->exchange.mac_valid) {
		if (atomic_read(&pen->cutoff_fired) || time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			break;
		}
		if (completion_done(&pen->lost)) {
			ret = -ENOTCONN;
			break;
		}
		if (time_after_eq(jiffies, sample_at)) {
			ret = pen_sample(pen);
			sample_at = jiffies + msecs_to_jiffies(100);
		}
		if (!ret)
			ret = pen_receive(pen);
		if (!ret && !pen->exchange.mac_valid)
			wait_for_completion_timeout(&pen->event, msecs_to_jiffies(20));
	}
	cancel_delayed_work_sync(&pen->cutoff_work);
	pen_off(pen);
	if (atomic_read(&pen->cutoff_fired))
		ret = -ETIMEDOUT;
	return ret;
}

static int pen_charge(struct pen_power *pen, unsigned long deadline)
{
	struct pen_exchange *ex = &pen->exchange;
	unsigned long started, sample_at;
	int ret;

	pen->phase = "charge-protect";
	if (!pen->charge_requested || !ex->mac_valid || pen->valid != 0xff ||
	    pen->values[0] != 0x8601 || pen->values[1] != 0x0118)
		return -EOPNOTSUPP;
	ret = pen_read(pen, 4, 1, &ex->mode_after);
	if (!ret && ex->mode_after != PEN_SYS_MODE_TX)
		ret = -EOPNOTSUPP;
	if (!ret)
		ret = pen_sample(pen);
	if (!ret)
		ret = pen_write_checked(pen, 0x28, 500);
	if (!ret)
		ret = pen_write_checked(pen, 0x2c, 4000);
	if (!ret)
		ret = pen_write_checked(pen, 0x2a, 12000);
	if (!ret)
		ret = pen_write_checked(pen, 0x2f, 400);
	/* Do not discard a removal, new attachment or stop packet during setup. */
	if (!ret)
		ret = pen_receive(pen);
	if (!ret && !ex->mac_valid)
		ret = -ENOLINK;
	if (!ret && (ex->flags_seen & PEN_INT_UNDEFINED))
		ret = -EPROTO;
	if (ret)
		return ret;
	mutex_lock(&pen->power_lock);
	if (atomic_read(&pen->cutoff_fired) || completion_done(&pen->lost) ||
	    time_after_eq(jiffies, deadline)) {
		ret = -ECANCELED;
	} else if (gpiod_get_value_cansleep(pen->disable) != 1 ||
		   gpiod_get_value_cansleep(pen->supply) != 1 ||
		   gpiod_get_value_cansleep(pen->wake) != 1) {
		ret = -EIO;
	} else {
		/* Keep the verified startup wake state; no reset or TX command. */
		gpiod_set_value_cansleep(pen->scan, 1);
		gpiod_set_value_cansleep(pen->disable, 0);
		ex->enabled = true;
		if (gpiod_get_value_cansleep(pen->disable) != 0 ||
		    gpiod_get_value_cansleep(pen->scan) != 1)
			ret = -EIO;
	}
	mutex_unlock(&pen->power_lock);
	if (ret)
		return ret;
	pen->phase = "charge-observe";
	started = jiffies;
	sample_at = jiffies;
	while (!ret) {
		if (time_after_eq(jiffies, deadline)) {
			ex->charge_complete = true;
			break;
		}
		if (atomic_read(&pen->cutoff_fired) || completion_done(&pen->lost)) {
			ret = -ECANCELED;
			break;
		}
		if (time_after_eq(jiffies, sample_at)) {
			ret = pen_sample(pen);
			if (ret)
				break;
			if (!ex->charge_samples || ex->iin < ex->charge_iin_min)
				ex->charge_iin_min = ex->iin;
			if (ex->iin > ex->charge_iin_max)
				ex->charge_iin_max = ex->iin;
			if (ex->temp > ex->charge_temp_max)
				ex->charge_temp_max = ex->temp;
			ex->charge_samples++;
			sample_at = jiffies + msecs_to_jiffies(100);
		}
		ret = pen_receive(pen);
		if (!ret && !ex->mac_valid)
			ret = -ENOLINK;
		if (!ret && (ex->flags_seen & PEN_INT_UNDEFINED))
			ret = -EPROTO;
		if (!ret)
			wait_for_completion_timeout(&pen->event, msecs_to_jiffies(20));
	}
	ex->charge_ms = jiffies_to_msecs(jiffies - started);
	return ret;
}

static int pen_startup(struct pen_power *pen)
{
	unsigned long started = jiffies;
	unsigned long deadline = started + msecs_to_jiffies(PEN_STARTUP_MS);
	unsigned int budget = pen->charge_requested ? PEN_ATTACH_MS : PEN_STARTUP_MS;
	unsigned long sample_at = jiffies;
	u16 id;
	int ret;

	pen->phase = "startup-identify";
	if (!queue_delayed_work(system_unbound_wq, &pen->cutoff_work,
				msecs_to_jiffies(budget)))
		return -EBUSY;
	/* An early ID ACK can precede a startup NACK. Retry only read-only
	 * identification, before any IRQ/protection write, within one deadline.
	 */
	for (;;) {
		if (atomic_read(&pen->cutoff_fired) || time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		pen->valid = 0;
		memset(pen->values, 0, sizeof(pen->values));
		pen->startup_ready_reads++;
		ret = pen_read(pen, 0, 2, &id);
		if (!ret && id != 0x8601)
			ret = -ENODEV;
		if (!ret)
			ret = pen_identify(pen);
		if (!ret && pen->values[1] != 0x0118)
			ret = -EOPNOTSUPP;
		pen->startup_ready_ms = jiffies_to_msecs(jiffies - started);
		if (ret != -ENXIO)
			break;
		ret = pen_delay(pen, 5);
		if (ret)
			goto out;
	}
	if (!ret && (atomic_read(&pen->cutoff_fired) || time_after_eq(jiffies, deadline)))
		ret = -ETIMEDOUT;
	if (ret)
		goto out;
	/* A cold start can consume 2 s before the first ACK. Give the charge
	 * request its full handshake window after identification, still covered
	 * by the original 15 s physical cutoff. Legacy startup stays unchanged.
	 */
	if (pen->charge_requested)
		deadline = jiffies + msecs_to_jiffies(PEN_STARTUP_MS);
	/* Initial reception keeps charge inhibit high and scan low. */
	if (gpiod_get_value_cansleep(pen->disable) != 1 ||
	    gpiod_get_value_cansleep(pen->supply) != 1 ||
	    gpiod_get_value_cansleep(pen->wake) != 1 ||
	    gpiod_get_value_cansleep(pen->scan) != 0) {
		ret = -EIO;
		goto out;
	}
	pen->exchange.initial_flags = pen->values[3];
	ret = pen_write_checked(pen, 5, 0xffff);
	pen->phase = "startup-observe";
	/* Consume the initial ASK, not a cleared baseline after the wake delay. */
	while (!ret && !pen->exchange.mac_valid) {
		if (atomic_read(&pen->cutoff_fired) || time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			break;
		}
		if (completion_done(&pen->lost)) {
			ret = -ENOTCONN;
			break;
		}
		ret = pen_receive(pen);
		if (!ret && time_after_eq(jiffies, sample_at)) {
			ret = pen_sample(pen);
			sample_at = jiffies + msecs_to_jiffies(100);
		}
		if (!ret && !pen->exchange.mac_valid)
			wait_for_completion_timeout(&pen->event, msecs_to_jiffies(5));
	}
	if (!ret && pen->charge_requested)
		ret = pen_charge(pen, started + msecs_to_jiffies(budget));
out:
	cancel_delayed_work_sync(&pen->cutoff_work);
	pen_off(pen);
	if (atomic_read(&pen->cutoff_fired) && !pen->exchange.charge_complete)
		ret = -ETIMEDOUT;
	return ret;
}

static int pen_run(struct pen_power *pen)
{
	int ret;

	if (!pen->hardware_ready)
		return -EOPNOTSUPP;
	pen_off(pen);
	pen->phase = "off-check";
	if (gpiod_get_value_cansleep(pen->disable) != 1 ||
	    gpiod_get_value_cansleep(pen->supply) != 0)
		return -EIO;
	pen->phase = "hboost";
	ret = pen_set_boost(pen, 76); /* (5800 mV - 2000 mV) / 50 */
	if (ret)
		goto out;
	pen->phase = "supply";
	gpiod_set_value_cansleep(pen->supply, 1);
	if (gpiod_get_value_cansleep(pen->supply) != 1) {
		ret = -EIO;
		goto out;
	}
	ret = pen_delay(pen, 10);
	if (ret)
		goto out;
	pen->phase = "wake";
	gpiod_set_value_cansleep(pen->wake, 1);
	if (gpiod_get_value_cansleep(pen->wake) != 1) {
		ret = -EIO;
		goto out;
	}
	if (pen->startup_requested) {
		ret = pen_startup(pen);
		goto out;
	}
	ret = pen_delay(pen, 2500);
	if (!ret) {
		pen->phase = "read-id-status";
		ret = pen_identify(pen);
	}
	if (!ret && pen->attach_requested)
		ret = pen_attach(pen);
out:
	/* Cut the physical path before attempting the minimum HBOOST request. */
	pen_off(pen);
	pen->cleanup = pen_set_boost(pen, 0);
	if (gpiod_get_value_cansleep(pen->disable) != 1 ||
	    gpiod_get_value_cansleep(pen->wake) != 0 ||
	    gpiod_get_value_cansleep(pen->supply) != 0)
		pen->cleanup = -EIO;
	if (!ret)
		pen->phase = pen->cleanup ? "cleanup" : "done";
	return ret ?: pen->cleanup;
}

static ssize_t pen_start(struct device *dev, const char *buf, size_t count)
{
	struct pen_power *pen = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	if (!sysfs_streq(buf, "1") && !sysfs_streq(buf, "tx") &&
	    !sysfs_streq(buf, "cycle") && !sysfs_streq(buf, "startup") &&
	    !sysfs_streq(buf, "charge"))
		return -EINVAL;
	if (!pen->hardware_ready)
		return -EOPNOTSUPP;
	if (!mutex_trylock(&pen->lock))
		return -EBUSY;
	if (pen_used || pen->suspended) {
		ret = pen_used ? -EALREADY : -EBUSY;
		goto unlock;
	}
	spin_lock_irqsave(&pen->ack_lock, flags);
	if (!pen->up) {
		spin_unlock_irqrestore(&pen->ack_lock, flags);
		ret = -ENOTCONN;
		goto unlock;
	}
	reinit_completion(&pen->lost);
	pen->active = true;
	spin_unlock_irqrestore(&pen->ack_lock, flags);
	pen_used = true;
	pen->attach_requested = true;
	pen->tx_requested = sysfs_streq(buf, "tx");
	pen->cycle_requested = sysfs_streq(buf, "cycle");
	pen->charge_requested = sysfs_streq(buf, "charge");
	pen->startup_requested = sysfs_streq(buf, "startup") || pen->charge_requested;
	pm_stay_awake(dev);
	pen->result = pen_run(pen);
	spin_lock_irqsave(&pen->ack_lock, flags);
	pen->active = false;
	spin_unlock_irqrestore(&pen->ack_lock, flags);
	pm_relax(dev);
	ret = pen->result;
unlock:
	mutex_unlock(&pen->lock);
	return ret ?: count;
}

static ssize_t attach_once_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return pen_start(dev, buf, count);
}
static DEVICE_ATTR_WO(attach_once);

static ssize_t attach_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pen_power *pen = dev_get_drvdata(dev);
	struct pen_exchange *ex = &pen->exchange;
	ssize_t count;

	mutex_lock(&pen->lock);
	count = sysfs_emit(buf,
		"enabled=%u cutoff=%d irq=%d events=%u attaches=%u removes=%u asks=%u checks=%u addresses=%u invalid=%u initial_flags=%#x last_flags=%#x\n"
		"tx_requested=%u command_before=%#x command_after=%#x mode_after=%#x\n"
		"startup_requested=%u startup_ready_reads=%u startup_ready_ms=%u\n"
		"charge_requested=%u charge_complete=%u charge_samples=%u charge_ms=%u charge_iin_min=%u charge_iin_max=%u charge_temp_max=%u stop_packets=%u stop_soc=%u flags_seen=%#x\n"
		"cycle_requested=%u cycled=%u defaults_valid=%#x defaults=%u,%u,%u,%u cycle_valid=%#x after_cycle=%u,%u,%u,%u\n"
		"initial_raw_valid=%u initial_raw=%*ph\n"
		"mac_valid=%u mac=%pM vin=%u iin=%u temperature=%u ept=%#x raw=%*ph\n",
		ex->enabled, atomic_read(&pen->cutoff_fired), atomic_read(&pen->irq_count),
		ex->events, ex->attaches, ex->removes, ex->asks, ex->checks, ex->addresses,
		ex->invalid, ex->initial_flags, ex->last_flags,
		pen->tx_requested, ex->command_before, ex->command_after, ex->mode_after,
		pen->startup_requested, pen->startup_ready_reads, pen->startup_ready_ms,
		pen->charge_requested, ex->charge_complete, ex->charge_samples, ex->charge_ms,
		ex->charge_iin_min, ex->charge_iin_max, ex->charge_temp_max,
		ex->stop_packets, ex->stop_soc, ex->flags_seen,
		pen->cycle_requested, ex->cycled, ex->defaults_valid,
		ex->defaults[0], ex->defaults[1], ex->defaults[2], ex->defaults[3],
		ex->cycle_valid, ex->after_cycle[0], ex->after_cycle[1],
		ex->after_cycle[2], ex->after_cycle[3],
		ex->initial_raw_valid, (int)sizeof(ex->initial_raw), ex->initial_raw,
		ex->mac_valid, ex->mac,
		ex->vin, ex->iin, ex->temp, ex->ept, (int)sizeof(ex->raw), ex->raw);
	mutex_unlock(&pen->lock);
	return count;
}
/* Contains a hardware address and packet payload; keep it local to root. */
static struct device_attribute dev_attr_attach_status = __ATTR(attach_status, 0400,
							     attach_status_show, NULL);

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pen_power *pen = dev_get_drvdata(dev);
	unsigned long flags;
	bool up, poisoned;
	u32 rejected;
	ssize_t count;

	mutex_lock(&pen->lock);
	spin_lock_irqsave(&pen->ack_lock, flags);
	up = pen->up;
	poisoned = pen->poisoned;
	rejected = pen->rejected;
	spin_unlock_irqrestore(&pen->ack_lock, flags);
	count = sysfs_emit(buf,
		"hardware_ready=%u attempted=%u transport_up=%u poisoned=%u rejected=%u phase=%s result=%d cleanup=%d valid=%#x\n"
		"chip_id=%#06x firmware=%#06x mode=%#x irq=%#x vin_raw=%u iin_raw=%u temperature_raw=%u ept=%#x\n"
		"charge_disable=%d supply=%d wake=%d scan=%d irq_level=%d\n",
		pen->hardware_ready, pen_used, up, poisoned, rejected,
		pen->phase, pen->result, pen->cleanup, pen->valid,
		pen->values[0], pen->values[1], pen->values[2], pen->values[3],
		pen->values[4], pen->values[5], pen->values[6], pen->values[7],
		pen->hardware_ready ? gpiod_get_value_cansleep(pen->disable) : -ENODEV,
		pen->hardware_ready ? gpiod_get_value_cansleep(pen->supply) : -ENODEV,
		pen->hardware_ready ? gpiod_get_value_cansleep(pen->wake) : -ENODEV,
		pen->hardware_ready ? gpiod_get_value_cansleep(pen->scan) : -ENODEV,
		pen->hardware_ready ? gpiod_get_value_cansleep(pen->irq) : -ENODEV);
	mutex_unlock(&pen->lock);
	return count;
}
static DEVICE_ATTR_RO(status);

static struct attribute *pen_attrs[] = {
	&dev_attr_attach_once.attr,
	&dev_attr_attach_status.attr,
	&dev_attr_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pen);

static void pen_put_adapter(void *data)
{
	i2c_put_adapter(data);
}

static int pen_get_hardware(struct pen_power *pen)
{
	struct device *dev = pen->dev;
	struct platform_device *tlmm;
	struct device_node *bus;
	struct i2c_adapter *adapter;
	struct pinctrl *pinctrl;
	struct pinctrl_state *irq_state;
	int ret, irq;

	bus = of_find_node_by_path(PEN_I2C_PATH);
	if (!bus)
		return -ENODEV;
	adapter = i2c_get_adapter_by_fwnode(of_fwnode_handle(bus));
	of_node_put(bus);
	if (!adapter)
		return -ENODEV;
	ret = devm_add_action_or_reset(dev, pen_put_adapter, adapter);
	if (ret)
		return ret;
	if (!device_link_add(dev, adapter->dev.parent, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -EINVAL;
	if (!device_is_bound(adapter->dev.parent))
		return -ENODEV;
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	pen->i2c = devm_i2c_new_dummy_device(dev, adapter, 0x41);
	if (IS_ERR(pen->i2c))
		return dev_err_probe(dev, PTR_ERR(pen->i2c), "reserve CPS8601 address\n");

	bus = of_find_node_by_path(PEN_TLMM_PATH);
	if (!bus)
		return -ENODEV;
	tlmm = of_find_device_by_node(bus);
	of_node_put(bus);
	if (!tlmm)
		return -ENODEV;
	ret = -ENODEV;
	if (of_device_is_compatible(tlmm->dev.of_node, "qcom,sm8650-tlmm") &&
	    !strcmp(dev_name(&tlmm->dev), "f100000.pinctrl") &&
	    device_link_add(dev, &tlmm->dev, DL_FLAG_AUTOREMOVE_CONSUMER) &&
	    device_is_bound(&tlmm->dev))
		ret = 0;
	put_device(&tlmm->dev);
	if (ret)
		return ret;

	/* Acquire the charge-inhibit line before any power/wake output. */
	pen->disable = devm_gpiod_get(dev, "charge-disable", GPIOD_OUT_HIGH);
	if (IS_ERR(pen->disable))
		return dev_err_probe(dev, PTR_ERR(pen->disable), "charge-disable GPIO\n");
	pen->supply = devm_gpiod_get(dev, "supply", GPIOD_OUT_LOW);
	if (IS_ERR(pen->supply))
		return dev_err_probe(dev, PTR_ERR(pen->supply), "supply GPIO\n");
	pen->wake = devm_gpiod_get(dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(pen->wake))
		return dev_err_probe(dev, PTR_ERR(pen->wake), "wake GPIO\n");
	pen->scan = devm_gpiod_get(dev, "scan", GPIOD_OUT_LOW);
	if (IS_ERR(pen->scan))
		return dev_err_probe(dev, PTR_ERR(pen->scan), "scan GPIO\n");
	pen->irq = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(pen->irq))
		return dev_err_probe(dev, PTR_ERR(pen->irq), "IRQ GPIO\n");
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl))
		return dev_err_probe(dev, PTR_ERR(pinctrl), "IRQ pinctrl\n");
	irq_state = pinctrl_lookup_state(pinctrl, "irq-pull-up");
	if (IS_ERR(irq_state))
		return dev_err_probe(dev, PTR_ERR(irq_state), "IRQ bias state\n");
	ret = pinctrl_select_state(pinctrl, irq_state);
	if (ret)
		return dev_err_probe(dev, ret, "IRQ pull-up\n");
	irq = gpiod_to_irq(pen->irq);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(dev, irq, NULL, pen_irq_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT, PEN_NAME, pen);
	if (ret)
		return dev_err_probe(dev, ret, "IRQ request\n");
	pen->hardware_ready = true;
	return 0;
}

static int pen_setup(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_power *pen;
	int ret;

	if (!dev->parent || !device_is_bound(dev->parent) ||
	    !of_device_is_compatible(dev->parent->of_node, "qcom,pmic-glink"))
		return -ENODEV;
	pen = devm_kzalloc(dev, sizeof(*pen), GFP_KERNEL);
	if (!pen)
		return -ENOMEM;
	pen->dev = dev;
	pen->result = pen->cleanup = -ENODATA;
	pen->phase = "idle";
	mutex_init(&pen->lock);
	mutex_init(&pen->power_lock);
	spin_lock_init(&pen->ack_lock);
	init_completion(&pen->ack);
	init_completion(&pen->lost);
	init_completion(&pen->event);
	INIT_DELAYED_WORK(&pen->cutoff_work, pen_cutoff);
	platform_set_drvdata(pdev, pen);

	pen->glink = devm_pmic_glink_client_alloc(dev, PEN_OWNER, pen_reply, pen_transport, pen);
	if (IS_ERR(pen->glink))
		return PTR_ERR(pen->glink);
	pmic_glink_client_register(pen->glink);
	ret = pen_get_hardware(pen);
	if (ret)
		return ret;
	ret = device_init_wakeup(dev, true);
	if (ret)
		return ret;
	ret = sysfs_create_groups(&dev->kobj, pen_groups);
	if (ret) {
		device_init_wakeup(dev, false);
		return ret;
	}
	dev_info(dev, "CPS8601 pen power controller ready\n");
	return 0;
}

static int pen_probe(struct platform_device *pdev)
{
	pen_probe_result = pen_setup(pdev);
	if (pen_probe_result)
		dev_err(&pdev->dev, "setup failed: %d\n", pen_probe_result);
	return pen_probe_result;
}

static void pen_stop(struct platform_device *pdev)
{
	struct pen_power *pen = platform_get_drvdata(pdev);

	mutex_lock(&pen->lock);
	pen->suspended = true;
	if (pen->hardware_ready)
		pen_off(pen);
	mutex_unlock(&pen->lock);
}

static void pen_remove(struct platform_device *pdev)
{
	sysfs_remove_groups(&pdev->dev.kobj, pen_groups);
	pen_stop(pdev);
	device_init_wakeup(&pdev->dev, false);
}

static int pen_suspend(struct device *dev)
{
	struct pen_power *pen = dev_get_drvdata(dev);

	if (!mutex_trylock(&pen->lock))
		return -EBUSY;
	pen->suspended = true;
	if (pen->hardware_ready)
		pen_off(pen);
	mutex_unlock(&pen->lock);
	return 0;
}

static int pen_resume(struct device *dev)
{
	struct pen_power *pen = dev_get_drvdata(dev);

	mutex_lock(&pen->lock);
	if (pen->hardware_ready)
		pen_off(pen);
	pen->suspended = false;
	mutex_unlock(&pen->lock);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(pen_pm, pen_suspend, pen_resume);

static struct platform_driver pen_driver = {
	.probe = pen_probe,
	.remove = pen_remove,
	.shutdown = pen_stop,
	.prevent_deferred_probe = true,
	.driver = {
		.name = PEN_NAME,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
		.suppress_bind_attrs = true,
		.pm = pm_sleep_ptr(&pen_pm),
	},
};

static int __init pen_init(void)
{
	struct device_node *np;
	struct platform_device *parent;
	int ret = -ENODEV;

	if (!of_machine_is_compatible("oneplus,caihong"))
		return ret;
	np = of_find_node_by_path("/pmic-glink");
	if (!np)
		return ret;
	parent = of_find_device_by_node(np);
	if (!parent)
		goto put_node;
	/* A real child plus supplier link, with no DT/boot image changes. */
	if (!device_trylock(&parent->dev)) {
		ret = -EBUSY;
		goto put_parent;
	}
	if (!device_is_bound(&parent->dev) ||
	    !of_device_is_compatible(parent->dev.of_node, "qcom,pmic-glink"))
		goto unlock_parent;
	pen_device = platform_device_alloc(PEN_NAME, PLATFORM_DEVID_NONE);
	if (!pen_device) {
		ret = -ENOMEM;
		goto unlock_parent;
	}
	pen_device->dev.parent = &parent->dev;
	ret = platform_device_add(pen_device);
	if (ret) {
		platform_device_put(pen_device);
		pen_device = NULL;
		goto unlock_parent;
	}
	if (!device_link_add(&pen_device->dev, &parent->dev, DL_FLAG_AUTOPROBE_CONSUMER)) {
		ret = -EINVAL;
		goto unlock_parent;
	}
	ret = 0;
unlock_parent:
	device_unlock(&parent->dev);
put_parent:
	put_device(&parent->dev);
put_node:
	of_node_put(np);
	if (ret) {
		if (pen_device)
			platform_device_unregister(pen_device);
		return ret;
	}
	ret = pinctrl_register_mappings(pen_pinmaps, ARRAY_SIZE(pen_pinmaps));
	if (ret)
		goto unregister_device;
	gpiod_add_lookup_table(&pen_gpios);
	ret = platform_driver_register(&pen_driver);
	if (!ret && pen_probe_result) {
		ret = pen_probe_result;
		platform_driver_unregister(&pen_driver);
	}
	if (ret) {
		gpiod_remove_lookup_table(&pen_gpios);
		pinctrl_unregister_mappings(pen_pinmaps);
		goto unregister_device;
	}
	return 0;

unregister_device:
	platform_device_unregister(pen_device);
	return ret;
}
module_init(pen_init);

static void __exit pen_exit(void)
{
	platform_device_unregister(pen_device);
	platform_driver_unregister(&pen_driver);
	gpiod_remove_lookup_table(&pen_gpios);
	pinctrl_unregister_mappings(pen_pinmaps);
}
module_exit(pen_exit);

MODULE_DESCRIPTION("Caihong CPS8601 bounded pen charging controller");
MODULE_VERSION("10.0");
MODULE_LICENSE("GPL");
