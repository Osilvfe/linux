// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/*
 * OnePlus/Oplus Caihong pogo keyboard and touchpad
 *
 * The accessory uses an Oplus one-wire framing protocol over a half-duplex
 * UART.  This keyboard has no keyboard backlight.  It does expose the
 * CapsLock indicator as an EV_LED so the input layer can keep that status
 * lamp usable; the desktop keyboard-backlight action is removed from the
 * image separately.
 */

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define POGO_SYNC			0x55
#define POGO_TAIL			0xaa
#define POGO_START			0xf1
#define POGO_REPEAT			0xf2
#define POGO_END			0xfe
#define POGO_KEYBOARD_ADDRESS		0xa1
#define POGO_TABLET_ADDRESS		0xa2

#define POGO_CMD_KEYBOARD		0x01
#define POGO_CMD_MEDIA			0x02
#define POGO_CMD_TOUCHPAD		0x03
#define POGO_CMD_PARAM_SET		0x20
#define POGO_CMD_HEARTBEAT		0x2f
#define POGO_CMD_USER_GENERAL		0x3a
#define POGO_CMD_USER_GENERAL_ACK	0x3b

#define POGO_PARAM_LED			0x0d
#define POGO_GENERAL_HOST_SLEEP		0x02
#define POGO_GENERAL_TOUCHPAD_DISABLE	0x11
#define POGO_TOUCHPAD_RESTORE_TRIES	1
#define POGO_TOUCHPAD_RESTORE_DELAY_MS	5
#define POGO_DISCONNECT_TIMEOUT_MS	2000
#define POGO_TX_SYNC_COUNT		8
#define POGO_TX_FRAME_MAX		32

#define POGO_CRC_INIT			0xc596
#define POGO_MAX_PAYLOAD		255
#define POGO_MAX_FRAME			(5 + POGO_MAX_PAYLOAD + 2 + 1 + 4)
#define POGO_MAX_TOUCHES		5
#define POGO_SEARCH_USAGE		0x72

struct oneplus_pogo_media_map {
	u16 usage;
	u16 keycode;
	u16 fn_keycode;
};

static const struct oneplus_pogo_media_map oneplus_pogo_media_map[] = {
	{ 0x0070, KEY_F1, KEY_BRIGHTNESSDOWN },
	{ 0x006f, KEY_F2, KEY_BRIGHTNESSUP },
	/* F3 through F6 and search arrive on the keyboard page. */
	{ 0x00b6, KEY_F7, KEY_PREVIOUSSONG },
	{ 0x00cd, KEY_F8, KEY_PLAYPAUSE },
	{ 0x00b5, KEY_F9, KEY_NEXTSONG },
	{ 0x00e2, KEY_F10, KEY_MUTE },
	{ 0x00ea, KEY_F11, KEY_VOLUMEDOWN },
	{ 0x00e9, KEY_F12, KEY_VOLUMEUP },
	/* The physical Esc key reports Android/consumer AC Back on Caihong. */
	{ 0x0224, KEY_ESC },
	{ 0x0244, KEY_APPSELECT },
};

/* Captured on Caihong: the touchpad key alternates between 0x6b and 0x6c. */
static const struct oneplus_pogo_media_map oneplus_pogo_keyboard_map[] = {
	{ 0x68, KEY_F3, KEY_MICMUTE },
	{ 0x6b, KEY_F4, KEY_TOUCHPAD_TOGGLE },
	{ 0x6c, KEY_F4, KEY_TOUCHPAD_TOGGLE },
	{ 0x46, KEY_F5, KEY_SYSRQ },
	{ 0x73, KEY_F6, KEY_SCREENLOCK },
	{ POGO_SEARCH_USAGE, KEY_FN },
};

/* USB HID keyboard usages used by the Oplus accessory protocol. */
static const u16 oneplus_pogo_keycode[256] = {
	[4] = KEY_A,		[5] = KEY_B,		[6] = KEY_C,
	[7] = KEY_D,		[8] = KEY_E,		[9] = KEY_F,
	[10] = KEY_G,		[11] = KEY_H,		[12] = KEY_I,
	[13] = KEY_J,		[14] = KEY_K,		[15] = KEY_L,
	[16] = KEY_M,		[17] = KEY_N,		[18] = KEY_O,
	[19] = KEY_P,		[20] = KEY_Q,		[21] = KEY_R,
	[22] = KEY_S,		[23] = KEY_T,		[24] = KEY_U,
	[25] = KEY_V,		[26] = KEY_W,		[27] = KEY_X,
	[28] = KEY_Y,		[29] = KEY_Z,		[30] = KEY_1,
	[31] = KEY_2,		[32] = KEY_3,		[33] = KEY_4,
	[34] = KEY_5,		[35] = KEY_6,		[36] = KEY_7,
	[37] = KEY_8,		[38] = KEY_9,		[39] = KEY_0,
	[40] = KEY_ENTER,	[41] = KEY_ESC,	[42] = KEY_BACKSPACE,
	[43] = KEY_TAB,		[44] = KEY_SPACE,	[45] = KEY_MINUS,
	[46] = KEY_EQUAL,	[47] = KEY_LEFTBRACE,	[48] = KEY_RIGHTBRACE,
	[49] = KEY_BACKSLASH,	[50] = KEY_BACKSLASH,	[51] = KEY_SEMICOLON,
	[52] = KEY_APOSTROPHE,	[53] = KEY_GRAVE,	[54] = KEY_COMMA,
	[55] = KEY_DOT,		[56] = KEY_SLASH,	[57] = KEY_CAPSLOCK,
	[58] = KEY_F1,		[59] = KEY_F2,		[60] = KEY_F3,
	[61] = KEY_F4,		[62] = KEY_F5,		[63] = KEY_F6,
	[64] = KEY_F7,		[65] = KEY_F8,		[66] = KEY_F9,
	[67] = KEY_F10,	[68] = KEY_F11,	[69] = KEY_F12,
	[70] = KEY_SYSRQ,	[71] = KEY_SCROLLLOCK,	[72] = KEY_PAUSE,
	[73] = KEY_INSERT,	[74] = KEY_HOME,	[75] = KEY_PAGEUP,
	[76] = KEY_DELETE,	[77] = KEY_END,		[78] = KEY_PAGEDOWN,
	[79] = KEY_RIGHT,	[80] = KEY_LEFT,	[81] = KEY_DOWN,
	[82] = KEY_UP,		[83] = KEY_NUMLOCK,	[84] = KEY_KPSLASH,
	[85] = KEY_KPASTERISK,	[86] = KEY_KPMINUS,	[87] = KEY_KPPLUS,
	[88] = KEY_KPENTER,	[89] = KEY_KP1,	[90] = KEY_KP2,
	[91] = KEY_KP3,	[92] = KEY_KP4,	[93] = KEY_KP5,
	[94] = KEY_KP6,	[95] = KEY_KP7,	[96] = KEY_KP8,
	[97] = KEY_KP9,	[98] = KEY_KP0,	[99] = KEY_KPDOT,
	[224] = KEY_LEFTCTRL,	[225] = KEY_LEFTSHIFT,
	[226] = KEY_LEFTALT,	[227] = KEY_LEFTMETA,
	[228] = KEY_RIGHTCTRL,	[229] = KEY_RIGHTSHIFT,
	[230] = KEY_RIGHTALT,	[231] = KEY_RIGHTMETA,
};

struct oneplus_pogo {
	struct serdev_device *serdev;
	struct input_dev *keyboard;
	struct input_dev *touchpad;
	struct gpio_desc *power_gpio;
	struct gpio_desc *wake_gpio;
	struct gpio_desc *tx_enable_gpio;
	struct regulator *vcc;
	struct work_struct led_work;
	struct delayed_work setup_work;
	struct delayed_work touchpad_work;
	struct delayed_work presence_work;
	/* Serializes receive parsing with touchpad state changes. */
	struct mutex lock;
	/* Serializes half-duplex UART transmissions without blocking RX parsing. */
	struct mutex tx_lock;

	u32 baud;
	u32 x_max;
	u32 y_max;
	u32 x_res;
	u32 y_res;
	u16 crc_init;

	u8 old_keys[8];
	u16 old_keycodes[6];
	u8 old_media[4];
	/* Remember the code chosen on key-down even if Fn is released first. */
	u16 old_media_keycodes[2];
	bool fn_down;
	u8 frame[POGO_MAX_FRAME];
	size_t frame_len;
	size_t expected_len;
	u8 sync_count;
	bool receiving;
	bool baud_locked;
	int touchpad_disabled;
	bool capslock_led;
	bool attached;
	bool touchpad_target_enabled;
	u8 touchpad_restore_left;
};

static u16 oneplus_pogo_crc(u16 crc, const u8 *data, size_t len)
{
	int bit;

	while (len--) {
		u8 byte = *data++;

		for (bit = 0; bit < 8; bit++) {
			if ((((crc & 0x8000) >> 8) ^ (byte & 0x80)) != 0)
				crc = (crc << 1) ^ 0x8005;
			else
				crc <<= 1;
			byte <<= 1;
		}
	}

	return crc;
}

static size_t oneplus_pogo_build_frame(struct oneplus_pogo *pogo, u8 command,
					const u8 *payload, size_t payload_len,
					u8 *frame)
{
	size_t pos = 0;
	u16 crc;

	memset(frame, POGO_SYNC, POGO_TX_SYNC_COUNT);
	pos += POGO_TX_SYNC_COUNT;
	frame[pos++] = POGO_START;
	frame[pos++] = POGO_TABLET_ADDRESS;
	frame[pos++] = POGO_KEYBOARD_ADDRESS;
	frame[pos++] = command;
	frame[pos++] = payload_len;
	memcpy(frame + pos, payload, payload_len);
	pos += payload_len;

	crc = oneplus_pogo_crc(pogo->crc_init,
				frame + POGO_TX_SYNC_COUNT, payload_len + 5);
	put_unaligned_be16(crc, frame + pos);
	pos += sizeof(crc);
	frame[pos++] = POGO_END;
	memset(frame + pos, POGO_TAIL, 4);
	pos += 4;

	return pos;
}

static int oneplus_pogo_send_locked(struct oneplus_pogo *pogo, u8 command,
				    const u8 *payload, size_t payload_len)
{
	u8 frame[POGO_TX_FRAME_MAX];
	ssize_t written;
	size_t frame_len;
	int ret = 0;

	if (!pogo->tx_enable_gpio)
		return -EOPNOTSUPP;
	if (payload_len + POGO_TX_SYNC_COUNT + 12 > sizeof(frame))
		return -EMSGSIZE;

	frame_len = oneplus_pogo_build_frame(pogo, command, payload,
						 payload_len, frame);

	gpiod_set_value_cansleep(pogo->tx_enable_gpio, 1);
	usleep_range(450, 550);
	written = serdev_device_write(pogo->serdev, frame, frame_len,
				      msecs_to_jiffies(20));
	if (written == frame_len) {
		serdev_device_wait_until_sent(pogo->serdev,
					      msecs_to_jiffies(20));
	} else {
		ret = written < 0 ? written : -EIO;
	}
	usleep_range(300, 400);
	gpiod_set_value_cansleep(pogo->tx_enable_gpio, 0);

	return ret;
}

static int oneplus_pogo_send(struct oneplus_pogo *pogo, u8 command,
				     const u8 *payload, size_t payload_len)
{
	int ret;

	mutex_lock(&pogo->tx_lock);
	ret = oneplus_pogo_send_locked(pogo, command, payload, payload_len);
	mutex_unlock(&pogo->tx_lock);
	return ret;
}

static int oneplus_pogo_set_host_awake(struct oneplus_pogo *pogo)
{
	u8 payload[] = { POGO_GENERAL_HOST_SLEEP, 0x01, 0x00 };

	return oneplus_pogo_send(pogo, POGO_CMD_USER_GENERAL, payload,
				 sizeof(payload));
}

/* Caller holds pogo->tx_lock. */
static int oneplus_pogo_set_touchpad_enabled_tx_locked(
							struct oneplus_pogo *pogo,
							bool enabled)
{
	u8 payload[] = {
		POGO_GENERAL_TOUCHPAD_DISABLE, 0x01, enabled ? 0x00 : 0x01,
	};

	return oneplus_pogo_send_locked(pogo, POGO_CMD_USER_GENERAL, payload,
					 sizeof(payload));
}

static int oneplus_pogo_set_touchpad_enabled(struct oneplus_pogo *pogo,
						      bool enabled)
{
	int ret;

	mutex_lock(&pogo->tx_lock);
	ret = oneplus_pogo_set_touchpad_enabled_tx_locked(pogo, enabled);
	mutex_unlock(&pogo->tx_lock);

	return ret;
}

static void oneplus_pogo_queue_touchpad_restore(struct oneplus_pogo *pogo)
{
	/* RX already holds the mutex: transmit only after leaving that path. */
	pogo->touchpad_restore_left = POGO_TOUCHPAD_RESTORE_TRIES;
	mod_delayed_work(system_dfl_wq, &pogo->touchpad_work,
			 msecs_to_jiffies(POGO_TOUCHPAD_RESTORE_DELAY_MS));
}

static void oneplus_pogo_touchpad_work(struct work_struct *work)
{
	struct oneplus_pogo *pogo = container_of(to_delayed_work(work),
						  struct oneplus_pogo, touchpad_work);
	bool enabled;
	int ret;

	mutex_lock(&pogo->lock);
	if (!pogo->touchpad_restore_left) {
		mutex_unlock(&pogo->lock);
		return;
	}
	pogo->touchpad_restore_left--;
	enabled = pogo->touchpad_target_enabled;
	mutex_unlock(&pogo->lock);

	/* TX has its own lock so incoming touch frames continue to be parsed. */
	ret = oneplus_pogo_set_touchpad_enabled(pogo, enabled);
	mutex_lock(&pogo->lock);
	if (ret)
		dev_dbg_ratelimited(&pogo->serdev->dev,
				    "failed to restore touchpad state: %d\n", ret);
	if (pogo->touchpad_restore_left)
		mod_delayed_work(system_dfl_wq, &pogo->touchpad_work,
				 msecs_to_jiffies(POGO_TOUCHPAD_RESTORE_DELAY_MS));
	mutex_unlock(&pogo->lock);
}

static void oneplus_pogo_stop_delayed_work(void *data)
{
	struct oneplus_pogo *pogo = data;

	/* RX may still run until devm closes serdev; disallow any new queueing. */
	disable_delayed_work_sync(&pogo->touchpad_work);
	disable_delayed_work_sync(&pogo->presence_work);
}

static void oneplus_pogo_setup_work(struct work_struct *work)
{
	struct oneplus_pogo *pogo = container_of(to_delayed_work(work),
						  struct oneplus_pogo, setup_work);
	int i, ret;

	/*
	 * The keyboard MCU keeps its LCD and touchpad states across some warm
	 * boots.  Android normally clears them from the pogo userspace service;
	 * mainline has no such service, so establish the usable defaults here.
	 */
	ret = oneplus_pogo_set_host_awake(pogo);
	if (ret)
		dev_dbg(&pogo->serdev->dev,
			"failed to report host-awake state: %d\n", ret);

	msleep(30);
	for (i = 0; i < 3; i++) {
		bool enabled;

		mutex_lock(&pogo->lock);
		enabled = pogo->touchpad_target_enabled;
		mutex_unlock(&pogo->lock);
		ret = oneplus_pogo_set_touchpad_enabled(pogo, enabled);
		if (ret)
			dev_dbg(&pogo->serdev->dev,
				"failed to set touchpad state: %d\n", ret);
		msleep(30);
	}
}

static void oneplus_pogo_led_work(struct work_struct *work)
{
	struct oneplus_pogo *pogo = container_of(work, struct oneplus_pogo,
							led_work);
	u8 payload[] = {
		POGO_PARAM_LED, 0x04, READ_ONCE(pogo->capslock_led), 0, 0, 0,
	};
	int ret;

	/* Keep the LED command away from the MCU heartbeat transaction. */
	usleep_range(15000, 16000);
	ret = oneplus_pogo_send(pogo, POGO_CMD_PARAM_SET, payload,
					sizeof(payload));
	if (ret)
		dev_dbg(&pogo->serdev->dev, "failed to set CapsLock LED: %d\n",
			ret);
}

static int oneplus_pogo_input_event(struct input_dev *input,
					unsigned int type, unsigned int code,
					int value)
{
	struct oneplus_pogo *pogo = input_get_drvdata(input);

	if (type != EV_LED || code != LED_CAPSL)
		return -EINVAL;

	WRITE_ONCE(pogo->capslock_led, value != 0);
	queue_work(system_dfl_wq, &pogo->led_work);

	return 0;
}

static bool oneplus_pogo_key_present(const u8 *report, u8 usage)
{
	int i;

	for (i = 2; i < 8; i++)
		if (report[i] == usage)
			return true;

	return false;
}

static u16 oneplus_pogo_keyboard_key(u8 usage, bool fn_down)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(oneplus_pogo_keyboard_map); i++)
		if (oneplus_pogo_keyboard_map[i].usage == usage) {
			if (fn_down && oneplus_pogo_keyboard_map[i].fn_keycode)
				return oneplus_pogo_keyboard_map[i].fn_keycode;
			return oneplus_pogo_keyboard_map[i].keycode;
		}
	return oneplus_pogo_keycode[usage];
}

static void oneplus_pogo_report_keyboard(struct oneplus_pogo *pogo,
					 const u8 *payload, size_t len)
{
	u8 report[8];
	u16 keycodes[6] = { 0 };
	bool report_changed;
	int i, j;

	if (len < sizeof(report))
		return;

	memcpy(report, payload, sizeof(report));
	report_changed = memcmp(report, pogo->old_keys, sizeof(report)) != 0;
	/* Compute Fn before translating any slot, including a simultaneous key. */
	pogo->fn_down = oneplus_pogo_key_present(report, POGO_SEARCH_USAGE);
	/* The MCU toggles its touchpad state for this usage.  Plain F4 is kept as
	 * an F4 key, while Fn+F4 intentionally toggles the target state.  Restore
	 * that target once after the report; repeated synchronous writes here used
	 * to hold the UART receive lock long enough to pause pointer events.
	 */
	if (oneplus_pogo_key_present(report, 0x6b) !=
		oneplus_pogo_key_present(pogo->old_keys, 0x6b) ||
	    oneplus_pogo_key_present(report, 0x6c) !=
		oneplus_pogo_key_present(pogo->old_keys, 0x6c)) {
		if (pogo->fn_down)
			pogo->touchpad_target_enabled =
				!pogo->touchpad_target_enabled;
	}

	for (i = 0; i < 8; i++)
		input_report_key(pogo->keyboard, oneplus_pogo_keycode[224 + i],
				 !!(report[0] & BIT(i)));

	for (i = 2; i < 8; i++) {
		u8 old = pogo->old_keys[i];

		if (old > 3 && !oneplus_pogo_key_present(report, old) &&
		    pogo->old_keycodes[i - 2])
			input_report_key(pogo->keyboard,
					 pogo->old_keycodes[i - 2], 0);
	}
	for (i = 2; i < 8; i++) {
		u8 usage = report[i];

		if (usage <= 3)
			continue;
		for (j = 2; j < i; j++)
			if (report[j] == usage)
				break;
		if (j < i)
			continue;
		for (j = 2; j < 8; j++)
			if (pogo->old_keys[j] == usage)
				break;
		if (j < 8) {
			keycodes[i - 2] = pogo->old_keycodes[j - 2];
			continue;
		}
		keycodes[i - 2] = oneplus_pogo_keyboard_key(usage, pogo->fn_down);
		if (keycodes[i - 2])
			input_report_key(pogo->keyboard, keycodes[i - 2], 1);
	}

	memcpy(pogo->old_keys, report, sizeof(report));
	memcpy(pogo->old_keycodes, keycodes, sizeof(keycodes));
	if (report_changed)
		oneplus_pogo_queue_touchpad_restore(pogo);
	input_sync(pogo->keyboard);
}

static u16 oneplus_pogo_media_key(u16 usage, bool fn_down)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(oneplus_pogo_media_map); i++)
		if (oneplus_pogo_media_map[i].usage == usage) {
			if (fn_down && oneplus_pogo_media_map[i].fn_keycode)
				return oneplus_pogo_media_map[i].fn_keycode;
			return oneplus_pogo_media_map[i].keycode;
		}

	return KEY_RESERVED;
}

static bool oneplus_pogo_media_present(const u8 *report, u16 usage)
{
	int i;

	for (i = 0; i < 2; i++)
		if (get_unaligned_le16(report + i * 2) == usage)
			return true;

	return false;
}

static void oneplus_pogo_report_media(struct oneplus_pogo *pogo,
				      const u8 *payload, size_t len)
{
	u8 report[4];
	u16 keycodes[2] = { 0 };
	bool report_changed;
	int i, j;

	if (len < sizeof(report))
		return;

	memcpy(report, payload, sizeof(report));
	report_changed = memcmp(report, pogo->old_media, sizeof(report)) != 0;

	for (i = 0; i < 2; i++) {
		u16 old = get_unaligned_le16(pogo->old_media + i * 2);

		if (old && (i == 0 || old != get_unaligned_le16(pogo->old_media)) &&
		    !oneplus_pogo_media_present(report, old)) {
			if (pogo->old_media_keycodes[i] != KEY_RESERVED)
				input_report_key(pogo->keyboard, pogo->old_media_keycodes[i], 0);
		}
	}
	for (i = 0; i < 2; i++) {
		u16 usage = get_unaligned_le16(report + i * 2);

		if (!usage || (i && usage == get_unaligned_le16(report)))
			continue;
		/* Reports may reorder their two slots while a key remains held. */
		for (j = 0; j < 2; j++)
			if (get_unaligned_le16(pogo->old_media + j * 2) == usage)
				break;
		if (j < 2) {
			keycodes[i] = pogo->old_media_keycodes[j];
			continue;
		}
		keycodes[i] = oneplus_pogo_media_key(usage, pogo->fn_down);
		if (keycodes[i] != KEY_RESERVED)
			input_report_key(pogo->keyboard, keycodes[i], 1);
	}

	memcpy(pogo->old_media, report, sizeof(report));
	memcpy(pogo->old_media_keycodes, keycodes, sizeof(keycodes));
	if (report_changed)
		oneplus_pogo_queue_touchpad_restore(pogo);
	input_sync(pogo->keyboard);
}

static void oneplus_pogo_report_touchpad(struct oneplus_pogo *pogo,
					 const u8 *payload, size_t len)
{
	u8 fingers, buttons;
	unsigned int active = 0;
	int i;

	/* Oplus format: contacts, finger count, then button bitmap. */
	if (len < 2)
		return;

	fingers = payload[len - 2];
	buttons = payload[len - 1];
	if (fingers > POGO_MAX_TOUCHES || fingers * 5 > len - 2)
		return;
	for (i = 0; i < fingers; i++) {
		const u8 *contact = payload + i * 5;
		u8 id = (contact[0] >> 4) & 0x0f;
		bool down = contact[0] & BIT(1);
		u16 x = get_unaligned_le16(contact + 1);
		u16 y = get_unaligned_le16(contact + 3);

		if (id >= POGO_MAX_TOUCHES)
			continue;

		input_mt_slot(pogo->touchpad, id);
		input_mt_report_slot_state(pogo->touchpad, MT_TOOL_FINGER, down);
		if (down) {
			input_report_abs(pogo->touchpad, ABS_MT_POSITION_X,
					 min_t(u16, x, pogo->x_max));
			input_report_abs(pogo->touchpad, ABS_MT_POSITION_Y,
					 min_t(u16, y, pogo->y_max));
			active++;
		}
	}

	input_report_key(pogo->touchpad, BTN_TOUCH, active != 0);
	input_report_key(pogo->touchpad, BTN_LEFT, buttons & BIT(0));
	input_report_key(pogo->touchpad, BTN_RIGHT, buttons & BIT(1));
	input_mt_sync_frame(pogo->touchpad);
	input_sync(pogo->touchpad);
}

static void oneplus_pogo_presence_work(struct work_struct *work)
{
	struct oneplus_pogo *pogo = container_of(to_delayed_work(work),
						  struct oneplus_pogo, presence_work);

	mutex_lock(&pogo->lock);
	if (pogo->attached) {
		pogo->attached = false;
		input_report_switch(pogo->keyboard, SW_TABLET_MODE, 1);
		input_sync(pogo->keyboard);
		dev_info(&pogo->serdev->dev, "keyboard detached; tablet mode on\n");
	}
	mutex_unlock(&pogo->lock);
}

static void oneplus_pogo_handle_frame(struct oneplus_pogo *pogo)
{
	const u8 *payload;
	u8 *frame = pogo->frame;
	u8 command, payload_len;
	u16 expected_crc, received_crc;

	if (pogo->frame_len < 12 ||
	    (frame[0] != POGO_START && frame[0] != POGO_REPEAT) ||
	    frame[1] != POGO_KEYBOARD_ADDRESS ||
	    frame[2] != POGO_TABLET_ADDRESS) {
		return;
	}

	command = frame[3];
	payload_len = frame[4];
	if (pogo->frame_len != payload_len + 12)
		return;

	received_crc = get_unaligned_be16(frame + 5 + payload_len);
	expected_crc = oneplus_pogo_crc(pogo->crc_init, frame, payload_len + 5);
	if (received_crc != expected_crc)
		return;

	if (frame[7 + payload_len] != POGO_END ||
	    frame[8 + payload_len] != POGO_TAIL ||
	    frame[9 + payload_len] != POGO_TAIL ||
	    frame[10 + payload_len] != POGO_TAIL ||
	    frame[11 + payload_len] != POGO_TAIL) {
		return;
	}

	if (!pogo->baud_locked) {
		pogo->baud_locked = true;
		dev_info(&pogo->serdev->dev,
			 "valid pogo frame; locked at %u baud\n", pogo->baud);
	}
	if (!pogo->attached) {
		pogo->attached = true;
		input_report_switch(pogo->keyboard, SW_TABLET_MODE, 0);
		input_sync(pogo->keyboard);
		dev_info(&pogo->serdev->dev,
			 "keyboard attached; tablet mode off\n");
	}
	mod_delayed_work(system_dfl_wq, &pogo->presence_work,
			 msecs_to_jiffies(POGO_DISCONNECT_TIMEOUT_MS));

	payload = frame + 5;
	switch (command) {
	case POGO_CMD_KEYBOARD:
		oneplus_pogo_report_keyboard(pogo, payload, payload_len);
		break;
	case POGO_CMD_MEDIA:
		oneplus_pogo_report_media(pogo, payload, payload_len);
		break;
	case POGO_CMD_TOUCHPAD:
		oneplus_pogo_report_touchpad(pogo, payload, payload_len);
		break;
	case POGO_CMD_HEARTBEAT:
		/*
		 * Oplus power-on (sub-command 0x01) and heartbeat (0x05)
		 * packets carry the persistent touchpad-disable state after the
		 * keyboard identity and MAC address.
		 */
		if (payload_len > 9 && payload[0] == 0x01 &&
		    payload[1] == 0x02)
			pogo->touchpad_disabled = !!payload[9];
		else if (payload_len > 10 && payload[0] == 0x05 &&
			 payload[1] == 0x02)
			pogo->touchpad_disabled = !!payload[10];
		break;
	case POGO_CMD_USER_GENERAL_ACK:
		/* Touchpad-status response: sub-command, length, state. */
		if (payload_len >= 3 && payload[0] == 0x0c &&
		    payload[1] >= 1)
			pogo->touchpad_disabled = !!payload[2];
		break;
	default:
		dev_dbg_ratelimited(&pogo->serdev->dev,
				    "unhandled command 0x%02x\n", command);
		break;
	}
}

static void oneplus_pogo_reset_receiver(struct oneplus_pogo *pogo)
{
	pogo->receiving = false;
	pogo->frame_len = 0;
	pogo->expected_len = 0;
}

static void oneplus_pogo_receive_byte(struct oneplus_pogo *pogo, u8 byte)
{
	if (!pogo->receiving) {
		if (byte == POGO_SYNC) {
			if (pogo->sync_count != 0xff)
				pogo->sync_count++;
			return;
		}

		if ((byte == POGO_START || byte == POGO_REPEAT) &&
		    pogo->sync_count >= 4) {
			pogo->receiving = true;
			pogo->frame_len = 1;
			pogo->frame[0] = byte;
			pogo->expected_len = 0;
		}
		pogo->sync_count = 0;
		return;
	}

	if (pogo->frame_len >= sizeof(pogo->frame)) {
		oneplus_pogo_reset_receiver(pogo);
		pogo->sync_count = byte == POGO_SYNC;
		return;
	}

	pogo->frame[pogo->frame_len++] = byte;
	if (pogo->frame_len == 5) {
		pogo->expected_len = pogo->frame[4] + 12;
		if (pogo->expected_len > sizeof(pogo->frame)) {
			oneplus_pogo_reset_receiver(pogo);
			return;
		}
	}

	if (pogo->expected_len && pogo->frame_len == pogo->expected_len) {
		oneplus_pogo_handle_frame(pogo);
		oneplus_pogo_reset_receiver(pogo);
	}
}

static size_t oneplus_pogo_receive(struct serdev_device *serdev,
				   const u8 *data, size_t count)
{
	struct oneplus_pogo *pogo = serdev_device_get_drvdata(serdev);
	size_t i;

	mutex_lock(&pogo->lock);
	for (i = 0; i < count; i++)
		oneplus_pogo_receive_byte(pogo, data[i]);
	mutex_unlock(&pogo->lock);

	return count;
}

static void oneplus_pogo_write_wakeup(struct serdev_device *serdev)
{
	serdev_device_write_wakeup(serdev);
}

static const struct serdev_device_ops oneplus_pogo_serdev_ops = {
	.receive_buf = oneplus_pogo_receive,
	.write_wakeup = oneplus_pogo_write_wakeup,
};

static ssize_t touchpad_enabled_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct oneplus_pogo *pogo = serdev_device_get_drvdata(serdev);
	int enabled;

	if (!pogo)
		return -ENODEV;

	mutex_lock(&pogo->lock);
	enabled = pogo->touchpad_disabled < 0 ? -1 : !pogo->touchpad_disabled;
	mutex_unlock(&pogo->lock);

	return sysfs_emit(buf, "%d\n", enabled);
}

static ssize_t touchpad_enabled_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct oneplus_pogo *pogo = serdev_device_get_drvdata(serdev);
	bool enabled;
	int ret;

	if (!pogo)
		return -ENODEV;

	ret = kstrtobool(buf, &enabled);
	if (ret)
		return ret;

	ret = oneplus_pogo_set_touchpad_enabled(pogo, enabled);
	if (!ret) {
		mutex_lock(&pogo->lock);
		pogo->touchpad_target_enabled = enabled;
		pogo->touchpad_restore_left = 0;
		mutex_unlock(&pogo->lock);
	}
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(touchpad_enabled);

static ssize_t attached_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct oneplus_pogo *pogo = serdev_device_get_drvdata(serdev);
	bool attached;

	if (!pogo)
		return -ENODEV;

	mutex_lock(&pogo->lock);
	attached = pogo->attached;
	mutex_unlock(&pogo->lock);

	return sysfs_emit(buf, "%u\n", attached);
}
static DEVICE_ATTR_RO(attached);

static struct attribute *oneplus_pogo_attrs[] = {
	&dev_attr_attached.attr,
	&dev_attr_touchpad_enabled.attr,
	NULL,
};
ATTRIBUTE_GROUPS(oneplus_pogo);

static int oneplus_pogo_register_inputs(struct oneplus_pogo *pogo)
{
	struct device *dev = &pogo->serdev->dev;
	int i, ret;

	pogo->keyboard = devm_input_allocate_device(dev);
	if (!pogo->keyboard)
		return -ENOMEM;

	pogo->keyboard->name = "OnePlus Pogo Keyboard";
	pogo->keyboard->phys = "oneplus-pogo/input0";
	pogo->keyboard->id.bustype = BUS_RS232;
	pogo->keyboard->id.vendor = 0x22d9;
	pogo->keyboard->id.product = 0x3869;
	pogo->keyboard->id.version = 0x0010;
	pogo->keyboard->event = oneplus_pogo_input_event;
	input_set_drvdata(pogo->keyboard, pogo);
	input_set_capability(pogo->keyboard, EV_LED, LED_CAPSL);
	input_set_capability(pogo->keyboard, EV_SW, SW_TABLET_MODE);
	for (i = 0; i < ARRAY_SIZE(oneplus_pogo_keycode); i++)
		if (oneplus_pogo_keycode[i])
			input_set_capability(pogo->keyboard, EV_KEY,
					     oneplus_pogo_keycode[i]);
	for (i = 0; i < ARRAY_SIZE(oneplus_pogo_media_map); i++) {
		input_set_capability(pogo->keyboard, EV_KEY,
				     oneplus_pogo_media_map[i].keycode);
		if (oneplus_pogo_media_map[i].fn_keycode)
			input_set_capability(pogo->keyboard, EV_KEY,
					     oneplus_pogo_media_map[i].fn_keycode);
	}
	for (i = 0; i < ARRAY_SIZE(oneplus_pogo_keyboard_map); i++) {
		input_set_capability(pogo->keyboard, EV_KEY,
				     oneplus_pogo_keyboard_map[i].keycode);
		if (oneplus_pogo_keyboard_map[i].fn_keycode)
			input_set_capability(pogo->keyboard, EV_KEY,
					     oneplus_pogo_keyboard_map[i].fn_keycode);
	}

	ret = input_register_device(pogo->keyboard);
	if (ret)
		return ret;

	pogo->touchpad = devm_input_allocate_device(dev);
	if (!pogo->touchpad)
		return -ENOMEM;

	pogo->touchpad->name = "OnePlus Pogo Touchpad";
	pogo->touchpad->phys = "oneplus-pogo/input1";
	pogo->touchpad->id.bustype = BUS_RS232;
	pogo->touchpad->id.vendor = 0x22d9;
	pogo->touchpad->id.product = 0x3869;
	pogo->touchpad->id.version = 0x0010;
	__set_bit(INPUT_PROP_POINTER, pogo->touchpad->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, pogo->touchpad->propbit);

	input_set_capability(pogo->touchpad, EV_KEY, BTN_TOUCH);
	input_set_capability(pogo->touchpad, EV_KEY, BTN_LEFT);
	input_set_capability(pogo->touchpad, EV_KEY, BTN_RIGHT);
	input_set_abs_params(pogo->touchpad, ABS_MT_POSITION_X,
			     0, pogo->x_max, 0, 0);
	input_set_abs_params(pogo->touchpad, ABS_MT_POSITION_Y,
			     0, pogo->y_max, 0, 0);
	input_abs_set_res(pogo->touchpad, ABS_MT_POSITION_X, pogo->x_res);
	input_abs_set_res(pogo->touchpad, ABS_MT_POSITION_Y, pogo->y_res);

	ret = input_mt_init_slots(pogo->touchpad, POGO_MAX_TOUCHES,
				  INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	return input_register_device(pogo->touchpad);
}

static int oneplus_pogo_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct oneplus_pogo *pogo;
	u32 value;
	int wake_before_power = -1;
	int ret;

	pogo = devm_kzalloc(dev, sizeof(*pogo), GFP_KERNEL);
	if (!pogo)
		return -ENOMEM;

	pogo->serdev = serdev;
	pogo->x_max = 2764;
	pogo->y_max = 1630;
	pogo->x_res = 23;
	pogo->y_res = 23;
	pogo->crc_init = POGO_CRC_INIT;
	pogo->baud = 921600;
	pogo->touchpad_disabled = -1;
	pogo->touchpad_target_enabled = true;
	mutex_init(&pogo->lock);
	mutex_init(&pogo->tx_lock);
	INIT_WORK(&pogo->led_work, oneplus_pogo_led_work);
	INIT_DELAYED_WORK(&pogo->setup_work, oneplus_pogo_setup_work);
	INIT_DELAYED_WORK(&pogo->touchpad_work, oneplus_pogo_touchpad_work);
	INIT_DELAYED_WORK(&pogo->presence_work, oneplus_pogo_presence_work);
	serdev_device_set_drvdata(serdev, pogo);

	device_property_read_u32(dev, "current-speed", &pogo->baud);
	device_property_read_u32(dev, "touchpad-size-x", &pogo->x_max);
	device_property_read_u32(dev, "touchpad-size-y", &pogo->y_max);
	device_property_read_u32(dev, "touchpad-resolution-x", &pogo->x_res);
	device_property_read_u32(dev, "touchpad-resolution-y", &pogo->y_res);
	if (!device_property_read_u32(dev, "oneplus,crc-ibm-init", &value)) {
		if (value > 0xffff)
			return dev_err_probe(dev, -EINVAL, "invalid CRC initial value\n");
		pogo->crc_init = value;
	}
	if (!pogo->baud)
		return dev_err_probe(dev, -EINVAL, "invalid baud rate\n");

	pogo->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(pogo->power_gpio))
		return dev_err_probe(dev, PTR_ERR(pogo->power_gpio), "power GPIO\n");

	pogo->wake_gpio = devm_gpiod_get_optional(dev, "wake", GPIOD_IN);
	if (IS_ERR(pogo->wake_gpio))
		return dev_err_probe(dev, PTR_ERR(pogo->wake_gpio), "wake GPIO\n");

	pogo->tx_enable_gpio = devm_gpiod_get_optional(dev, "tx-enable",
						       GPIOD_OUT_LOW);
	if (IS_ERR(pogo->tx_enable_gpio))
		return dev_err_probe(dev, PTR_ERR(pogo->tx_enable_gpio),
				     "TX-enable GPIO\n");

	pogo->vcc = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR(pogo->vcc)) {
		if (PTR_ERR(pogo->vcc) == -ENODEV)
			pogo->vcc = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(pogo->vcc), "vcc regulator\n");
	}

	if (pogo->vcc) {
		ret = regulator_enable(pogo->vcc);
		if (ret)
			return dev_err_probe(dev, ret, "enable vcc regulator\n");
	}

	/*
	 * The stock Caihong driver first selects pogo_power_disable and only
	 * enables the keyboard after sampling the active-low wake line.  Keep
	 * the rail low long enough to reset an MCU that survived a warm reboot,
	 * and retain the pre-power wake sample for the probe result.
	 */
	if (pogo->power_gpio) {
		gpiod_set_value_cansleep(pogo->power_gpio, 0);
		msleep(100);
	}
	if (pogo->wake_gpio)
		wake_before_power = gpiod_get_value_cansleep(pogo->wake_gpio);

	ret = oneplus_pogo_register_inputs(pogo);
	if (ret)
		goto disable_vcc;
	pogo->attached = wake_before_power > 0;
	input_report_switch(pogo->keyboard, SW_TABLET_MODE, !pogo->attached);
	input_sync(pogo->keyboard);

	serdev_device_set_client_ops(serdev, &oneplus_pogo_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		goto disable_vcc;
	ret = devm_add_action_or_reset(dev, oneplus_pogo_stop_delayed_work, pogo);
	if (ret)
		goto disable_vcc;
	if (pogo->attached)
		mod_delayed_work(system_dfl_wq, &pogo->presence_work,
				 msecs_to_jiffies(POGO_DISCONNECT_TIMEOUT_MS));

	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		goto disable_vcc;
	pogo->baud = serdev_device_set_baudrate(serdev, pogo->baud);

	if (pogo->power_gpio)
		gpiod_set_value_cansleep(pogo->power_gpio, 1);
	msleep(250);

	dev_info(dev,
		 "receiver ready at %u baud, pre-power keyboard %s, now %s\n",
		 pogo->baud,
		 wake_before_power > 0 ? "attached" : "not detected",
		 pogo->wake_gpio && gpiod_get_value_cansleep(pogo->wake_gpio) ?
		 "attached" : "not detected");
	queue_delayed_work(system_dfl_wq, &pogo->setup_work,
			   msecs_to_jiffies(500));

	return 0;

disable_vcc:
	if (pogo->vcc)
		regulator_disable(pogo->vcc);
	return ret;
}

static void oneplus_pogo_remove(struct serdev_device *serdev)
{
	struct oneplus_pogo *pogo = serdev_device_get_drvdata(serdev);

	cancel_work_sync(&pogo->led_work);
	devm_release_action(&serdev->dev, oneplus_pogo_stop_delayed_work, pogo);
	cancel_delayed_work_sync(&pogo->setup_work);
	if (pogo->power_gpio)
		gpiod_set_value_cansleep(pogo->power_gpio, 0);
	if (pogo->vcc)
		regulator_disable(pogo->vcc);
}

static const struct of_device_id oneplus_pogo_of_match[] = {
	{ .compatible = "oneplus,caihong-pogo" },
	{ }
};
MODULE_DEVICE_TABLE(of, oneplus_pogo_of_match);

static struct serdev_device_driver oneplus_pogo_driver = {
	.driver = {
		.name = "oneplus-pogo",
		.of_match_table = oneplus_pogo_of_match,
		.dev_groups = oneplus_pogo_groups,
	},
	.probe = oneplus_pogo_probe,
	.remove = oneplus_pogo_remove,
};
module_serdev_device_driver(oneplus_pogo_driver);

MODULE_DESCRIPTION("OnePlus Caihong pogo keyboard and touchpad driver");
MODULE_LICENSE("GPL");
