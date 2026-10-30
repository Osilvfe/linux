#ifndef HID_MULTITOUCH_ONEPLUS_H
#define HID_MULTITOUCH_ONEPLUS_H

struct mt_oneplus_key {
	u32 usage;
	u16 plain;
	u16 function;
};

static const struct mt_oneplus_key mt_oneplus_keys[] = {
	{ HID_UP_CONSUMER | 0x0224, KEY_ESC, 0 },
	{ HID_UP_KEYBOARD | 0x0072, KEY_FN, 0 },
	{ HID_UP_CONSUMER | 0x0070, KEY_F1, KEY_BRIGHTNESSDOWN },
	{ HID_UP_CONSUMER | 0x006f, KEY_F2, KEY_BRIGHTNESSUP },
	{ HID_UP_KEYBOARD | 0x0068, KEY_F3, KEY_MICMUTE },
	{ HID_UP_KEYBOARD | 0x006b, KEY_F4, KEY_TOUCHPAD_TOGGLE },
	{ HID_UP_KEYBOARD | 0x006c, KEY_F4, KEY_TOUCHPAD_TOGGLE },
	{ HID_UP_KEYBOARD | 0x0046, KEY_F5, KEY_SYSRQ },
	{ HID_UP_KEYBOARD | 0x0073, KEY_F6, KEY_SCREENLOCK },
	{ HID_UP_CONSUMER | 0x00b6, KEY_F7, KEY_PREVIOUSSONG },
	{ HID_UP_CONSUMER | 0x00cd, KEY_F8, KEY_PLAYPAUSE },
	{ HID_UP_CONSUMER | 0x00b5, KEY_F9, KEY_NEXTSONG },
	{ HID_UP_CONSUMER | 0x00e2, KEY_F10, KEY_MUTE },
	{ HID_UP_CONSUMER | 0x00ea, KEY_F11, KEY_VOLUMEDOWN },
	{ HID_UP_CONSUMER | 0x00e9, KEY_F12, KEY_VOLUMEUP },
};

struct mt_oneplus {
	bool function_down;
	bool media_keys;
	bool f4_compensation;
	u16 pressed[ARRAY_SIZE(mt_oneplus_keys)];
};

static u16 mt_oneplus_keycode(struct mt_oneplus *state,
			     const struct mt_oneplus_key *key)
{
	if (key->plain == KEY_F4 && !READ_ONCE(state->f4_compensation))
		return key->usage == (HID_UP_KEYBOARD | 0x006b) ? KEY_F16 : KEY_F17;
	if (key->function && (state->function_down != READ_ONCE(state->media_keys)))
		return key->function;
	return key->plain;
}

static bool mt_oneplus_match(const struct hid_device *hdev)
{
	return hdev->bus == BUS_BLUETOOTH && hdev->vendor == 0x22d9 &&
		hdev->product == 0x3869;
}

static int mt_oneplus_input_mapping(struct mt_oneplus *state,
	struct hid_input *hi, struct hid_usage *usage, unsigned long **bit, int *max)
{
	unsigned int index;

	if (!state)
		return 0;
	for (index = 0; index < ARRAY_SIZE(mt_oneplus_keys); index++) {
		const struct mt_oneplus_key *key = &mt_oneplus_keys[index];

		if (usage->hid != key->usage)
			continue;
		input_set_capability(hi->input, EV_KEY, key->plain);
		if (key->function)
			input_set_capability(hi->input, EV_KEY, key->function);
		hid_map_usage_clear(hi, usage, bit, max, EV_KEY,
				    key->plain == KEY_F4 ? mt_oneplus_keycode(state, key) : key->plain);
		return 1;
	}
	return 0;
}

static void mt_oneplus_update_function(struct mt_oneplus *state,
				      struct hid_report *report)
{
	bool keyboard = false;
	bool function_down = false;
	unsigned int field_index;

	for (field_index = 0; field_index < report->maxfield; field_index++) {
		struct hid_field *field = report->field[field_index];
		unsigned int value_index;

		if (field->application != HID_GD_KEYBOARD || field->ignored ||
		    field->flags & HID_MAIN_ITEM_VARIABLE)
			continue;
		keyboard = true;
		for (value_index = 0; value_index < field->report_count; value_index++) {
			s32 value = field->new_value[value_index];
			s64 usage_index = (s64)value - field->logical_minimum;

			if (value < field->logical_minimum || value > field->logical_maximum ||
			    usage_index >= field->maxusage)
				continue;
			if (field->usage[usage_index].hid == (HID_UP_KEYBOARD | 0x0072))
				function_down = true;
		}
	}
	if (keyboard)
		state->function_down = function_down;
}

static void mt_oneplus_event(struct mt_oneplus *state, struct hid_field *field,
			     struct hid_usage *usage, s32 value)
{
	unsigned int index;

	if (!state || usage->type != EV_KEY || field->report->type != HID_INPUT_REPORT)
		return;
	mt_oneplus_update_function(state, field->report);
	for (index = 0; index < ARRAY_SIZE(mt_oneplus_keys); index++) {
		const struct mt_oneplus_key *key = &mt_oneplus_keys[index];

		if (usage->hid != key->usage)
			continue;
		if (value && !state->pressed[index])
			state->pressed[index] = mt_oneplus_keycode(state, key);
		usage->code = state->pressed[index] ?: mt_oneplus_keycode(state, key);
		if (!value)
			state->pressed[index] = 0;
		return;
	}
}

#endif
