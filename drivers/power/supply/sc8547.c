// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/*
 * Southchip SC8547/SC8547A charge-pump driver for OnePlus Pad Pro (caihong).
 *
 * The driver exposes standard power-supply telemetry, coordinates wired PPS
 * with qcom_battmgr, and runs the board's dual-pump SVOOC protocol and stock
 * charging curves.  All register profiles and safety limits are fixed board
 * data; there are no raw-register or manual pump controls in userspace.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <linux/power/qcom_battmgr_caihong.h>
#include <linux/wcd939x-usbss.h>

#define SC8547_REG_BAT_OVP		0x00
#define SC8547_BAT_OVP_DIS		BIT(7)
#define SC8547_BAT_OVP_MASK		GENMASK(5, 0)

#define SC8547_REG_BAT_OCP		0x01

#define SC8547_REG_AC_OVP		0x02
#define SC8547_AC_OVP_STAT		BIT(5)
#define SC8547_AC_OVP_INT_MASK		BIT(3)
#define SC8547_AC_OVP_MASK		GENMASK(2, 0)
#define SC8547_AC_OVP_CONFIG_MASK	(SC8547_AC_OVP_INT_MASK | \
					 SC8547_AC_OVP_MASK)

#define SC8547_REG_VBUS_OVP		0x04

#define SC8547_REG_IBUS_PROT		0x05
#define SC8547_IBUS_UCP_DIS		BIT(7)

#define SC8547_REG_STATUS_06		0x06
#define SC8547_TSHUT_STAT		BIT(6)
#define SC8547_VBUS_ERRORLO_STAT	BIT(5)
#define SC8547_VBUS_ERRORHI_STAT	BIT(4)
#define SC8547_SS_TIMEOUT_FLAG		BIT(3)
#define SC8547_CP_SWITCHING_STAT	BIT(2)
#define SC8547_BLOCKING_06		(SC8547_TSHUT_STAT | \
					 SC8547_VBUS_ERRORLO_STAT | \
					 SC8547_VBUS_ERRORHI_STAT | \
					 SC8547_SS_TIMEOUT_FLAG)
#define SC8547_FATAL_06			(SC8547_TSHUT_STAT | \
					 SC8547_VBUS_ERRORHI_STAT)

#define SC8547_REG_CHG_CTRL		0x07
#define SC8547_CHG_EN			BIT(7)
#define SC8547_REG_RESET		BIT(6)

#define SC8547_REG_SS_CTRL		0x08
#define SC8547_SS_TIMEOUT_MASK		GENMASK(7, 5)

#define SC8547_REG_MODE_CTRL		0x09
#define SC8547_CHARGE_MODE		BIT(7)
#define SC8547_IBUS_UCP_RISE_MASK	BIT(4)
#define SC8547_WATCHDOG_MASK		GENMASK(2, 0)

#define SC8547_REG_IBUS_CTRL		0x0c

#define SC8547_REG_PMID2OUT		0x0d
#define SC8547_PMID2OUT_UVP_MASK	GENMASK(7, 6)
#define SC8547_PMID2OUT_OVP_MASK	GENMASK(5, 4)

#define SC8547_REG_STATUS_0E		0x0e
#define SC8547_VOUT_OVP_STAT		BIT(7)
#define SC8547_VBAT_OVP_STAT		BIT(6)
#define SC8547_IBAT_OCP_STAT		BIT(5)
#define SC8547_VBUS_OVP_STAT		BIT(4)
#define SC8547_IBUS_OCP_STAT		BIT(3)
#define SC8547_IBUS_UCP_FALL_STAT	BIT(2)
#define SC8547_ADAPTER_INSERT_STAT	BIT(1)
#define SC8547_VBAT_INSERT_STAT		BIT(0)
#define SC8547_BLOCKING_0E		(SC8547_VOUT_OVP_STAT | \
					 SC8547_VBAT_OVP_STAT | \
					 SC8547_IBAT_OCP_STAT | \
					 SC8547_VBUS_OVP_STAT | \
					 SC8547_IBUS_OCP_STAT | \
					 SC8547_IBUS_UCP_FALL_STAT)
#define SC8547_FATAL_0E			(SC8547_VOUT_OVP_STAT | \
					 SC8547_VBAT_OVP_STAT | \
					 SC8547_IBAT_OCP_STAT | \
					 SC8547_VBUS_OVP_STAT | \
					 SC8547_IBUS_OCP_STAT)

#define SC8547_REG_INT_MASK		0x10

#define SC8547_REG_ADC_CTRL		0x11
#define SC8547_ADC_EN			BIT(7)
#define SC8547_ADC_RATE_ONESHOT		BIT(6)
#define SC8547_ADC_FREEZE		BIT(5)
#define SC8547_ADC_MODE_MASK		(SC8547_ADC_EN | \
					 SC8547_ADC_RATE_ONESHOT | \
					 SC8547_ADC_FREEZE)

#define SC8547_REG_ADC_FN_DISABLE	0x12
#define SC8547_VBUS_ADC_DIS		BIT(6)
#define SC8547_VOUT_ADC_DIS		BIT(4)
#define SC8547_VBAT_ADC_DIS		BIT(3)
#define SC8547_REQUIRED_ADC_DIS_MASK	(SC8547_VBUS_ADC_DIS | \
					 SC8547_VOUT_ADC_DIS | \
					 SC8547_VBAT_ADC_DIS)

#define SC8547_REG_LOOSE_DET		0x33
#define SC8547_REG_VOOCPHY_TIMING	0x3a
#define SC8547_REG_VOOCPHY_CTRL		0x2b
#define SC8547_REG_VOOC_TX_DATA_H	0x2c
#define SC8547_REG_VOOC_RX_DATA		0x2e
#define SC8547_REG_VOOC_INT_STATUS	0x2f
#define SC8547_REG_VOOC_INT_MASK		0x30
#define SC8547_REG_VOOC_PRE_DATA_H	0x31
#define SC8547_REG_VOOC_DPDM_CTRL	0x21
#define SC8547_REG_VOOC_DPDM_CFG		0x22
#define SC8547_REG_SLAVE_CTRL		0x30
#define SC8547A_REG_OTG_CTRL		0x3b
#define SC8547_REG_VBUS_RANGE_CTRL	0x3c

#define SC8547_VOOC_CTRL_ENABLE		0x80
#define SC8547_VOOC_CTRL_SEND_SEQ	0x01
#define SC8547_VOOC_CTRL_HANDSHAKE	(SC8547_VOOC_CTRL_ENABLE | \
					 SC8547_VOOC_CTRL_SEND_SEQ)
#define SC8547_VOOC_DPDM_ACTIVE		0x21
#define SC8547_VOOC_DPDM_HIZ		0xc0
#define SC8547_VOOC_INT_ACTIVE		0x05
#define SC8547_VOOC_INT_MASKED		0x7f
#define SC8547_VOOC_LOOSE_DET		0xd1
#define SC8547_VOOC_TIMING_ACTIVE	0x60
#define SC8547_VOOC_HANDSHAKE_TIMEOUT_MS	9000
#define SC8547_VOOC_AUTO_RETRY_MAX	3
#define SC8547_VOOC_AUTO_RETRY_DELAY_MS	2000
#define SC8547_VOOC_USB_SUSPEND_MS	1500
#define SC8547_VOOC_USB_SETTLE_MS	700
#define SC8547_VOOC_REACTIVE_SETTLE_MS	350
#define SC8547_VOOC_CABLE_DEBOUNCE_MS	300
#define SC8547_VOOC_CABLE_RECHECK_MS	2500
#define SC8547_VOOC_CABLE_REMOVE_SAMPLES	2
#define SC8547_VOOC_CABLE_PRESENT_UV	3000000
#define SC8547_VOOC_BOOT_SETTLE_MS	30000
#define SC8547_VOOC_SESSION_MS		120000
#define SC8547_VOOC_CP_START_GRACE_MS	500
#define SC8547_VOOC_CP_START_SETTLE_MS	500
#define SC8547_VOOC_SESSION_END_GRACE_MS	750
#define SC8547_VOOC_CP_GATE_RETRY_MS	250
#define SC8547_VOOC_CP_START_MAX_VBUS_UV	9500000
#define SC8547_VOOC_VBUS_LOW_DELTA_UV	100000
#define SC8547_VOOC_VBUS_HIGH_DELTA_UV	200000
/* The SC8547 ADC is quantized at 3.75 mV and the gauge sample can lag the
 * adapter step by one protocol frame.  Allow a near-window CP start so a
 * source that jumps past the 100 mV lower edge is loaded before it can
 * overshoot the 10.5-V software ceiling. */
#define SC8547_VOOC_CP_START_LOW_DELTA_UV	50000
#define SC8547_VOOC_CP_START_HIGH_DELTA_UV	205000
#define SC8547_VOOC_CP_MIN_VBUS_UV	6500000
#define SC8547_VOOC_CP_MAX_IBUS_UA	3500000
#define SC8547_VOOC_TOTAL_MAX_IBUS_UA	6500000
/* Values from the stock oplus,sc8547a@6f DT node are expressed in mA. */
#define SC8547_VOOC_SECONDARY_ENABLE_MIN_IBUS_UA	1400000
#define SC8547_VOOC_SECONDARY_DISABLE_MIN_IBUS_UA	1100000
#define SC8547_VOOC_SECONDARY_SAMPLE_MS	25
#define SC8547_VOOC_SECONDARY_START_SETTLE_MS	500
#define SC8547_VOOC_SECONDARY_ENABLE_RETRY_MS	100
#define SC8547_VOOC_SECONDARY_ENABLE_TIMEOUT_MS	5000
#define SC8547_VOOC_SECONDARY_DISABLE_SAMPLES	20
#define SC8547_VOOC_POLL_MIN_US	100
#define SC8547_VOOC_POLL_MAX_US	200
#define SC8547_VOOC_ADC_SAMPLE_US	5000
#define SC8547_VOOC_BATTERY_SAMPLE_US	1000000
#define SC8547_VOOC_BATTERY_MAX_TEMP_DECIC	510
#define SC8547_VOOC_BATTERY_MIN_TEMP_DECIC	0
#define SC8547_VOOC_BATTERY_MAX_VOLTAGE_UV	4480000 /* stock one-/n-time full threshold */
#define SC8547_VOOC_BATTERY_FULL_CAPACITY	100
#define SC8547_VOOC_MAX_VBUS_UV	10500000 /* official VOOC ceiling */
#define SC8547_VOOC_CLEANUP_VBUS_WAIT_MS	2000
#define SC8547_VOOC_CLEANUP_VBUS_POLL_MS	50
#define SC8547_VOOC_MAX_VBUS_ADJUST	80
#define SC8547_VOOC_SESSION_ROUNDS	128
#define SC8547_VOOC_RXDATA_DONE		BIT(1)
#define SC8547_VOOC_RX_START		BIT(2)
#define SC8547_VOOC_TXDATA_WR_FAIL	BIT(3)
#define SC8547_VOOC_TXDATA_DONE	BIT(0)
#define SC8547_VOOC_ERR_TRANS		BIT(4)
#define SC8547_VOOC_NINTH_CLK_ERR	BIT(6)

/* Protocol data before the SC8547's 10-bit wire bit reversal. */
/*
 * The board has two parallel charge pumps.  The vendor SVOOC profile writes
 * NO_VOOC2_CIRCUIT_PARALLEL_R_L_DEF (0x16) to TX0 bits 3..7 and
 * NO_VOOC2_CIRCUIT_PARALLEL_R_H_DEF (0x01) to TX1 bits 0..1.  The local
 * seven-bit representation stores those fields as data[4:0] and data[6:5],
 * respectively, so the complete value is 0x16 | (0x01 << 5).
 */
#define SC8547_VOOC_SAFE_R_BITS		0x36
/* Official project-23926 SVOOC input ceiling: 6.1 A. */
#define SC8547_VOOC_DEFAULT_CURRENT_CODE	61
#define SC8547_VOOC_MIN_CURRENT_CODE	10 /* official 1.0-A terminal curve point */
#define SC8547_VOOC_MAX_CURRENT_CODE	61 /* official project-23926 default */
#define SC8547_VOOC_CURVE_CONFIRM_SAMPLES	4
#define SC8547_VOOC_CURVE_IBUS_MARGIN_UA	1000000
#define SC8547_VOOC_TEMP_LITTLE_COLD_DECIC	50
#define SC8547_VOOC_TEMP_COOL_DECIC	120
#define SC8547_VOOC_TEMP_LITTLE_COOL_DECIC	160
#define SC8547_VOOC_TEMP_NORMAL_LOW_DECIC	350
#define SC8547_VOOC_TEMP_NORMAL_HIGH_DECIC	440
#define SC8547_VOOC_TEMP_HIGH_ENTER_DECIC	420
#define SC8547_VOOC_TEMP_HIGH_EXIT_DECIC	385
#define SC8547_VOOC_TEMP_WARM_EXIT_DECIC	420
#define SC8547_VOOC_TEMP_HIGH_CURRENT_CODE	40
#define SC8547_VOOC_TEMP_WARM_CURRENT_CODE	20
#define SC8547_VOOC_PRE_SVOOC		0x2002

#define SC8547_VOOC_HEAD_VOOC20		0x05
#define SC8547_VOOC_HEAD_VOOC30		0x03
#define SC8547_VOOC_HEAD_SVOOC		0x04
#define SC8547_VOOC_INVERT_VOOC20		0x05
#define SC8547_VOOC_INVERT_VOOC30		0x06
#define SC8547_VOOC_INVERT_SVOOC		0x01
#define SC8547_VOOC_CMD_GET_BATT_VOL	0x01
#define SC8547_VOOC_CMD_IS_VBUS_OK	0x02
#define SC8547_VOOC_CMD_ASK_CURRENT	0x03
#define SC8547_VOOC_CMD_ASK_FAST		0x04
#define SC8547_VOOC_CMD_TELL_MODEL	0x05
#define SC8547_VOOC_CMD_IDENTIFICATION	0x0c
#define SC8547_VOOC_CMD_ASK_BAT_MODEL	0x0d
#define SC8547_VOOC_CMD_ASK_POWER_BANK	0x06
#define SC8547_VOOC_CMD_TELL_USB_BAD	0x09
#define SC8547_VOOC_CMD_ASK_AP_STATUS	0x0a
#define SC8547_VOOC_CMD_ADAPTER_CHECK	0xf1
#define SC8547_VOOC_CMD_TELL_MODEL_PROC	0xf2
#define SC8547_VOOC_CMD_BAT_MODEL_PROC	0xf3
#define SC8547_VOOC_BAT_SYS_COUNT	6

/*
 * OnePlus' wired PPS startup selects the 2:1 work mode, raises PPS VBUS, then
 * invokes CP_ENABLE followed by CP_SET_WORK_START. CP_ENABLE is intentionally
 * a no-op in oplus_chg_pps.c; the live path therefore asserts REG07 directly
 * and never runs the generic SC8547A reset/FORCE_VAC_OK helper. Preserve the
 * board profile's bounded OVP/OCP values while reproducing that live path.
 */
#define SC8547A_VENDOR_PPS_MODE_MASK	(SC8547_CHARGE_MODE | \
					 SC8547_IBUS_UCP_RISE_MASK)
#define SC8547A_VENDOR_PPS_MODE_2TO1	SC8547_IBUS_UCP_RISE_MASK
#define SC8547A_VENDOR_PPS_WATCHDOG	4
#define SC8547A_VENDOR_PPS_PMID2OUT	0x70
#define SC8547A_VENDOR_BAT_OVP		0x36 /* official 4.65-V protection profile */
#define SC8547A_VENDOR_BAT_OCP		0xbf
#define SC8547A_VENDOR_PPS_LOOSE_DET	0xd1
#define SC8547A_VENDOR_PPS_TIMING	0x60
#define SC8547A_VENDOR_VOOCPHY_DISABLE	0x00
#define SC8547A_VENDOR_INSERT_IRQ_MASK	0x02
#define SC8547A_VENDOR_OTG_CTRL		0x04
#define SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT	0x00
#define SC8547A_VENDOR_CP_ENABLE_VBUS_OVP	0x71
#define SC8547A_VENDOR_CP_ENABLE_IBUS_PROT	0xcf
#define SC8547A_VENDOR_CP_ENABLE_MODE	0x10
#define SC8547A_VENDOR_CP_ENABLE_SS_TIMEOUT	0x00
#define SC8547A_VENDOR_CP_RUNTIME_SS_TIMEOUT	0xe0
#define SC8547_VENDOR_SECONDARY_CTRL	0x7f
#define SC8547_VENDOR_SECONDARY_VBUS_RANGE	0x40
#define SC8547A_VENDOR_SECONDARY_IBUS_PROT	0x1c
#define SC8547_VENDOR_SECONDARY_IBUS_PROT	0x2c

/* oplus_pps_charge_start(): one initial start plus three 500-ms retries. */
#define SC8547_VENDOR_START_RETRY_MAX	3
#define SC8547_VENDOR_START_ATTEMPTS	(SC8547_VENDOR_START_RETRY_MAX + 1)
#define SC8547_VENDOR_START_CHECK_MS	500

#define SC8547_PREP_MISMATCH_REG02_CONFIG	BIT(0)
#define SC8547_PREP_MISMATCH_REG02_STATUS	BIT(1)
#define SC8547_PREP_MISMATCH_REG04		BIT(2)
#define SC8547_PREP_MISMATCH_REG05		BIT(3)
#define SC8547_PREP_MISMATCH_REG08		BIT(4)
#define SC8547_PREP_MISMATCH_REG09		BIT(5)
#define SC8547_PREP_MISMATCH_REG0D		BIT(6)
#define SC8547_PREP_MISMATCH_REG10		BIT(7)
#define SC8547_PREP_MISMATCH_REG11		BIT(8)
#define SC8547_PREP_MISMATCH_REG2B		BIT(9)
#define SC8547_PREP_MISMATCH_REG3A		BIT(10)
#define SC8547_PREP_MISMATCH_REG3B		BIT(11)
#define SC8547_PREP_MISMATCH_REG30		BIT(12)
#define SC8547_PREP_MISMATCH_REG3C		BIT(13)

#define SC8547_REG_IBUS_ADC_H		0x13
#define SC8547_REG_VBUS_ADC_H		0x15
#define SC8547_REG_VAC_ADC_H		0x17
#define SC8547_REG_VOUT_ADC_H		0x19
#define SC8547_REG_VBAT_ADC_H		0x1b
#define SC8547_REG_TDIE_ADC_H		0x1f
#define SC8547_REG_DEVICE_ID		0x36

#define SC8547_ADC_12BIT_H_MASK		GENMASK(3, 0)
#define SC8547_TDIE_H_MASK		BIT(0)

#define SC8547A_DEVICE_ID		0x67
#define SC8547D_DEVICE_ID		0x49

#define SC8547_IBUS_UA_PER_LSB		1875
#define SC8547_VBUS_UV_PER_LSB		3750
#define SC8547_VAC_UV_PER_LSB		5000
#define SC8547_VOUT_UV_PER_LSB		1250
#define SC8547_VBAT_UV_PER_LSB		1250
#define SC8547_TDIE_MC_PER_LSB		500

#define SC8547_POLICY_RUN_MS		900000
#define SC8547_POLICY_CANCEL_CMD		2
#define SC8547_POLICY_CONTINUOUS_CMD	3
#define SC8547_STABILITY_SAMPLE_MS	500
#define SC8547_RUNTIME_RANGE_RECHECKS	2
#define SC8547_RUNTIME_RANGE_RECHECK_MIN_US	10000
#define SC8547_RUNTIME_RANGE_RECHECK_MAX_US	20000
#define SC8547_POST_PREPARE_RANGE_RECHECKS	2
#define SC8547_PPS_FIXED_IBUS_MAX_UA		1200000
#define SC8547_VENDOR_CURRENT_IBUS_MAX_UA	1650000
#define SC8547_VENDOR_CURRENT_IBUS_ADC_MAX_UA	\
	(((SC8547_VENDOR_CURRENT_IBUS_MAX_UA + SC8547_IBUS_UA_PER_LSB - 1) / \
	  SC8547_IBUS_UA_PER_LSB) * SC8547_IBUS_UA_PER_LSB)
#define SC8547_RESET_SETTLE_MS		10
#define SC8547_ADC_READY_POLL_MS		10
#define SC8547_ADC_READY_TIMEOUT_MS	500
#define SC8547_ADC_READY_STABLE_SAMPLES	1

enum sc8547_variant {
	SC8547_VARIANT_UNKNOWN,
	SC8547_VARIANT_SC8547,
	SC8547_VARIANT_SC8547A,
	SC8547_VARIANT_SC8547D,
};

struct sc8547_chip_info {
	enum sc8547_variant variant;
	const char *name;
};

static const struct sc8547_chip_info sc8547_info = {
	.variant = SC8547_VARIANT_SC8547,
	.name = "sc8547",
};

static const struct sc8547_chip_info sc8547a_info = {
	.variant = SC8547_VARIANT_SC8547A,
	.name = "sc8547a",
};

struct sc8547_raw_profile {
	u8 reg00;
	u8 reg01;
	u8 reg02;
	u8 reg04;
	u8 reg05;
	u8 reg0d;
	bool has_reg01;
	bool has_reg0d;
	bool complete;
};

struct sc8547_enable_window {
	u32 vbus_min_uv;
	u32 vbus_max_uv;
	u32 active_vbus_min_uv;
	u32 ratio_error_max_uv;
	u32 start_delta_uv;
	u32 start_overshoot_max_uv;
	u32 start_reference_max_uv;
	u32 vbat_min_uv;
	u32 vbat_max_uv;
	bool complete;
};

struct sc8547_active_sample {
	int vac_uv;
	int vbus_uv;
	int vout_uv;
	int vbat_uv;
	int output_reference_uv;
	int ratio_error_uv;
	int ibus_ua;
	int tdie_mc;
};

struct sc8547_pps_run_state {
	int target_vbus_uv;
	bool vendor_current_control;
};

struct sc8547_vooc_curve;

enum sc8547_vooc_temp_state {
	SC8547_VOOC_TEMP_STATE_NORMAL,
	SC8547_VOOC_TEMP_STATE_HIGH,
	SC8547_VOOC_TEMP_STATE_WARM,
};

struct sc8547_device {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	const struct sc8547_chip_info *match_info;
	const char *role;
	enum sc8547_variant variant;
	struct mutex lock;
	struct sc8547_raw_profile profile;
	struct sc8547_enable_window window;
	atomic_t policy_cancel;
	atomic_t pps_running;
	bool pps_capable;
	bool vooc_capable;
	bool vooc_auto_retry;
	bool vooc_resume_retry;
	bool vooc_cable_online;
	bool vooc_cable_state_known;
	u8 vooc_cable_absent_count;
	u8 vooc_active_current_code;
	const struct sc8547_vooc_curve *vooc_curve;
	u8 vooc_curve_index;
	u8 vooc_curve_confirm_count;
	u32 vooc_curve_elapsed_s;
	bool vooc_curve_initialized;
	enum sc8547_vooc_temp_state vooc_temp_state;
	int vooc_policy_capacity;
	int vooc_policy_vbat_uv;
	int vooc_policy_temp_decic;
	bool vooc_policy_full;
	bool vooc_policy_clamped;
	bool init_done;
	bool pps_prepared;
	bool vooc_initialized;
	u32 pps_target_vbus_uv;
	u32 tdie_max_mc;
	u8 device_id;
	int vooc_last_vbus_uv;
	int vooc_last_ibus_ua;
	bool vooc_last_cp_started;
	/* Per-session protocol context.  These fields are reset for every session. */
	u8 vooc_tx_head;
	u8 vooc_adapter_model_detect_remaining;
	u8 vooc_commu_remaining;
	u8 vooc_model_remaining;
	u8 vooc_batt_remaining;
	u8 vooc_ident_next_l;
	u8 vooc_ident_next_h;
	u16 vooc_ident_local;
	u16 vooc_ident_far;
	u8 vooc_adapter_model;
	bool vooc_adapter_check_ok;
	bool vooc_model_done;
	bool vooc_batt_model_done;
	unsigned int vooc_auto_retry_count;
	int vooc_last_total_ibus_ua;
	atomic_t vooc_retry_running;
	struct workqueue_struct *vooc_wq;
	struct delayed_work vooc_retry_work;
	struct delayed_work vooc_cable_work;
	struct notifier_block vooc_psy_nb;
};

struct sc8547_vooc_battery_state {
	int capacity;
	int voltage_uv;
	int temp_decic;
	int status;
};

struct sc8547_vooc_curve_point {
	u32 target_vbat_uv;
	u8 target_ibus_code;
	bool exit;
	u16 target_time_s;
};

struct sc8547_vooc_curve {
	const struct sc8547_vooc_curve_point *points;
	u8 count;
	const char *name;
};

#define SC8547_CURVE_POINT(_mv, _code, _exit, _time) \
	{ .target_vbat_uv = (_mv) * 1000U, .target_ibus_code = (_code), \
	  .exit = (_exit), .target_time_s = (_time) }
#define SC8547_CURVE(_array) \
	{ .points = (_array), .count = ARRAY_SIZE(_array), .name = #_array }

/* Project 23926's exact SVOOC curves from oplus-chg-23926.dtsi. */
static const struct sc8547_vooc_curve_point svooc_0_50_little_cold[] = {
	SC8547_CURVE_POINT(4200, 25, 0, 0), SC8547_CURVE_POINT(4455, 20, 0, 0),
	SC8547_CURVE_POINT(4455, 15, 0, 0), SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_0_50_cool[] = {
	SC8547_CURVE_POINT(4180, 40, 0, 0), SC8547_CURVE_POINT(4455, 25, 0, 0),
	SC8547_CURVE_POINT(4455, 20, 0, 0), SC8547_CURVE_POINT(4455, 15, 0, 0),
	SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_0_50_little_cool[] = {
	SC8547_CURVE_POINT(4200, 50, 0, 1440), SC8547_CURVE_POINT(4400, 30, 0, 0),
	SC8547_CURVE_POINT(4400, 25, 0, 0), SC8547_CURVE_POINT(4470, 20, 0, 0),
	SC8547_CURVE_POINT(4470, 15, 0, 0), SC8547_CURVE_POINT(4480, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_0_50_normal_low[] = {
	SC8547_CURVE_POINT(4250, 61, 0, 840), SC8547_CURVE_POINT(4250, 50, 0, 1440),
	SC8547_CURVE_POINT(4400, 40, 0, 0), SC8547_CURVE_POINT(4470, 30, 0, 0),
	SC8547_CURVE_POINT(4470, 25, 0, 0), SC8547_CURVE_POINT(4470, 20, 0, 0),
	SC8547_CURVE_POINT(4470, 15, 0, 0), SC8547_CURVE_POINT(4480, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_0_50_normal_high[] = {
	SC8547_CURVE_POINT(4250, 50, 0, 1440), SC8547_CURVE_POINT(4400, 40, 0, 0),
	SC8547_CURVE_POINT(4470, 30, 0, 0), SC8547_CURVE_POINT(4470, 25, 0, 0),
	SC8547_CURVE_POINT(4470, 20, 0, 0), SC8547_CURVE_POINT(4470, 15, 0, 0),
	SC8547_CURVE_POINT(4480, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_warm[] = {
	SC8547_CURVE_POINT(4130, 20, 1, 0),
};

/* 50-75% is identical to 0-50% except for the little-cold first point. */
static const struct sc8547_vooc_curve_point svooc_50_75_little_cold[] = {
	SC8547_CURVE_POINT(4455, 20, 0, 0), SC8547_CURVE_POINT(4455, 15, 0, 0),
	SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_75_85_little_cold[] = {
	SC8547_CURVE_POINT(4455, 20, 0, 0), SC8547_CURVE_POINT(4455, 15, 0, 0),
	SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_75_85_cool[] = {
	SC8547_CURVE_POINT(4455, 25, 0, 0), SC8547_CURVE_POINT(4455, 20, 0, 0),
	SC8547_CURVE_POINT(4455, 15, 0, 0), SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_75_85_little_cool[] = {
	SC8547_CURVE_POINT(4400, 30, 0, 0), SC8547_CURVE_POINT(4400, 25, 0, 0),
	SC8547_CURVE_POINT(4470, 20, 0, 0), SC8547_CURVE_POINT(4470, 15, 0, 0),
	SC8547_CURVE_POINT(4480, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_75_85_normal[] = {
	SC8547_CURVE_POINT(4400, 40, 0, 0), SC8547_CURVE_POINT(4470, 30, 0, 0),
	SC8547_CURVE_POINT(4470, 25, 0, 0), SC8547_CURVE_POINT(4470, 20, 0, 0),
	SC8547_CURVE_POINT(4470, 15, 0, 0), SC8547_CURVE_POINT(4480, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_85_90_little_cold[] = {
	SC8547_CURVE_POINT(4455, 20, 0, 0), SC8547_CURVE_POINT(4455, 15, 0, 0),
	SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_85_90_cool[] = {
	SC8547_CURVE_POINT(4455, 25, 0, 0), SC8547_CURVE_POINT(4455, 20, 0, 0),
	SC8547_CURVE_POINT(4455, 15, 0, 0), SC8547_CURVE_POINT(4465, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_85_90_little_cool[] = {
	SC8547_CURVE_POINT(4470, 20, 0, 0), SC8547_CURVE_POINT(4470, 15, 0, 0),
	SC8547_CURVE_POINT(4480, 10, 1, 0),
};
static const struct sc8547_vooc_curve_point svooc_85_90_normal[] = {
	SC8547_CURVE_POINT(4470, 30, 0, 0), SC8547_CURVE_POINT(4470, 25, 0, 0),
	SC8547_CURVE_POINT(4470, 20, 0, 0), SC8547_CURVE_POINT(4470, 15, 0, 0),
	SC8547_CURVE_POINT(4480, 10, 1, 0),
};

static const struct sc8547_vooc_curve curve_0_50_little_cold =
	SC8547_CURVE(svooc_0_50_little_cold);
static const struct sc8547_vooc_curve curve_0_50_cool =
	SC8547_CURVE(svooc_0_50_cool);
static const struct sc8547_vooc_curve curve_0_50_little_cool =
	SC8547_CURVE(svooc_0_50_little_cool);
static const struct sc8547_vooc_curve curve_0_50_normal_low =
	SC8547_CURVE(svooc_0_50_normal_low);
static const struct sc8547_vooc_curve curve_0_50_normal_high =
	SC8547_CURVE(svooc_0_50_normal_high);
static const struct sc8547_vooc_curve curve_warm = SC8547_CURVE(svooc_warm);
static const struct sc8547_vooc_curve curve_50_75_little_cold =
	SC8547_CURVE(svooc_50_75_little_cold);
static const struct sc8547_vooc_curve curve_75_85_little_cold =
	SC8547_CURVE(svooc_75_85_little_cold);
static const struct sc8547_vooc_curve curve_75_85_cool =
	SC8547_CURVE(svooc_75_85_cool);
static const struct sc8547_vooc_curve curve_75_85_little_cool =
	SC8547_CURVE(svooc_75_85_little_cool);
static const struct sc8547_vooc_curve curve_75_85_normal =
	SC8547_CURVE(svooc_75_85_normal);
static const struct sc8547_vooc_curve curve_85_90_little_cold =
	SC8547_CURVE(svooc_85_90_little_cold);
static const struct sc8547_vooc_curve curve_85_90_cool =
	SC8547_CURVE(svooc_85_90_cool);
static const struct sc8547_vooc_curve curve_85_90_little_cool =
	SC8547_CURVE(svooc_85_90_little_cool);
static const struct sc8547_vooc_curve curve_85_90_normal =
	SC8547_CURVE(svooc_85_90_normal);

static const struct regmap_config sc8547_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static const char *sc8547_variant_name(enum sc8547_variant variant)
{
	switch (variant) {
	case SC8547_VARIANT_SC8547:
		return "sc8547";
	case SC8547_VARIANT_SC8547A:
		return "sc8547a";
	case SC8547_VARIANT_SC8547D:
		return "sc8547d";
	default:
		return "unknown";
	}
}

static bool sc8547_variant_control_supported(enum sc8547_variant variant)
{
	return variant == SC8547_VARIANT_SC8547 ||
	       variant == SC8547_VARIANT_SC8547A;
}

static bool sc8547_bounded_role_supported(const struct sc8547_device *sc)
{
	if (!strcmp(sc->role, "primary"))
		return sc->variant == SC8547_VARIANT_SC8547A;

	if (!strcmp(sc->role, "secondary"))
		return sc8547_variant_control_supported(sc->variant);

	return false;
}

static enum sc8547_variant
sc8547_detect_variant(struct sc8547_device *sc, u8 device_id)
{
	if (device_id == SC8547A_DEVICE_ID)
		return SC8547_VARIANT_SC8547A;
	if (device_id == SC8547D_DEVICE_ID)
		return SC8547_VARIANT_SC8547D;
	if (sc->match_info)
		return sc->match_info->variant;

	return SC8547_VARIANT_UNKNOWN;
}

static int sc8547_restore_idle_profile(struct sc8547_device *sc);
static int sc8547_prepare_vendor_pps_startup(
	struct sc8547_device *sc, bool *touched);
static int sc8547_run_pps_bounded(struct sc8547_device *sc,
				    unsigned int run_ms,
				    bool vendor_current_control);
static int sc8547_run_pps_continuous(struct sc8547_device *sc);

/*
 * Match the downstream whole-register charge commands.  The original
 * SC8547 uses bit 2 as a persistent role/configuration bit, while the
 * SC8547A vendor driver uses the plain enable bit (0x80/0x00). Secondary
 * devices carry the downstream bit-0 role setting. Bounded control is
 * separately gated per physical pump role.
 */
static u8 sc8547_charge_command(const struct sc8547_device *sc, bool enable)
{
	u8 value;

	if (sc->variant == SC8547_VARIANT_SC8547A)
		value = enable ? SC8547_CHG_EN : 0;
	else
		value = BIT(2) | (enable ? SC8547_CHG_EN : 0);

	if (!strcmp(sc->role, "secondary"))
		value |= BIT(0);

	return value;
}

static int sc8547_set_charge_enabled(struct sc8547_device *sc, bool enable)
{
	return regmap_write(sc->regmap, SC8547_REG_CHG_CTRL,
			    sc8547_charge_command(sc, enable));
}

static int sc8547_set_adc_enabled(struct sc8547_device *sc, bool enable)
{
	return regmap_update_bits(sc->regmap, SC8547_REG_ADC_CTRL,
				 SC8547_ADC_EN, enable ? SC8547_ADC_EN : 0);
}

static int sc8547_enable_required_adc_channels(struct sc8547_device *sc)
{
	return regmap_update_bits(sc->regmap, SC8547_REG_ADC_FN_DISABLE,
				 SC8547_REQUIRED_ADC_DIS_MASK, 0);
}

static int sc8547_set_watchdog_code(struct sc8547_device *sc, unsigned int code)
{
	if (code > 5)
		return -EINVAL;

	return regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
				 SC8547_WATCHDOG_MASK, code);
}

static int sc8547_read_adc(struct sc8547_device *sc, unsigned int reg,
			   unsigned int high_mask, int scale)
{
	u8 raw[2];
	int ret;

	/*
	 * The ADC high and low bytes are one sample.  Reading them through two
	 * independent I2C transactions can splice adjacent conversions when the
	 * continuous ADC updates between reads.  Keep each pair in one transfer;
	 * runtime range confirmation below still rejects persistent bad data.
	 */
	ret = regmap_bulk_read(sc->regmap, reg, raw, sizeof(raw));
	if (ret)
		return ret;

	return (((raw[0] & high_mask) << 8) | raw[1]) * scale;
}

static int sc8547_get_vbus_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VBUS_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VBUS_UV_PER_LSB);
}

static int sc8547_get_ibus_ua(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_IBUS_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_IBUS_UA_PER_LSB);
}

static int sc8547_get_vbat_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VBAT_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VBAT_UV_PER_LSB);
}

/* VOOC normally compares VBUS with the system-gauge battery voltage.  During
 * the hand-off from the USB buck to the charge pump the gauge can briefly
 * sag while the SC8547A local VBAT protection ADC still sees the cell's real
 * voltage.  Use the higher valid reading for the start decision so the pump
 * cannot be enabled below the hardware's own 2:1 VBUS threshold. */
static int sc8547_get_vooc_bat_uv(struct sc8547_device *sc)
{
	struct power_supply *psy;
	union power_supply_propval val = { };
	int ret;
	int gauge_uv = -ENODATA;
	int local_uv;

	psy = power_supply_get_by_name("qcom-battmgr-bat");
	if (psy) {
		ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		power_supply_put(psy);
		if (!ret && val.intval > 0)
			gauge_uv = val.intval;
	}

	local_uv = sc8547_get_vbat_uv(sc);
	if (local_uv < 0)
		return gauge_uv;
	if (gauge_uv < 0)
		return local_uv;

	return max(gauge_uv, local_uv);
}

/*
 * Read the system-gauge values used by the vendor current strategy.  The
 * charge-pump ADC is useful for the 2:1 gate, but it cannot tell us whether
 * the battery has entered the terminal taper region.  Fast charge is refused
 * when the two safety-critical gauge values (cell voltage and temperature)
 * are unavailable.
 */
static int sc8547_get_vooc_battery_state(struct sc8547_vooc_battery_state *state)
{
	struct power_supply *psy;
	union power_supply_propval val = { };
	int ret;

	if (!state)
		return -EINVAL;

	state->capacity = -ENODATA;
	state->voltage_uv = -ENODATA;
	state->temp_decic = -ENODATA;
	state->status = -ENODATA;
	psy = power_supply_get_by_name("qcom-battmgr-bat");
	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					       &val);
	if (ret)
		goto out_put;
	state->voltage_uv = val.intval;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		goto out_put;
	state->temp_decic = val.intval;
	ret = qcom_battmgr_caihong_get_raw_capacity(psy);
	if (ret >= 0)
		state->capacity = ret;
	else if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY,
					    &val))
		state->capacity = val.intval;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &val))
		state->status = val.intval;
	ret = 0;

out_put:
	power_supply_put(psy);
	return ret;
}

static void sc8547_queue_vooc_retry(struct sc8547_device *sc,
				    unsigned long delay)
{
	/* The continuous protocol exchange can remain active for the whole cable
	 * session. Keep it off generic system workqueues so early boot power-supply
	 * notifications,
	 * display setup and networking always retain their workers. */
	mod_delayed_work(sc->vooc_wq, &sc->vooc_retry_work, delay);
}

static void sc8547_destroy_vooc_workqueue(void *data)
{
	destroy_workqueue(data);
}

static const struct sc8547_vooc_curve *
sc8547_vooc_select_curve(const struct sc8547_vooc_battery_state *state)
{
	bool little_cold, cool, little_cool, normal_low, normal_high;

	little_cold = state->temp_decic < SC8547_VOOC_TEMP_LITTLE_COLD_DECIC;
	cool = state->temp_decic >= SC8547_VOOC_TEMP_LITTLE_COLD_DECIC &&
		state->temp_decic < SC8547_VOOC_TEMP_COOL_DECIC;
	little_cool = state->temp_decic >= SC8547_VOOC_TEMP_COOL_DECIC &&
		state->temp_decic < SC8547_VOOC_TEMP_LITTLE_COOL_DECIC;
	normal_low = state->temp_decic >= SC8547_VOOC_TEMP_LITTLE_COOL_DECIC &&
		state->temp_decic < SC8547_VOOC_TEMP_NORMAL_LOW_DECIC;
	normal_high = state->temp_decic >= SC8547_VOOC_TEMP_NORMAL_LOW_DECIC &&
		state->temp_decic < SC8547_VOOC_TEMP_NORMAL_HIGH_DECIC;

	if (!little_cold && !cool && !little_cool && !normal_low && !normal_high)
		return &curve_warm;

	if (state->capacity <= 50) {
		if (little_cold)
			return &curve_0_50_little_cold;
		if (cool)
			return &curve_0_50_cool;
		if (little_cool)
			return &curve_0_50_little_cool;
		if (normal_low)
			return &curve_0_50_normal_low;
		return &curve_0_50_normal_high;
	}

	if (state->capacity <= 75) {
		if (little_cold)
			return &curve_50_75_little_cold;
		if (cool)
			return &curve_0_50_cool;
		if (little_cool)
			return &curve_0_50_little_cool;
		if (normal_low)
			return &curve_0_50_normal_low;
		return &curve_0_50_normal_high;
	}

	if (state->capacity <= 85) {
		if (little_cold)
			return &curve_75_85_little_cold;
		if (cool)
			return &curve_75_85_cool;
		if (little_cool)
			return &curve_75_85_little_cool;
		return &curve_75_85_normal;
	}

	if (little_cold)
		return &curve_85_90_little_cold;
	if (cool)
		return &curve_85_90_cool;
	if (little_cool)
		return &curve_85_90_little_cool;
	return &curve_85_90_normal;
}

static void sc8547_vooc_reset_curve(struct sc8547_device *sc)
{
	sc->vooc_curve = NULL;
	sc->vooc_curve_index = 0;
	sc->vooc_curve_confirm_count = 0;
	sc->vooc_curve_elapsed_s = 0;
	sc->vooc_curve_initialized = false;
	sc->vooc_temp_state = SC8547_VOOC_TEMP_STATE_NORMAL;
}

static u8 sc8547_vooc_temp_current_limit(struct sc8547_device *sc,
					 int temp_decic)
{
	enum sc8547_vooc_temp_state previous = sc->vooc_temp_state;
	u8 limit;

	switch (sc->vooc_temp_state) {
	case SC8547_VOOC_TEMP_STATE_NORMAL:
		if (temp_decic > SC8547_VOOC_TEMP_NORMAL_HIGH_DECIC)
			sc->vooc_temp_state = SC8547_VOOC_TEMP_STATE_WARM;
		else if (temp_decic > SC8547_VOOC_TEMP_HIGH_ENTER_DECIC)
			sc->vooc_temp_state = SC8547_VOOC_TEMP_STATE_HIGH;
		break;
	case SC8547_VOOC_TEMP_STATE_HIGH:
		if (temp_decic > SC8547_VOOC_TEMP_NORMAL_HIGH_DECIC)
			sc->vooc_temp_state = SC8547_VOOC_TEMP_STATE_WARM;
		else if (temp_decic < SC8547_VOOC_TEMP_HIGH_EXIT_DECIC)
			sc->vooc_temp_state = SC8547_VOOC_TEMP_STATE_NORMAL;
		break;
	case SC8547_VOOC_TEMP_STATE_WARM:
		if (temp_decic < SC8547_VOOC_TEMP_WARM_EXIT_DECIC)
			sc->vooc_temp_state = SC8547_VOOC_TEMP_STATE_HIGH;
		break;
	}

	switch (sc->vooc_temp_state) {
	case SC8547_VOOC_TEMP_STATE_WARM:
		limit = SC8547_VOOC_TEMP_WARM_CURRENT_CODE;
		break;
	case SC8547_VOOC_TEMP_STATE_HIGH:
		limit = SC8547_VOOC_TEMP_HIGH_CURRENT_CODE;
		break;
	case SC8547_VOOC_TEMP_STATE_NORMAL:
	default:
		limit = SC8547_VOOC_MAX_CURRENT_CODE;
		break;
	}

	if (previous != sc->vooc_temp_state)
		dev_info(sc->dev,
			 "VOOC temperature state %u->%u temp=%d current_limit=%u\n",
			 previous, sc->vooc_temp_state, temp_decic, limit);

	return limit;
}

static int sc8547_vooc_initialize_curve(
		struct sc8547_device *sc,
		const struct sc8547_vooc_battery_state *state)
{
	const struct sc8547_vooc_curve_point *point;
	const struct sc8547_vooc_curve *curve;
	u8 i;

	if (state->capacity > 90)
		return -EALREADY;
	curve = sc8547_vooc_select_curve(state);
	if (!curve || !curve->count)
		return -EINVAL;

	sc->vooc_curve = curve;
	sc->vooc_curve_index = 0;
	/* The stock implementation enters at the first non-exit point whose
	 * target is not below the plug-in battery voltage. */
	for (i = 0; i < curve->count; i++) {
		point = &curve->points[i];
		if (!point->exit && state->voltage_uv <= point->target_vbat_uv) {
			sc->vooc_curve_index = i;
			break;
		}
	}
	sc->vooc_curve_confirm_count = 0;
	sc->vooc_curve_elapsed_s = 0;
	sc->vooc_curve_initialized = true;
	point = &curve->points[sc->vooc_curve_index];
	dev_info(sc->dev,
		 "VOOC selected stock curve=%s plugin_capacity=%d plugin_temp=%d index=%u target_vbat=%u target_code=%u time=%u exit=%u\n",
		 curve->name, state->capacity, state->temp_decic,
		 sc->vooc_curve_index, point->target_vbat_uv,
		 point->target_ibus_code, point->target_time_s, point->exit);
	return 0;
}

static int sc8547_update_vooc_current_policy(struct sc8547_device *sc)
{
	struct sc8547_vooc_battery_state state;
	const struct sc8547_vooc_curve_point *point;
	u8 requested, active;
	u8 temp_limit;
	bool full;
	bool reduce_active;
	u8 previous_active;
	bool advance = false;
	int ret;

	ret = sc8547_get_vooc_battery_state(&state);
	if (ret)
		return ret;
	if (state.voltage_uv <= 0 || state.temp_decic < -1000)
		return -ENODATA;

	sc->vooc_policy_capacity = state.capacity;
	sc->vooc_policy_vbat_uv = state.voltage_uv;
	sc->vooc_policy_temp_decic = state.temp_decic;
	full = state.status == POWER_SUPPLY_STATUS_FULL ||
		(state.capacity >= SC8547_VOOC_BATTERY_FULL_CAPACITY) ||
		(state.voltage_uv >= SC8547_VOOC_BATTERY_MAX_VOLTAGE_UV);
	sc->vooc_policy_full = full;
	if (state.temp_decic < SC8547_VOOC_BATTERY_MIN_TEMP_DECIC ||
	    state.temp_decic > SC8547_VOOC_BATTERY_MAX_TEMP_DECIC)
		return -ERANGE;
	if (full) {
		dev_info(sc->dev,
			 "VOOC terminal condition status=%d raw_capacity=%d vbat=%d\n",
			 state.status, state.capacity, state.voltage_uv);
		return -EALREADY;
	}
	if (!sc->vooc_curve_initialized) {
		ret = sc8547_vooc_initialize_curve(sc, &state);
		if (ret)
			return ret;
	}
	if (!sc->vooc_curve || sc->vooc_curve_index >= sc->vooc_curve->count)
		return -EINVAL;

	point = &sc->vooc_curve->points[sc->vooc_curve_index];
	if (point->exit && state.voltage_uv >= point->target_vbat_uv) {
		dev_info(sc->dev,
			 "VOOC curve terminal threshold reached curve=%s index=%u vbat=%d target=%u\n",
			 sc->vooc_curve->name, sc->vooc_curve_index,
			 state.voltage_uv, point->target_vbat_uv);
		return -EALREADY;
	}
	/* Match oplus_voocphy_request_fastchg_curv(): inspect the real terminal
	 * voltage once per second, require four consecutive qualifying samples,
	 * and retain the curve across the lower-current re-handshake.  target_time
	 * is a maximum dwell, not a minimum charge duration. */
	if (sc->vooc_initialized && sc->vooc_last_cp_started &&
	    sc->vooc_last_total_ibus_ua > 0 && !point->exit) {
		if (sc->vooc_curve_elapsed_s < U32_MAX)
			sc->vooc_curve_elapsed_s++;
		if (state.voltage_uv > point->target_vbat_uv &&
		    sc->vooc_last_total_ibus_ua <
			(point->target_ibus_code * 100000U +
			 SC8547_VOOC_CURVE_IBUS_MARGIN_UA)) {
			if (sc->vooc_curve_confirm_count < U8_MAX)
				sc->vooc_curve_confirm_count++;
		} else {
			sc->vooc_curve_confirm_count = 0;
		}
		advance = sc->vooc_curve_confirm_count >=
			SC8547_VOOC_CURVE_CONFIRM_SAMPLES ||
			(point->target_time_s &&
			 sc->vooc_curve_elapsed_s >= point->target_time_s);
		if (advance && sc->vooc_curve_index + 1 < sc->vooc_curve->count) {
			u8 old_index = sc->vooc_curve_index;

			sc->vooc_curve_index++;
			sc->vooc_curve_confirm_count = 0;
			sc->vooc_curve_elapsed_s = 0;
			point = &sc->vooc_curve->points[sc->vooc_curve_index];
			dev_info(sc->dev,
				 "VOOC stock curve advanced %u->%u vbat=%d total_ibus=%d next_code=%u next_vbat=%u exit=%u\n",
				 old_index, sc->vooc_curve_index, state.voltage_uv,
				 sc->vooc_last_total_ibus_ua,
				 point->target_ibus_code, point->target_vbat_uv,
				 point->exit);
		}
	}

	requested = clamp_t(u8, SC8547_VOOC_DEFAULT_CURRENT_CODE,
				SC8547_VOOC_MIN_CURRENT_CODE,
				SC8547_VOOC_MAX_CURRENT_CODE);
	temp_limit = sc8547_vooc_temp_current_limit(sc, state.temp_decic);
	active = min3(requested, point->target_ibus_code, temp_limit);
	if (active < SC8547_VOOC_MIN_CURRENT_CODE)
		active = SC8547_VOOC_MIN_CURRENT_CODE;
	previous_active = sc->vooc_active_current_code;
	reduce_active = sc->vooc_initialized && active < previous_active;

	if (sc->vooc_active_current_code != active ||
	    sc->vooc_policy_clamped != (active < requested))
		dev_info(sc->dev,
			 "VOOC battery policy capacity=%d vbat=%d temp=%d code=%u requested=%u curve=%s index=%u elapsed=%u confirm=%u\n",
			 state.capacity, state.voltage_uv, state.temp_decic,
			 active, requested, sc->vooc_curve->name,
			 sc->vooc_curve_index, sc->vooc_curve_elapsed_s,
			 sc->vooc_curve_confirm_count);
	sc->vooc_active_current_code = active;
	sc->vooc_policy_clamped = active < requested;
	if (reduce_active)
		dev_info(sc->dev,
			 "VOOC battery policy reduced current %u -> %u; applying on next request\n",
			 previous_active, active);
	return 0;
}

static int sc8547_get_vout_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VOUT_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VOUT_UV_PER_LSB);
}

/*
 * Caihong's secondary pump has a valid VOUT sense path but its local VBAT
 * ADC is not connected: hardware reads show VOUT tracking the system battery
 * while VBAT remains fixed at one 5-mV conversion.  The downstream legacy
 * slave PPS path likewise consumes REG19 (VOUT), not REG1B (VBAT).  Keep the
 * validated primary on VBAT and select VOUT only for the secondary role.
 */
static bool sc8547_uses_vout_reference(const struct sc8547_device *sc)
{
	return !strcmp(sc->role, "secondary");
}

static int sc8547_get_output_reference_uv(struct sc8547_device *sc)
{
	if (sc8547_uses_vout_reference(sc))
		return sc8547_get_vout_uv(sc);

	return sc8547_get_vbat_uv(sc);
}

static int sc8547_select_output_reference_uv(const struct sc8547_device *sc,
					       int vout_uv, int vbat_uv)
{
	return sc8547_uses_vout_reference(sc) ? vout_uv : vbat_uv;
}

static int sc8547_get_vac_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VAC_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VAC_UV_PER_LSB);
}

static int sc8547_get_tdie_mc(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_TDIE_ADC_H,
			       SC8547_TDIE_H_MASK, SC8547_TDIE_MC_PER_LSB);
}

static int sc8547_read_status(struct sc8547_device *sc,
				       unsigned int *reg06, unsigned int *reg0e)
{
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, reg06);
	if (ret)
		return ret;

	return regmap_read(sc->regmap, SC8547_REG_STATUS_0E, reg0e);
}

static bool sc8547_has_blocking_fault(unsigned int reg06, unsigned int reg0e)
{
	return !!((reg06 & SC8547_BLOCKING_06) ||
		  (reg0e & SC8547_BLOCKING_0E));
}

static bool sc8547_has_fatal_fault(unsigned int reg06, unsigned int reg0e)
{
	return !!((reg06 & SC8547_FATAL_06) ||
		  (reg0e & SC8547_FATAL_0E));
}

static enum power_supply_property sc8547_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CALIBRATE,
};

static int sc8547_psy_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sc8547_device *sc = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, &reg);
		if (ret)
			return ret;
		val->intval = reg & SC8547_CP_SWITCHING_STAT ?
			POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(sc->regmap, SC8547_REG_STATUS_0E, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SC8547_ADAPTER_INSERT_STAT);
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SC8547_CHG_EN);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8547_get_vbus_uv(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		/* Board-local coordination input: expose the pump's VBAT ADC. */
		ret = sc8547_get_vbat_uv(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		/* Board-local coordination input: expose the pump's VOUT ADC. */
		ret = sc8547_get_vout_uv(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8547_get_ibus_ua(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = sc8547_get_tdie_mc(sc);
		if (ret < 0)
			return ret;
		val->intval = ret / 100;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		val->intval = sc->pps_target_vbus_uv;
		return 0;
	case POWER_SUPPLY_PROP_CALIBRATE:
		val->intval = sc->pps_prepared;
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * qcom_battmgr coordinates source voltage and charge-pump startup through
 * these standard power-supply properties.  Keep this interface deliberately
 * small: target voltage, prepare, run, cancel and restore.
 */
static int sc8547_psy_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct sc8547_device *sc = power_supply_get_drvdata(psy);
	bool touched = false;
	int ret;

	if (psp != POWER_SUPPLY_PROP_CALIBRATE &&
	    psp != POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT)
		return -EINVAL;

	/*
	 * Cancellation must not wait behind the mutex held by the bounded run.
	 * It changes no register here; the run observes it within one 500-ms
	 * sample and executes its normal disable/readback/restore path.
	 */
	if (psp == POWER_SUPPLY_PROP_CALIBRATE &&
	    val->intval == SC8547_POLICY_CANCEL_CMD) {
		atomic_set(&sc->policy_cancel, 1);
		return 0;
	}

	mutex_lock(&sc->lock);
	if (psp == POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT) {
		if (val->intval == 0) {
			sc->pps_target_vbus_uv = 0;
			ret = 0;
			goto out;
		}
		if (val->intval < 0 || !sc->pps_capable ||
		    !sc->window.complete ||
		    !sc8547_bounded_role_supported(sc)) {
			ret = -EPERM;
			goto out;
		}
		if ((u32)val->intval < sc->window.vbus_min_uv ||
		    (u32)val->intval > sc->window.vbus_max_uv -
					 sc->window.start_overshoot_max_uv) {
			ret = -ERANGE;
			goto out;
		}
		/*
		 * The policy-owned target is the first controlled CP operation.
		 * Make it self-contained after boot by applying the same validated,
		 * fail-closed profile previously reached through apply_init.
		 */
		if (!sc->init_done) {
			ret = sc8547_restore_idle_profile(sc);
			if (ret)
				goto out;
		}
		sc->pps_target_vbus_uv = val->intval;
		ret = 0;
		goto out;
	}
	if (val->intval == SC8547_POLICY_CONTINUOUS_CMD) {
		ret = sc8547_run_pps_continuous(sc);
		goto out;
	}
	if (val->intval == SC8547_POLICY_RUN_MS) {
		if (strcmp(sc->role, "primary"))
			ret = -EPERM;
		else
			ret = sc8547_run_pps_bounded(sc, val->intval, false);
		goto out;
	}
	if (val->intval != 0 && val->intval != 1) {
		ret = -EINVAL;
		goto out;
	}
	if (!val->intval) {
		ret = sc8547_restore_idle_profile(sc);
		goto out;
	}

	if (!sc->pps_capable || !sc->window.complete ||
	    !sc8547_bounded_role_supported(sc)) {
		ret = -EPERM;
		goto out;
	}
	if (!sc->init_done) {
		ret = sc8547_restore_idle_profile(sc);
		if (ret)
			goto out;
	}
	atomic_set(&sc->policy_cancel, 0);

	ret = sc8547_prepare_vendor_pps_startup(sc, &touched);
	if (!ret)
		sc->pps_prepared = true;
	else if (touched)
		sc8547_restore_idle_profile(sc);
out:
	mutex_unlock(&sc->lock);
	return ret;
}

static int sc8547_psy_property_is_writeable(struct power_supply *psy,
					     enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CALIBRATE ||
	       psp == POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT;
}

static void sc8547_init_board_config(struct sc8547_device *sc)
{
	struct sc8547_raw_profile *p = &sc->profile;
	struct sc8547_enable_window *w = &sc->window;
	bool primary = !strcmp(sc->role, "primary");

	/* Fixed values from the stock project-23926 SC8547A nodes. */
	p->reg00 = 0x36;
	p->reg02 = 0x01;
	p->reg04 = 0x50;
	p->reg05 = 0x1b;
	p->complete = true;
	if (primary) {
		p->reg01 = 0xbf;
		p->reg0d = 0x50;
		p->has_reg01 = true;
		p->has_reg0d = true;
	}

	w->vbus_min_uv = 7650000;
	w->vbus_max_uv = 9400000;
	w->active_vbus_min_uv = 7300000;
	w->ratio_error_max_uv = 250000;
	w->start_delta_uv = 450000;
	w->start_overshoot_max_uv = 200000;
	w->vbat_min_uv = 3600000;
	w->vbat_max_uv = primary ? 4450000 : 4500000;
	w->start_reference_max_uv = primary ? 4399999 : 4350000;
	w->complete = true;

	sc->tdie_max_mc = 70000;
	sc->pps_capable = sc8547_bounded_role_supported(sc);
	sc->vooc_capable = primary &&
		sc->variant == SC8547_VARIANT_SC8547A;
	sc->vooc_active_current_code = SC8547_VOOC_DEFAULT_CURRENT_CODE;
	sc->vooc_policy_capacity = -ENODATA;
	sc->vooc_policy_vbat_uv = -ENODATA;
	sc->vooc_policy_temp_decic = -ENODATA;
	sc8547_vooc_reset_curve(sc);
}

static int sc8547_fail_closed(struct sc8547_device *sc)
{
	int first = 0;
	int ret;

	ret = sc8547_set_charge_enabled(sc, false);
	if (ret)
		first = ret;
	ret = sc8547_set_watchdog_code(sc, 0);
	if (ret && !first)
		first = ret;

	return first;
}

static int sc8547_verify_reg(struct sc8547_device *sc, unsigned int reg,
			     u8 expected, u8 mask)
{
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, reg, &val);
	if (ret)
		return ret;
	if ((val & mask) != (expected & mask))
		return -EIO;

	return 0;
}

static int sc8547_profile_readback(struct sc8547_device *sc)
{
	const struct sc8547_raw_profile *p = &sc->profile;
	unsigned int val;
	int ret;

	ret = sc8547_verify_reg(sc, SC8547_REG_BAT_OVP, p->reg00,
				SC8547_BAT_OVP_DIS | SC8547_BAT_OVP_MASK);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_AC_OVP, &val);
	if (ret)
		return ret;
	if ((val & SC8547_AC_OVP_CONFIG_MASK) !=
	    (p->reg02 & SC8547_AC_OVP_CONFIG_MASK) ||
	    (val & SC8547_AC_OVP_STAT))
		return -EIO;
	ret = sc8547_verify_reg(sc, SC8547_REG_VBUS_OVP, p->reg04, 0xff);
	if (ret)
		return ret;
	ret = sc8547_verify_reg(sc, SC8547_REG_IBUS_PROT, p->reg05, 0xff);
	if (ret)
		return ret;
	if (p->has_reg01) {
		ret = sc8547_verify_reg(sc, SC8547_REG_BAT_OCP, p->reg01,
					0xff);
		if (ret)
			return ret;
	}
	if (p->has_reg0d) {
		ret = sc8547_verify_reg(sc, SC8547_REG_PMID2OUT, p->reg0d,
					SC8547_PMID2OUT_UVP_MASK |
					SC8547_PMID2OUT_OVP_MASK);
		if (ret)
			return ret;
	}
	if (!strcmp(sc->role, "secondary")) {
		ret = sc8547_verify_reg(sc, SC8547_REG_VOOCPHY_CTRL,
					SC8547A_VENDOR_VOOCPHY_DISABLE, 0xff);
		if (ret)
			return ret;
		ret = sc8547_verify_reg(sc, SC8547_REG_SLAVE_CTRL,
					SC8547_VENDOR_SECONDARY_CTRL, 0xff);
		if (ret)
			return ret;
		ret = sc8547_verify_reg(sc, SC8547_REG_VBUS_RANGE_CTRL,
					SC8547_VENDOR_SECONDARY_VBUS_RANGE, 0xff);
		if (ret)
			return ret;
	}

	return 0;
}

static int sc8547_restore_idle_profile(struct sc8547_device *sc)
{
	const struct sc8547_raw_profile *p = &sc->profile;
	int ret;

	if (!sc8547_variant_control_supported(sc->variant))
		return -EOPNOTSUPP;
	if (!p->complete)
		return -EINVAL;

	sc->init_done = false;
	sc->pps_prepared = false;
	sc->pps_target_vbus_uv = 0;
	ret = sc8547_fail_closed(sc);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc->regmap, SC8547_REG_CHG_CTRL,
				 SC8547_REG_RESET, SC8547_REG_RESET);
	if (ret)
		goto fail;
	/* The vendor UFCS path allows 10 ms for reset to settle before init. */
	msleep(SC8547_RESET_SETTLE_MS);

	ret = sc8547_fail_closed(sc);
	if (ret)
		goto fail;

	ret = regmap_write(sc->regmap, SC8547_REG_BAT_OVP, p->reg00);
	if (ret)
		goto fail;
	if (p->has_reg01) {
		ret = regmap_write(sc->regmap, SC8547_REG_BAT_OCP, p->reg01);
		if (ret)
			goto fail;
	}
	ret = regmap_write(sc->regmap, SC8547_REG_AC_OVP, p->reg02);
	if (ret)
		goto fail;
	ret = regmap_write(sc->regmap, SC8547_REG_VBUS_OVP, p->reg04);
	if (ret)
		goto fail;
	ret = regmap_write(sc->regmap, SC8547_REG_IBUS_PROT, p->reg05);
	if (ret)
		goto fail;
	if (p->has_reg0d) {
		ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT, p->reg0d);
		if (ret)
			goto fail;
	}
	if (!strcmp(sc->role, "secondary")) {
		/* Complete the downstream slave idle profile after reset. */
		ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_CTRL,
				   SC8547A_VENDOR_VOOCPHY_DISABLE);
		if (ret)
			goto fail;
		ret = regmap_write(sc->regmap, SC8547_REG_SLAVE_CTRL,
				   SC8547_VENDOR_SECONDARY_CTRL);
		if (ret)
			goto fail;
		ret = regmap_write(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL,
				   SC8547_VENDOR_SECONDARY_VBUS_RANGE);
		if (ret)
			goto fail;
	}

	ret = sc8547_fail_closed(sc);
	if (ret)
		goto fail;
	ret = sc8547_set_adc_enabled(sc, true);
	if (ret)
		goto fail;
	ret = sc8547_enable_required_adc_channels(sc);
	if (ret)
		goto fail;
	ret = sc8547_profile_readback(sc);
	if (ret)
		goto fail;

	sc->init_done = true;
	return 0;

fail:
	sc8547_fail_closed(sc);
	sc->init_done = false;
	sc->pps_prepared = false;
	return ret;
}

static int sc8547_vooc_write_word(struct sc8547_device *sc,
				   unsigned int reg, u16 value)
{
	u8 data[2] = {
		value & 0xff,
		value >> 8,
	};

	/*
	 * Match the vendor i2c_smbus_write_word_data() transaction.  The SC8547
	 * expects the low byte at the named high-byte register followed by the
	 * upper byte; keep both bytes in one I2C transfer so a fast adapter cannot
	 * observe an incomplete TX word between separate register writes.
	 */
	return regmap_raw_write(sc->regmap, reg, data, sizeof(data));
}

/*
 * The vendor VOOC entry initializes the downstream pump before arming the
 * primary PHY. Keep that setup local to the VOOC session and restore every
 * register afterwards so a failed handshake cannot alter normal charging.
 */
struct sc8547_vooc_secondary_state {
	struct sc8547_device *sc;
	u8 reg00;
	u8 reg02;
	u8 reg04;
	u8 reg05;
	u8 reg08;
	u8 reg09;
	u8 reg0c;
	u8 reg0d;
	u8 reg07;
	u8 reg11;
	u8 reg2b;
	u8 reg30;
	u8 reg3c;
	u8 last_reg06;
	u8 last_reg07;
	u8 last_reg0e;
	int last_vbus_uv;
	int last_ibus_ua;
	int last_vout_uv;
	int last_tdie_mc;
	unsigned int low_current_samples;
	bool enabled;
	bool switching_confirmed;
	bool reenable_needed;
	bool held_off_logged;
	bool ever_enabled;
	bool locked;
	bool valid;
};

struct sc8547_secondary_find_context {
	struct sc8547_device *primary;
	struct sc8547_device *secondary;
};

static int sc8547_find_secondary_device(struct device *dev, void *data)
{
	struct sc8547_secondary_find_context *ctx = data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8547_device *candidate;

	if (!client || client == to_i2c_client(ctx->primary->dev))
		return 0;
	if (!client->dev.driver || strcmp(client->dev.driver->name, "sc8547"))
		return 0;

	candidate = i2c_get_clientdata(client);
	if (!candidate || !candidate->role ||
	    strcmp(candidate->role, "secondary"))
		return 0;
	if (candidate->variant != SC8547_VARIANT_SC8547A &&
	    candidate->variant != SC8547_VARIANT_SC8547)
		return 0;

	ctx->secondary = candidate;
	return 1;
}

static int sc8547_vooc_secondary_disable(
	struct sc8547_vooc_secondary_state *state)
{
	struct sc8547_device *sec = state->sc;
	int ret;

	if (!state->locked || !state->enabled)
		return 0;

	ret = sc8547_set_charge_enabled(sec, false);
	if (!ret)
		usleep_range(10000, 20000);
	state->enabled = false;
	state->switching_confirmed = false;
	if (ret)
		dev_warn(sec->dev, "VOOC secondary disable failed: %d\n", ret);
	return ret;
}

/*
 * The two SC8547 inputs are independent current-sense paths.  Do not infer
 * secondary health from the primary ADC: a stopped or overheated slave can
 * leave the primary looking normal while the parallel path is unsafe.
 */
static int sc8547_vooc_secondary_monitor(
	struct sc8547_vooc_secondary_state *state,
	int primary_ibus_ua, int *total_ibus_ua)
{
	struct sc8547_device *sec = state->sc;
	unsigned int reg06, reg07, reg0e;
	int vbus_uv, ibus_ua, vout_uv, tdie_mc;
	int ret;

	if (!state->locked || !state->valid || !state->enabled)
		return 0;

	ret = sc8547_read_status(sec, &reg06, &reg0e);
	if (ret)
		return ret;
	ret = regmap_read(sec->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;
	vbus_uv = sc8547_get_vbus_uv(sec);
	if (vbus_uv < 0) {
		dev_warn(sec->dev, "VOOC secondary VBUS ADC failed: %d\n", vbus_uv);
		return vbus_uv;
	}
	ibus_ua = sc8547_get_ibus_ua(sec);
	if (ibus_ua < 0) {
		dev_warn(sec->dev, "VOOC secondary IBUS ADC failed: %d\n", ibus_ua);
		return ibus_ua;
	}
	vout_uv = sc8547_get_vout_uv(sec);
	if (vout_uv < 0) {
		dev_warn(sec->dev, "VOOC secondary VOUT ADC failed: %d\n", vout_uv);
		return vout_uv;
	}
	tdie_mc = sc8547_get_tdie_mc(sec);
	if (tdie_mc < 0) {
		dev_warn(sec->dev, "VOOC secondary TDIE ADC failed: %d\n", tdie_mc);
		return tdie_mc;
	}

	state->last_reg06 = reg06;
	state->last_reg07 = reg07;
	state->last_reg0e = reg0e;
	state->last_vbus_uv = vbus_uv;
	state->last_ibus_ua = ibus_ua;
	state->last_vout_uv = vout_uv;
	state->last_tdie_mc = tdie_mc;
	if (total_ibus_ua)
		*total_ibus_ua = primary_ibus_ua >= 0 ?
			primary_ibus_ua + ibus_ua : -ENODATA;

	if (!(reg07 & SC8547_CHG_EN) ||
	    !(reg06 & SC8547_CP_SWITCHING_STAT) ||
	    sc8547_has_fatal_fault(reg06, reg0e)) {
		dev_warn(sec->dev,
			 "VOOC secondary status rejected reg06=0x%02x reg07=0x%02x "
			 "reg0e=0x%02x vbus=%d ibus=%d vout=%d tdie=%d\n",
			 reg06, reg07, reg0e, vbus_uv, ibus_ua, vout_uv, tdie_mc);
		return -EIO;
	}
	if (vbus_uv < SC8547_VOOC_CP_MIN_VBUS_UV ||
	    vbus_uv > SC8547_VOOC_MAX_VBUS_UV ||
	    ibus_ua > SC8547_VOOC_CP_MAX_IBUS_UA ||
	    (u32)tdie_mc > sec->tdie_max_mc) {
		dev_warn(sec->dev,
			 "VOOC secondary range rejected reg06=0x%02x reg07=0x%02x "
			 "reg0e=0x%02x vbus=%d ibus=%d vout=%d tdie=%d\n",
			 reg06, reg07, reg0e, vbus_uv, ibus_ua, vout_uv, tdie_mc);
		return -ERANGE;
	}

	/*
	 * The vendor CP_ENABLE path deliberately removes the startup UCP and
	 * soft-start timeout only until switching has been confirmed.  Restore
	 * both protections before exposing the pump to the longer session, then
	 * refresh the five-second watchdog on every sample.
	 */
	ret = regmap_write(sec->regmap, SC8547_REG_IBUS_PROT,
			   SC8547A_VENDOR_SECONDARY_IBUS_PROT);
	if (ret) {
		dev_warn(sec->dev, "VOOC secondary UCP/OCP restore failed: %d\n", ret);
		return ret;
	}
	ret = regmap_update_bits(sec->regmap, SC8547_REG_SS_CTRL,
				 SC8547_SS_TIMEOUT_MASK,
				 SC8547A_VENDOR_CP_RUNTIME_SS_TIMEOUT);
	if (ret) {
		dev_warn(sec->dev, "VOOC secondary soft-start restore failed: %d\n", ret);
		return ret;
	}
	ret = sc8547_set_watchdog_code(sec, SC8547A_VENDOR_PPS_WATCHDOG);
	if (ret) {
		dev_warn(sec->dev, "VOOC secondary watchdog update failed: %d\n", ret);
		return ret;
	}
	if (primary_ibus_ua < 0 ||
	    primary_ibus_ua > SC8547_VOOC_CP_MAX_IBUS_UA ||
	    (total_ibus_ua && *total_ibus_ua > SC8547_VOOC_TOTAL_MAX_IBUS_UA)) {
		dev_warn(sec->dev,
			 "VOOC secondary aggregate range rejected primary=%d "
			 "secondary=%d total=%d\n", primary_ibus_ua, ibus_ua,
			 total_ibus_ua ? *total_ibus_ua : -ENODATA);
		return -ERANGE;
	}

	/*
	 * The stock monitor removes the secondary after a sustained low-current
	 * window, then lets it return once the primary path recovers above the
	 * 1.4-A enable threshold.  Our ADC cadence is 25 ms, so 20 samples gives
	 * the vendor-like 500-ms hysteresis without rapid pump chatter.
	 */
	if (total_ibus_ua && *total_ibus_ua <
			SC8547_VOOC_SECONDARY_DISABLE_MIN_IBUS_UA) {
		if (state->low_current_samples <
			    SC8547_VOOC_SECONDARY_DISABLE_SAMPLES)
			state->low_current_samples++;
	} else {
		state->low_current_samples = 0;
	}
	if (state->low_current_samples >= SC8547_VOOC_SECONDARY_DISABLE_SAMPLES) {
		ret = sc8547_vooc_secondary_disable(state);
		if (ret)
			return ret;
		state->reenable_needed = true;
		state->low_current_samples = 0;
		dev_info(sec->dev,
			 "VOOC secondary disabled after low aggregate current total=%d\n",
			 total_ibus_ua ? *total_ibus_ua : -ENODATA);
		return 0;
	}

	state->switching_confirmed = true;
	return 0;
}

static int sc8547_vooc_secondary_enable(
	struct sc8547_vooc_secondary_state *state,
	int primary_ibus_ua, int *total_ibus_ua)
{
	struct sc8547_device *sec = state->sc;
	int ret;

	if (!state->locked || !state->valid)
		return -ENODEV;
	if (state->enabled)
		return sc8547_vooc_secondary_monitor(state, primary_ibus_ua,
						     total_ibus_ua);
	if (primary_ibus_ua < SC8547_VOOC_SECONDARY_ENABLE_MIN_IBUS_UA)
		return -EAGAIN;

	/*
	 * Mirror sc8547_slave_cp_enable() from the vendor driver.  The slave
	 * clears REG07 when VAC_OK/UCP/soft-start preconditions are not satisfied;
	 * writing 0x81 alone therefore looks successful on the bus but leaves the
	 * pump disabled a few hundred milliseconds later.
	 */
	ret = regmap_write(sec->regmap, SC8547_REG_AC_OVP, 0x01);
	if (ret)
		return ret;
	ret = regmap_write(sec->regmap, SC8547_REG_VBUS_OVP,
			   SC8547A_VENDOR_CP_ENABLE_VBUS_OVP);
	if (ret)
		return ret;
	ret = regmap_write(sec->regmap, SC8547_REG_IBUS_PROT,
			   SC8547A_VENDOR_CP_ENABLE_IBUS_PROT);
	if (ret)
		return ret;
	ret = regmap_update_bits(sec->regmap, SC8547_REG_SS_CTRL,
				 SC8547_SS_TIMEOUT_MASK,
				 SC8547A_VENDOR_CP_ENABLE_SS_TIMEOUT);
	if (ret)
		return ret;
	ret = regmap_write(sec->regmap, SC8547_REG_MODE_CTRL,
			   SC8547A_VENDOR_CP_ENABLE_MODE);
	if (ret)
		return ret;
	ret = regmap_write(sec->regmap, SC8547_REG_ADC_CTRL, SC8547_ADC_EN);
	if (ret)
		return ret;
	ret = regmap_write(sec->regmap, SC8547_REG_PMID2OUT,
			   SC8547A_VENDOR_PPS_PMID2OUT);
	if (ret)
		return ret;
	ret = regmap_write(sec->regmap, SC8547_REG_IBUS_CTRL, 0x02);
	if (ret)
		return ret;
	msleep(50);
	ret = sc8547_set_charge_enabled(sec, true);
	if (ret) {
		dev_warn(sec->dev, "VOOC secondary work-start write failed: %d\n", ret);
		return ret;
	}
	state->enabled = true;
	state->ever_enabled = true;
	state->reenable_needed = false;
	state->held_off_logged = false;
	state->low_current_samples = 0;
	msleep(SC8547_VOOC_SECONDARY_START_SETTLE_MS);
	ret = sc8547_vooc_secondary_monitor(state, primary_ibus_ua,
						    total_ibus_ua);
	if (ret) {
		dev_warn(sec->dev,
			 "VOOC secondary post-start monitor failed: %d\n", ret);
		sc8547_vooc_secondary_disable(state);
		return ret;
	}

	dev_info(sec->dev,
		 "VOOC secondary CP switching confirmed vbus=%d ibus=%d total=%d tdie=%d\n",
		 state->last_vbus_uv, state->last_ibus_ua,
		 total_ibus_ua ? *total_ibus_ua : -ENODATA,
		 state->last_tdie_mc);
	return 0;
}

static int sc8547_vooc_secondary_restore(
	struct sc8547_vooc_secondary_state *state)
{
	struct sc8547_device *sec = state->sc;
	int first = 0;
	int ret;

	if (!state->locked)
		return 0;

	ret = sc8547_vooc_secondary_disable(state);
	if (ret && !first)
		first = ret;
	if (state->valid) {
		ret = regmap_write(sec->regmap, SC8547_REG_BAT_OVP,
				   state->reg00);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_AC_OVP,
				   state->reg02);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_VBUS_OVP,
				   state->reg04);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_IBUS_PROT,
				   state->reg05);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_SS_CTRL,
				   state->reg08);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_MODE_CTRL,
				   state->reg09);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_IBUS_CTRL,
				   state->reg0c);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_PMID2OUT,
				   state->reg0d);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_ADC_CTRL,
				   state->reg11);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_VOOCPHY_CTRL,
				   state->reg2b);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_VOOC_INT_MASK,
				   state->reg30);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_VBUS_RANGE_CTRL,
				   state->reg3c);
		if (ret && !first)
			first = ret;
		ret = regmap_write(sec->regmap, SC8547_REG_CHG_CTRL,
				   state->reg07);
		if (ret && !first)
			first = ret;
	}

	mutex_unlock(&sec->lock);
	state->locked = false;
	if (first)
		dev_warn(sec->dev, "VOOC secondary restore failed: %d\n", first);
	return first;
}

static int sc8547_vooc_secondary_prepare(
	struct sc8547_device *primary,
	struct sc8547_vooc_secondary_state *state)
{
	struct sc8547_secondary_find_context ctx = {
		.primary = primary,
	};
	struct sc8547_device *sec;
	unsigned int val;
	int ret;

	i2c_for_each_dev(&ctx, sc8547_find_secondary_device);
	sec = ctx.secondary;
	if (!sec) {
		dev_warn(primary->dev,
			 "VOOC secondary device not found; probing primary only\n");
		return 0;
	}

	state->sc = sec;
	mutex_lock(&sec->lock);
	state->locked = true;

	ret = regmap_read(sec->regmap, SC8547_REG_BAT_OVP, &val);
	if (ret)
		goto fail;
	state->reg00 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_AC_OVP, &val);
	if (ret)
		goto fail;
	state->reg02 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_VBUS_OVP, &val);
	if (ret)
		goto fail;
	state->reg04 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_IBUS_PROT, &val);
	if (ret)
		goto fail;
	state->reg05 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_SS_CTRL, &val);
	if (ret)
		goto fail;
	state->reg08 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_MODE_CTRL, &val);
	if (ret)
		goto fail;
	state->reg09 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_IBUS_CTRL, &val);
	if (ret)
		goto fail;
	state->reg0c = val;
	ret = regmap_read(sec->regmap, SC8547_REG_PMID2OUT, &val);
	if (ret)
		goto fail;
	state->reg0d = val;
	ret = regmap_read(sec->regmap, SC8547_REG_CHG_CTRL, &val);
	if (ret)
		goto fail;
	state->reg07 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_ADC_CTRL, &val);
	if (ret)
		goto fail;
	state->reg11 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_VOOCPHY_CTRL, &val);
	if (ret)
		goto fail;
	state->reg2b = val;
	ret = regmap_read(sec->regmap, SC8547_REG_VOOC_INT_MASK, &val);
	if (ret)
		goto fail;
	state->reg30 = val;
	ret = regmap_read(sec->regmap, SC8547_REG_VBUS_RANGE_CTRL, &val);
	if (ret)
		goto fail;
	state->reg3c = val;
	state->valid = true;
	state->last_vbus_uv = -ENODATA;
	state->last_ibus_ua = -ENODATA;
	state->last_vout_uv = -ENODATA;
	state->last_tdie_mc = -ENODATA;

	ret = regmap_update_bits(sec->regmap, SC8547_REG_CHG_CTRL,
				 SC8547_REG_RESET, SC8547_REG_RESET);
	if (ret)
		goto fail;
	msleep(SC8547_RESET_SETTLE_MS);
	ret = regmap_write(sec->regmap, SC8547_REG_ADC_CTRL, 0x00);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_AC_OVP, 0x01);
	if (ret)
		goto fail;
	/* Match sc8547_slave_svooc_hw_setting() before the master starts. */
	ret = regmap_write(sec->regmap, SC8547_REG_VBUS_OVP, 0x50);
	if (ret)
		goto fail;
	/* Match sc8547_slave_svooc_hw_setting(): SC8547A uses the
	 * 5-ms UCP-fall setting in bit 4 plus the 3.6-A OCP code. */
	ret = regmap_write(sec->regmap, SC8547_REG_IBUS_PROT,
			   sec->variant == SC8547_VARIANT_SC8547A ?
			   SC8547A_VENDOR_SECONDARY_IBUS_PROT :
			   SC8547_VENDOR_SECONDARY_IBUS_PROT);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_MODE_CTRL, 0x03);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_IBUS_CTRL, 0x00);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_PMID2OUT, 0x70);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_VOOCPHY_CTRL, 0x00);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_VOOC_INT_MASK, 0x7f);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_VBUS_RANGE_CTRL, 0x40);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_ADC_CTRL, 0x80);
	if (ret)
		goto fail;
	ret = regmap_write(sec->regmap, SC8547_REG_CHG_CTRL, 0x01);
	if (ret)
		goto fail;

	dev_info(primary->dev,
		 "VOOC secondary initialized for bounded handshake (CP off)\n");
	return 0;

fail:
	if (state->valid) {
		sc8547_vooc_secondary_restore(state);
	} else {
		mutex_unlock(&sec->lock);
		state->locked = false;
	}
	return ret;
}

static int sc8547_vooc_cleanup(struct sc8547_device *sc, u8 saved_loose,
				       u8 saved_timing)
{
	int first = 0;
	int ret;

	/* Disable the protocol before disconnecting the analog DP/DM path. */
	ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_CTRL, 0);
	if (ret)
		first = ret;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOC_DPDM_CTRL,
				  SC8547_VOOC_DPDM_HIZ);
	if (ret && !first)
		first = ret;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOC_DPDM_CFG, 0);
	if (ret && !first)
		first = ret;
	ret = sc8547_vooc_write_word(sc, SC8547_REG_VOOC_TX_DATA_H, 0);
	if (ret && !first)
		first = ret;
	ret = sc8547_vooc_write_word(sc, SC8547_REG_VOOC_PRE_DATA_H, 0);
	if (ret && !first)
		first = ret;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOC_INT_MASK,
				  SC8547_VOOC_INT_MASKED);
	if (ret && !first)
		first = ret;

	/* These are normal idle-profile values and were not protocol inputs. */
	ret = regmap_write(sc->regmap, SC8547_REG_LOOSE_DET, saved_loose);
	if (ret && !first)
		first = ret;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_TIMING,
				  saved_timing);
	if (ret && !first)
		first = ret;

	sc->vooc_initialized = false;
	return first;
}

static void sc8547_vooc_select_tx_head(struct sc8547_device *sc, u8 rx)
{
	u8 head = (rx & 0xe0) >> 5;
	u8 move_head = (rx & 0x70) >> 4;

	/* Match the vendor priority and its staged VOOC2 detection. The first
	 * SVOOC probe frame is commonly 0xa8, whose framing bits look like VOOC2;
	 * changing the reply head immediately makes the adapter report ERR_TRANS. */
	if (head == SC8547_VOOC_HEAD_SVOOC ||
	    move_head == SC8547_VOOC_HEAD_SVOOC)
		sc->vooc_tx_head = SC8547_VOOC_INVERT_SVOOC;
	else if (head == SC8547_VOOC_HEAD_VOOC30 ||
		 move_head == SC8547_VOOC_HEAD_VOOC30)
		sc->vooc_tx_head = SC8547_VOOC_INVERT_VOOC30;
	else if (head == SC8547_VOOC_HEAD_VOOC20 ||
		 move_head == SC8547_VOOC_HEAD_VOOC20) {
		if (sc->vooc_adapter_model_detect_remaining)
			sc->vooc_adapter_model_detect_remaining--;
		else
			sc->vooc_tx_head = SC8547_VOOC_INVERT_VOOC20;
	}
}

static int sc8547_vooc_send_data(struct sc8547_device *sc, u8 data,
					 u8 *tx_h, u8 *tx_l)
{
	u16 source = sc->vooc_tx_head | ((u16)(data & 0x7f) << 3);
	u16 wire = 0;
	int bit;

	for (bit = 0; bit < 10; bit++)
		wire |= ((source >> bit) & 0x1) << (9 - bit);

	*tx_h = wire & 0xff;
	*tx_l = (wire >> 8) & 0x03;
	return sc8547_vooc_write_word(sc, SC8547_REG_VOOC_TX_DATA_H,
					      (*tx_h << 8) | *tx_l);
}

static void sc8547_vooc_identification(struct sc8547_device *sc, u8 *data)
{
	u32 random = get_random_u32();
	u8 l = random & 0x7f;
	u8 h = (random >> 8) & 0x7f;
	u8 first_l, first_h;

	sc->vooc_ident_local = ((u16)h << 9) | ((u16)l << 1);
	first_l = ((l >> 6) & 0x1) |
		  (((l >> 4) & 0x1) << 1) |
		  (((h >> 6) & 0x1) << 2) |
		  ((l & 0x1) << 3) |
		  ((h & 0x1) << 4);
	first_h = ((h >> 4) & 0x1) | (((l >> 2) & 0x1) << 1);
	sc->vooc_ident_next_l = ((l >> 3) & 0x1) |
				(((h >> 5) & 0x1) << 1) |
				(((l >> 5) & 0x1) << 2) |
				(((h >> 1) & 0x1) << 3) |
				(((h >> 2) & 0x1) << 4);
	sc->vooc_ident_next_h = ((l >> 1) & 0x1) |
				(((h >> 3) & 0x1) << 1);
	sc->vooc_ident_far = 0;
	sc->vooc_commu_remaining = 5;
	*data = first_l | (first_h << 5);
}

static u8 sc8547_vooc_batt_model_data(unsigned int row)
{
	/*
	 * OnePlus Pad Pro uses the six seven-bit SVOOC battery-system rows from
	 * the stock oplus,sc8547a@6f DT node.  The protocol asks for row 6 first
	 * and row 1 last; the caller passes that reverse-order index here.  Each
	 * value is the little-endian packing of the seven individual DT bits,
	 * matching oplus_voocphy_handle_ask_bat_model_process_cmd().
	 */
	static const u8 curve[SC8547_VOOC_BAT_SYS_COUNT] = {
		0x0a, 0x0f, 0x2c, 0x0f, 0x68, 0x0f,
	};

	return row < ARRAY_SIZE(curve) ? curve[row] : 0;
}

/*
 * VOOC places the seven-bit value on TX0 bit3..7 followed by TX1 bit0..1.
 * Those wire positions correspond to value bits 6..0 in that order; they are
 * not a little-endian integer field.  Keep battery/current replies identical
 * to oplus_voocphy's two write_mesg_mask() calls.
 */
static u8 sc8547_vooc_pack_value(u8 value)
{
	return ((value >> 6) & 0x01) |
		(((value >> 5) & 0x01) << 1) |
		(((value >> 4) & 0x01) << 2) |
		(((value >> 3) & 0x01) << 3) |
		(((value >> 2) & 0x01) << 4) |
		(((value >> 1) & 0x01) << 5) |
		((value & 0x01) << 6);
}

/*
 * The vendor SVOOC path reports the cell-side voltage from the live battery
 * gauge/CP ADC.  The half-bus estimate is only a fallback for the short
 * interval where that ADC is unavailable; feeding VBUS/2 back while a valid
 * VBAT sample exists creates a positive feedback loop and drives the adapter
 * above the intended 2:1 operating point.
 */
static int sc8547_vooc_report_bat_uv(int vbus_uv, int measured_vbat_uv,
				     int ibus_ua, bool fast_entry)
{
	int estimate_uv;

	/*
	 * The vendor SVOOC handler does not echo the noisy SC8547A VBAT ADC.
	 * It reports the cell-side voltage inferred from the live 2:1 bus and
	 * input-current drop.  Feeding the raw local ADC back to the adapter can
	 * move the requested VBUS by several hundred millivolts between frames;
	 * that eventually trips the pump's VBUS_ERRORLO protection.
	 */
	if (fast_entry && vbus_uv >= SC8547_VOOC_CP_MIN_VBUS_UV &&
	    vbus_uv <= SC8547_VOOC_MAX_VBUS_UV && ibus_ua >= 0) {
		estimate_uv = vbus_uv / 2 - div_s64((s64)ibus_ua * 75, 1000);
		if (estimate_uv >= 3404000 && estimate_uv <= 4650000)
			return estimate_uv;
	}
	if (measured_vbat_uv > 0)
		return measured_vbat_uv;
	return measured_vbat_uv;
}

/*
 * Reply to one adapter frame.  The state carried here mirrors the vendor
 * interrupt state machine, but the caller still bounds the whole exchange and
 * keeps the charge pump disabled.  Data is expressed in the pre-reversal
 * physical positions (bits 3..9), as in the vendor tx buffer.
 */
static int sc8547_vooc_send_safe_reply(struct sc8547_device *sc, u8 rx,
					       int vbus_uv, int cached_vbat_uv,
					       int cached_ibus_ua,
					       bool fast_entry,
					       u8 *tx_h, u8 *tx_l,
					       int *reply_vbat_uv,
					       bool *reply_vbus_window_ok,
					       bool *adapter_bad)
{
	u8 command = (rx & 0x1e) >> 1;
	u8 data = 0;
	int vbat_uv;
	int code;
	int ret;

	if (reply_vbat_uv)
		*reply_vbat_uv = -ENODATA;
	if (reply_vbus_window_ok)
		*reply_vbus_window_ok = false;
	if (adapter_bad)
		*adapter_bad = false;

	sc8547_vooc_select_tx_head(sc, rx);
	if (sc->vooc_commu_remaining)
		command = SC8547_VOOC_CMD_ADAPTER_CHECK;
	else if (sc->vooc_model_remaining)
		command = SC8547_VOOC_CMD_TELL_MODEL_PROC;
	else if (sc->vooc_batt_remaining)
		command = SC8547_VOOC_CMD_BAT_MODEL_PROC;

	switch (command) {
	case SC8547_VOOC_CMD_ASK_FAST:
		/* The vendor leaves the first offer bit clear.  It only advertises
		 * fast-charge acceptance after the identification/model exchange and
		 * the final battery-system curve response have completed. */
		data = SC8547_VOOC_SAFE_R_BITS |
			(sc->vooc_batt_model_done ? 1 : 0);
		break;
	case SC8547_VOOC_CMD_IDENTIFICATION:
		sc8547_vooc_identification(sc, &data);
		break;
	case SC8547_VOOC_CMD_ADAPTER_CHECK:
		if (sc->vooc_commu_remaining == 5) {
			data = sc->vooc_ident_next_l |
				(sc->vooc_ident_next_h << 5);
		} else {
			/* The vendor sends TX0_DET_BIT5 for the final four nibbles. */
			data = BIT(2);
			if (sc->vooc_commu_remaining == 4)
				sc->vooc_ident_far = ((u16)((rx & 0x1e) >> 1) << 12);
			else if (sc->vooc_commu_remaining == 3)
				sc->vooc_ident_far |= ((u16)((rx & 0x1e) >> 1) << 8);
			else if (sc->vooc_commu_remaining == 2)
				sc->vooc_ident_far |= ((u16)((rx & 0x1e) >> 1) << 4);
			else if (sc->vooc_commu_remaining == 1) {
				sc->vooc_ident_far |= (rx & 0x1e) >> 1;
				sc->vooc_ident_far &= 0xfefe;
				sc->vooc_adapter_check_ok =
					sc->vooc_ident_far == sc->vooc_ident_local;
			}
		}
		if (sc->vooc_commu_remaining)
			sc->vooc_commu_remaining--;
		break;
	case SC8547_VOOC_CMD_TELL_MODEL:
		/* Set TX0 bits 4 and 6, then receive two model nibbles. */
		data = BIT(1) | BIT(3);
		sc->vooc_model_remaining = 2;
		break;
	case SC8547_VOOC_CMD_TELL_MODEL_PROC:
		data = BIT(2); /* TX0_DET_BIT5 */
		if (sc->vooc_model_remaining == 2)
			sc->vooc_adapter_model = ((rx & 0x1e) >> 1) << 4;
		else if (sc->vooc_model_remaining == 1) {
			sc->vooc_adapter_model |= (rx & 0x1e) >> 1;
			sc->vooc_adapter_model &= 0x7f;
			sc->vooc_model_done = true;
		}
		if (sc->vooc_model_remaining)
			sc->vooc_model_remaining--;
		break;
	case SC8547_VOOC_CMD_ASK_BAT_MODEL:
		/* BATT_SYS_VOL_CUR_PAIR (0x0c) in TX bits 3..6. */
		data = 0x0c;
		sc->vooc_batt_remaining = SC8547_VOOC_BAT_SYS_COUNT;
		break;
	case SC8547_VOOC_CMD_BAT_MODEL_PROC:
		if (sc->vooc_batt_remaining)
			data = sc8547_vooc_batt_model_data(sc->vooc_batt_remaining - 1);
		if (sc->vooc_batt_remaining)
			sc->vooc_batt_remaining--;
		if (!sc->vooc_batt_remaining) {
			sc->vooc_batt_model_done = true;
		}
		break;
	case SC8547_VOOC_CMD_ASK_POWER_BANK:
		/* Match oplus_vooc_adapter_work_as_power_bank(): set TX0 bit 3. */
		data = BIT(0);
		break;
	case SC8547_VOOC_CMD_TELL_USB_BAD:
		/* The adapter is explicitly ending the session.  The wire reply is
		 * still the selected head with zero data, but the AP must stop using
		 * the pump and let the outer retry path perform a clean re-arm. */
		if (adapter_bad)
			*adapter_bad = true;
		dev_warn(sc->dev,
			 "VOOC adapter reported USB bad connection (rx=0x%02x)\n",
			 rx);
		break;
	case SC8547_VOOC_CMD_ASK_AP_STATUS:
		/* Vendor handler has no payload; keep the selected head. */
		break;
	case SC8547_VOOC_CMD_IS_VBUS_OK:
		vbat_uv = cached_vbat_uv;
		if (vbat_uv < 0)
			return vbat_uv;
		if (reply_vbat_uv)
			*reply_vbat_uv = vbat_uv;
		/* Match oplus_voocphy_vbus_vbatt_detect(): the adapter raises
		 * VBUS below +100 mV headroom and lowers it above +200 mV. */
		if (vbus_uv - vbat_uv * 2 > SC8547_VOOC_VBUS_HIGH_DELTA_UV)
			data |= BIT(1); /* TX0_DET_BIT4 */
		else if (vbus_uv - vbat_uv * 2 < SC8547_VOOC_VBUS_LOW_DELTA_UV)
			data |= BIT(2); /* TX0_DET_BIT5 */
		else
			data |= BIT(1) | BIT(2);
		if (reply_vbus_window_ok &&
		    vbus_uv - vbat_uv * 2 >= SC8547_VOOC_CP_START_LOW_DELTA_UV &&
		    vbus_uv - vbat_uv * 2 <= SC8547_VOOC_VBUS_HIGH_DELTA_UV)
			*reply_vbus_window_ok = true;
		break;
	case SC8547_VOOC_CMD_ASK_CURRENT:
		data = sc8547_vooc_pack_value(sc->vooc_active_current_code ?
						      sc->vooc_active_current_code :
						      SC8547_VOOC_DEFAULT_CURRENT_CODE);
		break;
	case SC8547_VOOC_CMD_GET_BATT_VOL:
		vbat_uv = sc8547_vooc_report_bat_uv(vbus_uv, cached_vbat_uv,
						    cached_ibus_ua, fast_entry);
		if (vbat_uv < 0)
			return vbat_uv;
		/* Vendor units are 10 mV above 3404 mV. */
		code = clamp((vbat_uv / 1000 - 3404) / 10, 0, 125);
		data = sc8547_vooc_pack_value(code);
		break;
	default:
		/* Unknown/null frames receive the selected head with zero data. */
		break;
	}

	ret = sc8547_vooc_send_data(sc, data, tx_h, tx_l);
	if (ret)
		return ret;
	dev_dbg(sc->dev,
		"VOOC reply cmd=0x%02x data=0x%02x head=0x%02x check=%u model=0x%02x commu=%u model_left=%u batt_left=%u\n",
		command, data, sc->vooc_tx_head, sc->vooc_adapter_check_ok,
		sc->vooc_adapter_model, sc->vooc_commu_remaining,
		sc->vooc_model_remaining, sc->vooc_batt_remaining);
	return 0;
}

static int sc8547_vooc_session(struct sc8547_device *sc)
{
	u8 saved_loose, saved_timing;
	u8 rx = 0, flags = 0;
	u8 tx_h = 0, tx_l = 0;
	unsigned int reg;
	u8 saved_reg07 = 0, saved_reg05 = 0, saved_reg08 = 0, saved_reg09 = 0,
	    saved_reg0c = 0, saved_reg0d = 0, saved_reg10 = 0, saved_reg11 = 0,
	    saved_reg3b = 0;
	s64 elapsed_us = 0;
	s64 timeout_us = SC8547_VOOC_HANDSHAKE_TIMEOUT_MS * 1000LL;
	s64 next_adc_sample_us = 0;
	s64 next_battery_policy_us = 0;
	s64 next_secondary_sample_us = 0;
	s64 next_secondary_enable_us = 0;
	s64 secondary_enable_deadline_us = 0;
	ktime_t start;
	u8 rx_count = 0;
	u8 vbus_ok_count = 0;
	int reply_vbat_uv = -ENODATA;
	int vooc_bat_uv = -ENODATA;
	bool fast_entry_sent = false;
	bool vooc_cp_enabled = false;
	bool vooc_cp_status_confirmed = false;
	bool secondary_enable_attempted = false;
	bool vooc_current_request_seen = false;
	bool vooc_first_current_tx_failed = false;
	bool vooc_tx_fail_seen = false;
	bool vooc_cp_gate_pending = false;
	bool vooc_session_ending = false;
	bool continuous_session;
	bool reply_vbus_window_ok = false;
	bool adapter_bad = false;
	bool skip_reply = false;
	s64 vooc_cp_gate_due_us = 0;
	s64 vooc_session_end_deadline_us = 0;
	s64 vooc_cp_start_us = 0;
	unsigned int vooc_cp_reg0c = 0;
	int vbus_uv = -ENODATA;
	int cleanup_vbus_uv = -ENODATA;
	int ibus_ua = -ENODATA;
	int secondary_total_ibus_ua = -ENODATA;
	int switch_ret;
	struct power_supply *usb_psy = NULL;
	int saved_input_current_limit = -1;
	bool charger_suspended = false;
	bool dpdm_switched = false;
	struct sc8547_vooc_secondary_state secondary_state = { };
	int ret, cleanup_ret, restore_ret, reg07_restore_ret;
	int secondary_restore_ret, secondary_disable_ret;

	if (!sc->vooc_capable)
		return -EPERM;
	if (strcmp(sc->role, "primary") ||
	    sc->variant != SC8547_VARIANT_SC8547A)
		return -EOPNOTSUPP;
	continuous_session = READ_ONCE(sc->vooc_cable_online);
	/* Reset session state before arming the PHY. */
	sc->vooc_last_cp_started = false;

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
	if (ret)
		return ret;
	saved_reg07 = reg;
	if (reg & SC8547_CHG_EN)
		return -EBUSY;

	ret = regmap_read(sc->regmap, SC8547_REG_LOOSE_DET, &reg);
	if (ret)
		return ret;
	saved_loose = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_TIMING, &reg);
	if (ret)
		return ret;
	saved_timing = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg);
	if (ret)
		return ret;
	saved_reg09 = reg;
	ret = regmap_read(sc->regmap, SC8547A_REG_OTG_CTRL, &reg);
	if (ret)
		return ret;
	saved_reg3b = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_INT_MASK, &reg);
	if (ret)
		return ret;
	saved_reg10 = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &reg);
	if (ret)
		return ret;
	saved_reg05 = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_CTRL, &reg);
	if (ret)
		return ret;
	saved_reg0c = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &reg);
	if (ret)
		return ret;
	saved_reg08 = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_PMID2OUT, &reg);
	if (ret)
		return ret;
	saved_reg0d = reg;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg);
	if (ret)
		return ret;
	saved_reg11 = reg;
	ret = sc8547_update_vooc_current_policy(sc);
	if (ret) {
		dev_info(sc->dev,
			 "VOOC fast session refused by battery policy: %d full=%u capacity=%d vbat=%d temp=%d\n",
			 ret, sc->vooc_policy_full, sc->vooc_policy_capacity,
			 sc->vooc_policy_vbat_uv, sc->vooc_policy_temp_decic);
		return ret;
	}

	/* The vendor init_vooc entry always resets the SC8547A PHY first.  This
	 * clears a latched ERR_TRANS/TX state left by a previous session while the
	 * charge pump remains disabled. */
	ret = regmap_update_bits(sc->regmap, SC8547_REG_CHG_CTRL,
				 SC8547_REG_RESET, SC8547_REG_RESET);
	if (ret)
		return ret;
	msleep(SC8547_RESET_SETTLE_MS);
	sc->vooc_last_total_ibus_ua = -ENODATA;

	ret = sc8547_vooc_secondary_prepare(sc, &secondary_state);
	if (ret)
		goto out_cleanup;

	sc->vooc_last_vbus_uv = -ENODATA;
	sc->vooc_last_ibus_ua = -ENODATA;
	sc->vooc_tx_head = SC8547_VOOC_INVERT_SVOOC;
	/* The vendor starts with the SVOOC head and only changes after
	 * repeated VOOC2-head frames during adapter detection. */
	sc->vooc_adapter_model_detect_remaining = 3;
	sc->vooc_commu_remaining = 0;
	sc->vooc_model_remaining = 0;
	sc->vooc_batt_remaining = 0;
	sc->vooc_ident_next_l = 0;
	sc->vooc_ident_next_h = 0;
	sc->vooc_ident_local = 0;
	sc->vooc_ident_far = 0;
	sc->vooc_adapter_model = 0;
	sc->vooc_adapter_check_ok = false;
	sc->vooc_model_done = false;
	sc->vooc_batt_model_done = false;
	sc->vooc_initialized = true;

	ret = wcd939x_usbss_set_vooc_dpdm(true);
	if (ret)
		goto out_cleanup;
	{
		u8 bat_ovp = sc->profile.reg00;
		u8 bat_ocp = sc->profile.has_reg01 ? sc->profile.reg01 :
			SC8547A_VENDOR_BAT_OCP;

		/* The vendor init_device() always restores the 4.65-V/0xbf battery
		 * protection profile before VOOC.  A previously loaded DT/module may
		 * leave a lower REG00 value (for example 0x22), which can latch
		 * VBAT_OVP during a fast-charge voltage step.  Keep a stale low profile
		 * from turning a protocol session into a false CP/VBUS failure. */
		if ((bat_ovp & GENMASK(4, 0)) <
		    (SC8547A_VENDOR_BAT_OVP & GENMASK(4, 0))) {
			dev_warn(sc->dev,
				 "VOOC replacing low battery OVP profile 0x%02x with official 0x%02x\n",
				 bat_ovp, SC8547A_VENDOR_BAT_OVP);
			bat_ovp = SC8547A_VENDOR_BAT_OVP;
		}
		dev_info(sc->dev,
			 "VOOC primary protection profile reg00=0x%02x reg01=0x%02x\n",
			 bat_ovp, bat_ocp);
		ret = regmap_write(sc->regmap, SC8547_REG_BAT_OVP, bat_ovp);
		if (ret)
			goto out_cleanup;
		ret = regmap_write(sc->regmap, SC8547_REG_BAT_OCP, bat_ocp);
		if (ret)
			goto out_cleanup;
		/* Match sc8547_svooc_hw_setting() while REG07 stays off. */
		ret = regmap_write(sc->regmap, SC8547_REG_AC_OVP,
				   sc->profile.reg02);
		if (ret)
			goto out_cleanup;
		ret = regmap_write(sc->regmap, SC8547_REG_VBUS_OVP,
				   sc->profile.reg04);
		if (ret)
			goto out_cleanup;
		ret = regmap_write(sc->regmap, SC8547_REG_IBUS_PROT,
				   sc->profile.reg05);
		if (ret)
			goto out_cleanup;
		/*
		 * The vendor init_vooc path only selects 2:1/UCP-rise here.  The
		 * 1-second watchdog and PMID-to-OUT SVOOC range are applied after
		 * the adapter accepts the first fast-charge reply below.
		 */
		ret = regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
					 SC8547_IBUS_UCP_RISE_MASK,
					 SC8547_IBUS_UCP_RISE_MASK);
		if (ret)
			goto out_cleanup;
		ret = regmap_write(sc->regmap, SC8547_REG_ADC_CTRL,
				   SC8547_ADC_EN);
		if (ret)
			goto out_cleanup;
		/* Keep the idle PMID-to-OUT range until fast entry is accepted. */
		ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT, saved_reg0d);
		if (ret)
			goto out_cleanup;
	}
	dpdm_switched = true;

	ret = sc8547_vooc_write_word(sc, SC8547_REG_VOOC_PRE_DATA_H, 0);
	if (ret)
		goto out_cleanup;
	/* Match the vendor init_vooc profile: 0x60 selects the 800-us
	 * bit-0 hold timing used by SC8547A VOOC. */
	ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_TIMING,
			   SC8547_VOOC_TIMING_ACTIVE);
	if (ret)
		goto out_cleanup;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOC_DPDM_CTRL,
				  SC8547_VOOC_DPDM_ACTIVE);
	if (ret)
		goto out_cleanup;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOC_DPDM_CFG, 0);
	if (ret)
		goto out_cleanup;
	ret = regmap_write(sc->regmap, SC8547_REG_LOOSE_DET,
				  SC8547_VOOC_LOOSE_DET);
	if (ret)
		goto out_cleanup;
	/* Match the vendor SC8547A init path before arming the VOOC PHY. */
	ret = regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
				 SC8547_IBUS_UCP_RISE_MASK,
				 SC8547_IBUS_UCP_RISE_MASK);
	if (ret)
		goto out_cleanup;
	ret = regmap_write(sc->regmap, SC8547A_REG_OTG_CTRL, 0x04);
	if (ret)
		goto out_cleanup;
	ret = regmap_write(sc->regmap, SC8547_REG_INT_MASK,
				  SC8547A_VENDOR_INSERT_IRQ_MASK);
	if (ret)
		goto out_cleanup;
	ret = sc8547_vooc_write_word(sc, SC8547_REG_VOOC_TX_DATA_H, 0);
	if (ret)
		goto out_cleanup;
	ret = regmap_write(sc->regmap, SC8547_REG_VOOC_INT_MASK,
				  SC8547_VOOC_INT_ACTIVE);
	if (ret)
		goto out_cleanup;

	/*
	 * Establish the safety baseline before starting the time-sensitive wire
	 * exchange.  The vendor IRQ path treats 1.5 ms as a late reply, so the
	 * protocol status must be polled before the slower ADC transactions.
	 */
	vbus_uv = sc8547_get_vbus_uv(sc);
	if (vbus_uv < 0) {
		ret = vbus_uv;
		goto out_cleanup;
	}
	sc->vooc_last_vbus_uv = vbus_uv;
	if (vbus_uv > SC8547_VOOC_MAX_VBUS_UV) {
		dev_warn(sc->dev,
			 "VOOC VBUS safety ceiling exceeded before handshake: %d uV (limit %d)\n",
			 vbus_uv, SC8547_VOOC_MAX_VBUS_UV);
		ret = -EOVERFLOW;
		goto out_cleanup;
	}
	ibus_ua = sc8547_get_ibus_ua(sc);
	if (ibus_ua >= 0)
		sc->vooc_last_ibus_ua = ibus_ua;
	vooc_bat_uv = sc8547_get_vooc_bat_uv(sc);
	if (vooc_bat_uv < 0) {
		ret = vooc_bat_uv;
		goto out_cleanup;
	}

	ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_CTRL,
				  SC8547_VOOC_CTRL_HANDSHAKE);
	if (ret)
		goto out_cleanup;

	ret = -ETIMEDOUT;
	start = ktime_get();
	while (elapsed_us < timeout_us) {
		if (atomic_read(&sc->policy_cancel)) {
			ret = -ECANCELED;
			break;
		}
		elapsed_us = ktime_us_delta(ktime_get(), start);
		if (!vooc_session_ending &&
		    elapsed_us >= next_battery_policy_us) {
			ret = sc8547_update_vooc_current_policy(sc);
			if (ret)
				break;
			next_battery_policy_us = elapsed_us +
				SC8547_VOOC_BATTERY_SAMPLE_US;
		}
		ret = regmap_read(sc->regmap, SC8547_REG_VOOC_INT_STATUS,
					&reg);
		if (ret) {
			if (ret == -EOVERFLOW)
				dev_warn(sc->dev,
					 "VOOC INT_STATUS read returned -EOVERFLOW at rx_count=%u\n",
					 rx_count);
			break;
		}
		flags = reg;
		/* RXDATA_DONE remains usable with a simultaneous TX error. */
		if (flags & SC8547_VOOC_RXDATA_DONE) {
			ret = regmap_read(sc->regmap, SC8547_REG_VOOC_RX_DATA,
						&reg);
			if (ret) {
				if (ret == -EOVERFLOW)
					dev_warn(sc->dev,
						 "VOOC RX_DATA read returned -EOVERFLOW at rx_count=%u\n",
						 rx_count);
				break;
			}
			rx = reg;
			if (rx_count < U8_MAX)
				rx_count++;
			/* The stock voocphy treats a write failure on the first
			 * post-model ASK_CURRENT frame as an adapter-state failure.  It
			 * consumes the combined 0x0f interrupt without overwriting TXDATA,
			 * then reports the failure on the following GET_BATT_VOL frame so
			 * the outer state machine can perform a clean handshake restart. */
			if (fast_entry_sent && sc->vooc_batt_model_done &&
			    !sc->vooc_commu_remaining &&
			    !sc->vooc_model_remaining &&
			    !sc->vooc_batt_remaining &&
			    ((rx & 0x1e) >> 1) == SC8547_VOOC_CMD_ASK_CURRENT &&
			    !vooc_current_request_seen) {
				vooc_current_request_seen = true;
				if (flags & SC8547_VOOC_TXDATA_WR_FAIL) {
					vooc_first_current_tx_failed = true;
					dev_warn(sc->dev,
						 "VOOC first current request TX failed (flags=0x%02x); deferring handshake restart\n",
						 flags);
				}
			}
			/* Match the vendor IRQ state machine: a complete 0x0f status
			 * consumes the received byte but does not overwrite TXDATA again
			 * after the battery-model exchange.  Rewriting during this combined
			 * TX-done/TX-fail state can race the PHY and make the adapter report
			 * USB bad after a long otherwise-valid session. */
			skip_reply = flags == (SC8547_VOOC_RXDATA_DONE |
					      SC8547_VOOC_RX_START |
					      SC8547_VOOC_TXDATA_WR_FAIL |
					      SC8547_VOOC_TXDATA_DONE);
			if (skip_reply && !sc->vooc_batt_model_done) {
				dev_warn(sc->dev,
					 "VOOC 0x0f before battery model completion; aborting\n");
				ret = -EAGAIN;
				break;
			}
			/* oplus_voocphy reports the abnormal first-current exchange when
			 * the adapter asks for the first battery-voltage report.  Do this
			 * before writing a replacement TX word, matching its IRQ path and
			 * leaving the adapter's failed frame intact for the reset path. */
			if (vooc_first_current_tx_failed &&
			    !sc->vooc_commu_remaining &&
			    !sc->vooc_model_remaining &&
			    !sc->vooc_batt_remaining &&
			    ((rx & 0x1e) >> 1) == SC8547_VOOC_CMD_GET_BATT_VOL) {
				dev_warn(sc->dev,
					 "VOOC first current request failed; restarting at battery-voltage request\n");
				ret = -EIO;
				break;
			}
			/* IS_VBUS_OK is the point where the adapter may have just
			 * changed VBUS.  Sample it synchronously before replying so a
			 * stale 5 V value cannot authorize an unsafe transition. */
			if (!sc->vooc_commu_remaining && !sc->vooc_model_remaining &&
			    !sc->vooc_batt_remaining &&
			    ((rx & 0x1e) >> 1) == SC8547_VOOC_CMD_IS_VBUS_OK) {
				vbus_uv = sc8547_get_vbus_uv(sc);
				if (vbus_uv < 0) {
					ret = vbus_uv;
					break;
				}
				sc->vooc_last_vbus_uv = vbus_uv;
				if (vbus_uv > SC8547_VOOC_MAX_VBUS_UV) {
					dev_warn(sc->dev,
						 "VOOC VBUS safety ceiling exceeded on reply: %d uV (limit %d)\n",
						 vbus_uv, SC8547_VOOC_MAX_VBUS_UV);
					ret = -EOVERFLOW;
					break;
				}
				if (fast_entry_sent) {
					if (vbus_ok_count >= SC8547_VOOC_MAX_VBUS_ADJUST) {
						dev_warn(sc->dev,
							 "VOOC VBUS adjustment exceeded %u rounds at %d uV\n",
							 SC8547_VOOC_MAX_VBUS_ADJUST, vbus_uv);
						ret = -ETIMEDOUT;
						break;
					}
					vbus_ok_count++;
				}
			}
			adapter_bad = false;
			if (!skip_reply) {
				reply_vbat_uv = -ENODATA;
				reply_vbus_window_ok = false;
				ret = sc8547_vooc_send_safe_reply(sc, rx, vbus_uv,
								  vooc_bat_uv, ibus_ua,
								  fast_entry_sent,
								  &tx_h, &tx_l,
								  &reply_vbat_uv,
								  &reply_vbus_window_ok,
								  &adapter_bad);
				if (ret) {
					if (ret == -EOVERFLOW)
						dev_warn(sc->dev,
							 "VOOC reply transaction returned -EOVERFLOW at rx_count=%u rx=0x%02x\n",
							 rx_count, rx);
					break;
				}
				if (adapter_bad) {
					ret = -EIO;
					break;
				}
			}
			/* The vendor AP enters the fast-charge stage as soon as the final
			 * battery-system row is sent.  Some adapters ask ASK_CURRENT
			 * directly after that row instead of repeating ASK_FAST; accepting
			 * only the latter leaves the PHY at 5 V and makes its next frame
			 * look like a TXDATA_WR_FAIL.  Keep REG07 disabled here; the
			 * adapter must still prove its voltage step in IS_VBUS_OK before
			 * either charge pump is enabled. */
			if (!fast_entry_sent &&
			    ((sc->vooc_batt_model_done &&
			      (((rx & 0x1e) >> 1) == SC8547_VOOC_CMD_ASK_FAST ||
			       ((rx & 0x1e) >> 1) == SC8547_VOOC_CMD_ASK_CURRENT)) ||
			     (sc->vooc_model_done && !sc->vooc_batt_remaining))) {
				fast_entry_sent = true;
				/* The vendor path votes WIRED_CHARGE_SUSPEND here.
				 * Mainline has no Oplus votable, so use the equivalent
				 * USB power-supply input limit and restore it on every
				 * exit path.  This does not touch the charge pump enable. */
				usb_psy = power_supply_get_by_name("qcom-battmgr-usb");
				if (usb_psy) {
					union power_supply_propval pval = { };

					ret = power_supply_get_property(usb_psy,
							POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
							&pval);
					if (!ret) {
						saved_input_current_limit = pval.intval;
						pval.intval = 0;
						ret = power_supply_set_property(usb_psy,
								POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
								&pval);
						if (!ret)
							charger_suspended = true;
						else
							dev_warn(sc->dev,
								 "USB input suspend rejected: %d\n", ret);
					}
					if (ret && saved_input_current_limit < 0)
						dev_warn(sc->dev,
							 "USB input limit read failed: %d\n", ret);
				} else {
					dev_warn(sc->dev,
						 "qcom-battmgr-usb power supply unavailable\n");
				}
				ret = sc8547_vooc_write_word(sc,
						SC8547_REG_VOOC_PRE_DATA_H,
						SC8547_VOOC_PRE_SVOOC);
				if (ret)
					break;
				/* Match sc8547_svooc_hw_setting(false): select 2:1 and
				 * UCP-rise while preserving the watchdog field. The vendor
				 * PPS path then explicitly selects its five-second watchdog
				 * before CP_SET_WORK_START, yielding REG09=0x14 here. */
				ret = regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
							 SC8547_CHARGE_MODE |
							 SC8547_IBUS_UCP_RISE_MASK,
							 SC8547_IBUS_UCP_RISE_MASK);
				if (ret)
					break;
				ret = sc8547_set_watchdog_code(sc,
							SC8547A_VENDOR_PPS_WATCHDOG);
				if (ret)
					break;
				ret = regmap_update_bits(sc->regmap, SC8547_REG_IBUS_PROT,
								 SC8547_IBUS_UCP_DIS,
								 SC8547_IBUS_UCP_DIS);
				if (ret)
					break;
				ret = regmap_update_bits(sc->regmap, SC8547_REG_SS_CTRL,
								 SC8547_SS_TIMEOUT_MASK, 0);
				if (ret)
					break;
				ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT, 0x70);
				if (ret)
					break;
				dev_info(sc->dev, "VOOC fast entry accepted\n");
			}
			/* The adapter may still be completing its VBUS step when the
			 * IS_VBUS_OK reply arrives.  Defer the slow ADC/CP gate work so
			 * this protocol iteration can return its reply immediately. */
			if (fast_entry_sent && !vooc_cp_enabled &&
			    ((rx & 0x1e) >> 1) == SC8547_VOOC_CMD_IS_VBUS_OK) {
				/* A low reply may have queued the 500-ms fallback, but a
				 * later normal reply is authoritative and must shorten that
				 * deadline to the official immediate CP-start path. */
				if (reply_vbus_window_ok) {
					vooc_cp_gate_pending = true;
					vooc_cp_gate_due_us =
						ktime_us_delta(ktime_get(), start) + 10000LL;
					dev_info(sc->dev,
						 "VOOC CP gate ready after normal VBUS reply\n");
				} else if (!vooc_cp_gate_pending) {
					vooc_cp_gate_pending = true;
					vooc_cp_gate_due_us =
						ktime_us_delta(ktime_get(), start) +
						SC8547_VOOC_CP_START_SETTLE_MS * 1000LL;
					dev_info(sc->dev,
						 "VOOC CP gate deferred %u ms for VBUS regulation\n",
						 SC8547_VOOC_CP_START_SETTLE_MS);
				}
			}
			if (vooc_cp_enabled) {
				/* RXDATA_DONE can arrive back-to-back without entering the
				 * idle polling branch. Keep the VBUS ceiling active on the
				 * protocol path as well, but sample at the normal ADC cadence. */
				elapsed_us = ktime_us_delta(ktime_get(), start);
				if (elapsed_us >= next_adc_sample_us) {
					vbus_uv = sc8547_get_vbus_uv(sc);
					if (vbus_uv < 0) {
						ret = vbus_uv;
						break;
					}
					sc->vooc_last_vbus_uv = vbus_uv;
					if (vbus_uv < SC8547_VOOC_CP_MIN_VBUS_UV ||
					    vbus_uv > SC8547_VOOC_MAX_VBUS_UV) {
						if (vbus_uv > SC8547_VOOC_MAX_VBUS_UV)
							dev_warn(sc->dev,
								 "VOOC VBUS safety ceiling exceeded during CP monitor: %d uV (limit %d)\n",
								 vbus_uv, SC8547_VOOC_MAX_VBUS_UV);
						ret = vbus_uv > SC8547_VOOC_MAX_VBUS_UV ?
							-EOVERFLOW : -ERANGE;
						break;
					}
						next_adc_sample_us = elapsed_us +
							SC8547_VOOC_ADC_SAMPLE_US;
					}
				}
				if (secondary_state.enabled &&
				    elapsed_us >= next_secondary_sample_us) {
					ret = sc8547_vooc_secondary_monitor(
						&secondary_state, ibus_ua,
						&secondary_total_ibus_ua);
					if (ret) {
						dev_warn(sc->dev,
							 "VOOC secondary CP monitor failed: %d\n", ret);
						break;
					}
					sc->vooc_last_total_ibus_ua =
						secondary_total_ibus_ua;
					next_secondary_sample_us = elapsed_us +
						SC8547_VOOC_SECONDARY_SAMPLE_MS * 1000LL;
				}
				/* A continuous SVOOC session exchanges this keepalive frame
				 * roughly every 45 ms.  Keep it available for dynamic debug,
				 * but do not evict policy transitions and faults from dmesg. */
				dev_dbg_ratelimited(sc->dev,
					"VOOC safe round=%u rx=0x%02x cmd=0x%02x flags=0x%02x tx=0x%02x%02x\n",
					rx_count, rx, (rx & 0x1e) >> 1,
					flags, tx_h, tx_l);
			ret = 0;
			elapsed_us = ktime_us_delta(ktime_get(), start);
			if (!vooc_cp_enabled &&
			    rx_count >= SC8547_VOOC_SESSION_ROUNDS)
				break;
			if (!continuous_session && vooc_cp_enabled &&
			    elapsed_us - vooc_cp_start_us >=
				(s64)SC8547_VOOC_SESSION_MS * 1000LL) {
				/* Keep both pumps loaded while the adapter observes a
				 * low-current termination window.  Abruptly disabling the
				 * CP at the session deadline leaves some adapters unloaded
				 * at their 11-V default and causes a short over-voltage step. */
				if (!vooc_session_ending) {
					vooc_session_ending = true;
					vooc_session_end_deadline_us = elapsed_us +
						SC8547_VOOC_SESSION_END_GRACE_MS * 1000LL;
					sc->vooc_active_current_code =
						SC8547_VOOC_MIN_CURRENT_CODE;
					dev_info(sc->dev,
						 "VOOC entering %u-ms low-current termination window\n",
						 SC8547_VOOC_SESSION_END_GRACE_MS);
				} else if (elapsed_us >= vooc_session_end_deadline_us) {
					ret = 0;
					break;
				}
			}
			usleep_range(SC8547_VOOC_POLL_MIN_US,
				     SC8547_VOOC_POLL_MAX_US);
			elapsed_us = ktime_us_delta(ktime_get(), start);
			continue;
		}
		if (flags & (SC8547_VOOC_ERR_TRANS |
				     SC8547_VOOC_TXDATA_WR_FAIL |
				     SC8547_VOOC_NINTH_CLK_ERR)) {
			/* The vendor IRQ path can report a transient TX write failure
			 * just after fast entry, while the adapter is stepping VBUS and
			 * before the primary CP gate has run.  Keep the bounded exchange
			 * alive in that state (and after CP start) so the next VBUS-OK
			 * frame can complete the state transition; any protocol
			 * transition or ninth-clock error still aborts immediately. */
			if ((vooc_cp_enabled || fast_entry_sent ||
			     (sc->vooc_model_done && sc->vooc_batt_remaining <= 1)) &&
			    (flags & SC8547_VOOC_TXDATA_WR_FAIL) &&
			    !(flags & (SC8547_VOOC_ERR_TRANS |
				       SC8547_VOOC_NINTH_CLK_ERR))) {
				if (!vooc_tx_fail_seen) {
					dev_warn(sc->dev,
						 "VOOC TXDATA_WR_FAIL during fast entry; continuing bounded monitor\n");
					vooc_tx_fail_seen = true;
				}
				/* Continue with the normal ADC and CP status checks below. */
				ret = 0;
			} else {
				ret = -EIO;
				break;
			}
		}

		elapsed_us = ktime_us_delta(ktime_get(), start);
		if (elapsed_us >= next_adc_sample_us) {
			vbus_uv = sc8547_get_vbus_uv(sc);
			if (vbus_uv < 0) {
				ret = vbus_uv;
				break;
			}
			sc->vooc_last_vbus_uv = vbus_uv;
			if (vbus_uv > SC8547_VOOC_MAX_VBUS_UV) {
				dev_warn(sc->dev,
					 "VOOC VBUS safety ceiling exceeded while polling: %d uV (limit %d)\n",
					 vbus_uv, SC8547_VOOC_MAX_VBUS_UV);
				ret = -EOVERFLOW;
				break;
			}

			ibus_ua = sc8547_get_ibus_ua(sc);
				if (ibus_ua >= 0)
					sc->vooc_last_ibus_ua = ibus_ua;
				{
					int sampled_bat_uv = sc8547_get_vbat_uv(sc);

					if (sampled_bat_uv >= 0)
						vooc_bat_uv = sampled_bat_uv;
				}
				if (fast_entry_sent && !vooc_cp_enabled &&
				    vooc_cp_gate_pending &&
				    elapsed_us >= vooc_cp_gate_due_us) {
					int vbat_now_uv;

					vooc_cp_gate_pending = false;
					vbat_now_uv = vooc_bat_uv;
					if (vbat_now_uv < 0) {
						ret = vbat_now_uv;
						break;
					}
					dev_info(sc->dev,
						 "VOOC CP gate vbus=%d vbat=%d delta=%d\n",
						 vbus_uv, vbat_now_uv,
						 vbus_uv - 2 * vbat_now_uv);
					if (vbus_uv >= SC8547_VOOC_CP_MIN_VBUS_UV &&
					    vbus_uv <= SC8547_VOOC_CP_START_MAX_VBUS_UV &&
					    vbus_uv - 2 * vbat_now_uv >=
						SC8547_VOOC_CP_START_LOW_DELTA_UV &&
					     vbus_uv - 2 * vbat_now_uv <=
						SC8547_VOOC_CP_START_HIGH_DELTA_UV) {
						/* The upstream SC8547A primary path asserts REG07
						 * directly.  REG0C=0x02 is the downstream slave
						 * CP_ENABLE precondition; forcing it on the primary
						 * can leave REG06 non-switching and immediately trip
						 * the fail-closed cleanup. */
						ret = sc8547_set_charge_enabled(sc, true);
						if (ret)
							break;
						/* Keep the pre-session value for a failure cleanup; the
						 * primary register is intentionally left untouched. */
						vooc_cp_reg0c = saved_reg0c;
						vooc_cp_enabled = true;
						sc->vooc_last_cp_started = true;
						vooc_cp_start_us = ktime_us_delta(ktime_get(), start);
						next_secondary_enable_us = vooc_cp_start_us;
						secondary_enable_deadline_us = vooc_cp_start_us +
							SC8547_VOOC_SECONDARY_ENABLE_TIMEOUT_MS * 1000LL;
						/* Once the adapter has accepted fast charge, keep the
						 * same bounded protocol/ADC loop alive for the long
						 * session window.  The outer timeout remains finite so a
						 * stalled source cannot hold the worker indefinitely. */
			if (continuous_session)
				timeout_us = S64_MAX;
			else
				/* Include the low-current termination window in the outer
				 * timeout so bounded sessions can send final reduced-
				 * current frames before CP is disabled. */
				timeout_us = ((s64)SC8547_VOOC_SESSION_MS +
						  SC8547_VOOC_SESSION_END_GRACE_MS +
						  SC8547_VOOC_CP_START_GRACE_MS) *
								  1000LL;
						dev_info(sc->dev,
							 "VOOC primary CP work-start asserted at vbus=%d vbat=%d; "
							 "switching check deferred %u ms continuous=%u\n",
							 vbus_uv, vbat_now_uv,
							 SC8547_VOOC_CP_START_SETTLE_MS,
							 continuous_session);
					} else if (vbus_uv >= 0 &&
						   vbus_uv <= SC8547_VOOC_CP_START_MAX_VBUS_UV) {
						vooc_cp_gate_pending = true;
						vooc_cp_gate_due_us = elapsed_us +
							SC8547_VOOC_CP_GATE_RETRY_MS * 1000LL;
						dev_info(sc->dev,
							 "VOOC waiting for VBUS regulation: vbus=%d vbat=%d delta=%d; retry %u ms\n",
							 vbus_uv, vbat_now_uv,
							 vbus_uv - 2 * vbat_now_uv,
							 SC8547_VOOC_CP_GATE_RETRY_MS);
					}
				}
				if (vooc_cp_enabled) {
					unsigned int cp_reg06, cp_reg07 = 0, cp_reg0e;
					int tdie_mc;

					if (vbus_uv < SC8547_VOOC_CP_MIN_VBUS_UV ||
					    vbus_uv > SC8547_VOOC_MAX_VBUS_UV ||
					    (ibus_ua >= 0 &&
					     ibus_ua > SC8547_VOOC_CP_MAX_IBUS_UA)) {
						ret = -ERANGE;
						break;
					}
					tdie_mc = sc8547_get_tdie_mc(sc);
					if (tdie_mc < 0) {
						ret = tdie_mc;
						break;
					}
					if ((u32)tdie_mc > sc->tdie_max_mc) {
						ret = -ERANGE;
						break;
					}
					ret = sc8547_read_status(sc, &cp_reg06, &cp_reg0e);
					if (ret)
						break;
					if (sc8547_has_fatal_fault(cp_reg06, cp_reg0e) ||
					    (!(cp_reg06 & SC8547_CP_SWITCHING_STAT) &&
					     elapsed_us - vooc_cp_start_us >=
						SC8547_VOOC_CP_START_GRACE_MS * 1000LL)) {
						ret = regmap_read(sc->regmap,
								  SC8547_REG_CHG_CTRL,
								  &cp_reg07);
						if (ret)
							break;
						dev_warn(sc->dev,
							 "VOOC primary CP switching check failed "
							 "reg06=0x%02x reg07=0x%02x reg0c=0x%02x reg0e=0x%02x\n",
							 cp_reg06, cp_reg07, vooc_cp_reg0c, cp_reg0e);
						ret = -EIO;
						break;
					}
					if (!vooc_cp_status_confirmed &&
					    elapsed_us - vooc_cp_start_us >=
						SC8547_VOOC_CP_START_GRACE_MS * 1000LL) {
						vooc_cp_status_confirmed = true;
						sc->vooc_auto_retry_count = 0;
						dev_info(sc->dev,
							 "VOOC primary CP switching confirmed "
							 "status=0x%02x after %u ms\n",
							 cp_reg06, SC8547_VOOC_CP_START_GRACE_MS);
					}
					if (vooc_cp_status_confirmed &&
					    secondary_state.valid &&
					    (!secondary_enable_attempted ||
					     secondary_state.reenable_needed) &&
					    elapsed_us >= next_secondary_enable_us) {
						int primary_ibus_now;

						primary_ibus_now = sc8547_get_ibus_ua(sc);
						if (primary_ibus_now < 0) {
							ret = primary_ibus_now;
							dev_warn(sc->dev,
								 "VOOC primary IBUS read failed before secondary start: %d\n",
								 ret);
							break;
						}
						ibus_ua = primary_ibus_now;
						sc->vooc_last_ibus_ua = ibus_ua;
						if (ibus_ua <
						    SC8547_VOOC_SECONDARY_ENABLE_MIN_IBUS_UA) {
							if (elapsed_us < secondary_enable_deadline_us) {
								next_secondary_enable_us = elapsed_us +
									SC8547_VOOC_SECONDARY_ENABLE_RETRY_MS * 1000LL;
							} else {
								secondary_enable_attempted = true;
								next_secondary_enable_us = elapsed_us +
									SC8547_VOOC_SECONDARY_ENABLE_RETRY_MS * 1000LL;
								if (!secondary_state.held_off_logged) {
									secondary_state.held_off_logged = true;
									dev_info(sc->dev,
										 "VOOC secondary held off after %u ms: primary ibus=%d below %u uA\n",
										 SC8547_VOOC_SECONDARY_ENABLE_TIMEOUT_MS,
										 ibus_ua,
										 SC8547_VOOC_SECONDARY_ENABLE_MIN_IBUS_UA);
								}
							}
						} else {
							secondary_enable_attempted = true;
							ret = sc8547_vooc_secondary_enable(
								&secondary_state, ibus_ua,
								&secondary_total_ibus_ua);
							if (ret) {
								dev_warn(sc->dev,
									 "VOOC secondary CP start failed: %d\n",
									 ret);
								break;
							}
							sc->vooc_last_total_ibus_ua =
								secondary_total_ibus_ua;
							next_secondary_sample_us = elapsed_us +
								SC8547_VOOC_SECONDARY_SAMPLE_MS * 1000LL;
						}
					}
					if (secondary_state.enabled &&
					    elapsed_us >= next_secondary_sample_us) {
						ret = sc8547_vooc_secondary_monitor(
							&secondary_state, ibus_ua,
							&secondary_total_ibus_ua);
						if (ret) {
							dev_warn(sc->dev,
								 "VOOC secondary CP monitor failed: %d\n", ret);
							break;
						}
						sc->vooc_last_total_ibus_ua =
							secondary_total_ibus_ua;
						next_secondary_sample_us = elapsed_us +
							SC8547_VOOC_SECONDARY_SAMPLE_MS * 1000LL;
					}
					if (!continuous_session &&
					    elapsed_us - vooc_cp_start_us >=
					    (s64)SC8547_VOOC_SESSION_MS * 1000LL) {
						if (!vooc_session_ending) {
							vooc_session_ending = true;
							vooc_session_end_deadline_us = elapsed_us +
								SC8547_VOOC_SESSION_END_GRACE_MS * 1000LL;
							sc->vooc_active_current_code =
								SC8547_VOOC_MIN_CURRENT_CODE;
							dev_info(sc->dev,
								 "VOOC entering %u-ms low-current termination window\n",
								 SC8547_VOOC_SESSION_END_GRACE_MS);
						} else if (elapsed_us >= vooc_session_end_deadline_us) {
							ret = 0;
							break;
						}
					}
				}
				next_adc_sample_us = elapsed_us +
						SC8547_VOOC_ADC_SAMPLE_US;
		}

		usleep_range(SC8547_VOOC_POLL_MIN_US,
			     SC8547_VOOC_POLL_MAX_US);
		elapsed_us = ktime_us_delta(ktime_get(), start);
	}
	if (!continuous_session && vooc_cp_enabled &&
	    !vooc_session_ending) {
		/* If the adapter stopped sending frames exactly at the deadline,
		 * the outer timeout still provided the bounded load-holding grace
		 * period. Record the same termination state before disabling CP. */
		vooc_session_ending = true;
		sc->vooc_active_current_code = SC8547_VOOC_MIN_CURRENT_CODE;
		dev_info(sc->dev,
			 "VOOC termination grace elapsed without a final protocol frame\n");
	}
	/* Sample once while the CP is still enabled.  This catches a source
	 * overshoot at the end of the grace window before cleanup reports success. */
	if (vooc_cp_enabled && !ret) {
		int final_vbus_uv = sc8547_get_vbus_uv(sc);

		if (final_vbus_uv < 0) {
			ret = final_vbus_uv;
		} else {
			sc->vooc_last_vbus_uv = final_vbus_uv;
			if (final_vbus_uv > SC8547_VOOC_MAX_VBUS_UV) {
				dev_warn(sc->dev,
					 "VOOC VBUS safety ceiling exceeded at termination: %d uV (limit %d)\n",
					 final_vbus_uv, SC8547_VOOC_MAX_VBUS_UV);
				ret = -EOVERFLOW;
			}
		}
	}
	if (!rx_count && ret == 0)
		ret = -ETIMEDOUT;
	/* A fast-charge session is successful only after the primary CP has
	 * actually started.  A source may continue sending protocol frames while
	 * declining the requested voltage; treating that bounded exchange as a
	 * success suppresses the adapter reset/retry path and leaves the user at
	 * ordinary 5 V input. */
	if (ret == 0 && !sc->vooc_last_cp_started) {
		dev_warn(sc->dev,
			 "VOOC fast entry not accepted: rx_count=%u vbus=%d; retrying\n",
			 rx_count, sc->vooc_last_vbus_uv);
		ret = -ETIMEDOUT;
	}

out_cleanup:
	secondary_disable_ret = sc8547_vooc_secondary_disable(&secondary_state);
	if (secondary_disable_ret) {
		if (!ret)
			ret = secondary_disable_ret;
	}
	if (vooc_cp_enabled) {
		int cp_disable_ret;

		cp_disable_ret = sc8547_set_charge_enabled(sc, false);
		if (cp_disable_ret)
			dev_warn(sc->dev,
				 "VOOC CP disable during cleanup failed: %d\n",
				 cp_disable_ret);
		if (cp_disable_ret && !ret)
			ret = cp_disable_ret;
		if (!cp_disable_ret)
			usleep_range(10000, 20000);
		vooc_cp_enabled = false;
	}
	cleanup_ret = sc8547_vooc_cleanup(sc, saved_loose, saved_timing);
	secondary_restore_ret = sc8547_vooc_secondary_restore(&secondary_state);
	if (secondary_restore_ret && !cleanup_ret)
		cleanup_ret = secondary_restore_ret;
	if (dpdm_switched) {
		switch_ret = wcd939x_usbss_set_vooc_dpdm(false);
		if (switch_ret && !cleanup_ret)
			cleanup_ret = switch_ret;
		if (switch_ret)
			dev_warn(sc->dev, "VOOC DPDM route restore failed: %d\n",
				 switch_ret);
	}
	if (cleanup_ret && !ret)
		ret = cleanup_ret;
	if (cleanup_ret)
		dev_warn(sc->dev, "VOOC session cleanup failed: %d\n", cleanup_ret);
	restore_ret = 0;
	/* Reset may change the low control bits; preserve the idle baseline. */
	reg07_restore_ret = regmap_write(sc->regmap, SC8547_REG_CHG_CTRL,
					 saved_reg07);
	if (reg07_restore_ret && !ret)
		ret = reg07_restore_ret;
	if (reg07_restore_ret)
		dev_warn(sc->dev, "VOOC charge control restore failed: %d\n",
			 reg07_restore_ret);
	restore_ret = regmap_write(sc->regmap, SC8547_REG_IBUS_PROT,
				   saved_reg05);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_MODE_CTRL,
					   saved_reg09);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_IBUS_CTRL,
					   saved_reg0c);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_SS_CTRL,
					   saved_reg08);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT,
					   saved_reg0d);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_ADC_CTRL,
					   saved_reg11);
	if (restore_ret && !ret)
		ret = restore_ret;
	if (restore_ret)
		dev_warn(sc->dev, "VOOC fast config restore failed: %d\n",
			 restore_ret);
	/* REG09 and REG3B are also touched for the vendor PHY setup. */
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_MODE_CTRL,
					   saved_reg09);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547A_REG_OTG_CTRL,
					   saved_reg3b);
	if (!restore_ret)
		restore_ret = regmap_write(sc->regmap, SC8547_REG_INT_MASK,
					   saved_reg10);
	if (restore_ret && !ret)
		ret = restore_ret;
	if (restore_ret)
		dev_warn(sc->dev, "VOOC PHY setup restore failed: %d\n", restore_ret);
	if (charger_suspended && usb_psy) {
		union power_supply_propval pval = {
			/* Remove the vendor WIRED_CHARGE_SUSPEND vote. */
			.intval = -1,
		};
		int unsuspend_ret;

		unsuspend_ret = power_supply_set_property(usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
		if (unsuspend_ret && !ret)
			ret = unsuspend_ret;
		if (unsuspend_ret)
			dev_warn(sc->dev,
				 "USB input limit restore failed: %d\n", unsuspend_ret);
	}
	if (usb_psy)
		power_supply_put(usb_psy);

	/* A response may have changed the adapter voltage before the PHY reset.
	 * Keep this safety check, but do not replace the last in-session sample
	 * with the invalid low ADC value returned after ADC/CP cleanup. */
	cleanup_vbus_uv = sc8547_get_vbus_uv(sc);
	if (sc->vooc_last_vbus_uv < 0 && cleanup_vbus_uv >= 0)
		sc->vooc_last_vbus_uv = cleanup_vbus_uv;
	/* Restoring the USB route can leave the adapter's output capacitor and
	 * SC8547 ADC briefly above the fast-charge ceiling while the source steps
	 * back to its idle 5 V contract.  Give that bounded, observable transient
	 * a short decay window; a persistent over-voltage still fails closed. */
	if (cleanup_vbus_uv > SC8547_VOOC_MAX_VBUS_UV) {
		unsigned int waited_ms;

		for (waited_ms = 0;
		     waited_ms < SC8547_VOOC_CLEANUP_VBUS_WAIT_MS;
		     waited_ms += SC8547_VOOC_CLEANUP_VBUS_POLL_MS) {
			msleep(SC8547_VOOC_CLEANUP_VBUS_POLL_MS);
			cleanup_vbus_uv = sc8547_get_vbus_uv(sc);
			if (cleanup_vbus_uv < 0 ||
			    cleanup_vbus_uv <= SC8547_VOOC_MAX_VBUS_UV)
				break;
		}
		if (cleanup_vbus_uv > 0 &&
		    cleanup_vbus_uv <= SC8547_VOOC_MAX_VBUS_UV)
			dev_info(sc->dev,
				 "VOOC cleanup VBUS settled to %d uV after %u ms\n",
				 cleanup_vbus_uv, waited_ms);
	}
	if (cleanup_vbus_uv > SC8547_VOOC_MAX_VBUS_UV && !ret)
		ret = -EOVERFLOW;

	if (ret == -EOVERFLOW)
		dev_warn(sc->dev,
			 "VOOC -EOVERFLOW source unresolved: rx_count=%u session_vbus=%d cleanup_vbus=%d\n",
			 rx_count, sc->vooc_last_vbus_uv,
			 cleanup_vbus_uv);
	dev_info(sc->dev,
		 "VOOC session rc=%d frames=%u vbus_uv=%d ibus_ua=%d\n",
		 ret, rx_count, sc->vooc_last_vbus_uv,
		 sc->vooc_last_ibus_ua);
	return ret;
}

static int sc8547_vooc_usb_online(void)
{
	struct power_supply *psy;
	union power_supply_propval val = { };
	int ret;

	psy = power_supply_get_by_name("qcom-battmgr-usb");
	if (!psy)
		return -ENODEV;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
	power_supply_put(psy);
	if (ret)
		return ret;

	return val.intval ? 1 : 0;
}

/*
 * The vendor WIRED_CHARGE_SUSPEND vote is also used to restart an adapter
 * which is still waiting in its previous VOOC state.  A direct PHY reset is
 * insufficient for that case: the source has to observe a short removal of
 * the normal USB input path before it will emit a new first frame.  The
 * Caihong battmgr exposes the same firmware request through INPUT_CURRENT_LIMIT;
 * zero is the suspend vote and -1 removes it (U32_MAX on the wire).
 */
static int sc8547_vooc_restart_adapter(struct sc8547_device *sc)
{
	struct power_supply *psy;
	union power_supply_propval val = { };
	int saved_limit;
	int suspend_ret;
	int restore_ret;
	int fallback_ret;
	int ret;

	psy = power_supply_get_by_name("qcom-battmgr-usb");
	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret)
		goto out_put;
	if (!val.intval) {
		ret = -ENOTCONN;
		goto out_put;
	}

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (ret)
		goto out_put;
	saved_limit = val.intval;
	if (saved_limit <= 0) {
		/* Do not override another policy's existing suspend vote. */
		ret = -EBUSY;
		goto out_put;
	}

	val.intval = 0;
	ret = power_supply_set_property(psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	suspend_ret = ret;
	if (ret)
		goto out_put;

	msleep(SC8547_VOOC_USB_SUSPEND_MS);

	/* Match the vendor votable removal rather than restoring a stale ICL. */
	val.intval = -1;
	restore_ret = power_supply_set_property(psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (restore_ret) {
		/* Leave the input usable if the firmware rejects U32_MAX. */
		val.intval = saved_limit;
		fallback_ret = power_supply_set_property(psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
		if (!fallback_ret) {
			dev_warn(sc->dev,
				 "VOOC USB restart restore rejected (%d); restored ICL=%d\n",
				restore_ret, saved_limit);
			ret = 0;
		} else {
			dev_warn(sc->dev,
				 "VOOC USB restart restore failed: remove=%d fallback=%d\n",
				restore_ret, fallback_ret);
			ret = restore_ret;
		}
	} else {
		ret = 0;
	}
	/* The vendor reset leaves the source detached briefly after the vote is
	 * removed, allowing its protocol state machine to return to 5 V before the
	 * PHY handshake is armed again. */
	msleep(SC8547_VOOC_USB_SETTLE_MS);

	dev_info(sc->dev,
		 "VOOC adapter restart suspend=%d restore=%d saved_icl=%d\n",
		 suspend_ret, restore_ret, saved_limit);

out_put:
	power_supply_put(psy);
	return ret;
}

/*
 * Keep the retry policy outside the protocol loop.  The vendor charger owns
 * this boundary too: one exchange is bounded, then the AP decides whether to
 * reactivate the PHY and start a new exchange.  This prevents a stalled
 * adapter from holding the I2C worker forever while still allowing retries
 * without a physical unplug.
 */
static void sc8547_vooc_retry_workfn(struct work_struct *work)
{
	struct sc8547_device *sc = container_of(to_delayed_work(work),
						struct sc8547_device, vooc_retry_work);
	int online;
	int ret;
	bool dpdm_normalized = false;
	bool dpdm_preselected = false;
	bool session_started = false;

	if (!READ_ONCE(sc->vooc_auto_retry))
		return;
	if (atomic_cmpxchg(&sc->vooc_retry_running, 0, 1))
		return;

	online = sc8547_vooc_usb_online();
	if (online <= 0) {
		/* USB online briefly drops while battmgr applies/removes the
		 * WIRED_CHARGE_SUSPEND vote.  Do not lose the retry window during
		 * that transition; a genuinely disconnected source is still bounded
		 * by the same three attempts and then disables the lab policy. */
		if (sc->vooc_auto_retry_count + 1 <
			    SC8547_VOOC_AUTO_RETRY_MAX) {
			sc->vooc_auto_retry_count++;
			dev_warn(sc->dev,
				 "VOOC USB temporarily offline; retry %u/%u\n",
				 sc->vooc_auto_retry_count,
				 SC8547_VOOC_AUTO_RETRY_MAX - 1);
			sc8547_queue_vooc_retry(sc,
				msecs_to_jiffies(SC8547_VOOC_AUTO_RETRY_DELAY_MS));
		} else {
			dev_warn(sc->dev,
				 "VOOC automatic session stopped: USB offline\n");
			WRITE_ONCE(sc->vooc_auto_retry, false);
			sc->vooc_auto_retry_count = 0;
		}
		atomic_set(&sc->vooc_retry_running, 0);
		return;
	}

	mutex_lock(&sc->lock);
	if (!sc->vooc_capable ||
	    !sc->vooc_auto_retry || atomic_read(&sc->pps_running)) {
		mutex_unlock(&sc->lock);
		atomic_set(&sc->vooc_retry_running, 0);
		return;
	}
	/* Do not wake the adapter or touch DP/DM when the gauge already reports
	 * full, an unsafe cell temperature, or a missing safety-critical sample. */
	ret = sc8547_update_vooc_current_policy(sc);
	if (ret) {
		WRITE_ONCE(sc->vooc_auto_retry, false);
		sc->vooc_auto_retry_count = 0;
		dev_info(sc->dev,
			 "VOOC automatic session stopped by battery policy rc=%d full=%u\n",
			 ret, sc->vooc_policy_full);
		mutex_unlock(&sc->lock);
		atomic_set(&sc->vooc_retry_running, 0);
		return;
	}

	/* The vendor recovery path first returns the USB switch to normal mode,
	 * then cycles the input suspend vote, waits for the adapter MCU to leave its
	 * previous VOOC state, and only afterwards re-arms the VOOC PHY. */
	ret = wcd939x_usbss_set_vooc_dpdm(false);
	if (!ret) {
		dpdm_normalized = true;
		msleep(SC8547_VOOC_REACTIVE_SETTLE_MS);
		ret = sc8547_vooc_restart_adapter(sc);
	}
	if (!ret) {
		/* The board's USBSS needs the VOOC analog path selected before the
		 * SC8547 PHY reset to generate the adapter's first frame.  This is
		 * equivalent to the vendor switch_fast_chg() edge; the session repeats
		 * the selection after both pumps are prepared. */
		ret = wcd939x_usbss_set_vooc_dpdm(true);
		if (!ret)
			dpdm_preselected = true;
	}
	if (!ret && (!READ_ONCE(sc->vooc_auto_retry) ||
		     atomic_read(&sc->policy_cancel)))
		ret = -ECANCELED;
	if (!ret) {
		session_started = true;
		ret = sc8547_vooc_session(sc);
	}
	if ((dpdm_normalized || dpdm_preselected) && !session_started)
		wcd939x_usbss_set_vooc_dpdm(false);
	if (!ret) {
		sc->vooc_auto_retry_count = 0;
		WRITE_ONCE(sc->vooc_auto_retry, false);
		dev_info(sc->dev, "VOOC automatic session completed\n");
	} else if (ret == -EALREADY || sc->vooc_policy_full) {
		struct power_supply *bat_psy;
		int ffc_ret;

		/* Full is a normal terminal state for unattended charging. */
		WRITE_ONCE(sc->vooc_auto_retry, false);
		sc->vooc_auto_retry_count = 0;
		bat_psy = power_supply_get_by_name("qcom-battmgr-bat");
		if (bat_psy) {
			ffc_ret = qcom_battmgr_caihong_notify_fastchg_complete(
				bat_psy);
			power_supply_put(bat_psy);
			if (ffc_ret && ffc_ret != -EOPNOTSUPP)
				dev_warn(sc->dev,
					 "failed to hand off VOOC completion to FFC: %d\n",
					 ffc_ret);
		}
		dev_info(sc->dev,
			 "VOOC session reached terminal threshold; normal charging resumed\n");
	} else if (sc->vooc_auto_retry_count + 1 <
			   SC8547_VOOC_AUTO_RETRY_MAX &&
			   sc->vooc_last_vbus_uv <= SC8547_VOOC_MAX_VBUS_UV) {
		sc->vooc_auto_retry_count++;
		dev_warn(sc->dev,
			 "VOOC automatic session failed rc=%d; retry %u/%u\n",
			 ret, sc->vooc_auto_retry_count,
			 SC8547_VOOC_AUTO_RETRY_MAX - 1);
		sc8547_queue_vooc_retry(sc,
			msecs_to_jiffies(SC8547_VOOC_AUTO_RETRY_DELAY_MS));
	} else {
		dev_warn(sc->dev,
			 "VOOC automatic session stopped after rc=%d\n", ret);
		WRITE_ONCE(sc->vooc_auto_retry, false);
		sc->vooc_auto_retry_count = 0;
	}
	mutex_unlock(&sc->lock);
	atomic_set(&sc->vooc_retry_running, 0);
}

static void sc8547_vooc_cable_workfn(struct work_struct *work)
{
	struct sc8547_device *sc = container_of(to_delayed_work(work),
						struct sc8547_device, vooc_cable_work);
	bool was_online = READ_ONCE(sc->vooc_cable_online);
	bool initial_check = !READ_ONCE(sc->vooc_cable_state_known);
	unsigned long retry_delay;
	int online;
	int vbus_uv;

	online = sc8547_vooc_usb_online();
	vbus_uv = sc8547_get_vbus_uv(sc);
	if (online < 0 && vbus_uv < 0) {
		mod_delayed_work(system_dfl_wq, &sc->vooc_cable_work,
				 msecs_to_jiffies(SC8547_VOOC_CABLE_RECHECK_MS));
		return;
	}
	WRITE_ONCE(sc->vooc_cable_state_known, true);

	/* battmgr intentionally reports offline while the USB buck input is
	 * suspended for VOOC.  A live SC8547 VBUS distinguishes that transition
	 * from a physical cable removal. */
	if (online <= 0 && vbus_uv < SC8547_VOOC_CABLE_PRESENT_UV) {
		/* The normal VOOC cleanup also removes VBUS briefly before the USB
		 * input vote is restored.  Defer the edge while protocol work owns
		 * that transition; a real unplug remains low after cleanup and is
		 * confirmed by the next check. */
		if (atomic_read(&sc->vooc_retry_running) ||
		    READ_ONCE(sc->vooc_initialized)) {
			mod_delayed_work(system_dfl_wq, &sc->vooc_cable_work,
					 msecs_to_jiffies(SC8547_VOOC_CABLE_RECHECK_MS));
			return;
		}
		/* The official adapter reset removes its output for one second and
		 * waits another 500 ms before reactivation.  Confirm a low-VBUS edge
		 * twice so that reset is not mistaken for a physical unplug after the
		 * retry worker releases its running flag.  Actual cable removal is
		 * already fail-safe: the protocol loop loses VBUS immediately. */
		if (was_online && ++sc->vooc_cable_absent_count <
				SC8547_VOOC_CABLE_REMOVE_SAMPLES) {
			mod_delayed_work(system_dfl_wq, &sc->vooc_cable_work,
					 msecs_to_jiffies(SC8547_VOOC_CABLE_RECHECK_MS));
			return;
		}
		WRITE_ONCE(sc->vooc_cable_online, false);
		sc->vooc_cable_absent_count = 0;
		sc8547_vooc_reset_curve(sc);
		WRITE_ONCE(sc->vooc_auto_retry, false);
		atomic_set(&sc->policy_cancel, 1);
		cancel_delayed_work(&sc->vooc_retry_work);
		if (was_online)
			dev_info(sc->dev,
				 "VOOC cable removed; automatic session cancelled (vbus=%d)\n",
				 vbus_uv);
		return;
	}

	WRITE_ONCE(sc->vooc_cable_online, true);
	sc->vooc_cable_absent_count = 0;
	if (was_online)
		return;

	sc8547_vooc_reset_curve(sc);
	atomic_set(&sc->policy_cancel, 0);
	WRITE_ONCE(sc->vooc_auto_retry, true);
	sc->vooc_auto_retry_count = 0;
	retry_delay = initial_check ?
		msecs_to_jiffies(SC8547_VOOC_BOOT_SETTLE_MS) : 0;
	dev_info(sc->dev,
		 "VOOC cable inserted; scheduling automatic session in %u ms (online=%d vbus=%d)\n",
		 initial_check ? SC8547_VOOC_BOOT_SETTLE_MS : 0,
		 online, vbus_uv);
	sc8547_queue_vooc_retry(sc, retry_delay);
}

static int sc8547_vooc_psy_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct sc8547_device *sc = container_of(nb, struct sc8547_device,
						vooc_psy_nb);
	struct power_supply *psy = data;

	if (event != PSY_EVENT_PROP_CHANGED || !psy ||
	    strcmp(psy->desc->name, "qcom-battmgr-usb"))
		return NOTIFY_DONE;

	mod_delayed_work(system_dfl_wq, &sc->vooc_cable_work,
			 msecs_to_jiffies(SC8547_VOOC_CABLE_DEBOUNCE_MS));
	return NOTIFY_OK;
}

static int sc8547_pps_preflight(struct sc8547_device *sc)
{
	unsigned int reg06, reg07, reg0e;
	int ret;

	if (!sc->init_done || !sc->window.complete)
		return -EPERM;
	if (!sc8547_variant_control_supported(sc->variant))
		return -EOPNOTSUPP;

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;
	if (reg07 & SC8547_CHG_EN)
		return -EBUSY;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	if (reg06 & SC8547_CP_SWITCHING_STAT)
		return -EBUSY;

	/*
	 * SC8547A reports VBUS_ERRORLO while the pump is stopped even after the
	 * source has reached the vendor 2:1 start voltage. The vendor start path
	 * does not reject that status before CP_SET_WORK_START. Keep every other
	 * blocking fault fatal here; sc8547_pps_wait_start_window() performs the
	 * authoritative, repeated ADC check against the live 2 * VBAT target.
	 */
	if ((reg06 & (SC8547_BLOCKING_06 & ~SC8547_VBUS_ERRORLO_STAT)) ||
	    (reg0e & SC8547_BLOCKING_0E))
		return -EIO;

	return 0;
}

static int sc8547_pps_rearm_runtime(struct sc8547_device *sc)
{
	unsigned int reg09, reg11;
	int ret;

	/*
	 * The vendor policy explicitly enables ADC and its five-second watchdog
	 * before work-start. Any I2C access refreshes that watchdog; its dedicated
	 * kick helper reads REG36, while each monitor sample performs several
	 * register/ADC reads. A split between the PPS ramp and pump start
	 * can expose the watchdog-expired state (REG09[3]) and ADC=0. Reproduce
	 * those harmless runtime setup calls only after the pump-off preflight;
	 * live VBUS/VBAT are sampled and checked again before REG07 is enabled.
	 */
	ret = regmap_write(sc->regmap, SC8547_REG_ADC_CTRL, SC8547_ADC_EN);
	if (ret)
		return ret;
	ret = sc8547_set_watchdog_code(sc, SC8547A_VENDOR_PPS_WATCHDOG);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	if ((reg09 & SC8547_WATCHDOG_MASK) !=
	    SC8547A_VENDOR_PPS_WATCHDOG ||
	    (reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN)
		return -EIO;

	return 0;
}

static int sc8547_pps_wait_start_window(struct sc8547_device *sc,
					struct sc8547_pps_run_state *state)
{
	const struct sc8547_enable_window *w = &sc->window;
	unsigned int reg06, reg0e;
	unsigned int stable_samples = 0;
	unsigned int elapsed_ms = 0;
	int output_reference_uv;
	int vbus_uv;
	u64 target_uv;
	int ret;

	/*
	 * Authorize the source while ADC and insertion detection remain valid.
	 * The later hardware reset makes the stopped pump's VBUS ADC read low, so
	 * establish the vendor target window before changing the work mode.
	 */
	for (;;) {
		vbus_uv = sc8547_get_vbus_uv(sc);
		if (vbus_uv < 0)
			return vbus_uv;
		output_reference_uv = sc8547_get_output_reference_uv(sc);
		if (output_reference_uv < 0)
			return output_reference_uv;

		if (sc->pps_target_vbus_uv) {
			target_uv = sc->pps_target_vbus_uv;
		} else {
			target_uv = ((u64)output_reference_uv * 2 / 100000) * 100000;
			target_uv += w->start_delta_uv;
		}
		state->target_vbus_uv = (int)target_uv;

		if ((u32)vbus_uv > w->vbus_max_uv ||
		    (u32)output_reference_uv > w->vbat_max_uv)
			return -ERANGE;

		if ((u32)vbus_uv >= w->vbus_min_uv &&
		    (u32)output_reference_uv >= w->vbat_min_uv &&
		    (u64)vbus_uv >= target_uv &&
		    (u64)vbus_uv <= target_uv + w->start_overshoot_max_uv) {
			if (++stable_samples >= SC8547_ADC_READY_STABLE_SAMPLES)
				return 0;
		} else {
			stable_samples = 0;
		}

		ret = sc8547_read_status(sc, &reg06, &reg0e);
		if (ret)
			return ret;
		if (sc8547_has_fatal_fault(reg06, reg0e))
			return -EIO;
		if (elapsed_ms >= SC8547_ADC_READY_TIMEOUT_MS)
			return -ERANGE;

		msleep(SC8547_ADC_READY_POLL_MS);
		elapsed_ms += SC8547_ADC_READY_POLL_MS;
	}
}

static int sc8547_prepare_vendor_pps_startup(struct sc8547_device *sc,
					     bool *touched)
{
	const struct sc8547_raw_profile *idle = &sc->profile;
	unsigned int reg02, reg04, reg05, reg08, reg09, reg0c, reg0d, reg10;
	unsigned int reg11, reg2b, reg30, reg3a, reg3b, reg3c;
	bool secondary = !strcmp(sc->role, "secondary");
	u8 pps_ibus_prot;
	u32 mismatch = 0;
	int ret;

	*touched = false;
	if (!sc8547_bounded_role_supported(sc))
		return -EOPNOTSUPP;

	/*
	 * Match the wired-PPS path, not the generic CP_ENABLE helper. In
	 * oplus_chg_pps.c the CP_ENABLE call is intentionally commented out; the
	 * policy uses SET_WORK_MODE followed by CP_SET_WORK_START. Keeping the
	 * pump out of reset also preserves the already-authorized adapter path.
	 */
	*touched = true;
	ret = regmap_write(sc->regmap, SC8547_REG_AC_OVP, idle->reg02);
	if (ret)
		return ret;
	ret = regmap_write(sc->regmap, SC8547_REG_VBUS_OVP, idle->reg04);
	if (ret)
		return ret;
	/*
	 * CP_HW_INIT runs once before the downstream PPS state machine. Preserve
	 * the live ADC path by avoiding its reset here. The primary and secondary
	 * vendor drivers use different side-register profiles, so only write the
	 * fields documented for this physical role.
	 */
	ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_CTRL,
			   SC8547A_VENDOR_VOOCPHY_DISABLE);
	if (ret)
		return ret;
	if (!secondary) {
		ret = regmap_write(sc->regmap, SC8547_REG_INT_MASK,
				   SC8547A_VENDOR_INSERT_IRQ_MASK);
		if (ret)
			return ret;
		ret = regmap_write(sc->regmap, SC8547A_REG_OTG_CTRL,
				   SC8547A_VENDOR_OTG_CTRL);
		if (ret)
			return ret;
	}
	/*
	 * The vendor low-current/FCL path keeps IBUS UCP and the coupled
	 * soft-start timeout disabled until measured input current exceeds
	 * 600 mA. At low source current, enabling the pair from startup can
	 * autonomously stop the pump near the 81.92-s REG08 timeout. Preserve all
	 * OCP/OVP fields while reproducing the vendor low-current state. Runtime
	 * VBUS/VAC/ratio/temperature checks remain fail-closed every 500 ms.
	 */
	pps_ibus_prot = idle->reg05 | SC8547_IBUS_UCP_DIS;
	ret = regmap_write(sc->regmap, SC8547_REG_IBUS_PROT,
			   pps_ibus_prot);
	if (ret)
		return ret;
	ret = regmap_update_bits(sc->regmap, SC8547_REG_SS_CTRL,
				 SC8547_SS_TIMEOUT_MASK,
				 SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT);
	if (ret)
		return ret;
	ret = regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
				 SC8547A_VENDOR_PPS_MODE_MASK,
				 SC8547A_VENDOR_PPS_MODE_2TO1);
	if (ret)
		return ret;
	ret = regmap_write(sc->regmap, SC8547_REG_ADC_CTRL, SC8547_ADC_EN);
	if (ret)
		return ret;
	ret = sc8547_enable_required_adc_channels(sc);
	if (ret)
		return ret;
	ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT,
			   SC8547A_VENDOR_PPS_PMID2OUT);
	if (ret)
		return ret;
	if (secondary) {
		/* Values used by the downstream SC8547 slave init/SVOOC path. */
		ret = regmap_write(sc->regmap, SC8547_REG_SLAVE_CTRL,
				   SC8547_VENDOR_SECONDARY_CTRL);
		if (ret)
			return ret;
		ret = regmap_write(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL,
				   SC8547_VENDOR_SECONDARY_VBUS_RANGE);
		if (ret)
			return ret;
	} else {
		ret = regmap_write(sc->regmap, SC8547_REG_LOOSE_DET,
				   SC8547A_VENDOR_PPS_LOOSE_DET);
		if (ret)
			return ret;
		ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_TIMING,
				   SC8547A_VENDOR_PPS_TIMING);
		if (ret)
			return ret;
	}
	/* oplus_chg_pps.c enables the five-second watchdog before its VBUS ramp. */
	ret = sc8547_set_watchdog_code(sc, SC8547A_VENDOR_PPS_WATCHDOG);
	if (ret)
		return ret;

	/* REG07 off is role/variant-specific (primary 0x00, secondary 0x05). */
	ret = sc8547_set_charge_enabled(sc, false);
	if (ret)
		return ret;

	ret = regmap_read(sc->regmap, SC8547_REG_AC_OVP, &reg02);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_OVP, &reg04);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &reg05);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &reg08);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_CTRL, &reg0c);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_PMID2OUT, &reg0d);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_INT_MASK, &reg10);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_CTRL, &reg2b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SLAVE_CTRL, &reg30);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_TIMING, &reg3a);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547A_REG_OTG_CTRL, &reg3b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL, &reg3c);
	if (ret)
		return ret;

	if ((reg02 & SC8547_AC_OVP_CONFIG_MASK) !=
	    (idle->reg02 & SC8547_AC_OVP_CONFIG_MASK))
		mismatch |= SC8547_PREP_MISMATCH_REG02_CONFIG;
	if (reg02 & SC8547_AC_OVP_STAT)
		mismatch |= SC8547_PREP_MISMATCH_REG02_STATUS;
	if (reg04 != idle->reg04)
		mismatch |= SC8547_PREP_MISMATCH_REG04;
	if (reg05 != pps_ibus_prot)
		mismatch |= SC8547_PREP_MISMATCH_REG05;
	if ((reg08 & SC8547_SS_TIMEOUT_MASK) !=
	    SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT)
		mismatch |= SC8547_PREP_MISMATCH_REG08;
	if ((reg09 & (SC8547A_VENDOR_PPS_MODE_MASK | SC8547_WATCHDOG_MASK)) !=
	    (SC8547A_VENDOR_PPS_MODE_2TO1 | SC8547A_VENDOR_PPS_WATCHDOG))
		mismatch |= SC8547_PREP_MISMATCH_REG09;
	if ((reg0d & (SC8547_PMID2OUT_UVP_MASK | SC8547_PMID2OUT_OVP_MASK)) !=
	    SC8547A_VENDOR_PPS_PMID2OUT)
		mismatch |= SC8547_PREP_MISMATCH_REG0D;
	if ((reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN)
		mismatch |= SC8547_PREP_MISMATCH_REG11;
	if (reg2b != SC8547A_VENDOR_VOOCPHY_DISABLE)
		mismatch |= SC8547_PREP_MISMATCH_REG2B;
	if (secondary) {
		if (reg30 != SC8547_VENDOR_SECONDARY_CTRL)
			mismatch |= SC8547_PREP_MISMATCH_REG30;
		if (reg3c != SC8547_VENDOR_SECONDARY_VBUS_RANGE)
			mismatch |= SC8547_PREP_MISMATCH_REG3C;
	} else {
		if (reg10 != SC8547A_VENDOR_INSERT_IRQ_MASK)
			mismatch |= SC8547_PREP_MISMATCH_REG10;
		if (reg3a != SC8547A_VENDOR_PPS_TIMING)
			mismatch |= SC8547_PREP_MISMATCH_REG3A;
		if (reg3b != SC8547A_VENDOR_OTG_CTRL)
			mismatch |= SC8547_PREP_MISMATCH_REG3B;
	}

	if (mismatch)
		return -EIO;

	return 0;
}

/*
 * The vendor policy prepares the pump while the source is still at its low
 * PPS entry voltage, then suspends the buck and raises VBUS.  qcom_battmgr
 * invokes the CALIBRATE coordination property at that point.  The pump start
 * must only verify the preserved state here; rewriting work-mode
 * registers after VBUS is high was the ordering error in earlier stages.
 */
static int sc8547_prepare_pps_startup(struct sc8547_device *sc,
				      bool *touched)
{
	const struct sc8547_raw_profile *idle = &sc->profile;
	unsigned int reg02, reg04, reg05, reg08, reg09, reg0c, reg0d, reg10;
	unsigned int reg11, reg2b, reg30, reg3a, reg3b, reg3c;
	bool secondary = !strcmp(sc->role, "secondary");
	u8 pps_ibus_prot;
	u32 mismatch = 0;
	int ret;

	*touched = false;
	if (!sc8547_bounded_role_supported(sc))
		return -EOPNOTSUPP;
	if (!sc->pps_prepared)
		return -EPERM;
	*touched = true;

	ret = regmap_read(sc->regmap, SC8547_REG_AC_OVP, &reg02);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_OVP, &reg04);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &reg05);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &reg08);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_CTRL, &reg0c);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_PMID2OUT, &reg0d);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_INT_MASK, &reg10);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_CTRL, &reg2b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SLAVE_CTRL, &reg30);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_TIMING, &reg3a);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547A_REG_OTG_CTRL, &reg3b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL, &reg3c);
	if (ret)
		return ret;

	pps_ibus_prot = idle->reg05 | SC8547_IBUS_UCP_DIS;

	if ((reg02 & SC8547_AC_OVP_CONFIG_MASK) !=
	    (idle->reg02 & SC8547_AC_OVP_CONFIG_MASK))
		mismatch |= SC8547_PREP_MISMATCH_REG02_CONFIG;
	if (reg02 & SC8547_AC_OVP_STAT)
		mismatch |= SC8547_PREP_MISMATCH_REG02_STATUS;
	if (reg04 != idle->reg04)
		mismatch |= SC8547_PREP_MISMATCH_REG04;
	if (reg05 != pps_ibus_prot)
		mismatch |= SC8547_PREP_MISMATCH_REG05;
	if ((reg08 & SC8547_SS_TIMEOUT_MASK) !=
	    SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT)
		mismatch |= SC8547_PREP_MISMATCH_REG08;
	if ((reg09 & (SC8547A_VENDOR_PPS_MODE_MASK | SC8547_WATCHDOG_MASK)) !=
	    (SC8547A_VENDOR_PPS_MODE_2TO1 | SC8547A_VENDOR_PPS_WATCHDOG))
		mismatch |= SC8547_PREP_MISMATCH_REG09;
	if ((reg0d & (SC8547_PMID2OUT_UVP_MASK | SC8547_PMID2OUT_OVP_MASK)) !=
	    SC8547A_VENDOR_PPS_PMID2OUT)
		mismatch |= SC8547_PREP_MISMATCH_REG0D;
	if ((reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN)
		mismatch |= SC8547_PREP_MISMATCH_REG11;
	if (reg2b != SC8547A_VENDOR_VOOCPHY_DISABLE)
		mismatch |= SC8547_PREP_MISMATCH_REG2B;
	if (secondary) {
		if (reg30 != SC8547_VENDOR_SECONDARY_CTRL)
			mismatch |= SC8547_PREP_MISMATCH_REG30;
		if (reg3c != SC8547_VENDOR_SECONDARY_VBUS_RANGE)
			mismatch |= SC8547_PREP_MISMATCH_REG3C;
	} else {
		if (reg10 != SC8547A_VENDOR_INSERT_IRQ_MASK)
			mismatch |= SC8547_PREP_MISMATCH_REG10;
		if (reg3a != SC8547A_VENDOR_PPS_TIMING)
			mismatch |= SC8547_PREP_MISMATCH_REG3A;
		if (reg3b != SC8547A_VENDOR_OTG_CTRL)
			mismatch |= SC8547_PREP_MISMATCH_REG3B;
	}

	if (mismatch)
		return -EIO;

	return 0;
}

static int sc8547_post_prepare_check(struct sc8547_device *sc,
				      struct sc8547_pps_run_state *p)
{
	unsigned int reg06, reg07, reg0e, reg11, reg12;
	int output_reference_uv;
	int vout_uv;
	int vbat_uv;
	int vbus_uv;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_FN_DISABLE, &reg12);
	if (ret)
		return ret;

	/* The no-reset PPS work-start path must retain live input telemetry. */
	vbus_uv = sc8547_get_vbus_uv(sc);
	if (vbus_uv < 0)
		return vbus_uv;
	vbat_uv = sc8547_get_vbat_uv(sc);
	if (vbat_uv < 0)
		return vbat_uv;
	vout_uv = sc8547_get_vout_uv(sc);
	if (vout_uv < 0)
		return vout_uv;
	output_reference_uv = sc8547_select_output_reference_uv(sc, vout_uv,
								vbat_uv);

	if ((reg07 & SC8547_CHG_EN) ||
	    (reg06 & SC8547_CP_SWITCHING_STAT))
		return -EBUSY;
	if ((reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN ||
	    (reg12 & SC8547_REQUIRED_ADC_DIS_MASK))
		return -EIO;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	/*
	 * The live vendor PPS path does not inspect the pre-start VBUS_ERRORLO
	 * bit before CP_SET_WORK_START. The independently sampled VBUS window
	 * below remains authoritative; every other blocking condition still
	 * aborts.
	 */
	if ((reg06 & (SC8547_BLOCKING_06 & ~SC8547_VBUS_ERRORLO_STAT)) ||
	    (reg0e & SC8547_BLOCKING_0E))
		return -EIO;
	if ((u32)vbus_uv < sc->window.vbus_min_uv ||
	    (u32)vbus_uv > sc->window.vbus_max_uv ||
	    (u32)output_reference_uv < sc->window.vbat_min_uv ||
	    (u32)output_reference_uv > sc->window.vbat_max_uv ||
	    vbus_uv < p->target_vbus_uv ||
	    vbus_uv > p->target_vbus_uv +
			(int)sc->window.start_overshoot_max_uv)
		return -ERANGE;

	return 0;
}

static int sc8547_post_prepare_check_confirmed(
	struct sc8547_device *sc, struct sc8547_pps_run_state *p)
{
	unsigned int attempt;
	int ret;

	ret = sc8547_post_prepare_check(sc, p);
	if (ret != -ERANGE)
		return ret;

	/*
	 * Preserve the exact vendor target-through-target+200-mV window, but do
	 * not reject an otherwise ready pump on one torn ADC conversion. Status,
	 * insertion, register and I/O failures remain immediate. This mirrors the
	 * bounded numeric-only confirmation already used by the active monitor.
	 */
	for (attempt = 0;
	     attempt < SC8547_POST_PREPARE_RANGE_RECHECKS; attempt++) {
		usleep_range(SC8547_RUNTIME_RANGE_RECHECK_MIN_US,
			      SC8547_RUNTIME_RANGE_RECHECK_MAX_US);
		ret = sc8547_post_prepare_check(sc, p);
		if (ret != -ERANGE)
			return ret;
	}

	return ret;
}

static int sc8547_retry_start_preflight(struct sc8547_device *sc,
					 struct sc8547_pps_run_state *p)
{
	const struct sc8547_enable_window *w = &sc->window;
	unsigned int reg06, reg0e;
	int output_reference_uv;
	int tdie_mc;
	int vbus_uv;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	if ((reg06 & SC8547_CP_SWITCHING_STAT) ||
	    sc8547_has_fatal_fault(reg06, reg0e))
		return -EIO;

	vbus_uv = sc8547_get_vbus_uv(sc);
	if (vbus_uv < 0)
		return vbus_uv;
	output_reference_uv = sc8547_get_output_reference_uv(sc);
	if (output_reference_uv < 0)
		return output_reference_uv;
	tdie_mc = sc8547_get_tdie_mc(sc);
	if (tdie_mc < 0)
		return tdie_mc;
	if ((u32)vbus_uv < w->vbus_min_uv ||
	    (u32)vbus_uv > w->vbus_max_uv ||
	    (u32)output_reference_uv < w->vbat_min_uv ||
	    (u32)output_reference_uv > w->vbat_max_uv ||
	    (u32)tdie_mc > sc->tdie_max_mc ||
	    vbus_uv < p->target_vbus_uv ||
	    vbus_uv > p->target_vbus_uv +
			(int)w->start_overshoot_max_uv)
		return -ERANGE;

	return 0;
}

static int sc8547_disable_charge_checked(struct sc8547_device *sc)
{
	unsigned int reg;
	int ret;

	ret = sc8547_set_charge_enabled(sc, false);
	if (ret)
		return ret;
	usleep_range(10000, 20000);

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
	if (ret)
		return ret;
	if (reg & SC8547_CHG_EN)
		return -EIO;

	return 0;
}

static int sc8547_capture_active_sample(struct sc8547_device *sc,
					 struct sc8547_pps_run_state *p,
					 struct sc8547_active_sample *s)
{
	const struct sc8547_enable_window *w = &sc->window;
	unsigned int reg06, reg07, reg0e;
	u32 ibus_max_ua = p->vendor_current_control ?
		SC8547_VENDOR_CURRENT_IBUS_ADC_MAX_UA :
		SC8547_PPS_FIXED_IBUS_MAX_UA;
	int headroom_uv;
	int ret;

	memset(s, 0, sizeof(*s));
	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;

	s->vbus_uv = sc8547_get_vbus_uv(sc);
	if (s->vbus_uv < 0)
		return s->vbus_uv;
	s->vbat_uv = sc8547_get_vbat_uv(sc);
	if (s->vbat_uv < 0)
		return s->vbat_uv;
	s->vac_uv = sc8547_get_vac_uv(sc);
	if (s->vac_uv < 0)
		return s->vac_uv;
	s->vout_uv = sc8547_get_vout_uv(sc);
	if (s->vout_uv < 0)
		return s->vout_uv;
	s->output_reference_uv = sc8547_select_output_reference_uv(
		sc, s->vout_uv, s->vbat_uv);
	s->ibus_ua = sc8547_get_ibus_ua(sc);
	if (s->ibus_ua < 0)
		return s->ibus_ua;
	s->tdie_mc = sc8547_get_tdie_mc(sc);
	if (s->tdie_mc < 0)
		return s->tdie_mc;

	/*
	 * The vendor's +100-mV third-party PPS floor constrains the requested
	 * source voltage and is derived from gauge VBAT.  It is not a physical
	 * worker shutdown threshold against a pump-local ADC: on this board the
	 * primary VBAT reading can sit above the gauge terminal reading, while the
	 * secondary VBAT channel is not connected.  Keep the role-correct local
	 * output for the role-correct conversion ratio and reject excessive low-side
	 * error here.  Positive headroom remains bounded independently by the
	 * absolute VBUS/VAC, output, current, temperature and hardware-fault gates.
	 */
	headroom_uv = s->vbus_uv - 2 * s->output_reference_uv;
	s->ratio_error_uv = s->vbus_uv - 2 * s->output_reference_uv;
	if (s->ratio_error_uv < 0)
		s->ratio_error_uv = -s->ratio_error_uv;
	if (!(reg07 & SC8547_CHG_EN) ||
	    !(reg06 & SC8547_CP_SWITCHING_STAT) ||
	    sc8547_has_blocking_fault(reg06, reg0e))
		return -EIO;
	if ((u32)s->vbus_uv < w->active_vbus_min_uv ||
	    (u32)s->vbus_uv > w->vbus_max_uv ||
	    (u32)s->vac_uv < w->active_vbus_min_uv ||
	    (u32)s->vac_uv > w->vbus_max_uv ||
	    (u32)s->output_reference_uv < w->vbat_min_uv ||
	    (u32)s->output_reference_uv > w->vbat_max_uv ||
	    (p->vendor_current_control ?
	     headroom_uv < -(int)w->ratio_error_max_uv :
	     (u32)s->ratio_error_uv > w->ratio_error_max_uv) ||
	    (u32)s->ibus_ua > ibus_max_ua ||
	    (u32)s->tdie_mc > sc->tdie_max_mc)
		return -ERANGE;

	return 0;
}

static int sc8547_capture_confirmed_active_sample(
	struct sc8547_device *sc, struct sc8547_pps_run_state *p,
	struct sc8547_active_sample *s)
{
	unsigned int attempt;
	int ret;

	ret = sc8547_capture_active_sample(sc, p, s);
	if (ret != -ERANGE)
		return ret;

	/*
	 * Status faults, a stopped pump and I2C errors remain immediate failures.
	 * Only a numeric-window rejection is sampled twice more.  This filters a
	 * torn or transient ADC conversion without relaxing any hardware limit.
	 */
	for (attempt = 0; attempt < SC8547_RUNTIME_RANGE_RECHECKS; attempt++) {
		if (atomic_read(&sc->policy_cancel))
			return -ECANCELED;
		usleep_range(SC8547_RUNTIME_RANGE_RECHECK_MIN_US,
			      SC8547_RUNTIME_RANGE_RECHECK_MAX_US);
		ret = sc8547_capture_active_sample(sc, p, s);
		if (ret != -ERANGE)
			return ret;
	}

	return ret;
}

static int sc8547_pps_run(struct sc8547_device *sc, unsigned int run_ms,
			  bool vendor_current_control, bool continuous)
{
	struct sc8547_pps_run_state state = { };
	struct sc8547_pps_run_state *p = &state;
	struct sc8547_active_sample active;
	unsigned int attempt;
	unsigned int sample;
	unsigned int sample_goal;
	unsigned int reg06, reg07, reg0e;
	bool prep_touched = false;
	int recovery_ret;
	int disable_ret = 0;
	int final_ret;
	int restore_ret;
	int tdie_mc;
	int ret;

	atomic_set(&sc->pps_running, 1);
	p->vendor_current_control = vendor_current_control;
	if ((!continuous &&
	     (!run_ms || run_ms % SC8547_STABILITY_SAMPLE_MS ||
	      run_ms > SC8547_POLICY_RUN_MS)) ||
	    (continuous && run_ms)) {
		ret = -EINVAL;
		goto record;
	}
	sample_goal = continuous ? 0 :
		run_ms / SC8547_STABILITY_SAMPLE_MS;
	if (!sc->pps_capable ||
	    !sc8547_bounded_role_supported(sc)) {
		ret = -EPERM;
		goto record;
	}
	if (!sc->pps_prepared) {
		ret = -EPERM;
		goto record;
	}

	/* Preserve the vendor watchdog while confirming the role-specific off state. */
	ret = sc8547_disable_charge_checked(sc);
	if (ret)
		goto record;
	usleep_range(10000, 20000);

	ret = sc8547_pps_preflight(sc);
	if (ret)
		goto record;
	ret = sc8547_pps_rearm_runtime(sc);
	if (ret)
		goto record;

	tdie_mc = sc8547_get_tdie_mc(sc);
	if (tdie_mc < 0) {
		ret = tdie_mc;
		goto record;
	}
	if ((u32)tdie_mc > sc->tdie_max_mc) {
		ret = -ERANGE;
		goto record;
	}
	ret = sc8547_pps_preflight(sc);
	if (ret)
		goto record;
	ret = sc8547_pps_wait_start_window(sc, p);
	if (ret)
		goto record;
	ret = sc8547_prepare_pps_startup(sc, &prep_touched);
	if (ret)
		goto disable;

	/*
	 * Re-check controls and live input after the no-reset PPS work-mode setup.
	 * Only then reproduce CP_SET_WORK_START with the bounded REG07 write.
	 */
	ret = sc8547_post_prepare_check_confirmed(sc, p);
	if (ret)
		goto disable;

	/*
	 * Match oplus_pps_charge_start() rather than failing its first immediate
	 * REG07 readback. The vendor state machine writes work-start, waits 500
	 * ms, checks REG07[7], and permits three failed checks before its terminal
	 * disable path. Revalidate the live target window before each retry and
	 * retain a bounded retry sequence.
	 */
	for (attempt = 0; attempt < SC8547_VENDOR_START_ATTEMPTS; attempt++) {
		if (atomic_read(&sc->policy_cancel)) {
			ret = -ECANCELED;
			goto disable;
		}
		if (attempt) {
			ret = sc8547_retry_start_preflight(sc, p);
			if (ret)
				goto disable;
		}

		/* Use the downstream role/variant-specific work-start command. */
		ret = sc8547_set_charge_enabled(sc, true);
		if (ret)
			goto disable;

		ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
		if (ret)
			goto disable;

		msleep(SC8547_VENDOR_START_CHECK_MS);
		ret = sc8547_read_status(sc, &reg06, &reg0e);
		if (ret)
			goto disable;
		ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
		if (ret)
			goto disable;
		if (reg07 & SC8547_CHG_EN)
			break;
		if (sc8547_has_fatal_fault(reg06, reg0e)) {
			ret = -EIO;
			goto reject;
		}
	}

	if (!(reg07 & SC8547_CHG_EN)) {
		ret = -EIO;
		goto reject;
	}

	/*
	 * The +450 mV target belongs only to the pre-start source ramp. For
	 * fixed-ratio PPS, retain the narrow 2:1 ratio-error window. The vendor-shaped
	 * current-controlled run instead uses the vendor runtime floor of
	 * 2 * VBAT + 100 mV. VOUT remains the conversion reference, not
	 * the floor reference. Absolute VBUS, IBUS, temperature and hardware-fault
	 * limits remain unchanged. The first sample is the vendor 500-ms start
	 * check; longer endpoints continue fail-closed sampling at 500-ms intervals
	 * while REG07 reads refresh the watchdog.
	 */
	for (sample = 0; continuous || sample < sample_goal; sample++) {
		if (sample)
			msleep(SC8547_STABILITY_SAMPLE_MS);
		if (atomic_read(&sc->policy_cancel)) {
			ret = -ECANCELED;
			goto disable;
		}
		ret = sc8547_capture_confirmed_active_sample(sc, p, &active);
		if (ret)
			goto reject;
	}
	goto disable;

reject:
disable:
	/* Always turn the pump off before returning to userspace. */
	disable_ret = sc8547_disable_charge_checked(sc);
	if (disable_ret) {
		/* Retry through the broader fail-closed path if readback/write failed. */
		recovery_ret = sc8547_fail_closed(sc);
		if (!recovery_ret)
			usleep_range(10000, 20000);
	}
	if (!ret && disable_ret)
		ret = disable_ret;
	if (prep_touched) {
		restore_ret = sc8547_restore_idle_profile(sc);
		if (!ret && restore_ret)
			ret = restore_ret;
	}

record:
	final_ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (!final_ret)
		final_ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (!final_ret) {
		if ((reg06 & SC8547_CP_SWITCHING_STAT) ||
		    (reg07 & SC8547_CHG_EN)) {
			if (!ret)
				ret = -EIO;
		}
	} else if (!ret) {
		ret = final_ret;
	}
	atomic_set(&sc->pps_running, 0);

	return ret;
}

static int sc8547_run_pps_bounded(struct sc8547_device *sc,
				    unsigned int run_ms,
				    bool vendor_current_control)
{
	return sc8547_pps_run(sc, run_ms, vendor_current_control, false);
}

static int sc8547_run_pps_continuous(struct sc8547_device *sc)
{
	return sc8547_pps_run(sc, 0, true, true);
}

static int sc8547_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct sc8547_device *sc;
	unsigned int id, mode, enabled, status;
	const char *role;
	char *psy_name;
	int ret;

	sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &client->dev;
	sc->match_info = device_get_match_data(&client->dev);
	mutex_init(&sc->lock);
	atomic_set(&sc->policy_cancel, 0);
	atomic_set(&sc->pps_running, 0);
	atomic_set(&sc->vooc_retry_running, 0);
	INIT_DELAYED_WORK(&sc->vooc_retry_work, sc8547_vooc_retry_workfn);
	INIT_DELAYED_WORK(&sc->vooc_cable_work, sc8547_vooc_cable_workfn);
	sc->vooc_psy_nb.notifier_call = sc8547_vooc_psy_notifier;
	sc->regmap = devm_regmap_init_i2c(client, &sc8547_regmap_config);
	if (IS_ERR(sc->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(sc->regmap),
				     "failed to initialize regmap\n");

	i2c_set_clientdata(client, sc);

	ret = regmap_read(sc->regmap, SC8547_REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read device ID\n");

	sc->device_id = id;
	sc->variant = sc8547_detect_variant(sc, sc->device_id);
	if (device_property_read_string(&client->dev, "southchip,role", &role))
		role = "standalone";
	sc->role = role;

	if (sc->match_info && sc->match_info->variant != sc->variant &&
	    sc->variant != SC8547_VARIANT_UNKNOWN)
		dev_warn(&client->dev,
			 "DT compatible suggests %s but device ID 0x%02x identifies %s\n",
			 sc->match_info->name, sc->device_id,
			 sc8547_variant_name(sc->variant));

	sc8547_init_board_config(sc);
	if (sc->vooc_capable) {
		sc->vooc_wq = alloc_ordered_workqueue("sc8547-vooc",
						     WQ_MEM_RECLAIM);
		if (!sc->vooc_wq)
			return -ENOMEM;
		ret = devm_add_action_or_reset(&client->dev,
					       sc8547_destroy_vooc_workqueue,
					       sc->vooc_wq);
		if (ret)
			return ret;
	}

	/* ADC enable alone does not start charge pumping. */
	ret = sc8547_set_adc_enabled(sc, true);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to enable ADC\n");
	ret = sc8547_enable_required_adc_channels(sc);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to enable required ADC channels\n");

	/* power_supply may register a thermal zone using this name; thermal zone
	 * names are limited to THERMAL_NAME_LENGTH (20 bytes including NUL).
	 * Keep role in the dedicated sysfs attribute and use the bus/address here.
	 */
	psy_name = devm_kasprintf(&client->dev, GFP_KERNEL, "sc8547-%d-%02x",
				   client->adapter->nr, client->addr);
	if (!psy_name)
		return -ENOMEM;

	sc->psy_desc.name = psy_name;
	sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	sc->psy_desc.no_thermal = true;
	sc->psy_desc.properties = sc8547_psy_props;
	sc->psy_desc.num_properties = ARRAY_SIZE(sc8547_psy_props);
	sc->psy_desc.get_property = sc8547_psy_get_property;
	sc->psy_desc.set_property = sc8547_psy_set_property;
	sc->psy_desc.property_is_writeable = sc8547_psy_property_is_writeable;
	psy_cfg.drv_data = sc;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	sc->psy = devm_power_supply_register(&client->dev, &sc->psy_desc, &psy_cfg);
	if (IS_ERR(sc->psy))
		return dev_err_probe(&client->dev, PTR_ERR(sc->psy),
				     "failed to register power supply\n");

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &enabled);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &mode);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, &status);
	if (ret)
		return ret;

	dev_info(&client->dev,
		 "%s ID=0x%02x role=%s CP=%s switching=%s mode=%s PPS=%s VOOC=%s\n",
		 sc8547_variant_name(sc->variant), sc->device_id, sc->role,
		 enabled & SC8547_CHG_EN ? "on" : "off",
		 status & SC8547_CP_SWITCHING_STAT ? "yes" : "no",
		 mode & SC8547_CHARGE_MODE ? "bypass" : "2:1",
		 sc->pps_capable ? "yes" : "no",
		 sc->vooc_capable ? "yes" : "no");

	if (!sc8547_variant_control_supported(sc->variant))
		dev_warn(&client->dev,
			 "silicon variant does not support charge-pump control\n");

	if (sc->vooc_capable) {
		ret = power_supply_reg_notifier(&sc->vooc_psy_nb);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to register VOOC power notifier\n");
		mod_delayed_work(system_dfl_wq, &sc->vooc_cable_work,
				 msecs_to_jiffies(SC8547_VOOC_CABLE_DEBOUNCE_MS));
		if (sc->vooc_auto_retry)
			sc8547_queue_vooc_retry(sc,
				msecs_to_jiffies(SC8547_VOOC_BOOT_SETTLE_MS));
	}

	return 0;
}

static int sc8547_suspend(struct device *dev)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);

	if (!sc || !sc->vooc_capable)
		return 0;

	sc->vooc_resume_retry = sc->vooc_initialized ||
		READ_ONCE(sc->vooc_auto_retry);
	WRITE_ONCE(sc->vooc_auto_retry, false);
	atomic_set(&sc->policy_cancel, 1);
	cancel_delayed_work_sync(&sc->vooc_cable_work);
	cancel_delayed_work_sync(&sc->vooc_retry_work);
	if (sc->vooc_resume_retry)
		dev_info(sc->dev, "VOOC session stopped for system suspend\n");

	return 0;
}

static int sc8547_resume(struct device *dev)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);

	if (!sc || !sc->vooc_capable || !sc->vooc_resume_retry)
		return 0;

	sc->vooc_resume_retry = false;
	atomic_set(&sc->policy_cancel, 0);
	WRITE_ONCE(sc->vooc_auto_retry, true);
	sc->vooc_auto_retry_count = 0;
	sc8547_queue_vooc_retry(sc, msecs_to_jiffies(1000));
	dev_info(sc->dev, "VOOC session retry queued after system resume\n");

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc8547_pm_ops, sc8547_suspend,
				 sc8547_resume);

static void sc8547_shutdown(struct i2c_client *client)
{
	struct sc8547_device *sc = i2c_get_clientdata(client);

	if (!sc)
		return;

	if (sc->vooc_capable)
		power_supply_unreg_notifier(&sc->vooc_psy_nb);
	WRITE_ONCE(sc->vooc_auto_retry, false);
	atomic_set(&sc->policy_cancel, 1);
	cancel_delayed_work_sync(&sc->vooc_cable_work);
	cancel_delayed_work_sync(&sc->vooc_retry_work);

	/* Best-effort fail closed; never leave a charge pump or watchdog active. */
	sc8547_fail_closed(sc);
}

static void sc8547_remove(struct i2c_client *client)
{
	struct sc8547_device *sc = i2c_get_clientdata(client);

	if (!sc)
		return;
	if (sc->vooc_capable)
		power_supply_unreg_notifier(&sc->vooc_psy_nb);
	WRITE_ONCE(sc->vooc_auto_retry, false);
	atomic_set(&sc->policy_cancel, 1);
	cancel_delayed_work_sync(&sc->vooc_cable_work);
	cancel_delayed_work_sync(&sc->vooc_retry_work);
}

static const struct of_device_id sc8547_of_match[] = {
	{ .compatible = "southchip,sc8547", .data = &sc8547_info },
	{ .compatible = "southchip,sc8547a", .data = &sc8547a_info },
	/* Accept downstream compatible aliases used by existing device trees. */
	{ .compatible = "oplus,sc8547a", .data = &sc8547a_info },
	{ .compatible = "slave_vphy_sc8547", .data = &sc8547_info },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8547_of_match);

static const struct i2c_device_id sc8547_i2c_id[] = {
	{ "sc8547" },
	{ "sc8547a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc8547_i2c_id);

static struct i2c_driver sc8547_driver = {
	.driver = {
		.name = "sc8547",
		.of_match_table = sc8547_of_match,
		.pm = pm_sleep_ptr(&sc8547_pm_ops),
	},
	.probe = sc8547_probe,
	.remove = sc8547_remove,
	.shutdown = sc8547_shutdown,
	.id_table = sc8547_i2c_id,
};
module_i2c_driver(sc8547_driver);

MODULE_DESCRIPTION("Southchip SC8547/SC8547A charge-pump driver");
MODULE_LICENSE("GPL");
