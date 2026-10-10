/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */
/*
 * Compile-time boundary between the generic Qualcomm BATTMGR driver and the
 * Caihong product policy.  Disabled builds use no-op hooks and retain the
 * generic driver's behaviour.
 */

#if IS_ENABLED(CONFIG_BATTERY_QCOM_BATTMGR_CAIHONG)
#include "qcom_battmgr_caihong_policy.inc"
#else
static inline u32 qcom_battmgr_oneplus_get_ui_soc(
	struct qcom_battmgr *battmgr)
{
	return battmgr->status.percent;
}

static inline int qcom_battmgr_oneplus_battery_status(
	struct qcom_battmgr *battmgr, int firmware_status)
{
	return firmware_status;
}

static inline int qcom_battmgr_oneplus_battery_health(
	struct qcom_battmgr *battmgr, int firmware_health)
{
	return firmware_health;
}

static inline int qcom_battmgr_oneplus_battery_current(
	struct qcom_battmgr *battmgr, int firmware_current)
{
	return firmware_current;
}

static inline int qcom_battmgr_caihong_update_battery_property(
	struct qcom_battmgr *battmgr, enum power_supply_property psp)
{
	return -EOPNOTSUPP;
}

static inline void qcom_battmgr_caihong_cache_vbat(
	struct qcom_battmgr *battmgr, u32 val)
{
}

static inline bool qcom_battmgr_caihong_handle_response(
	struct qcom_battmgr *battmgr, const void *data, size_t len)
{
	return false;
}

static inline bool qcom_battmgr_caihong_battery_property_received(
	struct qcom_battmgr *battmgr, u32 property, u32 value)
{
	return false;
}

static inline void qcom_battmgr_caihong_notifications_ready(
	struct qcom_battmgr *battmgr)
{
}

static inline void qcom_battmgr_caihong_service_event(
	struct qcom_battmgr *battmgr, bool service_up)
{
}

static inline int qcom_battmgr_caihong_init(struct qcom_battmgr *battmgr)
{
	return 0;
}

static inline int qcom_battmgr_caihong_register(struct qcom_battmgr *battmgr)
{
	return 0;
}

static inline void qcom_battmgr_caihong_remove(struct qcom_battmgr *battmgr)
{
}
#endif
