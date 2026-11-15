/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */
#ifndef _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H
#define _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct power_supply;

enum caihong_charge_protocol {
	CAIHONG_CHARGE_IDLE,
	CAIHONG_CHARGE_VOOC,
	CAIHONG_CHARGE_PPS,
	CAIHONG_CHARGE_PD,
};

/* Stable active-session gate; does not dereference driver-private memory. */

#if IS_REACHABLE(CONFIG_BATTERY_QCOM_BATTMGR_CAIHONG)
bool qcom_battmgr_caihong_pps_high_current_active(void);
int qcom_battmgr_caihong_get_raw_capacity(struct power_supply *psy);
bool qcom_battmgr_caihong_charge_is_inhibited(struct power_supply *psy);
int qcom_battmgr_caihong_notify_fastchg_complete(struct power_supply *psy);
int qcom_battmgr_caihong_source_has_pps(void);
int qcom_battmgr_caihong_source_present(void);
int qcom_battmgr_caihong_vooc_pd_allowed(void);
u32 qcom_battmgr_caihong_source_generation(void);
u32 qcom_battmgr_caihong_source_changed(u32 previous_generation);
bool qcom_battmgr_caihong_pps_allowed(void);
void qcom_battmgr_caihong_vooc_skip(u32 generation);
int qcom_battmgr_caihong_protocol_claim(enum caihong_charge_protocol protocol,
					u32 *generation);
void qcom_battmgr_caihong_protocol_release(enum caihong_charge_protocol protocol,
					   u32 generation, bool retry, bool safe);
#else
static inline bool qcom_battmgr_caihong_pps_high_current_active(void)
{
	return false;
}

static inline bool
qcom_battmgr_caihong_charge_is_inhibited(struct power_supply *psy)
{
	return false;
}

static inline int
qcom_battmgr_caihong_get_raw_capacity(struct power_supply *psy)
{
	return -EOPNOTSUPP;
}

static inline int
qcom_battmgr_caihong_notify_fastchg_complete(struct power_supply *psy)
{
	return -EOPNOTSUPP;
}

static inline int qcom_battmgr_caihong_source_has_pps(void)
{
	return false;
}

static inline int qcom_battmgr_caihong_source_present(void)
{
	return -EOPNOTSUPP;
}

static inline int qcom_battmgr_caihong_vooc_pd_allowed(void)
{
	return -EOPNOTSUPP;
}

static inline u32 qcom_battmgr_caihong_source_generation(void)
{
	return 0;
}

static inline u32 qcom_battmgr_caihong_source_changed(u32 previous_generation)
{
	return 0;
}

static inline bool qcom_battmgr_caihong_pps_allowed(void)
{
	return true;
}

static inline void qcom_battmgr_caihong_vooc_skip(u32 generation)
{
}

static inline int
qcom_battmgr_caihong_protocol_claim(enum caihong_charge_protocol protocol,
				    u32 *generation)
{
	*generation = 0;
	return 0;
}

static inline void
qcom_battmgr_caihong_protocol_release(enum caihong_charge_protocol protocol,
				      u32 generation, bool retry, bool safe)
{
}
#endif

#endif /* _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H */
