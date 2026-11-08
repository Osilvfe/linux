/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */
#ifndef _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H
#define _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct power_supply;

/* Stable active-session gate; does not dereference driver-private memory. */

#if IS_REACHABLE(CONFIG_BATTERY_QCOM_BATTMGR_CAIHONG)
bool qcom_battmgr_caihong_pps_high_current_active(void);
int qcom_battmgr_caihong_get_raw_capacity(struct power_supply *psy);
bool qcom_battmgr_caihong_charge_is_inhibited(struct power_supply *psy);
int qcom_battmgr_caihong_notify_fastchg_complete(struct power_supply *psy);
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
#endif

#endif /* _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H */
