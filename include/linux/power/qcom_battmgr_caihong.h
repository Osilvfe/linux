/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */
#ifndef _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H
#define _LINUX_POWER_QCOM_BATTMGR_CAIHONG_H

#include <linux/errno.h>
#include <linux/kconfig.h>

struct power_supply;

#if IS_REACHABLE(CONFIG_BATTERY_QCOM_BATTMGR_CAIHONG)
int qcom_battmgr_caihong_get_raw_capacity(struct power_supply *psy);
int qcom_battmgr_caihong_notify_fastchg_complete(struct power_supply *psy);
#else
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
