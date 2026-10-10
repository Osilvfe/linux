/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026 Osilvfe <osilvfe@gmail.com> */
#ifndef _LINUX_WCD939X_USBSS_H
#define _LINUX_WCD939X_USBSS_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#if IS_REACHABLE(CONFIG_TYPEC_MUX_WCD939X_USBSS)
int wcd939x_usbss_set_vooc_dpdm(bool enable);
#else
static inline int wcd939x_usbss_set_vooc_dpdm(bool enable)
{
	return -EOPNOTSUPP;
}
#endif

#endif
