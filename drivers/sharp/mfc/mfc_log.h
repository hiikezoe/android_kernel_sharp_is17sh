/* drivers/sharp/mfc/mfc_log.h (MFC LOG Header)
 *
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MFC_LOG_H
#define MFC_LOG_H

/* DEBUG_LOG */
#if 0
#define DEBUG_FELICA_DRV
#define DEBUG_MFCUART_DRV
#endif

#ifdef DEBUG_FELICA_DRV
#define FELICA_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[FeliCa][%s]" fmt "\n", __func__, ## args)
#else
#define FELICA_DRV_DBG_LOG(fmt, args...)
#endif

#ifdef DEBUG_MFCUART_DRV
#define MFCUART_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[MFCUART][%s]" fmt "\n", __func__, ## args)
#else
#define MFCUART_DRV_DBG_LOG(fmt, args...)
#endif

/* ERROR_LOG */
#define FELICA_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[FeliCa][%s]ERR " fmt "\n", __func__, ## args)
#define MFCUART_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[MFCUART][%s]ERR " fmt "\n", __func__, ## args)

#endif /* MFC_LOG_H */
