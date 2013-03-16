/* drivers/sharp/shcamsensor/sh_subcamdrv.h  (Camera Driver)
 *
 * Copyright (C) 2009-2012 SHARP CORPORATION
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

#ifndef __SH_CAMERA_H
#define __SH_CAMERA_H

#include <linux/poll.h>
#include <linux/types.h>

#include "msm_camera.h"

#define SH_CFG_READ_IRQ_KIND          (CFG_MAX + 1)
#define SH_CFG_ENABLE_IRQ             (CFG_MAX + 2)
#define SH_CFG_DISABLE_IRQ            (CFG_MAX + 3)
#define SH_CFG_CAMIF_PAD_RESET        (CFG_MAX + 4)
#define SH_CFG_REQUEST_IRQ            (CFG_MAX + 5)
#define SH_CFG_FREE_IRQ               (CFG_MAX + 6)
#define SH_CFG_CSI_REQUEST_IRQ        (CFG_MAX + 7)
#define SH_CFG_CSI_FREE_IRQ           (CFG_MAX + 8)
#define SH_CFG_GPIO_CTRL              (CFG_MAX + 9)
#define SH_CFG_EVENT_REQUEST_ISR      (CFG_MAX + 10)
#define SH_CFG_FRAMEEVENT_REQUEST_ISR (CFG_MAX + 11)
#define SH_CFG_PRODUCT_REQUEST_ISR    (CFG_MAX + 12)
#define SH_CFG_READ_ADJUST_DATA       (CFG_MAX + 13)
#define SH_CFG_SET_DEBUG_LOG          (CFG_MAX + 14)
#define SH_CFG_READ_DSP               (CFG_MAX + 15)
#define SH_CFG_WRITE_DSP              (CFG_MAX + 16)
#define SH_CFG_READ_DEVICE_ID         (CFG_MAX + 17)
#define SH_CFG_CAMINT_EXIT            (CFG_MAX + 18)

#define CAM_INT_TYPE_VS               1
#define CAM_INT_TYPE_INT              2
#define CAM_INT_TYPE_EVENT_ISR        3
#define CAM_INT_TYPE_FRAMEEVENT_ISR   4
#define CAM_INT_TYPE_PRODUCT_ISR      5
#define CAM_INT_TYPE_TIMEOUT          6
#define CAM_INT_TYPE_MIPI_ERR         7
#define CAM_INT_TYPE_EXIT             8

#define CAM_GPIO_LO (0)
#define CAM_GPIO_HI (1)

#endif
