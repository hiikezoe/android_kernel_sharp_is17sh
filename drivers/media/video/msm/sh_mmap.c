/* drivers/media/video/msm/sh_mmap.c
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

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/module.h>
#include <asm/pgtable.h>

#include "sh_mmap.h"

int sh_cam_sys_mmap(struct file *filep, struct vm_area_struct *vma)
{
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    if (io_remap_pfn_range(vma, vma->vm_start,
                         (unsigned long)SH_CAM_HCS1_ADDR >> PAGE_SHIFT,
                         SH_CAM_HCS1_SIZE,
                         vma->vm_page_prot)) {
        return -EAGAIN;
    }

    return 0;
}

EXPORT_SYMBOL(sh_cam_sys_mmap);

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.01");
