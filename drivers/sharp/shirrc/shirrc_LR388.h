/* drivers/sharp/shirrc/shirrc_LR388.h (Infrared driver)
 *
 * Copyright (C) 2010-2012 SHARP CORPORATION All rights reserved.
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

#ifndef _IRRC_REG_H
#define	_IRRC_REG_H

#include <stdbool.h>

#define IRRC_REG_SUCCESS	(0)
#define IRRC_REG_ERROR		(-1)

#define IRRC_ADDR_HCS1		0x8B000000
#define IRRC_ADDR_HCS1_SIZE	0x1000

extern void __iomem *irrc_lr388_hcs1;

#define	IRRC_GLT_REG_BASE	irrc_lr388_hcs1
#ifdef CONFIG_SHARP_INFRARED_LR388G7
#define	IRRC_GLT_REG_INTR	(IRRC_GLT_REG_BASE + 0x0002)
#define	IRRC_GLT_REG_IRRCDIV	(IRRC_GLT_REG_BASE + 0x0030)
#endif
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_GLT_REG_INTR0	(IRRC_GLT_REG_BASE + 0x0010)
#define	IRRC_GLT_REG_IRRCDIV	(IRRC_GLT_REG_BASE + 0x00CE)
#endif

#define	IRRC_GLT_WRITE_REG(Reg, Val)				\
{								\
	*((volatile uint16*)(Reg)) = (Val);			\
}

#ifdef CONFIG_SHARP_INFRARED_LR388G7
#define	IRRC_REG_OFFSET		0x04C0
#endif
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_REG_OFFSET		0x9900
#endif
#define	IRRC_REG_SYS		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x00)
#define	IRRC_REG_BASE		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x02)
#define	IRRC_REG_CLO		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x04)
#define	IRRC_REG_CHI		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x06)
#define	IRRC_REG_HLOL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x08)
#define	IRRC_REG_HLOH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x0a)
#define	IRRC_REG_HHIL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x0c)
#define	IRRC_REG_HHIH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x0e)
#define	IRRC_REG_D0LOL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x10)
#define	IRRC_REG_D0LOH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x12)
#define	IRRC_REG_D0HIL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x14)
#define	IRRC_REG_D0HIH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x16)
#define	IRRC_REG_D1LOL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x18)
#define	IRRC_REG_D1LOH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x1a)
#define	IRRC_REG_D1HIL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x1c)
#define	IRRC_REG_D1HIH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x1e)
#define	IRRC_REG_ENDLENL	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x20)
#define	IRRC_REG_ENDLENH	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x22)
#define	IRRC_REG_BITLEN		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x24)
#define	IRRC_REG_FRMLENL	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x26)
#define	IRRC_REG_FRMLENH	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x28)
#define	IRRC_REG_OUT0		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x2a)
#define	IRRC_REG_SEND		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x3a)
#define	IRRC_REG_REGS		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x3c)
#define	IRRC_REG_DBG		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x3e)
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_REG_INTSTAT	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x40)
#define	IRRC_REG_INTMASK	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x42)
#endif

#define	IRRC_READ_SYS		(*(volatile uint16*)IRRC_REG_SYS)
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_READ_INTSTAT	(*(volatile uint16*)IRRC_REG_INTSTAT)
#define	IRRC_READ_INTMASK	(*(volatile uint16*)IRRC_REG_INTMASK)
#endif

#define	IRRC_WRITE_SYS(v)	(*(volatile uint16*)IRRC_REG_SYS	= (v))
#define	IRRC_WRITE_BASE(v)	(*(volatile uint16*)IRRC_REG_BASE	= (v))
#define	IRRC_WRITE_CLO(v)	(*(volatile uint16*)IRRC_REG_CLO	= (v))
#define	IRRC_WRITE_CHI(v)	(*(volatile uint16*)IRRC_REG_CHI	= (v))
#define	IRRC_WRITE_HLOL(v)	(*(volatile uint16*)IRRC_REG_HLOL	= (v))
#define	IRRC_WRITE_HLOH(v)	(*(volatile uint16*)IRRC_REG_HLOH	= (v))
#define	IRRC_WRITE_HHIL(v)	(*(volatile uint16*)IRRC_REG_HHIL	= (v))
#define	IRRC_WRITE_HHIH(v)	(*(volatile uint16*)IRRC_REG_HHIH	= (v))
#define	IRRC_WRITE_D0LOL(v)	(*(volatile uint16*)IRRC_REG_D0LOL	= (v))
#define	IRRC_WRITE_D0LOH(v)	(*(volatile uint16*)IRRC_REG_D0LOH	= (v))
#define	IRRC_WRITE_D0HIL(v)	(*(volatile uint16*)IRRC_REG_D0HIL	= (v))
#define	IRRC_WRITE_D0HIH(v)	(*(volatile uint16*)IRRC_REG_D0HIH	= (v))
#define	IRRC_WRITE_D1LOL(v)	(*(volatile uint16*)IRRC_REG_D1LOL	= (v))
#define	IRRC_WRITE_D1LOH(v)	(*(volatile uint16*)IRRC_REG_D1LOH	= (v))
#define	IRRC_WRITE_D1HIL(v)	(*(volatile uint16*)IRRC_REG_D1HIL	= (v))
#define	IRRC_WRITE_D1HIH(v)	(*(volatile uint16*)IRRC_REG_D1HIH	= (v))
#define	IRRC_WRITE_ENDLENL(v)	(*(volatile uint16*)IRRC_REG_ENDLENL	= (v))
#define	IRRC_WRITE_ENDLENH(v)	(*(volatile uint16*)IRRC_REG_ENDLENH	= (v))
#define	IRRC_WRITE_BITLEN(v)	(*(volatile uint16*)IRRC_REG_BITLEN	= (v))
#define	IRRC_WRITE_FRMLENL(v)	(*(volatile uint16*)IRRC_REG_FRMLENL	= (v))
#define	IRRC_WRITE_FRMLENH(v)	(*(volatile uint16*)IRRC_REG_FRMLENH	= (v))
#define	IRRC_WRITE_OUT(u,v)	(*(volatile uint16*)(IRRC_REG_OUT0+2*(u)) \
									= (v))
#define	IRRC_WRITE_SEND(v)	(*(volatile uint16*)IRRC_REG_SEND	= (v))
#define	IRRC_WRITE_REGS(v)	(*(volatile uint16*)IRRC_REG_REGS	= (v))
#define	IRRC_WRITE_DBG(v)	(*(volatile uint16*)IRRC_REG_DBG	= (v))
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_WRITE_INTSTAT(v)	(*(volatile uint16*)IRRC_REG_INTSTAT	= (v))
#define	IRRC_WRITE_INTMASK(v)	(*(volatile uint16*)IRRC_REG_INTMASK	= (v))
#endif

#ifdef CONFIG_SHARP_INFRARED_LR388G7
#define	INTR_IRRCINT		(uint16)0x0004
#define	INTR_IRRC2INT		(uint16)0x0008
#endif
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	INTR0_IRRCINT		(uint16)0x0100
#endif

#define	IRRC_SYS_INIT		(uint16)0x0110
#define	IRRC_BASE_INIT		(uint16)0x0000
#define	IRRC_CLO_INIT		(uint16)0x0119
#define	IRRC_CHI_INIT		(uint16)0x008F
#define	IRRC_HLOL_INIT		(uint16)0x00AB
#define	IRRC_HLOH_INIT		(uint16)0x0000
#define	IRRC_HLO_INIT		(uint32)0x000000AB
#define	IRRC_HHIL_INIT		(uint16)0x0158
#define	IRRC_HHIH_INIT		(uint16)0x0000
#define	IRRC_HHI_INIT		(uint32)0x00000158
#define	IRRC_D0LOL_INIT 	(uint16)0x0014
#define	IRRC_D0LOH_INIT		(uint16)0x0000
#define	IRRC_D0LO_INIT		(uint32)0x00000014
#define	IRRC_D0HIL_INIT		(uint16)0x0014
#define	IRRC_D0HIH_INIT		(uint16)0x0000
#define	IRRC_D0HI_INIT		(uint32)0x00000014
#define	IRRC_D1LOL_INIT		(uint16)0x0014
#define	IRRC_D1LOH_INIT		(uint16)0x0000
#define	IRRC_D1LO_INIT		(uint32)0x00000014
#define	IRRC_D1HIL_INIT		(uint16)0x003C
#define	IRRC_D1HIH_INIT		(uint16)0x0000
#define	IRRC_D1HI_INIT		(uint32)0x0000003C
#define	IRRC_ENDLENL_INIT	(uint16)0x0014
#define	IRRC_ENDLENH_INIT	(uint16)0x0000
#define	IRRC_ENDLEN_INIT	(uint32)0x00000014
#define	IRRC_BITLEN_INIT	(uint16)0x0020
#define	IRRC_FRMLENH_INIT	(uint16)0x0000
#define	IRRC_FRMLENL_INIT	(uint16)0x0000
#define	IRRC_FRMLEN_INIT	(uint32)0x00000000
#define	IRRC_OUT_INIT		(uint16)0x0000
#define	IRRC_SEND_INIT		(uint16)0x0000
#define	IRRC_REGS_INIT		(uint16)0x0000
#define	IRRC_DBG_INIT		(uint16)0x0000
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_INTSTAT_INIT	(uint16)0x0000
#define	IRRC_INTMASK_INIT	(uint16)0x0000
#endif

#define IRRC_REG_OUTLEN_MAX	0x0080
#define IRRC_OUT_REG_MAX	8

#define	IRRC_PWR_OFF		(uint16)0xFFFE
#define	IRRC_PWR_ON		(uint16)0x0001
#define	IRRC_INV0		(uint16)0x0002
#define	IRRC_INV1		(uint16)0x0004
#define	IRRC_DIVS		(uint16)0x0010
#define	IRRC_OPM		(uint16)0x0020
#define	IRRC_RPT0		(uint16)0x0100
#define	IRRC_RPT_AREA		(uint16)0x0F00
#define	IRRC_FRME		(uint16)0x1000
#define	IRRC_FRMB		(uint16)0x2000
#define	IRRC_RESET		(uint16)0x8000
#define	IRRC_RESET_CLR		(uint16)0x7FFF
#define	IRRC_SEND		(uint16)0x0001
#define	IRRC_REGS_NOT_REPEAT	(uint16)0x0000
#define	IRRC_REGS_REPEAT	(uint16)0x0001
#define	IRRC_DBG_VER2		(uint16)0x0001
#ifdef CONFIG_SHARP_INFRARED_LR388J5
#define	IRRC_INTSTAT_INT0	(uint16)0x0001
#define	IRRC_INTSTAT_INT1	(uint16)0x0002
#define	IRRC_INTMASK_INT0	(uint16)0x0001
#define	IRRC_INTMASK_INT1	(uint16)0x0002
#endif

#ifdef CONFIG_SHARP_INFRARED_LR388J5
typedef enum {
	IRRC_INT_NONE = 0,
	IRRC_INT_DATASET,
	IRRC_INT_SENDCOMP,
} TYPE_IRRC_INT_FACTOR;
#endif

int irrc_reg_ioremap_nocache(void);

void irrc_reg_iounmap(void);

void irrc_reg_set_bitlen(int32 len);

void irrc_reg_send(void);

void irrc_reg_clear_int_irrc_factor(void);

#ifdef CONFIG_SHARP_INFRARED_LR388G7
void irrc_reg_clear_int_irrc2_factor(void);
#endif

void irrc_reg_set_data0(
   int32 pulse0_low, int32 pulse0_high);

void irrc_reg_set_data1(
   int32 pulse1_low, int32 pulse1_high);

void irrc_reg_set_leader(
	int32 leader_low, int32 leader_high);

void irrc_reg_set_trailer(
		int32 trailer_high);

void irrc_reg_set_frame_length(
		int32 frame_length);

void irrc_reg_set_sys(void);

void irrc_reg_set_modulation(
	int16 modulation0, int16 modulation1);

void irrc_reg_set_data(uint16 *reg_data);

void irrc_reg_set_regs(uint16 val);

void irrc_reg_set_dbg(void);

void irrc_reg_set_sys_pwron(void);

void irrc_reg_set_sys_pwroff(void);

void irrc_reg_set_base(void);

void irrc_reg_set_carrier(
  int32 carrier_high, int32 carrier_low);

void irrc_reg_set_init(void);

void irrc_reg_init(void);

void irrc_reg_sys_reset(void);

void irrc_reg_set_irrcdiv(void);

#ifdef CONFIG_SHARP_INFRARED_LR388J5
void irrc_reg_clear_irrcintstat_mask(void);

void irrc_reg_set_irrcintmask(TYPE_IRRC_INT_FACTOR factor);

TYPE_IRRC_INT_FACTOR irrc_reg_get_int_factor(void);
#endif
#endif
