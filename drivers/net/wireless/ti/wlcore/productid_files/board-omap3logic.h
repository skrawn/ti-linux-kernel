/*
 * linux/arch/arm/plat-omap/include/plat/board-omap3logic.h
 *
 * Copyright (C) 2011 Logic Product Devleopment, Inc.
 *
 * Initial code: Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern volatile int omap3logic_hang;
extern int __init omap3logic_nor_init(u32 nor_cs_mask, unsigned long nor_size);
extern int __init omap3logic_nand_init(void);

extern void omap3logic_init_twl_audio(void);
extern int omap3logic_wl12xx_exists(void);

extern void omap3logic_init_productid_specifics(void);
