/*
 * vicii-mem.h - Memory interface for the MOS6569 (VIC-II) emulation.
 *
 * Written by
 *  Ettore Perazzoli <ettore@comm2000.it>
 *  Andreas Boose <viceteam@t-online.de>
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

#ifndef VICE_VICII_MEM_H
#define VICE_VICII_MEM_H

#include "types.h"

extern unsigned char extraRegs[64];
extern unsigned char extraMem[65536];
extern unsigned char overlayMem[256];

extern void vicii_store(uint16_t addr, uint8_t value);
extern void vicii_poke(uint16_t addr, uint8_t value);
extern uint8_t vicii_read(uint16_t addr);
extern uint8_t vicii_peek(uint16_t addr);
extern void vicii_mem_vbank_store(uint16_t addr, uint8_t value);
extern void vicii_mem_vbank_39xx_store(uint16_t addr, uint8_t value);
extern void vicii_mem_vbank_3fxx_store(uint16_t addr, uint8_t value);
extern void vicii_palette_store(uint16_t addr, uint8_t value);
extern uint8_t vicii_palette_read(uint16_t addr);
extern int vicii_extended_regs(void);
extern void viciidtv_update_colorram(void);


extern void set_flash_file_name(char* fname);
extern void set_fpga_flash_file_name(char* fname);
extern void handle_color_change(uint8_t reg, uint8_t value);
extern void do_copy(void);
extern void do_fill(void);
extern void do_flash(void);
extern void do_dma_xfer(void);
extern void do_blit(void);
#endif
