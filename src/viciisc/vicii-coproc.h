/*
 * vicii-coproc.h - 6502 co-processor executing from extended video RAM.
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

#ifndef VICE_VICII_COPROC_H
#define VICE_VICII_COPROC_H

#include "types.h"

/* Reset all co-processor state. Called from vicii_reset(). */
void vicii_coproc_reset(void);

/* Advance the co-processor by one VIC cycle. Called from vicii_cycle(). */
void vicii_coproc_step(void);

/* Extra register overlay 0x20-0x2b. idx is the overlay address, matching
   EXT_REG_CPU_* in the Kawari hdl/common.vh. */
uint8_t vicii_coproc_read_reg(int idx);
void vicii_coproc_write_reg(int idx, uint8_t value);

#endif
