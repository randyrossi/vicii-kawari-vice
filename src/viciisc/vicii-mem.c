/*
 * vicii-mem.c - Memory interface for the MOS6569 (VIC-II) emulation.
 *
 * Written by
 *  Ettore Perazzoli <ettore@comm2000.it>
 *  Andreas Boose <viceteam@t-online.de>
 *  Daniel Kahlin <daniel@kahlin.net>
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

#include "vice.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug.h"
#include "types.h"
#include "mem.h"
#include "vicii-chip-model.h"
#include "vicii-color.h"
#include "vicii-draw-cycle.h"
#include "vicii-fetch.h"
#include "vicii-irq.h"
#include "vicii-resources.h"
#include "vicii-mem.h"
#include "vicii.h"
#include "viciitypes.h"

static char* flash_file_name = NULL;
static int extra_regs_activated = 0;
static int extra_regs_activation_counter = 0;
unsigned char extraRegs[64];
unsigned char extraMem[65536];
unsigned char overlayMem[256];
static unsigned char u_op_1_hi;
static unsigned char u_op_1_lo;
static unsigned char u_op_2_hi;
static unsigned char u_op_2_lo;
static unsigned char divzero;
static unsigned short u1;
static unsigned short u2;
static signed short s1;
static signed short s2;
static unsigned short uquotient;
static unsigned short uremain;
static signed short squotient;
static signed short sremain;
static unsigned long uresult;
static signed long sresult;
static uint8_t pixels_per_byte = 2;

static uint8_t blit_flags;
static uint8_t blit_done = 1;
static uint16_t blit_width;
static uint16_t blit_height;
static uint16_t blit_src_ptr;
static uint8_t blit_src_x;
static uint8_t blit_src_stride;
static uint16_t blit_dst_ptr;
static uint8_t blit_dst_x;
static uint8_t blit_dst_stride;
static uint8_t blit_state;
static uint8_t blit_init;

static uint16_t blit_src_cur;
static uint16_t blit_dst_cur;
static uint8_t blit_d;
static uint8_t blit_s;
static uint8_t blit_o;
static uint16_t blit_dst_pos;
static uint16_t blit_src_pos;
static uint16_t blit_line;
static uint8_t blit_dst_align;
static uint8_t blit_src_align;
static uint8_t blit_src_avail;
static uint8_t blit_dst_avail;
static uint8_t blit_out_avail;
static uint16_t blit_pixels_written;
static uint16_t blit_tmp_addr;

static uint8_t dma_op = 0;
static int copy_idx;
static uint16_t copy_num;
static uint16_t copy_dest;
static uint16_t copy_src;
static uint16_t fill_start;
static uint16_t fill_num;
static uint8_t fill_b;
static uint16_t fill_idx;

static void handle_eeprom_save(int reg, int value);

/* Unused bits in VIC-II registers: these are always 1 when read.  */
static const uint8_t unused_bits_in_registers[0x40] =
{
    0x00 /* $D000 */, 0x00 /* $D001 */, 0x00 /* $D002 */, 0x00 /* $D003 */,
    0x00 /* $D004 */, 0x00 /* $D005 */, 0x00 /* $D006 */, 0x00 /* $D007 */,
    0x00 /* $D008 */, 0x00 /* $D009 */, 0x00 /* $D00A */, 0x00 /* $D00B */,
    0x00 /* $D00C */, 0x00 /* $D00D */, 0x00 /* $D00E */, 0x00 /* $D00F */,
    0x00 /* $D010 */, 0x00 /* $D011 */, 0x00 /* $D012 */, 0x00 /* $D013 */,
    0x00 /* $D014 */, 0x00 /* $D015 */, 0xc0 /* $D016 */, 0x00 /* $D017 */,
    0x01 /* $D018 */, 0x70 /* $D019 */, 0xf0 /* $D01A */, 0x00 /* $D01B */,
    0x00 /* $D01C */, 0x00 /* $D01D */, 0x00 /* $D01E */, 0x00 /* $D01F */,
    0xf0 /* $D020 */, 0xf0 /* $D021 */, 0xf0 /* $D022 */, 0xf0 /* $D023 */,
    0xf0 /* $D024 */, 0xf0 /* $D025 */, 0xf0 /* $D026 */, 0xf0 /* $D027 */,
    0xf0 /* $D028 */, 0xf0 /* $D029 */, 0xf0 /* $D02A */, 0xf0 /* $D02B */,
    0xf0 /* $D02C */, 0xf0 /* $D02D */, 0xf0 /* $D02E */, 0xff /* $D02F */,
    0xff /* $D030 */, 0xff /* $D031 */, 0xff /* $D032 */, 0xff /* $D033 */,
    0xff /* $D034 */, 0xff /* $D035 */, 0xff /* $D036 */, 0xff /* $D037 */,
    0xff /* $D038 */, 0xff /* $D039 */, 0xff /* $D03A */, 0xff /* $D03B */,
    0xff /* $D03C */, 0xff /* $D03D */, 0xff /* $D03E */, 0xff    /* $D03F */
};


/* FIXME plus60k/256k needs these for now */
inline static void vicii_local_store_vbank(uint16_t addr, uint8_t value)
{
    vicii.ram_base_phi2[addr] = value;
}

void vicii_mem_vbank_store(uint16_t addr, uint8_t value)
{
    vicii_local_store_vbank(addr, value);
}

void vicii_mem_vbank_39xx_store(uint16_t addr, uint8_t value)
{
    vicii_local_store_vbank(addr, value);
}

void vicii_mem_vbank_3fxx_store(uint16_t addr, uint8_t value)
{
    vicii_local_store_vbank(addr, value);
}


inline static void store_sprite_x_position_lsb(const uint16_t addr, uint8_t value)
{
    int n;

    if (value == vicii.regs[addr]) {
        return;
    }

    vicii.regs[addr] = value;
    n = addr >> 1;                /* Number of changed sprite.  */

    VICII_DEBUG_REGISTER(("Sprite #%d X position LSB: $%02X", n, value));

    vicii.sprite[n].x = (value | (vicii.regs[0x10] & (1 << n) ? 0x100 : 0));
}

inline static void store_sprite_y_position(const uint16_t addr, uint8_t value)
{
    vicii.regs[addr] = value;
}

static inline void store_sprite_x_position_msb(const uint16_t addr, uint8_t value)
{
    int i;
    uint8_t b;

    VICII_DEBUG_REGISTER(("Sprite X position MSBs: $%02X", value));

    if (value == vicii.regs[addr]) {
        return;
    }

    vicii.regs[addr] = value;

    /* Recalculate the sprite X coordinates.  */
    for (i = 0, b = 0x01; i < 8; b <<= 1, i++) {
        vicii.sprite[i].x = (vicii.regs[2 * i] | (value & b ? 0x100 : 0));
    }
}

inline static void update_raster_line(void)
{
    unsigned int new_line;

    new_line = vicii.regs[0x12];
    new_line |= (vicii.regs[0x11] & 0x80) << 1;

    vicii.raster_irq_line = new_line;

    VICII_DEBUG_REGISTER(("Raster interrupt line set to $%04X",
                          vicii.raster_irq_line));
}

inline static void d011_store(uint8_t value)
{
    VICII_DEBUG_REGISTER(("Control register: $%02X", value));
    VICII_DEBUG_REGISTER(("$D011 tricks at cycle %d, line $%04X, "
                          "value $%02X", vicii.raster_cycle, vicii.raster_line, value));

    vicii.ysmooth = value & 0x7;

    vicii.regs[0x11] = value;

    update_raster_line();
}

inline static void d012_store(uint8_t value)
{
    VICII_DEBUG_REGISTER(("Raster compare register: $%02X", value));

    if (value == vicii.regs[0x12]) {
        return;
    }

    vicii.regs[0x12] = value;

    update_raster_line();
}

inline static void d015_store(const uint8_t value)
{
    vicii.regs[0x15] = value;
}

inline static void d016_store(const uint8_t value)
{
    VICII_DEBUG_REGISTER(("Control register: $%02X", value));

    vicii.regs[0x16] = value;
}

inline static void d017_store(const uint8_t value)
{
    int i;
    uint8_t b;

    VICII_DEBUG_REGISTER(("Sprite Y Expand register: $%02X", value));

    if (value == vicii.regs[0x17]) {
        return;
    }

    for (i = 0, b = 0x01; i < 8; b <<= 1, i++) {
        if (!(value & b) && !vicii.sprite[i].exp_flop) {
            /* sprite crunch */
            /* if (cycle == VICII_PAL_CYCLE(15))) { */
            if (cycle_is_check_spr_crunch(vicii.cycle_flags)) {
                uint8_t mc = vicii.sprite[i].mc;
                uint8_t mcbase = vicii.sprite[i].mcbase;

                /* 0x2a = 0b101010
                   0x15 = 0b010101 */
                vicii.sprite[i].mc = (0x2a & (mcbase & mc)) | (0x15 & (mcbase | mc));

                /* mcbase is set from mc on the following vicii_cycle() call */
            }

            vicii.sprite[i].exp_flop = 1;
        }
    }

    vicii.regs[0x17] = value;
}

inline static void d018_store(const uint8_t value)
{
    VICII_DEBUG_REGISTER(("Memory register: $%02X", value));

    if (vicii.regs[0x18] == value) {
        return;
    }

    vicii.regs[0x18] = value;
}

inline static void d019_store(const uint8_t value)
{
    vicii.irq_status &= ~((value & 0xf) | 0x80);
    vicii_irq_set_line();

    VICII_DEBUG_REGISTER(("IRQ flag register: $%02X", vicii.irq_status));
}

inline static void d01a_store(const uint8_t value)
{
    vicii.regs[0x1a] = value & 0xf;

    vicii_irq_set_line();

    VICII_DEBUG_REGISTER(("IRQ mask register: $%02X", vicii.regs[0x1a]));
}

inline static void d01b_store(const uint8_t value)
{
    VICII_DEBUG_REGISTER(("Sprite priority register: $%02X", value));

    vicii.regs[0x1b] = value;
}

inline static void d01c_store(const uint8_t value)
{
    VICII_DEBUG_REGISTER(("Sprite Multicolor Enable register: $%02X", value));

    vicii.regs[0x1c] = value;
}

inline static void d01d_store(const uint8_t value)
{
    VICII_DEBUG_REGISTER(("Sprite X Expand register: $%02X", value));

    vicii.regs[0x1d] = value;
}

inline static void collision_store(const uint16_t addr, const uint8_t value)
{
    VICII_DEBUG_REGISTER(("(collision register, Read Only)"));
}

inline static void color_reg_store(uint16_t addr, uint8_t value)
{
    vicii.regs[addr] = value;
    vicii.last_color_reg = (uint8_t)(addr);
    vicii.last_color_value = value;
}

inline static void d020_store(uint8_t value)
{
    VICII_DEBUG_REGISTER(("Border color register: $%02X", value));

    value &= 0x0f;

    color_reg_store(0x20, value);
}

inline static void d021_store(uint8_t value)
{
    VICII_DEBUG_REGISTER(("Background #0 color register: $%02X", value));

    value &= 0x0f;

    color_reg_store(0x21, value);
}

inline static void ext_background_store(uint16_t addr, uint8_t value)
{
    value &= 0x0f;

    VICII_DEBUG_REGISTER(("Background color #%d register: $%02X",
                          addr - 0x21, value));

    color_reg_store(addr, value);
}

inline static void d025_store(uint8_t value)
{
    value &= 0xf;

    VICII_DEBUG_REGISTER(("Sprite multicolor register #0: $%02X", value));

    color_reg_store(0x25, value);
}

inline static void d026_store(uint8_t value)
{
    value &= 0xf;

    VICII_DEBUG_REGISTER(("Sprite multicolor register #1: $%02X", value));

    color_reg_store(0x26, value);
}

inline static void sprite_color_store(uint16_t addr, uint8_t value)
{
    value &= 0xf;

    VICII_DEBUG_REGISTER(("Sprite #%d color register: $%02X",
                          addr - 0x27, value));

    color_reg_store(addr, value);
}

// Need to cheat a bit here
extern CLOCK maincpu_clk;
CLOCK last_auto = 0;
// Kawawri: Handle auto increment rules for two pointer regs
static void autoincdec(int fl, int reg)
{
    if (maincpu_clk - last_auto < 2)
       return;

    last_auto = maincpu_clk;
    if (fl == 1) {
           extraRegs[reg] = (extraRegs[reg] + 1) & 0xff;
           if (extraRegs[reg] == 0) {
                   extraRegs[reg+1] = (extraRegs[reg+1] + 1) & 0xff;
           }
    }
    else if (fl == 2) {
           extraRegs[reg] = (extraRegs[reg] - 1) & 0xff;
           if (extraRegs[reg] == 0) {
                   extraRegs[reg+1] = (extraRegs[reg+1] - 1) & 0xff;
           }
    }
}


static void handle_dma(int fl, int value) {
   if (fl == 3) {
        if (value == 1) { // block copy low to high
           dma_op = 1;
           copy_num = extraRegs[0x36]*256+extraRegs[0x35];
           copy_idx = 0;
           copy_dest = extraRegs[0x3a]*256+extraRegs[0x39];
           copy_src = extraRegs[0x3d]*256+extraRegs[0x3c];
        } else if (value == 2) { // block copy high to low
           dma_op = 2;
           copy_num = extraRegs[0x36]*256+extraRegs[0x35];
           copy_idx = copy_num - 1;
           copy_dest = extraRegs[0x3a]*256+extraRegs[0x39];
           copy_src = extraRegs[0x3d]*256+extraRegs[0x3c];
        } else if (value == 4) { // fill
           dma_op = 4;
           fill_num = extraRegs[0x36]*256+extraRegs[0x35];
           fill_start = extraRegs[0x3a]*256+extraRegs[0x39];
           fill_b = extraRegs[0x3c];
           fill_idx = fill_start;
        } else if (value == 8) { // dma DRAM to VMEM
           dma_op = 8;
           copy_src = extraRegs[0x3d]*256+extraRegs[0x3c];
           copy_dest = extraRegs[0x3a]*256+extraRegs[0x39];
           copy_num = extraRegs[0x36]*256+extraRegs[0x35];
           copy_idx = 0;
        } else if (value == 16) { // dma VMEM to DRAM
           dma_op = 16;
           copy_src = extraRegs[0x3d]*256+extraRegs[0x3c];
           copy_dest = extraRegs[0x3a]*256+extraRegs[0x39];
           copy_num = extraRegs[0x36]*256+extraRegs[0x35];
           copy_idx = 0;
        } else if (value == 32) { // Set Blitter SRC
           blit_width = (u_op_1_hi << 8) | u_op_1_lo;
           blit_width &= 0x3ff;
           blit_height = (u_op_2_hi << 8) | u_op_2_lo;
           blit_height &= 0x3ff;
           blit_src_ptr = ((extraRegs[0x35] << 8) | extraRegs[0x36]) +
                          (((extraRegs[0x3a] << 8) | extraRegs[0x39]) / pixels_per_byte) +
                             extraRegs[0x3c] * extraRegs[0x3d];
           blit_src_x = extraRegs[0x39] & 0x3;
           blit_src_stride = extraRegs[0x3d];
        } else if (value == 64) { // Set Blitter DST & Execute
           blit_flags = u_op_1_hi;
           blit_dst_ptr = ((extraRegs[0x35] << 8) | extraRegs[0x36]) +
              (((extraRegs[0x3a] << 8) | extraRegs[0x39]) / pixels_per_byte) +
                 extraRegs[0x3c] * extraRegs[0x3d];
           blit_dst_x = extraRegs[0x39] & 0x3;
           blit_dst_stride = extraRegs[0x3d];
           blit_done = 0;
           blit_state = 0;
           blit_init = 1;
        }
   } else {
        extraMem[extraRegs[0x39]+extraRegs[0x3a]*256+extraRegs[0x35]] = value;
        autoincdec(fl, 0x39);
   }
}

/* Store a value in a VIC-II register.  */
void vicii_store(uint16_t addr, uint8_t value)
{
    int fl;
    addr &= 0x3f;

    vicii.last_bus_phi2 = value;

    VICII_DEBUG_REGISTER(("WRITE $D0%02X at cycle %d of current_line $%04X",
                          addr, vicii.raster_cycle, vicii.raster_line));

    switch (addr) {
        case 0x0:                 /* $D000: Sprite #0 X position LSB */
        case 0x2:                 /* $D002: Sprite #1 X position LSB */
        case 0x4:                 /* $D004: Sprite #2 X position LSB */
        case 0x6:                 /* $D006: Sprite #3 X position LSB */
        case 0x8:                 /* $D008: Sprite #4 X position LSB */
        case 0xa:                 /* $D00a: Sprite #5 X position LSB */
        case 0xc:                 /* $D00c: Sprite #6 X position LSB */
        case 0xe:                 /* $D00e: Sprite #7 X position LSB */
            store_sprite_x_position_lsb(addr, value);
            break;

        case 0x1:                 /* $D001: Sprite #0 Y position */
        case 0x3:                 /* $D003: Sprite #1 Y position */
        case 0x5:                 /* $D005: Sprite #2 Y position */
        case 0x7:                 /* $D007: Sprite #3 Y position */
        case 0x9:                 /* $D009: Sprite #4 Y position */
        case 0xb:                 /* $D00B: Sprite #5 Y position */
        case 0xd:                 /* $D00D: Sprite #6 Y position */
        case 0xf:                 /* $D00F: Sprite #7 Y position */
            store_sprite_y_position(addr, value);
            break;

        case 0x10:                /* $D010: Sprite X position MSB */
            store_sprite_x_position_msb(addr, value);
            break;

        case 0x11:                /* $D011: video mode, Y scroll, 24/25 line
                                     mode and raster MSB */
            d011_store(value);
            break;

        case 0x12:                /* $D012: Raster line compare */
            d012_store(value);
            break;

        case 0x13:                /* $D013: Light Pen X */
        case 0x14:                /* $D014: Light Pen Y */
            break;

        case 0x15:                /* $D015: Sprite Enable */
            d015_store(value);
            break;

        case 0x16:                /* $D016 */
            d016_store(value);
            break;

        case 0x17:                /* $D017: Sprite Y-expand */
            d017_store(value);
            break;

        case 0x18:                /* $D018: Video and char matrix base
                                     address */
            d018_store(value);
            break;

        case 0x19:                /* $D019: IRQ flag register */
            d019_store(value);
            break;

        case 0x1a:                /* $D01A: IRQ mask register */
            d01a_store(value);
            break;

        case 0x1b:                /* $D01B: Sprite priority */
            d01b_store(value);
            break;

        case 0x1c:                /* $D01C: Sprite Multicolor select */
            d01c_store(value);
            break;

        case 0x1d:                /* $D01D: Sprite X-expand */
            d01d_store(value);
            break;

        case 0x1e:                /* $D01E: Sprite-sprite collision */
        case 0x1f:                /* $D01F: Sprite-background collision */
            collision_store(addr, value);
            break;

        case 0x20:                /* $D020: Border color */
            d020_store(value);
            break;

        case 0x21:                /* $D021: Background #0 color */
            d021_store(value);
            break;

        case 0x22:                /* $D022: Background #1 color */
        case 0x23:                /* $D023: Background #2 color */
        case 0x24:                /* $D024: Background #3 color */
            ext_background_store(addr, value);
            break;

        case 0x25:                /* $D025: Sprite multicolor register #0 */
            d025_store(value);
            break;

        case 0x26:                /* $D026: Sprite multicolor register #1 */
            d026_store(value);
            break;

        case 0x27:                /* $D027: Sprite #0 color */
        case 0x28:                /* $D028: Sprite #1 color */
        case 0x29:                /* $D029: Sprite #2 color */
        case 0x2a:                /* $D02A: Sprite #3 color */
        case 0x2b:                /* $D02B: Sprite #4 color */
        case 0x2c:                /* $D02C: Sprite #5 color */
        case 0x2d:                /* $D02D: Sprite #6 color */
        case 0x2e:                /* $D02E: Sprite #7 color */
            sprite_color_store(addr, value);
            break;

        case 0x2f:                /* $D02F: Unused */
            u_op_1_hi = value;
            break;
        case 0x30:                /* $D030: Unused */
            u_op_1_lo = value;
            break;
        case 0x31:                /* $D031: Unused */
            u_op_2_hi = value;
            break;
        case 0x32:                /* $D032: Unused */
            u_op_2_lo = value;
            break;
        case 0x33:                /* $D033: Unused */
            u1 = (u_op_1_hi * 256 + u_op_1_lo);
            u2 = (u_op_2_hi * 256 + u_op_2_lo);
            s1 = u1;
            s2 = u2;
            divzero = 0;
            if (value == 0) { // UMULT
               uresult  = u1 * u2;
               u_op_1_hi = (uresult >> 24) & 0xff;
               u_op_1_lo = (uresult >> 16) & 0xff;
               u_op_2_hi = (uresult >> 8) & 0xff;
               u_op_2_lo = (uresult) & 0xff;
            }
            else if (value == 1) { // UDIV
               if (u2 == 0) { divzero = 1; return; }
               uquotient  = u1 / u2;
               uremain  = u1 % u2;
               u_op_1_hi = (uremain >> 8) & 0xff;
               u_op_1_lo = (uremain) & 0xff;
               u_op_2_hi = (uquotient >> 8) & 0xff;
               u_op_2_lo = (uquotient) & 0xff;
            }
            else if (value == 2) { // SMULT
               sresult  = s1 * s2;
               u_op_1_hi = (sresult >> 24) & 0xff;
               u_op_1_lo = (sresult >> 16) & 0xff;
               u_op_2_hi = (sresult >> 8) & 0xff;
               u_op_2_lo = (sresult) & 0xff;
            }
            else if (value == 3) { // SDIV
               if (s2 == 0) { divzero = 1; return; }
               squotient  = s1 / s2;
               sremain  = s1 % s2;
               u_op_1_hi = (sremain >> 8) & 0xff;
               u_op_1_lo = (sremain) & 0xff;
               u_op_2_hi = (squotient >> 8) & 0xff;
               u_op_2_lo = (squotient) & 0xff;
            }
            break;
        case 0x34:                /* $D034: Unused */
        case 0x35:                /* $D035: Unused */
            if (extra_regs_activated)
               extraRegs[addr] = value;
            break;
        case 0x36:                /* $D036: Unused */
            if (extra_regs_activated)
               extraRegs[addr] = value;
            break;
        case 0x37:                /* $D037: Unused */
            if (extra_regs_activated) {
               vicii.hires_mode = (value & 0b11100000) >> 5;
               vicii.hires_enabled = (value & 0b00010000) >> 4;
               vicii.hires_allow_badlines = (value & 0b00001000) >> 3;
               vicii.hires_char_pixel_base = (value & 0b111);

               // This is changed with every change to hires_mode
               pixels_per_byte = vicii.hires_mode == 0b011 ? 4 : 2;
            }
            break;
        case 0x38:                /* $D038: Unused */
            if (extra_regs_activated) {
               vicii.hires_matrix_base = value & 0b1111;
               vicii.hires_color_base = (value & 0b11110000) >> 4;
            }
            break;
        case 0x39:                /* $D039: Unused */
            if (extra_regs_activated) {
               extraRegs[addr] = value;
            }
            break;
        case 0x3a:                /* $D03A: Unused */
            if (extra_regs_activated) {
               extraRegs[addr] = value;
            }
            break;
        case 0x3b:                /* $D03B: Unused */
            if (extra_regs_activated) {
              fl = (extraRegs[0x3f] & 0b11);

              if (extraRegs[0x3F] & 32) {
                 overlayMem[extraRegs[0x39]] = value;
                 handle_color_change(extraRegs[0x39], value);
                 handle_eeprom_save(extraRegs[0x39], value);
                 autoincdec(fl, 0x39);
              } else {
                 handle_dma(fl, value);
              }
            }
            break;
        case 0x3c:                /* $D03C: Unused */
           if (extra_regs_activated) {
               extraRegs[addr] = value;
            }
            break;
        case 0x3d:                /* $D03D: Unused */
            if (extra_regs_activated) {
              extraRegs[addr] = value;
            }
            break;
        case 0x3e:                /* $D03E: Unused */
            if (extra_regs_activated) {
              if (extraRegs[0x3F] & 32) {
                 overlayMem[extraRegs[0x3c]] = value;
                 handle_color_change(extraRegs[0x3c], value);
                 handle_eeprom_save(extraRegs[0x3c], value);
              } else {
                 extraMem[extraRegs[0x3c]+
                    extraRegs[0x3d]*256+extraRegs[0x36]] = value;
              }
              fl = (extraRegs[0x3f] & 0b1100) >> 2;
              autoincdec(fl, 0x3c);
            }
            break;
        case 0x3f:                /* $D03F: Unused */
            if (extra_regs_activated) {
                extraRegs[addr] = value;
            } else {
                if (value == 'V') {
                   extra_regs_activation_counter = 1;
                } else if (value == 'I' && extra_regs_activation_counter == 1) {
                   extra_regs_activation_counter++;
                } else if (value == 'C' && extra_regs_activation_counter == 2) {
                   extra_regs_activation_counter++;
                } else if (value == '2' && extra_regs_activation_counter == 3) {
                   extra_regs_activation_counter = 0;
                   extra_regs_activated = 1;
                } else {
                   extra_regs_activation_counter = 0;
                }
            }
            break;
    }
}

/* used by monitor when sfx off */
void vicii_poke(uint16_t addr, uint8_t value)
{
    addr &= 0x3f;
    if ((addr >= 0x20) && (addr <= 0x2e)) {
        vicii_monitor_colreg_store(addr, value);
        return;
    }
    vicii_store(addr, value);
}

/* Helper function for reading from $D011/$D012.  */
inline static unsigned int read_raster_y(void)
{
    int raster_y;

    raster_y = vicii.raster_line;

    return raster_y;
}

inline static uint8_t d01112_read(uint16_t addr)
{
    unsigned int tmp = read_raster_y();

    VICII_DEBUG_REGISTER(("Raster Line register %svalue = $%04X",
                          (addr == 0x11 ? "(highest bit) " : ""), tmp));
    if (addr == 0x11) {
        return (vicii.regs[addr] & 0x7f) | ((tmp & 0x100) >> 1);
    } else {
        return tmp & 0xff;
    }
}


inline static uint8_t d019_read(void)
{
    return vicii.irq_status | 0x70;
}

inline static uint8_t d01e_read(void)
{
    if (!vicii_resources.sprite_sprite_collisions_enabled) {
        VICII_DEBUG_REGISTER(("Sprite-sprite collision mask: $00 "
                              "(emulation disabled)"));
        vicii.sprite_sprite_collisions = 0;
        return 0;
    }

    vicii.regs[0x1e] = vicii.sprite_sprite_collisions;
    vicii.clear_collisions = 0x1e;
    VICII_DEBUG_REGISTER(("Sprite-sprite collision mask: $%02X",
                          vicii.regs[0x1e]));

    return vicii.regs[0x1e];
}

inline static uint8_t d01f_read(void)
{
    if (!vicii_resources.sprite_background_collisions_enabled) {
        VICII_DEBUG_REGISTER(("Sprite-background collision mask: $00 "
                              "(emulation disabled)"));
        vicii.sprite_background_collisions = 0;
        return 0;
    }

    vicii.regs[0x1f] = vicii.sprite_background_collisions;
    vicii.clear_collisions = 0x1f;
    VICII_DEBUG_REGISTER(("Sprite-background collision mask: $%02X",
                          vicii.regs[0x1f]));

#if defined (VICII_DEBUG_SB_COLLISIONS)
    log_message(vicii.log,
                "vicii.sprite_background_collisions reset by $D01F "
                "read at line 0x%X.",
                VICII_RASTER_Y(clk));
#endif

    return vicii.regs[0x1f];
}

/* Read a value from a VIC-II register.  */
uint8_t vicii_read(uint16_t addr)
{
    int fl;
    uint8_t value;
    addr &= 0x3f;

    VICII_DEBUG_REGISTER(("READ $D0%02X at cycle %d of current_line $%04X:",
                          addr, vicii.raster_cycle, vicii.raster_line));

    /* Note: we use hardcoded values instead of `unused_bits_in_registers[]'
       here because this is a little bit faster.  */
    switch (addr) {
        case 0x0:                 /* $D000: Sprite #0 X position LSB */
        case 0x2:                 /* $D002: Sprite #1 X position LSB */
        case 0x4:                 /* $D004: Sprite #2 X position LSB */
        case 0x6:                 /* $D006: Sprite #3 X position LSB */
        case 0x8:                 /* $D008: Sprite #4 X position LSB */
        case 0xa:                 /* $D00a: Sprite #5 X position LSB */
        case 0xc:                 /* $D00c: Sprite #6 X position LSB */
        case 0xe:                 /* $D00e: Sprite #7 X position LSB */
            VICII_DEBUG_REGISTER(("Sprite #%d X position LSB: $%02X",
                                  addr >> 1, vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x1:                 /* $D001: Sprite #0 Y position */
        case 0x3:                 /* $D003: Sprite #1 Y position */
        case 0x5:                 /* $D005: Sprite #2 Y position */
        case 0x7:                 /* $D007: Sprite #3 Y position */
        case 0x9:                 /* $D009: Sprite #4 Y position */
        case 0xb:                 /* $D00B: Sprite #5 Y position */
        case 0xd:                 /* $D00D: Sprite #6 Y position */
        case 0xf:                 /* $D00F: Sprite #7 Y position */
            VICII_DEBUG_REGISTER(("Sprite #%d Y position: $%02X",
                                  addr >> 1, vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x10:                /* $D010: Sprite X position MSB */
            VICII_DEBUG_REGISTER(("Sprite X position MSB: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x11:              /* $D011: video mode, Y scroll, 24/25 line mode
                                   and raster MSB */
        case 0x12:              /* $D012: Raster line compare */
            value = d01112_read(addr);
            break;

        case 0x13:                /* $D013: Light Pen X */
            VICII_DEBUG_REGISTER(("Light pen X: %d", vicii.light_pen.x));
            value = vicii.light_pen.x;
            break;

        case 0x14:                /* $D014: Light Pen Y */
            VICII_DEBUG_REGISTER(("Light pen Y: %d", vicii.light_pen.y));
            value = vicii.light_pen.y;
            break;

        case 0x15:                /* $D015: Sprite Enable */
            VICII_DEBUG_REGISTER(("Sprite Enable register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x16:                /* $D016 */
            VICII_DEBUG_REGISTER(("$D016 Control register read: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr] | 0xc0;
            break;

        case 0x17:                /* $D017: Sprite Y-expand */
            VICII_DEBUG_REGISTER(("Sprite Y Expand register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x18:              /* $D018: Video and char matrix base address */
            VICII_DEBUG_REGISTER(("Video memory address register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr] | 0x1;
            break;

        case 0x19:                /* $D019: IRQ flag register */
            value = d019_read();
            VICII_DEBUG_REGISTER(("Interrupt register: $%02X", value));
            break;

        case 0x1a:                /* $D01A: IRQ mask register  */
            value = vicii.regs[addr] | 0xf0;
            VICII_DEBUG_REGISTER(("Mask register: $%02X", value));
            break;

        case 0x1b:                /* $D01B: Sprite priority */
            VICII_DEBUG_REGISTER(("Sprite Priority register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x1c:                /* $D01C: Sprite Multicolor select */
            VICII_DEBUG_REGISTER(("Sprite Multicolor Enable register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x1d:                /* $D01D: Sprite X-expand */
            VICII_DEBUG_REGISTER(("Sprite X Expand register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr];
            break;

        case 0x1e:                /* $D01E: Sprite-sprite collision */
            value = d01e_read();
            break;

        case 0x1f:                /* $D01F: Sprite-background collision */
            value = d01f_read();
            break;

        case 0x20:                /* $D020: Border color */
            VICII_DEBUG_REGISTER(("Border Color register: $%02X",
                                  vicii.regs[addr]));
            value = vicii.regs[addr] | 0xf0;
            break;

        case 0x21:                /* $D021: Background #0 color */
        case 0x22:                /* $D022: Background #1 color */
        case 0x23:                /* $D023: Background #2 color */
        case 0x24:                /* $D024: Background #3 color */
            VICII_DEBUG_REGISTER(("Background Color #%d register: $%02X",
                                  addr - 0x21, vicii.regs[addr]));
            value = vicii.regs[addr] | 0xf0;
            break;

        case 0x25:                /* $D025: Sprite multicolor register #0 */
        case 0x26:                /* $D026: Sprite multicolor register #1 */
            VICII_DEBUG_REGISTER(("Multicolor register #%d: $%02X",
                                  addr - 0x22, vicii.regs[addr]));
            value = vicii.regs[addr] | 0xf0;
            break;

        case 0x27:                /* $D027: Sprite #0 color */
        case 0x28:                /* $D028: Sprite #1 color */
        case 0x29:                /* $D029: Sprite #2 color */
        case 0x2a:                /* $D02A: Sprite #3 color */
        case 0x2b:                /* $D02B: Sprite #4 color */
        case 0x2c:                /* $D02C: Sprite #5 color */
        case 0x2d:                /* $D02D: Sprite #6 color */
        case 0x2e:                /* $D02E: Sprite #7 color */
            VICII_DEBUG_REGISTER(("Sprite #%d color: $%02X",
                                  addr - 0x22, vicii.regs[addr]));
            value = vicii.regs[addr] | 0xf0;
            break;

        // Kawari: Extra regs PEEK handled here.
        case 0x2f:
            value = u_op_1_hi;
            break;
        case 0x30:
            value = u_op_1_lo;
            break;
        case 0x31:
            value = u_op_2_hi;
            break;
        case 0x32:
            value = u_op_2_lo;
            break;
        case 0x33:
            value = divzero;
            break;
        case 0x34:
            value = 0x0; // Use 0x04 to fake bad verify
            break;
        case 0x35: // ptr 1 idx
            if (extra_regs_activated)
               value = extraRegs[addr];
            else
               value = 0xff;
            break;
        case 0x36: // ptr 2 idx
            if (extra_regs_activated)
               value = extraRegs[addr];
            else
               value = 0xff;
            break;
        case 0x37: // vmode 1
            if (extra_regs_activated)
               value = (vicii.hires_mode << 5) | (vicii.hires_enabled << 4) |
                    (vicii.hires_allow_badlines << 3) | vicii.hires_char_pixel_base;
            else
               value = 0xff;
            break;
        case 0x38: // vmode 2
            if (extra_regs_activated)
               value = vicii.hires_matrix_base | (vicii.hires_color_base << 4);
            else
               value = 0xff;
            break;
        case 0x39: // ptr 1 lo
            if (extra_regs_activated)
               value = extraRegs[addr];
            else
               value = 0xff;
            break;
        case 0x3a: // ptr 1 hi
            if (extra_regs_activated)
               value = extraRegs[addr];
            else
               value = 0xff;
            break;
        case 0x3b: // ptr 1 value
            if (extra_regs_activated) {
               if (extraRegs[0x3F] & 32) {
                  value = overlayMem[extraRegs[0x39]];
               } else {
                  value = extraMem[extraRegs[0x39]+
                    extraRegs[0x3a]*256+extraRegs[0x35]];
               }
               fl = (extraRegs[0x3f] & 0b11);
               autoincdec(fl, 0x39);
            }
            else
               value = 0xff;
            break;
        case 0x3c: // ptr 2 lo
            if (extra_regs_activated)
               value = extraRegs[addr];
            else
                value = 0xff;
            break;
        case 0x3d: // ptr 2 hi
            if (extra_regs_activated)
               value = extraRegs[addr];
            else
                value = 0xff;
            break;
        case 0x3e: // ptr 2 value
            if (extra_regs_activated) {
               if (extraRegs[0x3F] & 32) {
                  value = overlayMem[extraRegs[0x3c]];
               } else {
                  value = extraMem[extraRegs[0x3c]+
                    extraRegs[0x3d]*256+extraRegs[0x36]];
               }
               fl = (extraRegs[0x3f] & 0b1100) >> 2;
               autoincdec(fl, 0x3c);
            }
            else
                value = 0xff;
            break;

        default:
            if (extra_regs_activated)
                value = extraRegs[addr];
            else
                value = 0xff;
            break;
    }

    vicii.last_bus_phi2 = value;
    return value;
}

inline static uint8_t d019_peek(void)
{
    return vicii.irq_status | 0x70;
}

uint8_t vicii_peek(uint16_t addr)
{
    addr &= 0x3f;

    switch (addr) {
        case 0x11:            /* $D011: video mode, Y scroll, 24/25 line mode
                                 and raster MSB */
            return (vicii.regs[addr] & 0x7f) | ((read_raster_y () & 0x100) >> 1);
        case 0x12:            /* $D012: Raster line LSB */
            return read_raster_y() & 0xff;
        case 0x13:            /* $D013: Light Pen X */
            return vicii.light_pen.x;
        case 0x14:            /* $D014: Light Pen Y */
            return vicii.light_pen.y;
        case 0x19:
            return d019_peek();
        case 0x1e:            /* $D01E: Sprite-sprite collision */
            return vicii.sprite_sprite_collisions;
        case 0x1f:            /* $D01F: Sprite-background collision */
            return vicii.sprite_background_collisions;
        default:
            return vicii.regs[addr] | unused_bits_in_registers[addr];
    }
}

void do_copy(void) {
   if (dma_op == 0) return;

   switch (dma_op) {
     case 1: // low to high
       extraMem[copy_dest + copy_idx] = extraMem[copy_src + copy_idx];
       copy_idx++;
       if (copy_idx == copy_num) {
           dma_op = 0;
           // done
           extraRegs[0x35] = 0;
           extraRegs[0x36] = 0;
       }
       break;
     case 2: // high to low
       extraMem[copy_dest+copy_idx] = extraMem[copy_src+copy_idx];
       copy_idx--;
       if (copy_idx < 0) {
           dma_op = 0;
           // done
           extraRegs[0x35] = 0;
           extraRegs[0x36] = 0;
       }
       break;
     default:
       break;
   }
}

void do_fill(void) {
   if (dma_op == 0) return;

   switch (dma_op) {
     case 4:
       extraMem[fill_idx] = fill_b;
       fill_idx++;
       if (fill_idx == fill_start + fill_num) {
          dma_op = 0;
          // done
          extraRegs[0x35] = 0;
          extraRegs[0x36] = 0;
       }
       break;
     default:
       break;
   }
}

void do_dma_xfer(void) {
   if (dma_op == 0) return;

   unsigned char cia2 = mem_bank_read(3, 56576,0);
   unsigned int bank = ~cia2 & 0x3;
   switch (dma_op) {
     case 8:
       extraMem[copy_dest] = mem_bank_read(bank,copy_src,0);
       copy_src++; copy_dest++;
       copy_idx++;
       if (copy_idx == copy_num) {
          dma_op = 0;
          // done
          extraRegs[0x35] = 0;
          extraRegs[0x36] = 0;
       }
       break;
     case 16:
       mem_bank_write(bank, copy_dest, extraMem[copy_src], 0);
       copy_src++; copy_dest++;
       copy_idx++;
       if (copy_idx == copy_num) {
          dma_op = 0;
          // done
          extraRegs[0x35] = 0;
          extraRegs[0x36] = 0;
       }
       break;
     default:
       break;
   }
}

void do_blit(void) {
    if (!blit_done) {
       switch (blit_state) {
           case 0:
               if (blit_init) {
                   blit_src_cur = blit_src_ptr;
                   blit_dst_cur = blit_dst_ptr;
                   if (pixels_per_byte == 2) {
                       blit_dst_align = blit_dst_x & 1;
                       blit_src_align = blit_src_x & 1;
                   } else {
                       blit_dst_align = blit_dst_x & 3;
                       blit_src_align = blit_src_x & 3;
                   }
                   blit_src_avail = 0;
                   blit_dst_avail = 0;
                   blit_out_avail = 0;
                   blit_pixels_written = 0;
                   blit_src_pos = 0;
                   blit_dst_pos = 0;
                   blit_line = 0;
                   blit_init = 0;
               }
               break;
            case 1:
               if (blit_dst_avail == 0) {
                   blit_tmp_addr = blit_dst_cur + blit_dst_pos;
               }
               break;
            case 2:
               break;
            case 3:
               if (blit_dst_avail == 0) {
                    blit_dst_avail = pixels_per_byte;
                    blit_d = extraMem[blit_tmp_addr];
               }
               if (blit_src_avail == 0) {
                    blit_tmp_addr = blit_src_cur + blit_src_pos;
               }
               break;
            case 4:
               break;
            case 5:
               if (blit_src_avail == 0) {
                   blit_src_avail = pixels_per_byte;
                   blit_s = extraMem[blit_tmp_addr];
                   blit_src_pos = blit_src_pos + 1;
               }
               // Handle dst misalignment
               if (blit_dst_align != 0) {
                  // If we
                  if (pixels_per_byte == 2) {
                      blit_o = (blit_d >> 4) & 0xf;
                      blit_d = (blit_d & 0xf) << 4;
                      blit_out_avail = 1;
                      blit_dst_avail = 1;
                  } else {
                      // This case is a litte more...
                      if (blit_dst_align == 1) {
                          blit_o = (blit_d >> 6) & 0b11;
                          blit_d = (blit_d & 0b111111) << 2;
                      } else if (blit_dst_align == 2) {
                          blit_o = (blit_d >> 4) & 0b1111;
                          blit_d = (blit_d & 0b1111) << 4;
                      } else if (blit_dst_align == 3) {
                          blit_o = (blit_d >> 2) & 0b111111;
                          blit_d = (blit_d & 0b11) << 6;
                      }
                      blit_out_avail = blit_dst_align;
                      blit_dst_avail = 4 - blit_dst_align;
                  }
                  blit_dst_align = 0;
               }

               if (blit_src_align != 0) {
                  // If we
                  if (pixels_per_byte == 2) {
                      blit_s = (blit_s & 0xf) << 4;
                      blit_src_avail = blit_src_avail - 1;
                  } else {
                      if (blit_src_align == 1) {
                         blit_s = (blit_s & 0b111111) << 2;
                      } else if (blit_src_align == 2) {
                         blit_s = (blit_s & 0b1111) << 4;
                      } else if (blit_src_align == 3) {
                         blit_s = (blit_s & 0b11) << 6;
                      }
                      blit_src_avail = 4 - blit_src_align;
                  }
                  blit_src_align = 0;
               }
               break;
           case 6:
               if (blit_pixels_written < blit_width) {
                  if (pixels_per_byte == 2) {
                     if ((blit_flags & 8) && (blit_s & 0b11110000) == (blit_flags & 0b11110000)) {
                        blit_o = ((blit_o & 0b1111) << 4) | ((blit_d & 0b11110000) >> 4);
                     } else {
                        switch (blit_flags & 0b111) {
                           case 0:
                               blit_o = ((blit_o & 0b1111) << 4) | ((blit_s & 0b11110000) >> 4);
                               break;
                           case 1:
                               blit_o = ((blit_o & 0b1111) << 4) | (((blit_s & 0b11110000) >> 4) | ((blit_d & 0b11110000) >> 4));
                               break;
                           case 2:
                               blit_o = ((blit_o & 0b1111) << 4) | (((blit_s & 0b11110000) >> 4) & ((blit_d & 0b11110000) >> 4));
                               break;
                           case 3:
                               blit_o = ((blit_o & 0b1111) << 4) | (((blit_s & 0b11110000) >> 4) ^ ((blit_d & 0b11110000) >> 4));
                               break;
                           default:
                               break;
                        }
                     }
                     blit_d = (blit_d & 0b1111) << 4;
                     blit_s = (blit_s & 0b1111) << 4;
                  } else {
                     if ((blit_flags & 8) && ( ((blit_s & 0b11000000) >>6) == ((blit_flags & 0b110000) >> 4))) {
                         blit_o = ((blit_o & 0b111111) << 2) | ((blit_d & 0b11) >> 6); // transp
                     } else {
                        switch (blit_flags & 0b111) {
                           case 0:
                               blit_o = ((blit_o & 0b111111) << 2) | ((blit_s & 0b11000000) >> 6);
                               break;
                           case 1:
                               blit_o = ((blit_o & 0b111111) << 2) | (((blit_s & 0b11000000) >> 6) | ((blit_d & 0b11000000) >> 6));
                               break;
                           case 2:
                               blit_o = ((blit_o & 0b111111) << 2) | (((blit_s & 0b11000000) >> 6) & ((blit_d & 0b11000000) >> 6));
                               break;
                           case 3:
                               blit_o = ((blit_o & 0b111111) << 2) | (((blit_s & 0b11000000) >> 6) ^ ((blit_d & 0b11000000) >> 6));
                               break;
                           default:
                               break;
                        }
                     }
                     blit_d = (blit_d & 0b111111) << 2;
                     blit_s = (blit_s & 0b111111) << 2;
                  }
                  blit_src_avail = blit_src_avail - 1;
                  blit_pixels_written = blit_pixels_written + 1;
               } else {
                  if (pixels_per_byte == 2) {
                     blit_o = ((blit_o & 0b1111) << 4) | ((blit_d & 0b11110000) >> 4);
                     blit_d = (blit_d & 0b1111) << 4;
                  } else {
                     blit_o = ((blit_o & 0b111111) << 2) | ((blit_d & 0b11000000) >> 6);
                     blit_d = (blit_d & 0b111111) << 2;
                  }
               }
               blit_dst_avail = blit_dst_avail - 1;
               blit_out_avail = blit_out_avail + 1;
               break;
            case 7:
               if (blit_out_avail == pixels_per_byte) {
                   extraMem[blit_dst_cur + blit_dst_pos] = blit_o;
                   blit_out_avail = 0;
                   blit_dst_pos = blit_dst_pos + 1;
                   if (blit_pixels_written >= blit_width) {
                       blit_pixels_written = 0;
                       blit_dst_pos = 0;
                       blit_dst_cur = blit_dst_cur + blit_dst_stride;
                       blit_line = blit_line + 1;
                       if (pixels_per_byte == 2) {
                          blit_dst_align = blit_dst_x & 0b1;
                       } else {
                          blit_dst_align = blit_dst_x & 0b11;
                       }
                       blit_src_pos = 0;
                       blit_src_cur = blit_src_cur + blit_src_stride;
                       if (pixels_per_byte == 2) {
                          blit_src_align = blit_src_x & 0b1;
                       } else {
                          blit_src_align = blit_src_x & 0b11;
                       }
                       blit_src_avail = 0;

                       if (blit_line == blit_height) {
                          blit_done = 1;
                          extraRegs[0x3d] = 0; // signal done
                       }
                   }
               }
               break;
            default:
               break;
       }
       blit_state = blit_state + 1;
       blit_state = blit_state & 0x7;
    }
}

void handle_color_change(uint8_t reg, uint8_t value) {

   if (!kawari_is_composite()) {
       if (reg >= 0x40 && reg <= 0x7f) {
          // RGB
          int colorIndex = (reg - 0x40) / 4;
          int channel = (reg - 0x40) % 4;

          if (channel < 3) {
              kawari_set_rgb(colorIndex, channel, value);
              vicii_color_update_palette(vicii.raster.canvas);
          }
       }
   } else {
       if (reg >= 0xa0 && reg <= 0xaf) {
           // Luma
           kawari_set_luma(reg - 0xa0, value);
           vicii_color_update_palette(vicii.raster.canvas);
       } else if (reg >= 0xb0 && reg <= 0xbf) {
           // Phase
           kawari_set_angle(reg - 0xb0, value);
           vicii_color_update_palette(vicii.raster.canvas);
       } else if (reg >= 0xc0 && reg <= 0xcf) {
           // Amplitude
           kawari_set_amplitude(reg - 0xc0, value);
           vicii_color_update_palette(vicii.raster.canvas);
       }
   }
}

static void handle_eeprom_save(int reg, int value) {
   if (extraRegs[0x3f] & 64) {

       if (!flash_file_name) return;

       // Save to flash
       // TODO: Use seek and write a single byte
       FILE *fp = fopen(flash_file_name,"w");
       if (fp != NULL) {
          for (int n=0;n<256;n++) {
              fprintf(fp,"%c", overlayMem[n]);
          }
          fclose(fp);
       }
       // Bit 16 goes off to indicate we are done, but we are not emulating
       // flash delay anyway, so it never went high. TODO: Set high and then
       // schedule this into the future.
       extraRegs[0x3f] = extraRegs[0x3f] & 239;
   }
}

void set_flash_file_name(char* fname) {
   flash_file_name = fname;
}

