/*
 * vicii-coproc.c - 6502 co-processor executing from extended video RAM.
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

/*
 * This models the WITH_CPU_COPROC 6502 that Kawari adds alongside the 6510.
 * It executes out of the 64K of extended video RAM (extraMem) and is
 * controlled through the extra register overlay at 0x20-0x2b.
 *
 * Behaviour mirrors hdl/videoram_cpu.v and hdl/cpu6502.v:
 *
 *  - the co-processor's 16 bit address space IS the 64K of video RAM
 *  - it sees RAW video RAM at $0000-$00ff even when the 6510 has the
 *    extra-register overlay enabled there, and it can neither read nor write
 *    the overlay registers
 *  - $fffc/$fffd read back as the start vector register rather than as video
 *    RAM, which is how the core is given an arbitrary entry point. Writes to
 *    those addresses still land in video RAM, matching the hardware where the
 *    shadow is only on the core's data-in path
 *  - it powers up held in reset
 *
 * The core is an Arlet Ottens design whose reset sequence ends up with
 * SP=$fd, I set, and A/X/Y cleared, which is what vicii_coproc_reset()
 * reproduces.
 */

#include "vice.h"

#include <string.h>

#include "vicii-coproc.h"

/* The 64K of extended video RAM lives in vicii-mem.c. */
extern uint8_t extraMem[65536];

/* Nonzero while a DMA, fill, copy or blitter is running. A CPU bus cycle needs
   consecutive VIC cycles, so any of those take the port away and stall the
   co-processor, exactly like the arbiter in videoram_cpu.v. */
extern int vicii_legacy_busy(void);

/* Processor status bits. Bits 5 and 4 always read as 1 in the exposed P
   register, matching the hardware's {N,V,2'b11,D,I,Z,C}. */
#define F_C 0x01
#define F_Z 0x02
#define F_I 0x04
#define F_D 0x08
#define F_B 0x10
#define F_U 0x20
#define F_V 0x40
#define F_N 0x80

typedef struct vicii_coproc_s {
    uint8_t a, x, y, sp, p;
    uint16_t pc;

    uint8_t reset;      /* 1 = held in reset */
    uint8_t hold;       /* 1 = paused */
    uint8_t irq;        /* level sensitive */
    uint8_t nmi;        /* edge sensitive */
    uint8_t nmi_edge;   /* a rising edge has been seen */

    uint8_t speed;      /* access rate divider, 0 = fastest */
    uint8_t start_lo, start_hi;
} vicii_coproc_t;

static vicii_coproc_t cp;
static uint8_t cp_nmi_prev;
/* Tracks the reset line so the vector is loaded on release even if no step
   happened while reset was asserted (the 6510 can set the vector and clear
   RESET back to back within one VIC cycle). */
static uint8_t cp_prev_reset;

/* ------------------------------------------------------------------ */
/* memory                                                             */
/* ------------------------------------------------------------------ */

static uint8_t cp_rd(uint16_t addr)
{
    /* The reset vector is supplied by the start vector register, not by RAM. */
    if (addr == 0xfffc) {
        return cp.start_lo;
    }
    if (addr == 0xfffd) {
        return cp.start_hi;
    }
    return extraMem[addr];
}

static void cp_wr(uint16_t addr, uint8_t value)
{
    /* Writes are NOT shadowed: the hardware only muxes the read path. */
    extraMem[addr] = value;
}

/* ------------------------------------------------------------------ */
/* reset                                                              */
/* ------------------------------------------------------------------ */

void vicii_coproc_reset(void)
{
    memset(&cp, 0, sizeof(cp));
    cp.reset = 1;               /* powers up held in reset */
    cp.p = F_I | F_U | F_B;
    cp.sp = 0xfd;
    cp_prev_reset = 1;
    cp_nmi_prev = 0;
}

/* Load the architectural reset state. Applied continuously while reset is
   asserted, so changing the start vector while held takes effect. */
static void cp_apply_reset(void)
{
    cp.a = 0;
    cp.x = 0;
    cp.y = 0;
    cp.sp = 0xfd;
    cp.p = F_I | F_U | F_B;
    cp.pc = (uint16_t)((cp.start_hi << 8) | cp.start_lo);
    cp.nmi_edge = 0;
}

/* ------------------------------------------------------------------ */
/* register interface (overlay 0x20-0x2b)                             */
/* ------------------------------------------------------------------ */

uint8_t vicii_coproc_read_reg(int idx)
{
    switch (idx) {
    case 0x20: return cp.start_lo;
    case 0x21: return cp.start_hi;
    case 0x22:
        return (uint8_t)((cp.reset ? 0x01 : 0) | (cp.irq ? 0x02 : 0) |
                         (cp.nmi ? 0x04 : 0) | (cp.hold ? 0x08 : 0));
    case 0x23: return cp.a;
    case 0x24: return cp.x;
    case 0x25: return cp.y;
    case 0x26: return cp.sp;
    case 0x27: return cp.p;
    case 0x28: return (uint8_t)(cp.pc & 0xff);
    case 0x29: return (uint8_t)(cp.pc >> 8);
    case 0x2a:
        return (uint8_t)((cp.reset ? 0 : 0x01) | (cp.hold ? 0x02 : 0) |
                         (vicii_legacy_busy() ? 0x04 : 0));
    case 0x2b: return cp.speed;
    default:   return 0;
    }
}

void vicii_coproc_write_reg(int idx, uint8_t value)
{
    switch (idx) {
    case 0x20: cp.start_lo = value; break;
    case 0x21: cp.start_hi = value; break;
    case 0x22:
        cp.reset = value & 0x01;
        cp.irq   = (value >> 1) & 0x01;
        cp.nmi   = (value >> 2) & 0x01;
        cp.hold  = (value >> 3) & 0x01;
        break;
    case 0x2b: cp.speed = value & 0x03; break;
    default:   break;   /* read-only registers ignore writes */
    }
}

/* ------------------------------------------------------------------ */
/* 6502                                                               */
/* ------------------------------------------------------------------ */

#define PUSH(v)  do { cp_wr((uint16_t)(0x100 + cp.sp), (uint8_t)(v)); cp.sp--; } while (0)
#define PULL()   (cp.sp++, cp_rd((uint16_t)(0x100 + cp.sp)))

#define SETNZ(v) do {                                                     \
        uint8_t nz_v = (uint8_t)(v);                                      \
        cp.p = (uint8_t)((cp.p & ~(F_N | F_Z)) | (nz_v & F_N) |           \
                         (nz_v == 0 ? F_Z : 0));                          \
    } while (0)

#define SETC(c)  do { if (c) { cp.p |= F_C; } else { cp.p &= ~F_C; } } while (0)
#define SETV(v)  do { if (v) { cp.p |= F_V; } else { cp.p &= ~F_V; } } while (0)
#define GETC()   ((cp.p & F_C) ? 1 : 0)

static uint16_t cp_imm(void)  { return cp.pc++; }
static uint16_t cp_zp(void)   { return cp_rd(cp.pc++); }
static uint16_t cp_zpx(void)  { return (uint16_t)((cp_rd(cp.pc++) + cp.x) & 0xff); }
static uint16_t cp_zpy(void)  { return (uint16_t)((cp_rd(cp.pc++) + cp.y) & 0xff); }

static uint16_t cp_abs(void)
{
    uint16_t lo = cp_rd(cp.pc++);
    uint16_t hi = cp_rd(cp.pc++);
    return (uint16_t)(lo | (hi << 8));
}

static uint16_t cp_abx(int *extra)
{
    uint16_t base = cp_abs();
    uint16_t addr = (uint16_t)(base + cp.x);
    if ((base ^ addr) & 0xff00) {
        (*extra)++;
    }
    return addr;
}

static uint16_t cp_aby(int *extra)
{
    uint16_t base = cp_abs();
    uint16_t addr = (uint16_t)(base + cp.y);
    if ((base ^ addr) & 0xff00) {
        (*extra)++;
    }
    return addr;
}

static uint16_t cp_indx(void)
{
    uint8_t z = (uint8_t)(cp_rd(cp.pc++) + cp.x);
    return (uint16_t)(cp_rd(z) | (cp_rd((uint8_t)(z + 1)) << 8));
}

static uint16_t cp_indy(int *extra)
{
    uint8_t z = cp_rd(cp.pc++);
    uint16_t base = (uint16_t)(cp_rd(z) | (cp_rd((uint8_t)(z + 1)) << 8));
    uint16_t addr = (uint16_t)(base + cp.y);
    if ((base ^ addr) & 0xff00) {
        (*extra)++;
    }
    return addr;
}

/* JMP (abs) has the famous page wrap bug: the high byte comes from the same
   page as the low byte. */
static uint16_t cp_ind(void)
{
    uint16_t ptr = cp_abs();
    uint16_t hi_addr = (uint16_t)((ptr & 0xff00) | ((ptr + 1) & 0x00ff));
    return (uint16_t)(cp_rd(ptr) | (cp_rd(hi_addr) << 8));
}

static void cp_adc(uint8_t m)
{
    uint8_t a = cp.a;
    unsigned int r = (unsigned int)a + m + GETC();

    if (cp.p & F_D) {
        /* NMOS decimal mode. N/V/Z come from the binary result, as on the
           real chip and in the Arlet ALU. */
        unsigned int lo = (a & 0x0f) + (m & 0x0f) + GETC();
        unsigned int hi = (a >> 4) + (m >> 4);
        if (lo > 9) {
            lo += 6;
            hi++;
        }
        if (hi > 9) {
            hi += 6;
        }
        cp.p = (uint8_t)((cp.p & ~(F_N | F_V | F_Z)) |
                         (r & 0x80 ? F_N : 0) |
                         ((~(a ^ m) & (a ^ (uint8_t)r) & 0x80) ? F_V : 0) |
                         (((uint8_t)r) == 0 ? F_Z : 0));
        SETC(hi > 15);
        cp.a = (uint8_t)((hi << 4) | (lo & 0x0f));
        return;
    }

    SETC(r > 0xff);
    SETV((~(a ^ m) & (a ^ (uint8_t)r) & 0x80) != 0);
    cp.a = (uint8_t)r;
    SETNZ(cp.a);
}

static void cp_sbc(uint8_t m)
{
    uint8_t a = cp.a;
    unsigned int r = (unsigned int)a - m - (GETC() ? 0 : 1);

    if (cp.p & F_D) {
        unsigned int lo = (a & 0x0f) - (m & 0x0f) - (GETC() ? 0 : 1);
        int hi = (a >> 4) - (m >> 4);
        if (lo & 0x10) {
            lo -= 6;
            hi--;
        }
        if (hi & 0x10) {
            hi -= 6;
        }
        cp.p = (uint8_t)((cp.p & ~(F_N | F_V | F_Z)) |
                         (r & 0x80 ? F_N : 0) |
                         (((a ^ m) & (a ^ (uint8_t)r) & 0x80) ? F_V : 0) |
                         (((uint8_t)r) == 0 ? F_Z : 0));
        SETC(r < 0x100);
        cp.a = (uint8_t)(((hi << 4) & 0xf0) | (lo & 0x0f));
        return;
    }

    SETC(r < 0x100);
    SETV(((a ^ m) & (a ^ (uint8_t)r) & 0x80) != 0);
    cp.a = (uint8_t)r;
    SETNZ(cp.a);
}

static void cp_cmp(uint8_t reg, uint8_t m)
{
    unsigned int r = (unsigned int)reg - m;
    SETC(reg >= m);
    SETNZ((uint8_t)r);
}

static uint8_t cp_asl(uint8_t v) { SETC(v & 0x80); v <<= 1; SETNZ(v); return v; }
static uint8_t cp_lsr(uint8_t v) { SETC(v & 0x01); v >>= 1; SETNZ(v); return v; }

static uint8_t cp_rol(uint8_t v)
{
    uint8_t c = (uint8_t)GETC();
    SETC(v & 0x80);
    v = (uint8_t)((v << 1) | c);
    SETNZ(v);
    return v;
}

static uint8_t cp_ror(uint8_t v)
{
    uint8_t c = (uint8_t)(GETC() << 7);
    SETC(v & 0x01);
    v = (uint8_t)((v >> 1) | c);
    SETNZ(v);
    return v;
}

static int cp_branch(int take, int *cyc)
{
    int8_t off = (int8_t)cp_rd(cp.pc++);
    if (take) {
        uint16_t old = cp.pc;
        cp.pc = (uint16_t)(cp.pc + off);
        (*cyc)++;
        if ((old ^ cp.pc) & 0xff00) {
            (*cyc)++;
        }
    }
    return 0;
}

static void cp_interrupt(uint16_t vector, int is_brk)
{
    PUSH((uint8_t)(cp.pc >> 8));
    PUSH((uint8_t)(cp.pc & 0xff));
    /* B is set for BRK, clear for IRQ/NMI. */
    PUSH((uint8_t)((cp.p & ~F_B) | (is_brk ? F_B : 0)));
    cp.p |= F_I;
    cp.pc = (uint16_t)(cp_rd(vector) | (cp_rd((uint16_t)(vector + 1)) << 8));
}

/* Execute one instruction. Returns the number of 6502 bus cycles it took. */
static int cp_exec(void)
{
    int cyc = 1;
    int extra = 0;
    uint8_t op = cp_rd(cp.pc++);
    uint16_t addr;
    uint8_t m;

    switch (op) {
    /* ---- loads ---- */
    case 0xa9: cp.a = cp_rd(cp_imm()); SETNZ(cp.a); cyc = 2; break;
    case 0xa5: cp.a = cp_rd(cp_zp());  SETNZ(cp.a); cyc = 3; break;
    case 0xb5: cp.a = cp_rd(cp_zpx()); SETNZ(cp.a); cyc = 4; break;
    case 0xad: cp.a = cp_rd(cp_abs()); SETNZ(cp.a); cyc = 4; break;
    case 0xbd: addr = cp_abx(&extra); cp.a = cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0xb9: addr = cp_aby(&extra); cp.a = cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0xa1: cp.a = cp_rd(cp_indx()); SETNZ(cp.a); cyc = 6; break;
    case 0xb1: addr = cp_indy(&extra); cp.a = cp_rd(addr); SETNZ(cp.a); cyc = 5 + extra; break;

    case 0xa2: cp.x = cp_rd(cp_imm()); SETNZ(cp.x); cyc = 2; break;
    case 0xa6: cp.x = cp_rd(cp_zp());  SETNZ(cp.x); cyc = 3; break;
    case 0xb6: cp.x = cp_rd(cp_zpy()); SETNZ(cp.x); cyc = 4; break;
    case 0xae: cp.x = cp_rd(cp_abs()); SETNZ(cp.x); cyc = 4; break;
    case 0xbe: addr = cp_aby(&extra); cp.x = cp_rd(addr); SETNZ(cp.x); cyc = 4 + extra; break;

    case 0xa0: cp.y = cp_rd(cp_imm()); SETNZ(cp.y); cyc = 2; break;
    case 0xa4: cp.y = cp_rd(cp_zp());  SETNZ(cp.y); cyc = 3; break;
    case 0xb4: cp.y = cp_rd(cp_zpx()); SETNZ(cp.y); cyc = 4; break;
    case 0xac: cp.y = cp_rd(cp_abs()); SETNZ(cp.y); cyc = 4; break;
    case 0xbc: addr = cp_abx(&extra); cp.y = cp_rd(addr); SETNZ(cp.y); cyc = 4 + extra; break;

    /* ---- stores ---- */
    case 0x85: cp_wr(cp_zp(), cp.a);  cyc = 3; break;
    case 0x95: cp_wr(cp_zpx(), cp.a); cyc = 4; break;
    case 0x8d: cp_wr(cp_abs(), cp.a); cyc = 4; break;
    case 0x9d: cp_wr(cp_abx(&extra), cp.a); cyc = 5; break;
    case 0x99: cp_wr(cp_aby(&extra), cp.a); cyc = 5; break;
    case 0x81: cp_wr(cp_indx(), cp.a); cyc = 6; break;
    case 0x91: cp_wr(cp_indy(&extra), cp.a); cyc = 6; break;

    case 0x86: cp_wr(cp_zp(), cp.x);  cyc = 3; break;
    case 0x96: cp_wr(cp_zpy(), cp.x); cyc = 4; break;
    case 0x8e: cp_wr(cp_abs(), cp.x); cyc = 4; break;

    case 0x84: cp_wr(cp_zp(), cp.y);  cyc = 3; break;
    case 0x94: cp_wr(cp_zpx(), cp.y); cyc = 4; break;
    case 0x8c: cp_wr(cp_abs(), cp.y); cyc = 4; break;

    /* ---- transfers ---- */
    case 0xaa: cp.x = cp.a; SETNZ(cp.x); cyc = 2; break;
    case 0xa8: cp.y = cp.a; SETNZ(cp.y); cyc = 2; break;
    case 0x8a: cp.a = cp.x; SETNZ(cp.a); cyc = 2; break;
    case 0x98: cp.a = cp.y; SETNZ(cp.a); cyc = 2; break;
    case 0xba: cp.x = cp.sp; SETNZ(cp.x); cyc = 2; break;
    case 0x9a: cp.sp = cp.x; cyc = 2; break;

    /* ---- stack ---- */
    case 0x48: PUSH(cp.a); cyc = 3; break;
    case 0x68: cp.a = PULL(); SETNZ(cp.a); cyc = 4; break;
    case 0x08: PUSH((uint8_t)(cp.p | F_B)); cyc = 3; break;
    case 0x28: cp.p = (uint8_t)((PULL() | F_U) & ~F_B); cyc = 4; break;

    /* ---- logic ---- */
    case 0x29: cp.a &= cp_rd(cp_imm()); SETNZ(cp.a); cyc = 2; break;
    case 0x25: cp.a &= cp_rd(cp_zp());  SETNZ(cp.a); cyc = 3; break;
    case 0x35: cp.a &= cp_rd(cp_zpx()); SETNZ(cp.a); cyc = 4; break;
    case 0x2d: cp.a &= cp_rd(cp_abs()); SETNZ(cp.a); cyc = 4; break;
    case 0x3d: addr = cp_abx(&extra); cp.a &= cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0x39: addr = cp_aby(&extra); cp.a &= cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0x21: cp.a &= cp_rd(cp_indx()); SETNZ(cp.a); cyc = 6; break;
    case 0x31: addr = cp_indy(&extra); cp.a &= cp_rd(addr); SETNZ(cp.a); cyc = 5 + extra; break;

    case 0x09: cp.a |= cp_rd(cp_imm()); SETNZ(cp.a); cyc = 2; break;
    case 0x05: cp.a |= cp_rd(cp_zp());  SETNZ(cp.a); cyc = 3; break;
    case 0x15: cp.a |= cp_rd(cp_zpx()); SETNZ(cp.a); cyc = 4; break;
    case 0x0d: cp.a |= cp_rd(cp_abs()); SETNZ(cp.a); cyc = 4; break;
    case 0x1d: addr = cp_abx(&extra); cp.a |= cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0x19: addr = cp_aby(&extra); cp.a |= cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0x01: cp.a |= cp_rd(cp_indx()); SETNZ(cp.a); cyc = 6; break;
    case 0x11: addr = cp_indy(&extra); cp.a |= cp_rd(addr); SETNZ(cp.a); cyc = 5 + extra; break;

    case 0x49: cp.a ^= cp_rd(cp_imm()); SETNZ(cp.a); cyc = 2; break;
    case 0x45: cp.a ^= cp_rd(cp_zp());  SETNZ(cp.a); cyc = 3; break;
    case 0x55: cp.a ^= cp_rd(cp_zpx()); SETNZ(cp.a); cyc = 4; break;
    case 0x4d: cp.a ^= cp_rd(cp_abs()); SETNZ(cp.a); cyc = 4; break;
    case 0x5d: addr = cp_abx(&extra); cp.a ^= cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0x59: addr = cp_aby(&extra); cp.a ^= cp_rd(addr); SETNZ(cp.a); cyc = 4 + extra; break;
    case 0x41: cp.a ^= cp_rd(cp_indx()); SETNZ(cp.a); cyc = 6; break;
    case 0x51: addr = cp_indy(&extra); cp.a ^= cp_rd(addr); SETNZ(cp.a); cyc = 5 + extra; break;

    case 0x24: m = cp_rd(cp_zp());  SETNZ((uint8_t)(cp.a & m)); SETV(m & 0x40); SETC(m & 0x80); cyc = 3; break;
    case 0x2c: m = cp_rd(cp_abs()); SETNZ((uint8_t)(cp.a & m)); SETV(m & 0x40); SETC(m & 0x80); cyc = 4; break;

    /* ---- arithmetic ---- */
    case 0x69: cp_adc(cp_rd(cp_imm())); cyc = 2; break;
    case 0x65: cp_adc(cp_rd(cp_zp()));  cyc = 3; break;
    case 0x75: cp_adc(cp_rd(cp_zpx())); cyc = 4; break;
    case 0x6d: cp_adc(cp_rd(cp_abs())); cyc = 4; break;
    case 0x7d: addr = cp_abx(&extra); cp_adc(cp_rd(addr)); cyc = 4 + extra; break;
    case 0x79: addr = cp_aby(&extra); cp_adc(cp_rd(addr)); cyc = 4 + extra; break;
    case 0x61: cp_adc(cp_rd(cp_indx())); cyc = 6; break;
    case 0x71: addr = cp_indy(&extra); cp_adc(cp_rd(addr)); cyc = 5 + extra; break;

    case 0xe9: cp_sbc(cp_rd(cp_imm())); cyc = 2; break;
    case 0xe5: cp_sbc(cp_rd(cp_zp()));  cyc = 3; break;
    case 0xf5: cp_sbc(cp_rd(cp_zpx())); cyc = 4; break;
    case 0xed: cp_sbc(cp_rd(cp_abs())); cyc = 4; break;
    case 0xfd: addr = cp_abx(&extra); cp_sbc(cp_rd(addr)); cyc = 4 + extra; break;
    case 0xf9: addr = cp_aby(&extra); cp_sbc(cp_rd(addr)); cyc = 4 + extra; break;
    case 0xe1: cp_sbc(cp_rd(cp_indx())); cyc = 6; break;
    case 0xf1: addr = cp_indy(&extra); cp_sbc(cp_rd(addr)); cyc = 5 + extra; break;

    case 0xc9: cp_cmp(cp.a, cp_rd(cp_imm())); cyc = 2; break;
    case 0xc5: cp_cmp(cp.a, cp_rd(cp_zp()));  cyc = 3; break;
    case 0xd5: cp_cmp(cp.a, cp_rd(cp_zpx())); cyc = 4; break;
    case 0xcd: cp_cmp(cp.a, cp_rd(cp_abs())); cyc = 4; break;
    case 0xdd: addr = cp_abx(&extra); cp_cmp(cp.a, cp_rd(addr)); cyc = 4 + extra; break;
    case 0xd9: addr = cp_aby(&extra); cp_cmp(cp.a, cp_rd(addr)); cyc = 4 + extra; break;
    case 0xc1: cp_cmp(cp.a, cp_rd(cp_indx())); cyc = 6; break;
    case 0xd1: addr = cp_indy(&extra); cp_cmp(cp.a, cp_rd(addr)); cyc = 5 + extra; break;

    case 0xe0: cp_cmp(cp.x, cp_rd(cp_imm())); cyc = 2; break;
    case 0xe4: cp_cmp(cp.x, cp_rd(cp_zp()));  cyc = 3; break;
    case 0xec: cp_cmp(cp.x, cp_rd(cp_abs())); cyc = 4; break;

    case 0xc0: cp_cmp(cp.y, cp_rd(cp_imm())); cyc = 2; break;
    case 0xc4: cp_cmp(cp.y, cp_rd(cp_zp()));  cyc = 3; break;
    case 0xcc: cp_cmp(cp.y, cp_rd(cp_abs())); cyc = 4; break;

    /* ---- inc / dec ---- */
    case 0xe6: addr = cp_zp();  m = cp_rd(addr); m++; cp_wr(addr, m); SETNZ(m); cyc = 5; break;
    case 0xf6: addr = cp_zpx(); m = cp_rd(addr); m++; cp_wr(addr, m); SETNZ(m); cyc = 6; break;
    case 0xee: addr = cp_abs(); m = cp_rd(addr); m++; cp_wr(addr, m); SETNZ(m); cyc = 6; break;
    case 0xfe: addr = cp_abx(&extra); m = cp_rd(addr); m++; cp_wr(addr, m); SETNZ(m); cyc = 7; break;

    case 0xc6: addr = cp_zp();  m = cp_rd(addr); m--; cp_wr(addr, m); SETNZ(m); cyc = 5; break;
    case 0xd6: addr = cp_zpx(); m = cp_rd(addr); m--; cp_wr(addr, m); SETNZ(m); cyc = 6; break;
    case 0xce: addr = cp_abs(); m = cp_rd(addr); m--; cp_wr(addr, m); SETNZ(m); cyc = 6; break;
    case 0xde: addr = cp_abx(&extra); m = cp_rd(addr); m--; cp_wr(addr, m); SETNZ(m); cyc = 7; break;

    case 0xe8: cp.x++; SETNZ(cp.x); cyc = 2; break;
    case 0xc8: cp.y++; SETNZ(cp.y); cyc = 2; break;
    case 0xca: cp.x--; SETNZ(cp.x); cyc = 2; break;
    case 0x88: cp.y--; SETNZ(cp.y); cyc = 2; break;

    /* ---- shifts ---- */
    case 0x0a: cp.a = cp_asl(cp.a); cyc = 2; break;
    case 0x06: addr = cp_zp();  m = cp_rd(addr); cp_wr(addr, cp_asl(m)); cyc = 5; break;
    case 0x16: addr = cp_zpx(); m = cp_rd(addr); cp_wr(addr, cp_asl(m)); cyc = 6; break;
    case 0x0e: addr = cp_abs(); m = cp_rd(addr); cp_wr(addr, cp_asl(m)); cyc = 6; break;
    case 0x1e: addr = cp_abx(&extra); m = cp_rd(addr); cp_wr(addr, cp_asl(m)); cyc = 7; break;

    case 0x4a: cp.a = cp_lsr(cp.a); cyc = 2; break;
    case 0x46: addr = cp_zp();  m = cp_rd(addr); cp_wr(addr, cp_lsr(m)); cyc = 5; break;
    case 0x56: addr = cp_zpx(); m = cp_rd(addr); cp_wr(addr, cp_lsr(m)); cyc = 6; break;
    case 0x4e: addr = cp_abs(); m = cp_rd(addr); cp_wr(addr, cp_lsr(m)); cyc = 6; break;
    case 0x5e: addr = cp_abx(&extra); m = cp_rd(addr); cp_wr(addr, cp_lsr(m)); cyc = 7; break;

    case 0x2a: cp.a = cp_rol(cp.a); cyc = 2; break;
    case 0x26: addr = cp_zp();  m = cp_rd(addr); cp_wr(addr, cp_rol(m)); cyc = 5; break;
    case 0x36: addr = cp_zpx(); m = cp_rd(addr); cp_wr(addr, cp_rol(m)); cyc = 6; break;
    case 0x2e: addr = cp_abs(); m = cp_rd(addr); cp_wr(addr, cp_rol(m)); cyc = 6; break;
    case 0x3e: addr = cp_abx(&extra); m = cp_rd(addr); cp_wr(addr, cp_rol(m)); cyc = 7; break;

    case 0x6a: cp.a = cp_ror(cp.a); cyc = 2; break;
    case 0x66: addr = cp_zp();  m = cp_rd(addr); cp_wr(addr, cp_ror(m)); cyc = 5; break;
    case 0x76: addr = cp_zpx(); m = cp_rd(addr); cp_wr(addr, cp_ror(m)); cyc = 6; break;
    case 0x6e: addr = cp_abs(); m = cp_rd(addr); cp_wr(addr, cp_ror(m)); cyc = 6; break;
    case 0x7e: addr = cp_abx(&extra); m = cp_rd(addr); cp_wr(addr, cp_ror(m)); cyc = 7; break;

    /* ---- jumps / subroutines ---- */
    case 0x4c: cp.pc = cp_abs(); cyc = 3; break;
    case 0x6c: cp.pc = cp_ind(); cyc = 5; break;
    case 0x20: {
        uint16_t target = cp_abs();
        uint16_t ret = (uint16_t)(cp.pc - 1);
        PUSH((uint8_t)(ret >> 8));
        PUSH((uint8_t)(ret & 0xff));
        cp.pc = target;
        cyc = 6;
        break;
    }
    case 0x60: {
        uint8_t lo = PULL();
        uint8_t hi = PULL();
        cp.pc = (uint16_t)(((hi << 8) | lo) + 1);
        cyc = 6;
        break;
    }
    case 0x40:
        cp.p = (uint8_t)((PULL() | F_U) & ~F_B);
        {
            uint8_t lo = PULL();
            uint8_t hi = PULL();
            cp.pc = (uint16_t)((hi << 8) | lo);
        }
        cyc = 6;
        break;
    case 0x00:
        cp.pc++;                       /* BRK has a padding byte */
        cp_interrupt(0xfffe, 1);
        cyc = 7;
        break;

    /* ---- branches ---- */
    case 0x10: cyc = 2; cp_branch(!(cp.p & F_N), &cyc); break;
    case 0x30: cyc = 2; cp_branch((cp.p & F_N) != 0, &cyc); break;
    case 0x50: cyc = 2; cp_branch(!(cp.p & F_V), &cyc); break;
    case 0x70: cyc = 2; cp_branch((cp.p & F_V) != 0, &cyc); break;
    case 0x90: cyc = 2; cp_branch(!(cp.p & F_C), &cyc); break;
    case 0xb0: cyc = 2; cp_branch((cp.p & F_C) != 0, &cyc); break;
    case 0xd0: cyc = 2; cp_branch(!(cp.p & F_Z), &cyc); break;
    case 0xf0: cyc = 2; cp_branch((cp.p & F_Z) != 0, &cyc); break;

    /* ---- flags ---- */
    case 0x18: cp.p &= ~F_C; cyc = 2; break;
    case 0x38: cp.p |= F_C;  cyc = 2; break;
    case 0x58: cp.p &= ~F_I; cyc = 2; break;
    case 0x78: cp.p |= F_I;  cyc = 2; break;
    case 0xb8: cp.p &= ~F_V; cyc = 2; break;
    case 0xd8: cp.p &= ~F_D; cyc = 2; break;
    case 0xf8: cp.p |= F_D;  cyc = 2; break;

    case 0xea: cyc = 2; break;   /* NOP */

    /* Everything else is an undocumented opcode. The Arlet core decodes
       several of those through casex wildcards; this model treats them all as
       a two cycle NOP and does not consume operand bytes, so a program that
       relies on undocumented behaviour will diverge here. */
    default: cyc = 2; break;
    }

    return cyc;
}

/* ------------------------------------------------------------------ */
/* stepping                                                           */
/* ------------------------------------------------------------------ */

/*
 * One VIC (C64) cycle is 32 clk_dot4x. The hardware gives the co-processor a
 * bus cycle every (1 + 2^speed) clk_dot4x, so with speed 0 it runs at exactly
 * 16x the 6510, which is the ratio the FPGA produces.
 */
void vicii_coproc_step(void)
{
    int per_cycle;
    int budget = 32;

    if (cp.reset) {
        cp_apply_reset();
        cp_prev_reset = 1;
        cp_nmi_prev = cp.nmi;
        return;
    }
    if (cp_prev_reset) {
        /* Falling edge of RESET: the core loads PC from the start vector and
           begins executing there. */
        cp_apply_reset();
        cp_prev_reset = 0;
    }
    if (cp.hold) {
        cp_nmi_prev = cp.nmi;
        return;
    }

    /* A DMA, fill, copy or blit owns video RAM port A for the whole operation
       and takes the port away from the CPU, so it makes no progress. */
    if (vicii_legacy_busy()) {
        cp_nmi_prev = cp.nmi;
        return;
    }

    per_cycle = 1 + (1 << cp.speed);

    /* NMI is edge sensitive, IRQ is level sensitive. */
    if (cp.nmi && !cp_nmi_prev) {
        cp.nmi_edge = 1;
    }
    cp_nmi_prev = cp.nmi;

    if (cp.nmi_edge) {
        cp_interrupt(0xfffa, 0);
        cp.nmi_edge = 0;
        budget -= 7 * per_cycle;
    } else if (cp.irq && !(cp.p & F_I)) {
        cp_interrupt(0xfffe, 0);
        budget -= 7 * per_cycle;
    }

    while (budget > 0) {
        int used = cp_exec();
        budget -= used * per_cycle;
    }
}
