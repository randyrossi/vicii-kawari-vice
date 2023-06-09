/*
 * vicii-draw.c - Rendering for the MOS6569 (VIC-II) emulation.
 *
 * Written by
 *  Andreas Boose <viceteam@t-online.de>
 *  Ettore Perazzoli <ettore@comm2000.it>
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

#include <string.h>

#include "raster-cache.h"
#include "raster-modes.h"
#include "raster.h"
#include "types.h"
#include "vicii-draw.h"
#include "viciitypes.h"
#include "viewport.h"

/* If unaligned 32-bit access is not allowed, the graphics is stored in a
   temporary aligned buffer, and later copied to the real frame buffer.  This
   is ugly, but should be hopefully faster than accessing 8 bits at a time
   anyway.  */

#ifndef ALLOW_UNALIGNED_ACCESS
static uint32_t _aligned_line_buffer[VICII_SCREEN_XPIX / 2 + 1];
static uint8_t *const aligned_line_buffer = (uint8_t *)_aligned_line_buffer;
#endif


/* Pointer to the start of the graphics area on the frame buffer.  */
#define GFX_PTR()  (vicii.raster.draw_buffer_ptr)


#ifdef ALLOW_UNALIGNED_ACCESS
#define ALIGN_DRAW_FUNC(name, xs, xe, gfx_msk_ptr)  name(GFX_PTR(), (xs), (xe), (gfx_msk_ptr))
#else
#define ALIGN_DRAW_FUNC(name, xs, xe, gfx_msk_ptr)            \
    do {                                                      \
        name(aligned_line_buffer, (xs), (xe), (gfx_msk_ptr)); \
        memcpy(GFX_PTR() + (xs) * 8,                          \
               aligned_line_buffer + (xs) * 8,                \
               ((xe) - (xs) + 1) * 8);                        \
    } while (0)
#endif


/* Dummy mode for using cycle based drawing.  */
#define FULL_WIDTH_CHARS ((vicii.screen_leftborderwidth / 8) + VICII_SCREEN_TEXTCOLS + (vicii.screen_rightborderwidth / 8))

// Doubled for kawari
#define DBUF_OFFSET (17 * 16 - vicii.screen_leftborderwidth)

static int get_dummy(raster_cache_t *cache, unsigned int *xs, unsigned int *xe,
                     int rr)
{
    if (rr || 1) {
        *xs = 0;
        *xe = FULL_WIDTH_CHARS - 1;
        return 1;
    } else {
        return 0;
    }
}

inline static void _draw_dummy(uint8_t *p, unsigned int xs, unsigned int xe,
                               uint8_t *gfx_msk_ptr)
{
    uint8_t *src;
    uint8_t *dest;

    // Doubled for Kawari
    src = &(vicii.dbuf[DBUF_OFFSET + xs * 16]);
    dest = (p + xs * 16);

    // Doubled for Kawari
    memcpy(dest, src, (xe - xs + 1) * 16);
}

static void draw_dummy(void)
{
    ALIGN_DRAW_FUNC(_draw_dummy, 0, FULL_WIDTH_CHARS - 1,
                    vicii.raster.gfx_msk);
}

static void draw_dummy_cached(raster_cache_t *cache, unsigned int xs,
                              unsigned int xe)
{
    ALIGN_DRAW_FUNC(_draw_dummy, xs, xe, cache->gfx_msk);
}

static void draw_dummy_foreground(unsigned int start_char,
                                  unsigned int end_char)
{
#if 1
    /* This is used on raster_changes, should not be needed anymore. */
    uint8_t *src;
    uint8_t *dest;

    src = &(vicii.dbuf[DBUF_OFFSET + start_char * 8]);
    dest = (GFX_PTR() + start_char * 8);

    memcpy(dest, src, (end_char - start_char + 1) * 8);
#endif
}

static void draw_dummy_background(unsigned int start_pixel,
                                  unsigned int end_pixel)
{
}

static void setup_modes(void)
{
    raster_modes_set(vicii.raster.modes, VICII_DUMMY_MODE,
                     get_dummy,
                     draw_dummy_cached,
                     draw_dummy,
                     draw_dummy_background,
                     draw_dummy_foreground);
}


void vicii_draw_init(void)
{
    setup_modes();
}
