# VIC-II Kawari Emulator

This is a fork of VICE emulator with VIC-II Kawari extensions added to the C64 emulator.  This is provided for development purposes.

## Extended Features

These additions are supported:

* 64k of VRAM
* RGB & YUV Color register changes
* Extra hires video modes (160x200x16,320x200x16,640x200x4)
* 80 column video mode (requires 80COL-51200.PRG from util disk)
* DMA Transfer
* Block Copy/Fill
* Math Operations
* Badline suppression in hires modes for extra compute cycles

## Caveats

I make no guarantees on performance.  The extra hires display is always rendered whether it is visible or not (like the real device) and that likely takes more CPU.  So don't expect this to perform as well as the non-extended version.
