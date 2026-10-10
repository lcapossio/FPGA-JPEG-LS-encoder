# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
Definitions shared by the four-encoder demo (example/de25_nano_x4), its
simulation (SIM/run_sim.py --ctrl4) and its host script, so all three agree
on the tiling, the input word packing and the test pattern.

The image is split into four vertical tiles, each encoded by its own
jls_encoder as an independent JPEG-LS image:

    Wt = W // 4;  tile t starts at column t*Wt;  tiles 0..2 are Wt wide,
    tile 3 takes the remainder (W - 3*Wt).

Pixels enter the hardware in raster order, four per 32-bit word: pixel x of a
row is byte lane x % 4 of word x // 4, and each row is padded to ceil(W/4)
words (the padding bytes are ignored).

Register map of axi_jls4_ctrl (byte addresses, 32-bit registers):

  0x000 CTRL       W   pulse: [0]=soft_reset [1]=start [2]=done_clear
  0x004 WIDTH      R/W [14:0] image width
  0x008 HEIGHT     R/W [14:0] image height (1..16384)
  0x00C STATUS     R   [0]=busy [1]=done_sticky [2]=in_full
                       [4]=out_overflow_sticky [5]=dim_error_sticky
                       [19:8]=in_count (input FIFO words)
  0x010 NEAR       R   compile-time NEAR
  0x014 MAX_TILE_W R   compile-time encoder MAX_WIDTH (widest tile)
  0x018 FIFO_DEPTH R   [15:0]=input FIFO words [31:16]=output FIFO words per tile
  0x01C MODE       R/W [0]=pattern: the on-chip test pattern replaces PIX_IN
                       and the outputs only feed the byte counts and CRCs
  0x020 CYCLES     R   clock cycles from start until the last tile finished
  0x024 CLK_KHZ    R   fabric clock in kHz (build-time)
  0x028 NTILES     R   4
  0x040 + 0x10*t   R   TILE_W:    width of tile t
  0x044 + 0x10*t   R   OUT_COUNT: [15:0]=words in tile t's output FIFO, [16]=tile done
  0x048 + 0x10*t   R   BYTES:     encoded bytes produced by tile t
  0x04C + 0x10*t   R   CRC:       CRC-32 (zlib) of tile t's encoded bytes

  0x1000           W   PIX_IN: push one 32-bit word (four pixels)
  0x2000 + 0x400*t R   OUT tile t: pop one word, [15:0]=two stream bytes
                       (low byte first), [16]=last word of the stream
"""

from __future__ import annotations

NTILES = 4

REG_CTRL     = 0x000
REG_WIDTH    = 0x004
REG_HEIGHT   = 0x008
REG_STATUS   = 0x00C
REG_NEAR     = 0x010
REG_MAXTW    = 0x014
REG_FIFOD    = 0x018
REG_MODE     = 0x01C
REG_CYCLES   = 0x020
REG_CLK_KHZ  = 0x024
REG_NTILES   = 0x028
REG_TILE     = 0x040   # + 0x10 * t
TILE_W       = 0x0
TILE_COUNT   = 0x4
TILE_BYTES   = 0x8
TILE_CRC     = 0xC
PIX_IN_BASE  = 0x1000
OUT_BASE     = 0x2000  # + 0x400 * t

CTRL_SOFT_RESET = 1 << 0
CTRL_START      = 1 << 1
CTRL_DONE_CLEAR = 1 << 2
MODE_PATTERN    = 1 << 0

MIN_WIDTH = 20  # every tile at least 5 columns, the encoder's minimum


def tile_bounds(w: int) -> list[tuple[int, int]]:
    """Return [(first_column, width)] for the four tiles of a w-wide image."""
    wt = w // NTILES
    return [(t * wt, wt if t < NTILES - 1 else w - (NTILES - 1) * wt)
            for t in range(NTILES)]


def check_dims(w: int, h: int, max_tile_w: int) -> None:
    """Raise ValueError unless the hardware accepts a w x h frame."""
    last_w = tile_bounds(w)[-1][1] if w >= MIN_WIDTH else 0
    if w < MIN_WIDTH or last_w > max_tile_w or not 1 <= h <= 16384:
        raise ValueError(
            f"{w}x{h} unsupported: need width >= {MIN_WIDTH}, widest tile "
            f"<= {max_tile_w} (width up to about {NTILES * max_tile_w}), "
            "height 1..16384")


def words_per_row(w: int) -> int:
    return (w + 3) // 4


def pack_rows(w: int, h: int, pix: bytes) -> list[int]:
    """Pack raster pixels into PIX_IN words, each row padded to whole words."""
    wpr = words_per_row(w)
    words = []
    for y in range(h):
        row = pix[y * w:(y + 1) * w] + bytes(4 * wpr - w)
        words.extend(int.from_bytes(row[4 * k:4 * k + 4], "little") for k in range(wpr))
    return words


def pattern(np, w: int, h: int):
    """The on-chip test pattern as a (h, w) uint8 array.

    Must match pat_px() in axi_jls4_ctrl.v: a diagonal gradient with a
    small texture, 64x64 checker blocks, and flat 32x32 patches that put
    the encoder into run mode.
    """
    y, x = np.mgrid[0:h, 0:w].astype(np.int64)
    p = ((x >> 2) + (y >> 1) + ((x ^ y) & 7)) & 0xFF
    p ^= (((x >> 6) ^ (y >> 6)) & 1) * 0x40
    p = np.where((((x >> 5) & 7) == 1) & (((y >> 5) & 3) == 1), 0x80, p)
    return p.astype(np.uint8)


def split_tiles(img) -> list:
    """Split an (h, w) array into the four tile arrays."""
    return [img[:, x0:x0 + tw] for x0, tw in tile_bounds(img.shape[1])]
