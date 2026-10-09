#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
Generate the synthetic regression images that are not natural photos.

test023.pgm : single-row lossless worst case for the context accumulator A.
              Every regular-mode pixel lands in the same context and produces
              |Errval| = 128, so A reaches 4 + 64*128 = 8196 at the first
              RESET halving.  That exceeds the 13-bit A register, which
              silently corrupted the stream before the A_update fix.

Usage:
    python gen_test_images.py            # (re)write the images into ./images
"""

from pathlib import Path

IMG_DIR = Path(__file__).parent.resolve() / "images"

# JPEG-LS defaults for 8-bit lossless (ITU-T T.87 C.2.4.1.1)
T3    = 21
RESET = 64


def write_pgm(path, w, h, pix):
    path.write_bytes(f"P5\n{w} {h}\n255\n".encode() + bytes(pix))


def accumulator_overflow_row(updates=96):
    """
    Single-row image. On the first row b = c = d = 0, so every pixel whose
    left neighbour a >= T3 maps to the same context (Q1 = Q2 = 0, Q3 = -4,
    SIGN = -1) with MED prediction Px = a. Each such pixel is chosen as
    Px + 128 (mod 256), so its modulo-reduced error is always -128 and the
    first 64 updates push A to 4 + 64*128 = 8196 right at the RESET halving.
    When that pixel would drop below T3 (taking the next pixel out of the
    context), a filler pixel of 255 is inserted; it is coded in another
    context and does not disturb the target context's A/B/C/N.
    Returns the pixels and the largest pre-halving A value seen.
    """
    A, B, C, N = 4, 0, 0, 1
    a_max = 0
    pix = [255]  # first pixel: run interruption out of an all-zero context
    for _ in range(updates):
        if pix[-1] < T3:
            pix.append(255)
        a = pix[-1]
        px = min(max(a - C, 0), 255)  # SIGN = -1: Px -= C
        x = (px + 128) & 0xFF
        err = -(x - px)  # SIGN = -1 flips the error
        if err < 0:
            err += 256
        if err >= 128:
            err -= 256
        B += err
        A += abs(err)
        a_max = max(a_max, A)
        if N == RESET:
            A >>= 1
            B >>= 1
            N >>= 1
        N += 1
        if B <= -N:
            B += N
            C = max(C - 1, -128)
            if B <= -N:
                B = -N + 1
        elif B > 0:
            B -= N
            C = min(C + 1, 127)
            if B > 0:
                B = 0
        pix.append(x)
    return pix, a_max


def main():
    pix, a_max = accumulator_overflow_row()
    assert a_max > 8191, f"generator no longer overflows a 13-bit A (max {a_max})"
    out = IMG_DIR / "test023.pgm"
    write_pgm(out, len(pix), 1, pix)
    print(f"wrote {out} ({len(pix)}x1, peak A before halving = {a_max})")


if __name__ == "__main__":
    main()
