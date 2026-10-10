#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
JPEG-LS encoder simulation runner.

Usage:
    python run_sim.py                        # NEAR=0, default random bubbles, self-check
    python run_sim.py --near 0 1 2           # multiple NEAR values
    python run_sim.py --bubble 0 -1 1 3      # multiple BUBBLE_CONTROL values (full matrix)
    python run_sim.py --max-width 16384 512  # also run with a narrower MAX_WIDTH (row cropping)
    python run_sim.py --no-check             # compress only, skip all checks
    python run_sim.py --ctrl --near 0 3      # only the Arty AXI wrapper testbench
    python run_sim.py --ctrl4 --near 0 3     # only the four-encoder (DE25 x4) wrapper testbench

Goldens are produced by CharLS (via the `imagecodecs` package), an independent
JPEG-LS implementation, so the check does not depend on the RTL under test.
The RTL stream must equal CharLS's output byte-for-byte from SOI through EOI
(CharLS's leading SPIFF header is dropped), plus one 0x00 pad byte when needed
to fill the encoder's final 16-bit output word. Each RTL output is also decoded
with CharLS and compared against the source pixels (|error| <= NEAR).

Requires: pip install imagecodecs numpy   (not needed with --no-check)
"""

import argparse
import os
import re
import subprocess
import sys
import zlib
from pathlib import Path

SIM_DIR    = Path(__file__).parent.resolve()
RTL_FILE   = SIM_DIR / ".." / "RTL" / "jls_encoder.v"
TB_FILE    = SIM_DIR / "tb_jls_encoder.v"
CTRL_TB    = SIM_DIR / "tb_axi_jls_ctrl.v"
CTRL_RTL   = [SIM_DIR / ".." / "example" / "arty_a7" / "rtl" / f
              for f in ("axi_jls_ctrl.v", "sync_fifo.v")]
CTRL_IMAGE = 8  # images/test008.pgm (205x191) fits the ctrl TB's MAX_WIDTH=256
CTRL4_TB   = SIM_DIR / "tb_axi_jls4_ctrl.v"
CTRL4_RTL  = [SIM_DIR / ".." / "example" / "de25_nano_x4" / "rtl" / "axi_jls4_ctrl.v",
              SIM_DIR / ".." / "example" / "arty_a7" / "rtl" / "sync_fifo.v"]
# Frames the ctrl4 TB runs: two images through PIX_IN (205 and 256 wide, so one
# with tiles off the 4-pixel word grid), then two frames of the test pattern.
CTRL4_IMAGES   = {"imgA": 8, "imgB": 9}
CTRL4_PATTERNS = {"pat1": (256, 72), "pat2": (250, 40)}
IMG_DIR    = SIM_DIR / "images"
SIM_OUT    = SIM_DIR / "sim.out"
GOLDEN_DIR = SIM_DIR / "golden"
OUT_DIR    = SIM_DIR / "out"
CONFIG_VH  = SIM_DIR / "sim_config.vh"

# Must match the testbench's supported-size filter
MIN_W, MAX_W_LIMIT, MAX_H_LIMIT = 5, 16384, 16384

IMAGECODECS_HINT = "pip install imagecodecs==2026.8.16 numpy"

# Common Windows install locations for iverilog
_IVERILOG_SEARCH = [
    r"C:\iverilog\bin",
    r"C:\Program Files\iverilog\bin",
    r"C:\msys64\mingw64\bin",
    r"C:\msys64\usr\bin",
]

_IVERILOG = None
_VVP      = None


def _find_tool(name):
    """Return the full Path to a tool, searching PATH then known locations."""
    for p in os.environ.get("PATH", "").split(os.pathsep):
        for suffix in ("", ".exe"):
            c = Path(p) / (name + suffix)
            if c.exists():
                return c
    for loc in _IVERILOG_SEARCH:
        for suffix in ("", ".exe"):
            c = Path(loc) / (name + suffix)
            if c.exists():
                print(f"[INFO] Found {name} at {c}", flush=True)
                return c
    raise FileNotFoundError(
        f"{name} not found. Install iverilog or add its bin directory to PATH.\n"
        "  Download: https://bleyer.org/icarus/"
    )


def _ensure_tools():
    global _IVERILOG, _VVP
    if _IVERILOG is None:
        _IVERILOG = _find_tool("iverilog")
        _VVP      = _find_tool("vvp")


def _import_codecs():
    try:
        import numpy as np
        import imagecodecs
    except ImportError:
        print(f"ERROR: the self-check needs CharLS via imagecodecs: {IMAGECODECS_HINT}\n"
              "       (or run with --no-check)", flush=True)
        sys.exit(1)
    if not imagecodecs.JPEGLS.available:
        print("ERROR: this imagecodecs build has no JPEG-LS (CharLS) codec", flush=True)
        sys.exit(1)
    return np, imagecodecs


# ---------------------------------------------------------------------------
# Images
# ---------------------------------------------------------------------------
def list_images():
    """Return {file_no: path} for images/testNNN.pgm, requiring 1..N contiguous."""
    imgs = {}
    for p in IMG_DIR.iterdir():
        m = re.fullmatch(r"test(\d{3})\.pgm", p.name)
        if m:
            imgs[int(m.group(1))] = p
    nos = sorted(imgs)
    if not nos or nos != list(range(1, len(nos) + 1)):
        raise RuntimeError(f"images must be test001.pgm..testNNN.pgm with no gaps, found {nos}")
    return imgs


def load_pgm(path):
    """Return (w, h, pixel bytes) for a binary P5 PGM with maxval 255."""
    data = path.read_bytes()
    fields, i = [], 0
    while len(fields) < 4:
        while data[i:i+1].isspace():
            i += 1
        if data[i:i+1] == b"#":
            while data[i:i+1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j+1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    i += 1  # single whitespace after maxval
    if fields[0] != b"P5" or int(fields[3]) != 255:
        raise ValueError(f"{path}: need a P5 PGM with maxval 255")
    w, h = int(fields[1]), int(fields[2])
    pix = data[i:i + w * h]
    if len(pix) != w * h:
        raise ValueError(f"{path}: truncated pixel data")
    return w, h, pix


def supported(w, h):
    return MIN_W <= w <= MAX_W_LIMIT and 1 <= h <= MAX_H_LIMIT


def expected_pixels(np, path, max_width):
    """Source pixels as the encoder should code them (rows cropped to MAX_WIDTH)."""
    w, h, pix = load_pgm(path)
    img = np.frombuffer(pix, np.uint8).reshape(h, w)
    return img[:, :min(w, max_width)]


# ---------------------------------------------------------------------------
# Goldens (CharLS) and decode check
# ---------------------------------------------------------------------------
SOI_SOF55 = bytes([0xFF, 0xD8, 0xFF, 0xF7])


def charls_stream(np, imagecodecs, img, near):
    """Expected encoder output for img: CharLS stream without SPIFF, padded to 16 bits."""
    enc = bytes(imagecodecs.jpegls_encode(np.ascontiguousarray(img), level=near,
                                          out=img.size * 2 + 4096))
    start = enc.find(SOI_SOF55)
    if start < 0:
        raise RuntimeError("CharLS output has no SOI+SOF55 sequence")
    stream = enc[start:]
    if len(stream) % 2:
        stream += b"\x00"
    return stream


def generate_golden(images, near, max_width, gdir):
    np, imagecodecs = _import_codecs()
    gdir.mkdir(parents=True, exist_ok=True)
    for old in gdir.glob("*.jls"):
        old.unlink()
    n = 0
    for no, path in sorted(images.items()):
        w, h, _ = load_pgm(path)
        if not supported(w, h):
            continue
        img = expected_pixels(np, path, max_width)
        (gdir / f"test{no:03d}.jls").write_bytes(charls_stream(np, imagecodecs, img, near))
        n += 1
    print(f"  CharLS goldens: {n} files in {gdir}", flush=True)
    return n


def decode_check(images, near, max_width, odir):
    """Decode every RTL output with CharLS and compare against the source pixels."""
    np, imagecodecs = _import_codecs()
    ok = True
    for no, path in sorted(images.items()):
        w, h, _ = load_pgm(path)
        if not supported(w, h):
            continue
        jls = odir / f"test{no:03d}.jls"
        if not jls.exists():
            print(f"  [DECODE FAIL] {jls.name}: missing", flush=True)
            ok = False
            continue
        ref = expected_pixels(np, path, max_width)
        try:
            dec = imagecodecs.jpegls_decode(jls.read_bytes())
        except Exception as e:  # noqa: BLE001 - report any decoder error as a failure
            print(f"  [DECODE FAIL] {jls.name}: {e}", flush=True)
            ok = False
            continue
        if dec.shape != ref.shape:
            print(f"  [DECODE FAIL] {jls.name}: shape {dec.shape} != {ref.shape}", flush=True)
            ok = False
            continue
        err = int(np.abs(dec.astype(np.int16) - ref.astype(np.int16)).max())
        if err > near:
            print(f"  [DECODE FAIL] {jls.name}: max |error| {err} > NEAR {near}", flush=True)
            ok = False
    if ok:
        print("  CharLS decode check: all outputs within NEAR", flush=True)
    return ok


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
def write_config(near, bubble, max_width, n_images, odir, gdir, check):
    """Write sim_config.vh so the testbench picks up the right settings."""
    lines = [
        "// Auto-generated by run_sim.py - do not edit by hand",
        f"`define NEAR           {near}",
        f"`define MAX_WIDTH      {max_width}",
        f"`define BUBBLE_CONTROL ({bubble})",
        "`define FILE_NO_FIRST  1",
        f"`define FILE_NO_FINAL  {n_images}",
        f'`define OUTPUT_JLS_DIR "{odir.as_posix()}"',
        f'`define GOLDEN_JLS_DIR "{gdir.as_posix()}"',
    ]
    if check:
        lines.append("`define SELF_CHECK")
    CONFIG_VH.write_text("\n".join(lines) + "\n")


def run_cmd(cmd, tag):
    print(f"\n[{tag}] {' '.join(str(c) for c in cmd)}", flush=True)
    print("-" * 60, flush=True)
    rc = subprocess.run([str(c) for c in cmd], cwd=SIM_DIR).returncode
    print("-" * 60, flush=True)
    return rc == 0


def run_case(images, near, bubble, max_width, check):
    _ensure_tools()
    gdir = GOLDEN_DIR / f"near{near}_w{max_width}"
    odir = OUT_DIR / f"near{near}_w{max_width}_b{bubble}"
    odir.mkdir(parents=True, exist_ok=True)
    for old in odir.glob("*.jls"):
        old.unlink()

    write_config(near, bubble, max_width, len(images), odir, gdir, check)
    if not run_cmd([_IVERILOG, "-g2001", "-o", SIM_OUT, "-D", "HAVE_SIM_CONFIG",
                    TB_FILE, RTL_FILE], "COMPILE"):
        print("ERROR: compilation failed", flush=True)
        return False
    if not run_cmd([_VVP, "-n", SIM_OUT], "SIM"):
        return False
    if check:
        return decode_check(images, near, max_width, odir)
    return True


def run_ctrl(images, near):
    """Arty AXI wrapper: dimension checks, write back-pressure, STATUS, output vs CharLS."""
    _ensure_tools()
    np, imagecodecs = _import_codecs()
    img_path = images[CTRL_IMAGE]
    gold = GOLDEN_DIR / f"ctrl_near{near}.jls"
    gold.parent.mkdir(parents=True, exist_ok=True)
    gold.write_bytes(charls_stream(np, imagecodecs, expected_pixels(np, img_path, MAX_W_LIMIT), near))
    CONFIG_VH.write_text("\n".join([
        "// Auto-generated by run_sim.py - do not edit by hand",
        f"`define NEAR        {near}",
        f'`define CTRL_IMG    "{img_path.as_posix()}"',
        f'`define CTRL_GOLDEN "{gold.as_posix()}"',
    ]) + "\n")
    if not run_cmd([_IVERILOG, "-g2001", "-o", SIM_OUT, "-D", "HAVE_SIM_CONFIG",
                    CTRL_TB, *CTRL_RTL, RTL_FILE], "COMPILE"):
        print("ERROR: compilation failed", flush=True)
        return False
    return run_cmd([_VVP, "-n", SIM_OUT], "SIM")


def run_ctrl4(images, near):
    """Four-encoder wrapper: every tile vs CharLS, and its BYTES/CRC registers vs the stream."""
    _ensure_tools()
    np, imagecodecs = _import_codecs()
    sys.path.insert(0, str(SIM_DIR / ".." / "example" / "common"))
    import jls4_demo

    odir = OUT_DIR / f"ctrl4_near{near}"
    odir.mkdir(parents=True, exist_ok=True)
    for old in odir.iterdir():
        old.unlink()
    (pw1, ph1), (pw2, ph2) = CTRL4_PATTERNS["pat1"], CTRL4_PATTERNS["pat2"]
    CONFIG_VH.write_text("\n".join([
        "// Auto-generated by run_sim.py - do not edit by hand",
        f"`define NEAR    {near}",
        f'`define IMG_A   "{images[CTRL4_IMAGES["imgA"]].as_posix()}"',
        f'`define IMG_B   "{images[CTRL4_IMAGES["imgB"]].as_posix()}"',
        f"`define PAT1_W  {pw1}",
        f"`define PAT1_H  {ph1}",
        f"`define PAT2_W  {pw2}",
        f"`define PAT2_H  {ph2}",
        f'`define OUT_DIR "{odir.as_posix()}"',
    ]) + "\n")
    if not run_cmd([_IVERILOG, "-g2001", "-o", SIM_OUT, "-D", "HAVE_SIM_CONFIG",
                    CTRL4_TB, *CTRL4_RTL, RTL_FILE], "COMPILE"):
        print("ERROR: compilation failed", flush=True)
        return False
    if not run_cmd([_VVP, "-n", SIM_OUT], "SIM"):
        return False

    frames = {name: expected_pixels(np, images[no], MAX_W_LIMIT)
              for name, no in CTRL4_IMAGES.items()}
    frames.update({name: jls4_demo.pattern(np, pw, ph)
                   for name, (pw, ph) in CTRL4_PATTERNS.items()})
    ok = True
    for name, img in frames.items():
        regs = {}
        for line in (odir / f"{name}_regs.txt").read_text().splitlines():
            f = line.split()
            if f[0] == "tile":
                regs[int(f[1])] = dict(width=int(f[3]), bytes=int(f[5]), crc=int(f[7]))
            else:
                cycles = int(f[1])
        for t, tile in enumerate(jls4_demo.split_tiles(img)):
            got = (odir / f"{name}_tile{t}.jls").read_bytes()
            want = charls_stream(np, imagecodecs, tile, near)
            r = regs[t]
            checks = [
                (got == want, f"stream differs from CharLS ({len(got)} vs {len(want)} bytes)"),
                (r["width"] == tile.shape[1], f"TILE_W {r['width']} != {tile.shape[1]}"),
                (r["bytes"] == len(got), f"BYTES {r['bytes']} != {len(got)}"),
                (r["crc"] == zlib.crc32(got), f"CRC {r['crc']:08X} != {zlib.crc32(got):08X}"),
            ]
            bad = [msg for good, msg in checks if not good]
            tag = f"{name} tile {t} ({tile.shape[1]}x{tile.shape[0]}, {len(got)} bytes)"
            print(f"  [{'FAIL' if bad else 'PASS'}] {tag}" + (": " + "; ".join(bad) if bad else ""),
                  flush=True)
            ok &= not bad
        ideal = max(tw for _, tw in jls4_demo.tile_bounds(img.shape[1])) * img.shape[0]
        print(f"  {name}: {cycles} cycles for {img.shape[1]}x{img.shape[0]} "
              f"(widest tile x height = {ideal})", flush=True)
    return ok


def main():
    parser = argparse.ArgumentParser(description="JPEG-LS encoder simulation runner")
    parser.add_argument(
        "--near", type=int, nargs="+", default=[0], metavar="N",
        help="NEAR value(s) 0-7 to test (default: 0)",
    )
    parser.add_argument(
        "--bubble", type=int, nargs="+", default=[-1], metavar="B",
        help="BUBBLE_CONTROL value(s) (default: -1 = random bubbles). "
             "Multiple values run the full (near x bubble) matrix.",
    )
    parser.add_argument(
        "--max-width", type=int, nargs="+", default=[16384], metavar="W",
        help="jls_encoder MAX_WIDTH value(s) (default: 16384). Images wider "
             "than W are expected to be cropped to their first W columns.",
    )
    parser.add_argument(
        "--no-check", action="store_true",
        help="Compress only, no golden or decode check",
    )
    parser.add_argument(
        "--ctrl", action="store_true",
        help="Run only the Arty AXI wrapper testbench (tb_axi_jls_ctrl.v) for each NEAR",
    )
    parser.add_argument(
        "--ctrl4", action="store_true",
        help="Run only the four-encoder wrapper testbench (tb_axi_jls4_ctrl.v) for each NEAR",
    )
    args = parser.parse_args()

    for n in args.near:
        if not 0 <= n <= 7:
            parser.error(f"NEAR={n} is out of range [0, 7]")
    for mw in args.max_width:
        if not MIN_W <= mw <= MAX_W_LIMIT:
            parser.error(f"MAX_WIDTH={mw} is out of range [{MIN_W}, {MAX_W_LIMIT}]")

    images = list_images()
    print(f"[INFO] {len(images)} images in {IMG_DIR}", flush=True)

    overall_pass = True
    results = []  # (near, max_width, bubble, status)

    if args.ctrl:
        for near in args.near:
            print(f"\n--- AXI ctrl testbench  NEAR={near} ---", flush=True)
            ok = run_ctrl(images, near)
            results.append((near, "ctrl", "-", "PASS" if ok else "FAIL"))
            overall_pass &= ok

    if args.ctrl4:
        for near in args.near:
            print(f"\n--- four-encoder ctrl testbench  NEAR={near} ---", flush=True)
            ok = run_ctrl4(images, near)
            results.append((near, "ctrl4", "-", "PASS" if ok else "FAIL"))
            overall_pass &= ok

    for max_width in ([] if args.ctrl or args.ctrl4 else args.max_width):
        for near in args.near:
            print(f"\n{'='*60}", flush=True)
            print(f"  NEAR = {near}   MAX_WIDTH = {max_width}", flush=True)
            print(f"{'='*60}", flush=True)

            # Goldens depend on NEAR and MAX_WIDTH only, not on bubble timing.
            if not args.no_check:
                generate_golden(images, near, max_width, GOLDEN_DIR / f"near{near}_w{max_width}")

            for bubble in args.bubble:
                print(f"\n--- NEAR={near}  MAX_WIDTH={max_width}  BUBBLE={bubble} ---", flush=True)
                ok = run_case(images, near, bubble, max_width, check=not args.no_check)
                results.append((near, max_width, bubble, "PASS" if ok else "FAIL"))
                overall_pass &= ok

    # Cleanup
    if SIM_OUT.exists():
        SIM_OUT.unlink()
    if CONFIG_VH.exists():
        CONFIG_VH.unlink()

    print(f"\n{'='*60}", flush=True)
    print("  REGRESSION MATRIX SUMMARY", flush=True)
    print(f"{'='*60}", flush=True)
    for near, max_width, bubble, status in results:
        print(f"  NEAR={near}  MAX_WIDTH={max_width!s:>5}  BUBBLE={bubble!s:>3}  ->  {status}", flush=True)
    print(f"{'='*60}", flush=True)
    print(f"  RESULT: {'ALL PASSED' if overall_pass else 'FAILED'}", flush=True)
    print(f"{'='*60}\n", flush=True)

    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
