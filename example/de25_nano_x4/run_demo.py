#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
DE25-Nano (Agilex 5) four-encoder JPEG-LS demo runner.

Four jls_encoder instances encode the four vertical tiles of an image in
parallel (tiling, packing and register map: example/common/jls4_demo.py).

  --image a.pgm ...   send each image over the fcapz EJTAG-AXI bridge, read
                      back the four tile streams, and check each one
                      byte-for-byte against CharLS's encoding of that tile;
                      the four decoded tiles must also reassemble the image.
                      JTAG limits this to about 50-60 bytes/s: it checks
                      correctness, not speed.
  --pattern [WxH]     encode the on-chip test pattern (default 3840x2160) at
                      full speed, then compare each tile's byte count and
                      CRC-32 with CharLS's encoding of the same pattern, and
                      report the measured cycles as throughput and fps.

Exits non-zero if any check fails.

Environment:
    QUARTUS_ROOTDIR  Quartus install (tools from its bin64 or bin), else PATH
    JLS_JTAG_CABLE   quartus_pgm / quartus_stp cable name, e.g.
                     "DE25-Nano [USB-1]" (default: the only cable present)

Usage:
    python run_demo.py --build                       # synthesis + fit + .sof
    python run_demo.py --pattern                     # 3840x2160 at full speed
    python run_demo.py --pattern 1920x1080 --image ../../SIM/images/test008.pgm
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
import zlib
from pathlib import Path

HERE      = Path(__file__).parent.resolve()
JLS_ROOT  = HERE.parent.parent
SOF_FILE  = HERE / "de25_jls4_top.sof"
BUILD_TCL = HERE / "build.tcl"
CABLE     = os.environ.get("JLS_JTAG_CABLE") or None

sys.path.insert(0, str(JLS_ROOT / "example" / "common"))
import jls_demo            # noqa: E402
import jls4_demo as j4     # noqa: E402

BATCH_MAX = 256  # words per JTAG batch; PIX_IN accepts up to 1024 in one burst


def sh(cmd, **kw):
    print(f"$ {' '.join(str(c) for c in cmd)}", flush=True)
    return subprocess.run(cmd, check=False, **kw)


def quartus_tool(name: str) -> str:
    root = os.environ.get("QUARTUS_ROOTDIR")
    if root:
        # bin64 on Windows, bin on Linux
        for sub in ("bin64", "bin"):
            for exe in (f"{name}.exe", name):
                cand = Path(root) / sub / exe
                if cand.is_file():
                    return str(cand)
    found = shutil.which(name)
    if not found:
        raise RuntimeError(f"{name} not found: set QUARTUS_ROOTDIR or add Quartus bin64 to PATH")
    return found


# ---------------------------------------------------------------------------
# Build and bridge
# ---------------------------------------------------------------------------
def build_bitstream():
    r = sh([quartus_tool("quartus_sh"), "-t", str(BUILD_TCL)], cwd=HERE)
    if r.returncode != 0:
        raise RuntimeError("Quartus build failed")
    if not SOF_FILE.exists():
        raise RuntimeError(f"Build completed but {SOF_FILE} not found")


def program_fpga():
    if not SOF_FILE.exists():
        raise RuntimeError(f"{SOF_FILE} not found, build it with --build")
    cmd = [quartus_tool("quartus_pgm"), "-m", "JTAG"]
    if CABLE:
        cmd += ["-c", CABLE]
    cmd += ["-o", f"p;{SOF_FILE}@1"]
    if sh(cmd).returncode != 0:
        raise RuntimeError("quartus_pgm failed to program the FPGA")
    time.sleep(0.5)  # let the virtual JTAG hub come up before the first scan


def open_bridge(program: bool):
    from fcapz.transport import QuartusStpTransport
    from fcapz.ejtagaxi import EjtagAxiController

    if program:
        program_fpga()
    transport = QuartusStpTransport(
        hardware_name=CABLE,
        quartus_stp_path=quartus_tool("quartus_stp"),
    )
    # sld_virtual_jtag instance 4, as in fpgacapZero's DE25-Nano example
    return EjtagAxiController(transport, chain=4)


# ---------------------------------------------------------------------------
# Register helpers
# ---------------------------------------------------------------------------
def tile_reg(bridge, t: int, field: int) -> int:
    return bridge.axi_read(j4.REG_TILE + 0x10 * t + field)


def status(bridge) -> dict:
    s = bridge.axi_read(j4.REG_STATUS)
    return dict(busy=bool(s & 1), done=bool(s & 2), in_full=bool(s & 4),
                out_overflow=bool(s & 0x10), dim_error=bool(s & 0x20),
                in_count=(s >> 8) & 0xFFF)


def start_frame(bridge, w: int, h: int, mode: int, info: dict):
    j4.check_dims(w, h, info["max_tile_w"])
    bridge.axi_write(j4.REG_CTRL, j4.CTRL_SOFT_RESET)
    bridge.axi_write(j4.REG_MODE, mode)
    bridge.axi_write(j4.REG_WIDTH, w)
    bridge.axi_write(j4.REG_HEIGHT, h)
    bridge.axi_write(j4.REG_CTRL, j4.CTRL_START)
    st = status(bridge)
    if st["dim_error"] or not (st["busy"] or st["done"]):
        raise RuntimeError(f"{w}x{h} frame not started: {st}")


def tile_results(bridge) -> list[dict]:
    return [dict(width=tile_reg(bridge, t, j4.TILE_W),
                 bytes=tile_reg(bridge, t, j4.TILE_BYTES),
                 crc=tile_reg(bridge, t, j4.TILE_CRC)) for t in range(j4.NTILES)]


def read_info(bridge) -> dict:
    fifo_d = bridge.axi_read(j4.REG_FIFOD)
    info = dict(near=bridge.axi_read(j4.REG_NEAR) & 7,
                max_tile_w=bridge.axi_read(j4.REG_MAXTW) & 0xFFFF,
                in_depth=fifo_d & 0xFFFF,
                ntiles=bridge.axi_read(j4.REG_NTILES),
                clk_khz=bridge.axi_read(j4.REG_CLK_KHZ))
    if info["ntiles"] != j4.NTILES or not 5 <= info["max_tile_w"] <= 4096:
        raise RuntimeError(f"unexpected register values {info}: is the four-encoder "
                           "bitstream loaded? (rebuild with --build)")
    print(f"[axi] NEAR={info['near']} MAX_TILE_W={info['max_tile_w']} "
          f"in_fifo={info['in_depth']} words, clock {info['clk_khz'] / 1000:g} MHz")
    return info


# ---------------------------------------------------------------------------
# Image mode: pixels in and four streams out over JTAG
# ---------------------------------------------------------------------------
def encode_image(bridge, info: dict, pgm: Path) -> list[bytes]:
    w, h, pix = jls_demo.load_pgm(pgm)
    print(f"[pgm] {pgm}: {w}x{h}, tiles {j4.tile_bounds(w)}")
    start_frame(bridge, w, h, 0, info)
    words = j4.pack_rows(w, h, pix)
    streams = [bytearray() for _ in range(j4.NTILES)]
    last = [False] * j4.NTILES
    t0 = time.time()

    def drain():
        # Every tile must be drained: one stalled tile stops the row writer.
        for t in range(j4.NTILES):
            if last[t]:
                continue
            n = tile_reg(bridge, t, j4.TILE_COUNT) & 0xFFFF
            for _ in range(min(n, BATCH_MAX)):
                w32 = bridge.axi_read(j4.OUT_BASE + 0x400 * t)
                streams[t] += (w32 & 0xFFFF).to_bytes(2, "little")
                if (w32 >> 16) & 1:
                    last[t] = True
                    break

    k = 0
    while k < len(words):
        st = status(bridge)
        if st["out_overflow"]:
            raise RuntimeError(f"output FIFO overflow: {st}")
        free = info["in_depth"] - st["in_count"]
        if free > 0:
            batch = words[k:k + min(free, BATCH_MAX)]
            bridge.write_block(j4.PIX_IN_BASE, batch)
            k += len(batch)
        drain()

    deadline = time.time() + 60.0
    while not all(last):
        drain()
        if time.time() > deadline:
            raise RuntimeError(f"timeout waiting for the last tile words, done={last}")

    dt = time.time() - t0
    total = sum(len(s) for s in streams)
    print(f"[axi] {w}x{h}: {total} bytes in four tiles, {dt:.1f} s over JTAG")

    regs = tile_results(bridge)
    for t, (s, r) in enumerate(zip(streams, regs)):
        if r["bytes"] != len(s) or r["crc"] != zlib.crc32(s):
            raise RuntimeError(f"tile {t}: BYTES/CRC registers {r} do not match the "
                               f"{len(s)} bytes read back")
    return [bytes(s) for s in streams]


def verify_image(np, imagecodecs, rs, pgm: Path, streams: list[bytes], near: int) -> bool:
    w, h, pix = jls_demo.load_pgm(pgm)
    img = np.frombuffer(pix, np.uint8).reshape(h, w)
    ok = True
    decoded = []
    for t, (tile, got) in enumerate(zip(j4.split_tiles(img), streams)):
        want = rs.charls_stream(np, imagecodecs, tile, near)
        same = got == want
        print(f"[verify] tile {t} ({tile.shape[1]}x{h}): "
              + ("matches CharLS byte-for-byte" if same else
                 f"FAIL, differs from CharLS ({len(got)} vs {len(want)} bytes)"))
        ok &= same
        decoded.append(imagecodecs.jpegls_decode(got))
    full = np.hstack(decoded)
    err = int(np.abs(full.astype(np.int16) - img.astype(np.int16)).max()) \
        if full.shape == img.shape else None
    if err is None or err > near:
        print(f"[verify] FAIL: reassembled image shape {full.shape}, max |error| {err}")
        return False
    print(f"[verify] reassembled {w}x{h} image decodes within NEAR={near} (max |error| {err})")
    return ok


# ---------------------------------------------------------------------------
# Pattern mode: full-speed run, checked by byte count and CRC
# ---------------------------------------------------------------------------
def run_pattern(bridge, info: dict, np, imagecodecs, rs, w: int, h: int, verify: bool) -> bool:
    start_frame(bridge, w, h, j4.MODE_PATTERN, info)
    while not status(bridge)["done"]:
        time.sleep(0.01)
    cycles = bridge.axi_read(j4.REG_CYCLES)
    regs = tile_results(bridge)

    clk_hz = info["clk_khz"] * 1000
    widest = max(tw for _, tw in j4.tile_bounds(w))
    print(f"[pattern] {w}x{h}: {cycles} cycles (widest tile x height = {widest * h}, "
          f"+{cycles - widest * h} for SOF and pipeline)")
    if clk_hz:
        secs = cycles / clk_hz
        print(f"[pattern] at {clk_hz / 1e6:g} MHz: {secs * 1e3:.2f} ms per frame, "
              f"{w * h / secs / 1e6:.0f} Mpixel/s, {1 / secs:.1f} fps")
    total = sum(r["bytes"] for r in regs)
    print(f"[pattern] {total} bytes in four tiles ({w * h / total:.2f}:1)")
    if not verify:
        return True

    ok = True
    for t, (tile, r) in enumerate(zip(j4.split_tiles(j4.pattern(np, w, h)), regs)):
        want = rs.charls_stream(np, imagecodecs, tile, info["near"])
        good = (r["width"] == tile.shape[1] and r["bytes"] == len(want)
                and r["crc"] == zlib.crc32(want))
        print(f"[verify] tile {t} ({tile.shape[1]}x{h}): {r['bytes']} bytes, CRC {r['crc']:08X} "
              + ("match CharLS" if good else
                 f"FAIL, CharLS has {len(want)} bytes, CRC {zlib.crc32(want):08X}"))
        ok &= good
    return ok


# ---------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------
def parse_size(s: str) -> tuple[int, int]:
    try:
        w, h = (int(v) for v in s.lower().split("x"))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected WxH, got {s!r}") from None
    return w, h


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--build", action="store_true", help="Synthesize and write the bitstream first")
    ap.add_argument("--image", type=Path, nargs="+", help=".pgm image(s) to encode over JTAG")
    ap.add_argument("--pattern", type=parse_size, nargs="?", const=(3840, 2160), metavar="WxH",
                    help="encode the on-chip test pattern at full speed (default 3840x2160)")
    ap.add_argument("--out", type=Path, default=None,
                    help="directory for the tile streams of --image (default: next to each image)")
    ap.add_argument("--no-program", action="store_true",
                    help="Skip bitstream programming (assume FPGA already loaded)")
    ap.add_argument("--no-verify", action="store_true", help="Skip the CharLS checks")
    args = ap.parse_args()

    if not (args.build or args.image or args.pattern):
        ap.error("supply --build, --image, --pattern, or a combination")
    if args.build:
        build_bitstream()
    if not (args.image or args.pattern):
        return
    for img in args.image or []:
        if not img.is_file():
            ap.error(f"image not found: {img}")

    rs = jls_demo._load_run_sim()
    np, imagecodecs = rs._import_codecs()
    bridge = open_bridge(not args.no_program)
    print(f"[axi] connected: {bridge.connect()}")
    results = []
    try:
        info = read_info(bridge)
        for img in args.image or []:
            try:
                streams = encode_image(bridge, info, img)
                out_dir = args.out or img.parent
                out_dir.mkdir(parents=True, exist_ok=True)
                for t, s in enumerate(streams):
                    (out_dir / f"{img.stem}_tile{t}.jls").write_bytes(s)
                ok = True if args.no_verify else verify_image(
                    np, imagecodecs, rs, img, streams, info["near"])
            except Exception as e:  # noqa: BLE001 - report and continue
                print(f"[error] {img}: {e}")
                ok = False
            results.append((str(img), ok))
        if args.pattern:
            w, h = args.pattern
            try:
                ok = run_pattern(bridge, info, np, imagecodecs, rs, w, h, not args.no_verify)
            except Exception as e:  # noqa: BLE001
                print(f"[error] pattern {w}x{h}: {e}")
                ok = False
            results.append((f"pattern {w}x{h}", ok))
    finally:
        try:
            bridge.close()
        except Exception as e:  # noqa: BLE001
            print(f"[warn] bridge.close failed: {e}")

    print()
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    failed = sum(1 for _, ok in results if not ok)
    print(f"  {len(results) - failed}/{len(results)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
