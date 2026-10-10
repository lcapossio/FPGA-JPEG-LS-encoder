# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
Board-independent host flow for the JPEG-LS encoder hardware demos.

The board scripts (example/arty_a7/run_demo.py, example/de25_nano/run_demo.py)
supply how to build the bitstream and how to open the fcapz EJTAG-AXI bridge;
everything else lives here, against the axi_jls_ctrl register map:

  1. load a .pgm image
  2. push pixels into the encoder over EJTAG-AXI (fpgacapZero)
  3. drain the encoded JPEG-LS byte stream over EJTAG-AXI
  4. verify the stream against CharLS (imagecodecs): byte-for-byte against
     CharLS's own encoding of the image at the bitstream's NEAR, plus a CharLS
     decode checked to be within NEAR of the source pixels
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time
from pathlib import Path

JLS_ROOT       = Path(__file__).resolve().parent.parent.parent
RUN_SIM        = JLS_ROOT / "SIM" / "run_sim.py"
FCAPZ_ROOT     = Path(
    os.environ.get("FCAPZ_ROOT", str(JLS_ROOT / "fcapz"))
).resolve()

# -- Register map (must match axi_jls_ctrl.v) --
REG_CTRL    = 0x0000
REG_WIDTH   = 0x0004
REG_HEIGHT  = 0x0008
REG_STATUS  = 0x000C
REG_NEAR    = 0x0010
REG_MAXW    = 0x0014
REG_FIFOD   = 0x0018
PIX_IN_BASE = 0x1000
OUT_BASE    = 0x2000

CTRL_SOFT_RESET = 1 << 0
CTRL_SOF        = 1 << 1
CTRL_DONE_CLEAR = 1 << 2

BATCH_MAX = 256  # chunk pushes/pulls this many words per JTAG batch


# ---------------------------------------------------------------------------
# PGM load
# ---------------------------------------------------------------------------
def load_pgm(path: Path):
    """Return (width, height, bytes) for a P5 (binary) PGM with depth 255."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path}: not a P5 (binary) PGM")

        def read_int():
            while True:
                line = f.readline()
                if not line:
                    raise ValueError(f"{path}: unexpected EOF in header")
                line = line.split(b"#", 1)[0].strip()
                if line:
                    return line

        hdr = b""
        while hdr.count(b" ") + hdr.count(b"\n") < 2:
            hdr += read_int() + b" "
        parts = hdr.split()
        w, h = int(parts[0]), int(parts[1])

        depth_line = read_int()
        depth = int(depth_line.split()[0])
        if depth != 255:
            raise ValueError(f"{path}: depth {depth} unsupported (need 255)")

        data = f.read(w * h)
        if len(data) != w * h:
            raise ValueError(
                f"{path}: expected {w*h} pixel bytes, got {len(data)}"
            )
    return w, h, data


# ---------------------------------------------------------------------------
# AXI helpers
# ---------------------------------------------------------------------------
def decode_status(s: int):
    return dict(
        busy         = bool(s & 0x1),
        done         = bool(s & 0x2),
        in_full      = bool(s & 0x4),
        out_empty    = bool(s & 0x8),
        out_overflow = bool(s & 0x10),
        dim_error    = bool(s & 0x20),
        in_count     = (s >> 8)  & 0xFFF,   # [19:8]
        out_count    = (s >> 20) & 0xFFF,   # [31:20]
    )


def format_status(st: dict[str, int | bool]) -> str:
    return (
        "busy={busy} done={done} in_full={in_full} out_empty={out_empty} "
        "in_count={in_count} out_count={out_count} out_overflow={out_overflow} "
        "dim_error={dim_error}"
    ).format(**st)


# ---------------------------------------------------------------------------
# Encode flow
# ---------------------------------------------------------------------------
def encode_image(pgm_path: Path, out_jls: Path, open_bridge, program: bool) -> int:
    """Encode pgm_path on the board, write the stream to out_jls, return NEAR.

    open_bridge(program) returns an unconnected fcapz EjtagAxiController for
    the board, programming the FPGA first when program is true.
    """
    w, h, pix = load_pgm(pgm_path)
    print(f"[pgm] {pgm_path}: {w}x{h} ({len(pix)} bytes)")

    try:
        bridge = open_bridge(program)
    except ImportError:
        print("ERROR: fpgacapZero host stack not importable.", file=sys.stderr)
        print(f"  Install with: pip install -e {FCAPZ_ROOT}", file=sys.stderr)
        raise

    info = bridge.connect()
    print(f"[axi] connected: {info}")
    t0 = time.time()

    try:
        # -- Soft reset -----------------------------------------------------
        bridge.axi_write(REG_CTRL, CTRL_SOFT_RESET)
        time.sleep(0.01)
        bridge.axi_write(REG_CTRL, 0)

        # -- Build-time limits ------------------------------------------------
        max_w    = bridge.axi_read(REG_MAXW) & 0xFFFF
        fifo_d   = bridge.axi_read(REG_FIFOD)
        in_depth = fifo_d & 0xFFFF
        near     = bridge.axi_read(REG_NEAR) & 0x7
        print(f"[axi] NEAR={near} MAX_WIDTH={max_w} "
              f"in_fifo={in_depth} out_fifo={fifo_d >> 16}")
        if not (5 <= max_w <= 16384 and 1 <= in_depth <= 2048):
            raise RuntimeError(
                "MAX_WIDTH/FIFO_DEPTH registers read back invalid values: the "
                "bitstream predates this register map, rebuild it with --build"
            )
        if not (5 <= w <= max_w and 1 <= h <= 16384):
            raise ValueError(
                f"Image size {w}x{h} unsupported by this bitstream "
                f"(width 5..{max_w}, height 1..16384)"
            )

        # -- Width / height --------------------------------------------------
        bridge.axi_write(REG_WIDTH,  w)
        bridge.axi_write(REG_HEIGHT, h)
        rw = bridge.axi_read(REG_WIDTH)  & 0x7FFF
        rh = bridge.axi_read(REG_HEIGHT) & 0x7FFF
        if rw != w or rh != h:
            raise RuntimeError(f"W/H readback mismatch: {rw}x{rh} != {w}x{h}")

        # -- SOF strobe ------------------------------------------------------
        bridge.axi_write(REG_CTRL, CTRL_SOF)
        st = decode_status(bridge.axi_read(REG_STATUS))
        if st["dim_error"]:
            raise RuntimeError(f"Encoder rejected {w}x{h}: {format_status(st)}")

        # -- Push pixels while draining output in-between -------------------
        out_bytes = bytearray()
        got_last  = False
        pix_idx   = 0
        N         = len(pix)

        def drain_some():
            """Drain everything currently in the out FIFO."""
            nonlocal got_last
            while not got_last:
                st = decode_status(bridge.axi_read(REG_STATUS))
                if st["out_overflow"]:
                    raise RuntimeError(
                        "Output FIFO overflow detected before drain: "
                        f"{format_status(st)}"
                    )
                n = st["out_count"]
                if n == 0:
                    return
                # Read words one at a time via WRITE_INC-style reads. burst_read
                # showed bridge-state issues at >9 beats on this slave; use
                # single-word reads which are proven reliable.
                for batch_idx in range(min(n, BATCH_MAX)):
                    out_word_idx = len(out_bytes) // 2
                    try:
                        w32 = bridge.axi_read(OUT_BASE)
                    except Exception as e:
                        try:
                            post_st = decode_status(bridge.axi_read(REG_STATUS))
                            post_status_msg = format_status(post_st)
                        except Exception as status_err:
                            post_status_msg = f"<status read failed: {status_err}>"
                        raise RuntimeError(
                            "OUT_BASE read failed at "
                            f"word={out_word_idx} byte_off={len(out_bytes)} "
                            f"batch_idx={batch_idx} "
                            f"pre_status=({format_status(st)}) "
                            f"post_status=({post_status_msg}): {e}"
                        ) from e
                    data = w32 & 0xFFFF
                    last = bool((w32 >> 16) & 1)
                    out_bytes.append(data & 0xFF)
                    out_bytes.append((data >> 8) & 0xFF)
                    if last:
                        got_last = True
                        break

        # Push pixels via write_block (AXI WRITE_INC; auto-increments address).
        # Every word lands in the 0x1000 pix-in window and pushes 1 pixel.
        while pix_idx < N:
            st = decode_status(bridge.axi_read(REG_STATUS))
            if st["out_overflow"]:
                raise RuntimeError(
                    "Output FIFO overflow detected during input push: "
                    f"{format_status(st)}"
                )
            free = in_depth - st["in_count"]
            if free <= 0:
                drain_some()
                continue
            batch = min(N - pix_idx, free, BATCH_MAX)
            chunk = [pix[pix_idx + i] for i in range(batch)]
            bridge.write_block(PIX_IN_BASE, chunk, wstrb=0x1)
            pix_idx += batch
            drain_some()

        # All pixels queued — wait for completion
        deadline = time.time() + 60.0  # generous; 256x256 over JTAG ~ minutes
        while not got_last and time.time() < deadline:
            drain_some()
            if got_last:
                break
            st = decode_status(bridge.axi_read(REG_STATUS))
            if st["out_overflow"]:
                raise RuntimeError(
                    "Output FIFO overflow detected while waiting for completion: "
                    f"{format_status(st)}"
                )
            if st["done"] and st["out_count"] == 0:
                got_last = True
                break
            time.sleep(0.005)

        if not got_last:
            raise RuntimeError("Timeout waiting for o_last")

        dt = time.time() - t0
        print(f"[axi] encoded {N}B -> {len(out_bytes)}B in {dt:.2f}s "
              f"({N/dt:.0f} B/s in)")
        out_jls.write_bytes(out_bytes)
        print(f"[jls] wrote {out_jls} ({len(out_bytes)} bytes)")
        return near
    finally:
        try:
            bridge.close()
        except Exception as e:
            print(f"[warn] bridge.close failed: {e}")


# ---------------------------------------------------------------------------
# Verify against CharLS
# ---------------------------------------------------------------------------
def _load_run_sim():
    """Reuse the simulation flow's CharLS helpers (SIM/run_sim.py)."""
    spec = importlib.util.spec_from_file_location("run_sim", RUN_SIM)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def verify_charls(jls_path: Path, src_pgm: Path, near: int) -> bool:
    rs = _load_run_sim()
    np, imagecodecs = rs._import_codecs()
    got = jls_path.read_bytes()
    img = rs.expected_pixels(np, src_pgm, rs.MAX_W_LIMIT)
    ok = True

    want = rs.charls_stream(np, imagecodecs, img, near)
    if got == want:
        print(f"[verify] stream matches CharLS byte-for-byte ({len(got)} bytes, NEAR={near})")
    else:
        n = min(len(got), len(want))
        first = next((i for i in range(n) if got[i] != want[i]), n)
        print(f"[verify] FAIL: stream differs from CharLS at byte {first} "
              f"(got {len(got)} bytes, CharLS {len(want)} bytes)")
        ok = False

    try:
        dec = imagecodecs.jpegls_decode(got)
    except Exception as e:  # noqa: BLE001 - any decoder error is a failure
        print(f"[verify] FAIL: CharLS cannot decode the stream: {e}")
        return False
    if dec.shape != img.shape:
        print(f"[verify] FAIL: decoded shape {dec.shape} != {img.shape}")
        return False
    err = int(np.abs(dec.astype(np.int16) - img.astype(np.int16)).max())
    if err > near:
        print(f"[verify] FAIL: max |error| {err} > NEAR {near}")
        return False
    print(f"[verify] CharLS decode OK: max |error| {err} <= NEAR {near}")
    return ok


# ---------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------
def main(build, open_bridge, description: str | None = None):
    """Command-line flow shared by the board scripts.

    build() builds the bitstream; open_bridge(program) is passed to
    encode_image().
    """
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("--build",       action="store_true",
                    help="Synthesize and write bitstream before running")
    ap.add_argument("--image",       type=Path, nargs="+",
                    help=".pgm image(s) to encode")
    ap.add_argument("--out",         type=Path, default=None,
                    help="output .jls for a single image, or a directory "
                         "(default: <image>.jls next to each image)")
    ap.add_argument("--no-program",  action="store_true",
                    help="Skip bitstream programming (assume FPGA already loaded)")
    ap.add_argument("--no-verify",   action="store_true",
                    help="Skip the CharLS verification")
    args = ap.parse_args()

    if not args.build and not args.image:
        ap.error("supply --build, --image, or both")

    if args.build:
        build()

    if not args.image:
        return
    for img in args.image:
        if not img.is_file():
            ap.error(f"image not found: {img}")
    out_dir = args.out if args.out and (len(args.image) > 1 or args.out.is_dir()) else None
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for k, img in enumerate(args.image):
        if out_dir:
            out = out_dir / img.with_suffix(".jls").name
        else:
            out = args.out if args.out else img.with_suffix(".jls")
        out.unlink(missing_ok=True)  # never verify a stale file
        try:
            # Program the bitstream once, before the first image.
            near = encode_image(img, out, open_bridge,
                                not args.no_program and k == 0)
            ok = True if args.no_verify else verify_charls(out, img, near)
        except Exception as e:  # noqa: BLE001 - report and continue with the next image
            print(f"[error] {img}: {e}")
            ok = False
        results.append((img, ok))

    print()
    for img, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {img}")
    failed = sum(1 for _, ok in results if not ok)
    print(f"  {len(results) - failed}/{len(results)} passed")
    sys.exit(1 if failed else 0)
