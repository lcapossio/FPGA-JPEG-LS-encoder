#!/usr/bin/env python3
"""
Arty A7-100T JPEG-LS encoder demo runner.

End-to-end flow:
  1. (optional) build the bitstream via Vivado
  2. program the FPGA over JTAG (Xilinx hw_server)
  3. load a .pgm image
  4. push pixels into the encoder over EJTAG-AXI (fpgacapZero)
  5. drain the encoded JPEG-LS byte stream over EJTAG-AXI
  6. (optional) decode the .jls with ../SIM/JPEGLSdec.exe and byte-compare
     against the source .pgm (lossless roundtrip check for NEAR=0)

The single-binary bitstream has NEAR baked in via synthesis. For this demo
the compile-time value is 0 (lossless). See arty_jls_top.v to change it.

Usage:
    python run_demo.py --build                  # synth+impl+bitstream
    python run_demo.py --image test001.pgm      # end-to-end run
    python run_demo.py --image x.pgm --no-verify
    python run_demo.py --image x.pgm --build    # do everything
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

HERE           = Path(__file__).parent.resolve()
JLS_ROOT       = HERE.parent.parent
BITFILE        = HERE / "arty_jls_top.bit"
BUILD_TCL      = HERE / "build.tcl"
REF_DECODER    = JLS_ROOT / "SIM" / "JPEGLSdec.exe"
FCAPZ_ROOT     = Path(os.environ.get("FCAPZ_ROOT", "C:/Projects/fpgacapZero"))

# -- Register map (must match axi_jls_ctrl.v) --
REG_CTRL    = 0x0000
REG_WIDTH   = 0x0004
REG_HEIGHT  = 0x0008
REG_STATUS  = 0x000C
REG_NEAR    = 0x0010
PIX_IN_BASE = 0x1000
OUT_BASE    = 0x2000

CTRL_SOFT_RESET = 1 << 0
CTRL_SOF        = 1 << 1
CTRL_DONE_CLEAR = 1 << 2

BURST_MAX = 256  # AXI4 max (and FIFO_DEPTH in bridge = 256)


def sh(cmd, **kw):
    print(f"$ {' '.join(str(c) for c in cmd)}", flush=True)
    return subprocess.run(cmd, check=False, **kw)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_bitstream():
    vivado = os.environ.get("VIVADO", "vivado")
    cmd = [vivado, "-mode", "batch", "-source", str(BUILD_TCL)]
    r = sh(cmd, cwd=HERE)
    if r.returncode != 0:
        raise RuntimeError("Vivado build failed")
    if not BITFILE.exists():
        raise RuntimeError(f"Build completed but {BITFILE} not found")


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
        busy      = bool(s & 0x1),
        done      = bool(s & 0x2),
        in_full   = bool(s & 0x4),
        out_empty = bool(s & 0x8),
        in_count  = (s >> 8)  & 0x7FF,   # 11 bits, 0..1024
        out_count = (s >> 16) & 0x1FF,   # 9 bits, 0..256
    )


# ---------------------------------------------------------------------------
# Encode flow
# ---------------------------------------------------------------------------
def encode_image(pgm_path: Path, out_jls: Path, no_program: bool):
    try:
        from fcapz.transport import XilinxHwServerTransport
        from fcapz.ejtagaxi import EjtagAxiController
    except ImportError:
        print("ERROR: fpgacapZero host stack not importable.", file=sys.stderr)
        print(f"  Install with: pip install -e {FCAPZ_ROOT}", file=sys.stderr)
        raise

    w, h, pix = load_pgm(pgm_path)
    print(f"[pgm] {pgm_path}: {w}x{h} ({len(pix)} bytes)")

    if w < 5 or w > 16384 or h < 1 or h > 16384:
        raise ValueError(f"Image size {w}x{h} outside encoder range")

    transport = XilinxHwServerTransport(
        fpga_name="xc7a100t",
        bitfile=str(BITFILE).replace("\\", "/") if not no_program else None,
    )
    bridge = EjtagAxiController(transport, chain=4)

    info = bridge.connect()
    print(f"[axi] connected: {info}")
    t0 = time.time()

    try:
        # -- Soft reset -----------------------------------------------------
        bridge.axi_write(REG_CTRL, CTRL_SOFT_RESET)
        time.sleep(0.01)
        bridge.axi_write(REG_CTRL, 0)

        # -- Width / height --------------------------------------------------
        bridge.axi_write(REG_WIDTH,  w)
        bridge.axi_write(REG_HEIGHT, h)
        rw = bridge.axi_read(REG_WIDTH)  & 0x3FFF
        rh = bridge.axi_read(REG_HEIGHT) & 0x3FFF
        if rw != w or rh != h:
            raise RuntimeError(f"W/H readback mismatch: {rw}x{rh} != {w}x{h}")

        # -- SOF strobe ------------------------------------------------------
        bridge.axi_write(REG_CTRL, CTRL_SOF)

        # -- Push pixels while draining output in-between -------------------
        out_bytes = bytearray()
        got_last  = False
        pix_idx   = 0
        N         = len(pix)

        def drain_some(target_count=None):
            """Drain up to target_count words (None = drain until empty)."""
            nonlocal got_last
            while not got_last:
                st = decode_status(bridge.axi_read(REG_STATUS))
                if st["out_count"] == 0:
                    return
                batch = min(st["out_count"], BURST_MAX)
                words = bridge.burst_read(OUT_BASE, batch)
                for w32 in words:
                    data  = w32 & 0xFFFF
                    last  = bool((w32 >> 16) & 1)
                    out_bytes.append(data & 0xFF)
                    out_bytes.append((data >> 8) & 0xFF)
                    if last:
                        got_last = True
                if target_count is not None and len(out_bytes) >= target_count:
                    return

        # Push pixels: send one pixel per AXI beat (wstrb=0x1), in bursts.
        while pix_idx < N:
            st = decode_status(bridge.axi_read(REG_STATUS))
            free = 1024 - st["in_count"]
            if free <= 0:
                drain_some()
                continue
            batch = min(N - pix_idx, free, BURST_MAX)
            chunk = [pix[pix_idx + i] for i in range(batch)]
            bridge.burst_write(PIX_IN_BASE, chunk, wstrb=0x1)
            pix_idx += batch
            # Interleave a drain so out FIFO doesn't stall encoder
            drain_some()

        # All pixels queued — wait for completion
        deadline = time.time() + 60.0  # generous; 256x256 over JTAG ~ minutes
        while not got_last and time.time() < deadline:
            drain_some()
            if got_last:
                break
            st = decode_status(bridge.axi_read(REG_STATUS))
            if st["done"] and st["out_count"] == 0:
                got_last = True
                break
            time.sleep(0.005)

        if not got_last:
            raise RuntimeError("Timeout waiting for o_last")

        dt = time.time() - t0
        print(f"[axi] encoded {N}B -> {len(out_bytes)}B in {dt:.2f}s "
              f"({N/dt/1024:.1f} kB/s in)")
        out_jls.write_bytes(out_bytes)
        print(f"[jls] wrote {out_jls} ({len(out_bytes)} bytes)")
    finally:
        try:
            bridge.close()
        except Exception as e:
            print(f"[warn] bridge.close failed: {e}")


# ---------------------------------------------------------------------------
# Roundtrip verify
# ---------------------------------------------------------------------------
def verify_roundtrip(jls_path: Path, src_pgm: Path):
    if not REF_DECODER.exists():
        print(f"[verify] skipping, decoder not at {REF_DECODER}")
        return None

    decoded = jls_path.with_suffix(".decoded.pgm")
    cmd = [str(REF_DECODER), f"-i{jls_path}", f"-o{decoded}"]
    r = sh(cmd)
    if r.returncode != 0 or not decoded.exists():
        raise RuntimeError(f"Decoder failed: {r.returncode}")

    # Compare pixels only (skip PGM header)
    _, _, ref_pix = load_pgm(src_pgm)
    _, _, got_pix = load_pgm(decoded)

    if ref_pix == got_pix:
        print(f"[verify] PASS: {len(ref_pix)} bytes match")
        return True
    diffs = sum(1 for a, b in zip(ref_pix, got_pix) if a != b)
    print(f"[verify] FAIL: {diffs}/{len(ref_pix)} bytes differ")
    return False


# ---------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build",       action="store_true",
                    help="Synthesize and write bitstream before running")
    ap.add_argument("--image",       type=Path,
                    help=".pgm image to encode")
    ap.add_argument("--out",         type=Path, default=None,
                    help="output .jls (default: <image>.jls)")
    ap.add_argument("--no-program",  action="store_true",
                    help="Skip bitstream programming (assume FPGA already loaded)")
    ap.add_argument("--no-verify",   action="store_true",
                    help="Skip decode+compare roundtrip")
    args = ap.parse_args()

    if not args.build and not args.image:
        ap.error("supply --build, --image, or both")

    if args.build:
        build_bitstream()

    if args.image:
        if not args.image.is_file():
            ap.error(f"image not found: {args.image}")
        out = args.out if args.out else args.image.with_suffix(".jls")
        encode_image(args.image, out, args.no_program)
        if not args.no_verify:
            verify_roundtrip(out, args.image)


if __name__ == "__main__":
    main()
