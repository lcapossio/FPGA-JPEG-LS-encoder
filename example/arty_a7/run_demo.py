#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
Arty A7-100T JPEG-LS encoder demo runner.

End-to-end flow:
  1. (optional) build the bitstream via Vivado
  2. program the FPGA over JTAG (Xilinx hw_server)
  3. load a .pgm image
  4. push pixels into the encoder over EJTAG-AXI (fpgacapZero)
  5. drain the encoded JPEG-LS byte stream over EJTAG-AXI
  6. verify the stream against CharLS (imagecodecs): byte-for-byte against
     CharLS's own encoding of the image at the bitstream's NEAR, plus a CharLS
     decode checked to be within NEAR of the source pixels

The single-binary bitstream has NEAR baked in via synthesis (read back from
the NEAR register). For this demo the compile-time value is 0 (lossless). See
arty_jls_top.v to change it.

Exits non-zero if any image fails to encode or verify.

Usage:
    python run_demo.py --build                  # synth+impl+bitstream
    python run_demo.py --image test001.pgm      # end-to-end run
    python run_demo.py --image a.pgm b.pgm      # several images, one programming
    python run_demo.py --image x.pgm --no-verify
    python run_demo.py --image x.pgm --build    # do everything
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE           = Path(__file__).parent.resolve()
JLS_ROOT       = HERE.parent.parent
BITFILE        = HERE / "arty_jls_top.bit"
BUILD_TCL      = HERE / "build.tcl"

sys.path.insert(0, str(JLS_ROOT / "example" / "common"))
import jls_demo  # noqa: E402


def sh(cmd, **kw):
    print(f"$ {' '.join(str(c) for c in cmd)}", flush=True)
    return subprocess.run(cmd, check=False, **kw)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_bitstream():
    # shutil.which also finds vivado.bat on Windows, which CreateProcess does not.
    vivado = os.environ.get("VIVADO") or shutil.which("vivado") or "vivado"
    cmd = [vivado, "-mode", "batch", "-source", str(BUILD_TCL)]
    r = sh(cmd, cwd=HERE)
    if r.returncode != 0:
        raise RuntimeError("Vivado build failed")
    if not BITFILE.exists():
        raise RuntimeError(f"Build completed but {BITFILE} not found")


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------
def open_bridge(program: bool):
    from fcapz.transport import XilinxHwServerTransport
    from fcapz.ejtagaxi import EjtagAxiController

    transport = XilinxHwServerTransport(
        fpga_name="xc7a100t",
        bitfile=str(BITFILE).replace("\\", "/") if program else None,
        # Skip ELA/USER1 probe — this design only uses USER4 (EJTAG-AXI).
        ready_probe_addr=None,
    )
    return EjtagAxiController(transport, chain=4)


if __name__ == "__main__":
    jls_demo.main(build_bitstream, open_bridge)
