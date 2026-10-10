#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

"""
DE25-Nano (Agilex 5) JPEG-LS encoder demo runner.

End-to-end flow:
  1. (optional) build the bitstream via Quartus Prime Pro
  2. program the FPGA over the onboard USB-Blaster III (quartus_pgm)
  3. encode .pgm images over the fcapz EJTAG-AXI bridge (quartus_stp)
  4. verify each stream against CharLS (see example/common/jls_demo.py)

The host flow is shared with the Arty A7 demo; only the build, programming
and JTAG transport differ. NEAR is baked in at synthesis (0 by default, see
rtl/de25_jls_top.v) and read back from the NEAR register.

Exits non-zero if any image fails to encode or verify.

Environment:
    QUARTUS_ROOTDIR  Quartus install (tools taken from its bin64), else PATH
    JLS_JTAG_CABLE   quartus_pgm / quartus_stp cable name, e.g.
                     "DE25-Nano [USB-1]" (default: the only cable present)

Usage:
    python run_demo.py --build                  # synthesis + fit + .sof
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
import time
from pathlib import Path

HERE           = Path(__file__).parent.resolve()
JLS_ROOT       = HERE.parent.parent
SOF_FILE       = HERE / "de25_jls_top.sof"
BUILD_TCL      = HERE / "build.tcl"
CABLE          = os.environ.get("JLS_JTAG_CABLE") or None

sys.path.insert(0, str(JLS_ROOT / "example" / "common"))
import jls_demo  # noqa: E402


def sh(cmd, **kw):
    print(f"$ {' '.join(str(c) for c in cmd)}", flush=True)
    return subprocess.run(cmd, check=False, **kw)


def quartus_tool(name: str) -> str:
    root = os.environ.get("QUARTUS_ROOTDIR")
    if root:
        for exe in (f"{name}.exe", name):
            cand = Path(root) / "bin64" / exe
            if cand.is_file():
                return str(cand)
    found = shutil.which(name)
    if not found:
        raise RuntimeError(f"{name} not found: set QUARTUS_ROOTDIR or add Quartus bin64 to PATH")
    return found


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_bitstream():
    r = sh([quartus_tool("quartus_sh"), "-t", str(BUILD_TCL)], cwd=HERE)
    if r.returncode != 0:
        raise RuntimeError("Quartus build failed")
    if not SOF_FILE.exists():
        raise RuntimeError(f"Build completed but {SOF_FILE} not found")


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------
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


if __name__ == "__main__":
    jls_demo.main(build_bitstream, open_bridge)
