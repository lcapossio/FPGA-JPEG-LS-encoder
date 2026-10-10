<!-- SPDX-License-Identifier: GPL-3.0-only -->
<!-- Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com -->

# DE25-Nano example — hardware JPEG-LS encoder demo

End-to-end hardware demo of the JPEG-LS encoder on a [Terasic DE25-Nano](https://www.terasic.com.tw/) (Agilex 5, `A5EB013BB23BE4SR1`). Pixels and compressed bytes move over the onboard USB-Blaster III using the [fpgacapZero](https://github.com/lcapossio/fpgacapZero) EJTAG-AXI bridge (Intel `sld_virtual_jtag` wrapper).

It is a port of the [Arty A7 demo](../arty_a7/README.md): the encoder, the AXI wrapper (`../arty_a7/rtl/axi_jls_ctrl.v`, `../arty_a7/rtl/sync_fifo.v`) and the host flow (`../common/jls_demo.py`) are shared, so the register map, the CharLS verification and the command-line options are the same. Only the top level, the build and the JTAG transport differ.

## Architecture

```
┌─────────────────────┐  USB-Blaster III ┌───────────────────────────────┐
│ run_demo.py (host)  │ ←─────────────→ │ de25_jls_top                   │
│  - load .pgm        │  quartus_stp    │  ├─ CLOCK1_50 (no PLL)         │
│  - AXI writes pix   │                 │  ├─ fcapz_ejtagaxi_intel       │
│  - AXI reads .jls   │                 │  │    (vJTAG instance 4)       │
│  - CharLS verify    │                 │  └─ axi_jls_ctrl               │
└─────────────────────┘                 │       ├─ in_fifo (1024×8b)     │
                                        │       ├─ jls_encoder (NEAR=0)  │
                                        │       └─ out_fifo (256×17b)    │
                                        └───────────────────────────────┘
```

The whole fabric runs from the 50 MHz `CLOCK1_50`. Reset is a power-on counter plus `KEY[0]` (active low). `LEDR[3:0]` show the encoder status (same bits as the Arty LEDs); the LEDs are active low.

The bridge uses the same parameters as fpgacapZero's own DE25-Nano example (16-deep FIFOs, portable behavioral async FIFO, `CHAIN=4`).

## Prerequisites

- Quartus Prime Pro with Agilex 5 support (tested with 26.1). The scripts take the tools from `$QUARTUS_ROOTDIR/bin64`, or from `PATH`.
- DE25-Nano connected over USB (check with `jtagconfig`)
- `fcapz/` submodule initialized and its host package installed, plus CharLS for verification:
  `python -m pip install -e fcapz imagecodecs==2026.8.16 numpy`
- With more than one JTAG cable attached, set `JLS_JTAG_CABLE` to the cable name, e.g. `DE25-Nano [USB-1]`.

## Build

```bash
cd example/de25_nano
python run_demo.py --build
```

Runs `quartus_sh -t build.tcl` (project in `quartus_out/`, reports in `quartus_out/output_files/`) and copies the bitstream to `de25_jls_top.sof`.

Quartus Prime Pro 26.1 results: timing met at 50 MHz (setup slack +9.667 ns, hold +0.055 ns) and 5,459 of 46,800 ALMs (12 %). The large setup slack suggests the encoder would run at roughly 95 MHz on this part, compared with about 64 MHz on the Artix-7 -1. Quartus reports one critical warning recommending the Reset Release IP on Agilex 5; like fpgacapZero's DE25 example, this design uses a power-on counter instead, and the host soft-resets the encoder before every image.

## Run

```bash
python run_demo.py --image ../../SIM/images/test001.pgm
```

The script programs the FPGA with `quartus_pgm`, then encodes and verifies through `quartus_stp`. Verification and options are the same as for the [Arty demo](../arty_a7/README.md#run): the stream must equal CharLS's encoding byte-for-byte, and must decode within NEAR of the source. With several images the FPGA is programmed only once, and the script exits non-zero if any image fails.

Measured on the board (all byte-identical to CharLS):

| Image | Size | Stream | Time |
| --- | --- | --- | --- |
| `test001.pgm` | 5×1 | 34 B | 0.1 s |
| `test023.pgm` | 115×1 | 154 B | 0.5 s |
| `test008.pgm` | 205×191 | 42,266 B | 783 s |

The `quartus_stp` transport issues one Tcl virtual-JTAG call per scan, so it is about 4× slower than the Arty's hw_server path (206 s for `test008.pgm`); a 512×512 image would take about an hour.

## Files

| Path                     | Role                                                     |
| ------------------------ | -------------------------------------------------------- |
| `rtl/de25_jls_top.v`     | Top: reset, EJTAG-AXI (Intel) wrapper, `axi_jls_ctrl`    |
| `de25_jls.qsf`           | Device, top level and pin assignments                    |
| `de25_jls.sdc`           | Clock constraint                                         |
| `build.tcl`              | Quartus Prime Pro batch build                            |
| `run_demo.py`            | Host driver: Quartus build, programming, JTAG transport  |
| `../common/jls_demo.py`  | Board-independent host flow, shared with the Arty demo   |
