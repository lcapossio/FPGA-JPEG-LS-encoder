<!-- SPDX-License-Identifier: GPL-3.0-only -->
<!-- Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com -->

# DE25-Nano example — hardware JPEG-LS encoder demo

End-to-end hardware demo of the JPEG-LS encoder on a [Terasic DE25-Nano](https://www.terasic.com.tw/) (Agilex 5, `A5EB013BB23BE4SR1`). Pixels and compressed bytes move over the onboard USB-Blaster III using the [fpgacapZero](https://github.com/lcapossio/fpgacapZero) EJTAG-AXI bridge (Intel `sld_virtual_jtag` wrapper).

It is a port of the [Arty A7 demo](../arty_a7/README.md): the encoder, the AXI wrapper (`../arty_a7/rtl/axi_jls_ctrl.v`, `../arty_a7/rtl/sync_fifo.v`) and the host flow (`../common/jls_demo.py`) are shared, so the register map, the CharLS verification and the command-line options are the same. Only the top level, the build and the JTAG transport differ.

## Architecture

```
┌─────────────────────┐  USB-Blaster III ┌───────────────────────────────┐
│ run_demo.py (host)  │ ←─────────────→ │ de25_jls_top                   │
│  - load .pgm        │  quartus_stp    │  ├─ CLOCK1_50 → PLL (150 MHz)  │
│  - AXI writes pix   │                 │  ├─ fcapz_ejtagaxi_intel       │
│  - AXI reads .jls   │                 │  │    (vJTAG instance 4)       │
│  - CharLS verify    │                 │  └─ axi_jls_ctrl               │
└─────────────────────┘                 │       ├─ in_fifo (1024×8b)     │
                                        │       ├─ jls_encoder (NEAR=0)  │
                                        │       └─ out_fifo (256×17b)    │
                                        └───────────────────────────────┘
```

The whole fabric runs from one IOPLL output, 150 MHz by default, generated from the 50 MHz `CLOCK1_50`. Reset is a counter restarted by power-on, PLL lock and `KEY[0]` (active low). `LEDR[3:0]` show the encoder status (same bits as the Arty LEDs); the LEDs are active low.

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

`build.tcl` first generates the IOPLL (`jls_iopll`) with `ip-deploy` for the fabric clock set by `JLS_CLK_MHZ` (default 150, in MHz). After the compile it reads the multicorner worst-case slacks from the timing report and fails, without copying the `.sof`, if setup, hold, recovery or removal is negative. To build for another clock:

```bash
JLS_CLK_MHZ=125 python run_demo.py --build
```

Quartus Prime Pro 26.1 results at the default 150 MHz: 5,755 of 46,800 ALMs (12 %) and worst-case slacks of setup +0.268 ns, hold +0.051 ns, recovery +4.125 ns and removal +0.300 ns (Fmax 156.3 MHz).

Frequency sweep (worst-case multicorner slack, one fitter run per point):

| `JLS_CLK_MHZ` | Setup slack | Hold slack | Fmax (slow corner) | Timing |
| --- | --- | --- | --- | --- |
| 50 (no PLL, first port) | +9.667 ns | +0.055 ns | 96.8 MHz | met |
| 100 | +1.283 ns | +0.069 ns | 114.7 MHz | met |
| 125 | +0.247 ns | +0.048 ns | 129.0 MHz | met, board-tested |
| 140 | +0.349 ns | +0.045 ns | 147.2 MHz | met |
| 145 | +0.291 ns | +0.053 ns | 151.4 MHz | met |
| **150** | **+0.268 ns** | **+0.051 ns** | **156.3 MHz** | **met, default, board-tested** |
| 160 | +0.003 ns | +0.048 ns | 160.1 MHz | met, no margin |
| 170 | −0.474 ns | +0.043 ns | 157.3 MHz | fails |
| 180 | −0.555 ns | +0.051 ns | 163.7 MHz | fails |

The ceiling is about 160 MHz, 2.5× the Artix-7 -1 (about 64 MHz). 160 MHz closes with no margin on this run, so the default stays at 150 MHz. The limiting path is the encoder's context update loop (the `B`/`C` bias update fed from the context RAM); the Hyper-Retimer reports it as "Insufficient Registers", because a loop cannot be retimed, only shortened.

At 150 MHz the encoder (one pixel per clock, plus about 400 cycles per frame) sustains 150 Mpixel/s, i.e. 1920×1080 at 72 fps or 1280×720 at 162 fps. Over JTAG the demo is far slower than this; see [Run](#run).

Two constraints in `de25_jls.sdc` go with the PLL. The reset synchroniser is cleared asynchronously by the PLL's `locked` output and samples `KEY[0]`, so both inputs are false paths. Without them, Quartus reports recovery violations from the IOPLL's control register above about 135 MHz.

Quartus reports one critical warning recommending the Reset Release IP on Agilex 5. Like fpgacapZero's DE25 example, this design uses a reset counter instead, and the host soft-resets the encoder before every image.

## Run

```bash
python run_demo.py --image ../../SIM/images/test001.pgm
```

The script programs the FPGA with `quartus_pgm`, then encodes and verifies through `quartus_stp`. Verification and options are the same as for the [Arty demo](../arty_a7/README.md#run): the stream must equal CharLS's encoding byte-for-byte, and must decode within NEAR of the source. With several images the FPGA is programmed only once, and the script exits non-zero if any image fails.

Measured on the board at 150 MHz (all byte-identical to CharLS):

| Image | Size | Stream | Time |
| --- | --- | --- | --- |
| `test001.pgm` | 5×1 | 34 B | 0.1 s |
| `test023.pgm` | 115×1 | 154 B | 0.4 s |
| `test008.pgm` | 205×191 | 42,266 B | 608 s |

These times are set by the JTAG link, not by the encoder, which needs 0.3 ms for `test008.pgm` at 150 MHz. The `quartus_stp` transport issues one Tcl virtual-JTAG call per scan, so it is about 3× slower than the Arty's hw_server path (206 s for `test008.pgm`); the 50 MHz build took 783 s for the same image. A 512×512 image takes a little over an hour.

## Files

| Path                     | Role                                                          |
| ------------------------ | ------------------------------------------------------------- |
| `rtl/de25_jls_top.v`     | Top: IOPLL, reset, EJTAG-AXI (Intel) wrapper, `axi_jls_ctrl`  |
| `de25_jls.qsf`           | Device, top level and pin assignments                         |
| `de25_jls.sdc`           | Board clock and reset-synchroniser false paths                |
| `build.tcl`              | Quartus Prime Pro batch build: IOPLL generation, timing check |
| `run_demo.py`            | Host driver: Quartus build, programming, JTAG transport       |
| `../common/jls_demo.py`  | Board-independent host flow, shared with the Arty demo        |
