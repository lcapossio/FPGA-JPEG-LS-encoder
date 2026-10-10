<!-- SPDX-License-Identifier: GPL-3.0-only -->
<!-- Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com -->

# DE25-Nano example — four encoders, 4K-class throughput

Four `jls_encoder` instances on a [Terasic DE25-Nano](https://www.terasic.com.tw/) (Agilex 5, `A5EB013BB23BE4SR1`) encode the four vertical tiles of each image in parallel. One encoder takes one pixel per clock, so at 150 MHz the four together take 600 Mpixel/s: enough for 3840×2160 at 60 fps, which needs 498 Mpixel/s.

Each tile is encoded as an independent, standard JPEG-LS image; a frame is four streams, and the receiver puts the decoded tiles side by side. A single standard stream would need JPEG-LS restart intervals instead, which means changes inside the encoder; this example keeps the verified encoder unchanged.

It builds on the [single-encoder DE25 demo](../de25_nano/README.md) (same board files, IOPLL, JTAG bridge and timing gate) and uses the same encoder RTL.

## Architecture

```
                    ┌──────────────────────── axi_jls4_ctrl ─────────────────────────┐
 PIX_IN (AXI) ──┐   │                    ┌─ tile 0 row buffer → reader → encoder 0 ──┼─ FIFO, bytes, CRC
 32-bit words   ├──→│ row writer ────────┼─ tile 1 ...                               │
 test pattern ──┘   │ 4 pixels/clock     ├─ tile 2 ...                               │
                    │                    └─ tile 3 ...           1 pixel/clock each  │
                    └────────────────────────────────────────────────────────────────┘
```

- **Tiling.** With `Wt = floor(W/4)`, tile *t* starts at column `t·Wt`; tiles 0–2 are `Wt` wide and tile 3 takes the remainder. Every tile must be 5 to 1024 columns wide, so images can be 20 to about 4096 pixels wide.
- **Input.** Pixels arrive in raster order, four per 32-bit word (pixel *x* in byte lane *x* mod 4), each row padded to whole words. The source is either the AXI `PIX_IN` window or an on-chip test pattern.
- **Row buffers.** The writer takes one word per clock and stores it into the row buffer of every tile its pixels belong to. Each tile's buffer holds two rows in four byte-lane RAMs, so a word can straddle a tile boundary. A tile's reader streams a row into its encoder, one pixel per clock, once the whole row is written; the writer reuses a row half only after every reader has finished with it. Latency from a row's last pixel to its encoding is therefore one row.
- **Output.** Every tile has a byte count and a CRC-32 of its stream. In image mode its words also go to a 256-word FIFO that the host drains over JTAG, and the tile's reader stalls while that FIFO is nearly full. In pattern mode the FIFOs are bypassed, so the frame runs at full speed and a cycle counter measures it.

The tiling, the word packing, the test pattern and the register map are defined once in [`../common/jls4_demo.py`](../common/jls4_demo.py), which the host script and the simulation share.

## Simulation

```bash
cd SIM
python run_sim.py --ctrl4 --near 0 1 2 3 4 5 6 7
```

`tb_axi_jls4_ctrl.v` runs two images through `PIX_IN` (205 and 256 wide, so one with tiles off the 4-pixel word grid) under input and output back-pressure, then two frames of the test pattern. Every tile must equal CharLS's encoding of that tile, and the `BYTES` and `CRC` registers must match the stream. The testbench also checks that no word is dropped, that a reader never reads an unwritten row, that the writer never overwrites a row still in use, and that each encoder receives exactly its tile's pixels. CI runs it for NEAR 0–7.

## Build

```bash
cd example/de25_nano_x4
python run_demo.py --build
```

As in the single-encoder demo, `build.tcl` generates the IOPLL for `JLS_CLK_MHZ` (default 150) and refuses to copy the `.sof` if any timing slack is negative. The clock is also passed to the controller, which reports it in a register so the host can convert cycles to time.

Quartus Prime Pro 26.1 results at 150 MHz:

| | ALMs | Registers | M20K |
| --- | --- | --- | --- |
| Each encoder | ~1,290 | 1,660–2,370 | 4 |
| Controller (writer, readers, row buffers, FIFOs) | ~1,095 | | 21 |
| JTAG bridge | ~1,490 | 2,645 | 0 |
| **Whole design** | **7,804 of 46,800 (17 %)** | | **37 of 358 (10 %)** |

Timing closes with setup slack +0.035 ns (Fmax 150.8 MHz), hold +0.038 ns. The limiting path is the same as in the single-encoder build, the encoders' context update loop, not the new controller. With four encoders the margin is thinner than the single encoder's +0.336 ns, so 150 MHz is about the limit for this part, and a rebuild can land on a placement that misses it. In that case build with `JLS_CLK_MHZ=140` or `125`: 3840×2160 at 60 fps needs 124.5 MHz, so both still cover 4K60.

## Run

```bash
python run_demo.py --pattern                                    # 3840x2160 at full speed
python run_demo.py --pattern 1920x1080 --image ../../SIM/images/test008.pgm
```

**`--pattern [WxH]`** (default 3840×2160) encodes the on-chip test pattern at full speed, then compares each tile's byte count and CRC-32 with CharLS's encoding of the same pattern, computed by the host. It reports the measured cycles as frame time, throughput and frame rate. This is the throughput measurement.

**`--image`** sends `.pgm` images over JTAG, reads the four tile streams back, and checks each one byte-for-byte against CharLS's encoding of that tile; the four decoded tiles must also reassemble the image. JTAG moves about 50–60 bytes per second, so this mode checks correctness, not speed. The tile streams are written next to each image (or to `--out`) as `<image>_tile<t>.jls`.

Measured on the board at 150 MHz:

| Run | Size | Output | Time |
| --- | --- | --- | --- |
| `--pattern` | 3840×2160 | 1,723,456 bytes in four tiles (4.81:1 on this synthetic pattern; camera images compress less); every tile's byte count and CRC match CharLS | 2,074,577 cycles = 13.83 ms: **72.3 fps, 600 Mpixel/s** |
| `test007.pgm` | 999×1 | 742 bytes, all four tiles byte-identical to CharLS | 1.9 s over JTAG |
| `test023.pgm` | 115×1 | 248 bytes, all four tiles byte-identical to CharLS | 0.7 s over JTAG |
| `test008.pgm` | 205×191 | 43,496 bytes, all four tiles byte-identical to CharLS | 623 s over JTAG |

The 4K frame takes the ideal 960 × 2160 cycles plus 977: the readers start once the first row is written, 960 cycles after the start (this overlaps the encoders' 513-cycle start-of-frame hold), and the encoder pipeline adds about 17. So 3840×2160 at 60 fps needs 124.5 MHz, and 150 MHz leaves 20 % headroom.

Tiling costs some compression, because each tile has its own header, its own context statistics, and a first column with no left neighbour. The four tiles of the 4K test pattern take 1,723,456 bytes against 1,605,234 for the whole image encoded at once (+7.4 %); for `test008.pgm` it is 43,496 against 42,266 bytes (+2.9 %).

## Files

| Path                        | Role                                                         |
| --------------------------- | ------------------------------------------------------------ |
| `rtl/axi_jls4_ctrl.v`       | AXI slave, row writer, tile row buffers, four encoders       |
| `rtl/de25_jls4_top.v`       | Top: IOPLL, reset, EJTAG-AXI (Intel) wrapper, controller      |
| `de25_jls4.qsf`             | Device, top level and pin assignments                        |
| `de25_jls4.sdc`             | Board clock and reset-synchroniser false paths               |
| `build.tcl`                 | Quartus Prime Pro batch build: IOPLL generation, timing check |
| `run_demo.py`               | Host driver: build, programming, image and pattern modes     |
| `../common/jls4_demo.py`    | Tiling, word packing, test pattern, register map             |
| `../../SIM/tb_axi_jls4_ctrl.v` | Testbench (`run_sim.py --ctrl4`)                          |
