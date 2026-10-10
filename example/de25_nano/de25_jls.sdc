# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

create_clock -name clk_50mhz -period 20.000 [get_ports {CLOCK1_50}]

# The fabric clock comes from the jls_iopll IOPLL; its generated clock is
# created by the IP's own SDC.

# Reset synchroniser in de25_jls_top: PLL lock loss clears hold_sync
# asynchronously and KEY[0] is an asynchronous button. Release is synchronised
# (hold_sync, then the reset counter), so neither input needs timing.
set_false_path -from [get_keepers {*tennm_ph2_iopll~pll_ctrl_reg}] -to [get_keepers {hold_sync[*]}]
set_false_path -from [get_ports {KEY[0]}]

# The sld_virtual_jtag instance provides its own TCK domain; the fcapz
# EJTAG-AXI bridge crosses it with explicit CDC (as in fpgacapZero's
# DE25-Nano example).
derive_clock_uncertainty
