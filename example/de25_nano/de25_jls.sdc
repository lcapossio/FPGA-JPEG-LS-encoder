# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

create_clock -name clk_50mhz -period 20.000 [get_ports {CLOCK1_50}]

# The sld_virtual_jtag instance provides its own TCK domain; the fcapz
# EJTAG-AXI bridge crosses it with explicit CDC (as in fpgacapZero's
# DE25-Nano example).
derive_clock_uncertainty
