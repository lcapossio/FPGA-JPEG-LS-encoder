# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Leonardo Capossio - bard0 design - hello@bard0.com

# Quartus Prime Pro batch build for the DE25-Nano JPEG-LS encoder demo.
# Usage: quartus_sh -t build.tcl
#
# Environment:
#   FCAPZ_ROOT - path to fpgacapZero repo (defaults to <repo>/fcapz)

set example_dir [file normalize [file dirname [info script]]]
set jls_root    [file normalize $example_dir/../..]

set default_fcapz_root [file normalize $jls_root/fcapz]
set fcapz_root [expr {[info exists ::env(FCAPZ_ROOT)] ? $::env(FCAPZ_ROOT) : $default_fcapz_root}]
if {![file isdirectory $fcapz_root]} {
    puts "ERROR: fpgacapZero not found at $fcapz_root"
    puts "Set FCAPZ_ROOT or initialize the fcapz submodule"
    exit 1
}

set project_name de25_jls_demo
set project_dir  $example_dir/quartus_out
set output_dir   $project_dir/output_files

if {[file isdirectory $project_dir]} {
    file delete -force $project_dir
}
file mkdir $project_dir

project_new -overwrite -revision $project_name $project_dir/$project_name
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY $output_dir
set_global_assignment -name SEARCH_PATH $fcapz_root/rtl
set_global_assignment -name SEARCH_PATH $fcapz_root/rtl/jtag_tap
source $example_dir/de25_jls.qsf

# -- Source RTL --------------------------------------------------
foreach src [list \
    $jls_root/RTL/jls_encoder.v \
    $jls_root/example/arty_a7/rtl/sync_fifo.v \
    $jls_root/example/arty_a7/rtl/axi_jls_ctrl.v \
    $example_dir/rtl/de25_jls_top.v \
    $fcapz_root/rtl/fcapz_async_fifo.v \
    $fcapz_root/rtl/fcapz_ejtagaxi.v \
    $fcapz_root/rtl/fcapz_ejtagaxi_intel.v \
    $fcapz_root/rtl/jtag_tap/jtag_tap_intel.v \
] {
    set_global_assignment -name VERILOG_FILE $src
}
set_global_assignment -name SDC_FILE $example_dir/de25_jls.sdc

# -- Compile (synthesis, fit, timing, assembler) ------------------
if {[catch {execute_flow -compile} err]} {
    project_close
    puts "ERROR: Quartus compile failed: $err"
    exit 1
}
project_close

file copy -force $output_dir/${project_name}.sof $example_dir/de25_jls_top.sof
puts "\n=== Build complete: example/de25_nano/de25_jls_top.sof ==="
