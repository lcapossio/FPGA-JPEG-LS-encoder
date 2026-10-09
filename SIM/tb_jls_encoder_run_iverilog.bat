rem Compress all images with default settings (no golden check).
rem For the self-checking regression use: python run_sim.py
del sim.out dump.vcd
iverilog  -g2001  -o sim.out  tb_jls_encoder.v  ../RTL/jls_encoder.v
vvp -n sim.out
del sim.out
pause
