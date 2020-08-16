#!/bin/sh
# JTAG load
openocd -f ~/Desktop/FPGA_Stuff/orange_crab/orangecrab_simple_6502/trellis/openocd_jlink.ocd -c "svf -tap lfe5u25.tap -quiet $1 ; shutdown"

