# OrangeCrab Litex ADC
This repository contains the firmware and gateware for the OrangeCrab to
enable testing of the OrangeCrab ADC in a simple SDR application

Gateware is done in Litex and Verilog and the firmware is C.

---
## Hardware ##
* OrangeCrab: [OrangeCrab Repo](https://github.com/gregdavill/OrangeCrab)
* OrangeCrab ADC: [orangecrab_adc](https://github.com/emeb/orangecrab_adc)

## Usage ##

Current status: in developement - basic SDR operations working

To create the bitstream for the OrangeCrab run the following,
use `--update-firmware` flag when updating changes in firmware only.

    cd hw
    python3 OrangeCrab-bitstream.py [--update-firmware]

To load the bitstream

    dfu-util -D build/orangecrab/gateware/orangecrab.dfu

To operate, connect a terminal application to the USB CDC ACM device

    gtkterm -p /dev/ttyACM0

This will run a few simple tests and start the command software which
presents a "litex>" prompt. Type "help" to see what commands are available.

Currently the supported operations for the SDR core are:

* sdr_set_demod    - Set SDR Demod Type
* sdr_set_rate     - Set SDR Sample Rate
* sdr_set_cicgain  - Set SDR CIC Gain
* sdr_set_noiseshape - Set LO Noise shaping
* sdr_set_freq     - Set SDR LO Frequency
* sdr_status       - Get SDR settings & status
