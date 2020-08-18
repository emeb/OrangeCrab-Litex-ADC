#!/usr/bin/env python3

# This file is Copyright (c) Greg Davill <greg.davill@gmail.com>
# License: BSD

# Modified for OrangeCrab ADC board Eric Brombaugh <ebrombaugh1@cox.net>

# Power Dissipation suggestions from Greg on OrangeCrab Discord:
# You can save a reasonable amount of power by not using the "virtual"
# power pins. The next step is disabling ODT (Which requires altering
# a LiteX source file.) You can also disable termination on the DQ group.
# (In the OrangeCrab platform file.)
#
# Measurements:
# Original current : 0.29A
# Disable 75ohm term : 0.20A
# Disable Virtual VCCIO : 0.17A
# Disable ODT : 0.11A

# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

import sys
import os
import shutil
import argparse
import subprocess

import inspect

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import orangecrab
#from litex_boards.targets.orangecrab import _CRG

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.build.generic_platform import IOStandard, Subsignal, Pins, Misc

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *

from litedram.modules import MT41K64M16, MT41K128M16, MT41K256M16
from litedram.phy import ECP5DDRPHY

from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.bitbang import I2CMaster

from litex.soc.doc import generate_docs


from migen.genlib.cdc import MultiReg

import valentyusb

from litex.soc.cores import spi_flash
from litex.soc.cores.gpio import GPIOTristate, GPIOOut, GPIOIn

from rtl.sdr import sdr

#  fn       RX  TX  SDA SCL                           MISO SCK MOSI  a0 a1 a2 a3 a4 a5
#  GPIO#     0   1   2   3 4  5   6 7 8  9 10 11 12 13 14  15  16  17 18 19 20 21 22 23
# ("GPIO", "N17 M18 C10 C9 - B10 B9 - - C8 B8 A8 H2 J2 N15 R17 N16 - L4 N3 N4 H4 G4 T17"),
# adc data - M18 N17 N15 B10 B9 C8 B8 A8 H2 J2

# connect all remaining GPIO pins out
extras = [
    ("ad9203", 0,
        Subsignal("data", Pins("GPIO:1 GPIO:0 GPIO:14 GPIO:5 GPIO:6 GPIO:9 GPIO:10 GPIO:11 GPIO:12 GPIO:13")),
        Subsignal("clk", Pins("GPIO:23")),
        IOStandard("LVCMOS33")
    ),
    ("pdm_out", 0,
        Subsignal("l", Pins("GPIO:15")),
        Subsignal("r", Pins("GPIO:16")),
        IOStandard("LVCMOS33")
    ),
    ("i2c", 0,
        Subsignal("sda", Pins("GPIO:2")),
        Subsignal("scl", Pins("GPIO:3")),
        IOStandard("LVCMOS33")
    ),
    ("analog", 0,
        Subsignal("mux", Pins("F4 F3 F2 H1")),
        Subsignal("enable", Pins("F1")),
        Subsignal("ctrl", Pins("G1")),
        Subsignal("sense_p", Pins("H3"), IOStandard("LVCMOS33D")),
        Subsignal("sense_n", Pins("G3")),
        IOStandard("LVCMOS33")
    )
]

# CRG ---------------------------------------------------------------------------------------------

class CRG(Module):
    def __init__(self, platform, sys_clk_freq, with_usb_pll=False):
        self.clock_domains.cd_init     = ClockDomain()
        self.clock_domains.cd_por      = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys      = ClockDomain()
        #self.clock_domains.cd_sys2x    = ClockDomain()
        #self.clock_domains.cd_sys2x_i  = ClockDomain()


        # # #

        self.stop = Signal()
        self.reset = Signal()

        
        # Use OSCG for generating por clocks.
        osc_g = Signal()
        self.specials += Instance("OSCG",
            p_DIV=6, # 38MHz
            o_OSC=osc_g
        )

        # Clk
        clk48 = platform.request("clk48")
        por_done  = Signal()

        # Power on reset 10ms.
        por_count = Signal(24, reset=int(48e6 * 50e-3))
        self.comb += self.cd_por.clk.eq(clk48)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        #sys2x_clk_ecsout = Signal()
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk48, 48e6)
        #pll.create_clkout(self.cd_sys2x_i, 2*sys_clk_freq)
        #pll.create_clkout(self.cd_init, 24e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.specials += [
            #Instance("ECLKBRIDGECS",
            #    i_CLK0   = self.cd_sys2x_i.clk,
            #    i_SEL    = 0,
            #    o_ECSOUT = sys2x_clk_ecsout),
            #Instance("ECLKSYNCB",
            #    i_ECLKI = sys2x_clk_ecsout,
            #    i_STOP  = self.stop,
            #    o_ECLKO = self.cd_sys2x.clk),
            #Instance("CLKDIVF",
            #    p_DIV     = "2.0",
            #    i_ALIGNWD = 0,
            #    i_CLKI    = self.cd_sys2x.clk,
            #    i_RST     = self.reset,
            #    o_CDIVX   = self.cd_sys.clk),
            #AsyncResetSynchronizer(self.cd_init,  ~por_done | ~pll.locked),
            AsyncResetSynchronizer(self.cd_sys,   ~por_done | ~pll.locked | self.reset),
            #AsyncResetSynchronizer(self.cd_sys2x, ~por_done | ~pll.locked | self.reset),
            #AsyncResetSynchronizer(self.cd_sys2x_i, ~por_done | ~pll.locked | self.reset),
        ]

        # USB PLL
        if with_usb_pll:
            self.clock_domains.cd_usb_12 = ClockDomain()
            self.clock_domains.cd_usb_48 = ClockDomain()
            usb_pll = ECP5PLL()
            self.submodules += usb_pll
            usb_pll.register_clkin(clk48, 48e6)
            usb_pll.create_clkout(self.cd_usb_48, 48e6)
            usb_pll.create_clkout(self.cd_usb_12, 12e6)
            self.specials += [
                AsyncResetSynchronizer(self.cd_usb_48,  ~por_done | ~usb_pll.locked),
                AsyncResetSynchronizer(self.cd_usb_12,  ~por_done | ~usb_pll.locked)
            ]



# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    csr_map = {
        "ctrl":           0,  # provided by default (optional)
        "crg":            1,  # user
        "identifier_mem": 4,  # provided by default (optional)
        "timer0":         5,  # provided by default (optional)
       
        "gpio_led":       10,
        "self_reset":     12,
        "version":        14,
        "lxspi":          15,
        "button":         17,
        "i2c":            19,
        "sdr":            20
    }
    csr_map.update(SoCCore.csr_map)

    mem_map = {
        "rom":      0x00000000,  # (default shadow @0x80000000)
        "sram":     0x10000000,  # (default shadow @0xa0000000)
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
        "main_ram": 0x40000000,  # (default shadow @0xc0000000)
        "csr":      0xe0000000,  # (default shadow @0xe0000000)
    }
    mem_map.update(SoCCore.mem_map)

    interrupt_map = {
        "timer0": 2,
        "usb": 3,
    }
    interrupt_map.update(SoCCore.interrupt_map)

    def __init__(self, sys_clk_freq=int(48e6), toolchain="trellis", **kwargs):
        # Board Revision ---------------------------------------------------------------------------
        revision = kwargs.get("revision", "0.2")
        device = kwargs.get("device", "25F")

        platform = orangecrab.Platform(revision=revision, device=device ,toolchain=toolchain)

        # Serial -----------------------------------------------------------------------------------
        #platform.add_extension(orangecrab.feather_serial)


        # USB hardware Abstract Control Model.
        kwargs['uart_name']="usb_acm"

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, csr_data_width=32, **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = crg = CRG(platform, sys_clk_freq, with_usb_pll=True)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if 0:
            if not self.integrated_main_ram_size:
                available_sdram_modules = {
                    'MT41K64M16': MT41K64M16,
                    'MT41K128M16': MT41K128M16,
                    'MT41K256M16': MT41K256M16
                }
                sdram_module = available_sdram_modules.get(
                    kwargs.get("sdram_device", "MT41K64M16"))

                ddr_pads = platform.request("ddram")
                self.submodules.ddrphy = ECP5DDRPHY(
                    ddr_pads,
                    sys_clk_freq=sys_clk_freq)
                self.add_csr("ddrphy")
                self.add_constant("ECP5DDRPHY")
                self.comb += crg.stop.eq(self.ddrphy.init.stop)
                self.comb += crg.reset.eq(self.ddrphy.init.reset)
                self.add_sdram("sdram",
                    phy                     = self.ddrphy,
                    module                  = sdram_module(sys_clk_freq, "1:2"),
                    origin                  = self.mem_map["main_ram"],
                    size                    = kwargs.get("max_sdram_size", 0x40000000),
                    l2_cache_size           = kwargs.get("l2_size", 8192),
                    l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                    l2_cache_reverse        = True
                )

                # Virtual power pins - suggested to reduce SSO noise
                #self.comb += ddr_pads.vccio.eq(1)
                self.comb += ddr_pads.gnd.eq(0)

        # Add extra pin definitions
        platform.add_extension(extras)

        # RGB LED
        led = platform.request("rgb_led", 0)
        self.submodules.gpio_led = GPIOTristate(Cat(led.r,led.g,led.b))

        # i2c
        self.submodules.i2c = I2CMaster(platform.request("i2c"))

        # SDR processor
        self.submodules.sdr = sdr(platform.request("ad9203"), platform.request("pdm_out"))
        platform.add_source_dir('vsrc')
        
        # Controllable Self Reset
        reset_code = Signal(32, reset=0)
        self.submodules.self_reset = GPIOOut(reset_code)
        self.comb += platform.request("rst_n").eq(reset_code != 0xAA550001)

        self.submodules.button = GPIOIn(platform.request("usr_btn"))
        
        # The litex SPI module supports memory-mapped reads, as well as a bit-banged mode
        # for doing writes.
        spi_pads = platform.request("spiflash4x")
        self.submodules.lxspi = spi_flash.SpiFlashDualQuad(spi_pads, dummy=6, endianness="little")
        self.lxspi.add_clk_primitive(platform.device)
        self.register_mem("spiflash", self.mem_map["spiflash"], self.lxspi.bus, size=16*1024*1024)

        # Add GIT repo to the firmware
        git_rev_cmd = subprocess.Popen(["git", "rev-parse", "--short", "HEAD"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
        (git_stdout, _) = git_rev_cmd.communicate()
        self.add_constant('REPO_GIT_SHA1',git_stdout.decode('ascii').strip('\n'))



    # This function will build our software and create a oc-fw.init file that can be patched directly into blockram in the FPGA
    def PackageFirmware(self, builder):  
        self.finalize()

        os.makedirs(builder.output_dir, exist_ok=True)

        src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "fw"))
        builder.add_software_package("fw", src_dir)

        builder._prepare_rom_software()
        builder._generate_includes()
        builder._generate_rom_software(compile_bios=False)

        firmware_file = os.path.join(builder.output_dir, "software", "fw","oc-fw.bin")
        firmware_data = get_mem_data(firmware_file, self.cpu.endianness)
        self.initialize_rom(firmware_data)

        # lock out compiling firmware during build steps
        builder.compile_software = False


def CreateFirmwareInit(init, output_file):
    content = ""
    for d in init:
        content += "{:08x}\n".format(d)
    with open(output_file, "w") as o:
        o.write(content)    


# Build --------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on OrangeCrab")
    parser.add_argument("--gateware-toolchain", dest="toolchain", default="trellis",
        help="gateware toolchain to use, trellis (default) or diamond")
    builder_args(parser)
    soc_sdram_args(parser)
    trellis_args(parser)
    parser.add_argument("--sys-clk-freq", default=48e6,
                        help="system clock frequency (default=48MHz)")
    parser.add_argument("--revision", default="0.2",
                        help="Board Revision {0.1, 0.2} (default=0.2)")
    parser.add_argument("--device", default="25F",
                        help="ECP5 device (default=25F)")
    parser.add_argument("--sdram-device", default="MT41K64M16",
                        help="ECP5 device (default=MT41K64M16)")
    parser.add_argument(
        "--update-firmware", default=False, action='store_true',
        help="compile firmware and update existing gateware"
    )
    args = parser.parse_args()

    soc = BaseSoC(toolchain=args.toolchain, sys_clk_freq=int(float(args.sys_clk_freq)),**argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    


    # Build firmware
    soc.PackageFirmware(builder)
    #generate_docs(soc, "build/documentation/", project_name="OrangeCrab Test SoC", author="Greg Davill")
        
    # Check if we have the correct files
    firmware_file = os.path.join(builder.output_dir, "software", "fw", "oc-fw.bin")
    firmware_data = get_mem_data(firmware_file, soc.cpu.endianness)
    firmware_init = os.path.join(builder.output_dir, "software", "fw", "oc-fw.init")
    CreateFirmwareInit(firmware_data, firmware_init)
    
    rand_rom = os.path.join(builder.output_dir, "gateware", "rand.data")

    

    # If we don't have a random file, create one, and recompile gateware
    if (os.path.exists(rand_rom) == False) or (args.update_firmware == False):
        os.makedirs(os.path.join(builder.output_dir,'gateware'), exist_ok=True)
        os.makedirs(os.path.join(builder.output_dir,'software'), exist_ok=True)

        os.system(f"ecpbram  --generate {rand_rom} --seed {0} --width {32} --depth {soc.integrated_rom_size}")

        # patch random file into BRAM
        data = []
        with open(rand_rom, 'r') as inp:
            for d in inp.readlines():
                data += [int(d, 16)]
        soc.initialize_rom(data)

        # Build gateware
        builder_kargs = trellis_argdict(args) if args.toolchain == "trellis" else {}
        vns = builder.build(**builder_kargs)
        soc.do_exit(vns)   
    

    input_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}.config")
    output_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}_patched.config")

    # Insert Firmware into Gateware
    os.system(f"ecpbram  --input {input_config} --output {output_config} --from {rand_rom} --to {firmware_init}")


    # create compressed config (ECP5 specific)
    output_bitstream = os.path.join(builder.gateware_dir, f"{soc.platform.name}.bit")
    #os.system(f"ecppack --freq 38.8 --spimode qspi --compress --input {output_config} --bit {output_bitstream}")
    os.system(f"ecppack --freq 38.8 --compress --input {output_config} --bit {output_bitstream}")

    dfu_file = os.path.join(builder.gateware_dir, f"{soc.platform.name}.dfu")
    shutil.copyfile(output_bitstream, dfu_file)
    os.system(f"dfu-suffix -v 1209 -p 5bf0 -a {dfu_file}")


def argdict(args):
    r = soc_sdram_argdict(args)
    for a in ["device", "revision", "sdram_device"]:
        arg = getattr(args, a, None)
        if arg is not None:
            r[a] = arg
    return r

if __name__ == "__main__":
    main()
