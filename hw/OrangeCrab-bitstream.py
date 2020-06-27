#!/usr/bin/env python3

# This file is Copyright (c) Greg Davill <greg.davill@gmail.com>
# License: BSD


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


import inspect

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import orangecrab
from litex_boards.targets.orangecrab import _CRG

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *

from litedram.modules import MT41K64M16, MT41K128M16, MT41K256M16
from litedram.phy import ECP5DDRPHY

import valentyusb

from rtl.rgb import RGB
from litex.soc.cores import spi_flash

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    csr_map = {
        "ctrl":           0,  # provided by default (optional)
        "crg":            1,  # user
        "identifier_mem": 4,  # provided by default (optional)
        "timer0":         5,  # provided by default (optional)
        "usb":            9,
        "rgb":            13,
        "version":        14,
        "lxspi":          15,
        "button":         17,
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

        # Self Reset
        self.comb += platform.request("rst_n").eq(1)

        # USB hardware Abstract Control Model.
        kwargs['uart_name']="usb_acm"


        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, csr_data_width=32, **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq, with_usb_pll=True)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        #if 0:
        if not self.integrated_main_ram_size:
            available_sdram_modules = {
                'MT41K64M16': MT41K64M16,
                'MT41K128M16': MT41K128M16,
                'MT41K256M16': MT41K256M16,
#                'MT41K512M16': MT41K512M16 # Todo push definition for this part
            }
            sdram_module = available_sdram_modules.get(
                kwargs.get("sdram_device", "MT41K64M16"))

            self.submodules.ddrphy = ECP5DDRPHY(
                platform.request("ddram"),
                sys_clk_freq=sys_clk_freq)
            self.add_csr("ddrphy")
            self.add_constant("ECP5DDRPHY")
            self.comb += self.crg.stop.eq(self.ddrphy.init.stop)
            self.add_sdram("sdram",
                phy                     = self.ddrphy,
                module                  = sdram_module(sys_clk_freq, "1:2"),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )

        # RGB LED
        led = platform.request("rgb_led", 0)
        self.submodules.rgb = RGB(led)

        # The litex SPI module supports memory-mapped reads, as well as a bit-banged mode
        # for doing writes.
        spi_pads = platform.request("spiflash4x")
        self.submodules.lxspi = spi_flash.SpiFlashDualQuad(spi_pads, dummy=6, endianness="little")
        self.lxspi.add_clk_primitive(platform.device)
        self.register_mem("spiflash", self.mem_map["spiflash"], self.lxspi.bus, size=16*1024*1024)

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
        
    # Check if we have the correct files
    firmware_file = os.path.join(builder.output_dir, "software", "fw", "oc-fw.bin")
    firmware_data = get_mem_data(firmware_file, soc.cpu.endianness)
    firmware_init = os.path.join(builder.output_dir, "software", "fw", "oc-fw.init")
    CreateFirmwareInit(firmware_data, firmware_init)
    
    rand_rom = os.path.join(builder.output_dir, "gateware", "rand.data")

    input_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}.config")
    output_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}_patched.config")
    
    

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

    # Insert Firmware into Gateware
    os.system(f"ecpbram  --input {input_config} --output {output_config} --from {rand_rom} --to {firmware_init}")

    # create compressed config (ECP5 specific)
    output_bitstream = os.path.join(builder.gateware_dir, f"{soc.platform.name}.bit")
    os.system(f"ecppack --freq 38.8 --compress --input {output_config} --bit {output_bitstream}")

    dfu_file = os.path.join(builder.gateware_dir, f"{soc.platform.name}.dfu")
    shutil.copyfile(output_bitstream, dfu_file)
    os.system(f"dfu-suffix -v 1209 -p 5bf0 -a {dfu_file}")

    print(firmware_init)
    print(output_config)
    print(dfu_file)

def argdict(args):
    r = soc_sdram_argdict(args)
    for a in ["device", "revision", "sdram_device"]:
        arg = getattr(args, a, None)
        if arg is not None:
            r[a] = arg
    return r

if __name__ == "__main__":
    main()
