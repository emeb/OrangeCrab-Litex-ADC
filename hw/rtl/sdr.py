# sdr.py - top level of instantiated Verilog SDR function
# 2020-08-16 E. Brombaugh

from migen import *

from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRField, CSRStatus

class sdr(Module, AutoCSR):
    def __init__(self, adc_pins, pdm_pins):
        
        # DDC control registers
        self.ddc_freq = CSRStorage(26, reset=2415919)
        self.ddc_ns_ena = CSRStorage(1, reset=0)
        self.ddc_cic_shf = CSRStorage(4, reset=6)
        self.ddc_dr = CSRStorage(2, reset=0)
        self.demod_type = CSRStorage(3, reset=0)
        self.ddc_satcnt = CSRStatus(7, reset=0)
        
        # interfaces between instances
        self.reset = Signal()
        sathld = Signal(7)
        ddc_i = Signal(16)
        ddc_q = Signal(16)
        ddc_valid = Signal()
        diag = Signal()
        demod_valid = Signal()
        demod_l = Signal(16)
        demod_r = Signal(16)
        
        # instantiate Verilog modules
        self.specials += [
            # DDC instance
            Instance("ddc_14",
                p_isz       = 10,
                p_fsz       = 26,
                p_osz       = 16,
                i_clk       = adc_pins.clk,
                i_reset     = self.reset,
                i_in        = adc_pins.data,
                i_dr        = self.ddc_dr.storage,
                i_frq       = self.ddc_freq.storage,
                i_cic_shf   = self.ddc_cic_shf.storage,
                i_ns_ena    = self.ddc_ns_ena.storage,
                o_sathld    = self.ddc_satcnt.status,
                o_valid     = ddc_valid,
                o_i_out     = ddc_i,
                o_q_out     = ddc_q,
                o_diag      = diag
            ),
        
            # Demod converts I/Q to audio
            Instance("demods",       
                i_clk       = adc_pins.clk,
                i_reset     = self.reset,
                i_i         = ddc_i,
                i_q         = ddc_q,
                i_ena       = ddc_valid,
                i_type      = self.demod_type.storage,
                o_valid     = demod_valid,
                o_l_out     = demod_l,
                o_r_out     = demod_r
            ),

            # PDM Audio output
            Instance("pdm_dac",       
                i_clk       = adc_pins.clk,
                i_reset     = self.reset,
                i_load      = demod_valid,
                i_lin       = demod_l,
                i_rin       = demod_r,
                o_lpdm      = pdm_pins.l,
                o_rpdm      = pdm_pins.r
            )
        ]