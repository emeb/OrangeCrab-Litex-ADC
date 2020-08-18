// SPDX-License-Identifier: BSD-Source-Code
// SDR extensions to Litex BIOS
// 2020-08-18 E. Brombaugh

#include <stdio.h>
#include <stdlib.h>

#include <generated/csr.h>

#include "command.h"
#include "helpers.h"

#define FSAMP 40000000
const char * sr_names[] = 
{
    "19.53",
    "39.06",
    "78.13",
    "156.25",
};

const char * demod_names[] =
{
    "AM",
    "NBDM",
    "I/Q",
    "I/Q",
    "I/Q",
    "I/Q",
    "I/Q",
    "I/Q",
};

/**
 * Command "sdr_status"
 *
 * display SDR settings & status
 *
 */
static void sdr_status(int nb_params, char **params)
{
    uint32_t freq = sdr_ddc_freq_read();
    uint64_t temp = freq;
    freq = (temp*FSAMP)>>26;
    printf("LO Frequency: %d Hz\n", freq);
    printf("LO Noise Shaping: %s\n", sdr_ddc_ns_ena_read() ? "On" : "Off");
    printf("CIC Gain Shift: %d bits\n", sdr_ddc_cic_shf_read());
    printf("Sample Rate: %skSPS (%d)\n", sr_names[sdr_ddc_dr_read()&3], sdr_ddc_dr_read());
    printf("Demod: %s (%d)\n", demod_names[sdr_demod_type_read()&7], sdr_demod_type_read());
    printf("Saturation Rate: %d %%\n", (100*sdr_ddc_satcnt_read())>>7);
}

define_command(sdr_status, sdr_status, "Get SDR settings & status", SDR_CMDS);

/**
 * Command "sdr_set_freq"
 *
 * Set DDC LO frequency
 *
 */
static void sdr_set_freq(int nb_params, char **params)
{
	char *c;
	unsigned int freq;
    
	if (nb_params < 1) {
		printf("sdr_set_freq <frequency>");
		return;
	}
	freq = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("bad frequency");
		return;
	}
    
    uint64_t temp = ((uint64_t)freq)<<26;
    temp = temp / FSAMP;
    freq = temp;    

	sdr_ddc_freq_write(freq);
}

define_command(sdr_set_freq, sdr_set_freq, "Set SDR LO Frequency", SDR_CMDS);

/**
 * Command "sdr_set_noiseshape"
 *
 * Set LO Noise Shaping
 *
 */
static void sdr_set_noiseshape(int nb_params, char **params)
{
	char *c;
	unsigned int state;
    
	if (nb_params < 1) {
		printf("sdr_set_noiseshape <state (0 = off, 1 = on)>");
		return;
	}
	state = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("bad state");
		return;
	}
    
	sdr_ddc_ns_ena_write(state&1);
}

define_command(sdr_set_noiseshape, sdr_set_noiseshape, "Set LO Noise shaping", SDR_CMDS);

/**
 * Command "sdr_set_cicgain"
 *
 * Set CIC output shift
 *
 */
static void sdr_set_cicgain(int nb_params, char **params)
{
	char *c;
	unsigned int shift;
    
	if (nb_params < 1) {
		printf("sdr_set_cicgain <shift bits>");
		return;
	}
	shift = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("bad shift");
		return;
	}
    shift = shift > 7 ? 7 : shift;
    
	sdr_ddc_cic_shf_write(shift);
}

define_command(sdr_set_cicgain, sdr_set_cicgain, "Set SDR CIC Gain", SDR_CMDS);

/**
 * Command "sdr_set_rate"
 *
 * Set sample rate
 *
 */
static void sdr_set_rate(int nb_params, char **params)
{
	char *c;
	unsigned int rate;
    
	if (nb_params < 1) {
		printf("sdr_set_rate <index (0-3)>");
		return;
	}
	rate = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("bad index");
		return;
	}
    
	sdr_ddc_dr_write(rate&3);
}

define_command(sdr_set_rate, sdr_set_rate, "Set SDR Sample Rate", SDR_CMDS);

/**
 * Command "sdr_set_demod"
 *
 * Set demod type
 *
 */
static void sdr_set_demod(int nb_params, char **params)
{
	char *c;
	unsigned int demod;
    
	if (nb_params < 1) {
		printf("sdr_set_demod <type (0-7)>");
		return;
	}
	demod = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("bad demod");
		return;
	}
    
	sdr_demod_type_write(demod&7);
}

define_command(sdr_set_demod, sdr_set_demod, "Set SDR Demod Type", SDR_CMDS);

