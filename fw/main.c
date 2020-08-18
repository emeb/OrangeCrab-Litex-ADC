/* This file is part of OrangeCrab-test
 *
 * Copyright 2020 Gregory Davill <greg.davill@gmail.com> 
 * Copyright 2020 Michael Welling <mwelling@ieee.org>
 */

#include <stdlib.h>
#include <stdio.h>

#include <uart.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/git.h>

#include "irq.h"
#include "sleep.h"
#include "flash-spi.h"
#include "command.h"
#include "readline.h"
#include "helpers.h"

void print_buffer(uint8_t* ptr, uint8_t len){
	for(int i = 0; i < len; i++){
		printf("%02x ", ptr[i]);
	}
}

void test_fail(const char* str){
	printf("%s\n", str);

	while(1);
}

int main(int i, char **c)
{
	char buffer[CMD_LINE_BUFFER_SIZE];
	struct command_struct *cmd;
	char *command;
	int nb_params;
	char *params[MAX_PARAM];
    
	/* Setup IRQ, needed for UART */
	irq_setmask(0);
	irq_setie(1);
	uart_init();

    /* banner */
	printf("\n");
	printf("Info:Hello from OrangeCrab! o/ \n");
	printf("Info:build_date "__DATE__" "__TIME__ "\n");
	printf("Info:test-repo "REPO_GIT_SHA1"\n");
	printf("Info:migen "MIGEN_GIT_SHA1"\n");
	printf("Info:litex "LITEX_GIT_SHA1"\n");

	/* Check for SPI FLASH */
	printf("Test:SPI-FLASH, Start\n");

	spiInit();
	unsigned char id[8] = {0};
	unsigned char uuid[8] = {0};
	spiId(id);

	printf("Info:SPI-FLASH-ID=");
	print_buffer(id, 5);
	printf("\n");
	if((id[0] != 0xef) |
	   (id[1] != 0x17) |
	   (id[2] != 0xef) |
	   (id[3] != 0x40) |
	   (id[4] != 0x18) ){
		   test_fail("Test:SPI-FLASH|Fail");
	   }

	// Check Flash UUID
	spi_read_uuid(uuid);
	printf("Info:SPI-FLASH-UUID=");
	print_buffer(uuid, 8);
	printf("\n");
	
	printf("Test:SPI-FLASH|Pass\n");

#if 0
       printf("Test:DDR3 Start\n");
	/* Init Memory */
	int sdr_ok = sdrinit();
	if(sdr_ok == 0){
		test_fail("Test:DDR3|Fail");
		//self_reset_out_write(0xAA550001);
	}
	printf("Test:DDR3|Pass\n");
#endif
       
	/* Test of LED GPIO */
	uint8_t led_gpio_patterns[] = {0x0, 0x1, 0x2, 0x4, 0x7};
	for(int i = 0; i < sizeof(led_gpio_patterns); i++){
		uint8_t out_pattern = led_gpio_patterns[i];
		gpio_led_out_write(0);
		gpio_led_oe_write(out_pattern);

		msleep(1);

		uint8_t read_pattern = gpio_led_in_read();

		/* Print values */
		printf("{%01X:%01X}\n",out_pattern, read_pattern);
	}
	gpio_led_out_write(~2);
    
	printf("Test:DONE, Finish\n");
#if 0
	msleep(50);
	while(1){
		msleep(10);
	}
#else
    printf("\n%s", PROMPT);
	while(1) {
		readline(buffer, CMD_LINE_BUFFER_SIZE);
		if (buffer[0] != 0) {
			printf("\n");
			nb_params = get_param(buffer, &command, params);
			cmd = command_dispatcher(command, nb_params, params);
			if (!cmd)
				printf("Command not found");
		}
		printf("\n%s", PROMPT);
	}
#endif
	return 0;
}
