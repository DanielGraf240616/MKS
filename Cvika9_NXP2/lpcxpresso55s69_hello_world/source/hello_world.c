/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_power.h"
#include <stdio.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t suma;
uint32_t DWT1, DWT2;
uint32_t x,y;
volatile uint32_t g_systickCounter;
char password_stored[20] = "12345678";
char input[20];
uint32_t random;
uint32_t max_number;
uint32_t min_number;
/*******************************************************************************
 * Code
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************/
/*!
 *
 *
 *
 * @brief Main function
 */



void SysTick_Handler(void)
{
	if(g_systickCounter != 0U)
	{
		g_systickCounter--;
	}

}

void SysTick_DelayTicks(uint32_t n)
{
	g_systickCounter = n;
	while(g_systickCounter != 0U)
	{

	}

}



int main(void)
{
	char ch;

	/* Init board hardware. */
	/* set BOD VBAT level to 1.65V */
	POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
	/* attach main clock divide to FLEXCOMM0 (debug console) */
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
#if !defined(DONT_ENABLE_FLASH_PREFETCH)
	/* enable flash prefetch for better performance */
	SYSCON->FMCCR |= SYSCON_FMCCR_PREFEN_MASK;
#endif

	//PRINTF("hello world.\r\n");

	while (1)
	{
		/* Delay 1000 ms */

		PRINTF("\r\nEnter password: ");
		SCANF("%s", input);

		while (getchar() != '\n');

		DWT1 = DWT->CYCCNT;
		uint32_t status;
		status = strcmp(input, password_stored);
		//SysTick_DelayTicks(random);
		DWT2 = DWT -> CYCCNT;
		suma = DWT2 - DWT1;
		PRINTF("\r\nPassword: %s", input);

		min_number = 31;
		max_number = 2532;

		random = rand() % (max_number + 1 - min_number) + min_number;

		if(status == 0)
		{

			PRINTF("\r\n Password correct");
		}
		else
		{

			PRINTF("\r\n Password incorrect");
		}

		printf("\r\nCycles for strcmp function: %d", suma);
	}
}
