
/*------------------------------------------------------------------------------
 * Copyright (c) 2018 Arm Limited (or its affiliates). All
 * rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   3.Neither the name of Arm nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *------------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------*/

#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS:Keil RTX5
#include "stm32f407xx.h"
void Led_swift(void);
void LED_Off(void);
void LED_Initialize(void);
void LED_On(void);
void button_init(void);
uint32_t button_status(void);
static osThreadId_t tid_thrLED;         // Thread id of thread: LED
static osThreadId_t tid_thrBUT;         // Thread id of thread: BUT

/*------------------------------------------------------------------------------
  thrLED: blink LED
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrLED (void *argument) {
  uint32_t active_flag = 1U;
	 uint32_t state;
	while(1)
	{	
		state = (button_status () & 1U); 
		if(state)
		{
			LED_Off();
			osDelay(500U);
		}
		else
			LED_On();
	}
  /*for (;;) {
    if (osThreadFlagsWait (1U, osFlagsWaitAny, 0U) == 1U) {
      active_flag ^=1U; 
    }

    if (active_flag == 1U){
      LED_On ();                                // Switch LED on
      osDelay (500U);                             // Delay 500 ms
      LED_Off ();                               // Switch off
      osDelay (500U);                            // Delay 500 ms
			
    }
    else {
      osDelay (500U);                             // Delay 500 ms
    }
  }*/
}

/*------------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
void app_main (void *argument)
{

  LED_Initialize ();                    // Initialize LEDs
	button_init();

  tid_thrLED = osThreadNew (thrLED, NULL, NULL);  // Create LED thread
  if (tid_thrLED == NULL) { /* add error handling */ }

  osThreadExit();
}

void LED_Initialize(void)
{	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	GPIOD->MODER |= GPIO_MODER_MODER12_0;
	GPIOD->MODER |= GPIO_MODER_MODER13_0;
	GPIOD->MODER |= GPIO_MODER_MODER14_0;
	GPIOD->MODER |= GPIO_MODER_MODER15_0;

}
void LED_On(void)
{
	GPIOD->BSRR |= 1<<12;
	GPIOD->BSRR |= 1<<13;
	GPIOD->BSRR |= 1<<14;
	GPIOD->BSRR |= 1<<15;
}
void LED_Off(void)
{
	GPIOD->BSRR |= 1<<28;
	GPIOD->BSRR |= 1<<29;
	GPIOD->BSRR |= 1<<30;
	GPIOD->BSRR |= 1<<31;
}
void Led_swift(void)
{
	GPIOD->BSRR |= 1<<31;
	GPIOD->BSRR |= 1<<12;
	osDelay (500U);
	GPIOD->BSRR |= 1<<28;
	GPIOD->BSRR |= 1<<13;
	osDelay (500U);
	GPIOD->BSRR |= 1<<29;
	GPIOD->BSRR |= 1<<14;
	osDelay (500U);
	GPIOD->BSRR |= 1<<30;
	GPIOD->BSRR |= 1<<15;
	osDelay (500U);
}
void button_init(void)
{
	RCC->AHB1ENR |= 0x01;
	GPIOA->MODER |= 0x00;
}
uint32_t button_status(void)
{
		return GPIOA->IDR;
}