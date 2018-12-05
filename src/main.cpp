/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
#define mainDELAY_LOOP_COUNT		( 0xfffff )
// Definitions visible only within this translation unit.
namespace
{
  // ----- Timing definitions -------------------------------------------------

  // Keep the LED on for 2/3 of a second.
  constexpr Timer::ticks_t BLINK_ON_TICKS = Timer::FREQUENCY_HZ * 3 / 4;
  constexpr Timer::ticks_t BLINK_OFF_TICKS = Timer::FREQUENCY_HZ
      - BLINK_ON_TICKS;
}

// ----- LED definitions ------------------------------------------------------

#if defined(STM32F401xE)

#warning "Assume a NUCLEO-F401RE board, PA5, active high."

// PA5
#define BLINK_PORT_NUMBER         (0)
#define BLINK_PIN_NUMBER          (5)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[1] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER, BLINK_ACTIVE_LOW },
  };

#elif defined(STM32F407xx)

//#warning "Assume a STM32F4-Discovery board, PD12-PD15, active high."

#define BLINK_PORT_NUMBER         (3)
#define BLINK_PIN_NUMBER_GREEN    (12)
#define BLINK_PIN_NUMBER_ORANGE   (13)
#define BLINK_PIN_NUMBER_RED      (14)
#define BLINK_PIN_NUMBER_BLUE     (15)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[4] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_GREEN, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_ORANGE, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_RED, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_BLUE, BLINK_ACTIVE_LOW },
  };

#elif defined(STM32F411xE)

#warning "Assume a NUCLEO-F411RE board, PA5, active high."

#define BLINK_PORT_NUMBER         (0)
#define BLINK_PIN_NUMBER          (5)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[1] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER, BLINK_ACTIVE_LOW },
  };

#elif defined(STM32F429xx)

#warning "Assume a STM32F429I-Discovery board, PG13-PG14, active high."

#define BLINK_PORT_NUMBER         (6)
#define BLINK_PIN_NUMBER_GREEN    (13)
#define BLINK_PIN_NUMBER_RED      (14)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[2] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_GREEN, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_RED, BLINK_ACTIVE_LOW },
  };

#else

#warning "Unknown board, assume PA5, active high."

#define BLINK_PORT_NUMBER         (0)
#define BLINK_PIN_NUMBER          (5)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[1] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER, BLINK_ACTIVE_LOW },
  };

#endif

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize bothe manager and employee task */

/* The interrupt number to use for the software interrupt generation.  This
could be any unused number.  In this case the first chip level (non system)
interrupt is used, which happens to be the watchdog on the LPC1768. */
#define mainSW_INTERRUPT_ID		( ( IRQn_Type ) WWDG_IRQn )
#define mainSOFTWARE_INTERRUPT_PRIORITY 		( 10 )
/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )

xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xBinarySemaphoreWWDG;
/* this is the queue manager uses to put the work ticket id */
xQueueHandle xWorkQueue;
extern "C" void EXTI0_IRQHandler(void);
static void EXTILine0_Config(void);
extern "C" void WWDG_IRQHandler( void );
static void prvSetupSoftwareInterrupt();

void vTask0( void *pvParameters );
void vTask1( void *pvParameters );
void vTask2( void *pvParameters );
void vTask3( void *pvParameters );

SemaphoreHandle_t xSemaphore = NULL;
int
main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Light Up!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  Timer timer;
  timer.start ();

  // Perform all necessary initialisations for the LEDs.
  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
    {
      blinkLeds[i].powerUp ();
    }

  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
    {
      blinkLeds[i].turnOn ();
    }

  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
    {
      blinkLeds[i].turnOff ();
    }
  prvSetupSoftwareInterrupt();
  EXTILine0_Config();

    //prvSetupSoftwareInterrupt();
	vTraceEnable(TRC_START);

   /* Before a semaphore is used it must be explicitly created.  In this example a binary semaphore is created. */
	vSemaphoreCreateBinary( xBinarySemaphore );
	vSemaphoreCreateBinary( xBinarySemaphoreWWDG );

    /* Check the semaphore and queue was created successfully. */
    if( xBinarySemaphore != NULL)
    {
		/* Create one of the two tasks. */
		xTaskCreate(	vTask1,		/* Pointer to the function that implements the task. */
						"EXTI-Handler",	/* Text name for the task.  This is to facilitate debugging only. */
						240,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						3,			/* This task will run at priority 1. */
						NULL );		/* We are not using the task handle. */

		/* Create the other task in exactly the same way. */
		xTaskCreate( vTask2, "Periodic-EXTI-EventMaker", 240, NULL, 1, NULL );
		xTaskCreate( vTask0, "WWDG-EventMaker", 240, NULL, 1, NULL );
		xTaskCreate( vTask3, "WWDG-Handler", 240, NULL, 3, NULL );

		/* Start the scheduler so our tasks start executing. */
		vTaskStartScheduler();
    }
    for(;;);
}

void vTask0( void *pvParameters )
{
const char *pcTaskName = "Task 0 is Evoking WWDG Event";
static unsigned int val;


xSemaphoreTake( xBinarySemaphoreWWDG, 0 );

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		 trace_printf( "%s: Initiated\n",pcTaskName );

		 mainTRIGGER_INTERRUPT();

		 trace_printf( "%s: Finished\n",pcTaskName );
		vTaskDelay( 125 / portTICK_RATE_MS );
		blinkLeds[(++val)%4].toggle ();
	}
}
/*-----------------------------------------------------------*/


void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is processing EXTI Handler";
static unsigned int val= 2;


xSemaphoreTake( xBinarySemaphore, 0 );

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Use the semaphore to wait for the event.  The task blocks
		indefinitely meaning this function call will only return once the
		semaphore has been successfully obtained - so there is no need to check
		the returned value. */

		xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
		trace_printf( "%s \n",pcTaskName );
		blinkLeds[(val)%4].toggle ();
	}
}

void vTask2( void *pvParameters )
{
const char *pcTaskName = "Task 2 is Evoking EXTI Event";
static unsigned int val= 2;


	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		 trace_printf( "%s: Initiated\n",pcTaskName );
		 blinkLeds[(val)%4].toggle ();

		 EXTI->SWIER |=  ((uint32_t)0x00001);

		 trace_printf( "%s: Finished\n",pcTaskName );
		 vTaskDelay( 250 / portTICK_RATE_MS );
	}
}


void vTask3( void *pvParameters )
{
	const char *pcTaskName = "Task 3 is Handling WWDG Event";
	static unsigned int val;


	xSemaphoreTake( xBinarySemaphore, 0 );

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Use the semaphore to wait for the event.  The task blocks
		indefinitely meaning this function call will only return once the
		semaphore has been successfully obtained - so there is no need to check
		the returned value. */

		xSemaphoreTake( xBinarySemaphoreWWDG, portMAX_DELAY );
		trace_printf( "%s \n",pcTaskName );
		blinkLeds[(++val)%4].toggle ();
	}
}

/*-----------------------------------------------------------*/

static void EXTILine0_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_SetPriority(EXTI0_IRQn, mainSOFTWARE_INTERRUPT_PRIORITY );
	/* Enable the interrupt. */
	NVIC_EnableIRQ( EXTI0_IRQn );
}

void EXTI0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* 'Give' the semaphore to unblock the task. */
	xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );

	/* Clear the software interrupt bit using the interrupt controllers
	Clear Pending register. */
	//mainCLEAR_INTERRUPT();
	EXTI->PR = ((uint32_t)0x00001);

	/* Giving the semaphore may have unblocked a task - if it did and the
	unblocked task has a priority equal to or above the currently executing
	task then xHigherPriorityTaskWoken will have been set to pdTRUE and
	portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
	higher priority task.

	NOTE: The syntax for forcing a context switch within an ISR varies between
	FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
	the Cortex M3 port layer for this purpose.  taskYIELD() must never be called
	from an ISR! */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}

static void prvSetupSoftwareInterrupt()
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	function so the interrupt priority must be at or below the priority defined
	by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );
	/* Enable the interrupt. */
	NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-----------------------------------------------------------*/

void WWDG_IRQHandler( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* 'Give' the semaphore to unblock the task. */
    xSemaphoreGiveFromISR( xBinarySemaphoreWWDG, &xHigherPriorityTaskWoken );

    /* Clear the software interrupt bit using the interrupt controllers
    Clear Pending register. */
    mainCLEAR_INTERRUPT();

    /* Giving the semaphore may have unblocked a task - if it did and the
    unblocked task has a priority equal to or above the currently executing
    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
    higher priority task.

    NOTE: The syntax for forcing a context switch within an ISR varies between
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the Cortex M3 port layer for this purpose.  taskYIELD() must never be called
    from an ISR! */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
