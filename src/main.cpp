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
xSemaphoreHandle xBinarySemaphore;
/* this is the queue manager uses to put the work ticket id */
xQueueHandle xWorkQueue;

void vTask1( void *pvParameters );
void vTask2( void *pvParameters );
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

	vTraceEnable(TRC_START);

   /* Before a semaphore is used it must be explicitly created.  In this example a binary semaphore is created. */
	vSemaphoreCreateBinary( xBinarySemaphore );

	/* lets create the binary semaphore dynamically */
	xSemaphore = xSemaphoreCreateBinary();

	/* lets make the semaphore token available for the first time */
	xSemaphoreGive( xSemaphore);

	/* The queue is created to hold a maximum of 1 Element. */
	xWorkQueue = xQueueCreate( 1, sizeof( unsigned int ) );

	/* The tasks are going to use a pseudo random delay, seed the random number generator. */
	srand( 789 );

    /* Check the semaphore and queue was created successfully. */
    if( (xBinarySemaphore != NULL) && (xWorkQueue != NULL) )
    {
		/* Create one of the two tasks. */
		xTaskCreate(	vTask1,		/* Pointer to the function that implements the task. */
						"Manager",	/* Text name for the task.  This is to facilitate debugging only. */
						240,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						3,			/* This task will run at priority 1. */
						NULL );		/* We are not using the task handle. */

		/* Create the other task in exactly the same way. */
		xTaskCreate( vTask2, "Employee", 240, NULL, 1, NULL );

		/* Start the scheduler so our tasks start executing. */
		vTaskStartScheduler();
    }
    for(;;);
}

void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is sending";
static unsigned int val;

unsigned int xWorkTicketId;
portBASE_TYPE xStatus;

xSemaphoreGive( xBinarySemaphore);
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* get a work ticket id */
		xWorkTicketId = ( rand() & 0x1FF );

		/* Sends work ticket id to the work queue */
		xStatus = xQueueSend( xWorkQueue, &xWorkTicketId , portMAX_DELAY );

		if( xStatus != pdPASS )
		{
			trace_printf( "Could not send to the queue.\n" );

		}else
		{

			/* Manager notifying the employee by "Giving" semaphore */
			xSemaphoreGive( xBinarySemaphore);
			/* after assigning the work , just yield the processor because nothing to do */
			 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
			 trace_printf( "%s: %d\n",pcTaskName, xWorkTicketId );
			  blinkLeds[(++val)%4].toggle ();
			/* lets make the sema available */
			 xSemaphoreGive( xSemaphore);

			vTaskDelay(100/portTICK_RATE_MS);

			taskYIELD();
		}
	}
}
/*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
const char *pcTaskName = "Task 2 is working on";
static unsigned int val;

unsigned char xWorkTicketId;
portBASE_TYPE xStatus;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* First Employee tries to take the semaphore, if it is available that means there is a task assigned by manager, otherwise employee task will be blocked */
		xSemaphoreTake( xBinarySemaphore, 0 );
		vTaskDelay(100/portTICK_RATE_MS);

		/*if we are here means, Semaphore take successfull. So, get the ticket id from the work queue */
		xStatus = xQueueReceive( xWorkQueue, &xWorkTicketId, 0 );

		if( xStatus == pdPASS )
		{
			/* lets make the sema un-available */
			 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
			  blinkLeds[(++val)%4].toggle ();
			/* lets make the sema available */
			 xSemaphoreGive( xSemaphore);
			 trace_printf( "%s: %d\n",pcTaskName, xWorkTicketId );

		}
		else
		{
			/* We did not receive anything from the queue.  This must be an error as this task should only run when the manager assigns at least one work. */
			trace_printf( "Error getting the xWorkTicketId from queue\n" );
		}

	}
}
/*-----------------------------------------------------------*/

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
