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

#warning "Assume a STM32F4-Discovery board, PD12-PD15, active high."

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

void vTask1( void *pvParameters );
void vTask2( void *pvParameters );
SemaphoreHandle_t xSemaphore = NULL;

#define TEMP_CODE			0
#define DISP_CODE			1
#define IMU_CODE			2
#define OTHER_CODE          3
#define MAX_LEN				10
#define MAX_QUEUE           4
/* vGenSenderHandler is a task function which takes care of putting data in to the queue. vGenReceiverHandler is task function which takes care of getting data from the queue */
static void vGenSenderHandler( void *pvParameters );
static void vGenReceiverHandler( void *pvParameters );

/* Define the structure type that will be passed on the queue. */
typedef struct
{
	unsigned char code;
	int data[MAX_LEN];
} xData;

/* Declare 3 variables of type xData that will be passed on the queue. */
static const xData xStructsToSend[ 4 ] =
{
	{ TEMP_CODE, 33,22 }, /* Used by temp monitoring task. */
	{ DISP_CODE, 64,-32,10 }, /* Used by display monitoring task. */
	{ IMU_CODE, 45, 25, 125 },
	{ OTHER_CODE, 2017,2016 }  /* Used by other task. */
};

#define mainMAX_MSG_LEN	( 80 )

QueueHandle_t xQueue = NULL;
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

  uint32_t seconds = 0;

  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
    {
      blinkLeds[i].turnOn ();
    }

  // First second is long.
  //timer.sleep (Timer::FREQUENCY_HZ);

  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
    {
      blinkLeds[i].turnOff ();
    }

  //timer.sleep (BLINK_OFF_TICKS);

  ++seconds;
  trace_printf ("Second %u\n", seconds);


	trace_printf("Eclipse-FreeRTOS Project starting \n");
	vTraceEnable(TRC_START);

    /* The queue is created to hold a maximum of 3 structures of type xData. */
    xQueue = xQueueCreate( MAX_QUEUE, sizeof( xData ) );

	if( xQueue != NULL )
	{
		/* Create 3 instances of the task that will write to the queue.  The
		parameter is used to pass the structure that the task should write to the
		queue, all 3 sender tasks are created at priority 2 which is above the priority of the receiver. */
		xTaskCreate( vGenSenderHandler, "Temp-task", 240, ( void * ) &( xStructsToSend[ 0 ] ), 2, NULL );

		xTaskCreate( vGenSenderHandler, "Disp-task", 240, ( void * ) &( xStructsToSend[ 1 ] ), 2, NULL );

		xTaskCreate( vGenSenderHandler, "IMU-task", 240, ( void * ) &( xStructsToSend[ 2 ] ), 2, NULL );

		xTaskCreate( vGenSenderHandler, "Other-task", 240, ( void * ) &( xStructsToSend[ 3 ] ), 2, NULL );

		/* Create the task that will read from the queue.  The task is created with
		priority 1, so below the priority of the sender tasks. */
		xTaskCreate( vGenReceiverHandler, "Receive-task", 240, NULL, 1, NULL );

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}
	else
	{
		/* The queue could not be created. */
	}
}

static void vGenSenderHandler( void *pvParameters )
{
portBASE_TYPE xStatus;
const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;

	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
		/* The first parameter is the queue to which data is being sent.  The
		queue was created before the scheduler was started, so before this task
		started to execute.

		The second parameter is the address of the structure being sent.  The
		address is passed in as the task parameter.

		The third parameter is the Block time - the time the task should be kept
		in the Blocked state to wait for space to become available on the queue
		should the queue already be full.  A block time is specified as the queue
		will become full.  Items will only be removed from the queue when both
		sending tasks are in the Blocked state.. */
		blinkLeds[((xData*)pvParameters)->code].toggle ();
		xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );


		if( xStatus != pdPASS )
		{
			/* We could not write to the queue because it was full - this must
			be an error as the receiving task should make space in the queue
			as soon as both sending tasks are in the Blocked state. */
			trace_printf( "Could not send to the queue.\n" );
		}else
		{
			trace_printf("Sender: data sent \n");
		}

		/* Allow the other sender task to execute. */
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

static void vGenReceiverHandler( void *pvParameters )
{
/* Declare the structure that will hold the values received from the queue. */
xData xReceivedStructure;
portBASE_TYPE xStatus;
const portTickType xTicksToWait = 250 / portTICK_RATE_MS;

	/* This task is also defined within an infinite loop. */
	for( ;; )
	{
		/* As this task only runs when the sending tasks are in the Blocked state,
		and the sending tasks only block when the queue is full, this task should
		always find the queue to be full.  3 is the queue length. */
		if( uxQueueMessagesWaiting( xQueue ) == MAX_QUEUE )
		{
			trace_printf( "Queue should have been full!\n" );
		}

		/* The first parameter is the queue from which data is to be received.  The
		queue is created before the scheduler is started, and therefore before this
		task runs for the first time.

		The second parameter is the buffer into which the received data will be
		placed.  In this case the buffer is simply the address of a variable that
		has the required size to hold the received structure.

		The last parameter is the block time - the maximum amount of time that the
		task should remain in the Blocked state to wait for data to be available
		should the queue already be empty.  A block time is not necessary as this
		task will only run when the queue is full so data will always be available. */
		xStatus = xQueueReceive( xQueue, &xReceivedStructure, 0 );

		if( xStatus == pdPASS )
		{
			blinkLeds[xReceivedStructure.code].toggle ();

			/* Data was successfully received from the queue, print out the received
			value and the source of the value. */
			if( xReceivedStructure.code == TEMP_CODE )
			{
				trace_printf( "Data From Temp-task = %d,%d \n", xReceivedStructure.data[0],xReceivedStructure.data[1] );
			}
			else if (xReceivedStructure.code == DISP_CODE)
			{
				trace_printf( "Data From disp-task = %d,%d,%d \n", xReceivedStructure.data[0],xReceivedStructure.data[1], xReceivedStructure.data[2]);
			}
			else if (xReceivedStructure.code == IMU_CODE)
			{
				trace_printf( "Data From IMU-task = %d,%d,%d\n", xReceivedStructure.data[0],xReceivedStructure.data[1], xReceivedStructure.data[2]);
			}
			else if (xReceivedStructure.code == OTHER_CODE)
			{
				trace_printf( "Data From other-task = %d,%d \n", xReceivedStructure.data[0],xReceivedStructure.data[1] );
			}else
			{
				trace_printf( "in-valid code \n");
			}

			vTaskDelay(xTicksToWait);
		}
		else
		{
			/* We did not receive anything from the queue.  This must be an error
			as this task should only run when the queue is full. */
			trace_printf( "Could not receive from the queue.\n" );
		}
	}
}

void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is running\n";
volatile unsigned long ul;
static unsigned int val;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		/* lets make the sema un-available */

		 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
		 trace_printf( "%s\n",pcTaskName );
	      blinkLeds[(++val)%4].toggle ();
		/* lets make the sema available */
		 xSemaphoreGive( xSemaphore);

		/* Delay for a period. */
		for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
		{
			/* This loop is just a very crude delay implementation.  There is
			nothing to do in here.  Later exercises will replace this crude
			loop with a proper delay/sleep function. */
		}
	}
}
/*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
const char *pcTaskName = "Task 2 is running\n";
volatile unsigned long ul;
static unsigned int val;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		/* lets make the sema un-available */
		 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
	  	 trace_printf( "%s\n",pcTaskName );
	      blinkLeds[(++val)%4].toggle ();
		/* lets make the sema available */
		 xSemaphoreGive( xSemaphore);

		/* Delay for a period. */
	for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
		{
			/* This loop is just a very crude delay implementation.  There is
			nothing to do in here.  Later exercises will replace this crude
			loop with a proper delay/sleep function. */
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
