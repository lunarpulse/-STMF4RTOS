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
#include <string>

#include "diag/Trace.h"
#include "stm32f4xx_hal.h"

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

/* Dimensions the buffer into which messages destined for stdout are placed. */
#define mainMAX_MSG_LEN	( 80 )

/* The task that sends messages to the stdio gatekeeper.  Two instances of this
task are created. */
//static void prvPrintTask( void *pvParameters );

/* The gatekeeper task itself. */
static void prvStdioGatekeeperTask( void *pvParameters );

void Error_Handler(void);
void SystemClock_Config(void);

/* Define the strings that the tasks and interrupt will print out via the gatekeeper. */
std::string pcStringsToPrint [3] = {"Task 1 ****************************************************\n",
		"Task 2 ----------------------------------------------------\n",
		"Message printed from the tick hook interrupt ##############\n"};

/*-----------------------------------------------------------*/

/* Declare a variable of type xQueueHandle.  This is used to send messages from
the print tasks to the gatekeeper task. */
xQueueHandle xPrintQueue;

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void vTask1( void *pvParameters );
void vTask2( void *pvParameters );
extern "C" void vApplicationIdleHook( void );
extern "C" void vApplicationTickHook( void );

SemaphoreHandle_t xSemaphore = NULL;

const char* pcTaskName2 = "Task 2 is running\n";
xTaskHandle xTask2Handle;
static volatile unsigned long ulIdleCount = 0UL;
int
main(int argc, char* argv[])
{

	HAL_Init();
	SystemClock_Config();
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

	const char* pcTaskName1 = "Task 1 is running\n";

	/* Create one of the two tasks. */
	xTaskCreate(	vTask1,		/* Pointer to the function that implements the task. */
					"Task 1",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					(void*)pcTaskName1,		/* We are not using the task parameter. */
					2,			/* This task will run at priority 1. */
					NULL );		/* We are not using the task handle. */

	/* Create the other task in exactly the same way. */
	xTaskCreate( vTask2, "Task 2", 240, (void*)pcTaskName2, 1, &xTask2Handle );


/* lets create the binary semaphore dynamically */
	xSemaphore = xSemaphoreCreateBinary();

	/* lets make the semaphore token available for the first time */
	xSemaphoreGive( xSemaphore);

	/* Start the scheduler so our tasks start executing. */
	//vTaskStartScheduler();

    /* Before a queue is used it must be explicitly created.  The queue is created
	to hold a maximum of 5 character pointers. */
    xPrintQueue = xQueueCreate( 5, sizeof( char * ) );

	/* The tasks are going to use a pseudo random delay, seed the random number
	generator. */
	//srand( 567 );

	/* Check the queue was created successfully. */

	/* Create two instances of the tasks that send messages to the gatekeeper.
	The	index to the string they attempt to write is passed in as the task
	parameter (4th parameter to xTaskCreate()).  The tasks are created at
	different priorities so some pre-emption will occur. */

	/* Create the gatekeeper task.  This is the only task that is permitted
	to access standard out. */
	xTaskCreate( prvStdioGatekeeperTask, "Gatekeeper", 240, NULL, 0, NULL );

	/* Start the scheduler so the created tasks start executing. */
	vTaskStartScheduler();

}

static void prvStdioGatekeeperTask( void *pvParameters )
{
char *pcMessageToPrint;
static char cBuffer[ mainMAX_MSG_LEN ];

	/* This is the only task that is allowed to write to the terminal output.
	Any other task wanting to write to the output does not access the terminal
	directly, but instead sends the output to this task.  As only one task
	writes to standard out there are no mutual exclusion or serialization issues
	to consider within this task itself. */
	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xPrintQueue, &pcMessageToPrint, portMAX_DELAY );

		/* There is no need to check the return	value as the task will block
		indefinitely and only run again when a message has arrived.  When the
		next line is executed there will be a message to be output. */
		sprintf( cBuffer, "%s", pcMessageToPrint );
		trace_printf( "%s\n",cBuffer );

		/* Now simply go back to wait for the next message. */
	}
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
static int iCount = 0;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Print out a message every 200 ticks.  The message is not written out
	directly, but sent to the gatekeeper task. */
	iCount++;
	if( iCount >= 200 )
	{
		/* In this case the last parameter (xHigherPriorityTaskWoken) is not
		actually used but must still be supplied. */
		xQueueSendToFrontFromISR( xPrintQueue, &( pcStringsToPrint[ 2 ] ), &xHigherPriorityTaskWoken );

		/* Reset the count ready to print out the string again in 200 ticks
		time. */
		iCount = 0;
	}
}
/*-----------------------------------------------------------*/

void vTask1( void *pvParameters )
{
//const char *pcTaskName = "Task 1 is running\n";
static unsigned int val;
char* toSay = (char*)pvParameters;
//const portTickType xDelayCustom = 250/portTICK_RATE_MS;
unsigned portBASE_TYPE uxPiority;
uxPiority = uxTaskPriorityGet(xTask2Handle);

TickType_t xLastWakeTime;
const TickType_t xFrequency = 125;
xLastWakeTime = xTaskGetTickCount();

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		/* lets make the sema un-available */

		 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
		 trace_printf( "%s, at idle count: %d\n",toSay ,ulIdleCount);
	      blinkLeds[(++val)%4].toggle ();
		/* lets make the sema available */
		 xSemaphoreGive( xSemaphore);
		 if(val%4 == 0){
			 xQueueSendToBack( xPrintQueue, &pcStringsToPrint[ 0 ], 0 );

			 trace_printf( "%s after task2 priority increased by 2\n",pvParameters );
			 vTaskPrioritySet(xTask2Handle, (uxPiority+2));

		 }else
		 {
			 //vTaskDelay(xDelayCustom);
			 vTaskDelayUntil(&xLastWakeTime, xFrequency);
		 }
	}
}
/*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
const char *pcTaskName = "Task 2 is running\n";
unsigned int count = 0;
unsigned portBASE_TYPE uxPiority;
uxPiority = uxTaskPriorityGet(NULL);

TickType_t xLastWakeTime;
const TickType_t xFrequency = 1000;
xLastWakeTime = xTaskGetTickCount();

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		/* lets make the sema un-available */
		 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
	  	 trace_printf( "%s, at idle count: %d\n",pcTaskName ,ulIdleCount );
	      blinkLeds[(count++)%4].toggle ();
		/* lets make the sema available */
		 xSemaphoreGive( xSemaphore);

		 vTaskDelay(8/portTICK_RATE_MS);
		 if(count> 31) {

			xQueueSendToBack( xPrintQueue, &pcStringsToPrint[ 1 ], 0 );

			trace_printf( "%s for 31 iterations and lower its priority\n", pcTaskName );
			count = 0;
			//vTaskDelay(1000/portTICK_RATE_MS);
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
			if(uxPiority>3) vTaskPrioritySet(NULL, (uxPiority-2));
			else vTaskPrioritySet(NULL, 1);
		 }
	}

}
/*-----------------------------------------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
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
	ulIdleCount++;
#if  0
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
#endif
}
/*-----------------------------------------------------------*/
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
