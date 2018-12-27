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
#include <cstring>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "stm32f4xx_hal_usart.h"

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

// #warning "Assume a STM32F4-Discovery board, PD12-PD15, active high."

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
void vTaskConsole (void *pvParameters);

static void vTask_Menu_display(void *pvParameters );
static void vTask_Command_handling(void *pvParameters );
static void vTask_Command_Processing(void *pvParameters );
static void prvUARTStdioGatekeeperTask( void *pvParameters );

void BSP_PB_Init();
extern "C" void EXTI0_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);

SemaphoreHandle_t xSemaphore = NULL;
xSemaphoreHandle xSem;

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
// Kernel Objects
xQueueHandle	xConsoleQueue = NULL;
xQueueHandle xUARTPrintQueue = NULL;
xQueueHandle	xCommandQueue = NULL;

TimerHandle_t GreenledTimerHandle = NULL;
TimerHandle_t OrangeledTimerHandle = NULL;
static volatile uint8_t IsOrangeTimerActive = 0;

TaskHandle_t tDISP_MenuTaskHandle;
TaskHandle_t tCMD_H_Handle;
TaskHandle_t tCMD_P_Handle;
TaskHandle_t tUARTGatekeeperHandle;

QueueHandle_t xQueue = NULL;
// Define the message_t type as an array of 60 char
typedef uint8_t message_t[60];

static void MX_USART2_UART_Init(void);
static void Error_Handler(void);
void SystemClock_Config(void);
void os_Delay(uint32_t delay_in_ms);

/* UART handler declaration */
UART_HandleTypeDef huart2;
uint8_t usart_temp_buffer;
static uint8_t pb_toggle_count = 0;

typedef struct App_CMD{
	uint8_t CMD_NO;
	uint8_t CMD_ARGS[10];
} CMD_t;
uint8_t command_len = 0;
uint8_t command_buffer[20]= {0,};
typedef enum ENUM_CMD{
	NO_ACTION,LED_ON,LED_OFF,LED_TOGGLE,LED_TOGGLE_OFF,LED_STATUS,RTC_PRINT_DATETIME,EXIT_APP
} cmd_e;
char menu[] = {"\
		\r\nLED_ON\t\t\t-------->\t1\
		\r\nLED_OFF\t\t\t-------->\t2\
		\r\nLED_TOGGLE\t\t-------->\t3\
		\r\nLED_TOGGLE_OFF\t\t-------->\t4\
		\r\nLED_STATUS\t\t-------->\t5\
		\r\nRTC_PRINT_DATETIME\t-------->\t6\
		\r\nEXIT_APP\t\t-------->\t7\
		\r\nType your choice:\t\t"
};

uint8_t getCommandCode(uint8_t* buffer);
void getArguments(uint8_t* arg);
void blue_led_toggle(TimerHandle_t xTimer);
void red_led_toggle(TimerHandle_t xTimer);
void orange_led_toggle(TimerHandle_t xTimer);
void green_led_toggle(TimerHandle_t xTimer);

void led_start_toggle(void);
void led_stop_toggle(void);
void read_led_status(char*);
void read_RTC(char*);
void Exit_APP(void);
void print_error_msg(char*);
int
main(int argc, char* argv[])
{
	SystemInit();
	SystemCoreClockUpdate();
	HAL_Init();

	SystemClock_Config();
	DWT->CTRL |= (1 << 0); //enebale CYCNT in Data watch point trace register

  /* Configure the system clock */
  MX_USART2_UART_Init();

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

  // First second is long.
  //timer.sleep (Timer::FREQUENCY_HZ);

  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
    {
      blinkLeds[i].turnOff ();
    }

  //timer.sleep (BLINK_OFF_TICKS);

  NVIC_SetPriorityGrouping( 0 );
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	GreenledTimerHandle = xTimerCreate( "GLEDTGTimer", pdMS_TO_TICKS(250),pdTRUE,NULL,green_led_toggle);
	OrangeledTimerHandle = xTimerCreate( "OLEDTGTimer", pdMS_TO_TICKS(250),pdTRUE,NULL,orange_led_toggle);
	IsOrangeTimerActive = 1;

	xSem = xSemaphoreCreateBinary();
	vSemaphoreCreateBinary( xSemaphore );

    /* The queue is created to hold a maximum of 3 structures of type xData. */
    xQueue = xQueueCreate( MAX_QUEUE, sizeof( xData ) );

	xCommandQueue = xQueueCreate(10, sizeof (CMD_t *));
	xUARTPrintQueue = xQueueCreate( 10, sizeof( char * ) );

	if( xCommandQueue != NULL && xUARTPrintQueue!= NULL )
	{
		/* Create 3 instances of the task that will write to the queue.  The
		parameter is used to pass the structure that the task should write to the
		queue, all 3 sender tasks are created at priority 2 which is above the priority of the receiver. */

		// Create Tasks
		xTaskCreate( prvUARTStdioGatekeeperTask, "UARTGatekeeper", 512, NULL, 1, &tUARTGatekeeperHandle );

		xTaskCreate( vTask_Menu_display, "tDISP_Menu", 512, NULL, 2, &tDISP_MenuTaskHandle );
		xTaskCreate( vTask_Command_handling, "tCMD_H", 512, NULL, 3, &tCMD_H_Handle );
		xTaskCreate( vTask_Command_Processing, "tCMD_P", 512, NULL, 3, &tCMD_P_Handle );

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
const portTickType xTicksToWait = pdMS_TO_TICKS(1000);

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
			//trace_printf( "Could not send to the queue.\n" );
		}else
		{
			//trace_printf("Sender: data sent \n");
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
const portTickType xTicksToWait = pdMS_TO_TICKS(500);

	/* This task is also defined within an infinite loop. */
	for( ;; )
	{
		/* As this task only runs when the sending tasks are in the Blocked state,
		and the sending tasks only block when the queue is full, this task should
		always find the queue to be full.  3 is the queue length. */
		if( uxQueueMessagesWaiting( xQueue ) == MAX_QUEUE )
		{
			//trace_printf( "Queue should have been full!\n" );
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
				//trace_printf( "Data From Temp-task = %d,%d \n", xReceivedStructure.data[0],xReceivedStructure.data[1] );
			}
			else if (xReceivedStructure.code == DISP_CODE)
			{
				//trace_printf( "Data From disp-task = %d,%d,%d \n", xReceivedStructure.data[0],xReceivedStructure.data[1], xReceivedStructure.data[2]);
			}
			else if (xReceivedStructure.code == IMU_CODE)
			{
				//trace_printf( "Data From IMU-task = %d,%d,%d\n", xReceivedStructure.data[0],xReceivedStructure.data[1], xReceivedStructure.data[2]);
			}
			else if (xReceivedStructure.code == OTHER_CODE)
			{
				//trace_printf( "Data From other-task = %d,%d \n", xReceivedStructure.data[0],xReceivedStructure.data[1] );
			}else
			{
				//trace_printf( "in-valid code \n");
			}

			vTaskDelay(xTicksToWait);
		}
		else
		{
			/* We did not receive anything from the queue.  This must be an error
			as this task should only run when the queue is full. */
			//trace_printf( "Could not receive from the queue.\n" );
		}
	}
}

void vTask1( void *pvParameters )
{
	//const char *pcTaskName = "Task 1 is running\n";
	static unsigned int val;
	message_t 	message;
	message_t	*pm;
	portTickType	xLastWakeTime;
	// Initialize timing
	xLastWakeTime = xTaskGetTickCount();
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		/* lets make the sema un-available */

		 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
		 //trace_printf( "%s\n",pcTaskName );
	      blinkLeds[(++val)%4].toggle ();
		/* lets make the sema available */
		 xSemaphoreGive( xSemaphore);

		// Prepare message
		sprintf((char *)message, "With great power comes great responsibility\r\n");

		// Send message to the Console Queue
		pm = &message;
		xQueueSendToBack(xConsoleQueue, &pm, 0);

		// Wait here for 20ms since last wakeup
		vTaskDelayUntil (&xLastWakeTime, pdMS_TO_TICKS(125));

	}
}
/*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
	//const char *pcTaskName = "Task 2 is running\n";
	static unsigned int val;
	message_t 	message;
	message_t	*pm;
	portTickType	xLastWakeTime;
	// Initialize timing
	xLastWakeTime = xTaskGetTickCount();
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		/* lets make the sema un-available */
		 xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
	  	 //trace_printf( "%s\n",pcTaskName );
	      blinkLeds[(++val)%4].toggle ();
		/* lets make the sema available */
		 xSemaphoreGive( xSemaphore);

		// Prepare message
		 sprintf((char *)message, "#");

		// Send message to Console Queue
		pm = &message;
		xQueueSendToBack(xConsoleQueue, &pm, 0);

		// Wait here for 2ms since last wakeup
		vTaskDelayUntil (&xLastWakeTime, pdMS_TO_TICKS(375));

	}
}

/*
 * Task_Console
 */
void vTaskConsole (void *pvParameters)
{
	message_t *message;
	uint8_t * m;

	while(1)
	{
		// Wait for something in the message Queue
		xQueueReceive(xConsoleQueue, &message, portMAX_DELAY);

		// Send message to console
		//my_printf((const char *)message);
		m = *message;
		while (m && *m){
				while ( __HAL_UART_GET_FLAG (&huart2 , UART_FLAG_TXE ) == RESET);
				USART2 ->DR = *m++ & 0xFF;
		}

	}

}

static void vTask_Menu_display(void *pvParameters ){

	char * pData = &menu[0];
	 uint32_t pulNotificationValue  = 0x02;
	 uint32_t ulBitsToClearOnEntry = 0, ulBitsToClearOnExitRXNIE = 0x01,  ulBitsToClearOnExitPrintMenu = 0x02; //, ulBitsToClearOnExitALL = 0xFFFFFFFF;

	while(1){
		//send data to print manager and if it is full wait indefinitely
		//wait until print manage finishes and let it get more values
		//vTaskDelay(pdMS_TO_TICKS(5));
		if( ( pulNotificationValue & ulBitsToClearOnExitRXNIE ) != 0 )
		{
			/* Bit 0 was set - process whichever event is represented by bit 0. */
			if((huart2.Instance->CR1&USART_CR1_RXNEIE) != USART_CR1_RXNEIE)
				SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
			//wait until print manager finish. two times processing for one order, why?
			xTaskNotifyWait( ulBitsToClearOnEntry,  ulBitsToClearOnExitRXNIE, &pulNotificationValue, portMAX_DELAY);
		}
		if( ( pulNotificationValue & ulBitsToClearOnExitPrintMenu ) != 0 )
		{
			/* Bit 0 was set - process whichever event is represented by bit 0. */
			pb_toggle_count = 0; //from EXTI0 toogle
			xQueueSend(xUARTPrintQueue, &pData, portMAX_DELAY);
			//wait until display manager wakes up this task and send data again( one line above ).
			xTaskNotifyWait( ulBitsToClearOnEntry,  ulBitsToClearOnExitPrintMenu, &pulNotificationValue, portMAX_DELAY);
		}
	}
}

static void prvUARTStdioGatekeeperTask( void *pvParameters )
{

	char * pcUARTMessageToPrint = NULL;

	/* This is the only task that is allowed to write to the terminal output.
	Any other task wanting to write to the output does not access the terminal
	directly, but instead sends the output to this task.  As only one task
	writes to standard out there are no mutual exclusion or serialization issues
	to consider within this task itself. */
	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xUARTPrintQueue, &pcUARTMessageToPrint, portMAX_DELAY );
		if((huart2.Instance->CR1&USART_CR1_RXNEIE) == USART_CR1_RXNEIE)
		CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
		/* There is no need to check the return	value as the task will block
		indefinitely and only run again when a message has arrived.  When the
		next line is executed there will be a message to be output. */
		//trace_printf( "%s\n",*pcUARTMessageToPrint );
		//	vTracePrint(ue3, "UART");
		for(uint32_t i = 0; i < strlen(pcUARTMessageToPrint); i++){
			while ( __HAL_UART_GET_FLAG (&huart2 , UART_FLAG_TXE ) == RESET);
			USART2 ->DR = pcUARTMessageToPrint[i] & 0xFF;
		}

		while ( __HAL_UART_GET_FLAG (&huart2 , UART_FLAG_TC ) == RESET);
		/* Now simply go back to wait for the next message. */
		xTaskNotify(tDISP_MenuTaskHandle,0x01,eSetBits);

	}
}
static void vTask_Command_handling(void *pvParameters ){
    uint8_t cmd_code = 0;
    CMD_t * new_cmd;
	 uint32_t ulBitsToClearOnExit = 0, ulBitsToClearOnEntry = 0;
	 uint32_t pulNotificationValue  = 0x00;

	while(1){
		xTaskNotifyWait(ulBitsToClearOnEntry,ulBitsToClearOnExit,&pulNotificationValue, portMAX_DELAY);
		new_cmd = (CMD_t*)pvPortMalloc(sizeof(CMD_t));

		taskENTER_CRITICAL();
		cmd_code = getCommandCode(command_buffer);
		new_cmd->CMD_NO = cmd_code;
		getArguments(new_cmd->CMD_ARGS);
		taskEXIT_CRITICAL();

		xQueueSend(xCommandQueue, &new_cmd, portMAX_DELAY);

	}
}

uint8_t getCommandCode(uint8_t* buffer){
	return buffer[0] - 48;
}

void getArguments(uint8_t* arg){

}

static void vTask_Command_Processing(void *pvParameters ){
	CMD_t * new_cmd  = NULL;
	char task_msg[50];
	BSP_PB_Init();

	while(1){
		xQueueReceive( xCommandQueue, &new_cmd, portMAX_DELAY );
		switch(new_cmd->CMD_NO){
		case NO_ACTION:
			break;
		case LED_ON:
			blinkLeds[0].turnOn ();
			break;
		case LED_OFF:
			blinkLeds[0].turnOff();
			break;
		case LED_TOGGLE:
			led_start_toggle();
			break;
		case LED_TOGGLE_OFF:
			led_stop_toggle();
			break;
		case LED_STATUS:
			read_led_status(task_msg);
			break;
		case RTC_PRINT_DATETIME:
			read_RTC(task_msg);
			break;
		case EXIT_APP:
			Exit_APP();
			break;
		default:
			print_error_msg(task_msg);
			break;
		}
	}
}

void blue_led_toggle(TimerHandle_t xTimer){
	blinkLeds[3].toggle();
}
void red_led_toggle(TimerHandle_t xTimer){
	blinkLeds[2].toggle();
}
void orange_led_toggle(TimerHandle_t xTimer){
	blinkLeds[1].toggle();
}
void green_led_toggle(TimerHandle_t xTimer){
	blinkLeds[0].toggle();
}
void led_start_toggle(void){
	xTimerStart(GreenledTimerHandle, portMAX_DELAY); // waiting for timer queue available for portMAX_DELAY
}
void led_stop_toggle(void){
	xTimerStop(GreenledTimerHandle, portMAX_DELAY);
}
void read_led_status(char* task_msg){
	sprintf(task_msg, "\r\nG LED : %d\tO LED : %d\tR LED : %d\tB LED : %d\r\n"
			,(int)(GPIOD->IDR & GPIO_PIN_12)
			,(int)(GPIOD->IDR & GPIO_PIN_13)
			,(int)(GPIOD->IDR & GPIO_PIN_14)
			,(int)(GPIOD->IDR & GPIO_PIN_15));
	xQueueSend(xUARTPrintQueue, &task_msg, portMAX_DELAY );
}
void read_RTC(char* task_msg){

}
void Exit_APP(void){

}
void print_error_msg(char* task_msg){
	sprintf(task_msg, "\r\nError: Invalid Command Received\r\n");
	xQueueSend(xUARTPrintQueue, &task_msg, portMAX_DELAY );
}

void os_Delay(uint32_t delay_in_ms){
	uint32_t current_tick_count = xTaskGetTickCount();
	uint32_t delay_tick = delay_in_ms * configTICK_RATE_HZ / 1000;// 1000 ms
	uint32_t wait_until = delay_tick+ current_tick_count;
	while(wait_until > xTaskGetTickCount());
}
void BSP_PB_Init(){
	GPIO_InitTypeDef   GPIO_InitStructure;

	/* Enable GPIOA clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	/* Enable the interrupt. */
	NVIC_EnableIRQ( EXTI0_IRQn );

}

char  toggle_on_msg[] = "\r\nOrange led toggle on:\t\t";
char  toggle_off_msg[] = "\r\nOrange led toggle off:\t\t";

void EXTI0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	// Test for line 13 pending interrupt
	if ((EXTI->PR & EXTI_PR_PR0_Msk) != 0)
	{
		char * pData = &menu[0];
		char * pDataStart = &toggle_on_msg[0];
		char * pDataStop = &toggle_off_msg[0];
		// Clear pending bit 13 by writing a '1'
		EXTI->PR |= EXTI_PR_PR0;
		if( IsOrangeTimerActive )
		{
			xTimerStopFromISR(OrangeledTimerHandle, &xHigherPriorityTaskWoken); // waiting for timer queue available for portMAX_DELAY
			IsOrangeTimerActive = 0; pb_toggle_count++;
			xQueueSendFromISR(xUARTPrintQueue, &pDataStop, &xHigherPriorityTaskWoken);
		}
		else
		{
			xTimerStartFromISR(OrangeledTimerHandle, &xHigherPriorityTaskWoken); // waiting for timer queue available for portMAX_DELAY
			IsOrangeTimerActive = 1; pb_toggle_count++;
			xQueueSendFromISR(xUARTPrintQueue, &pDataStart, &xHigherPriorityTaskWoken);
		}
		if(pb_toggle_count> 8){
			xQueueSendFromISR(xUARTPrintQueue, &pData, &xHigherPriorityTaskWoken);
			pb_toggle_count = 0;
		}
		// Release the semaphore
		//xSemaphoreGiveFromISR(xSem, &xHigherPriorityTaskWoken);

		// Perform a context switch to the waiting task
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

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

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void USART2_IRQHandler(void){

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)){

		uint16_t data_byte=(uint16_t)(huart2.Instance->DR & (uint16_t)0x01FFU);

		command_buffer[command_len++] = data_byte & 0xFF;

		if(data_byte == '\r') //user finished entering data
		{
			command_len = 0;

			xTaskNotifyFromISR(tCMD_H_Handle,0x00,eNoAction, &pxHigherPriorityTaskWoken);
			xTaskNotifyFromISR(tDISP_MenuTaskHandle,0x02,eSetBits, &pxHigherPriorityTaskWoken);
		}

	}
	//if higher priority task woke up after interrupt handler executed, yield to the higher pririty task
	if(pxHigherPriorityTaskWoken == pdTRUE) taskYIELD();
}

static void Error_Handler(void)
{
  /* Turn LED5 on */
  while(1)
  {
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
