/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "event_groups.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )



/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/* Define the ports and pins of the buttons. */
#define BUTTON_1_PORT	PORT_0
#define BUTTON_2_PORT PORT_0
#define BUTTON_1_PIN	PIN0
#define BUTTON_2_PIN	PIN1

/* Define the periods for the tasks in ticks. */
#define BUTTON_1_MONITOR_PERIOD			50
#define BUTTON_2_MONITOR_PERIOD 		50
#define PERIODIC_TRANSMITTER_PERIOD	100
#define UART_RECEIVER_PERIOD				20
#define LOAD_1_SIMULATION_PERIOD		10
#define LOAD_2_SIMULATION_PERIOD		100

/* Define the load values for the simulation tasks. */
#define LOAD_1_ITERATIONS	37650
#define	LOAD_2_ITERATIONS 90360

/* Define the bits of the task events in the event group. */
#define BUTTON_1_RISE_BIT ( 1 << 0 )
#define BUTTON_1_FALL_BIT ( 1 << 1 )
#define BUTTON_2_RISE_BIT ( 1 << 2 )
#define BUTTON_2_FALL_BIT ( 1 << 3 )
#define PERIODIC_STR_BIT	( 1 << 4 )

/* Define the event strings to be sent to the UART and their lengths. */
#define EVENT_1_STR			( const signed char * ) "Button 1 Rising\n"
#define EVENT_2_STR			( const signed char * ) "Button 1 Falling\n"
#define EVENT_3_STR			( const signed char * ) "Button 2 Rising\n"
#define EVENT_4_STR			( const signed char * ) "Button 2 Falling\n"
#define EVENT_5_STR			( const signed char * ) "Periodic String\n"
#define EVENT_1_STRLEN	( ( unsigned short ) 17 )
#define EVENT_2_STRLEN	( ( unsigned short ) 18 )
#define EVENT_3_STRLEN	( ( unsigned short ) 17 )
#define EVENT_4_STRLEN	( ( unsigned short ) 18 )
#define EVENT_5_STRLEN	( ( unsigned short ) 17 )
	
/* Define the size of the buffer to hold the run-time stats. */
#define BUFFER_SIZE	190

/* Declare the variables to hold the created tasks. */
TaskHandle_t Button_1_Monitor_Handle = NULL;
TaskHandle_t Button_2_Monitor_Handle = NULL;
TaskHandle_t Periodic_Transmitter_Handle = NULL;
TaskHandle_t Uart_Receiver_Handle = NULL;
TaskHandle_t Load_1_Simulation_Handle = NULL;
TaskHandle_t Load_2_Simulation_Handle = NULL;

/* Declare the variables to hold the task execution time. */
int Task_1_Time_In = 0;
int Task_2_Time_In = 0;
int Task_3_Time_In = 0;
int Task_4_Time_In = 0;
int Task_5_Time_In = 0;
int Task_6_Time_In = 0;

int Task_1_Time_Out = 0;
int Task_2_Time_Out = 0;
int Task_3_Time_Out = 0;
int Task_4_Time_Out = 0;
int Task_5_Time_Out = 0;
int Task_6_Time_Out = 0;

int Task_1_Time_Total = 0;
int Task_2_Time_Total = 0;
int Task_3_Time_Total = 0;
int Task_4_Time_Total = 0;
int Task_5_Time_Total = 0;
int Task_6_Time_Total = 0;

/* Declare the variables to hold the system total time and CPU load. */
int System_Time = 0;
int CPU_Load = 0;

/* Declare a character array to hold the run-time stats. */
char runTimeStatsBuff[BUFFER_SIZE];

/* Declare a variable to hold the created event group. */
EventGroupHandle_t xCreatedEventGroup;

/* Declare a semaphore for the UART Channel. */
SemaphoreHandle_t xSemaphore;

/* Task to be created. */
void Button_1_Monitor(void)
{
	// Declare the xLastWakeTime variable.
	TickType_t xLastWakeTime;
	
	// Declare the previous and current states of button 1.
	pinState_t Button_1_State_Current = PIN_IS_LOW;
	pinState_t Button_1_State_Previous = PIN_IS_LOW;
	
	// Initialize the timer for the first task entrance.
	Task_1_Time_In = T1TC;
	
	/* This task is going to be represented by a tag. */
  vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	
	// Set the GPIO corresponding to the task and clear the idle task GPIO.
	GPIO_write(TASKS_PORT, TASK_1_PIN, PIN_IS_HIGH);
	GPIO_write(TASKS_PORT, TASK_0_PIN, PIN_IS_LOW);
	
	for( ; ; )
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		
		// Read the previous state of button 1.
		Button_1_State_Current = GPIO_read( BUTTON_1_PORT, BUTTON_1_PIN );
		
		// Check if button 1 has been pushed or released.
		if( Button_1_State_Current == PIN_IS_HIGH && Button_1_State_Previous == PIN_IS_LOW )
		{
			// Send "Rising Edge Event" to the consumer task and take the UART channel semaphore.
			xEventGroupSetBits( xCreatedEventGroup, BUTTON_1_RISE_BIT );
				
			// Store the current state of button 1.
			Button_1_State_Previous = Button_1_State_Current;
		}
		else if( Button_1_State_Current == PIN_IS_LOW && Button_1_State_Previous == PIN_IS_HIGH )
		{
			// Send "Falling Edge Event" to the consumer task and take the UART channel semaphore.
			xEventGroupSetBits( xCreatedEventGroup, BUTTON_1_FALL_BIT );
		
			// Store the current state of button 1.
			Button_1_State_Previous = Button_1_State_Current;
		}
		
		// Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, BUTTON_1_MONITOR_PERIOD );
	}
}

void Button_2_Monitor(void)
{
	// Declare the xLastWakeTime variable.
	TickType_t xLastWakeTime;
	
	// Declare the previous and current states of button 1.
	pinState_t Button_2_State_Current = PIN_IS_LOW;
	pinState_t Button_2_State_Previous = PIN_IS_LOW;
	
	// Initialize the timer for the first task entrance.
	Task_2_Time_In = T1TC;
	
	/* This task is going to be represented by a tag. */
  vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	
	// Set the GPIO corresponding to the task and clear the idle task GPIO.
	GPIO_write(TASKS_PORT, TASK_2_PIN, PIN_IS_HIGH);
	GPIO_write(TASKS_PORT, TASK_0_PIN, PIN_IS_LOW);
	
	for( ; ; )
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		
		// Read the previous state of button 1.
		Button_2_State_Current = GPIO_read( BUTTON_2_PORT, BUTTON_2_PIN );
		
		// Check if button 2 has been pushed or released.
		if( Button_2_State_Current == PIN_IS_HIGH && Button_2_State_Previous == PIN_IS_LOW )
		{
			// Send "Rising Edge Event" to the consumer task and take the UART channel semaphore.
			xEventGroupSetBits( xCreatedEventGroup, BUTTON_2_RISE_BIT );
		
			// Store the current state of button 2.
			Button_2_State_Previous = Button_2_State_Current;
		}
		else if( Button_2_State_Current == PIN_IS_LOW && Button_2_State_Previous == PIN_IS_HIGH )
		{
			// Send "Falling Edge Event" to the consumer task and take the UART channel semaphore.
			xEventGroupSetBits( xCreatedEventGroup, BUTTON_2_FALL_BIT );
		
			// Store the current state of button 2.
			Button_2_State_Previous = Button_2_State_Current;
		}
		
		// Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, BUTTON_1_MONITOR_PERIOD );
	}
}

void Periodic_Transmitter(void)
{
	// Declare the xLastWakeTime variable.
	TickType_t xLastWakeTime;
		
	// Initialize the timer for the first task entrance.
	Task_3_Time_In = T1TC;
	
	/* This task is going to be represented by a tag. */
  vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	
	// Set the GPIO corresponding to the task and clear the idle task GPIO.
	GPIO_write(TASKS_PORT, TASK_3_PIN, PIN_IS_HIGH);
	GPIO_write(TASKS_PORT, TASK_0_PIN, PIN_IS_LOW);
	
	for( ; ; )
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		
		// Send "Periodic String Event" to the consumer task and take the UART channel semaphore.
		xEventGroupSetBits( xCreatedEventGroup, PERIODIC_STR_BIT );
		
		// Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, PERIODIC_TRANSMITTER_PERIOD );
	}
}

void Uart_Receiver(void)
{
	// Declare the xLastWakeTime variable.
	TickType_t xLastWakeTime;
	
	// Declare the variable to hold the event group bits.
	EventBits_t uxBits;
	
	// Initialize the timer for the first task entrance.
	Task_4_Time_In = T1TC;
		
	/* This task is going to be represented by a tag. */
  vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	
	// Set the GPIO corresponding to the task and clear the idle task GPIO.
	GPIO_write(TASKS_PORT, TASK_4_PIN, PIN_IS_HIGH);
	GPIO_write(TASKS_PORT, TASK_0_PIN, PIN_IS_LOW);
	
	for( ; ; )
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		
		// Get the bit values of the event group.
		uxBits = xEventGroupGetBits( xCreatedEventGroup );
		
		// Give the semaphore of the UART channel.
		xSemaphoreGive( xSemaphore );
		
		// Check if the bits corresponding to the events have been set.
		if( ( uxBits & BUTTON_1_RISE_BIT ) == BUTTON_1_RISE_BIT && xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) == pdTRUE )
		{
			// Write on UART that event 1 has occured
			vSerialPutString( EVENT_1_STR, EVENT_1_STRLEN );
			
			// Clear the bit corresponding to the event.
			xEventGroupClearBits ( xCreatedEventGroup, BUTTON_1_RISE_BIT );
		}
			
		if( ( uxBits & BUTTON_1_FALL_BIT ) == BUTTON_1_FALL_BIT && xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) == pdTRUE  )
		{
			// Write on UART that event 2 has occured
			vSerialPutString( EVENT_2_STR, EVENT_2_STRLEN );
			
			// Clear the bit corresponding to the event.
			xEventGroupClearBits ( xCreatedEventGroup, BUTTON_1_FALL_BIT );
		}
			
		if( ( uxBits & BUTTON_2_RISE_BIT ) == BUTTON_2_RISE_BIT && xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) == pdTRUE  )
		{
			// Write on UART that event 3 has occured
			vSerialPutString( EVENT_3_STR, EVENT_3_STRLEN );
			
			// Clear the bit corresponding to the event.
			xEventGroupClearBits ( xCreatedEventGroup, BUTTON_2_RISE_BIT );
		}
			
		if( ( uxBits & BUTTON_2_FALL_BIT ) == BUTTON_2_FALL_BIT && xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) == pdTRUE  )
		{
			// Write on UART that event 4 has occured
			vSerialPutString( EVENT_4_STR, EVENT_4_STRLEN );
			
			// Clear the bit corresponding to the event.
			xEventGroupClearBits ( xCreatedEventGroup, BUTTON_2_FALL_BIT );
		}
			
		if( ( uxBits & PERIODIC_STR_BIT ) == PERIODIC_STR_BIT && xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) == pdTRUE  )
		{
			// Write on UART that event 5 has occured
			vSerialPutString( EVENT_5_STR, EVENT_5_STRLEN );
			
			// Clear the bit corresponding to the event.
			xEventGroupClearBits ( xCreatedEventGroup, PERIODIC_STR_BIT );
		}

		// Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, UART_RECEIVER_PERIOD );
	}
}

void Load_1_Simulation(void)
{
	// Declare the empty loop iteration variable.
	uint32_t i = 0;
	
	// Declare the xLastWakeTime variable.
	TickType_t xLastWakeTime;
		
	// Initialize the timer for the first task entrance.
	Task_5_Time_In = T1TC;
	
	/* This task is going to be represented by a tag. */
  vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	
	// Set the GPIO corresponding to the task and clear the idle task GPIO.
	GPIO_write(TASKS_PORT, TASK_5_PIN, PIN_IS_HIGH);
	GPIO_write(TASKS_PORT, TASK_0_PIN, PIN_IS_LOW);
	
	for( ; ; )
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		
		// Create an empty iteration loop to extend the execution time.
		for(i = 0; i < LOAD_1_ITERATIONS; i++)
		{
			
		}
		
		// Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, LOAD_1_SIMULATION_PERIOD );
	}
}

void Load_2_Simulation(void)
{
	// Declare the empty loop iteration variable.
	uint32_t j = 0;
	
	// Declare the xLastWakeTime variable.
	TickType_t xLastWakeTime;
	
	// Initialize the timer for the first task entrance.
	Task_6_Time_In = T1TC;
	
	/* This task is going to be represented by a tag. */
  vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	
	// Set the GPIO corresponding to the task and clear the idle task GPIO.
	GPIO_write(TASKS_PORT, TASK_6_PIN, PIN_IS_HIGH);
	GPIO_write(TASKS_PORT, TASK_0_PIN, PIN_IS_LOW);
	
	for( ; ; )
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();
		
		// Create an empty iteration loop to extend the execution time.
		for(j = 0; j < LOAD_2_ITERATIONS; j++)
		{
			
		}
		
		// Get the run-time stats and store them in the character array.
		//vTaskGetRunTimeStats(runTimeStatsBuff);
		// Write the character array on the UART
		//xSerialPutChar('\n');
		//vSerialPutString((signed char *)runTimeStatsBuff, BUFFER_SIZE);
		
		// Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, LOAD_2_SIMULATION_PERIOD );
	}
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
  /* Create Tasks here */
	xTaskCreatePeriodic(Button_1_Monitor,       		/* Function that implements the task. */
											"B1M",         							/* Text name for the task. */
											100,      									/* Stack size in words, not bytes. */
											( void * ) 0,    						/* Parameter passed into the task. */
											1,													/* Priority at which the task is created. */
											&Button_1_Monitor_Handle,		/* Used to pass out the created task's handle. */
											BUTTON_1_MONITOR_PERIOD);		/* Periodicity of the task. */
																									
	xTaskCreatePeriodic(Button_2_Monitor,       		/* Function that implements the task. */
											"B2M",						          /* Text name for the task. */
											100,      									/* Stack size in words, not bytes. */
											( void * ) 0,    						/* Parameter passed into the task. */
											1,													/* Priority at which the task is created. */
											&Button_2_Monitor_Handle,		/* Used to pass out the created task's handle. */
											BUTTON_2_MONITOR_PERIOD);		/* Periodicity of the task. */			

  xTaskCreatePeriodic(Periodic_Transmitter,       		/* Function that implements the task. */
											"PT",									          /* Text name for the task. */
											100,      											/* Stack size in words, not bytes. */
											( void * ) 0,    								/* Parameter passed into the task. */
											1,															/* Priority at which the task is created. */
											&Periodic_Transmitter_Handle,		/* Used to pass out the created task's handle. */
											PERIODIC_TRANSMITTER_PERIOD);		/* Periodicity of the task. */		

  xTaskCreatePeriodic(Uart_Receiver,       		/* Function that implements the task. */
											"UR",						        /* Text name for the task. */
											100,      							/* Stack size in words, not bytes. */
											( void * ) 0,    				/* Parameter passed into the task. */
											1,											/* Priority at which the task is created. */
											&Uart_Receiver_Handle,	/* Used to pass out the created task's handle. */
											UART_RECEIVER_PERIOD);	/* Periodicity of the task. */	
											
	xTaskCreatePeriodic(Load_1_Simulation,      		/* Function that implements the task. */
											"L1S",							    		/* Text name for the task. */
											100,      									/* Stack size in words, not bytes. */
											( void * ) 0,    						/* Parameter passed into the task. */
											1,													/* Priority at which the task is created. */
											&Load_1_Simulation_Handle,	/* Used to pass out the created task's handle. */
											LOAD_1_SIMULATION_PERIOD);	/* Periodicity of the task. */	
											
	xTaskCreatePeriodic(Load_2_Simulation,       		/* Function that implements the task. */
											"L2S",							        /* Text name for the task. */
											100,      									/* Stack size in words, not bytes. */
											( void * ) 0,    						/* Parameter passed into the task. */
											1,													/* Priority at which the task is created. */
											&Load_2_Simulation_Handle,	/* Used to pass out the created task's handle. */
											LOAD_2_SIMULATION_PERIOD);	/* Periodicity of the task. */	

	/* Attempt to create the event group. */
  xCreatedEventGroup = xEventGroupCreate();
											
	/* Attempt to create a semaphore. */
  xSemaphore = xSemaphoreCreateBinary();
											
	/* Was the event group created successfully? */
  if( xCreatedEventGroup == NULL )
  {
      /* The event group was not created because there was insufficient
      FreeRTOS heap available. */
  }
  else
  {
      /* The event group was created. */
  }

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1. */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1. */
static void configTimer1(void)
{
	T1PR = 100;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART. */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO. */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick. */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


