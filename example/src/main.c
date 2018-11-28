/*
 * @brief FreeRTOS examples
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdlib.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define EJ4		(1)		/* Creating tasks */
#define EJ5		(2)		/* Using the task parameter */
#define EJ6		(3)		/* Experimenting with priorities */

#define TP (EJ6)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED state is off */
	Board_LED_Set(LED3, LED_OFF);
}


#if (TP == EJ4)		/* Creating tasks */

const char *pcTextForMain = "\r\nEjercicio 4 - TP3\r\n";

#define mainDELAY_LOOP_COUNT		(0xfffff)

#define mainSW_INTERRUPT_ID		(0)
/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ(mainSW_INTERRUPT_ID)

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ(mainSW_INTERRUPT_ID)

/* The priority of the software interrupt.  The interrupt service routine uses
 * an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
 * be equal to or lower than the priority set by
 * configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex-M3 high
 * numeric values represent low priority values, which can be confusing as it is
 * counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY	(5)

/* The two task functions. */
static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);
static void vTask3(void *pvParameters);

static void prvSetupSoftwareInterrupt();

#define vSoftwareInterruptHandler (DAC_IRQHandler)

xSemaphoreHandle xBinarySemaphore;
xQueueHandle xQueue;

/* A variable that is incremented by the idle task hook function. */
unsigned long ulIdleCycleCount = 0UL;

/* UART (or output) & LED ON thread */
static void vTask1(void *pvParameters)
{
	volatile unsigned long ul;
	while(1){
		DEBUGOUT("Tarea 1 espera bloqueando\r\n");
		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}
		DEBUGOUT("T1 - A punto de generar interrupción.\r\n");
		mainTRIGGER_INTERRUPT();
		vTaskDelay(50 / portTICK_RATE_MS);
		DEBUGOUT("T1 - Continuando...\r\n");
	}
}

/* UART (or output) & LED OFF thread */
static void vTask2(void *pvParameters)
{
	uint8_t IntToSend=0;
	portBASE_TYPE xStatus;
	volatile unsigned long ul;

	xSemaphoreTake(xBinarySemaphore, (portTickType) 0);
	DEBUGOUT("Tarea 2 toma el semáforo\r\n");

	/* As per most tasks, this task is implemented in an infinite loop. */
	while (1) {
		Board_LED_Set(LEDB, LED_ON);

		IntToSend++;
		if (IntToSend>=3){
			IntToSend=0;
		}

		/* Print out the name of this task. */


		xStatus = xQueueSendToBack(xQueue, &IntToSend, (portTickType)0);
		DEBUGOUT("Tarea 2 pone valor en cola\r\n");
		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}

		Board_LED_Set(LEDB, LED_OFF);
		DEBUGOUT("Tarea 2 espera a volver a tomar semáforo\r\n");
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		/* Delay for a period. */
/*		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}*/
	}
}

static void vTask3(void *pvParameters){
	/* Declare the variable that will hold the values received from the queue. */
	uint8_t IntReceived;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 500 / portTICK_RATE_MS;
	volatile unsigned long ul;

	while (1) {

		/* As this task unblocks immediately that data is written to the queue this
		 * call should always find the queue empty. */
		if (uxQueueMessagesWaiting(xQueue) != 0) {
			DEBUGOUT("Queue should have been empty!\r\n");
		}

		/* The first parameter is the queue from which data is to be received.  The
		 * queue is created before the scheduler is started, and therefore before this
		 * task runs for the first time.
		 *
		 * The second parameter is the buffer into which the received data will be
		 * placed.  In this case the buffer is simply the address of a variable that
		 * has the required size to hold the received data.
		 *
		 * The last parameter is the block time � the maximum amount of time that the
		 * task should remain in the Blocked state to wait for data to be available should
		 * the queue already be empty. */
		xStatus = xQueueReceive(xQueue, &IntReceived, xTicksToWait);
		DEBUGOUT("Tarea 3 recibe de cola\r\n");

		switch (IntReceived) {
		case 0:
			Board_LED_Set(LED1, LED_ON);
			DEBUGOUT("Tarea 3 prende y apaga LED1\r\n");
			for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
			}
			Board_LED_Set(LED1, LED_OFF);
			break;
		case 1:
			Board_LED_Set(LED2, LED_ON);
			DEBUGOUT("Tarea 3 prende y apaga LED2\r\n");
			for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
			}
			Board_LED_Set(LED2, LED_OFF);
			break;
		case 2:
			Board_LED_Set(LED3, LED_ON);
			DEBUGOUT("Tarea 3 prende y apaga LED3\r\n");
			for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
			}
			Board_LED_Set(LED3, LED_OFF);
			break;
		default:
			break;
		}

//		if (xStatus == pdPASS) {
			/* Data was successfully received from the queue, print out the received
			 * value. */
//			DEBUGOUT("Received = %d\r\n", lReceivedValue);
//		}
//		else {
			/* We did not receive anything from the queue even after waiting for 100ms.
			 * This must be an error as the sending tasks are free running and will be
			 * continuously writing to the queue. */
//			DEBUGOUT("Could not receive from the queue.\r\n");
//		}

	}
}

void vApplicationIdleHook(void)
{
	/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;

	DEBUGOUT("Tarea idle\r\n");

	/* Best to sleep here until next systick */
	__WFI();
}

static void prvSetupSoftwareInterrupt()
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	 * function so the interrupt priority must be at or below the priority defined
	 * by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority(mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY);

	/* Enable the interrupt. */
	NVIC_EnableIRQ(mainSW_INTERRUPT_ID);
}


void vSoftwareInterruptHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	DEBUGOUT("Interrupción\r\n");

    /* 'Give' the semaphore to unblock the task. */
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

    /* Clear the software interrupt bit using the interrupt controllers
     * Clear Pending register. */
    mainCLEAR_INTERRUPT();

    /* Giving the semaphore may have unblocked a task - if it did and the
     * unblocked task has a priority equal to or above the currently executing
     * task then xHigherPriorityTaskWoken will have been set to pdTRUE and
     * portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
     * higher priority task.
     *
     * NOTE: The syntax for forcing a context switch within an ISR varies between
     * FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
     * the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
     * from an ISR! */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	main routine for FreeRTOS example 1 - Creating tasks
 * @return	Nothing, function should not exit
 */
int main(void)
{
	/* Sets up system hardware */
	prvSetupHardware();

	/* Print out the name of this example. */
	DEBUGOUT(pcTextForMain);

	vSemaphoreCreateBinary(xBinarySemaphore);
	xQueue=xQueueCreate(1, sizeof(uint8_t));

	if ((xBinarySemaphore != (xSemaphoreHandle) NULL) && (xQueue != (xQueueHandle)NULL) ) {
		/* Habilita interrupción de software y establece su prioridad. */
		prvSetupSoftwareInterrupt();

		/* Tarea a sincronizar con la interrupción. Tiene la prioridad más alta así es lo
		 * primero que se ejecuta al salir de la ISR
		 */
		xTaskCreate(vTask2, (char *) "Task2", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 3UL), (xTaskHandle *) NULL);

		/* Creación de la tarea que va a generar la interrupción de software.*/
		xTaskCreate(vTask1, (char *) "Task1", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

		/* Creación de la tarea sincronizada con vTask2 */
		xTaskCreate(vTask3, (char *) "Task3", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 2UL), (xTaskHandle *) NULL);

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

	/* If all is well we will never reach here as the scheduler will now be
	 * running.  If we do reach here then it is likely that there was insufficient
	 * heap available for the idle task to be created. */
	while (1);

	/* Should never arrive here */
	return ((int) NULL);
}
#endif

#if (TP == EJ5)		/* Creating tasks */

const char *pcTextForMain = "\r\nEjercicio 5 - TP3\r\n";

#define mainDELAY_LOOP_COUNT		(0xfffff)

#define mainSW_INTERRUPT_ID		(0)
/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ(mainSW_INTERRUPT_ID)

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ(mainSW_INTERRUPT_ID)

/* The priority of the software interrupt.  The interrupt service routine uses
 * an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
 * be equal to or lower than the priority set by
 * configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex-M3 high
 * numeric values represent low priority values, which can be confusing as it is
 * counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY	(5)

/* The two task functions. */
static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);
static void vTask3(void *pvParameters);

static void prvSetupSoftwareInterrupt();

#define vSoftwareInterruptHandler (DAC_IRQHandler)

xSemaphoreHandle xBinarySemaphore;
xQueueHandle xQueue;

/* A variable that is incremented by the idle task hook function. */
unsigned long ulIdleCycleCount = 0UL;

/* UART (or output) & LED ON thread */
static void vTask1(void *pvParameters)
{
	volatile unsigned long ul;
	while(1){
		DEBUGOUT("Tarea 1 espera bloqueando\r\n");
		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}
		DEBUGOUT("T1 - A punto de generar interrupción.\r\n");
		mainTRIGGER_INTERRUPT();
		vTaskDelay(50 / portTICK_RATE_MS);
		DEBUGOUT("T1 - Continuando...\r\n");
	}
}

/* UART (or output) & LED OFF thread */
static void vTask2(void *pvParameters)
{
	uint8_t CurrentLED=0;
	uint8_t IntReceived=0;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;
	volatile unsigned long ul;

	/* As per most tasks, this task is implemented in an infinite loop. */
	while (1) {

		/* As this task unblocks immediately that data is written to the queue this
		 * call should always find the queue empty. */
		if (uxQueueMessagesWaiting(xQueue) != 0) {
			DEBUGOUT("Queue should have been empty!\r\n");
		}

		xStatus = xQueueReceive(xQueue, &IntReceived, xTicksToWait);
		DEBUGOUT("Tarea 2 recibe de cola\r\n");

		if (IntReceived==1){
			CurrentLED++;
		}
		if (CurrentLED>=3){
			CurrentLED=0;
		}
		IntReceived=0;
		/* Print out the name of this task. */

		switch (CurrentLED) {
		case 0:
			Board_LED_Set(LED1, LED_ON);
			DEBUGOUT("Tarea 2 prende y apaga LED1\r\n");
			for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
			}
			Board_LED_Set(LED1, LED_OFF);
			break;
		case 1:
			Board_LED_Set(LED2, LED_ON);
			DEBUGOUT("Tarea 2 prende y apaga LED2\r\n");
			for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
			}
			Board_LED_Set(LED2, LED_OFF);
			break;
		case 2:
			Board_LED_Set(LED3, LED_ON);
			DEBUGOUT("Tarea 2 prende y apaga LED3\r\n");
			for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
			}
			Board_LED_Set(LED3, LED_OFF);
			break;
		default:
			break;
		}

		DEBUGOUT("Tarea 2 da el semáforo\r\n");
		xSemaphoreGive(xBinarySemaphore);
		/* Delay for a period. */
/*		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}*/
	}
}

static void vTask3(void *pvParameters){

	volatile unsigned long ul;
	const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

	DEBUGOUT("Tarea 3 toma el semáforo\r\n");
	xSemaphoreTake(xBinarySemaphore,(portTickType)0);

	while (1) {

		DEBUGOUT("Tarea 3 toma semáforo\r\n");
		Board_LED_Set(LEDB, LED_ON);

		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}

		Board_LED_Set(LEDB, LED_OFF);
		DEBUGOUT("Tarea 3 espera a volver a tomar semáforo\r\n");
		xSemaphoreTake(xBinarySemaphore, xTicksToWait);

	}
}

void vApplicationIdleHook(void)
{
	/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;

	DEBUGOUT("Tarea idle\r\n");

	/* Best to sleep here until next systick */
	__WFI();
}

static void prvSetupSoftwareInterrupt()
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	 * function so the interrupt priority must be at or below the priority defined
	 * by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority(mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY);

	/* Enable the interrupt. */
	NVIC_EnableIRQ(mainSW_INTERRUPT_ID);
}


void vSoftwareInterruptHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	DEBUGOUT("Interrupción\r\n");
	uint8_t data=1;
    /* 'Give' the semaphore to unblock the task. */
    //xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
	DEBUGOUT("ISR envía dato a cola\r\n");
    xQueueSendFromISR(xQueue,&data,&xHigherPriorityTaskWoken);
    /* Clear the software interrupt bit using the interrupt controllers
     * Clear Pending register. */
    mainCLEAR_INTERRUPT();

    /* Giving the semaphore may have unblocked a task - if it did and the
     * unblocked task has a priority equal to or above the currently executing
     * task then xHigherPriorityTaskWoken will have been set to pdTRUE and
     * portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
     * higher priority task.
     *
     * NOTE: The syntax for forcing a context switch within an ISR varies between
     * FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
     * the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
     * from an ISR! */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	main routine for FreeRTOS example 1 - Creating tasks
 * @return	Nothing, function should not exit
 */
int main(void)
{
	/* Sets up system hardware */
	prvSetupHardware();

	/* Print out the name of this example. */
	DEBUGOUT(pcTextForMain);

	vSemaphoreCreateBinary(xBinarySemaphore);
	xQueue=xQueueCreate(1, sizeof(uint8_t));

	if ((xBinarySemaphore != (xSemaphoreHandle) NULL) && (xQueue != (xQueueHandle)NULL) ) {
		/* Habilita interrupción de software y establece su prioridad. */
		prvSetupSoftwareInterrupt();

		/* Tarea a sincronizar con la interrupción. Tiene la prioridad más alta así es lo
		 * primero que se ejecuta al salir de la ISR
		 */
		xTaskCreate(vTask2, (char *) "Task2", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 3UL), (xTaskHandle *) NULL);

		/* Creación de la tarea que va a generar la interrupción de software.*/
		xTaskCreate(vTask1, (char *) "Task1", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

		/* Creación de la tarea sincronizada con vTask2 */
		xTaskCreate(vTask3, (char *) "Task3", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 2UL), (xTaskHandle *) NULL);

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

	/* If all is well we will never reach here as the scheduler will now be
	 * running.  If we do reach here then it is likely that there was insufficient
	 * heap available for the idle task to be created. */
	while (1);

	/* Should never arrive here */
	return ((int) NULL);
}
#endif

#if (TP == EJ6)

const char *pcTextForMain = "\r\nEjercicio 6 - TP3\r\n";

#define mainDELAY_LOOP_COUNT		(0xfffff)

#define mainSW_INTERRUPT_ID		(0)
/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ(mainSW_INTERRUPT_ID)

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ(mainSW_INTERRUPT_ID)

/* The priority of the software interrupt.  The interrupt service routine uses
 * an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
 * be equal to or lower than the priority set by
 * configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex-M3 high
 * numeric values represent low priority values, which can be confusing as it is
 * counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY	(5)

/* The two task functions. */
static void vTaskA(void *pvParameters);
static void vTaskB(void *pvParameters);
static void vTaskC(void *pvParameters);

static void prvSetupSoftwareInterrupt();

#define vSoftwareInterruptHandler (DAC_IRQHandler)

xSemaphoreHandle xBinarySemaphoreA,xBinarySemaphoreB,xBinarySemaphoreC;
xTaskHandle xTaskAHandle,xTaskBHandle,xTaskCHandle;


/* A variable that is incremented by the idle task hook function. */
unsigned long ulIdleCycleCount = 0UL;

/* UART (or output) & LED ON thread */
static void vTaskA(void *pvParameters){

	volatile unsigned long ul;
	const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

	DEBUGOUT("Tarea A se ejecuta por primera vez\r\n");
	xSemaphoreTake(xBinarySemaphoreA,xTicksToWait);

	while(1){
		Board_LED_Set(LED1, LED_ON);
		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}
		/*
		 * Acá va rutina de encendido del LED, leyendo de una cola exclusiva de Task A
		 * */
		Board_LED_Set(LED1, LED_OFF);
		xSemaphoreGive(xBinarySemaphoreB);
		xSemaphoreTake(xBinarySemaphoreA,xTicksToWait);
		DEBUGOUT("Tarea A se ejecuta\r\n");
	}
}

/* UART (or output) & LED OFF thread */
static void vTaskB(void *pvParameters){

	volatile unsigned long ul;
	const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

	vTaskPrioritySet(xTaskAHandle,(tskIDLE_PRIORITY + 1UL));
	vTaskPrioritySet(xTaskBHandle,(tskIDLE_PRIORITY + 2UL));
	DEBUGOUT("Tarea B se ejecuta por primera vez\r\n");

	xSemaphoreTake(xBinarySemaphoreB,xTicksToWait);


	while(1){
		Board_LED_Set(LED2, LED_ON);
		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}
		/*
		 * Acá va rutina de encendido del LED, leyendo de una cola exclusiva de Task B
		 * */
		Board_LED_Set(LED2, LED_OFF);
		xSemaphoreGive(xBinarySemaphoreC);
		xSemaphoreTake(xBinarySemaphoreB,xTicksToWait);
		DEBUGOUT("Tarea B se ejecuta\r\n");
	}
}

static void vTaskC(void *pvParameters){
	volatile unsigned long ul;
	const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

	DEBUGOUT("Tarea C se ejecuta por primera vez\r\n");
	xSemaphoreGive(xBinarySemaphoreA);
	xSemaphoreTake(xBinarySemaphoreC,xTicksToWait);
	vTaskPrioritySet(xTaskCHandle,(tskIDLE_PRIORITY + 3UL));
	while(1){
		Board_LED_Set(LED3, LED_ON);
		for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
		}
		/*
		 * Acá va rutina de encendido del LED, leyendo de una cola exclusiva de Task C
		 * */
		Board_LED_Set(LED3, LED_OFF);
		xSemaphoreGive(xBinarySemaphoreA);
		xSemaphoreTake(xBinarySemaphoreC,xTicksToWait);
		DEBUGOUT("Tarea C se ejecuta\r\n");
	}

}

void vApplicationIdleHook(void)
{
	/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;

	Board_LED_Set(LEDG, LED_ON);
	DEBUGOUT("Tarea idle\r\n");
	Board_LED_Set(LED3, LED_OFF);
	/* Best to sleep here until next systick */
	__WFI();
}

static void prvSetupSoftwareInterrupt()
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	 * function so the interrupt priority must be at or below the priority defined
	 * by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority(mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY);

	/* Enable the interrupt. */
	NVIC_EnableIRQ(mainSW_INTERRUPT_ID);
}


void vSoftwareInterruptHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	DEBUGOUT("Interrupción\r\n");
    /* Clear the software interrupt bit using the interrupt controllers
     * Clear Pending register. */
    mainCLEAR_INTERRUPT();

    /* Giving the semaphore may have unblocked a task - if it did and the
     * unblocked task has a priority equal to or above the currently executing
     * task then xHigherPriorityTaskWoken will have been set to pdTRUE and
     * portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
     * higher priority task.
     *
     * NOTE: The syntax for forcing a context switch within an ISR varies between
     * FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
     * the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
     * from an ISR! */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	main routine for FreeRTOS example 1 - Creating tasks
 * @return	Nothing, function should not exit
 */
int main(void)
{
	/* Sets up system hardware */
	prvSetupHardware();

	/* Print out the name of this example. */
	DEBUGOUT(pcTextForMain);

	vSemaphoreCreateBinary(xBinarySemaphoreA);
	vSemaphoreCreateBinary(xBinarySemaphoreB);
	vSemaphoreCreateBinary(xBinarySemaphoreC);
	//xQueue=xQueueCreate(1, sizeof(uint8_t));

	if ((xBinarySemaphoreA != (xSemaphoreHandle) NULL) && (xBinarySemaphoreB != (xSemaphoreHandle) NULL) && (xBinarySemaphoreC != (xSemaphoreHandle) NULL)){
		/* Habilita interrupción de software y establece su prioridad. */
		prvSetupSoftwareInterrupt();

		xTaskCreate(vTaskA, (char *) "TaskA", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 2UL), &xTaskAHandle);

		/* Creación de la tarea que va a generar la interrupción de software.*/
		xTaskCreate(vTaskB, (char *) "TaskB", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 1UL), &xTaskBHandle);

		/* Creación de la tarea sincronizada con vTask2 */
		xTaskCreate(vTaskC, (char *) "TaskC", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 1UL), &xTaskCHandle);

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

	/* If all is well we will never reach here as the scheduler will now be
	 * running.  If we do reach here then it is likely that there was insufficient
	 * heap available for the idle task to be created. */
	while (1);

	/* Should never arrive here */
	return ((int) NULL);
}


#endif
