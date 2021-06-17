/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

volatile uint8_t uartErrorFlag = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern UART_HandleTypeDef huart5;

extern CAN_HandleTypeDef hcan1;

uint16_t adcRawBuffer[ADC_BUF_LEN] = {0};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for analogTask */
osThreadId_t analogTaskHandle;
const osThreadAttr_t analogTask_attributes = {
  .name = "analogTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartTxTask */
osThreadId_t uartTxTaskHandle;
const osThreadAttr_t uartTxTask_attributes = {
  .name = "uartTxTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartRxTask */
osThreadId_t uartRxTaskHandle;
const osThreadAttr_t uartRxTask_attributes = {
  .name = "uartRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for canTxTask */
osThreadId_t canTxTaskHandle;
const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for adcDataQueue */
osMessageQueueId_t adcDataQueueHandle;
const osMessageQueueAttr_t adcDataQueue_attributes = {
  .name = "adcDataQueue"
};
/* Definitions for canDataQueue */
osMessageQueueId_t canDataQueueHandle;
const osMessageQueueAttr_t canDataQueue_attributes = {
  .name = "canDataQueue"
};
/* Definitions for canRxDataQueue */
osMessageQueueId_t canRxDataQueueHandle;
const osMessageQueueAttr_t canRxDataQueue_attributes = {
  .name = "canRxDataQueue"
};
/* Definitions for adcPinConversionReady */
osSemaphoreId_t adcPinConversionReadyHandle;
const osSemaphoreAttr_t adcPinConversionReady_attributes = {
  .name = "adcPinConversionReady"
};
/* Definitions for adcTemperatureConversionReady */
osSemaphoreId_t adcTemperatureConversionReadyHandle;
const osSemaphoreAttr_t adcTemperatureConversionReady_attributes = {
  .name = "adcTemperatureConversionReady"
};
/* Definitions for uartTxReady */
osSemaphoreId_t uartTxReadyHandle;
const osSemaphoreAttr_t uartTxReady_attributes = {
  .name = "uartTxReady"
};
/* Definitions for uartRxReady */
osSemaphoreId_t uartRxReadyHandle;
const osSemaphoreAttr_t uartRxReady_attributes = {
  .name = "uartRxReady"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint32_t getAverage(uint16_t* buffer, uint16_t len) {
	uint32_t average = 0;
	for(uint16_t i = 0; i < len; ++i)
		average  += buffer[i];
	return (average / len);
}

int32_t calculateTemperature(const uint32_t* const rawData) {
	if (rawData == NULL)
		return TEMPERATURE_ERROR;

	// Calculate temperature with calibration coefficients
	int32_t temperature = (*rawData - (int32_t) *TEMP30_CAL_ADDR );
	temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
	temperature = temperature + 25;
	return temperature;
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAnalogTask(void *argument);
void StartUartTxTask(void *argument);
void StartUartRxTask(void *argument);
void StartCanTxTask(void *argument);
void StartCanRxTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcPinConversionReady */
  adcPinConversionReadyHandle = osSemaphoreNew(1, 1, &adcPinConversionReady_attributes);

  /* creation of adcTemperatureConversionReady */
  adcTemperatureConversionReadyHandle = osSemaphoreNew(1, 1, &adcTemperatureConversionReady_attributes);

  /* creation of uartTxReady */
  uartTxReadyHandle = osSemaphoreNew(1, 1, &uartTxReady_attributes);

  /* creation of uartRxReady */
  uartRxReadyHandle = osSemaphoreNew(1, 1, &uartRxReady_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of adcDataQueue */
  adcDataQueueHandle = osMessageQueueNew (16, sizeof(struct adc_data_t),
		  &adcDataQueue_attributes);

  /* creation of canDataQueue */
  canDataQueueHandle = osMessageQueueNew (16, UART_BUF_LEN,
		  &canDataQueue_attributes);

  /* creation of canRxDataQueue */
  canRxDataQueueHandle = osMessageQueueNew (20, CAN_BUF_LEN,
		  &canRxDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of analogTask */
  analogTaskHandle = osThreadNew(StartAnalogTask, NULL, &analogTask_attributes);

  /* creation of uartTxTask */
  uartTxTaskHandle = osThreadNew(StartUartTxTask, NULL, &uartTxTask_attributes);

  /* creation of uartRxTask */
  uartRxTaskHandle = osThreadNew(StartUartRxTask, NULL, &uartRxTask_attributes);

  /* creation of canTxTask */
  canTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &canTxTask_attributes);

  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(StartCanRxTask, NULL, &canRxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAnalogTask */
/**
* @brief Function implementing the analogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAnalogTask */
void StartAnalogTask(void *argument)
{
  /* USER CODE BEGIN StartAnalogTask */
	/* Infinite loop */
	osStatus_t osStatus = osError;
	HAL_StatusTypeDef halStatus = HAL_ERROR;

	uint32_t averageValue = 0;
	struct adc_data_t adcValues;

	// Reset semaphores
	osSemaphoreAcquire(adcTemperatureConversionReadyHandle, osWaitForever);
	osSemaphoreAcquire(adcPinConversionReadyHandle, osWaitForever);

	for(;;) {

		// Run temperature conversion
		halStatus = HAL_ADC_Start_DMA(&hadc1,
				(uint32_t*)adcRawBuffer, ADC_BUF_LEN);

		// Wait for temperature conversion ready
		osStatus = osSemaphoreAcquire(adcTemperatureConversionReadyHandle,
				osWaitForever);

		if (halStatus == HAL_OK && osStatus == osOK) {
			averageValue = getAverage(adcRawBuffer, ADC_BUF_LEN);
			adcValues.temperature = calculateTemperature(&averageValue);
		}
		else {
			adcValues.temperature = TEMPERATURE_ERROR;
		}

		// Run pin conversion
		halStatus = HAL_ADC_Start_DMA(&hadc2,
				(uint32_t*)adcRawBuffer, ADC_BUF_LEN);

		// Wait for pin conversion ready
		osStatus = osSemaphoreAcquire(adcPinConversionReadyHandle,
				osWaitForever);

		if (halStatus == HAL_OK && osStatus == osOK) {
			averageValue = getAverage(adcRawBuffer, ADC_BUF_LEN);

			// Convert ADC value to voltage;
			adcValues.voltage = (float)averageValue * VSENSE;
		}
		else {
			adcValues.voltage = VOLTAGE_ERROR;
		}

		// Transmit ADC data
		osMessageQueuePut(adcDataQueueHandle, &adcValues, 0, osWaitForever);
		osDelay(25);
	}

	// Never get here
	osThreadTerminate(analogTaskHandle);
  /* USER CODE END StartAnalogTask */
}

/* USER CODE BEGIN Header_StartUartTxTask */
/**
* @brief Function implementing the uartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTxTask */
void StartUartTxTask(void *argument)
{
  /* USER CODE BEGIN StartUartTxTask */
  /* Infinite loop */
	osStatus_t osStatus = osError;
	struct adc_data_t adcValues;
	char uartTxMessage[UART_BUF_LEN] = {""};
	int length = 0;

	// Reset semaphore
	osSemaphoreAcquire(uartTxReadyHandle, osWaitForever);
	for(;;) {
		// Wait until the queue is empty
		osStatus = osMessageQueueGet(adcDataQueueHandle, &adcValues,
				NULL, osWaitForever);

		if (osStatus != osOK)
			continue;

		// Set ADC data in the buffer
		length = snprintf(uartTxMessage, UART_BUF_LEN,
				"Temperature: %d C | Voltage %.2f V\n",
				(int)adcValues.temperature, adcValues.voltage);

		// If copy failed
		if (length <= 0) {
			memset(uartTxMessage, 0, UART_BUF_LEN);
			continue;
		}

		// Transmit buffer via UART
		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)uartTxMessage, UART_BUF_LEN);

		// Wait for the end of transaction
		osSemaphoreAcquire(uartTxReadyHandle, osWaitForever);

		// Clear buffer
		memset(uartTxMessage, 0, UART_BUF_LEN);
	}
  /* USER CODE END StartUartTxTask */
}

/* USER CODE BEGIN Header_StartUartRxTask */
/**
* @brief Function implementing the uartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartRxTask */
void StartUartRxTask(void *argument)
{
  /* USER CODE BEGIN StartUartRxTask */
  /* Infinite loop */
	osStatus_t osStatus = osError;
	HAL_StatusTypeDef halStatus = HAL_ERROR;
	char uartRxMessage[UART_BUF_LEN] = {""};

	// Reset semaphore
	osSemaphoreAcquire(uartRxReadyHandle, osWaitForever);
	for(;;) {
		// Receive UART message
		halStatus = HAL_UART_Receive_DMA(&huart5,
				(uint8_t*)uartRxMessage, UART_BUF_LEN);

		// Wait until receiving is done
		osStatus = osSemaphoreAcquire(uartRxReadyHandle, osWaitForever);

		if (halStatus == HAL_OK && osStatus == osOK) {

			// clear corrupted data
			if (uartErrorFlag) {
				uartErrorFlag = 0;
				memset(uartRxMessage, 0, UART_BUF_LEN);
				continue;
			}

			// Send UART message CAN_TX task, JUST FOR DEMO ONLY!!!
			osMessageQueuePut(canDataQueueHandle,
					uartRxMessage, 0, osWaitForever);
		}
	}
  /* USER CODE END StartUartRxTask */
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief Function implementing the canTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
void StartCanTxTask(void *argument)
{
  /* USER CODE BEGIN StartCanTxTask */
  /* Infinite loop */

	osStatus_t osStatus = osError;
	HAL_StatusTypeDef halStatus = HAL_ERROR;

	char messageBuffer[UART_BUF_LEN] = {""};
	char canChunk[CAN_BUF_LEN] = {""};
	uint32_t canTxMailbox = 0;

	CAN_TxHeaderTypeDef canTxHeader;
	canTxHeader.StdId = 0x111;
	canTxHeader.ExtId = 0;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.TransmitGlobalTime = DISABLE;

	// Start the CAN module
	HAL_CAN_Start(&hcan1);

	// enable RX data; case ONLY FOR DEMO MODE !!!
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	for(;;) {
		// Receive data chunk
		osStatus = osMessageQueueGet(canDataQueueHandle, messageBuffer,
				NULL, osWaitForever);

		if (osStatus != osOK)
			continue;

		uint8_t i = 0;
		uint8_t run = 1;
		uint8_t shift = 0;
		uint8_t copySize = 0;

		// Cut UART message on CAN chunks
		while(run) {
			shift = i++ * CAN_BUF_LEN;
			if (shift + CAN_BUF_LEN >= UART_BUF_LEN) {
				copySize = UART_BUF_LEN - shift;
				run = 0;

				// Clear junk before partial filling
				memset(canChunk, 0, CAN_BUF_LEN);
			}
			else {
				copySize = CAN_BUF_LEN;
			}
			memcpy(canChunk, messageBuffer + shift, copySize);
			canTxHeader.DLC = copySize;

			// Wait until at least one CAN mailbox is free
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);

			halStatus = HAL_CAN_AddTxMessage(&hcan1, &canTxHeader,
						(uint8_t*)canChunk, &canTxMailbox);

			if (halStatus != HAL_OK) {
				run = 0;
			}
		}
		memset(canChunk, 0, CAN_BUF_LEN);
	}
  /* USER CODE END StartCanTxTask */
}

/* USER CODE BEGIN Header_StartCanRxTask */
/**
* @brief Function implementing the canRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void *argument)
{
  /* USER CODE BEGIN StartCanRxTask */
  /* Infinite loop */
	osStatus_t osStatus = osError;
	uint8_t canData[CAN_BUF_LEN] = {""};

	for(;;) {
		osStatus = osMessageQueueGet(canRxDataQueueHandle, canData,
				NULL, osWaitForever);
		// Do something with data
	}
  /* USER CODE END StartCanRxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
