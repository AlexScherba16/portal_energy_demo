/*
 * callbacks.c
 *
 *  Created on: Jun 13, 2021
 *      Author: alexander
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "constants.h"

/* ADC handlers */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* ADC semaphores */
extern osSemaphoreId_t adcPinConversionReadyHandle;
extern osSemaphoreId_t adcTemperatureConversionReadyHandle;

/* ADC periphery callbacks */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == NULL)
		goto fail;

	if (hadc == &hadc1)
		osSemaphoreRelease(adcTemperatureConversionReadyHandle);

	if (hadc == &hadc2)
		osSemaphoreRelease(adcPinConversionReadyHandle);
	return;

fail:
// process failed handler
	osSemaphoreRelease(adcTemperatureConversionReadyHandle);
	osSemaphoreRelease(adcPinConversionReadyHandle);
	return;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	osSemaphoreRelease(adcTemperatureConversionReadyHandle);
	osSemaphoreRelease(adcPinConversionReadyHandle);
	return;
}

////////////////////////////////////////////////////////////////////////////////

/* UART handlers */
extern UART_HandleTypeDef huart5;

/* UART semaphores */
extern osSemaphoreId_t uartTxReadyHandle;
extern osSemaphoreId_t uartRxReadyHandle;

/* UART error flag */
extern volatile uint8_t uartErrorFlag;

/* UART periphery callbacks */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	osSemaphoreRelease(uartTxReadyHandle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	osSemaphoreRelease(uartRxReadyHandle);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart == NULL) {
		osSemaphoreRelease(uartRxReadyHandle);
		return;
	}

	if(huart == &huart5) {
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UART_DMAStop(&huart5);
		uint32_t er = HAL_UART_GetError(&huart5);

		switch(er)
		{
			// Parity error
			case HAL_UART_ERROR_PE:
				__HAL_UART_CLEAR_PEFLAG(&huart5);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			// Noise error
			case HAL_UART_ERROR_NE:
				__HAL_UART_CLEAR_NEFLAG(&huart5);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			// Frame error
			case HAL_UART_ERROR_FE:
				__HAL_UART_CLEAR_FEFLAG(&huart5);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			// Overrun error
			case HAL_UART_ERROR_ORE:
				__HAL_UART_CLEAR_OREFLAG(huart);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			// DMA transfer error
			case HAL_UART_ERROR_DMA:
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			// Undetected error
			default:
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		}
	}
	uartErrorFlag = 1;
	osSemaphoreRelease(uartTxReadyHandle);
	osSemaphoreRelease(uartRxReadyHandle);
}

////////////////////////////////////////////////////////////////////////////////

/* CAN queue*/
extern osMessageQueueId_t canRxDataQueueHandle;

/* CAN periphery callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef canRxHeader;
	char canData[CAN_BUF_LEN] = {""};

	// Receive CAN message, if OK - put it in the queue
	HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
			&canRxHeader, (uint8_t*)canData);

	if(status != HAL_OK)
		return;

	osMessageQueuePut(canRxDataQueueHandle, (const void*)canData, 0, 0);
}
