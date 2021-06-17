/*
 * constants.h
 *
 *  Created on: Jun 13, 2021
 *      Author: alexander
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define TEMPERATURE_ERROR -300
#define VOLTAGE_ERROR -300

#define ADC_BUF_LEN 2048
#define AVG_SLOPE .0025   // V/C
#define V25  0.76   // V
#define VSENSE 3.3/4096  // 12 bit res

#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2C))
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2E))

#define CAN_BUF_LEN 8
#define UART_BUF_LEN 41

#endif /* CONSTANTS_H_ */
