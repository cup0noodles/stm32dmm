/*
 * dac_functions.h
 *
 *  Created on: Apr 22, 2023
 *      Author: lucga, matthew wong
 */

#ifndef SRC_SUPPORT_H_
#define SRC_SUPPORT_H_
#define DACCLOCK RCC_AHB2ENR_GPIOAEN
#define DAC_port GPIOA

#define offset -9

#define MCP4922_DAC_SEL 1 << 15
#define MCP4922_BUFF_EN 1 << 14
#define MCP4922_GAIN_SEL 1 << 13
#define MCP4922_SHDN_EN 1 << 12
#define MCP4922_DATA_Mask 0xFFF



#include <main.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void DAC_init(void);
void DAC_write(uint16_t val);
uint16_t DAC_volt_conv(uint16_t voltage);

#endif /* SRC_SUPPORT_H_ */
