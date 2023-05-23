/*
 * Simple UART
 * UART.h
 * Matthew Wong, Joshua Cheruvelil
 * May 5, 2023
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include <main.h>
#include "stm32l4xx_hal.h"
void UART_init(void);
void UART_print(char*);
void UART_send_byte(char);
void UART_ESC_code(char*);


#endif /* INC_UART_H_ */
