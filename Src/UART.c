/*
 * Simple UART
 * UART.c
 * Matthew Wong, Joshua Cheruvelil
 * May 5, 2023
 */

#include "main.h"
#include <UART.h>
#define USART_DIV 208


void UART_init(void)
{
	/* GPIOA Config */
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN); 	   // enable clock for GPIOA

	/* MODE CONFIG PC */
	GPIOA->MODER   &= ~(GPIO_MODER_MODE2
				   	   |GPIO_MODER_MODE3); 	   // clears MODE

	GPIOA->MODER   |=  (GPIO_MODER_MODE2_1
					   |GPIO_MODER_MODE3_1);   // set alt func

	GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL2
					   |GPIO_AFRL_AFSEL3);	   //clear alt func

	GPIOA->AFR[0]  |=  ((7 << GPIO_AFRL_AFSEL2_Pos)
			 	 	   |(7 << GPIO_AFRL_AFSEL3_Pos));
											   // set AF7 (USART)

	/* USART Config */
	RCC->APB1ENR1 |= (RCC_APB1ENR1_USART2EN);

	USART2->CR1 &= ~(USART_CR1_OVER8);			// oversample 16

	USART2->CR1 &= ~(USART_CR1_PCE);			// disable parity

	USART2->CR2 &= ~(USART_CR2_MSBFIRST);		// LSB First

	USART2->CR1 &= ~(USART_CR1_M1);				// 8-bit send length
	USART2->CR1 &= ~(USART_CR1_M0);				// 8-bit send length

	USART2->BRR &= ~(USART_DIV);
	USART2->BRR |=  (USART_DIV);				// set clk div

	USART2->CR2 &= ~(USART_CR2_STOP);			// 1 stop bit (00)

	USART2->CR1 &= ~(USART_CR1_RXNEIE);
	USART2->CR1 |=  (USART_CR1_RXNEIE);

	USART2->CR1 |= (USART_CR1_UE);				// USART EN, this comes last

	USART2->CR1 |= (USART_CR1_RE);				// RX En

	// enable interrupts in NVIC
	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));

	__enable_irq();							// enable interrupts globally
}

void UART_print(char* text){
	char *t;
	for (t = text; *t != '\0'; t++) {
		UART_send_byte(*t);
	}

}

void UART_send_byte(char byte){
	USART2->CR1 |= (USART_CR1_TE);				// TX En
	while(((USART2->ISR) & (USART_ISR_TXE)) == 0){}
	USART2->TDR = byte;
}

void UART_ESC_code(char* text){
	UART_send_byte(0x1b);
	UART_print(text);

}


