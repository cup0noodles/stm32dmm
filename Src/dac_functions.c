/*
 * Keypad.c
 *
 *  Created on: Apr 30, 2023
 *      Authors: Matthew Wong, Luc Garcia
 */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "dac_functions.h"

void DAC_init(void){
//initialize the SPI peripheral to communicate with the DAC
//Turn on CLK for DAC

	  RCC -> AHB2ENR |= DACCLOCK;

	  //Configure columns

	  //GPIO Alternate Function (10)
	  DAC_port->MODER &= ~(GPIO_MODER_MODE4 |
						GPIO_MODER_MODE5 |
						GPIO_MODER_MODE7); //clear MODE

	  DAC_port->MODER |= (2 << GPIO_MODER_MODE4_Pos |
					   2 << GPIO_MODER_MODE5_Pos |
					   2 << GPIO_MODER_MODE7_Pos); //set MODE = 10 (alternate function)

	  DAC_port->AFR[0] &= ~(GPIO_AFRL_AFSEL4 |
							GPIO_AFRL_AFSEL5 |
							GPIO_AFRL_AFSEL7); //clear AFR

	  DAC_port->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos |
							   5 << GPIO_AFRL_AFSEL5_Pos |
							   5 << GPIO_AFRL_AFSEL7_Pos); //set AF5

	  DAC_port->OSPEEDR |= ((GPIO_OSPEEDR_OSPEED4) |
						  (GPIO_OSPEEDR_OSPEED5) |
						  (GPIO_OSPEEDR_OSPEED7));  //Setting to very high speed (11)


	  //SPI Configuration

	  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	  //Clock Phase
	  SPI1->CR1 &= ~(SPI_CR1_CPHA); // (0)

	  //Clock Polarity
	  SPI1->CR1 &= ~(SPI_CR1_CPOL); // (0)

	  //Master Selection
	  SPI1->CR1 |= SPI_CR1_MSTR; // (1)

	  //Baud Rate Control
	  SPI1->CR1 &= ~(001 <<SPI_CR1_BR_Pos); // div 4
	  SPI1->CR1 |= (001 <<SPI_CR1_BR_Pos); // (000)

	  //Frame Format
	  SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); // (0) - MSB First

	  //Set Data Size to 16-bit
	  SPI1->CR2 |= SPI_CR2_DS; // (1111)

	  //NSS pulse management enabled
	  SPI1->CR2 |= SPI_CR2_NSSP; // (1)

	  //unsure if following are okay for full duplex, transmit only in simplex
	  //Transmit Only Mode
	  SPI1->CR1 &= ~(SPI_CR1_RXONLY); // (0)

//	  //Bidirectional Data Mode Enable
//	  SPI1->CR1 &= ~(SPI_CR1_BIDIMODE); // (0)

	  //SPI Enable
	  SPI1->CR1 |= SPI_CR1_SPE; // (1)

}

void DAC_write(uint16_t val){
	//SPI EN (in case in power save mode
	//SPI1->CR1 |= SPI_CR1_SPE;
	// clear upper 4 bits of val

	val &= MCP4922_DATA_Mask;

	// set upper config bits
	// DAC A, No Buff, 1x Gain, SHDN disabled
	val &= ~((3 << 12));
	val |= ((3 << 12));
	//write to DR, all 16 bit no mask
	while ((SPI1->SR & (1 << SPI_SR_TXE_Pos)) == 0){}

	SPI1->DR = val;

	// Wait until buffers cleared, then shut down SPI for power
//	uint8_t spi_bsy = 1;
//	while (spi_bsy == 1);
//		spi_bsy = ((SPI1->SR) & SPI_SR_BSY_Msk) >> SPI_SR_BSY_Pos;
//	SPI1->CR1 &= ~SPI_CR1_SPE;
}

uint16_t DAC_volt_conv(uint16_t voltage){
	if (voltage > 3300){
		return 4095;
	}
	else if(voltage == 0){
		return 0;
	}
	//adc correct and convert
	float adj,m,b;
	uint64_t x;
	x = ((uint64_t)voltage<<12)/3300;
	m = DAC_MZ0;
	b = DAC_BZ0;
	adj = (((float)x)*m)+b;
	return (uint16_t)adj;
}
