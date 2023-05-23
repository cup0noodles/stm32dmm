/*
 * adc.c
 * Matthew Wong, Joshua Cheruvelil
 *
 */
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "adc.h"

void adc_init(void){

	/* ADC Setup */
	RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN);
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);

	//Set PA0 to analog input mode 11
	GPIOA->MODER |= (GPIO_MODER_MODE0);
	GPIOA->ASCR |= (GPIO_ASCR_ASC0); //connect gpio to analog in

	ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos); // run at cpu speed

	//Power Up ADC and wait for Vreg to come on
	ADC1->CR &= !(ADC_CR_DEEPPWD); // disable deep sleep
	ADC1->CR |= (ADC_CR_ADVREGEN);
	for(uint32_t i=2000000;i>0;i--); // wait 20us

	//Calibrate ADC
	//Ensure ADC Disabled and Single Ended mode
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR  & ADC_CR_ADCAL);	//wait for calibration to finish

	//configure for single ended mode on C5
	//set before en ADC
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);	//use ADC123_IN5 channel


	// enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);	//clear ready bit with a 1
	ADC1->CR |= ADC_CR_ADEN;		//enable adc
	while(!(ADC1->ISR & ADC_ISR_ADRDY)); //wait for ready flag == 1
	ADC1->ISR |= (ADC_ISR_ADRDY);	//clear ready bit with a 1

	//Configure ADC
	//set sequence 1 for 1 conversion on C5
	ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

	// 12-bit res, software trigger, right allign
	// single conv
	ADC1->CFGR = 0;

	//ADC 2.5clk hold - 000
	//47.5clk - 100
	//640.5clk - 111
	ADC1->SMPR1 &= ~(100 << ADC_SMPR1_SMP5_Pos);
	ADC1->SMPR1 |=  (100 << ADC_SMPR1_SMP5_Pos);


	//EN INTR for end of conv
	ADC1->IER |= (ADC_IER_EOCIE);
	NVIC->ISER[0] = (1<<(ADC1_2_IRQn & 0x1F));
	ADC1->ISR |= (ADC_ISR_EOC);			// clear flag with 1

	ADC1->CR |= ADC_CR_ADSTART;

}


uint16_t adcmax(uint16_t a, uint16_t b)
{
	if(a>b)
	{
		return a;
	}
	return b;
}

uint16_t adcmin(uint16_t a, uint16_t b)
{
	if(a<b)
	{
		return a;
	}
	return b;
}

uint16_t adc_cc(uint16_t sample)
{
	//adc correct and convert
	float adj,m,b;
	uint64_t x;
	x = ((uint64_t)sample*3300)>>12;
	if(x<ZONE_01){
		m = MZ0;
		b = BZ0;
		adj = (((float)x)*m)+b;
	}
	else if(x>= ZONE_01 && x < ZONE_12){
		m = MZ1;
		b = BZ1;
		adj = (((float)x)*m)+b;

	}
	else{
		m = MZ2;
		b = BZ2;
		adj = (((float)x)*m)+b;
	}
	return (uint16_t)adj;
}


