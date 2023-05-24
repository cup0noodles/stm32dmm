/*
 * Project 2 DMM
 * Matthew Wong
 */
#include "main.h"
#include <math.h>
#include <adc.h>
#include <UART.h>
#include <dac_functions.h>

/* Define Sampling System */
#define CLK_SPEED 32100000
#define DC_SAMPLE_INTERVAL 16000 //100 sample in 50ms
#define DC_SAMPLE_COUNT 200
#define AC_SAMPLE_COUNT 250


void TIM_init(void);
void COMP_init(void);
void print_int(int data);
void print_dec(int data);
void print_DC_screen(void);
void print_AC_screen(void);
void update_graph(char* x, char* y, uint16_t value);
void SystemClock_Config(void);

typedef enum state_var{
	SAMPLE_INIT,
	SAMPLE_BUSY,
	DC_CALC,
	FREQ_INIT,
	FREQ_TRIG,
	FREQ_CALC,
	AC_INIT,
	AC_CALC,
	AC_DISPLAY,
	DC_DISPLAY,
	MODE_CHECK
};

enum state_var state = SAMPLE_INIT;
uint8_t value_flag,cap_flag;
uint16_t adc_value;
uint32_t captured_count;
uint8_t mode = 0; ///flag
uint8_t timeout = 0;

int main(void)
{
 	HAL_Init();
	SystemClock_Config();
	/* CLK Config */
	RCC->CR &= ~(RCC_CR_MSIRANGE);  //clear MSI range
	RCC->CR |= (RCC_CR_MSIRANGE_10); //config MSI clk to ~32MHz

	DAC_init();
	COMP_init();
	UART_init();
	adc_init();
	TIM_init();

	print_DC_screen();

	/*Init Variables*/

	//For ADC Sampling
	uint16_t sample_count;			   //inc to keep track of how many samples taken
	uint16_t sample_N; 				   //num of samples to take
	uint16_t vsample[1000]; //array hold samples, not fully used for AC
	uint32_t ac_sample_interval;	   //period to sample values for AC measurements - variable
	uint8_t current_mode = 0;		   // 0DC 1 AC, updated once per sample cycle

	//DC sample analysis
	uint16_t vdc, vmax, vmin, vrms, vpp;		   //analysis results
	uint64_t vsum2, vsum, vrmssum;

	uint8_t ac_measure = 0; //0 when measuring DC components, 1 when measuring AC

	//Freq Measurements
	uint32_t sampletimes[2];
	uint32_t period, period_cor;
	uint32_t freq;
	uint32_t timeout_cnt;

	while (1)
	{
		switch(state)
		{
			case SAMPLE_INIT:
				//set sample param based on AC or DC
				if(ac_measure)
				{
					// setup for AC
					sample_N = AC_SAMPLE_COUNT;
					TIM2->CCR2 = TIM2->CNT + ac_sample_interval;   //set CCR1 16k ahead of count
				}
				else
				{
					//setup for DC
					sample_N = DC_SAMPLE_COUNT;
					TIM2->CCR2 = TIM2->CNT + DC_SAMPLE_INTERVAL;   //set CCR1 16k ahead of count
				}
				//set up CCR2 to trigger AC measurements
				TIM2->SR &= ~(TIM_SR_CC2IF);
				TIM2->CCER |=  (TIM_CCER_CC2E);   //enable capture from the counter
				TIM2->DIER |= (TIM_DIER_CC2IE);
				state = SAMPLE_BUSY;
				break;
			case SAMPLE_BUSY:
				while(sample_count < sample_N)
				{
					if(mode)
					{
						//end collection - stop tim2
						TIM2->CCER &= ~(TIM_CCER_CC2E);
						TIM2->DIER &= ~(TIM_DIER_UIE);
						TIM2->SR &= ~(TIM_SR_CC2IF);
						TIM2->DIER &= ~(TIM_DIER_CC2IE);
						break;
					}
					if(value_flag)
					{
						//UART_print("x");
						//save ADC read
						vsample[sample_count] = adc_value;
						value_flag = 0;

						if(sample_count==sample_N-1)
						{
							//end collection - stop tim2
							TIM2->CCER &= ~(TIM_CCER_CC2E);
							TIM2->DIER &= ~(TIM_DIER_UIE);
							TIM2->SR &= ~(TIM_SR_CC2IF);
							TIM2->DIER &= ~(TIM_DIER_CC2IE);
						}
						sample_count++;
					}
				}
				sample_count = 0;
				// switch depending on AC or DC sampling
				if(ac_measure)
				{
					//data taken for AC
					state = AC_CALC;
					ac_measure = 0;
				}
				else if(mode) //if switching modes
				{
					state = MODE_CHECK;
				}
				else
				{
					state = DC_CALC;
				}
				break;
			case DC_CALC:
				vsum2 = 0;
				//todo this only works for 2k samples
				for(uint8_t j=0;j < 20; j++)
				{
					vsum = 0;
					for(uint8_t k=0;k < 10; k++)
					{
						vsum += vsample[(j*10)+k];
					}
					vsum2 += (vsum / 10	);
				}

				vdc = adc_cc(vsum2/20);
				//set comp neg value
				if(current_mode)
				{
					//AC mode, move to freq measyre
					DAC_write(DAC_volt_conv(vdc));
					state = FREQ_INIT;
				}
				else
				{
					DAC_write(DAC_volt_conv(vdc));
					//dc mode, display value
					state = DC_DISPLAY;
				}
				break;
			case FREQ_INIT:
				TIM2->SR &= ~(TIM_SR_CC2IF);
				TIM2->SR &= ~(TIM_SR_CC4IF);
				TIM2->DIER |= (TIM_DIER_UIE);
				TIM2->DIER |= (TIM_DIER_CC4IE);
				TIM2->DIER &= ~(TIM_DIER_CC2IE);
				TIM2->CCER &=  ~(TIM_CCER_CC2E); //disable timeout counter
				TIM2->CCER |= (TIM_CCER_CC4E);
				state = FREQ_TRIG;

				break;
			case FREQ_TRIG:
//				timeout_cnt = 0;
				sample_count = 0;
				while(sample_count < 2)
				{
					if(cap_flag)
					{
						sampletimes[sample_count] = captured_count;
						cap_flag = 0;
						sample_count++;
					}
				}
				TIM2->DIER &= ~(TIM_DIER_UIE);
				TIM2->CCER &=  ~(TIM_CCER_CC4E); //disable capture from counter
				TIM2->DIER &= ~(TIM_DIER_CC4IE);
				state = FREQ_CALC;
				break;
			case FREQ_CALC:

				if(sampletimes[0] > sampletimes[1])
					{
						//rollover occurred
						period = (0xFFFFFFFF - sampletimes[0]) + sampletimes[1];
					}
					else
					{
						//normal no rollover
						period = sampletimes[1] - sampletimes[0];
					}
				freq = CLK_SPEED/period;
				freq = adcmax(freq, 1);
				freq = adcmin(freq, 1000);


				state = AC_INIT;
				break;
			case AC_INIT:
				//take 200 measurements with interval of period
				//TODO debug value
				//period = 64000;
				period_cor = CLK_SPEED/freq;
				ac_measure = 1;
				ac_sample_interval = (period_cor*2)/AC_SAMPLE_COUNT;
				state = SAMPLE_INIT;
				break;
			case AC_CALC:
				TIM2->CCER &= ~TIM_CCER_CC2E;
				TIM2->DIER &= ~TIM_DIER_CC2IE;
				//calculate RMS
				vsum = 0;
				vmax = 0;
				vmin = 3300;
				vrmssum = 0;
				for(uint8_t k=0;k < AC_SAMPLE_COUNT; k++)
				{
					vmax = adcmax(vmax, vsample[k]);
					vmin = adcmin(vmin, vsample[k]);
					vrmssum += pow(adc_cc(vsample[k]), 2);
					vsum += adc_cc(vsample[k]);
				}
				vdc = (vsum / AC_SAMPLE_COUNT);
				vrmssum = vrmssum / AC_SAMPLE_COUNT;
				vrms = sqrt(vrmssum);
				vpp = adc_cc(vmax-vmin);
				state = AC_DISPLAY;
				break;
			case AC_DISPLAY:
				//todo
				//update values
				//freq
				UART_ESC_code("[H");
				UART_ESC_code("[2B");
				UART_ESC_code("[23C");
				print_int(freq);
				//dc voltage
				UART_ESC_code("[H");
				UART_ESC_code("[4B");
				UART_ESC_code("[22C");
				print_dec(vdc);
				//rms
				UART_ESC_code("[H");
				UART_ESC_code("[8B");
				UART_ESC_code("[22C");
				print_dec(vrms);
				//vpp
				UART_ESC_code("[H");
				UART_ESC_code("[12B");
				UART_ESC_code("[22C");
				print_dec(vpp);

				//update graphs
				update_graph("[4B", "[33C", vdc);
				update_graph("[8B", "[33C", vrms);
				update_graph("[12B", "[33C", vpp);
				UART_ESC_code("[H");
				state = MODE_CHECK;
				break;
			case DC_DISPLAY:
				//TODO
				//dc voltage
				UART_ESC_code("[H");
				UART_ESC_code("[2B");
				UART_ESC_code("[18C");
				print_dec(vdc);
				//dc voltage
				update_graph("[2B", "[29C", vdc);
				UART_ESC_code("[H");
				state = MODE_CHECK;
				break;
			case MODE_CHECK:
				if(mode){
					mode = 0;
					//mode switch
					if(current_mode)
					{
						current_mode = 0;
						print_DC_screen();
					}
					else
					{
						current_mode = 1;
				        print_AC_screen();
					}
				}
				state = SAMPLE_INIT;
				break;
			default:
				state = SAMPLE_INIT;
				break;
		}
	}
}

void USART2_IRQHandler(void){
	char byte = USART2->RDR;
	//don't really care what it is, just toggle
	mode = 1;
	// Timer 2 ISR
}
void TIM2_IRQHandler(void)
{
	if(state == SAMPLE_BUSY){
		//CC2 - VDC sampling
		TIM2->SR &= ~(TIM_SR_CC2IF);
		TIM2->CCR2 += DC_SAMPLE_INTERVAL; //2kHz ADC en
		ADC1->CR |= ADC_CR_ADSTART;
	}
	else if(state == FREQ_TRIG)
	{
		TIM2->SR &= ~(TIM_SR_CC4IF); //clear flag
		captured_count = TIM2->CCR4;
		cap_flag = 1;
	}
}


void ADC1_2_IRQHandler(void)
{
	//read value
	adc_value = ADC1->DR;
	value_flag = 1;
}

void TIM_init(void)
{

	/* TIM2 Config */
	// Configure TIM2 to interrupt every 0.5s
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);

	TIM2->SR   &= ~(TIM_SR_UIF);			// clear update interrupt flag
	TIM2->ARR   = 0xFFFFFFFF;				// 5 KHz clock = 1/200E-6s, 800 * 2.5E-7s = 200E-6s
	TIM2->CR1  |= (TIM_CR1_CEN);			// start timer

	//CCR4 for edge det/measure
	// 01 = COMP1 output to INP4
	TIM2->OR1 &= ~(TIM2_OR1_TI4_RMP_0);
	TIM2->OR1 |= (TIM2_OR1_TI4_RMP_0);

	//map TI4 to CC4S input
	TIM2->CCMR2 &= ~(1 << TIM_CCMR2_CC4S_Pos);
	TIM2->CCMR2 |= (1 << TIM_CCMR2_CC4S_Pos);

	TIM2->CCMR2 &= ~(3 << TIM_CCMR2_IC4F_Pos);     //validating transition after fclk, N=8 samples
	TIM2->CCMR2 |= (3 << TIM_CCMR2_IC4F_Pos);     //validating transition after fclk, N=8 samples

	TIM2->CCMR2 &= ~(TIM_CCMR2_IC4PSC); //disable prescale

	TIM2->CCER  &= ~(TIM_CCER_CC4P);         //rising edge which is 0
	TIM2->CCER  &= ~(TIM_CCER_CC4NP);         //rising edge which is 0

	//TIM2->CCER  |=  (TIM_CCER_CC4E);                //enable capture from the counter
	//TIM2->CCER  |=  (TIM_CCER_CC2E);                //enable capture from the counter


	//ccr2 CCR and IE are en/den in main

	// enable interrupts in NVIC
	TIM2->DIER |= (TIM_DIER_UIE);
	NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));

	__enable_irq();							// enable interrupts globally
	return;
}

void COMP_init(void)
{
	//Comp Clock EN
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC -> AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC -> AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	//PB2, PC4 analog mode 11, no other config needed
	GPIOB->MODER |= (GPIO_MODER_MODE2);
	GPIOC->MODER |= (GPIO_MODER_MODE4);

	//Med hyster 0-3
	COMP1->CSR &= ~(3 << COMP_CSR_HYST_Pos);
	COMP1->CSR |= (3 << COMP_CSR_HYST_Pos);

	//Pos in PB2
	COMP1->CSR &= ~(COMP_CSR_INPSEL);
	COMP1->CSR |= (COMP_CSR_INPSEL);

	//Neg in PC4
	COMP1->CSR &= ~(7 << COMP_CSR_INMSEL_Pos);
	COMP1->CSR |= (7 << COMP_CSR_INMSEL_Pos);

	//high speed mode
	COMP1->CSR &= ~(COMP_CSR_PWRMODE);

	//EN
	COMP1->CSR |= (COMP_CSR_EN);

	//PB10 COMP Out debug
	GPIOB->MODER &= ~(GPIO_MODER_MODE10);
	GPIOB->MODER |=  (2 << GPIO_MODER_MODE10_Pos);   // set alt func

	GPIOB->AFR[1]  &= ~(GPIO_AFRH_AFSEL10);	   //clear alt func

	GPIOB->AFR[1]  |=  (12 << GPIO_AFRH_AFSEL10_Pos);

}

void print_int(int data)
{
	uint8_t NUMCHAR_OFFSET = 48;
	uint8_t d3 = data/1000;
	uint8_t d2 = data/100 - d3*10;
	uint8_t d1 = data/10 - d3*100 - d2*10;
	uint8_t d0 = data - d3*1000 - d2*100 - d1*10;
	if(data == 0)
	{
		UART_print("   0");
	}
	else if(data < 10)
	{
		UART_print("   ");
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
	else if(data < 100)
	{
		UART_print("  ");
		UART_send_byte(d1+NUMCHAR_OFFSET);
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
	else if(data < 1000)
	{
		UART_print(" ");
		UART_send_byte(d2+NUMCHAR_OFFSET);
		UART_send_byte(d1+NUMCHAR_OFFSET);
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
	else
	{
		UART_send_byte(d3+NUMCHAR_OFFSET);
		UART_send_byte(d2+NUMCHAR_OFFSET);
		UART_send_byte(d1+NUMCHAR_OFFSET);
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
}

void print_dec(int data)
{
	uint8_t NUMCHAR_OFFSET = 48;
	uint8_t d3 = data/1000;
	uint8_t d2 = data/100 - d3*10;
	uint8_t d1 = data/10 - d3*100 - d2*10;
	uint8_t d0 = data - d3*1000 - d2*100 - d1*10;
	if(data == 0)
	{
		UART_print("0.000");
	}
	else if(data < 10)
	{
		UART_print("0.00");
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
	else if(data < 100)
	{
		UART_print("0.0");
		UART_send_byte(d1+NUMCHAR_OFFSET);
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
	else if(data < 1000)
	{
		UART_print("0.");
		UART_send_byte(d2+NUMCHAR_OFFSET);
		UART_send_byte(d1+NUMCHAR_OFFSET);
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
	else
	{
		UART_send_byte(d3+NUMCHAR_OFFSET);
		UART_print(".");
		UART_send_byte(d2+NUMCHAR_OFFSET);
		UART_send_byte(d1+NUMCHAR_OFFSET);
		UART_send_byte(d0+NUMCHAR_OFFSET);
	}
}

void print_DC_screen(void)
{
	UART_ESC_code("[2J");//clear screen
	UART_ESC_code("[24C");
	UART_ESC_code("[1m"); //bold mode
	UART_print("PolyDMM - DC Mode");
	UART_ESC_code("[H");
	UART_ESC_code("[2B"); //next line down
	// setup each graph
	UART_print("Current Voltage:");
	UART_ESC_code("[0m");
	UART_print("  x.xxx V   |");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[1D");
	UART_print("|-----|-----|-----|-----|-----|-----|");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[37D"); //move across
	UART_print("0    0.5   1.0   1.5   2.0   2.5   3.0");
	UART_ESC_code("[H"); //return home
}

void print_AC_screen(void)
{
	UART_ESC_code("[2J");//clear screen
	UART_ESC_code("[26C");
	UART_ESC_code("[1m"); //bold mode
	UART_print("PolyDMM - AC Mode");
	UART_ESC_code("[H");
	UART_ESC_code("[2B"); //next line down
	// setup each graph
	UART_print("Current Frequency:");
	UART_ESC_code("[0m");
	UART_print("        x Hz");
	UART_ESC_code("[H");

	//first graph
	UART_ESC_code("[4B"); // move to 0,4
	UART_ESC_code("[1m");
	UART_print("Average Voltage:");
	UART_ESC_code("[0m");
	UART_print("      x.xxx V   |");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[1D");
	UART_print("|-----|-----|-----|-----|-----|-----|");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[37D"); //move across
	UART_ESC_code("[2m");
	UART_print("0    0.5   1.0   1.5   2.0   2.5   3.0");
	UART_ESC_code("[0m");
	UART_ESC_code("[H"); //return home

	//second graph
	UART_ESC_code("[8B"); // move to 0,8
	UART_ESC_code("[1m");
	UART_print("RMS Voltage:");
	UART_ESC_code("[0m");
	UART_print("          x.xxx V   |");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[1D");
	UART_print("|-----|-----|-----|-----|-----|-----|");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[37D"); //move across
	UART_ESC_code("[2m");
	UART_print("0    0.5   1.0   1.5   2.0   2.5   3.0");
	UART_ESC_code("[0m");
	UART_ESC_code("[H"); //return home

	//second graph
	UART_ESC_code("[12B"); // move to 0,12
	UART_ESC_code("[1m");
	UART_print("Peak-to-Peak Voltage:");
	UART_ESC_code("[0m");
	UART_print(" x.xxx V	|");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[1D");
	UART_print("|-----|-----|-----|-----|-----|-----|");
	UART_ESC_code("[1B"); //move down
	UART_ESC_code("[37D"); //move across
	UART_ESC_code("[2m");
	UART_print("0    0.5   1.0   1.5   2.0   2.5   3.0");
	UART_ESC_code("[0m");
	UART_ESC_code("[H"); //return home

}

void update_graph(char* x, char* y, uint16_t value)
{
	//x,y is position of start of bar graph
	UART_ESC_code("[H"); //dont assume home
	UART_ESC_code("[0m"); //clear any formatting
	UART_ESC_code(x);
	UART_ESC_code(y);
	UART_ESC_code("[0K"); //clear existing graph
	//UART_ESC_code("[1A"); //move across
	//37 ticks for 3000mv
	float scale_factor = (float)37/(float)3000;
	uint16_t ticks = scale_factor * value;
	ticks = adcmin(37, ticks);
	for(int pos = 0; pos<ticks; pos++)
	{
		if(pos == ticks-1)
		{
			UART_print("|");
		}
		else
		{
			UART_print("=");
		}
	}
	UART_ESC_code("[H");
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
