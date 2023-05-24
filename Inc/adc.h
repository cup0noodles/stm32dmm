/*
 * adc.h
 * Matthew Wong, Joshua Cheruvelil
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/* Correction values */
#define ZONE_01 510
#define ZONE_12 2250

#define MZ0 1
#define MZ1 1.0241
#define MZ2 0.9678
#define BZ0 -1
#define BZ1 -20
#define BZ2 115

/*
 * Configure the keypad to be usable
 */
void adc_init(void);

uint16_t adcmax(uint16_t a, uint16_t b);
uint16_t adcmin(uint16_t a, uint16_t b);
uint16_t adc_cc(uint16_t sample);

#endif /* INC_ADC_H_ */
