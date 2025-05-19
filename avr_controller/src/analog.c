/*
 * analog.c
 *
 * Created: 5/6/2025 4:37:23 AM
 *  Author: Endeavor360
 */ 

#include "analog.h"
#include <avr/io.h>

static inline void adc_select_channel(uint8_t ch)
{
	/* For channels 0-7, MUX[3:0]=ch ; for 8-13 set MUX5 in ADCSRB */
	if (ch <= 7) {
		ADMUX  = (ADMUX & 0xF0) | ch;          // keep REFS & ADLAR
		ADCSRB &= ~(1<<MUX5);
		} else {
		ADMUX  = (ADMUX & 0xF0) | (ch - 8);
		ADCSRB |=  (1<<MUX5);
	}
}

/* ------------------------------------------------------- */
void analog_init(void)
{
	/* AVcc reference, right-adjust, start on channel 0 */
	ADMUX  = (1<<REFS0);            /* AVcc with ext. cap on AREF */
	adc_select_channel(0);

	/* prescaler, enable, no free-running */
	ADCSRA = (1<<ADEN) | ADC_PRESCALER_BITS;

	/* Disable digital input buffers on the used analog pins to save power/noise */
	DIDR0 =  (1<<ADC0D) | (1<<ADC1D) | (1<<ADC4D) | (1<<ADC5D) | (1<<ADC6D);
}

/* blocking, software-averaged read ---------------------- */
uint16_t analog_read_raw(uint8_t channel)
{
	uint32_t acc = 0;

	adc_select_channel(channel);

	for (uint8_t i = 0; i < ADC_NUM_SAMPLES; ++i) {
		ADCSRA |= (1<<ADSC);               /* start conversion          */
		while (ADCSRA & (1<<ADSC)) {;}     /* wait until ADSC clears    */
		acc += ADC;                        /* read ADCL then ADCH       */
	}
	return (uint16_t)(acc / ADC_NUM_SAMPLES);
}

/* ---------------- convenience wrappers -----------------*/
static inline uint16_t to_millivolt(uint16_t adc)
{
	/*  (adc * 1100 mV /1023) * scale-factor  */
	uint32_t mv = (uint32_t)adc * 1100UL / 1023UL;   /* ? AVcc=3.3 V ? update */
	mv = mv * BAT_DIV_FACTOR_NUM / BAT_DIV_FACTOR_DEN;
	return (uint16_t)mv;
}

uint16_t analog_get_battery_1_mV(void)
{
	return to_millivolt(analog_read_raw(ADC_CH_BAT_MAIN));
}

uint16_t analog_get_battery_2_mV(void)
{
	return to_millivolt(analog_read_raw(ADC_CH_BAT_AUX));
}

uint16_t analog_get_cliff_left (void){ return analog_read_raw(ADC_CH_CLIFF_LEFT);  }
uint16_t analog_get_cliff_front(void){ return analog_read_raw(ADC_CH_CLIFF_FRONT); }
uint16_t analog_get_cliff_right(void){ return analog_read_raw(ADC_CH_CLIFF_RIGHT); }
