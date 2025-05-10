/*
 * config.h
 *
 * Created: 5/6/2025 4:35:44 AM
 *  Author: Endeavor360
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

// MCU clock
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>

// Steps per revolution (full-step mode)
#define STEPS_PER_REV 200U

// Left motor pins (Arduino Leonardo)
//  PUL- -> D6  = PD7 / OC1A
//  DIR- -> D12  = PD6
//  ENA- -> D5  = PC6  (active High in inverted logic ; HIGH at mcu = signal low at the other side of the optocoupler)
#define LEFT_PUL_DDR   DDRD
#define LEFT_PUL_PORT  PORTD
#define LEFT_PUL_BIT   PD7

#define LEFT_DIR_DDR   DDRD
#define LEFT_DIR_PORT  PORTD
#define LEFT_DIR_BIT   PD6

#define LEFT_ENA_DDR   DDRC
#define LEFT_ENA_PORT  PORTC
#define LEFT_ENA_BIT   PC6

// Right motor pins (Arduino Leonardo)
//  PUL- -> D9  = PB5 / OC4D
//  DIR- -> D8  = PB4
//  ENA- -> Not terminated at header (Change this when PCB arrives) = PB0  (active High; explained above) 
#define RIGHT_PUL_DDR   DDRB
#define RIGHT_PUL_PORT  PORTB
#define RIGHT_PUL_BIT   PB5

#define RIGHT_DIR_DDR   DDRB
#define RIGHT_DIR_PORT  PORTB
#define RIGHT_DIR_BIT   PB4

#define RIGHT_ENA_DDR   DDRF
#define RIGHT_ENA_PORT  PORTF
#define RIGHT_ENA_BIT   PF7   //for the moment use pin A0 of the Leonardo

#ifndef _BV //this is just to silence the shitty linter in microchip studio
#define _BV(bit) (1 << (bit))
#endif

#define CLOCK_DIVISOR 1024U
#define PRE_SCALE_1024 (_BV(CS43) | _BV(CS41) | _BV(CS40))

#endif // CONFIG_H