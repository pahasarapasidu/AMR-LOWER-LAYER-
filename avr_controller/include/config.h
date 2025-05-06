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
//  PUL+ ? D9  = PB5 / OC1A
//  DIR+ ? D8  = PB6
//  ENA+ ? D7  = PE6  (active LOW)
#define LEFT_PUL_DDR   DDRB
#define LEFT_PUL_PORT  PORTB
#define LEFT_PUL_BIT   PB5

#define LEFT_DIR_DDR   DDRB
#define LEFT_DIR_PORT  PORTB
#define LEFT_DIR_BIT   PB6

#define LEFT_ENA_DDR   DDRE
#define LEFT_ENA_PORT  PORTE
#define LEFT_ENA_BIT   PE6

// Right motor pins
//  PUL+ ? D6  = PD7 / OC4D
//  DIR+ ? D5  = PE2
//  ENA+ ? D4  = PD4  (active LOW)
#define RIGHT_PUL_DDR   DDRD
#define RIGHT_PUL_PORT  PORTD
#define RIGHT_PUL_BIT   PD7

#define RIGHT_DIR_DDR   DDRE
#define RIGHT_DIR_PORT  PORTE
#define RIGHT_DIR_BIT   PE2

#define RIGHT_ENA_DDR   DDRD
#define RIGHT_ENA_PORT  PORTD
#define RIGHT_ENA_BIT   PD4

#endif // CONFIG_H