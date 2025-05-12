// -----------------------------------------------------------------------------
// AVR USB communication subsystem for ATmega32u4
// version: 2.2
// same as version 2.1 built for M2 MAEVARM
// date: June 16, 2012
// authors: J. Fiene & J. Romano
// -----------------------------------------------------------------------------

#ifndef m_usb__
#define m_usb__

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "config.h"

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

// INITIALIZATION: -------------------------------------------------------------

void m_usb_init(void);
// initialize the USB subsystem

char m_usb_isconnected(void);
// confirm that the USB port is connected to a PC

// RECEIVE: -------------------------------------------------------------------

unsigned char m_usb_rx_available(void);
// returns the number of bytes (up to 255) waiting in the receive FIFO buffer

char m_usb_rx_char(void);
// retrieve a oldest byte from the receive FIFO buffer (-1 if timeout/error)

void m_usb_rx_flush(void);
// discard all data in the receive buffer

// TRANSMIT: ------------------------------------------------------------------

char m_usb_tx_char(unsigned char c);
// add a single 8-bit unsigned char to the transmit buffer, return -1 if error

void m_usb_tx_hexchar(unsigned char i);
// add an unsigned char to the transmit buffer, send as two hex-value characters

void m_usb_tx_hex(unsigned int i);
// add an unsigned int to the transmit buffer, send as four hex-value characters

void m_usb_tx_int(int i);
// add a signed int to the transmit buffer, send as a sign character then 5 decimal-value characters

void m_usb_tx_uint(unsigned int i);
// add an unsigned int to the transmit buffer, send as 5 decimal-value characters

void m_usb_tx_long(long i);
// add a signed long to the transmit buffer, send as a sign character then 5 decimal-value characters

void m_usb_tx_ulong(unsigned long i);
// add an unsigned long to the transmit buffer, send as 5 decimal-value characters

#define m_usb_tx_string(s) print_P(PSTR(s))
// add a string to the transmit buffer

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// ---- OVERLOADS FOR M1 BACK COMPATIBILITY ----
#define usb_init() m_usb_init()
#define usb_configured() m_usb_isconnected()

#define usb_rx_available() m_usb_rx_available()
#define usb_rx_flush() m_usb_rx_flush()
#define usb_rx_char() m_usb_rx_char()

#define usb_tx_char(val) m_usb_tx_char(val)
#define usb_tx_hex(val) m_usb_tx_hex(val)
#define usb_tx_decimal(val) m_usb_tx_uint(val)
#define usb_tx_string(val) m_usb_tx_string(val)
#define usb_tx_push() m_usb_tx_push()

#define m_usb_rx_ascii() m_usb_rx_char()
#define m_usb_tx_ascii(val) m_usb_tx_char(val)

// EVERYTHING ELSE *****************************************************************

// setup

int8_t usb_serial_putchar(uint8_t c);                          // transmit a character
int8_t usb_serial_putchar_nowait(uint8_t c);                   // transmit a character, do not wait
int8_t usb_serial_write(const uint8_t *buffer, uint16_t size); // transmit a buffer
void print_P(const char *s);
void phex(unsigned char c);
void phex16(unsigned int i);
void m_usb_tx_hex8(unsigned char i);
void m_usb_tx_push(void);

// serial parameters
uint32_t usb_serial_get_baud(void);             // get the baud rate
uint8_t usb_serial_get_stopbits(void);          // get the number of stop bits
uint8_t usb_serial_get_paritytype(void);        // get the parity type
uint8_t usb_serial_get_numbits(void);           // get the number of data bits
uint8_t usb_serial_get_control(void);           // get the RTS and DTR signal state
int8_t usb_serial_set_control(uint8_t signals); // set DSR, DCD, RI, etc

// This file does not include the HID debug functions, so these empty
// macros replace them with nothing, so users can compile code that
// has calls to these functions.
#define usb_debug_putchar(c)
#define usb_debug_flush_output()

#endif