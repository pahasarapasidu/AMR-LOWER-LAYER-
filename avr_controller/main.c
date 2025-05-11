/*
 * avr_controller.c
 *
 * Created: 5/5/2025 7:28:21 PM
 * Author : Endeavor360
 */ 

#include <avr/io.h>


#include "config.h"
#include "motors.h"
#include "m_usb.h"

int main(void) {
	// — initialize everything —
	motors_init();

	// — quick test sequence —
	motors_enable_left (true);
	motors_enable_right(true);
	motors_set_speed_left (2);
	motors_set_speed_right(2);
	_delay_ms(1000);

	motors_stop_all();
	
	m_usb_init();                      /* start PLL, attach to bus    */
	while (!m_usb_isconnected()) { }   /* wait until host opens port  */

	m_usb_tx_string("M2 ready\r\n");   /* greeting so you know it’s alive */

	/* ------------- main service loop ------------------- */
	for (int i = 0;i<1000; i++)
	{
		/* m_usb_rx_char()  returns ?1 if buffer empty     */
		int16_t c = m_usb_rx_char();
		if (c >= 0)
		{
			/* 1. echo raw character back to terminal      */
			m_usb_tx_char((uint8_t)c);

			/* 2. print a TAB + decimal value + CR/LF      */
			m_usb_tx_char('\t');
			m_usb_tx_uint((uint8_t)c);
			m_usb_tx_string("\r\n");
		}
		_delay_ms(1000);
		/* -------- put your real-time tasks here -------- */
		/* eg. sensor sampling, PID updates, watchdog      */
	}
}
