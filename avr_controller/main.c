/*
 * avr_controller.c
 *
 * Created: 5/5/2025 7:28:21 PM
 * Author : Endeavor360
 */ 

#include <avr/io.h>
#include <stdio.h>


#include "config.h"
#include "motors.h"
#include "m_usb.h"
#include "bno055_ll.h"

static void usb_send_ram(const char *s)
{
	while (*s) {
		m_usb_tx_char(*s++);
	}
}

int main(void) {
	// — initialize everything —
	motors_init();
	// — quick test sequence —
	motors_enable_left (true);
	motors_enable_right(true);
	motors_set_speed_left (12);
	motors_set_speed_right(12);
	_delay_ms(10000);

	motors_stop_all();
	
	m_usb_init();                      /* start PLL, attach to bus    */
	while (!m_usb_isconnected()) { }   /* wait until host opens port  */

	m_usb_tx_string("M2 ready\r\n");  
	
	twi_init();            /* called inside bno055_init() too — harmless    */

	if (!bno055_init()) {
		/* blink LED or hang here if IMU not found */
		for (;;) { }
	}

	char line[64];

	/* ----------  main telemetry loop  ---------- */
	while (1) {
		int16_t h16, r16, p16;
		bno055_get_euler(&h16, &r16, &p16);   /* raw = deg·16              */

		/* convert to float degrees for nicer printing */
		float h = h16 / 16.0f;
		float r = r16 / 16.0f;
		float p = p16 / 16.0f;

		uint8_t cal_ok = bno055_is_fully_calibrated() ? 1u : 0u;

		/* craft one ASCII line */
		snprintf(line, sizeof(line),
		"H:%6.1f R:%6.1f P:%6.1f CAL:%u\r\n", h, r, p, cal_ok);

		usb_send_ram(line);
		m_usb_tx_push();                 /* flush buffer immediately      */

		_delay_ms(20);                   /* 50 Hz output                  */
	}
}
