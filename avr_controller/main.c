/*
 * avr_controller.c
 * Modified: 13-May-2025  (adds ADC telemetry)
 * Author   : Endeavor360
 */

#include "config.h"
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>


#include "motors.h"
#include "m_usb.h"
#include "bno055_ll.h"
#include "analog.h"          /* << NEW */

static void usb_send_ram(const char *s)
{
    while (*s) m_usb_tx_char(*s++);
}

int main(void)
{
    /* ---- initialise everything ---- */
    _delay_ms(100);                  /* after flashing, USB needs a pause   */
    m_usb_init();                    /* start PLL, attach to bus            */
    while (!m_usb_isconnected()) { } /* wait until host opens port          */

    motors_init();
    analog_init();                   /* << NEW – ADC/HW set-up              */

    /* quick test sequence -------------------------------------------------- */
    motors_enable_left (true);
    motors_enable_right(true);
	
	motors_set_dir_left(false);
	motors_set_dir_right(true);
	_delay_ms(2); 
    motors_set_speed_left (1000);
    motors_set_speed_right(1000);
    _delay_ms(10000);
    motors_stop_all();

    m_usb_tx_string("M2 ready\r\n");

    twi_init();

    if (!bno055_init()) {
        for (;;) { }                 /* block forever if IMU fails          */
    }

    char line[160];                  /* bigger buffer for extra fields      */

    /* ------------------- main telemetry loop ----------------------------- */
    while (1)
    {
        /* ------------ IMU ------------- */
        int16_t h16,r16,p16;
        bno055_get_euler(&h16,&r16,&p16);

        int16_t gx16,gy16,gz16;
        bno055_get_omega(&gx16,&gy16,&gz16);

        float h  = h16  / 16.0f;
        float r  = r16  / 16.0f;
        float p  = p16  / 16.0f;
        float wx = gx16 / 16.0f;
        float wy = gy16 / 16.0f;
        float wz = gz16 / 16.0f;

        uint8_t cal = bno055_is_fully_calibrated() ? 1u : 0u;

        /* ------------ ADC ------------- */
        uint16_t vbat_main = analog_get_battery_1_mV();
        uint16_t vbat_aux  = analog_get_battery_2_mV();

        uint16_t cliffL = analog_get_cliff_left();
        uint16_t cliffF = analog_get_cliff_front();
        uint16_t cliffR = analog_get_cliff_right();

        /* ------------ build line ------ */
        /* Format example:
         * H: 42.12  R: -1.25  P:  0.50  Wx:  0.00 Wy:  0.03 Wz: -0.06 CAL:1
         * VB:11980/11980mV  Cliff: 512 501 509
         */
        snprintf(line, sizeof(line),
            "H:%6.2f R:%6.2f P:%6.2f  "
            "Wx:%6.2f Wy:%6.2f Wz:%6.2f CAL:%u  "
            "VB:%umV/%umV  Cliff:%u %u %u\r\n",
            h,r,p, wx,wy,wz, cal,
            vbat_main, vbat_aux,
            cliffL, cliffF, cliffR);

        usb_send_ram(line);
        m_usb_tx_push();

        _delay_ms(10);                /* 50 Hz overall output rate           */
    }
}
