/*
 * avr_controller.c
 * Modified: 21-May-2025  (telemetry via Timer-3 @ 50 Hz)
 * Author   : Endeavor360
 */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#include "motors.h"
#include "m_usb.h"
#include "bno055_ll.h"
#include "analog.h"
#include "encoder.h"

/* -------------------------------  PROTOTYPES ----------------------------*/
static void timer3_init_50Hz(void); /* sets up periodic IRQ            */
static void send_telemetry(void);   /* heavy USB / sensor work         */
static void usb_send_ram(const char *s);

/*  ------------------------------ GLOBAL FLAG ------------------------*/
static volatile uint8_t flag_telemetry_due = 0; /* set in ISR, cleared in main */

/* ====================================================*/
int main(void)
{
    /* ---- initialize everything ---- */
    _delay_ms(100);
    m_usb_init();
    while (!m_usb_isconnected())
    {
    } /* wait for host terminal      */
	
	m_usb_tx_string("M2 ready\r\n");
	
    motors_init();
    encoder_init();
    analog_init();

    // /* --------------------- quick test sequence ------------------------------ */
    motors_enable_left(true);
    motors_enable_right(true);

    motors_set_dir_left(false);
    motors_set_dir_right(true);
    _delay_ms(2);
    motors_set_speed_left(1000);
    motors_set_speed_right(1000);
    _delay_ms(20000);
    motors_stop_all();

    m_usb_tx_string("M2 ready\r\n");

    twi_init();
    if (!bno055_init())
    {
        for (;;)
        {
        }
    } /* halt if IMU fails */

    /* ---- start 50 Hz timer & enable global IRQs ---- */
    timer3_init_50Hz(); /* Timer-3 compare-match every 20 ms        */
    sei();              /* global interrupt enable                 */

    /* ---------------- MAIN LOOP ---------------------- */
    while (1)
    {
        _delay_ms(1);
    }
}

/* --------------------- TIMER-3 INITIALISATION (CTC, 50 Hz) ------------------
   16 MHz / 256 prescale = 62 500 Hz
   62 500 Hz 0.02 s = 1 250 counts ? OCR3A = 1249               */
static void timer3_init_50Hz(void)
{
    TCCR3A = 0;                          /* normal port operation       */
    TCCR3B = (1 << WGM32) | (1 << CS32); /* CTC, prescaler = 256        */
    OCR3A = 1249;
    TIMSK3 = (1 << OCIE3A); /* enable compare-match A IRQ  */
}

/* ------------------------- TIMER-3 COMPARE ISR --------------------------- */
ISR(TIMER3_COMPA_vect)
{
    send_telemetry(); /* quick! no heavy work here   */
}

/* ------------------- TELEMETRY SENDER (called from main) ----------------- */
static void send_telemetry(void)
{
    char line[160];

    /* ---------- IMU ---------- */
    int16_t h16, r16, p16;
    bno055_get_euler(&h16, &r16, &p16);

    float h = h16 / 16.0f;
    float r = r16 / 16.0f;
    float p = p16 / 16.0f;

    //uint8_t cal = bno055_is_fully_calibrated() ? 1u : 0u;

    /* ---------- ADC ---------- */
    uint16_t vbat_main = analog_get_battery_1_mV();
    uint16_t vbat_aux = analog_get_battery_2_mV();
    uint16_t cliffL = analog_get_cliff_left();
    uint16_t cliffF = analog_get_cliff_front();
    uint16_t cliffR = analog_get_cliff_right();

    /* ---------- Encoders ---------- */
    int32_t encL = encoder_get_left();
    int32_t encR = encoder_get_right();

    /* ---------- Format & ship ---------- */
    snprintf(line, sizeof(line),
             "H:%6.2f R:%6.2f P:%6.2f  "
             "VB:%umV/%umV   Cliff:%u %u %u  "
             "Enc:%11ld/%11ld\r\n", /* 11 chars wide, signed     */
             h, r, p,               /* argument now matches %u   */
             vbat_main, vbat_aux,
             cliffL, cliffF, cliffR,
             (long)encL, (long)encR); /* cast silences -format    */

    usb_send_ram(line);
    m_usb_tx_push();
}

/* ------------------- Tiny helper ------------------------- */
static void usb_send_ram(const char *s)
{
    while (*s)
        m_usb_tx_char(*s++);
}