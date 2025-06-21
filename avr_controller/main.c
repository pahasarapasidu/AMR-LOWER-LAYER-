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
#include <math.h>

#include "motors.h"
#include "m_usb.h"
#include "bno055_ll.h"
#include "analog.h"
#include "encoder.h"
#include "profiler.h"

#define RX_BUF_SIZE 64

// Receive buffer
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static bool profile_requested = false;

// Parsed parameters
static float rx_distance;     // mm
static float rx_angle;        // degrees
static uint16_t rx_max_vel;   // mm/s
static uint16_t rx_max_omega; // deg/s
static uint16_t rx_last_vel;
static uint16_t rx_last_omega;
static float rx_lin_acc;     // mm/s�
static float rx_max_ang_acc; // deg/s�

/* -------------------------------  PROTOTYPES ----------------------------*/
static void timer3_init_50Hz(void); /* sets up periodic IRQ            */
static void send_telemetry(void);   /* heavy USB / sensor work         */
static void usb_send_ram(const char *s);
static uint8_t parse_jetson_line(const char *line);
static void receive_from_jetson(void);

/*  ------------------------------ GLOBAL FLAG ------------------------*/
static volatile uint8_t flag_telemetry_due = 0; /* set in ISR, cleared in main */

/* ====================================================*/
int main(void)
{
    /* ---- initialize everything ---- */
    _delay_ms(100);
    m_usb_init();

    // If usb handshake fails, this will block the entire execution of the code, remove at production ready code
    while (!m_usb_isconnected())
    {
    } /* wait for host terminal      */

    m_usb_tx_string("M2 ready\r\n");

    motors_init();
    encoder_init();
    analog_init();

    motors_enable_left(true);
    motors_enable_right(true);
    // /* --------------------- quick test sequence ------------------------------ */
    // motors_set_dir_left(false);
    // motors_set_dir_right(true);
    //_delay_ms(2);
    // motors_set_speed_left(1000);
    // motors_set_speed_right(1000);
    //_delay_ms(20000);
    // motors_stop_all();

    m_usb_tx_string("M2 ready\r\n");

    twi_init();
    if (!bno055_init())
    {
        m_usb_tx_string("IMU Failed\r\n");
    }

    /* ---- start 50 Hz timer & enable global IRQs ---- */
    timer3_init_50Hz(); /* Timer-3 compare-match every 20 ms        */
    sei();              /* global interrupt enable                 */

    /* ---------------- MAIN LOOP ---------------------- */
    while (1)
    {
        // check for any incoming Jetson data
        receive_from_jetson();

        if (profile_requested)
        {
            // decide pure turn vs straight?line
            if (fabsf(rx_angle) > 0.01f && fabsf(rx_distance) < 1e-3f)
            {
                profiler_turn_init(rx_angle,
                                   rx_max_omega,
                                   rx_max_ang_acc);
            }
            else
            {
                profiler_init(rx_distance,
                              rx_max_vel,
                              rx_lin_acc);
            }
            profile_requested = false;
        }

        if (profiler_turn_is_running())
        {
            profiler_turn_update();
        }
        else if (profiler_is_running())
        {
            profiler_update();
        }
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

    // uint8_t cal = bno055_is_fully_calibrated() ? 1u : 0u;

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

static uint8_t parse_jetson_line(const char *line)
{

    extern float rx_distance, rx_angle, rx_lin_acc, rx_max_ang_acc;
    extern uint16_t rx_max_vel, rx_max_omega, rx_last_vel, rx_last_omega;

    // Note: "%f" for floats, "%u" for uint16_t on AVR
    int cnt = sscanf(line,
                     "%f,%f,%u,%u,%u,%u,%f,%f",
                     &rx_distance,
                     &rx_angle,
                     &rx_max_vel,
                     &rx_max_omega,
                     &rx_last_vel,
                     &rx_last_omega,
                     &rx_lin_acc,
                     &rx_max_ang_acc);
    return (cnt == 8) ? 1 : 0;
}

static void receive_from_jetson(void)
{
    while (m_usb_rx_available())
    {
        char c = m_usb_rx_char();
        // start parsing if new line detected
        if (c == '\n' || c == '\r')
        {
            if (rx_index > 0)
            {
                rx_buf[rx_index] = '\0';
                if (parse_jetson_line(rx_buf))
                {
                }
                rx_index = 0;
            }
        }
        else if (rx_index < (RX_BUF_SIZE - 1))
        {
            rx_buf[rx_index++] = c;
        }
    }
}
