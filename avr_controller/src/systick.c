/*
 * systick.c – periodic control & telemetry scheduler (Timer 3, USB?struct TX)
 * -------------------------------------------------------------------------
 * AMR low?level motor controller
 * Author : Kiran Gunathilaka
 * Updated: 14 May 2025
 *
 *  • Uses **Timer 3** so Timer 1 stays free for step?pulse PWM.
 *  • Sends the telemetry packet by streaming the packed struct byte?by?byte
 *    with m_usb_tx_char(), then flushing with m_usb_tx_push().  No helper
 *    like usb_tx_enqueue() is needed from the legacy USB library.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stddef.h>

#include "systick.h"
#include "m_usb.h"
#include "bno055_ll.h"
#include "adc.h"
#include "profiler.h"

#ifndef F_CPU
#   error "F_CPU must be defined before compiling systick.c"
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define SYSTICK_HZ      100            /* 10 ms tick                    */
#define IR_CH_COUNT     3              /* active Sharp IR channels      */

/* Timer 3: clk/8 ? 2 MHz (for 16 MHz system)                                */
#define TIMER3_PRESC_BITS  (_BV(CS31))
#define TIMER3_PRESC_DIV   8UL
#define OCR3A_VALUE  ((uint16_t)(F_CPU / TIMER3_PRESC_DIV / SYSTICK_HZ) - 1UL)

/* -------------------------------------------------------------------------- */
/* Globals consumed by the main loop                                          */
/* -------------------------------------------------------------------------- */

volatile float g_targetVel   = 0.0f;   /* cm s?¹  */
volatile float g_targetOmega = 0.0f;   /* deg s?¹ */

/* -------------------------------------------------------------------------- */
/* Telemetry frame (tightly packed)                                           */
/* -------------------------------------------------------------------------- */

typedef struct __attribute__((packed))
{
    int32_t enc_left;
    int32_t enc_right;

    int16_t eul_cdeg[3];           /* ×100 deg        */
    int16_t gyr_cdegps[3];         /* ×100 deg s?¹    */

    int16_t last_vel_cms;          /* ×100 cm s?¹     */
    int16_t last_omega_cdegps;     /* ×100 deg s?¹    */

    uint16_t ir_mm[IR_CH_COUNT];
    uint16_t batt_mV;

    uint8_t  flags;
} TelemetryPacket_t;

static TelemetryPacket_t tx_pkt;

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static inline void consume_usb_stream(void);
static inline void sample_sensors(void);
static inline void update_profiler(void);
static inline void push_telemetry(void);

/* If you have a parser for incoming packets, declare it elsewhere. */
extern void parse_usb_byte(uint8_t b);

/* -------------------------------------------------------------------------- */
/* TIMER 3 Compare?Match ISR                                                  */
/* -------------------------------------------------------------------------- */

ISR(TIMER3_COMPA_vect, ISR_BLOCK)
{
    consume_usb_stream();
    sample_sensors();
    update_profiler();
    push_telemetry();
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void systick_init(void)
{
    cli();

    /* CTC mode – TOP in OCR3A */
    TCCR3A = 0;
    TCCR3B = _BV(WGM32);
    OCR3A  = OCR3A_VALUE;

    /* prescaler clk/8 */
    TCCR3B |= TIMER3_PRESC_BITS;

    /* enable compare?match interrupt */
    TIMSK3 |= _BV(OCIE3A);

    sei();
}

/* -------------------------------------------------------------------------- */
/* Helper implementations                                                     */
/* -------------------------------------------------------------------------- */

static inline void consume_usb_stream(void)
{
    while (m_usb_rx_available())
        parse_usb_byte(m_usb_rx_char());
}

static inline void sample_sensors(void)
{
    /* Encoders not yet implemented – use zero placeholders */
    tx_pkt.enc_left  = 0;
    tx_pkt.enc_right = 0;

    /* BNO055 – raw 1/16?deg readings already fit our ×100 scale (?6.25) */
    int16_t ang[3];
    int16_t omg[3];
    bno055_get_euler(&ang[0], &ang[1], &ang[2]);
    bno055_get_omega(&omg[0], &omg[1], &omg[2]);

    for (uint8_t i = 0; i < 3; ++i)
    {
        tx_pkt.eul_cdeg[i]   = ang[i];
        tx_pkt.gyr_cdegps[i] = omg[i];
    }

    for (uint8_t ch = 0; ch < IR_CH_COUNT; ++ch)
        tx_pkt.ir_mm[ch] = adc_get_ir_mm(ch);

    tx_pkt.batt_mV = adc_get_battery_mV();
    tx_pkt.flags   = 0;            /* flag infrastructure TBD */
}

static inline void update_profiler(void)
{
    float vel, omega;
    profiler_step(&vel, &omega);

    g_targetVel   = vel;
    g_targetOmega = omega;

    tx_pkt.last_vel_cms      = (int16_t)(vel   * 100.0f);
    tx_pkt.last_omega_cdegps = (int16_t)(omega * 100.0f);
}

static inline void push_telemetry(void)
{
    const uint8_t *p = (const uint8_t *)&tx_pkt;
    for (uint8_t i = 0; i < sizeof(tx_pkt); ++i)
        m_usb_tx_char(*p++);

    m_usb_tx_push();
}
