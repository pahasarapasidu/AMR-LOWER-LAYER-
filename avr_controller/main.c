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
#include <stdbool.h> 
#include <math.h>

#include "motors.h"
#include "m_usb.h"
#include "bno055_ll.h"
#include "analog.h"
#include "encoder.h"
#include "profiler.h"
#include "systime.h"

#define RX_BUF_SIZE 64

// Receive buffer
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;

// Parsed parameters
static float rx_distance;     // mm
static float rx_angle;        // degrees
static uint16_t rx_max_vel;   // mm/s
static uint16_t rx_max_omega; // deg/s
static uint16_t rx_last_vel;
static uint16_t rx_last_omega;
static float rx_lin_acc;     // mm/s
static float rx_ang_acc; // deg/s

static bool f,b,l,r;

/* -------------------------------  PROTOTYPES ----------------------------*/
static void timer4_init(void);           /* sets up periodic IRQ            */
static void send_telemetry(bool emerg, bool profileDone); /* heavy USB / sensor work         */
static void usb_send_ram(const char *s);
static uint8_t parse_jetson_auto(const char *line);
static uint8_t parse_jetson_teleop(const char *line);
static void receive_from_jetson(void);

static void send_debug(void);
static void send_cmd_echo(void);

/*  ------------------------------ GLOBAL FLAG ------------------------*/
static volatile uint8_t loop_execute = 0; /* set in ISR, cleared in main */

bool emerg = false;
bool profile_done = true;

/* ====================================================*/
int main(void)
{
    /* ---- initialize everything ---- */
    _delay_ms(100);
	
    m_usb_init();
	systime_init();
    motors_init();
    encoder_init();
    analog_init();
    twi_init();
	
    if (!bno055_init())
    {
        m_usb_tx_string("IMU Failed\r\n");
    }

    /* ---- start 125 Hz timer & enable global IRQs ---- */
    timer4_init(); /* Timer-3 compare-match every 8 ms        */
    sei();         /* global interrupt enable                 */

    /* ---------------- MAIN LOOP ---------------------- */
	motion_reset_drive_system();  //initialize the encoder counts, velocities, omegas, distances, angles to 0
    while (1)
    {	
        /* ---------- Emergency Button press status ---------- */
        emerg = encoder_emergency_hit();

        if (loop_execute)
        {
			loop_execute = 0;
			
			encoder_odometry_update();
			receive_from_jetson();
			
			if (!emerg)
			{
				motion_update(); 
				
				if (motion_turn_finished() && motion_move_finished())
				{
					profile_done = true;
					motors_update(0, 0);	
				} else{
					motors_update(motion_velocity(), motion_omega());  
				}                         
			}
			else
			{
				motors_stop_all();
			}
			
			//send_debug();
            send_telemetry(emerg, profile_done);
        }

    }
}

/* -------------------- TIMER-4 INITIALISATION (CTC, 50 Hz) --------------------
   16 MHz / 1024 = 15625 Hz
  15625 Hz × 0.01 s = 156  → OCR4A = 100 ⇒ 100Hz         */
static void timer4_init(void)
{
    /* reset all Timer-4 control registers (mandatory for this timer) */
    TCCR4A = 0;
    TCCR4B = 0;
    TCCR4C = 0;
    TCCR4D = 0; // set WGM40 and WGM41 to normal mode

    TCNT4 = 0;  /* start from zero                        */
    OCR4A = 156; /* compare after 156 counts (~10 ms)      */

    TIMSK4 |= _BV(OCIE4A); /* enable Compare-A interrupt             */

    /* start clock: prescaler = 1024 →  CS43 | CS41                        */
    TCCR4B |= _BV(CS43) | _BV(CS41) | _BV(CS40);
}

/* ----------------------- TIMER-4 COMPARE ISR --------------------------- */
ISR(TIMER4_COMPA_vect)
{
    TCNT4 = 0;              /* emulate CTC                */
    loop_execute = 1; /* signal main loop           */
}

/* ------------------- TELEMETRY SENDER (called from main) ----------------- */
static void send_telemetry(bool emerg, bool profileDone)
{
    char line[100];

    /* ---------- IMU ---------- */
    int16_t h16, r16, p16;
    bno055_get_euler(&h16, &r16, &p16);

    float h = h16 / 16.0f;
    float r = r16 / 16.0f;
    float p = p16 / 16.0f;

    // uint8_t cal = bno055_is_fully_calibrated() ? 1u : 0u;

    /* ---------- ADC ---------- */
    uint16_t vbat_1 = analog_get_battery_1_mV();
    uint16_t vbat_2 = analog_get_battery_2_mV();
    uint16_t cliffL = analog_get_cliff_left();
    uint16_t cliffC = analog_get_cliff_front();
    uint16_t cliffR = analog_get_cliff_right();

    /* ---------- Encoders ---------- */
    int32_t encL = encoder_get_left();
    int32_t encR = encoder_get_right();
	
    /* ---------- Format & ship ---------- */
    /* Packet Structure: { Yaw Roll Pitch encoderLeft encoderRight bat1Voltage bat2Voltage LeftCliff CenterCliff RightCliff emergencyFlag }  */
    snprintf(line, sizeof(line),
             "%6.2f %6.2f %6.2f %10ld %10ld %u %u %u %u %u %u %u\r\n", h, r, p, (long)encL, (long)encR, vbat_1, vbat_2, cliffL, cliffC, cliffR, emerg, profileDone);

    usb_send_ram(line);
    m_usb_tx_push();
}

static void send_debug(void)
{
	char line[128];

	/* grab data */
	const float sl  = encoder_left_speed_mm_s();
	const float sr  = encoder_right_speed_mm_s();
	const float v   = encoder_robot_speed_mm_s();
	const float w   = encoder_robot_omega_dps();
	const float d   = encoder_robot_distance_mm();
	const float a   = encoder_robot_angle_deg();
	const float dt  = encoder_loop_time_us();

	/*  +xxxx.x  → 7 chars (sign + 5 digits + decimal)           */
	/* +xxxxx.x  → 8 chars -- reserve that for distance           */
	/*      xx.x → 7 chars with %7.1f when value < 100            */
	/* adjust widths below only if your ranges are larger          */
	snprintf(line, sizeof(line),
	"SpdL: %+7.1f  SpdR: %+7.1f   Vel: %+7.1f   Omg: %+6.1f  dist: %+8.1f  ang: %+7.1f  dt: %7.1f  ",
	sl, sr, v, w, d, a, dt);

	usb_send_ram(line);
	m_usb_tx_push();
}

/* ------------------- Tiny helper ------------------------- */
static void usb_send_ram(const char *s)
{
    while (*s)
        m_usb_tx_char(*s++);
}

static uint8_t parse_jetson_auto(const char *line)
{

    extern float rx_distance, rx_angle, rx_lin_acc, rx_ang_acc;
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
                     &rx_ang_acc);
    return cnt;
}

static uint8_t parse_jetson_teleop(const char *line)
{

	extern bool f,b,l,r;

	int cnt = sscanf(line, "%u,%u,%u,%u",
	(unsigned int *)&f, (unsigned int *)&b,
	(unsigned int *)&l, (unsigned int *)&r);
	
	return cnt;
}

static void send_cmd_echo(void)
{
	/* use the existing globals filled by parse_jetson_auto() */
	extern float    rx_distance,  rx_angle,  rx_lin_acc,  rx_ang_acc;
	extern uint16_t rx_max_vel,   rx_max_omega,
	rx_last_vel,  rx_last_omega;

	char buf[150];
	/* Format:  “CMD d=500.0 a=90.0 vmax=300 wmax=120 vend=0 wend=0 acc=500.0 aacc=360.0” */
	snprintf(buf, sizeof(buf),
	"CMD d=%g a=%g vmax=%u wmax=%u vend=%u wend=%u acc=%g aacc=%g\r\n",
	rx_distance, rx_angle,
	rx_max_vel, rx_max_omega,
	rx_last_vel, rx_last_omega,
	rx_lin_acc, rx_ang_acc);

	usb_send_ram(buf);
	m_usb_tx_push();
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
                if (parse_jetson_auto(rx_buf))
                {
					//send_cmd_echo();
					
					/* linear segment first */
					motion_start_move(rx_distance,
					rx_max_vel,
					rx_last_vel,     // final speed
					rx_lin_acc);

					/* rotational segment next – if needed */
					motion_start_turn(rx_angle,
					rx_max_omega,
					rx_last_omega,
					rx_ang_acc);
					
					profile_done = false;
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
