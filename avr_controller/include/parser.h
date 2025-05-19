/*
 * parser.h
 *
 * Created: 5/13/2025 12:02:23 PM
 *  Author: Endeavor360
 
 * parser.h – byte?wise command packet decoder
 * -----------------------------------------
 * Packet format (little?endian, minimal size):
 *   0xAA               – start byte
 *   CMD                – command identifier
 *   PAYLOAD            – variable (see below)
 *   CHK                – 8?bit checksum = ?(CMD+payload) mod 256
 *
 * Commands
 *   0x01  Set velocity / omega
 *         payload: int16 vel_cm_s_x100, int16 omega_deg_s_x100
 *   0x02  Set flag byte (E?stop etc.)
 *         payload: uint8 flags
 */

#ifndef PARSER_H
#define PARSER_H

#include <stdint.h>

/* latest goals exposed to the profiler/main loop */
extern volatile float  g_goalVel_cm_s;
extern volatile float  g_goalOmega_deg_s;
extern volatile uint8_t g_cmdFlags;

/* feed a single byte from the USB RX stream */
void parse_usb_byte(uint8_t b);

#endif /* PARSER_H */
