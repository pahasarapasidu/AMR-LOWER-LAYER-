/**
 * @file bno055_ll.h
 * @brief Low-level BNO055 IMU driver for ATmega32U4 (register-only, no Wire).
 *
 * Author : Kiran Gunathilaka (May 2025)
 *

 *  TWI overview (ATmega32U4, DS40002085A, 25):
 *    SCL freq = F_CPU / (16 + 2*TWBR*4^TWPS)
 *    Write sequence : START ? SLA+W ? reg ? data ? STOP
 *    Read sequence  : START ? SLA+W ? reg
 *                     REPEATED START ? SLA+R ? data ? NACK ? STOP

 */
#ifndef BNO055_LL_H_
#define BNO055_LL_H_

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>



/* ------------------------------------------------------------------------- */
/*                       public driver prototypes                            */
/* ------------------------------------------------------------------------- */
void twi_init(void);                                          /* 25.6 */
bool twi_write(uint8_t sla, const uint8_t *buf, uint8_t len); /* low lvl */
bool twi_read(uint8_t sla, uint8_t *buf, uint8_t len);        /* low lvl */

bool bno055_init(void); /* reset + NDOF  (blocking) */
bool bno055_write8(uint8_t reg, uint8_t val);
bool bno055_read8(uint8_t reg, uint8_t *val);
bool bno055_read(uint8_t reg, uint8_t *buf, uint8_t len);

void bno055_get_euler(int16_t *h, int16_t *r, int16_t *p); /* deg/16 */
void bno055_get_omega(int16_t *gx, int16_t *gy, int16_t *gz);
bool bno055_is_fully_calibrated(void);

bool bno055_apply_offsets(const uint8_t calib[22]); /* write 22-byte blob */
bool bno055_read_offsets(uint8_t calib[22]);        /* read  22-byte blob */

#endif /* BNO055_LL_H_ */
