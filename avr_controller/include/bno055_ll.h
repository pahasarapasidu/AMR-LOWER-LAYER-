/**
 * @file bno055_ll.h
 * @brief Low-level BNO055 IMU driver for ATmega32U4 (register-only, no Wire).
 *
 * Author : Kiran Gunathilaka (May 2025)
 *
 * ????????????????????????????????????????????????????????????????????????????
 *  TWI overview (ATmega32U4, DS40002085A, §25):
 *    SCL freq = F_CPU / (16 + 2·TWBR·4^TWPS)
 *    Write sequence : START ? SLA+W ? reg ? data… ? STOP
 *    Read sequence  : START ? SLA+W ? reg
 *                     REPEATED START ? SLA+R ? data… ? NACK ? STOP
 * ????????????????????????????????????????????????????????????????????????????
 */
#ifndef BNO055_LL_H_
#define BNO055_LL_H_

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>


/* BNO055 I2C addresses (BOOT pin low = ADDR = 0x28, high = 0x29) */
#define BNO055_ADDR_A          0x28u
#define BNO055_ADDR_B          0x29u
#define BNO055_I2C_ADDR        BNO055_ADDR_A   /**< change if needed        */

/* Desired I²C speed (Hz) */
#define TWI_SCL_HZ             400000UL         /**< 400 kbit “Fast” mode   */

/* ------------------------------------------------------------------------- */
/*                       2.  public driver prototypes                        */
/* ------------------------------------------------------------------------- */
void     twi_init(void);                                          /* §25.6 */
bool     twi_write(uint8_t sla, const uint8_t *buf, uint8_t len); /* low lvl */
bool     twi_read (uint8_t sla, uint8_t *buf, uint8_t len);       /* low lvl */

bool     bno055_init(void);                       /* reset + NDOF  (blocking) */
bool     bno055_write8(uint8_t reg, uint8_t val);
bool     bno055_read8 (uint8_t reg, uint8_t *val);
bool     bno055_read  (uint8_t reg, uint8_t *buf, uint8_t len);

void     bno055_get_euler(int16_t *h, int16_t *r, int16_t *p);    /* deg/16 */
bool     bno055_is_fully_calibrated(void);

#endif /* BNO055_LL_H_ */
