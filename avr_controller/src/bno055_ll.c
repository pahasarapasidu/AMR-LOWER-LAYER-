/**
 * @file bno055_ll.c
 * @brief Implementation of low-level BNO055 IMU driver for ATmega32U4
 *        using only the TWI registers (see datasheet pp. 225-255).
 */

#include "bno055_ll.h"

/* ??????????????????????????????  LOCAL MACROS ????????????????????????????? */
#define TW_START          (1u << TWSTA)
#define TW_STOP           (1u << TWSTO)
#define TW_INT_FLAG       (1u << TWINT)
#define TW_ENABLE         (1u << TWEN)
#define TW_ACK            (1u << TWEA)

/* TW_STATUS = TWSR & 0xF8  (§25.7.4) */
#define TW_STATUS         (TWSR & 0xF8u)

/* ???????????????????????????  INTERNAL UTILITIES ?????????????????????????? */
static inline void twi_wait(void)
{
    while (!(TWCR & TW_INT_FLAG)) { /* wait for TWINT set */ }
}

static bool twi_start(uint8_t addr_rw)
{
    TWCR = TW_INT_FLAG | TW_START | TW_ENABLE;
    twi_wait();
    uint8_t st = TW_STATUS;
    if (st != 0x08 && st != 0x10) return false;       /* START / REP START */

    /* load SLA+R/W */
    TWDR = addr_rw;
    TWCR = TW_INT_FLAG | TW_ENABLE;
    twi_wait();
    st = TW_STATUS;
    return (st == 0x18 /* SLA+W ACK */) || (st == 0x40 /* SLA+R ACK */);
}

static void twi_stop(void)
{
    TWCR = TW_INT_FLAG | TW_STOP | TW_ENABLE;
}

/* ???????????????????????????  PUBLIC TWI ROUTINES ????????????????????????? */
void twi_init(void)
{
    /* prescaler bits (TWPS1:0) = 0 ? prescaler = 1 */
    TWSR &= ~((1u << TWPS0) | (1u << TWPS1));

    /* choose TWBR such that SCL ? TWI_SCL_HZ                   *
     * TWBR = (F_CPU / SCL - 16) / (2·4^TWPS)                  */
    TWBR = (uint8_t)((F_CPU / TWI_SCL_HZ - 16UL) / 2UL);

    TWCR = TW_ENABLE; /* enable module, no interrupt yet */
}

bool twi_write(uint8_t sla, const uint8_t *buf, uint8_t len)
{
    if (!twi_start((sla << 1) | 0)) return false;
    for (uint8_t i = 0; i < len; ++i) {
        TWDR = buf[i];
        TWCR = TW_INT_FLAG | TW_ENABLE;
        twi_wait();
        if (TW_STATUS != 0x28) { twi_stop(); return false; } /* data ACK */
    }
    twi_stop();
    return true;
}

bool twi_read(uint8_t sla, uint8_t *buf, uint8_t len)
{
    if (!twi_start((sla << 1) | 1)) return false;
    for (uint8_t i = 0; i < len; ++i) {
        /* ACK all bytes except last */
        TWCR = TW_INT_FLAG | TW_ENABLE | (i == len - 1 ? 0 : TW_ACK);
        twi_wait();
        if ((i < len - 1 && TW_STATUS != 0x50) || /* data with ACK */
            (i == len - 1 && TW_STATUS != 0x58))   /* data with NACK */
        { twi_stop(); return false; }
        buf[i] = TWDR;
    }
    twi_stop();
    return true;
}

/* ???????????????????????????  BNO055 BASIC ACCESS ????????????????????????? */
bool bno055_write8(uint8_t reg, uint8_t val)
{
    uint8_t pkt[2] = { reg, val };
    return twi_write(BNO055_I2C_ADDR, pkt, 2);
}

bool bno055_read8(uint8_t reg, uint8_t *val)
{
    if (!twi_write(BNO055_I2C_ADDR, &reg, 1)) return false;
    return twi_read(BNO055_I2C_ADDR, val, 1);
}

bool bno055_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!twi_write(BNO055_I2C_ADDR, &reg, 1)) return false;
    return twi_read(BNO055_I2C_ADDR, buf, len);
}

/* ???????????????????????????  HIGH-LEVEL HELPERS ?????????????????????????? */
static bool bno055_set_mode(uint8_t mode)
{
    return bno055_write8(0x3D, mode); /* BNO055_OPR_MODE_ADDR */
}

bool bno055_init(void)
{
    twi_init();

    /* Ensure sensor is present */
    uint8_t id = 0;
    if (!bno055_read8(0x00, &id) || id != 0xA0) return false; /* CHIP_ID */

    /* Switch to CONFIG, reset, then NDOF */
    bno055_set_mode(0x00);              /* CONFIG      */
    _delay_ms(25);

    bno055_write8(0x3F, 0x20);          /* SYS_TRIGGER, reset */
    _delay_ms(650);                     /* ~650 ms boot time */

    bno055_set_mode(0x0C);              /* NDOF fusion */
    _delay_ms(20);
    return true;
}

void bno055_get_euler(int16_t *h, int16_t *r, int16_t *p)
{
    uint8_t buf[6];
    if (bno055_read(0x1A, buf, 6)) {    /* EULER_H_LSB */
        *h = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
        *r = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
        *p = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    }
}

bool bno055_is_fully_calibrated(void)
{
    uint8_t cal;
    if (!bno055_read8(0x35, &cal)) return false;   /* CALIB_STAT */
    return ((cal >> 6) & 0x03) == 3;               /* SYS == 3   */
}