/*******************************************************************************
 * bmm350_iface.c — ModusToolbox HAL ↔ Bosch BMM350 SensorAPI bridge
 *
 * How the I2C dummy bytes work:
 *   The Bosch SensorAPI (bmm350.c get_regs) calls dev->read() with
 *   len = (actual_bytes + BMM350_DUMMY_BYTES), where BMM350_DUMMY_BYTES = 2.
 *   It then copies reg_data[index] = buf[index + 2], discarding the first two
 *   bytes itself.  Our read callback therefore just does a plain I2C read of
 *   exactly `len` bytes — no manual dummy handling needed here.
 *******************************************************************************/

#include "bmm350_iface.h"
#include "cyhal.h"
#include <string.h>

/* Shared bmm350_dev instance — populated in main.c bmm350_init() */
struct bmm350_dev g_bmm350;

/* ---------------------------------------------------------------------------
 * I2C read  (plain repeated-start read; API adds 2 to len for dummy bytes)
 * --------------------------------------------------------------------------- */
int8_t bmm350_hal_read(uint8_t  reg_addr,
                        uint8_t *reg_data,
                        uint32_t len,
                        void    *intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t *)intf_ptr;

    cy_rslt_t rc = cyhal_i2c_master_write(i2c, BMM350_I2C_ADDR_BOARD,
                                           &reg_addr, 1, 20, false);
    if (rc != CY_RSLT_SUCCESS) return -1;

    rc = cyhal_i2c_master_read(i2c, BMM350_I2C_ADDR_BOARD,
                                reg_data, (uint16_t)len, 20, true);
    return (rc == CY_RSLT_SUCCESS) ? 0 : -1;
}

/* ---------------------------------------------------------------------------
 * I2C write
 * --------------------------------------------------------------------------- */
int8_t bmm350_hal_write(uint8_t        reg_addr,
                         const uint8_t *reg_data,
                         uint32_t       len,
                         void          *intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t *)intf_ptr;

    uint8_t buf[33]; /* reg_addr + up to 32 data bytes */
    if (len > 32U) return -1;
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);

    cy_rslt_t rc = cyhal_i2c_master_write(i2c, BMM350_I2C_ADDR_BOARD,
                                           buf, (uint16_t)(len + 1U), 20, true);
    return (rc == CY_RSLT_SUCCESS) ? 0 : -1;
}

/* ---------------------------------------------------------------------------
 * Microsecond delay — rounds up to next millisecond (all BMM350 delays > 1 ms)
 * --------------------------------------------------------------------------- */
void bmm350_hal_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    uint32_t ms = (period_us + 999U) / 1000U;
    if (ms == 0U) ms = 1U;
    cyhal_system_delay_ms(ms);
}
