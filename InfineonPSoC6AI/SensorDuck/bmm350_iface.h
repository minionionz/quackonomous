/*******************************************************************************
 * bmm350_iface.h — ModusToolbox HAL adapter for the Bosch BMM350 SensorAPI
 *
 * Provides the read/write/delay callbacks and the global bmm350_dev instance.
 * Include this header from main.c instead of bmm350/bmm350.h directly.
 *******************************************************************************/
#pragma once

#include "bmm350/bmm350.h"
#include "cyhal.h"
#include <stdint.h>

/* Confirmed I2C address for this board (SDO tied HIGH → 0x15) */
#define BMM350_I2C_ADDR_BOARD  0x15U

/* Global SensorAPI device struct — filled by main.c after cyhal_i2c_init() */
extern struct bmm350_dev g_bmm350;

/*
 * I2C read callback for bmm350_dev.read
 * The SensorAPI calls this with (len = actual_bytes + 2) to account for the
 * BMM350's 2-byte I2C read dummy; the first 2 bytes returned are discarded
 * internally by the API. Our callback just performs a plain I2C transaction.
 */
int8_t bmm350_hal_read(uint8_t  reg_addr,
                        uint8_t *reg_data,
                        uint32_t len,
                        void    *intf_ptr);

/* I2C write callback for bmm350_dev.write */
int8_t bmm350_hal_write(uint8_t        reg_addr,
                         const uint8_t *reg_data,
                         uint32_t       len,
                         void          *intf_ptr);

/* Microsecond delay callback for bmm350_dev.delayUs */
void bmm350_hal_delay_us(uint32_t period_us, void *intf_ptr);
