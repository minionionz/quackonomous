/*******************************************************************************
 * main.c — SensorDuck JSON telemetry
 *
 * Reads BMI270 (accel + gyro) and BMM350 (magnetometer + heading) and sends
 * one JSON line every 100 ms on two UARTs:
 *
 *   ESP32 UART  P9_1 TX → ESP32 RX          115200 baud
 *               P9_0 RX ← ESP32 TX
 *               GND     → ESP32 GND          (mandatory!)
 *
 *   Debug UART  KitProg3 USB                 115200 baud  (mirrors ESP32 stream)
 *
 * JSON format (one line, \r\n terminated):
 * {
 *   "t"  : ms,          // uptime in milliseconds (wraps at ~49 days)
 *   "imu": {
 *     "accel": {        // linear acceleration from BMI270
 *       "x": mg,        // +X = board right,   -X = board left   (1 g ≈ 9810 mg)
 *       "y": mg,        // +Y = board forward, -Y = board back
 *       "z": mg         // +Z = board up,      -Z = board down   (gravity ≈ +9810 mg when flat)
 *     },
 *     "gyro": {         // angular rate from BMI270
 *       "x": dps,       // roll  rate (right-hand rule around X axis)
 *       "y": dps,       // pitch rate
 *       "z": dps        // yaw   rate — duck turning left/right
 *     }
 *   },
 *   "mag": {
 *     "hdg": deg,       // compass heading 0–360° (0 = magnetic north, 90 = east)
 *                       // derived from atan2(field.y, field.x) — valid only when board is flat
 *     "field": {        // raw magnetic field vector from BMM350, unit: µT
 *       "x": uT,        // horizontal east–west component  (used for heading)
 *       "y": uT,        // horizontal north–south component (used for heading)
 *       "z": uT         // vertical dip component — NOT used for heading, but sending
 *                       // it lets the ESP32 do tilt-compensated heading by combining
 *                       // field.x/y/z with the IMU accel to correct for board tilt
 *     }
 *   },
 *   "ok": {
 *     "imu": bool,      // false if BMI270 did not initialise or read failed
 *     "mag": bool       // false if BMM350 did not initialise or read failed
 *   }
 * }
 *
 * Target : CY8CKIT-062S2-AI  (TARGET=APP_CY8CKIT-062S2-AI)
 * Build  : make program
 *******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "bmm350_iface.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * Configuration
 * ============================================================================ */
#define SENSOR_I2C_SCL    CYBSP_I2C_SCL   /* P0_2 */
#define SENSOR_I2C_SDA    CYBSP_I2C_SDA   /* P0_3 */
#define SENSOR_I2C_FREQ   400000U

/* UART to Duck ESP32 (Arduino header D0/D1) */
#define ESP_UART_TX       P9_1            /* board TX  →  ESP32 RX */
#define ESP_UART_RX       P9_0            /* board RX  ←  ESP32 TX */
#define ESP_UART_BAUD     115200U

#define SEND_INTERVAL_MS  100U            /* 10 Hz */

/* ============================================================================
 * BMI270 register map  (I2C 0x68)
 * ============================================================================ */
#define BMI270_ADDR             0x68U
#define BMI270_REG_CHIP_ID      0x00U
#define BMI270_CHIP_ID_VAL      0x24U
#define BMI270_REG_INTERNAL_STS 0x21U
#define BMI270_REG_ACC_X_LSB    0x0CU    /* 6 bytes: ax_l ax_h ay_l ay_h az_l az_h */
#define BMI270_REG_GYR_X_LSB    0x12U    /* 6 bytes: gx_l gx_h gy_l gy_h gz_l gz_h */
#define BMI270_REG_ACCEL_CONF   0x40U
#define BMI270_REG_ACCEL_RANGE  0x41U    /* 0x03 = ±16 g  →  2048 LSB/g */
#define BMI270_REG_GYR_CONF     0x42U
#define BMI270_REG_GYR_RANGE    0x43U    /* 0x00 = ±2000 dps  →  16.384 LSB/dps */
#define BMI270_REG_PWR_CONF     0x7CU
#define BMI270_REG_PWR_CTRL     0x7DU
#define BMI270_REG_CMD          0x7EU
#define BMI270_CMD_SOFTRESET    0xB6U
#define BMI270_REG_INIT_CTRL    0x59U
#define BMI270_REG_INIT_ADDR_0  0x5BU
#define BMI270_REG_INIT_ADDR_1  0x5CU
#define BMI270_REG_INIT_DATA    0x5EU
#define BMI270_CONFIG_CHUNK     64U

#define BMI270_LOAD_CONFIG_BLOB
#ifdef BMI270_LOAD_CONFIG_BLOB
#include "bmi270_config.h"
#endif

/* ============================================================================
 * HAL objects
 * ============================================================================ */
static cyhal_i2c_t  g_i2c;
static cyhal_uart_t g_esp_uart;

static bool bmi270_ok   = false;
static bool bmm350_ok   = false;
static bool esp_uart_ok = false;

/* ============================================================================
 * I2C helpers
 * ============================================================================ */
static cy_rslt_t i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return cyhal_i2c_master_write(&g_i2c, dev, buf, 2, 20, true);
}

static cy_rslt_t i2c_read_regs(uint8_t dev, uint8_t reg,
                                uint8_t *out, uint16_t len)
{
    cy_rslt_t rc = cyhal_i2c_master_write(&g_i2c, dev, &reg, 1, 20, false);
    if (rc != CY_RSLT_SUCCESS) return rc;
    return cyhal_i2c_master_read(&g_i2c, dev, out, len, 20, true);
}

/* ============================================================================
 * BMI270 init
 * ============================================================================ */
#ifdef BMI270_LOAD_CONFIG_BLOB
static void bmi270_load_config_blob(void)
{
    i2c_write_reg(BMI270_ADDR, BMI270_REG_INIT_CTRL, 0x00U);
    cyhal_system_delay_ms(1);

    uint8_t buf[BMI270_CONFIG_CHUNK + 1];
    for (size_t i = 0; i < bmi270_config_size; i += BMI270_CONFIG_CHUNK) {
        uint16_t word_addr = (uint16_t)(i >> 1);
        i2c_write_reg(BMI270_ADDR, BMI270_REG_INIT_ADDR_0,
                      (uint8_t)(word_addr & 0x0FU));
        i2c_write_reg(BMI270_ADDR, BMI270_REG_INIT_ADDR_1,
                      (uint8_t)((word_addr >> 4) & 0xFFU));
        size_t chunk = (i + BMI270_CONFIG_CHUNK <= bmi270_config_size)
                       ? BMI270_CONFIG_CHUNK : (bmi270_config_size - i);
        buf[0] = BMI270_REG_INIT_DATA;
        memcpy(&buf[1], &bmi270_config_file[i], chunk);
        cyhal_i2c_master_write(&g_i2c, BMI270_ADDR, buf,
                               (uint16_t)(chunk + 1), 100, true);
    }
    i2c_write_reg(BMI270_ADDR, BMI270_REG_INIT_CTRL, 0x01U);
    cyhal_system_delay_ms(150);
}
#endif /* BMI270_LOAD_CONFIG_BLOB */

static void bmi270_init(void)
{
    uint8_t id = 0;
    i2c_write_reg(BMI270_ADDR, BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
    cyhal_system_delay_ms(200);

    if (i2c_read_regs(BMI270_ADDR, BMI270_REG_CHIP_ID, &id, 1) != CY_RSLT_SUCCESS
            || id != BMI270_CHIP_ID_VAL) {
        printf("[IMU] BMI270: init failed (id=0x%02X)\r\n", id);
        return;
    }

    i2c_write_reg(BMI270_ADDR, BMI270_REG_PWR_CONF, 0x00U);
    cyhal_system_delay_ms(1);

#ifdef BMI270_LOAD_CONFIG_BLOB
    printf("[IMU] loading config blob (%u bytes)...\r\n",
           (unsigned)bmi270_config_size);
    bmi270_load_config_blob();
#endif

    i2c_write_reg(BMI270_ADDR, BMI270_REG_PWR_CTRL, 0x0EU); /* accel+gyro+temp on */
    cyhal_system_delay_ms(100);
    i2c_write_reg(BMI270_ADDR, BMI270_REG_ACCEL_CONF,  0xA8U); /* 100 Hz, perf mode */
    i2c_write_reg(BMI270_ADDR, BMI270_REG_ACCEL_RANGE, 0x03U); /* ±16 g             */
    i2c_write_reg(BMI270_ADDR, BMI270_REG_GYR_CONF,    0xA8U); /* 100 Hz, perf mode */
    i2c_write_reg(BMI270_ADDR, BMI270_REG_GYR_RANGE,   0x00U); /* ±2000 dps         */
    cyhal_system_delay_ms(50);

    bmi270_ok = true;
    printf("[IMU] BMI270: OK (id=0x%02X, ±16g / ±2000dps @ 100 Hz)\r\n", id);
}

/* ============================================================================
 * BMM350 init  (Bosch SensorAPI)
 * ============================================================================ */
static void bmm350_init(void)
{
    g_bmm350.intfPtr = &g_i2c;
    g_bmm350.read    = bmm350_hal_read;
    g_bmm350.write   = bmm350_hal_write;
    g_bmm350.delayUs = bmm350_hal_delay_us;

    if (bmm350Init(&g_bmm350) != BMM350_OK) {
        printf("[MAG] BMM350: init failed\r\n");
        return;
    }
    if (bmm350SetOdrPerformance(BMM350_DATA_RATE_25HZ,
                                BMM350_LOWNOISE, &g_bmm350) != BMM350_OK) {
        printf("[MAG] BMM350: ODR config failed\r\n");
        return;
    }
    if (bmm350SetPowerMode(eBmm350NormalMode, &g_bmm350) != BMM350_OK) {
        printf("[MAG] BMM350: power-mode failed\r\n");
        return;
    }
    bmm350_ok = true;
    printf("[MAG] BMM350: OK (id=0x%02X, 25 Hz, low-noise)\r\n", g_bmm350.chipId);
}

/* ============================================================================
 * ESP32 UART init
 * ============================================================================ */
static void esp_uart_init(void)
{
    const cyhal_uart_cfg_t cfg = {
        .data_bits      = 8,
        .stop_bits      = 1,
        .parity         = CYHAL_UART_PARITY_NONE,
        .rx_buffer      = NULL,
        .rx_buffer_size = 0,
    };
    uint32_t actual_baud = 0;

    if (cyhal_uart_init(&g_esp_uart, ESP_UART_TX, ESP_UART_RX,
                         NC, NC, NULL, &cfg) != CY_RSLT_SUCCESS) {
        printf("[UART] ESP32 UART init failed\r\n");
        return;
    }
    if (cyhal_uart_set_baud(&g_esp_uart, ESP_UART_BAUD, &actual_baud) != CY_RSLT_SUCCESS) {
        printf("[UART] ESP32 UART baud set failed\r\n");
        cyhal_uart_free(&g_esp_uart);
        return;
    }
    esp_uart_ok = true;
    printf("[UART] ESP32 UART: OK (%lu baud, TX=P9_1, RX=P9_0)\r\n",
           (unsigned long)actual_baud);
}

/* ============================================================================
 * Build and send one JSON packet
 * ============================================================================ */
static void send_json(uint32_t t_ms)
{
    /* --- IMU (BMI270) --- */
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    bool  imu_valid = false;

    if (bmi270_ok) {
        uint8_t raw[6];
        if (i2c_read_regs(BMI270_ADDR, BMI270_REG_ACC_X_LSB, raw, 6) == CY_RSLT_SUCCESS) {
            /* ±16 g range: 1 LSB = 0.488 mg */
            ax = (float)(int16_t)((uint16_t)raw[1] << 8 | raw[0]) * 0.488f;
            ay = (float)(int16_t)((uint16_t)raw[3] << 8 | raw[2]) * 0.488f;
            az = (float)(int16_t)((uint16_t)raw[5] << 8 | raw[4]) * 0.488f;
        }
        if (i2c_read_regs(BMI270_ADDR, BMI270_REG_GYR_X_LSB, raw, 6) == CY_RSLT_SUCCESS) {
            /* ±2000 dps range: 1 LSB = 1/16.384 dps */
            gx = (float)(int16_t)((uint16_t)raw[1] << 8 | raw[0]) / 16.384f;
            gy = (float)(int16_t)((uint16_t)raw[3] << 8 | raw[2]) / 16.384f;
            gz = (float)(int16_t)((uint16_t)raw[5] << 8 | raw[4]) / 16.384f;
            imu_valid = true;
        }
    }

    /* --- Magnetometer (BMM350) --- */
    float mx = 0, my = 0, mz = 0, hdg = 0;
    bool  mag_valid = false;

    if (bmm350_ok) {
        struct sBmm350MagTempData_t m;
        if (bmm350GetCompensatedMagXYZTempData(&m, &g_bmm350) == BMM350_OK) {
            mx = m.x; my = m.y; mz = m.z;
            float h = atan2f(my, mx) * (180.0f / 3.14159265f);
            if (h < 0.0f) h += 360.0f;
            hdg = h;
            mag_valid = true;
        }
    }

    /* --- Emit --- */
    char line[256];
    int n = snprintf(line, sizeof(line),
        "{\"t\":%lu,"
        "\"imu\":{\"accel\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f},"
                 "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}},"
        "\"mag\":{\"hdg\":%.1f,"
                 "\"field\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}},"
        "\"ok\":{\"imu\":%s,\"mag\":%s}}\r\n",
        (unsigned long)t_ms,
        (double)ax, (double)ay, (double)az,
        (double)gx, (double)gy, (double)gz,
        (double)hdg,
        (double)mx, (double)my, (double)mz,
        imu_valid ? "true" : "false",
        mag_valid ? "true" : "false");
    (void)n;

    /* Send to ESP32 */
    if (esp_uart_ok) {
        size_t len = strlen(line);
        cyhal_uart_write(&g_esp_uart, (void *)line, &len);
    }

    /* Mirror to debug UART (KitProg3 USB) */
    printf("%s", line);
}

/* ============================================================================
 * main
 * ============================================================================ */
int main(void)
{
    cy_rslt_t result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    __enable_irq();

    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                     CYBSP_DEBUG_UART_CTS, CYBSP_DEBUG_UART_RTS,
                                     CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    printf("\x1b[2J\x1b[;H");
    printf("SensorDuck — IMU + Magnetometer JSON @ %u ms\r\n", SEND_INTERVAL_MS);
    printf("CY8CKIT-062S2-AI\r\n\r\n");

    const cyhal_i2c_cfg_t i2c_cfg = {
        .is_slave        = false,
        .address         = 0,
        .frequencyhal_hz = SENSOR_I2C_FREQ,
    };
    result = cyhal_i2c_init(&g_i2c, SENSOR_I2C_SDA, SENSOR_I2C_SCL, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    cyhal_i2c_configure(&g_i2c, &i2c_cfg);

    bmi270_init();
    cyhal_system_delay_ms(10);
    bmm350_init();
    esp_uart_init();

    printf("\r\nStreaming...\r\n\r\n");

    uint32_t t_ms = 0;
    for (;;) {
        send_json(t_ms);
        cyhal_system_delay_ms(SEND_INTERVAL_MS);
        t_ms += SEND_INTERVAL_MS;
    }
}
