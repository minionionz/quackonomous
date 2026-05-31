/*******************************************************************************
 * main.c — SensorDuck: UART telemetry + MQTT publish
 *
 * Two FreeRTOS tasks:
 *   sensor_task  reads BMI270 + BMM350 at 10 Hz, sends JSON on UART to ESP32,
 *                and pushes a sensor_data_t into the MQTT queue.
 *   mqtt_task    connects to WiFi + MQTT broker, then publishes every field
 *                from the queue as individual topics:
 *
 *     duck/sensor/t              ms uptime
 *     duck/sensor/imu/accel/{x,y,z}   mg
 *     duck/sensor/imu/gyro/{x,y,z}    dps
 *     duck/sensor/mag/hdg             degrees (0–360)
 *     duck/sensor/mag/field/{x,y,z}   µT
 *     duck/sensor/ok/{imu,mag}        0 / 1
 *
 * WiFi credentials and broker address → mqtt_config.h
 *
 * Target: CY8CKIT-062S2-AI (PSoC 6 + CYW43439)
 * Build:  make getlibs && make program
 *******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* WiFi + MQTT */
#include "cy_wcm.h"
#include "cy_mqtt_api.h"

#include "bmm350_iface.h"
#include "mqtt_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * Configuration
 * ============================================================================ */
#define SENSOR_I2C_SCL    CYBSP_I2C_SCL
#define SENSOR_I2C_SDA    CYBSP_I2C_SDA
#define SENSOR_I2C_FREQ   400000U

#define ESP_UART_TX       P9_1
#define ESP_UART_RX       P9_0
#define ESP_UART_BAUD     115200U

#define SEND_INTERVAL_MS  100U

/* ============================================================================
 * BMI270 register map  (I2C 0x68)
 * ============================================================================ */
#define BMI270_ADDR             0x68U
#define BMI270_REG_CHIP_ID      0x00U
#define BMI270_CHIP_ID_VAL      0x24U
#define BMI270_REG_INTERNAL_STS 0x21U
#define BMI270_REG_ACC_X_LSB    0x0CU
#define BMI270_REG_GYR_X_LSB    0x12U
#define BMI270_REG_ACCEL_CONF   0x40U
#define BMI270_REG_ACCEL_RANGE  0x41U
#define BMI270_REG_GYR_CONF     0x42U
#define BMI270_REG_GYR_RANGE    0x43U
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
 * Shared sensor data
 * ============================================================================ */
typedef struct {
    uint32_t t_ms;
    float    ax, ay, az;   /* mg  */
    float    gx, gy, gz;   /* dps */
    float    hdg;           /* degrees 0-360 */
    float    mx, my, mz;   /* µT  */
    bool     imu_valid;
    bool     mag_valid;
} sensor_data_t;

static QueueHandle_t g_data_q;   /* depth 1, sensor_task overwrites */

/* ============================================================================
 * HAL objects (used exclusively by sensor_task)
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
    vTaskDelay(pdMS_TO_TICKS(1));

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
    vTaskDelay(pdMS_TO_TICKS(150));
}
#endif

static void bmi270_init(void)
{
    uint8_t id = 0;
    i2c_write_reg(BMI270_ADDR, BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (i2c_read_regs(BMI270_ADDR, BMI270_REG_CHIP_ID, &id, 1) != CY_RSLT_SUCCESS
            || id != BMI270_CHIP_ID_VAL) {
        printf("[IMU] BMI270 init failed (id=0x%02X)\r\n", id);
        return;
    }
    i2c_write_reg(BMI270_ADDR, BMI270_REG_PWR_CONF, 0x00U);
    vTaskDelay(pdMS_TO_TICKS(1));

#ifdef BMI270_LOAD_CONFIG_BLOB
    printf("[IMU] loading config blob (%u bytes)...\r\n",
           (unsigned)bmi270_config_size);
    bmi270_load_config_blob();
#endif

    i2c_write_reg(BMI270_ADDR, BMI270_REG_PWR_CTRL,     0x0EU);
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_write_reg(BMI270_ADDR, BMI270_REG_ACCEL_CONF,   0xA8U);
    i2c_write_reg(BMI270_ADDR, BMI270_REG_ACCEL_RANGE,  0x03U);
    i2c_write_reg(BMI270_ADDR, BMI270_REG_GYR_CONF,     0xA8U);
    i2c_write_reg(BMI270_ADDR, BMI270_REG_GYR_RANGE,    0x00U);
    vTaskDelay(pdMS_TO_TICKS(50));

    bmi270_ok = true;
    printf("[IMU] BMI270 OK (id=0x%02X)\r\n", id);
}

/* ============================================================================
 * BMM350 init
 * ============================================================================ */
static void bmm350_init(void)
{
    g_bmm350.intfPtr = &g_i2c;
    g_bmm350.read    = bmm350_hal_read;
    g_bmm350.write   = bmm350_hal_write;
    g_bmm350.delayUs = bmm350_hal_delay_us;

    if (bmm350Init(&g_bmm350) != BMM350_OK) {
        printf("[MAG] BMM350 init failed\r\n");
        return;
    }
    if (bmm350SetOdrPerformance(BMM350_DATA_RATE_25HZ,
                                BMM350_LOWNOISE, &g_bmm350) != BMM350_OK) {
        printf("[MAG] BMM350 ODR config failed\r\n");
        return;
    }
    if (bmm350SetPowerMode(eBmm350NormalMode, &g_bmm350) != BMM350_OK) {
        printf("[MAG] BMM350 power-mode failed\r\n");
        return;
    }
    bmm350_ok = true;
    printf("[MAG] BMM350 OK (id=0x%02X)\r\n", g_bmm350.chipId);
}

/* ============================================================================
 * ESP32 UART init
 * ============================================================================ */
static void esp_uart_init(void)
{
    const cyhal_uart_cfg_t cfg = {
        .data_bits = 8, .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL, .rx_buffer_size = 0,
    };
    uint32_t actual_baud = 0;

    if (cyhal_uart_init(&g_esp_uart, ESP_UART_TX, ESP_UART_RX,
                        NC, NC, NULL, &cfg) != CY_RSLT_SUCCESS) {
        printf("[UART] ESP32 UART init failed\r\n");
        return;
    }
    if (cyhal_uart_set_baud(&g_esp_uart, ESP_UART_BAUD,
                            &actual_baud) != CY_RSLT_SUCCESS) {
        printf("[UART] ESP32 baud set failed\r\n");
        cyhal_uart_free(&g_esp_uart);
        return;
    }
    esp_uart_ok = true;
    printf("[UART] ESP32 UART OK (%lu baud)\r\n", (unsigned long)actual_baud);
}

/* ============================================================================
 * MQTT helpers
 * ============================================================================ */
static void mqtt_pub(cy_mqtt_t h, const char *topic, const char *payload)
{
    cy_mqtt_publish_info_t pub = {
        .qos        = CY_MQTT_QOS0,
        .topic      = topic,
        .topic_len  = (uint16_t)strlen(topic),
        .retain     = false,
        .payload     = payload,
        .payload_len = (uint32_t)strlen(payload),
    };
    cy_rslt_t rc = cy_mqtt_publish(h, &pub);
    if (rc != CY_RSLT_SUCCESS)
        printf("[MQTT] publish failed 0x%lX  topic=%s\r\n",
               (unsigned long)rc, topic);
}

static void mqtt_pub_f(cy_mqtt_t h, const char *topic, float v,
                       const char *fmt)
{
    char buf[24];
    snprintf(buf, sizeof(buf), fmt, (double)v);
    mqtt_pub(h, topic, buf);
}

static void mqtt_event_cb(cy_mqtt_t handle, cy_mqtt_event_t event,
                          void *user_data)
{
    (void)handle; (void)user_data;
    if (event.type == CY_MQTT_EVENT_TYPE_DISCONNECT)
        printf("[MQTT] disconnected (reason=%d)\r\n",
               (int)event.data.reason);
}

static void publish_sensor(cy_mqtt_t h, const sensor_data_t *d)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)d->t_ms);
    mqtt_pub(h, MQTT_TOPIC_ROOT "/t", buf);

    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/imu/accel/x", d->ax, "%.1f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/imu/accel/y", d->ay, "%.1f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/imu/accel/z", d->az, "%.1f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/imu/gyro/x",  d->gx, "%.2f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/imu/gyro/y",  d->gy, "%.2f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/imu/gyro/z",  d->gz, "%.2f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/mag/hdg",      d->hdg, "%.1f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/mag/field/x",  d->mx, "%.1f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/mag/field/y",  d->my, "%.1f");
    mqtt_pub_f(h, MQTT_TOPIC_ROOT "/mag/field/z",  d->mz, "%.1f");
    mqtt_pub(h, MQTT_TOPIC_ROOT "/ok/imu", d->imu_valid ? "1" : "0");
    mqtt_pub(h, MQTT_TOPIC_ROOT "/ok/mag", d->mag_valid ? "1" : "0");
}

/* ============================================================================
 * sensor_task — reads sensors, sends JSON over UART, feeds MQTT queue
 * ============================================================================ */
static void sensor_task(void *arg)
{
    (void)arg;

    const cyhal_i2c_cfg_t i2c_cfg = {
        .is_slave = false, .address = 0,
        .frequencyhal_hz = SENSOR_I2C_FREQ,
    };
    CY_ASSERT(cyhal_i2c_init(&g_i2c, SENSOR_I2C_SDA,
                              SENSOR_I2C_SCL, NULL) == CY_RSLT_SUCCESS);
    cyhal_i2c_configure(&g_i2c, &i2c_cfg);

    bmi270_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    bmm350_init();
    esp_uart_init();

    printf("\r\nStreaming...\r\n\r\n");

    uint32_t   t_ms = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        sensor_data_t d = { .t_ms = t_ms };

        /* ── IMU ── */
        if (bmi270_ok) {
            uint8_t raw[6];
            if (i2c_read_regs(BMI270_ADDR, BMI270_REG_ACC_X_LSB,
                              raw, 6) == CY_RSLT_SUCCESS) {
                d.ax = (float)(int16_t)((uint16_t)raw[1] << 8 | raw[0]) * 0.488f;
                d.ay = (float)(int16_t)((uint16_t)raw[3] << 8 | raw[2]) * 0.488f;
                d.az = (float)(int16_t)((uint16_t)raw[5] << 8 | raw[4]) * 0.488f;
            }
            if (i2c_read_regs(BMI270_ADDR, BMI270_REG_GYR_X_LSB,
                              raw, 6) == CY_RSLT_SUCCESS) {
                d.gx = (float)(int16_t)((uint16_t)raw[1] << 8 | raw[0]) / 16.384f;
                d.gy = (float)(int16_t)((uint16_t)raw[3] << 8 | raw[2]) / 16.384f;
                d.gz = (float)(int16_t)((uint16_t)raw[5] << 8 | raw[4]) / 16.384f;
                d.imu_valid = true;
            }
        }

        /* ── Magnetometer ── */
        if (bmm350_ok) {
            struct sBmm350MagTempData_t m;
            if (bmm350GetCompensatedMagXYZTempData(&m, &g_bmm350) == BMM350_OK) {
                d.mx = m.x; d.my = m.y; d.mz = m.z;
                float h = atan2f(m.y, m.x) * (180.0f / 3.14159265f);
                if (h < 0.0f) h += 360.0f;
                d.hdg      = h;
                d.mag_valid = true;
            }
        }

        /* ── JSON → ESP32 UART ── */
        char line[256];
        snprintf(line, sizeof(line),
            "{\"t\":%lu,"
            "\"imu\":{\"accel\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f},"
                     "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}},"
            "\"mag\":{\"hdg\":%.1f,"
                     "\"field\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}},"
            "\"ok\":{\"imu\":%s,\"mag\":%s}}\r\n",
            (unsigned long)t_ms,
            (double)d.ax, (double)d.ay, (double)d.az,
            (double)d.gx, (double)d.gy, (double)d.gz,
            (double)d.hdg,
            (double)d.mx, (double)d.my, (double)d.mz,
            d.imu_valid ? "true" : "false",
            d.mag_valid ? "true" : "false");

        if (esp_uart_ok) {
            size_t len = strlen(line);
            cyhal_uart_write(&g_esp_uart, (void *)line, &len);
        }
        printf("%s", line);

        /* ── Hand off to MQTT task (non-blocking overwrite) ── */
        xQueueOverwrite(g_data_q, &d);

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SEND_INTERVAL_MS));
        t_ms += SEND_INTERVAL_MS;
    }
}

/* ============================================================================
 * mqtt_task — WiFi connect → MQTT connect → publish loop
 * ============================================================================ */
static void mqtt_task(void *arg)
{
    (void)arg;

    /* ── WiFi ── */
    cy_wcm_config_t wcm_cfg = { .interface = CY_WCM_INTERFACE_TYPE_STA };
    CY_ASSERT(cy_wcm_init(&wcm_cfg) == CY_RSLT_SUCCESS);

    cy_wcm_connect_params_t conn_params = {};
    strncpy((char *)conn_params.ap_credentials.SSID,
            WIFI_SSID, sizeof(conn_params.ap_credentials.SSID) - 1);
    strncpy((char *)conn_params.ap_credentials.password,
            WIFI_PASSWORD, sizeof(conn_params.ap_credentials.password) - 1);
    conn_params.ap_credentials.security = WIFI_SECURITY;

    cy_wcm_ip_address_t ip;
    cy_rslt_t rc;
    do {
        printf("[WiFi] connecting to \"%s\"...\r\n", WIFI_SSID);
        rc = cy_wcm_connect_ap(&conn_params, &ip);
        if (rc != CY_RSLT_SUCCESS) {
            printf("[WiFi] failed (0x%08lX), retry in 3 s\r\n",
                   (unsigned long)rc);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    } while (rc != CY_RSLT_SUCCESS);

    printf("[WiFi] connected  IP %lu.%lu.%lu.%lu\r\n",
           (unsigned long)( ip.ip.v4        & 0xFF),
           (unsigned long)((ip.ip.v4 >>  8) & 0xFF),
           (unsigned long)((ip.ip.v4 >> 16) & 0xFF),
           (unsigned long)((ip.ip.v4 >> 24) & 0xFF));

    /* ── MQTT ── */
    CY_ASSERT(cy_mqtt_init() == CY_RSLT_SUCCESS);

    cy_mqtt_broker_info_t broker = {
        .hostname = MQTT_BROKER,
        .port     = MQTT_PORT,
    };
    static uint8_t mqtt_buf[4096];
    cy_mqtt_t mqh;
    CY_ASSERT(cy_mqtt_create(mqtt_buf, sizeof(mqtt_buf),
                              NULL,        /* no TLS */
                              &broker,
                              NULL,        /* descriptor */
                              &mqh) == CY_RSLT_SUCCESS);
    CY_ASSERT(cy_mqtt_register_event_callback(mqh, mqtt_event_cb, NULL) == CY_RSLT_SUCCESS);

    cy_mqtt_connect_info_t ci = {
        .client_id     = MQTT_CLIENT_ID,
        .client_id_len = sizeof(MQTT_CLIENT_ID) - 1,
        .clean_session = true,
        .keep_alive_sec = 60,
        .will_info     = NULL,
        .username      = NULL, .username_len = 0,
        .password      = NULL, .password_len = 0,
    };
    CY_ASSERT(cy_mqtt_connect(mqh, &ci) == CY_RSLT_SUCCESS);
    printf("[MQTT] connected to %s:%u\r\n", MQTT_BROKER, MQTT_PORT);

    /* ── Publish loop ── */
    sensor_data_t d;
    for (;;) {
        if (xQueueReceive(g_data_q, &d, portMAX_DELAY) == pdTRUE)
            publish_sensor(mqh, &d);
    }
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
    printf("SensorDuck — UART + MQTT @ %u ms\r\n\r\n", SEND_INTERVAL_MS);

    g_data_q = xQueueCreate(1, sizeof(sensor_data_t));
    CY_ASSERT(g_data_q != NULL);

    xTaskCreate(sensor_task, "Sensor", 4096, NULL, 2, NULL);
    xTaskCreate(mqtt_task,   "MQTT",   4096, NULL, 1, NULL);

    vTaskStartScheduler();
    CY_ASSERT(0);   /* never reached */
    return 0;
}
