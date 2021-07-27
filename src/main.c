#include <esp_log.h>
#include <libi2c.h>
#include <libiot.h>
#include <libsensor-drivers.h>

#include "secret.h"

#define DEVICE_NAME "radio-snoop"

#define PIN_SPI_SCLK 5
#define PIN_SPI_MOSI 18
#define PIN_SPI_MISO 19

#define PIN_RFM69HCW_IRQ 15
#define PIN_RFM69HCW_CS 32
#define PIN_RFM69HCW_RST 14

#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 23

// FIXME move all of this to the libsensor-drivers lib
// Coming...
#define AS7341_I2C_ADDR 0x39
#define SI1145_I2C_ADDR 0x60

static const char* TAG = "app";

static void mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    if (event->event_id != MQTT_EVENT_DATA) {
        return;
    }

    libsensor_dispatch_mqtt_message(event->topic, event->topic_len, event->data, event->data_len);
}

static void app_init() {
    libsensor_init();
}

static void app_run() {
    // Configure the "HSPI" SPI peripheral.
    spi_bus_config_t buscfg = {
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MISO,

        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCLK,

        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

        .max_transfer_sz = SPI_MAX_DMA_LEN,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 1));

    // Configure the "port 0" I2C peripheral.
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    // NOTE: The CCS811 uses clock stretching, neccesitating the use of this (max) timeout.
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFFF));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    i2c_7bit_general_call_reset(I2C_NUM_0);

    // Register all of the sensor drivers for this device.
    libsensor_drv_register_rfm69hcw_lacrosse(HSPI_HOST, PIN_RFM69HCW_CS, PIN_RFM69HCW_RST, PIN_RFM69HCW_IRQ, "1");
    // libsensor_drv_register_tmp117(I2C_NUM_0, TMP117_I2C_ADDR_GND, "1");
    libsensor_drv_register_tmp117(I2C_NUM_0, TMP117_I2C_ADDR_VCC, "2");
    // libsensor_drv_register_dps368(I2C_NUM_0, DPS368_I2C_ADDR_HIGH, "1");
    libsensor_drv_register_ccs811(I2C_NUM_0, CCS811_I2C_ADDR_LOW, "1", true);
    libsensor_drv_register_scd41(I2C_NUM_0, SCD41_I2C_ADDR, "1", true);
    libsensor_drv_register_bme280(I2C_NUM_0, BME280_I2C_ADDR_HIGH, "1");

    ESP_LOGI(TAG, "started");
}

static node_config_t CONFIG = {
    .name = DEVICE_NAME,
    .ssid = SECRET_WIFI_SSID,
    .pass = SECRET_WIFI_PASS,

    .uri = SECRET_MQTT_URI,
    .cert = SECRET_MQTT_CERT,
    .key = SECRET_MQTT_KEY,
    .mqtt_pass = SECRET_MQTT_PASS,
    .mqtt_cb = mqtt_event_handler_cb,

    .app_init = app_init,
    .app_run = app_run,
};

void app_main() {
    libiot_startup(&CONFIG);
}
