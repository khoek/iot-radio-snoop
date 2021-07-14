
#include <esp_log.h>
#include <libesp.h>
#include <libiot.h>

#include "radio.h"
#include "secret.h"
#include "temp.h"

#define DEVICE_NAME "radio-snoop"

#define PIN_SPI_SCLK 5
#define PIN_SPI_MOSI 18
#define PIN_SPI_MISO 19

#define PIN_RFM69HCW_IRQ 15
#define PIN_RFM69HCW_CS 32
#define PIN_RFM69HCW_RST 14

#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 23

#define TMP117_I2C_ADDR 0x48

static const char* TAG = "app";

void app_init() {
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
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    radio_init(HSPI_HOST, PIN_RFM69HCW_CS, PIN_RFM69HCW_RST, PIN_RFM69HCW_IRQ);
    temp_init(I2C_NUM_0, TMP117_I2C_ADDR);
}

void app_run() {
    radio_start();
    temp_start();

    ESP_LOGI(TAG, "started");
}

static struct node_config CONFIG = {
    .name = DEVICE_NAME,
    .ssid = SECRET_WIFI_SSID,
    .pass = SECRET_WIFI_PASS,

    .uri = "mqtts://storagebox.local",
    .cert = SECRET_MQTT_CERT,
    .key = SECRET_MQTT_KEY,
    .mqtt_pass = SECRET_MQTT_PASS,
    .mqtt_cb = NULL,

    .app_init = &app_init,
    .app_run = &app_run,
};

void app_main() {
    libiot_startup(&CONFIG);
}
