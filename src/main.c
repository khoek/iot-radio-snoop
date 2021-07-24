#include <device/ccs811.h>
#include <device/dps368.h>
#include <device/rfm69hcw.h>
#include <device/tmp117.h>
#include <esp_log.h>
#include <libiot.h>

#include "secret.h"
#include "sensor.h"

#define DEVICE_NAME "radio-snoop"

#define PIN_SPI_SCLK 5
#define PIN_SPI_MOSI 18
#define PIN_SPI_MISO 19

#define PIN_RFM69HCW_IRQ 15
#define PIN_RFM69HCW_CS 32
#define PIN_RFM69HCW_RST 14

#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 23

#define TMP117_I2C_ADDR1 0x48
#define TMP117_I2C_ADDR2 0x49
#define DPS368_I2C_ADDR 0x77
#define CCS811_I2C_ADDR 0x5A

static const char* TAG = "app";

void app_run() {
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
    // FIXME hack? (13.1071875ms @80MHz, note ccs811 datasheet says max clock stretch is 100ms!)
    // FIXME are there still timeout problems with the ccs811?
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFFF));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    // FIXME FIXME FIXME create "libisr" to have very lightweight messages passed from
    // a GPIO isr, so this fits into the sensor framework (and then change the driver around)
    sensor_rfm69hcw_init(HSPI_HOST, PIN_RFM69HCW_CS, PIN_RFM69HCW_RST, PIN_RFM69HCW_IRQ);

    {
        tmp117_handle_t dev;

        // ESP_ERROR_CHECK(tmp117_init(I2C_NUM_0, TMP117_I2C_ADDR1, &dev));
        // ESP_ERROR_CHECK(libsensor_create(&SENSOR_TMP117, "wired-1", dev));
        ESP_ERROR_CHECK(tmp117_init(I2C_NUM_0, TMP117_I2C_ADDR2, &dev));
        ESP_ERROR_CHECK(libsensor_create(&SENSOR_TMP117, "wired-2", dev));
    }

    {
        dps368_handle_t dev;

        ESP_ERROR_CHECK(dps368_init(I2C_NUM_0, DPS368_I2C_ADDR, &dev));
        ESP_ERROR_CHECK(libsensor_create(&SENSOR_DPS368, "wired", dev));
    }

    {
        ccs811_handle_t dev;

        ESP_ERROR_CHECK(ccs811_init(I2C_NUM_0, CCS811_I2C_ADDR, &dev));
        ESP_ERROR_CHECK(libsensor_create(&SENSOR_CCS811, "wired", dev));
    }

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
    .mqtt_cb = NULL,

    .app_run = app_run,
};

void app_main() {
    libiot_startup(&CONFIG);
}
