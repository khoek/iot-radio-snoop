#include <cJSON.h>
#include <esp_log.h>
#include <libesp.h>
#include <libiot.h>
#include <rfm69hcw.h>

#include "../../secrets/secret.radio-snoop.h"
#include "lacrosse_crc.h"

#define DEVICE_NAME "radio-snoop"

#define PIN_SPI_SCLK 5
#define PIN_SPI_MOSI 18
#define PIN_SPI_MISO 19

#define PIN_RFM69HCW_CS 14
#define PIN_RFM69HCW_IRQ 32
#define PIN_RFM69HCW_RST 22

#define TASK_STACK_SIZE 4096

static const char* TAG = "app";

typedef struct lacrosse_payload {
    uint8_t b[6];
} lacrosse_payload_t;

typedef struct lacrosse_packet {
    union {
        struct {
            uint8_t id[3];
            uint8_t status;
            lacrosse_payload_t payload;
        } data;
        uint8_t raw[10];
    };
    uint8_t crc8;
    uint8_t trailer[4];
} __attribute__((packed)) lacrosse_packet_t;

#define MASK_LACROSSE_ID_TYPE 0x00FFF000
#define MASK_LACROSSE_SEQ_NUM 0x0E
#define SHIFT_LACROSSE_ID_TYPE 12
#define SHIFT_LACROSSE_SEQ_NUM 1

#define LACROSSE_ID_TYPE_LTV_RV3 0x70F
#define LACROSSE_ID_TYPE_LTV_WSDTH04 0x88F

static inline void bit24_to_bit32(uint32_t* v, uint8_t o1, uint8_t o2, uint8_t o3) {
    *v = (((uint32_t) o1) << 16) | (((uint32_t) o2) << 8) | (((uint32_t) o3) << 0);
}

static inline void bit24_to_2bit16(uint16_t* v1, uint16_t* v2, uint8_t o1, uint8_t o2, uint8_t o3) {
    *v1 = (((uint16_t) o1) << 4) | (((uint16_t) o2) >> 4);
    *v2 = (((uint16_t) o2 & 0x0F) << 8) | (((uint16_t) o3) >> 0);
}

#define RAIN_MM_MAX (500.0)

static char log_buff[256];

static void handle_lacrosse_ltv_rv3_payload(const lacrosse_payload_t* payload, uint8_t rssi) {
    uint32_t rain1;
    uint32_t rain2;
    bit24_to_bit32(&rain1, payload->b[0], payload->b[1], payload->b[2]);
    bit24_to_bit32(&rain2, payload->b[3], payload->b[4], payload->b[5]);

    // FIXME detect skips

    // Rain in 1/10ths of an inch.
    uint16_t rain_in = rain1 - rain2;
    double delta_rain_mm = 0.254 * ((double) rain_in);

    if (delta_rain_mm >= RAIN_MM_MAX) {
        snprintf(log_buff, sizeof(log_buff), "too much rain! %lf (0x%X,0x%X)", delta_rain_mm, rain1, rain2);
        esp_mqtt_client_publish(mqtt_get_client(), IOT_MQTT_LOG_NOTICE, log_buff, strlen(log_buff), 2, 0);
    }

    ESP_LOGI(TAG, "ltv_rv3: delta_rain(mm)=%.1lf (rain1=0x%03X, rain2=0x%03X)", delta_rain_mm, rain1, rain2);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        ESP_LOGE(TAG, "%s: cannot create JSON root object", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }

    cJSON* json_delta_mm = cJSON_CreateNumber(delta_rain_mm);
    if (!json_delta_mm) {
        ESP_LOGE(TAG, "%s: cannot create JSON delta_mm number", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "delta_mm", json_delta_mm);

    cJSON* json_raw = cJSON_CreateObject();
    if (!json_raw) {
        ESP_LOGE(TAG, "%s: cannot create JSON raw object", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "raw", json_raw);

    cJSON* json_rain1 = cJSON_CreateNumber(rain1);
    if (!json_rain1) {
        ESP_LOGE(TAG, "%s: cannot create JSON rain1 number", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_raw, "rain1", json_rain1);

    cJSON* json_rain2 = cJSON_CreateNumber(rain2);
    if (!json_rain2) {
        ESP_LOGE(TAG, "%s: cannot create JSON rain2 number", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_raw, "rain2", json_rain2);

    cJSON* json_rssi = cJSON_CreateNumber(((double) rssi) * (-0.5));
    if (!json_rssi) {
        ESP_LOGE(TAG, "%s: cannot create JSON rssi number", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rssi", json_rssi);

    char* output = cJSON_PrintUnformatted(json_root);
    if (!output) {
        ESP_LOGE(TAG, "%s: cannot print JSON output", __func__);
        goto handle_lacrosse_rv3_payload_out;
    }

    esp_mqtt_client_publish(mqtt_get_client(), IOT_MQTT_DEVICE_TOPIC(DEVICE_NAME, "rain"), output, strlen(output), 2, 0);

    free(output);

handle_lacrosse_rv3_payload_out:
    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
}

static void handle_lacrosse_ltv_wsdth04_payload(const lacrosse_payload_t* payload, uint8_t rssi) {
    uint16_t raw_temp;
    uint16_t raw_hum;
    bit24_to_2bit16(&raw_temp, &raw_hum, payload->b[0], payload->b[1], payload->b[2]);

    uint16_t raw_wind_speed;
    uint16_t raw_wind_dir;
    bit24_to_2bit16(&raw_wind_speed, &raw_wind_dir, payload->b[3], payload->b[4], payload->b[5]);

    // Temp in celsius
    double temp = ((double) (raw_temp - 400)) * 0.1;
    // Wind speed in kph
    double wind_speed = ((double) raw_wind_speed) * 0.1;

    ESP_LOGI(TAG, "ltv_wsdth04: temp(degC)=%.1lf, rel_hum=%d%%, wind_speed(kph)=%.1lf wind_dir(deg)=%d", temp, raw_hum, wind_speed, raw_wind_dir);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        ESP_LOGE(TAG, "%s: cannot create JSON root object", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }

    cJSON* json_temp_c = cJSON_CreateNumber(temp);
    if (!json_temp_c) {
        ESP_LOGE(TAG, "%s: cannot create JSON temp_c number", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

    cJSON* json_rel_humidity = cJSON_CreateNumber(raw_hum);
    if (!json_rel_humidity) {
        ESP_LOGE(TAG, "%s: cannot create JSON rel_humidity number", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rel_humidity", json_rel_humidity);

    cJSON* json_wind_speed = cJSON_CreateNumber(wind_speed);
    if (!json_wind_speed) {
        ESP_LOGE(TAG, "%s: cannot create JSON wind_speed number", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "wind_speed_kph", json_wind_speed);

    cJSON* json_wind_dir = cJSON_CreateNumber(raw_wind_dir);
    if (!json_wind_dir) {
        ESP_LOGE(TAG, "%s: cannot create JSON wind_dir number", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "wind_dir_deg", json_wind_dir);

    cJSON* json_rssi = cJSON_CreateNumber(((double) rssi) * (-0.5));
    if (!json_rssi) {
        ESP_LOGE(TAG, "%s: cannot create JSON rssi number", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rssi", json_rssi);

    char* output = cJSON_PrintUnformatted(json_root);
    if (!output) {
        ESP_LOGE(TAG, "%s: cannot print JSON output", __func__);
        goto handle_lacrosse_breezepro_payload_out;
    }

    esp_mqtt_client_publish(mqtt_get_client(), IOT_MQTT_DEVICE_TOPIC(DEVICE_NAME, "weather-station"), output, strlen(output), 2, 0);

    free(output);

handle_lacrosse_breezepro_payload_out:
    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
}

static void handle_lacrosse_packet(const lacrosse_packet_t* pkt, uint8_t rssi) {
    uint32_t id;
    bit24_to_bit32(&id, pkt->data.id[0], pkt->data.id[1], pkt->data.id[2]);

    uint8_t seq_num = (pkt->data.status & MASK_LACROSSE_SEQ_NUM) >> SHIFT_LACROSSE_SEQ_NUM;
    uint8_t status = pkt->data.status & ~MASK_LACROSSE_SEQ_NUM;

    uint8_t true_crc8 = lacrosse_calc_crc8(pkt->raw, sizeof(pkt->raw));
    bool crc8_valid = pkt->crc8 == true_crc8;

    ESP_LOGI(TAG, "packet(rssi=0x%02X): id=0x%06X seq=%d status=0x%02X crc=%s (0x%02X vs 0x%02X)",
             rssi, id, seq_num, status, crc8_valid ? "OK" : "BAD", pkt->crc8, true_crc8);

    if (!crc8_valid) {
        ESP_LOGW(TAG, "bad crc!");
        return;
    }

    switch ((id & MASK_LACROSSE_ID_TYPE) >> SHIFT_LACROSSE_ID_TYPE) {
        case LACROSSE_ID_TYPE_LTV_RV3: {
            handle_lacrosse_ltv_rv3_payload(&pkt->data.payload, rssi);
            break;
        }
        case LACROSSE_ID_TYPE_LTV_WSDTH04: {
            handle_lacrosse_ltv_wsdth04_payload(&pkt->data.payload, rssi);
            break;
        }
        default: {
            snprintf(log_buff, sizeof(log_buff), "well-formed packet with unknown id type: 0x%X", id);
            esp_mqtt_client_publish(mqtt_get_client(), IOT_MQTT_LOG_NOTICE, log_buff, strlen(log_buff), 2, 0);
            break;
        }
    }
}

static bool handle_irq(rfm69hcw_irq_task_handle_t task, rfm69hcw_handle_t dev) {
    // Must read RSSI_VALUE before emptying the FIFO, since this will cause an RX restart
    // and we get a race.
    uint8_t rssi = rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE);

    uint8_t buff[sizeof(lacrosse_packet_t)];
    for (int i = 0; i < sizeof(lacrosse_packet_t); i++) {
        buff[i] = rfm69hcw_reg_read(dev, RFM69HCW_REG_FIFO);
    }

    handle_lacrosse_packet((lacrosse_packet_t*) buff, rssi);

    return true;
}

static rfm69hcw_handle_t dev;

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

    // Configure the ESP32 to communicate with the RFM69HCW on "HSPI".
    rfm69hcw_init(HSPI_HOST, PIN_RFM69HCW_CS, PIN_RFM69HCW_RST, PIN_RFM69HCW_IRQ, &dev);
}

void app_run() {
    rfm69hcw_reset(dev);

    rfm69hcw_rx_config_t cfg = {
        .data_mode = RFM69HCW_DATA_MODE_PACKET_MODE,
        .type = RFM69HCW_MODULATION_TYPE_FSK,
        .fsk_shaping = RFM69HCW_MODULATION_SHAPING_FSK_NONE,

        .freq_khz = 915000,
        .bit_period_ns = 106842,
        .rx_bw = RFM69HCW_RX_BW_100_kHz,
        .dcc_cutoff = RFM69HCW_DCC_CUTOFF_8_PERCENT,

        .sync_bit_tol = 1,
        .sync_value = {0xD2, 0xAA, 0x2D, 0xD4, 0x00},

        .payload_len = sizeof(lacrosse_packet_t),
    };

    rfm69hcw_irq_task_handle_t task;
    ESP_ERROR_CHECK(rfm69hcw_enter_rx(dev, &cfg, &handle_irq, 1, TASK_STACK_SIZE, &task));

    // TODO consider measuring FEI
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