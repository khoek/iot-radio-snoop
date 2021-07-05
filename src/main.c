#include <cJSON.h>
#include <esp_log.h>
#include <libesp.h>
#include <libiot.h>
#include <rfm69hcw.h>

#include "../../secrets/secret.radio-snoop.h"
#include "lacrosse_crc.h"

#define DEVICE_NAME "radio-snoop"

#define MQTT_WARN_TOPIC IOT_MQTT_DEVICE_TOPIC(DEVICE_NAME, "log/warn")

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

static inline void convert_3b8_to_1b24(uint32_t* v, uint8_t o1, uint8_t o2, uint8_t o3) {
    *v = (((uint32_t) o1) << 16) | (((uint32_t) o2) << 8) | (((uint32_t) o3) << 0);
}

static inline void convert_2b8_to_1b16(uint16_t* v, uint8_t o1, uint8_t o2) {
    *v = (((uint32_t) o1) << 8) | (((uint32_t) o2) << 0);
}

static inline void convert_3b8_to_2b16(uint16_t* v1, uint16_t* v2, uint8_t o1, uint8_t o2, uint8_t o3) {
    *v1 = (((uint16_t) o1) << 4) | (((uint16_t) o2) >> 4);
    *v2 = (((uint16_t) o2 & 0x0F) << 8) | (((uint16_t) o3) >> 0);
}

// Only for use on the irq dispatch thread.
static char log_buff[256];

#define MIDDLE_BYTE_MAGIC 0xAA

static void check_magic_byte(uint8_t pos, uint8_t b) {
    if (b != 0xAA) {
        snprintf(log_buff, sizeof(log_buff), "magic byte (%d) is 0x%02X not 0x%02X!", pos, b, MIDDLE_BYTE_MAGIC);
        ESP_LOGW(TAG, "%s", log_buff);
        esp_mqtt_client_publish(mqtt_get_client(), MQTT_WARN_TOPIC, log_buff, strlen(log_buff), 2, 0);
    }
}

#define RAIN_MM_WARN_MAX (500.0)

static int32_t last_rain = -1;

static void handle_lacrosse_ltv_rv3_payload(const lacrosse_payload_t* payload, uint8_t rssi) {
    check_magic_byte(1, payload->b[1]);
    check_magic_byte(4, payload->b[4]);

    uint16_t rain_now;
    uint16_t rain_before;
    convert_2b8_to_1b16(&rain_now, payload->b[0], payload->b[2]);
    convert_2b8_to_1b16(&rain_before, payload->b[3], payload->b[5]);

    if (rain_before != last_rain && last_rain != -1) {
        snprintf(log_buff, sizeof(log_buff), "ltv_rv3: packet skipped! rain was 0x%02X, but last reported is 0x%02X", last_rain, rain_before);
        ESP_LOGW(TAG, "%s", log_buff);
        esp_mqtt_client_publish(mqtt_get_client(), MQTT_WARN_TOPIC, log_buff, strlen(log_buff), 2, 0);
    }

    last_rain = rain_now;

    // FIXME detect skips

    // Rain in 1/10ths of an inch.
    uint16_t rain_in = rain_now - rain_before;
    double delta_rain_mm = 0.254 * ((double) rain_in);

    if (delta_rain_mm >= RAIN_MM_WARN_MAX) {
        snprintf(log_buff, sizeof(log_buff), "ltv_rv3: too much rain! %lf (0x%X,0x%X)", delta_rain_mm, rain_now, rain_before);
        ESP_LOGW(TAG, "%s", log_buff);
        esp_mqtt_client_publish(mqtt_get_client(), MQTT_WARN_TOPIC, log_buff, strlen(log_buff), 2, 0);
    }

    ESP_LOGI(TAG, "ltv_rv3: delta_rain(mm)=%.1lf (rain_now=0x%03X, rain_before=0x%03X)", delta_rain_mm, rain_now, rain_before);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto handle_lacrosse_rv3_payload_out;
    }

    cJSON* json_delta_mm = cJSON_CreateNumber(delta_rain_mm);
    if (!json_delta_mm) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "delta_mm", json_delta_mm);

    cJSON* json_raw = cJSON_CreateObject();
    if (!json_raw) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "raw", json_raw);

    cJSON* json_rain_now = cJSON_CreateNumber(rain_now);
    if (!json_rain_now) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_raw, "rain_now", json_rain_now);

    cJSON* json_rain_before = cJSON_CreateNumber(rain_before);
    if (!json_rain_before) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_raw, "rain_before", json_rain_before);

    cJSON* json_rssi = cJSON_CreateNumber(((double) rssi) * (-0.5));
    if (!json_rssi) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rssi", json_rssi);

    char* output = cJSON_PrintUnformatted(json_root);
    if (!output) {
        goto handle_lacrosse_rv3_payload_out;
    }

    esp_mqtt_client_publish(mqtt_get_client(), IOT_MQTT_DEVICE_TOPIC(DEVICE_NAME, "rain"), output, strlen(output), 2, 0);

    free(output);
    cJSON_Delete(json_root);
    return;

handle_lacrosse_rv3_payload_out:
    ESP_LOGE(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
}

static void handle_lacrosse_ltv_wsdth04_payload(const lacrosse_payload_t* payload, uint8_t rssi) {
    uint16_t raw_temp;
    uint16_t raw_hum;
    convert_3b8_to_2b16(&raw_temp, &raw_hum, payload->b[0], payload->b[1], payload->b[2]);

    uint16_t raw_wind_speed;
    uint16_t raw_wind_dir;
    convert_3b8_to_2b16(&raw_wind_speed, &raw_wind_dir, payload->b[3], payload->b[4], payload->b[5]);

    // Temp in celsius
    double temp = ((double) (raw_temp - 400)) * 0.1;
    // Wind speed in kph
    double wind_speed = ((double) raw_wind_speed) * 0.1;

    ESP_LOGI(TAG, "ltv_wsdth04: temp(degC)=%.1lf, rel_hum=%d%%, wind_speed(kph)=%.1lf, wind_dir(deg)=%d", temp, raw_hum, wind_speed, raw_wind_dir);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto handle_lacrosse_breezepro_payload_out;
    }

    cJSON* json_temp_c = cJSON_CreateNumber(temp);
    if (!json_temp_c) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

    cJSON* json_rel_humidity = cJSON_CreateNumber(raw_hum);
    if (!json_rel_humidity) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rel_humidity", json_rel_humidity);

    cJSON* json_wind_speed = cJSON_CreateNumber(wind_speed);
    if (!json_wind_speed) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "wind_speed_kph", json_wind_speed);

    cJSON* json_wind_dir = cJSON_CreateNumber(raw_wind_dir);
    if (!json_wind_dir) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "wind_dir_deg", json_wind_dir);

    cJSON* json_rssi = cJSON_CreateNumber(((double) rssi) * (-0.5));
    if (!json_rssi) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rssi", json_rssi);

    char* output = cJSON_PrintUnformatted(json_root);
    if (!output) {
        goto handle_lacrosse_breezepro_payload_out;
    }

    esp_mqtt_client_publish(mqtt_get_client(), IOT_MQTT_DEVICE_TOPIC(DEVICE_NAME, "weather-station"), output, strlen(output), 2, 0);

    free(output);
    cJSON_Delete(json_root);
    return;

handle_lacrosse_breezepro_payload_out:
    ESP_LOGE(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
}

static void handle_lacrosse_packet(const lacrosse_packet_t* pkt, uint8_t rssi) {
    uint32_t id;
    convert_3b8_to_1b24(&id, pkt->data.id[0], pkt->data.id[1], pkt->data.id[2]);

    uint8_t seq_num = (pkt->data.status & MASK_LACROSSE_SEQ_NUM) >> SHIFT_LACROSSE_SEQ_NUM;
    uint8_t status = pkt->data.status & ~MASK_LACROSSE_SEQ_NUM;

    uint8_t true_crc8 = lacrosse_calc_crc8(pkt->raw, sizeof(pkt->raw));
    bool crc8_valid = pkt->crc8 == true_crc8;

    ESP_LOGI(TAG, "packet(rssi=0x%02X): id=0x%06X, seq=%d, status=0x%02X, crc=%s (0x%02X vs 0x%02X)",
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
            ESP_LOGW(TAG, "%s", log_buff);
            esp_mqtt_client_publish(mqtt_get_client(), MQTT_WARN_TOPIC, log_buff, strlen(log_buff), 2, 0);
            break;
        }
    }
}

static void handle_payload_ready(rfm69hcw_handle_t dev) {
    // Must read RSSI_VALUE before emptying the FIFO, since this will cause an RX restart
    // and we get a race.
    uint8_t rssi = rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE);

    uint8_t buff[sizeof(lacrosse_packet_t)];
    for (int i = 0; i < sizeof(lacrosse_packet_t); i++) {
        buff[i] = rfm69hcw_reg_read(dev, RFM69HCW_REG_FIFO);
    }

    handle_lacrosse_packet((lacrosse_packet_t*) buff, rssi);
}

#define MAX_WAIT_ITERS 100

static bool handle_irq(rfm69hcw_irq_task_handle_t task, rfm69hcw_handle_t dev) {
    int i;
    const char* fail_msg = NULL;

    for (i = 0; i < MAX_WAIT_ITERS; i++) {
        if (rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2) & RFM69HCW_IRQ_FLAGS_2_PAYLOAD_READY) {
            handle_payload_ready(dev);
            return true;
        }

        if (rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1) & RFM69HCW_IRQ_FLAGS_1_TIMEOUT) {
            break;
        }

        vTaskDelay(1);
    }

    if (i >= MAX_WAIT_ITERS) {
        fail_msg = "max iters exceeded while after RSSI sampling!";
        goto handle_irq_out;
    }

    uint8_t rssi = rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE);
    uint8_t afc_value_msb = rfm69hcw_reg_read(dev, RFM69HCW_REG_AFC_MSB);
    uint8_t afc_value_lsb = rfm69hcw_reg_read(dev, RFM69HCW_REG_AFC_LSB);
    uint16_t afc_value = (((uint16_t) afc_value_msb) << 8) | (((uint16_t) afc_value_lsb) << 0);

    uint8_t val = rfm69hcw_reg_read(dev, RFM69HCW_REG_PACKET_CONFIG_2);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_PACKET_CONFIG_2, val | RFM69HCW_PACKET_CONFIG_2_RESTART_RX);

    for (i = 0; i < MAX_WAIT_ITERS; i++) {
        if (!(rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1) & RFM69HCW_IRQ_FLAGS_1_TIMEOUT)) {
            break;
        }

        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "rssi timeout, restarting rx (rssi=0x%02X, afc_value=0x%04X)", rssi, afc_value);

    if (i >= MAX_WAIT_ITERS) {
        fail_msg = "max iters exceeded while after rx restart!";
        goto handle_irq_out;
    }

handle_irq_out:
    if (fail_msg) {
        snprintf(log_buff, sizeof(log_buff), "%s (0x%02X,0x%02X,0x%02X)", fail_msg, rfm69hcw_reg_read(dev, RFM69HCW_REG_OP_MODE), rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1), rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2));
        ESP_LOGW(TAG, "%s", log_buff);
        esp_mqtt_client_publish(mqtt_get_client(), MQTT_WARN_TOPIC, log_buff, strlen(log_buff), 2, 0);
    }

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

        .inter_packet_rx_delay = 6,  // ~6.85ms
        .timeout_rssi_thresh = 30,   // ~50ms
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