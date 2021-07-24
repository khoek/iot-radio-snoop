#include <cJSON.h>
#include <device/ccs811.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <libiot.h>
#include <libsensor.h>

#include "../sensor.h"

static const char* TAG = "sensor(ccs811)";

#define MAX_READY_ITERS

static void dev_start(ccs811_handle_t dev) {
    ccs811_start_app(dev);

    const uint8_t reg_meas_mode = CCS811_MEAS_MODE_DRIVE_MODE_1;
    ccs811_reg_write(dev, CCS811_REG_MEAS_MODE, 1, &reg_meas_mode);
}

typedef struct measurement {
    uint16_t eco2_ppm;
    uint16_t etvoc_ppb;
    uint16_t raw_data;
} measurement_t;

static poll_result_t poll(const char* tag, ccs811_handle_t dev, QueueHandle_t queue) {
    uint8_t reg_status;
    ccs811_reg_read(dev, CCS811_REG_STATUS, 1, &reg_status);
    if (!(reg_status & CCS811_STATUS_DATA_READY)) {
        return POLL_RESULT_UNEVENTFUL;
    }

    uint8_t error_id;
    measurement_t meas;
    ccs811_read_alg_result_data(dev, &meas.eco2_ppm, &meas.etvoc_ppb, NULL, &error_id, &meas.raw_data);

    if (error_id) {
        libiot_logf_error(TAG, "(%s) error! (0x%02X)", tag, error_id);
        return POLL_RESULT_FAIL;
    }

    if (xQueueSend(queue, &meas, 0) != pdTRUE) {
        libiot_logf_error(TAG, "can't queue result");
        return POLL_RESULT_FAIL;
    }

    return POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, ccs811_handle_t dev, measurement_t* meas) {
    ESP_LOGI(TAG, "(%-10s) eCO2(ppm)=%u, eTVOC(ppb)=%u, raw_data=0x%04X", tag, meas->eco2_ppm, meas->etvoc_ppb, meas->raw_data);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto report_out;
    }

    cJSON* json_eco2_ppm = cJSON_CreateNumber(meas->eco2_ppm);
    if (!json_eco2_ppm) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "eco2_ppm", json_eco2_ppm);

    cJSON* json_etvoc_ppb = cJSON_CreateNumber(meas->etvoc_ppb);
    if (!json_etvoc_ppb) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "etvoc_ppb", json_etvoc_ppb);

    return json_root;

report_out:
    ESP_LOGE(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

sensor_type_t SENSOR_CCS811 = {
    .name = "ccs811",

    .queue_item_size = sizeof(measurement_t),
    .poll_delay_ms = 250,
    .max_uneventful_iters = 40,

    .dev_start = (sensor_dev_fn_t) dev_start,
    .dev_reset = (sensor_dev_fn_t) ccs811_reset,
    .dev_destroy = (sensor_dev_fn_t) ccs811_destroy,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};
