#include <cJSON.h>
#include <device/tmp117.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <libiot.h>
#include <libsensor.h>

#include "../sensor.h"

static const char* TAG = "sensor(tmp117)";

static void dev_start(tmp117_handle_t dev) {
    // First place the TMP117 into shutdown mode to stop it converting, and clear
    // the EEPROM_BUSY flag by reading the TEMP_RESULT.
    tmp117_reg_write(dev, TMP117_REG_CONFIGURATION, TMP117_CONFIGURATION_MOD_SD);
    tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT);

    // Next place the TMP117 into continuous conversion mode with 64 samples averaged
    // in order to obtain each final value. A value will be reported roughly every
    // one second.
    tmp117_reg_write(dev, TMP117_REG_CONFIGURATION, TMP117_CONFIGURATION_MOD_CC | TMP117_CONFIGURATION_AVG_64);
}

static poll_result_t poll(const char* _tag, tmp117_handle_t dev, QueueHandle_t queue) {
    if (!(tmp117_reg_read(dev, TMP117_REG_CONFIGURATION) & TMP117_CONFIGURATION_DATA_READY)) {
        return POLL_RESULT_UNEVENTFUL;
    }

    int16_t result = (int16_t) tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT);

    if (xQueueSend(queue, &result, 0) != pdTRUE) {
        libiot_logf_error(TAG, "can't queue result");
        return POLL_RESULT_FAIL;
    }

    return POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, tmp117_handle_t dev, int16_t* result) {
    double temp_c = ((double) *result) * 0.0078125;

    ESP_LOGI(TAG, "(%-10s) temp(oC)=%.7lf", tag, temp_c);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto report_out;
    }

    cJSON* json_temp_c = cJSON_CreateNumber(temp_c);
    if (!json_temp_c) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

    return json_root;

report_out:
    ESP_LOGE(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

sensor_type_t SENSOR_TMP117 = {
    .name = "tmp117",

    .queue_item_size = sizeof(int16_t),
    .poll_delay_ms = 250,
    .max_uneventful_iters = 10,

    .dev_start = (sensor_dev_fn_t) dev_start,
    .dev_reset = (sensor_dev_fn_t) tmp117_reset,
    .dev_destroy = (sensor_dev_fn_t) tmp117_destroy,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};
