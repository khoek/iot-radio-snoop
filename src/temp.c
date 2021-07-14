#include "temp.h"

#include <cJSON.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <libiot.h>
#include <tmp117.h>

static const char* TAG = "temp";

#define QUEUE_LENGTH 16

static StaticQueue_t result_queue_static;
static uint8_t result_queue_buff[QUEUE_LENGTH * sizeof(int16_t)];
static QueueHandle_t result_queue;

static tmp117_handle_t dev;

static void report_result(int16_t result) {
    double temp_c = ((double) result) * 0.0078125;

    ESP_LOGI(TAG, "tmp117: temp(oC)=%.7lf", temp_c);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto report_result_out;
    }

    cJSON* json_temp_c = cJSON_CreateNumber(temp_c);
    if (!json_temp_c) {
        goto report_result_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

    char* output = cJSON_PrintUnformatted(json_root);
    if (!output) {
        goto report_result_out;
    }

    libiot_mqtt_publish_local("wired-temp", 2, 0, output);

    free(output);
    cJSON_Delete(json_root);
    return;

report_result_out:
    ESP_LOGE(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
}

void temp_init(i2c_port_t port, uint8_t addr) {
    result_queue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(int16_t), result_queue_buff, &result_queue_static);

    // Configure the ESP32 to communicate with the TMP117 on `port`.
    ESP_ERROR_CHECK(tmp117_init(port, addr, &dev));

    /* FIXME show dad
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_CONFIGURATION));
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_THIGH_LIMIT));
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_TLOW_LIMIT));

    ESP_LOGE(TAG, "");

    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT));
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_TEMP_OFFSET));

    ESP_LOGE(TAG, "");

    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_EEPROM_UL));
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_EEPROM1));
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_EEPROM2));
    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_EEPROM3));

    ESP_LOGE(TAG, "");

    ESP_LOGE(TAG, "XXX 0x%04X", tmp117_reg_read(dev, TMP117_REG_DEVICE_ID));
    */
}

void task_temp_report(void* unused) {
    while (1) {
        int16_t result;
        while (xQueueReceive(result_queue, &result, portMAX_DELAY) == pdFALSE)
            ;

        report_result(result);
    }
}

#define TEMP_POLL_TASK_DELAY_MS 250

void task_temp_poll(void* unused) {
    // First place the TMP117 into shutdown mode to stop it converting, and clear
    // the EEPROM_BUSY flag by reading the TEMP_RESULT.
    tmp117_reg_write(dev, TMP117_REG_CONFIGURATION, TMP117_CONFIGURATION_MOD_SD);
    tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT);

    // Next place the TMP117 into continuous conversion mode with 64 samples averaged
    // in order to obtain each final value. A value will be reported roughtly every
    // one second.
    tmp117_reg_write(dev, TMP117_REG_CONFIGURATION, TMP117_CONFIGURATION_MOD_CC | TMP117_CONFIGURATION_AVG_64);

    // Given this, so long as `TEMP_POLL_TASK_DELAY_MS` is quite a bit less than a
    // 1/2 second we won't miss any samples if we poll the DATA_READY bit and just
    // read the currently reported result straight after.
    while (1) {
        if (tmp117_reg_read(dev, TMP117_REG_CONFIGURATION) & TMP117_CONFIGURATION_DATA_READY) {
            int16_t result = (int16_t) tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT);

            if (xQueueSend(result_queue, &result, 0) != pdTRUE) {
                libiot_logf_error(TAG, "can't queue result");
            }
        }

        vTaskDelay(TEMP_POLL_TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

#define TEMP_REPORT_TASK_STACK_SIZE 4096
#define TEMP_POLL_TASK_STACK_SIZE 2048

void temp_start() {
    BaseType_t result;

    result = xTaskCreate(&task_temp_report, "task_temp_report", TEMP_REPORT_TASK_STACK_SIZE, NULL, 5, NULL);
    if (result != pdPASS) {
        libiot_logf_error(TAG, "failed to create temp_report task! (0x%X)", result);
    }

    result = xTaskCreate(&task_temp_poll, "task_temp_poll", TEMP_POLL_TASK_STACK_SIZE, NULL, 10, NULL);
    if (result != pdPASS) {
        libiot_logf_error(TAG, "failed to create temp_poll task! (0x%X)", result);
    }
}
