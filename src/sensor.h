#ifndef SENSOR_H
#define SENSOR_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <libsensor.h>

void sensor_rfm69hcw_init(spi_host_device_t host, gpio_num_t pin_cs, gpio_num_t pin_rst, gpio_num_t pin_irq);

extern sensor_type_t SENSOR_TMP117;
extern sensor_type_t SENSOR_DPS368;
extern sensor_type_t SENSOR_CCS811;

#endif
