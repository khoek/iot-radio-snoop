#ifndef RADIO_H
#define RADIO_H

#include <driver/gpio.h>
#include <driver/spi_master.h>

void radio_init(spi_host_device_t host, gpio_num_t pin_cs, gpio_num_t pin_rst, gpio_num_t pin_irq);
void radio_start();

#endif
