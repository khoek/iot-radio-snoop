#ifndef TEMP_H
#define TEMP_H

#include <driver/i2c.h>

void temp_init(i2c_port_t port, uint8_t addr);
void temp_start();

#endif
