// ds18b20.h
#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>

#include "driver/gpio.h"

#define DS18B20_GPIO_PIN   GPIO_NUM_32
#define DS18B20_INVALID_TEMP_C (-99.9f)


void ds18b20_init(void);
void ow_output_low(void);
void ow_input(void);
int ow_read(void);

int ow_reset(void);
void ow_write_bit(int bit);
int ow_read_bit(void);
void ow_write_byte(uint8_t b);
uint8_t ow_read_byte(void);
float ds18b20_read_temp(void);

#endif // DS18B20_H
