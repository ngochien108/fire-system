#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "ds18b20.h"

static const char *TAG = "DS18B20_SENSOR";
static portMUX_TYPE s_onewire_mux = portMUX_INITIALIZER_UNLOCKED;

static void ow_release(void)
{
    gpio_set_level(DS18B20_GPIO_PIN, 1);
}

void ow_output_low(void)
{
    gpio_set_level(DS18B20_GPIO_PIN, 0);
}

void ow_input(void)
{
    ow_release();
}

int ow_read(void)
{
    return gpio_get_level(DS18B20_GPIO_PIN);
}

void ds18b20_init(void)
{
    gpio_reset_pin(DS18B20_GPIO_PIN);
    gpio_set_direction(DS18B20_GPIO_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(DS18B20_GPIO_PIN, GPIO_PULLUP_ONLY);
    ow_release();

    if (ow_reset() == 0) {
        ESP_LOGI(TAG, "DS18B20 detected on GPIO %d", DS18B20_GPIO_PIN);
    } else {
        ESP_LOGW(TAG,
                 "No DS18B20 detected on GPIO %d. Check DQ pin, GND/VCC, and 4.7k pull-up to 3.3V.",
                 DS18B20_GPIO_PIN);
    }
}

static uint8_t ds18b20_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }

    return crc;
}

int ow_reset(void)
{
    taskENTER_CRITICAL(&s_onewire_mux);
    ow_output_low();
    esp_rom_delay_us(480);
    ow_release();
    esp_rom_delay_us(70);
    int presence = ow_read();
    esp_rom_delay_us(410);
    taskEXIT_CRITICAL(&s_onewire_mux);

    return presence == 0 ? 0 : -1;
}

void ow_write_bit(int bit)
{
    taskENTER_CRITICAL(&s_onewire_mux);
    ow_output_low();

    if (bit) {
        esp_rom_delay_us(6);
        ow_release();
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);
        ow_release();
        esp_rom_delay_us(10);
    }
    taskEXIT_CRITICAL(&s_onewire_mux);
}

int ow_read_bit(void)
{
    taskENTER_CRITICAL(&s_onewire_mux);
    ow_output_low();
    esp_rom_delay_us(3);
    ow_release();
    esp_rom_delay_us(12);
    int bit = ow_read();
    esp_rom_delay_us(55);
    taskEXIT_CRITICAL(&s_onewire_mux);

    return bit;
}

void ow_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit(byte & 1);
        byte >>= 1;
    }
}

uint8_t ow_read_byte(void)
{
    uint8_t byte = 0;

    for (int i = 0; i < 8; i++) {
        byte |= ow_read_bit() << i;
    }

    return byte;
}

float ds18b20_read_temp(void)
{
    if (ow_reset() != 0) {
        ESP_LOGW(TAG, "Read failed: no presence pulse on GPIO %d", DS18B20_GPIO_PIN);
        return DS18B20_INVALID_TEMP_C;
    }

    ow_write_byte(0xCC);
    ow_write_byte(0x44);
    vTaskDelay(pdMS_TO_TICKS(750));

    if (ow_reset() != 0) {
        ESP_LOGW(TAG, "Scratchpad read failed: no presence pulse on GPIO %d", DS18B20_GPIO_PIN);
        return DS18B20_INVALID_TEMP_C;
    }

    ow_write_byte(0xCC);
    ow_write_byte(0xBE);

    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
        data[i] = ow_read_byte();
    }

    if (ds18b20_crc8(data, 8) != data[8]) {
        ESP_LOGW(TAG,
                 "Scratchpad CRC failed: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 data[0],
                 data[1],
                 data[2],
                 data[3],
                 data[4],
                 data[5],
                 data[6],
                 data[7],
                 data[8]);
        return DS18B20_INVALID_TEMP_C;
    }

    int16_t raw = (data[1] << 8) | data[0];
    float temp = raw / 16.0f;
    ESP_LOGI(TAG, "Temp=%.2f", temp);

    return temp;
}
