#include "driver/i2c_master.h"

// used for ESP32-C6
static gpio_num_t i2c_gpio_sda = 6;
static gpio_num_t i2c_gpio_scl = 7;
// used for ESP32
// static gpio_num_t i2c_gpio_sda = 21;
// static gpio_num_t i2c_gpio_scl = 22;

// Station 1 - Greenhouse
// Station 2 - Garden
#define STATION_ID 2