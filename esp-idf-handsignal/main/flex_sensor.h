#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H

#include "esp_err.h"
#include "driver/adc.h"

typedef struct {
    adc1_channel_t channel;
    int raw_value;
    float calibrated_value;
} flex_sensor_t;

// Function declarations
esp_err_t flex_sensors_init(void);
esp_err_t flex_sensors_read_all(int* values, size_t count);
esp_err_t flex_sensors_calibrate(void);
void flex_sensors_apply_mean_filter(float* filtered_values, size_t count);
void flex_sensors_apply_median_filter(float* filtered_values, size_t count);

#endif // FLEX_SENSOR_H