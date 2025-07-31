#include "flex_sensor.h"
#include "handsignal_config.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "FLEX_SENSOR";
static esp_adc_cal_characteristics_t *adc_chars;

// ADC channels for 5 flex sensors (A0-A4 pins)
static const adc1_channel_t flex_channels[FLEX_SENSOR_COUNT] = {
    ADC1_CHANNEL_0,  // GPIO36 (A0)
    ADC1_CHANNEL_3,  // GPIO39 (A3)
    ADC1_CHANNEL_6,  // GPIO34 (A2)
    ADC1_CHANNEL_7,  // GPIO35 (A1)
    ADC1_CHANNEL_4   // GPIO32 (A4)
};

// Calibration parameters (adjust these based on your sensors)
static float calibration_min[FLEX_SENSOR_COUNT] = {200, 200, 200, 200, 200};
static float calibration_max[FLEX_SENSOR_COUNT] = {4000, 4000, 4000, 4000, 4000};

// Filter buffers for mean and median filtering
static float filter_buffer[FLEX_SENSOR_COUNT][FILTER_WINDOW_SIZE];
static int filter_index = 0;
static bool filter_initialized = false;

esp_err_t flex_sensors_init(void)
{
    ESP_LOGI(TAG, "Initializing flex sensors");
    
    // Configure ADC1
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure each flex sensor channel
    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        adc1_config_channel_atten(flex_channels[i], ADC_ATTEN_DB_11);
    }
    
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref");
    }
    
    // Initialize filter buffers
    memset(filter_buffer, 0, sizeof(filter_buffer));
    filter_index = 0;
    filter_initialized = false;
    
    ESP_LOGI(TAG, "Flex sensors initialized successfully");
    return ESP_OK;
}

esp_err_t flex_sensors_read_all(int* values, size_t count)
{
    if (count != FLEX_SENSOR_COUNT) {
        ESP_LOGE(TAG, "Invalid count: expected %d, got %zu", FLEX_SENSOR_COUNT, count);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw ADC values
    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        int raw_value = adc1_get_raw(flex_channels[i]);
        if (raw_value < 0) {
            ESP_LOGE(TAG, "Failed to read ADC channel %d", i);
            return ESP_FAIL;
        }
        
        // Convert to voltage (mV)
        uint32_t voltage = esp_adc_cal_raw_to_voltage(raw_value, adc_chars);
        
        // Apply calibration and scaling
        float normalized = (float)(voltage - calibration_min[i]) / (calibration_max[i] - calibration_min[i]);
        normalized = (normalized < 0.0f) ? 0.0f : (normalized > 1.0f) ? 1.0f : normalized;
        
        // Scale to 0-1000 range for compatibility with Arduino code
        values[i] = (int)(normalized * 1000.0f);
        
        // Store in filter buffer
        filter_buffer[i][filter_index] = (float)values[i];
    }
    
    // Update filter index
    filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;
    if (!filter_initialized && filter_index == 0) {
        filter_initialized = true;
    }
    
    return ESP_OK;
}

esp_err_t flex_sensors_calibrate(void)
{
    ESP_LOGI(TAG, "Starting flex sensor calibration");
    
    int min_values[FLEX_SENSOR_COUNT];
    int max_values[FLEX_SENSOR_COUNT];
    
    // Initialize with first reading
    int temp_values[FLEX_SENSOR_COUNT];
    esp_err_t ret = flex_sensors_read_all(temp_values, FLEX_SENSOR_COUNT);
    if (ret != ESP_OK) {
        return ret;
    }
    
    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        min_values[i] = max_values[i] = temp_values[i];
    }
    
    // Collect samples for calibration
    const int calibration_samples = 100;
    ESP_LOGI(TAG, "Collecting %d samples for calibration...", calibration_samples);
    
    for (int sample = 0; sample < calibration_samples; sample++) {
        ret = flex_sensors_read_all(temp_values, FLEX_SENSOR_COUNT);
        if (ret != ESP_OK) {
            return ret;
        }
        
        for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
            if (temp_values[i] < min_values[i]) {
                min_values[i] = temp_values[i];
            }
            if (temp_values[i] > max_values[i]) {
                max_values[i] = temp_values[i];
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay between samples
    }
    
    // Update calibration parameters
    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        calibration_min[i] = (float)min_values[i];
        calibration_max[i] = (float)max_values[i];
        ESP_LOGI(TAG, "Sensor %d: min=%f, max=%f", i, calibration_min[i], calibration_max[i]);
    }
    
    ESP_LOGI(TAG, "Flex sensor calibration completed");
    return ESP_OK;
}

void flex_sensors_apply_mean_filter(float* filtered_values, size_t count)
{
    if (count != FLEX_SENSOR_COUNT || !filter_initialized) {
        return;
    }
    
    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        float sum = 0.0f;
        for (int j = 0; j < FILTER_WINDOW_SIZE; j++) {
            sum += filter_buffer[i][j];
        }
        filtered_values[i] = sum / FILTER_WINDOW_SIZE;
    }
}

static int compare_float(const void *a, const void *b)
{
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

void flex_sensors_apply_median_filter(float* filtered_values, size_t count)
{
    if (count != FLEX_SENSOR_COUNT || !filter_initialized) {
        return;
    }
    
    float temp_buffer[FILTER_WINDOW_SIZE];
    
    for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
        // Copy values to temporary buffer
        memcpy(temp_buffer, filter_buffer[i], sizeof(temp_buffer));
        
        // Sort the values
        qsort(temp_buffer, FILTER_WINDOW_SIZE, sizeof(float), compare_float);
        
        // Get median value
        filtered_values[i] = temp_buffer[FILTER_WINDOW_SIZE / 2];
    }
}