#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "handsignal_config.h"
#include "flex_sensor.h"
#include "mpu6050_driver.h"

static const char *TAG = "HANDSIGNAL_MAIN";

// Data structure for sensor readings
//为传感器读取数据创建一个结构体
typedef struct {
    int flex_values[FLEX_SENSOR_COUNT];
    float mpu_accel[3];
    float mpu_gyro[3];
    float mpu_temp;
    uint32_t timestamp;
} sensor_data_t;

// Task handles
//任务句柄
static TaskHandle_t data_collection_task_handle = NULL;
static TaskHandle_t data_transmission_task_handle = NULL;

// Queue for sensor data
//传感器数据队列
static QueueHandle_t sensor_data_queue = NULL;

// Function to initialize UART for serial communication
//初始化UART进行串行通信
static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized");
}

// Data collection task
//数据采集任务
static void data_collection_task(void *pvParameters)
{
    sensor_data_t sensor_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_RATE_MS);
    
    ESP_LOGI(TAG, "Data collection task started");
    
    while (1) {
        // Read flex sensors
        //读取弯曲传感器
        if (flex_sensors_read_all(sensor_data.flex_values, FLEX_SENSOR_COUNT) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read flex sensors");
            continue;
        }
        
        // Read MPU6050
        //读取MPU6050传感器
        if (mpu6050_read_accel(sensor_data.mpu_accel) != ESP_OK ||
            mpu6050_read_gyro(sensor_data.mpu_gyro) != ESP_OK ||
            mpu6050_read_temp(&sensor_data.mpu_temp) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MPU6050");
            continue;
        }
        
        // Add timestamp
        //添加时间戳
        sensor_data.timestamp = esp_log_timestamp();
        
        // Send data to queue
        //将数据发送到队列
        if (xQueueSend(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send data to queue");
        }
        
        // Wait for next sampling period
        //等待下一个采样周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Data transmission task
//数据传输任务
static void data_transmission_task(void *pvParameters)
{
    sensor_data_t sensor_data;
    char tx_buffer[512];
    
    ESP_LOGI(TAG, "Data transmission task started");
    
    while (1) {
        // Wait for sensor data
        //等待传感器数据
        if (xQueueReceive(sensor_data_queue, &sensor_data, portMAX_DELAY)) {
            // Format data according to PyQt expected format (Arduino compatible)
            //根据PyQt预期格式格式化数据(Arduino兼容格式)
            // Original Arduino format: 30 comma-separated values
            //原Arduino格式: 30个逗号分隔的值
            // flex1,flex2,flex3,flex4,flex5,qw,qx,qy,qz,gyrx,gyry,gyrz,accx,accy,accz,aaRealx,aaRealy,aaRealz,aaWorldx,aaWorldy,aaWorldz,gravx,gravy,gravz,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw
            
            // Use default quaternion (1,0,0,0) and calculate missing values for compatibility
            //使用默认四元数(1,0,0,0)并计算缺失值以保持兼容性
            float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
            float gravity_x = 0.0f, gravity_y = 0.0f, gravity_z = 9.8f;
            
            int len = snprintf(tx_buffer, sizeof(tx_buffer),
                "%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d\n",
                sensor_data.flex_values[0], sensor_data.flex_values[1], sensor_data.flex_values[2], sensor_data.flex_values[3], sensor_data.flex_values[4],
                qw, qx, qy, qz,
                (int)(sensor_data.mpu_gyro[0]*100), (int)(sensor_data.mpu_gyro[1]*100), (int)(sensor_data.mpu_gyro[2]*100),
                (int)(sensor_data.mpu_accel[0]*100), (int)(sensor_data.mpu_accel[1]*100), (int)(sensor_data.mpu_accel[2]*100),
                (int)(sensor_data.mpu_accel[0]*100), (int)(sensor_data.mpu_accel[1]*100), (int)(sensor_data.mpu_accel[2]*100),
                (int)(sensor_data.mpu_accel[0]*100), (int)(sensor_data.mpu_accel[1]*100), (int)(sensor_data.mpu_accel[2]*100),
                gravity_x, gravity_y, gravity_z,
                (int)(sensor_data.mpu_accel[0]*1000), (int)(sensor_data.mpu_accel[1]*1000), (int)(sensor_data.mpu_accel[2]*1000),
                (int)(sensor_data.mpu_gyro[0]*1000), (int)(sensor_data.mpu_gyro[1]*1000), (int)(sensor_data.mpu_gyro[2]*1000));
            
            if (len > 0 && len < sizeof(tx_buffer)) {
                uart_write_bytes(UART_PORT_NUM, tx_buffer, len);
            } else {
                ESP_LOGE(TAG, "Buffer overflow in data formatting");
            }
        }
    }
}

// System initialization
//系统初始化
static void system_init(void)
{
    ESP_LOGI(TAG, "Initializing hand signal recognition system");
    
    // Initialize UART
    //初始化UART
    uart_init();
    
    // Initialize flex sensors
    //初始化弯曲传感器
    ESP_ERROR_CHECK(flex_sensors_init());
    
    // Initialize MPU6050
    //初始化MPU6050传感器
    ESP_ERROR_CHECK(mpu6050_init());
    
    // Create sensor data queue
    //创建传感器数据队列
    sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_t));
    if (sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        return;
    }
    
    ESP_LOGI(TAG, "System initialization completed");
}

// Application main function
//应用程序主函数
void app_main(void)
{
    ESP_LOGI(TAG, "Hand Signal Recognition System Starting...");
    
    // Initialize system
    //初始化系统
    system_init();
    
    // Create tasks
    //创建任务
    xTaskCreate(data_collection_task, "data_collection", 4096, NULL, 5, &data_collection_task_handle);
    xTaskCreate(data_transmission_task, "data_transmission", 4096, NULL, 4, &data_transmission_task_handle);
    
    ESP_LOGI(TAG, "Tasks created successfully");
    
    // Main loop - monitor system status
    //主循环 - 监控系统状态
    while (1) {
        // Print system status every 10 seconds
        //每10秒打印一次系统状态
        ESP_LOGI(TAG, "System running - Free heap: %d bytes", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}