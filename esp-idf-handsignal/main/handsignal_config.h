/*
 * @Author: quenchkidney 1418273341@qq.com
 * @Date: 2025-07-31 17:44:13
 * @LastEditors: quenchkidney 1418273341@qq.com
 * @LastEditTime: 2025-08-01 01:02:23
 * @FilePath: \Arduino2IDFtest\esp-idf-handsignal\main\handsignal_config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef HANDSIGNAL_CONFIG_H
#define HANDSIGNAL_CONFIG_H

// Flex sensor GPIO pins
#define FLEX_PIN_1      GPIO_NUM_32
#define FLEX_PIN_2      GPIO_NUM_35  
#define FLEX_PIN_3      GPIO_NUM_34
#define FLEX_PIN_4      GPIO_NUM_39
#define FLEX_PIN_5      GPIO_NUM_36

// MPU6050 I2C configuration
#define I2C_MASTER_SCL_IO    GPIO_NUM_22
#define I2C_MASTER_SDA_IO    GPIO_NUM_21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000
#define MPU6050_ADDR         0x68
#define MPU6050_INT_PIN      GPIO_NUM_23

// UART configuration
#define UART_PORT_NUM        UART_NUM_0
#define UART_BAUD_RATE       115200
#define UART_BUF_SIZE        1024

// ADC configuration
#define ADC_UNIT             ADC_UNIT_1
#define ADC_ATTEN            ADC_ATTEN_DB_11
#define ADC_WIDTH            ADC_WIDTH_BIT_12

// Flex sensor configuration
#define FLEX_SENSOR_COUNT    5
#define FILTER_WINDOW_SIZE   5
#define DEFAULT_VREF         1100

// Data processing
#define DATA_BUFFER_SIZE     1000
#define SAMPLE_RATE_MS       10

#endif // HANDSIGNAL_CONFIG_H