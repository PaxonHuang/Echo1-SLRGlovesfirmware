<!--
 * @Author: quenchkidney 1418273341@qq.com
 * @Date: 2025-07-31 17:50:48
 * @LastEditors: quenchkidney 1418273341@qq.com
 * @LastEditTime: 2025-08-01 01:10:06
 * @FilePath: \Arduino2IDFtest\esp-idf-handsignal\README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# Hand Signal Recognition System - ESP-IDF Version

This project is an ESP-IDF implementation of a real-time hand gesture recognition system using ESP32, flex sensors, and MPU6050.


## Hardware Requirements

- DOIT ESP32 DevKit C
- 5x Flexible bend sensors (connected to ADC pins)
- MPU6050 6-axis IMU sensor
- Breadboard and connecting wires

## Pin Configuration

### Flex Sensors (ADC)
- Sensor 1: GPIO36 (ADC1_CH0)
- Sensor 2: GPIO39 (ADC1_CH3) 
- Sensor 3: GPIO34 (ADC1_CH6)
- Sensor 4: GPIO35 (ADC1_CH7)
- Sensor 5: GPIO32 (ADC1_CH4)

### MPU6050 (I2C)
- SDA: GPIO21
- SCL: GPIO22
- INT: GPIO23 (optional)

### Serial Communication
- TX: GPIO1 (UART0)
- RX: GPIO3 (UART0)
- Baud Rate: 115200

## Building and Flashing

1. Set up ESP-IDF environment:
   ```bash
   get_idf
   ```

2. Configure the project:
   ```bash
   idf.py menuconfig
   ```

3. Build the firmware:
   ```bash
   idf.py build
   ```

4. Flash to ESP32:
   ```bash
   idf.py -p COM_PORT flash monitor
   ```

## Data Output Format

The system outputs sensor data in the following format over UART:
```
FLEX:val1,val2,val3,val4,val5;ACCEL:x,y,z;GYRO:x,y,z;TEMP:temp;TIME:timestamp
```

This format is compatible with the PyQt desktop application for visualization and gesture recognition.

## Features

- Real-time sensor data collection at 100Hz
- Digital filtering (mean and median filters)
- Automatic sensor calibration
- Low-latency data transmission
- FreeRTOS task-based architecture
- Error handling and logging

## Project Structure

```
├── main/
│   ├── main.c                 # Main application
│   ├── flex_sensor.c/h        # Flex sensor driver
│   ├── mpu6050_driver.c/h     # MPU6050 driver
│   └── handsignal_config.h    # Configuration constants
├── CMakeLists.txt             # Main CMake file
├── sdkconfig.defaults         # Default configuration
└── README.md                  # This file
```

## Compatibility

This ESP-IDF version maintains full compatibility with the original Arduino firmware and PyQt desktop application.