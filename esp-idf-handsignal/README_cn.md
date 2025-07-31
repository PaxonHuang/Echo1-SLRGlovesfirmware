# ESP-IDF 手势识别系统

## 项目概述

本项目是基于ESP-IDF框架的实时手势识别系统，使用ESP32开发板、弯曲传感器和MPU6050传感器实现手势数据采集。系统通过FreeRTOS任务管理，实现高频率传感器数据采集、滤波处理和实时传输，可与PyQt桌面应用程序配合使用进行手势识别和可视化。

## 前置条件

### 硬件要求
- DOIT ESP32 DevKit C 开发板
- 5个柔性弯曲传感器（连接到ADC引脚）
- MPU6050 6轴IMU传感器
- 面包板和连接线

### 软件要求
- ESP-IDF开发环境（v4.4或更高版本）
- Python 3.x（用于PyQt桌面应用）
- 串口调试工具（如PuTTY或Arduino IDE串口监视器）

## 引脚配置

### 弯曲传感器（ADC）
- 传感器1: GPIO36 (ADC1_CH0)
- 传感器2: GPIO39 (ADC1_CH3) 
- 传感器3: GPIO34 (ADC1_CH6)
- 传感器4: GPIO35 (ADC1_CH7)
- 传感器5: GPIO32 (ADC1_CH4)

### MPU6050（I2C）
- SDA: GPIO21
- SCL: GPIO22
- INT: GPIO23（可选）

### 串口通信
- TX: GPIO1 (UART0)
- RX: GPIO3 (UART0)
- 波特率: 115200

## 运行方法

1. **硬件连接**
   - 按照引脚配置连接所有传感器
   - 确保电源供应稳定（建议使用外部5V电源）

2. **数据接收**
   - 使用串口工具连接ESP32（波特率115200）
   - 或运行配套的PyQt桌面应用程序进行实时可视化

3. **数据格式**
   ```
   FLEX:val1,val2,val3,val4,val5;ACCEL:x,y,z;GYRO:x,y,z;TEMP:temp;TIME:timestamp
   ```

## 构建方法

1. **设置ESP-IDF环境**
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

2. **配置项目**
   ```bash
   cd esp-idf-handsignal
   idf.py menuconfig
   ```

3. **编译固件**
   ```bash
   idf.py build
   ```

4. **烧录到ESP32**
   ```bash
   idf.py -p COM_PORT flash monitor
   ```
   （将COM_PORT替换为实际的串口名称）

## 项目特性

- **实时数据采集**: 100Hz高频率传感器数据采集
- **数字滤波**: 内置均值和中值滤波算法
- **自动校准**: 开机自动进行传感器校准
- **低延迟传输**: 优化的数据传输机制
- **任务管理**: 基于FreeRTOS的多任务架构
- **错误处理**: 完善的错误检测和日志记录
- **兼容性**: 与原Arduino版本和PyQt应用完全兼容

## 项目结构

```
├── main/
│   ├── main.c                 # 主应用程序
│   ├── flex_sensor.c/h        # 弯曲传感器驱动
│   ├── mpu6050_driver.c/h     # MPU6050驱动
│   └── handsignal_config.h    # 配置常量
├── CMakeLists.txt             # 主CMake文件
├── sdkconfig.defaults         # 默认配置
└── README.md                  # 说明文件
```

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 注意事项

1. 首次使用时，请确保传感器校准完成（系统会自动进行）
2. 弯曲传感器需要正确安装在手部关节位置
3. MPU6050传感器应固定在手背以获得最佳效果
4. 长时间使用时注意监控传感器温度，避免过热

## 技术支持

如有问题或建议，请通过以下方式联系：
- 提交Issue到项目仓库
- 发送邮件至：1418273341@qq.com