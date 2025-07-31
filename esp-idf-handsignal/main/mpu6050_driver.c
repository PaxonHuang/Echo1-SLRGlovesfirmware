#include "mpu6050_driver.h"
#include "handsignal_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char* TAG = "MPU6050";

static bool dmp_ready = false;
static uint16_t packet_size = 0;

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t* data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init(void)
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wake up MPU6050
    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_test_connection(void)
{
    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_bytes(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (who_am_i == 0x68) {
        ESP_LOGI(TAG, "MPU6050 connection successful");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "MPU6050 connection failed, WHO_AM_I = 0x%02X", who_am_i);
        return ESP_FAIL;
    }
}

esp_err_t mpu6050_dmp_initialize(void)
{
    // Simplified DMP initialization - in a real implementation, 
    // you would need the full DMP firmware and initialization sequence
    ESP_LOGI(TAG, "DMP initialization (simplified)");
    
    // Basic configuration
    esp_err_t ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(MPU6050_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(MPU6050_SMPLRT_DIV, 0x04);
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(MPU6050_GYRO_CONFIG, 0x18);
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(MPU6050_ACCEL_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;
    
    packet_size = DMP_PACKET_SIZE;
    
    ESP_LOGI(TAG, "DMP initialized with packet size: %d", packet_size);
    return ESP_OK;
}

esp_err_t mpu6050_set_dmp_enabled(bool enabled)
{
    dmp_ready = enabled;
    ESP_LOGI(TAG, "DMP %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t mpu6050_get_motion_6(int16_t* ax, int16_t* ay, int16_t* az, 
                               int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    *gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    *gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    *gz = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return ESP_OK;
}

esp_err_t mpu6050_dmp_get_fifo_packet_size(uint16_t* size)
{
    *size = packet_size;
    return ESP_OK;
}

esp_err_t mpu6050_get_fifo_count(uint16_t* count)
{
    uint8_t buffer[2];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_FIFO_COUNTH, buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *count = (uint16_t)((buffer[0] << 8) | buffer[1]);
    return ESP_OK;
}

esp_err_t mpu6050_get_fifo_bytes(uint8_t* data, uint8_t length)
{
    return mpu6050_read_bytes(MPU6050_FIFO_R_W, data, length);
}

esp_err_t mpu6050_reset_fifo(void)
{
    esp_err_t ret = mpu6050_write_byte(MPU6050_USER_CTRL, 0x04);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}

esp_err_t mpu6050_get_int_status(uint8_t* status)
{
    return mpu6050_read_bytes(MPU6050_INT_STATUS, status, 1);
}

esp_err_t mpu6050_dmp_get_quaternion(quaternion_t* q, const uint8_t* packet)
{
    // Simplified quaternion extraction from DMP packet
    // In real implementation, this would parse the actual DMP packet format
    int32_t qI[4];
    
    qI[0] = ((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3];
    qI[1] = ((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7];
    qI[2] = ((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11];
    qI[3] = ((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15];
    
    q->w = (float)(qI[0] >> 16) / 16384.0f;
    q->x = (float)(qI[1] >> 16) / 16384.0f;
    q->y = (float)(qI[2] >> 16) / 16384.0f;
    q->z = (float)(qI[3] >> 16) / 16384.0f;
    
    return ESP_OK;
}

esp_err_t mpu6050_dmp_get_accel(vector_int16_t* v, const uint8_t* packet)
{
    v->x = (int16_t)((packet[28] << 8) | packet[29]);
    v->y = (int16_t)((packet[32] << 8) | packet[33]);
    v->z = (int16_t)((packet[36] << 8) | packet[37]);
    return ESP_OK;
}

esp_err_t mpu6050_dmp_get_gyro(vector_int16_t* v, const uint8_t* packet)
{
    v->x = (int16_t)((packet[16] << 8) | packet[17]);
    v->y = (int16_t)((packet[20] << 8) | packet[21]);
    v->z = (int16_t)((packet[24] << 8) | packet[25]);
    return ESP_OK;
}

esp_err_t mpu6050_dmp_get_gravity(vector_float_t* v, const quaternion_t* q)
{
    v->x = 2 * (q->x * q->z - q->w * q->y);
    v->y = 2 * (q->w * q->x + q->y * q->z);
    v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    return ESP_OK;
}

esp_err_t mpu6050_dmp_get_linear_accel(vector_int16_t* v, const vector_int16_t* v_raw, const vector_float_t* gravity)
{
    v->x = v_raw->x - (int16_t)(gravity->x * 8192);
    v->y = v_raw->y - (int16_t)(gravity->y * 8192);
    v->z = v_raw->z - (int16_t)(gravity->z * 8192);
    return ESP_OK;
}

esp_err_t mpu6050_dmp_get_linear_accel_in_world(vector_int16_t* v, const vector_int16_t* v_real, const quaternion_t* q)
{
    // Rotate linear acceleration vector by quaternion
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    float vx = v_real->x, vy = v_real->y, vz = v_real->z;
    
    v->x = (int16_t)(vx*(qw*qw + qx*qx - qy*qy - qz*qz) + vy*(2*qx*qy - 2*qw*qz) + vz*(2*qx*qz + 2*qw*qy));
    v->y = (int16_t)(vx*(2*qx*qy + 2*qw*qz) + vy*(qw*qw - qx*qx + qy*qy - qz*qz) + vz*(2*qy*qz - 2*qw*qx));
    v->z = (int16_t)(vx*(2*qx*qz - 2*qw*qy) + vy*(2*qy*qz + 2*qw*qx) + vz*(qw*qw - qx*qx - qy*qy + qz*qz));
    
    return ESP_OK;
}

esp_err_t mpu6050_set_offsets(int16_t xg_offset, int16_t yg_offset, int16_t zg_offset,
                              int16_t xa_offset, int16_t ya_offset, int16_t za_offset)
{
    // Set gyro offsets
    esp_err_t ret = mpu6050_write_byte(0x13, (uint8_t)(xg_offset >> 8));
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(0x14, (uint8_t)(xg_offset & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(0x15, (uint8_t)(yg_offset >> 8));
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(0x16, (uint8_t)(yg_offset & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(0x17, (uint8_t)(zg_offset >> 8));
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(0x18, (uint8_t)(zg_offset & 0xFF));
    if (ret != ESP_OK) return ret;
    
    // Set accel offsets
    ret = mpu6050_write_byte(0x06, (uint8_t)(xa_offset >> 8));
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(0x07, (uint8_t)(xa_offset & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(0x08, (uint8_t)(ya_offset >> 8));
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(0x09, (uint8_t)(ya_offset & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_write_byte(0x0A, (uint8_t)(za_offset >> 8));
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(0x0B, (uint8_t)(za_offset & 0xFF));
    
    ESP_LOGI(TAG, "MPU6050 offsets set successfully");
    return ret;
}