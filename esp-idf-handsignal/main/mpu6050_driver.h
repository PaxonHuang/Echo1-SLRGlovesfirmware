#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "esp_err.h"
#include "driver/i2c.h"

// MPU6050 register addresses
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_USER_CTRL       0x6A
#define MPU6050_FIFO_COUNTH     0x72
#define MPU6050_FIFO_R_W        0x74

// DMP related
#define MPU6050_DMP_CODE_SIZE   1929
#define DMP_PACKET_SIZE         42

// Data structures
typedef struct {
    float w, x, y, z;
} quaternion_t;

typedef struct {
    int16_t x, y, z;
} vector_int16_t;

typedef struct {
    float x, y, z;
} vector_float_t;

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    quaternion_t q;
    vector_int16_t aa;
    vector_int16_t gyr;
    vector_int16_t aa_real;
    vector_int16_t aa_world;
    vector_float_t gravity;
    float euler[3];
    float ypr[3];
} mpu6050_data_t;

// Function declarations
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_test_connection(void);
esp_err_t mpu6050_dmp_initialize(void);
esp_err_t mpu6050_set_dmp_enabled(bool enabled);
esp_err_t mpu6050_get_motion_6(int16_t* ax, int16_t* ay, int16_t* az, 
                               int16_t* gx, int16_t* gy, int16_t* gz);
esp_err_t mpu6050_dmp_get_fifo_packet_size(uint16_t* size);
esp_err_t mpu6050_get_fifo_count(uint16_t* count);
esp_err_t mpu6050_get_fifo_bytes(uint8_t* data, uint8_t length);
esp_err_t mpu6050_reset_fifo(void);
esp_err_t mpu6050_get_int_status(uint8_t* status);
esp_err_t mpu6050_dmp_get_quaternion(quaternion_t* q, const uint8_t* packet);
esp_err_t mpu6050_dmp_get_accel(vector_int16_t* v, const uint8_t* packet);
esp_err_t mpu6050_dmp_get_gyro(vector_int16_t* v, const uint8_t* packet);
esp_err_t mpu6050_dmp_get_gravity(vector_float_t* v, const quaternion_t* q);
esp_err_t mpu6050_dmp_get_linear_accel(vector_int16_t* v, const vector_int16_t* v_raw, const vector_float_t* gravity);
esp_err_t mpu6050_dmp_get_linear_accel_in_world(vector_int16_t* v, const vector_int16_t* v_real, const quaternion_t* q);
esp_err_t mpu6050_set_offsets(int16_t xg_offset, int16_t yg_offset, int16_t zg_offset,
                              int16_t xa_offset, int16_t ya_offset, int16_t za_offset);

// I2C functions
esp_err_t i2c_master_init(void);
esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t* data, size_t len);
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data);

#endif // MPU6050_DRIVER_H