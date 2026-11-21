#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <sensor_msgs/msg/imu.h>

class MPU6050 {
private:
    uint8_t addr = 0x68;
    uint sda_pin;
    uint scl_pin;
    i2c_inst_t *i2c_port;

    // Conversion factors
    const float G_TO_MS2 = 9.80665f;
    const float DEG_TO_RAD = 0.0174533f;

    // ---> FIX: Added the missing offset array here <---
    float gyro_offset[3] = {0.0, 0.0, 0.0}; 

public:
    MPU6050(i2c_inst_t *port, uint sda, uint scl) 
        : i2c_port(port), sda_pin(sda), scl_pin(scl) {}

    void init() {
        i2c_init(i2c_port, 400 * 1000);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);

        // Wake up MPU6050
        uint8_t wakeup_data[2] = {0x6B, 0x00};
        i2c_write_blocking(i2c_port, addr, wakeup_data, 2, false);
    }

    // Call this in main() AFTER init() but BEFORE the micro-ROS loop
    void calibrate() {
        long g_sum[3] = {0, 0, 0};
        const int samples = 1000;
        uint8_t dummy_reg = 0x3B;
        uint8_t dummy_buf[14];

        // Discard first 100 readings
        for (int i = 0; i < 100; i++) {
            i2c_write_blocking(i2c_port, addr, &dummy_reg, 1, true);
            i2c_read_blocking(i2c_port, addr, dummy_buf, 14, false);
            sleep_ms(2);
        }

        // Take 1000 samples to find the Gyroscope zero-bias offset
        for (int i = 0; i < samples; i++) {
            i2c_write_blocking(i2c_port, addr, &dummy_reg, 1, true);
            i2c_read_blocking(i2c_port, addr, dummy_buf, 14, false);
            
            g_sum[0] += (int16_t)((dummy_buf[8] << 8) | dummy_buf[9]);
            g_sum[1] += (int16_t)((dummy_buf[10] << 8) | dummy_buf[11]);
            g_sum[2] += (int16_t)((dummy_buf[12] << 8) | dummy_buf[13]);
            sleep_ms(2);
        }

        // Only calculate offsets for the Gyroscope
        gyro_offset[0] = (float)g_sum[0] / samples;
        gyro_offset[1] = (float)g_sum[1] / samples;
        gyro_offset[2] = (float)g_sum[2] / samples;
    }

    void read_data(sensor_msgs__msg__Imu *msg) {
        uint8_t reg = 0x3B;
        i2c_write_blocking(i2c_port, addr, &reg, 1, true);

        uint8_t buffer[14];
        i2c_read_blocking(i2c_port, addr, buffer, 14, false);

        int16_t accelX = (buffer[0] << 8) | buffer[1];
        int16_t accelY = (buffer[2] << 8) | buffer[3];
        int16_t accelZ = (buffer[4] << 8) | buffer[5];
        
        int16_t gyroX = (buffer[8] << 8) | buffer[9];
        int16_t gyroY = (buffer[10] << 8) | buffer[11];
        int16_t gyroZ = (buffer[12] << 8) | buffer[13];

        // ---------------------------------------------------------
        // AXIS REMAPPING (REP-103 COMPLIANCE)
        // Based on your raw data, the physical X-axis is feeling gravity.
        // We map physical X -> ROS Z.
        // You may need to invert X or Y depending on which way is "forward" 
        // on your physical robot, but this puts gravity in the right place.
        // ---------------------------------------------------------
        
        float raw_accel_x_ms2 = (accelX / 16384.0f) * G_TO_MS2;
        float raw_accel_y_ms2 = (accelY / 16384.0f) * G_TO_MS2;
        float raw_accel_z_ms2 = (accelZ / 16384.0f) * G_TO_MS2;

        float raw_gyro_x_rads = ((gyroX - gyro_offset[0]) / 131.0f) * DEG_TO_RAD;
        float raw_gyro_y_rads = ((gyroY - gyro_offset[1]) / 131.0f) * DEG_TO_RAD;
        float raw_gyro_z_rads = ((gyroZ - gyro_offset[2]) / 131.0f) * DEG_TO_RAD;

        // Swapped mapping: 
        // ROS Z gets raw X (Gravity)
        // ROS X gets raw Y 
        // ROS Y gets raw Z
        msg->linear_acceleration.x = raw_accel_y_ms2; 
        msg->linear_acceleration.y = raw_accel_z_ms2;
        msg->linear_acceleration.z = raw_accel_x_ms2; 

        // Ensure gyros follow the exact same mapping!
        msg->angular_velocity.x = raw_gyro_y_rads;
        msg->angular_velocity.y = raw_gyro_z_rads;
        msg->angular_velocity.z = raw_gyro_x_rads;

        // Basic covariance diagonals (required by EKF)
        msg->linear_acceleration_covariance[0] = 0.01;
        msg->linear_acceleration_covariance[4] = 0.01;
        msg->linear_acceleration_covariance[8] = 0.01;
        
        msg->angular_velocity_covariance[0] = 0.01;
        msg->angular_velocity_covariance[4] = 0.01;
        msg->angular_velocity_covariance[8] = 0.01;
        
        // No magnetometer
        msg->orientation_covariance[0] = -1.0; 
    }
};

#endif
