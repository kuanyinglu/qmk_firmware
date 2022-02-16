#pragma once

//Filter 
//  1 - NONE
//  2 - Madgwick
//  3 - Mahony

// void quaternion_select_filter(uint8_t sel);
// void quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
// void quaternion_no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
// Madgwick Quaternion Update
// void quaternion_madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);

// Mahony accelleration filter
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// float Kp = 30.0;
// float Ki = 0.0;
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
// with MPU-6050, some instability observed at Kp=100 Now set to 30.
void quaternion_mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
