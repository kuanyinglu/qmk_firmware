#pragma once

typedef struct {
    uint8_t accel_fs_sel;
    uint8_t gyro_fs_sel;
    uint8_t mag_output_bits;
    uint8_t fifo_sample_rate;
    uint8_t gyro_fchoice;
    uint8_t gyro_dlpf_cfg;
    uint8_t accel_fchoice;
    uint8_t accel_dlpf_cfg;
} mpu9250_setting;

bool mpu9250_setup(void);

void sleep(bool b);

// void verbose(bool b);

// void ahrs(bool b);

void calibrateAccelGyro(void);

// void calibrateMag(void);

bool isConnected(void);

bool isConnectedMPU9250(void);

// bool isConnectedAK8963(void);

bool isSleeping(void);

bool available(void);

bool mpu9250_update(void);
void mpu9250_calibrate(void);

// float getAccX(void);
// float getAccY(void);
// float getAccZ(void);
int8_t getGyroX(void);
int8_t getGyroY(void);
int8_t getGyroZ(void);
// float getLinearAccX(void);
// float getLinearAccY(void);
// float getLinearAccZ(void);

// float getAccBias(uint8_t i);
// float getGyroBias(uint8_t i);
// float getMagBias(uint8_t i);
// float getMagScale(uint8_t i);

// float getAccBiasX(void);
// float getAccBiasY(void);
// float getAccBiasZ(void);
float getGyroBiasX(void);
float getGyroBiasY(void);
float getGyroBiasZ(void);

// void setAccBias(const float x, const float y, const float z);
void setGyroBias(float x, float y, float z);
// void setMagneticDeclination(float d);
// void selectFilter(uint8_t sel);
bool selftest(void);
void initMPU9250(void);
// void update_rpy(float qw, float qx, float qy, float qz);
// void update_rpy(float ax, float ay, float az, float gx, float gy, float gz);
void update_accel_gyro(void);
void read_accel_gyro(int8_t* destination);
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// ACCEL_FS_SEL: 2g (maximum sensitivity)
// GYRO_FS_SEL: 250dps (maximum sensitivity)
void calibrate_acc_gyro_impl(void);
void set_acc_gyro_to_calibration(void);
// void collect_acc_gyro_data_to(float* a_bias, float* g_bias);
void collect_acc_gyro_data_to(float* g_bias);
void write_accel_offset(void);
void write_gyro_offset(void);
void collect_mag_data_to(float* m_bias, float* m_scale);
float get_acc_resolution(uint8_t accel_af_sel);
float get_gyro_resolution(uint8_t gyro_fs_sel);
void write_byte(uint8_t address, uint8_t data);
uint8_t read_byte(uint8_t address);
void read_bytes(uint8_t address, uint8_t count, uint8_t* dest);

bool self_test_impl(void);
