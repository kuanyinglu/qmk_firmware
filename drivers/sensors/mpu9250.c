#include "mpu9250_register_map.h"
#include "spi_master.h"
#include "debug.h"
#include "print.h"
#include "mpu9250.h"
#include "wait.h"
#include <math.h>
#include "timer.h"

// enum class ACCEL_FS_SEL {
//     A2G,
//     A4G,
//     A8G,
//     A16G
// };
// enum class GYRO_FS_SEL {
//     G250DPS,
//     G500DPS,
//     G1000DPS,
//     G2000DPS
// };
// enum class MAG_OUTPUT_BITS {
//     M14BITS,
//     M16BITS
// };

// enum class FIFO_SAMPLE_RATE : uint8_t {
//     SMPL_1000HZ,
//     SMPL_500HZ,
//     SMPL_333HZ,
//     SMPL_250HZ,
//     SMPL_200HZ,
//     SMPL_167HZ,
//     SMPL_143HZ,
//     SMPL_125HZ,
// };

// enum class GYRO_DLPF_CFG : uint8_t {
//     DLPF_250HZ,
//     DLPF_184HZ,
//     DLPF_92HZ,
//     DLPF_41HZ,
//     DLPF_20HZ,
//     DLPF_10HZ,
//     DLPF_5HZ,
//     DLPF_3600HZ,
// };

// enum class ACCEL_DLPF_CFG : uint8_t {
//     DLPF_218HZ_0,
//     DLPF_218HZ_1,
//     DLPF_99HZ,
//     DLPF_45HZ,
//     DLPF_21HZ,
//     DLPF_10HZ,
//     DLPF_5HZ,
//     DLPF_420HZ,
// };

#define MPU9250_WHOAMI_DEFAULT_VALUE         0x71
#define MPU9255_WHOAMI_DEFAULT_VALUE         0x73
#define MPU6500_WHOAMI_DEFAULT_VALUE         0x70
#define MPU9250_DEFAULT_ADDRESS              0x68// Device address when ADO = 0
#define AK8963_ADDRESS                       0x0C//  Address of magnetometer
#define AK8963_WHOAMI_DEFAULT_VALUE          0x48
#define MAG_MODE                             0x06// 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
#define CALIB_GYRO_SENSITIVITY               131// LSB/degrees/sec
#define CALIB_ACCEL_SENSITIVITY              16384// LSB/g

#define SPI_MODE                             3
#define MPU9250_CLOCK_SPEED                  400000
#define SPI_DIVISOR (F_CPU / MPU9250_CLOCK_SPEED)
#define WRITE_FLAG                           0x7F
#define READ_FLAG                            0x80

// settings
mpu9250_setting setting = { 1, 0, 1, 4, 3, 3, 1, 3};
// TODO: this should be configured!!
float acc_resolution = 0;                // scale resolutions per LSB for the sensors
float gyro_resolution = 0;               // scale resolutions per LSB for the sensors
float mag_resolution = 0;                // scale resolutions per LSB for the sensors

// Calibration Parameters
float acc_bias[3] = {0, 0, 0};   // acc calibration value in ACCEL_FS_SEL: 2g
float gyro_bias[3] = {0, 0, 0};  // gyro calibration value in GYRO_FS_SEL: 250dps
// float magnetic_declination = -7.51;  // Japan, 24th June
uint8_t gravity_count = 1;

// Self Test
float self_test_result[6] = {0, 0, 0, 0, 0, 0};  // holds results of gyro and accelerometer self test

// IMU Data
float a[3] = {0, 0, 0};
float g[3] = {0, 0, 0};
float m[3] = {0, 0, 0};
float q[4] = {1, 0, 0, 0};  // vector to hold quaternion
float rpy[3] = {0, 0, 0};
float lin_acc[3] = {0, 0, 0};  // linear acceleration (acceleration with gravity component subtracted)
size_t n_filter_iter  = 1;
double deltaT = 0;
uint32_t newTime = 0;

// Other settings
bool has_connected = false;
bool b_ahrs = true;
bool b_verbose = true;

bool mpu9250_setup(void) {

    setPinOutput(SPI_SS2_PIN);

    spi_init();
    spi_stop();

    spi_start(SPI_SS2_PIN, false, SPI_MODE, SPI_DIVISOR);
    spi_stop();
    wait_ms(10); 


    // if (isConnectedMPU9250()) {
        initMPU9250();
        // initAK8963();
        // if (!isConnectedAK8963())
            // has_connected = false;
            // return false;
        // }
    // } else {
        // has_connected = false;
        // return false;
    // }
    has_connected = true;
    return true;
}

void sleep(bool b) {
    uint8_t c = read_byte(PWR_MGMT_1);  // read the value, change sleep bit to match b, write byte back to register
    if (b) {
        c = c | 0x40;  // sets the sleep bit
    } else {
        c = c & 0xBF;  // mask 1011111 keeps all the previous bits
    }
    write_byte(PWR_MGMT_1, c);
}

void verbose(bool b) {
    b_verbose = b;
}

void ahrs(bool b) {
    b_ahrs = b;
}

void calibrateAccelGyro(void) {
    calibrate_acc_gyro_impl();
}

bool isConnected(void) {
    // has_connected = isConnectedMPU9250() && isConnectedAK8963();
    has_connected = isConnectedMPU9250();
    return has_connected;
}

bool isConnectedMPU9250(void) {
    write_byte(WHO_AM_I_MPU9250, 0x68);
    uint8_t c = read_byte(WHO_AM_I_MPU9250);
    if (b_verbose) {
        print("MPU9250 WHO AM I = ");
        print_hex8(c);
        print("\n");
    }
    bool b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
    b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
    b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
    return b;
}

// bool isConnectedAK8963(void) {
//     write_byte(I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG); //Set the I2C slave addres of AK8963 and set for write
//     write_byte(I2C_SLV0_REG, AK8963_WHO_AM_I); //I2C slave 0 register address from where to begin data transfer
//     write_byte(I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
//     wait_ms(10);
//     uint8_t c = read_byte(EXT_SENS_DATA_00);
//     if (b_verbose) {
//         print("AK8963 WHO AM I = ");
//         print_hex8(c);
//         print("\n");
//     }
//     return (c == AK8963_WHOAMI_DEFAULT_VALUE);
// }

bool isSleeping(void) {
    uint8_t c = read_byte(PWR_MGMT_1);
    return (c & 0x40) == 0x40;
}

bool available(void) {
    return has_connected && (read_byte(INT_STATUS) & 0x01);
}

bool mpu9250_update(void) {
    if (!available()) return false;

    update_accel_gyro();

    // update_mag();

    // Madgwick function needs to be fed North, East, and Down direction like
    // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
    // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
    // Magneto direction is Right-Hand, Y-Forward, Z-Down
    // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
    // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
    // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
    // because gravity is by convention positive down, we need to ivnert the accel data

    // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
    // acc[mg], gyro[deg/s], mag [mG]
    // gyro will be convert from [deg/s] to [rad/s] inside of this function
    // quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2], q);
    // DEG_TO_RAD 0.017453292519943295769236907684886

    //take a few measures to estimate gravity here
    if (gravity_count < 65) {
        m[0] += a[0];
        m[1] += a[1];
        m[2] += a[2];
        gyro_bias[0] += g[0];
        gyro_bias[1] += g[1];
        gyro_bias[2] += g[2];
        gravity_count ++;
        return false;
    }

    if (gravity_count == 65) {
        m[0] = m[0] / 64;
        m[1] = m[1] / 64;
        m[2] = m[2] / 64;
        gyro_bias[0] = gyro_bias[0] / 64;
        gyro_bias[1] = gyro_bias[1] / 64;
        gyro_bias[2] = gyro_bias[2] / 64;
        gravity_count ++;
    }
        // print("MAG: ");
        // print_hex16((uint16_t)m[0]);
        // print(" ");
        // print_hex16((uint16_t)m[1]);
        // print(" ");
        // print_hex16((uint16_t)m[2]);
        // print("\n");
        // print("ACC: ");
        // print_hex16((uint16_t)lin_acc[0]);
        // print(" ");
        // print_hex16((uint16_t)lin_acc[1]);
        // print(" ");
        // print_hex16((uint16_t)lin_acc[2]);
        // print("\n");

    float an = -a[0];
    float ae = +a[1];
    float ad = +a[2];
    float gn = +(g[0] - gyro_bias[0]) * 0.017453292519943295769236907684886;
    float ge = -(g[1] - gyro_bias[1]) * 0.017453292519943295769236907684886;
    float gd = -(g[2] - gyro_bias[2]) * 0.017453292519943295769236907684886;

    update_rpy(an, ae, ad, gn, ge, gd);
    return true;
}

float getRoll(void) { return rpy[0]; }
float getPitch(void) { return rpy[1]; }
float getYaw(void) { return rpy[2]; }

float getEulerX(void) { return rpy[0]; }
float getEulerY(void) { return -rpy[1]; }
float getEulerZ(void) { return -rpy[2]; }

float getQuaternionX(void) { return q[1]; }
float getQuaternionY(void) { return q[2]; }
float getQuaternionZ(void) { return q[3]; }
float getQuaternionW(void) { return q[0]; }

// float getAcc(uint8_t i) { return (i < 3) ? a[i] : 0; }
// float getGyro(uint8_t i) { return (i < 3) ? g[i] : 0; }
// float getMag(uint8_t i) { return (i < 3) ? m[i] : 0; }
// float getLinearAcc(uint8_t i) { return (i < 3) ? lin_acc[i] : 0; }

float getAccX(void) { return a[0]; }
float getAccY(void) { return a[1]; }
float getAccZ(void) { return a[2]; }
float getGyroX(void) { return g[0]; }
float getGyroY(void) { return g[1]; }
float getGyroZ(void) { return g[2]; }
float getMagX(void) { return m[0]; }
float getMagY(void) { return m[1]; }
float getMagZ(void) { return m[2]; }
float getLinearAccX(void) { return lin_acc[0]; }
float getLinearAccY(void) { return lin_acc[1]; }
float getLinearAccZ(void) { return lin_acc[2]; }

// float getAccBias(uint8_t i) { return (i < 3) ? acc_bias[i] : 0; }
// float getGyroBias(uint8_t i) { return (i < 3) ? gyro_bias[i] : 0; }

float getAccBiasX(void) { return acc_bias[0]; }
float getAccBiasY(void) { return acc_bias[1]; }
float getAccBiasZ(void) { return acc_bias[2]; }
float getGyroBiasX(void) { return gyro_bias[0]; }
float getGyroBiasY(void) { return gyro_bias[1]; }
float getGyroBiasZ(void) { return gyro_bias[2]; }


void setAccBias(float x, float y, float z) {
    acc_bias[0] = x;
    acc_bias[1] = y;
    acc_bias[2] = z;
    write_accel_offset();
}
void setGyroBias(float x, float y, float z) {
    gyro_bias[0] = x;
    gyro_bias[1] = y;
    gyro_bias[2] = z;
    write_gyro_offset();
}
// void setMagneticDeclination(float d) { magnetic_declination = d; }

// void selectFilter(uint8_t sel) {
//     quaternion_select_filter(sel);
// }

void setFilterIterations(size_t n) {
    if (n > 0) n_filter_iter = n;
}

bool selftest(void) {
    return self_test_impl();
}

void initMPU9250(void) {
    acc_resolution = get_acc_resolution(setting.accel_fs_sel);
    gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);
    // mag_resolution = get_mag_resolution(setting.mag_output_bits);

    // reset device
    write_byte(PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    wait_ms(100);

    // wake up device
    write_byte(PWR_MGMT_2, 0x00);  // Clear sleep mode bit (6), enable all sensors
    wait_ms(100);

    // get stable time source
    write_byte(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    wait_ms(100);

    // disable I2C
    write_byte(USER_CTRL, 0x20);// I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
    wait_ms(100);

    write_byte(I2C_MST_CTRL, 0x0D);// I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
    wait_ms(100);

    write_byte(I2C_MST_DELAY_CTRL, 0x81);// I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
    wait_ms(100);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    uint8_t mpu_config = (uint8_t)setting.gyro_dlpf_cfg;
    write_byte(MPU_CONFIG, mpu_config);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    uint8_t sample_rate = (uint8_t)setting.fifo_sample_rate;
    write_byte(SMPLRT_DIV, sample_rate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                        // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = read_byte(GYRO_CONFIG);  // get current GYRO_CONFIG register value
    c = c & ~0xE0;                                     // Clear self-test bits [7:5]
    c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
    c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
    c = c | ((uint8_t)(setting.gyro_fs_sel) << 3);       // Set full scale range for the gyro
    c = c | ((uint8_t)(~setting.gyro_fchoice) & 0x03);   // Set Fchoice for the gyro
    write_byte(GYRO_CONFIG, c);          // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = read_byte(ACCEL_CONFIG);     // get current ACCEL_CONFIG register value
    c = c & ~0xE0;                                 // Clear self-test bits [7:5]
    c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
    c = c | ((uint8_t)(setting.accel_fs_sel) << 3);  // Set full scale range for the accelerometer
    write_byte(ACCEL_CONFIG, c);     // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = read_byte(ACCEL_CONFIG2);        // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | (~(setting.accel_fchoice << 3) & 0x08);    // Set accel_fchoice_b to 1
    c = c | ((uint8_t)(setting.accel_dlpf_cfg) & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    write_byte(ACCEL_CONFIG2, c);        // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    write_byte(INT_PIN_CFG, 0x10);
    write_byte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    wait_ms(100);

}

// void update_rpy(float qw, float qx, float qy, float qz) {
//     // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
//     // In this coordinate system, the positive z-axis is down toward Earth.
//     // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
//     // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
//     // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
//     // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
//     // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
//     // applied in the correct order which for this configuration is yaw, pitch, and then roll.
//     // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
//     //define PI 3.1415926535897932384626433832795
//     float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
//     a12 = 2.0f * (qx * qy + qw * qz);
//     a22 = qw * qw + qx * qx - qy * qy - qz * qz;
//     a31 = 2.0f * (qw * qx + qy * qz);
//     a32 = 2.0f * (qx * qz - qw * qy);
//     a33 = qw * qw - qx * qx - qy * qy + qz * qz;
//     rpy[0] = atan2f(a31, a33);
//     rpy[1] = -asinf(a32);
//     rpy[2] = atan2f(a12, a22);
//     rpy[0] *= 180.0f / 3.1415926535897932384626433832795;
//     rpy[1] *= 180.0f / 3.1415926535897932384626433832795;
//     rpy[2] *= 180.0f / 3.1415926535897932384626433832795;
//     // rpy[2] += magnetic_declination;
//     if (rpy[2] >= +180.f)
//         rpy[2] -= 360.f;
//     else if (rpy[2] < -180.f)
//         rpy[2] += 360.f;

//     lin_acc[0] = a[0] + a31;
//     lin_acc[1] = a[1] + a32;
//     lin_acc[2] = a[2] - a33;
// }

void update_rpy(float ax, float ay, float az, float gx, float gy, float gz) {
    if (newTime == 0) {
        deltaT = 0;
    } else {
        deltaT = timer_elapsed(newTime);
    }
    newTime = timer_read();
    q[0] += 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz) * deltaT;
    q[1] += 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy) * deltaT;
    q[2] += 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx) * deltaT;
    q[3] += 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx) * deltaT;
    float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;

    float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
    a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    rpy[0] = atan2f(a31, a33);
    rpy[1] = -asinf(a32);
    rpy[2] = atan2f(a12, a22);
    if (rpy[2] >= +3.1415926535897932384626433832795f)
        rpy[2] -= (2 * 3.1415926535897932384626433832795f);
    else if (rpy[2] < -3.1415926535897932384626433832795f)
        rpy[2] += (2 * 3.1415926535897932384626433832795f);

    lin_acc[0] = ((a[0] + a31) * cos(rpy[0]) * cos(rpy[1])) - m[0];
    lin_acc[1] = ((a[1] + a32) * cos(rpy[0]) * cos(rpy[2])) - m[1];
    lin_acc[2] = ((a[2] - a33) * cos(rpy[1]) * cos(rpy[2])) - m[2];

    rpy[0] *= 180.0f / 3.1415926535897932384626433832795;
    rpy[1] *= 180.0f / 3.1415926535897932384626433832795;
    rpy[2] *= 180.0f / 3.1415926535897932384626433832795;
}

void update_accel_gyro(void) {
    int16_t raw_acc_gyro_data[7];        // used to read all 14 bytes at once from the MPU9250 accel/gyro
    read_accel_gyro(raw_acc_gyro_data);  // INT cleared on any read

    // Now we'll calculate the accleration value into actual g's
    a[0] = (float)raw_acc_gyro_data[0] * acc_resolution;  // get actual g value, this depends on scale being set
    a[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
    a[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

    // Calculate the gyro value into actual degrees per second
    g[0] = (float)raw_acc_gyro_data[4] * gyro_resolution;  // get actual gyro value, this depends on scale being set
    g[1] = (float)raw_acc_gyro_data[5] * gyro_resolution;
    g[2] = (float)raw_acc_gyro_data[6] * gyro_resolution;
}

void read_accel_gyro(int16_t* destination) {
    uint8_t raw_data[14];                                                 // x/y/z accel register data stored here
    read_bytes(ACCEL_XOUT_H, 14, &raw_data[0]);             // Read the 14 raw data registers into data array
    destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1];  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
    destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
    destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
    destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
    destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
    destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// ACCEL_FS_SEL: 2g (maximum sensitivity)
// GYRO_FS_SEL: 250dps (maximum sensitivity)
void calibrate_acc_gyro_impl(void) {
    set_acc_gyro_to_calibration();
    collect_acc_gyro_data_to(acc_bias, gyro_bias);
    write_accel_offset();
    write_gyro_offset();
    wait_ms(100);
    initMPU9250();
    wait_ms(1000);
}

void set_acc_gyro_to_calibration(void) {
    // reset device
    write_byte(PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    wait_ms(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    write_byte(PWR_MGMT_1, 0x01);
    write_byte(PWR_MGMT_2, 0x00);
    wait_ms(100);

    // Configure device for bias calculation
    write_byte(INT_ENABLE, 0x00);    // Disable all interrupts
    write_byte(FIFO_EN, 0x00);       // Disable FIFO
    write_byte(PWR_MGMT_1, 0x00);    // Turn on internal clock source
    write_byte(I2C_MST_CTRL, 0x00);  // Disable I2C master
    write_byte(USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
    write_byte(USER_CTRL, 0x0C);     // Reset FIFO and DMP
    wait_us(15000);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    write_byte(MPU_CONFIG, 0x01);    // Set low-pass filter to 188 Hz
    write_byte(SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
    write_byte(GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    write_byte(ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    write_byte(USER_CTRL, 0x40);  // Enable FIFO
    write_byte(FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    wait_ms(40);
}

void collect_acc_gyro_data_to(float* a_bias, float* g_bias) {
    // At end of sample accumulation, turn off FIFO sensor read
    uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
    write_byte(FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
    read_bytes(FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
    uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
    uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

    for (uint16_t ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        read_bytes(FIFO_R_W, 12, &data[0]);              // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        a_bias[1] += (float)accel_temp[1];
        a_bias[2] += (float)accel_temp[2];
        g_bias[0] += (float)gyro_temp[0];
        g_bias[1] += (float)gyro_temp[1];
        g_bias[2] += (float)gyro_temp[2];
    }
    a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
    a_bias[1] /= (float)packet_count;
    a_bias[2] /= (float)packet_count;
    g_bias[0] /= (float)packet_count;
    g_bias[1] /= (float)packet_count;
    g_bias[2] /= (float)packet_count;

    if (a_bias[2] > 0L) {
        a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
    }
}

void write_accel_offset(void) {
    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
    read_bytes(XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
    acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
    read_bytes(YA_OFFSET_H, 2, &read_data[0]);
    acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
    read_bytes(ZA_OFFSET_H, 2, &read_data[0]);
    acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

    int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
    for (int i = 0; i < 3; i++) {
        if (acc_bias_reg[i] % 2) {
            mask_bit[i] = 0;
        }
        acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
        if (mask_bit[i]) {
            acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
        } else {
            acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
        }
    }

    uint8_t write_data[6] = {0};
    write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    write_data[1] = (acc_bias_reg[0]) & 0xFF;
    write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    write_data[3] = (acc_bias_reg[1]) & 0xFF;
    write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    write_data[5] = (acc_bias_reg[2]) & 0xFF;

    // Push accelerometer biases to hardware registers
    write_byte(XA_OFFSET_H, write_data[0]);
    write_byte(XA_OFFSET_L, write_data[1]);
    write_byte(YA_OFFSET_H, write_data[2]);
    write_byte(YA_OFFSET_L, write_data[3]);
    write_byte(ZA_OFFSET_H, write_data[4]);
    write_byte(ZA_OFFSET_L, write_data[5]);
}

void write_gyro_offset(void) {
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    uint8_t gyro_offset_data[6] = {0, 0, 0, 0, 0, 0};
    gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
    gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    write_byte(XG_OFFSET_H, gyro_offset_data[0]);
    write_byte(XG_OFFSET_L, gyro_offset_data[1]);
    write_byte(YG_OFFSET_H, gyro_offset_data[2]);
    write_byte(YG_OFFSET_L, gyro_offset_data[3]);
    write_byte(ZG_OFFSET_H, gyro_offset_data[4]);
    write_byte(ZG_OFFSET_L, gyro_offset_data[5]);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
bool self_test_impl(void)  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0;

    write_byte(SMPLRT_DIV, 0x00);       // Set gyro sample rate to 1 kHz
    write_byte(MPU_CONFIG, 0x02);       // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    write_byte(GYRO_CONFIG, FS << 3);   // Set full scale range for the gyro to 250 dps
    write_byte(ACCEL_CONFIG2, 0x02);    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    write_byte(ACCEL_CONFIG, FS << 3);  // Set full scale range for the accelerometer to 2 g

    for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

        read_bytes(ACCEL_XOUT_H, 6, &raw_data[0]);          // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        read_bytes(GYRO_XOUT_H, 6, &raw_data[0]);           // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        gAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    write_byte(ACCEL_CONFIG, 0xE0);  // Enable self test on all three axes and set accelerometer range to +/- 2 g
    write_byte(GYRO_CONFIG, 0xE0);   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    wait_us(25000);                                     // Delay a while to let the device stabilize

    for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

        read_bytes(ACCEL_XOUT_H, 6, &raw_data[0]);            // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        read_bytes(GYRO_XOUT_H, 6, &raw_data[0]);             // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        gSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    write_byte(ACCEL_CONFIG, 0x00);
    write_byte(GYRO_CONFIG, 0x00);
    wait_us(25000);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    uint8_t self_test_data[6];
    self_test_data[0] = read_byte(SELF_TEST_X_ACCEL);  // X-axis accel self-test results
    self_test_data[1] = read_byte(SELF_TEST_Y_ACCEL);  // Y-axis accel self-test results
    self_test_data[2] = read_byte(SELF_TEST_Z_ACCEL);  // Z-axis accel self-test results
    self_test_data[3] = read_byte(SELF_TEST_X_GYRO);   // X-axis gyro self-test results
    self_test_data[4] = read_byte(SELF_TEST_Y_GYRO);   // Y-axis gyro self-test results
    self_test_data[5] = read_byte(SELF_TEST_Z_GYRO);   // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[0] - 1.0)));  // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[1] - 1.0)));  // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[2] - 1.0)));  // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[3] - 1.0)));  // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[4] - 1.0)));  // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[5] - 1.0)));  // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++) {
        self_test_result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;          // Report percent differences
        self_test_result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.;  // Report percent differences
    }

    if (b_verbose) {
        dprintf("x-axis self test: acceleration trim within : %f percent of factory value\n", self_test_result[0]);
        dprintf("y-axis self test: acceleration trim within : %f percent of factory value\n", self_test_result[1]);
        dprintf("z-axis self test: acceleration trim within : %f percent of factory value\n", self_test_result[2]);
        dprintf("x-axis self test: gyration trim within : %f percent of factory value\n", self_test_result[3]);
        dprintf("y-axis self test: gyration trim within : %f percent of factory value\n", self_test_result[4]);
        dprintf("z-axis self test: gyration trim within : %f percent of factory value\n", self_test_result[5]);
    }

    bool b = true;
    for (uint8_t i = 0; i < 6; ++i) {
        b &= fabs(self_test_result[i]) <= 14.f;
    }
    return b;
}

float get_acc_resolution(uint8_t accel_af_sel) {
    switch (accel_af_sel) {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case 0:
            return 2.0 / 32768.0;
        case 1:
            return 4.0 / 32768.0;
        case 2:
            return 8.0 / 32768.0;
        case 3:
            return 16.0 / 32768.0;
        default:
            return 0.;
    }
}

float get_gyro_resolution(uint8_t gyro_fs_sel) {
    switch (gyro_fs_sel) {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case 0:
            return 250.0 / 32768.0;
        case 1:
            return 500.0 / 32768.0;
        case 2:
            return 1000.0 / 32768.0;
        case 3:
            return 2000.0 / 32768.0;
        default:
            return 0.;
    }
}

void write_byte(uint8_t address, uint8_t data) {
    spi_start(SPI_SS2_PIN, false, SPI_MODE, SPI_DIVISOR);
    wait_us(1);
    spi_write(address&WRITE_FLAG);
    wait_us(1);
    spi_write(data);
    wait_us(1);
    spi_stop();
    // wait_us(50);
}

uint8_t read_byte(uint8_t address) {
    spi_start(SPI_SS2_PIN, false, SPI_MODE, SPI_DIVISOR);
    wait_us(1);
    spi_write(address|READ_FLAG);
    wait_us(1);
    uint8_t data = spi_read();
    wait_us(1);
    spi_stop();
    // wait_us(50);
    return data;                                 // Return data read from slave register
}

void read_bytes(uint8_t address, uint8_t count, uint8_t* dest) {
    spi_start(SPI_SS2_PIN, false, SPI_MODE, SPI_DIVISOR);
    wait_us(1);
    spi_write(address|READ_FLAG);
    wait_us(1);
    for (uint8_t i = 0; i < count; i++) {
        dest[i] = spi_read();
        wait_us(1);
    }
    // wait_us(50);
}