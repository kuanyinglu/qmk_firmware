#include "bno055.h"
#include "i2c_master.h"
#include "print.h"
#include "timer.h"
#include "wait.h"
#include <math.h>

#define I2C_TIMEOUT 1000

double x = 0;
double y = 0;
double z = 0;
double w = 0;
const uint8_t offset_data[18] = { 0xE7, 0xFF, 0xF0, 0xFF, 0xE2, 0xFF, 0x45, 0x00, 0x9C, 0x01, 0x57, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x00, 0x00 };

bool bno055_init(void) {
  /* Make sure we have the right device */
  uint8_t data = 0;
  i2c_readReg(I2C_ADDR << 1, BNO055_CHIP_ID_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  if (data != BNO055_ID) {
    wait_ms(1000); // hold on for boot
    i2c_readReg(I2C_ADDR << 1, BNO055_CHIP_ID_ADDR, &data, sizeof(data), I2C_TIMEOUT);
    if (data != BNO055_ID) {
      return false; // still not? ok bail
    }
  }

  data = OPERATION_MODE_CONFIG;
  /* Switch to config mode (just in case since this is the default) */
  i2c_writeReg(I2C_ADDR << 1, BNO055_OPR_MODE_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  // Write calibration data
  i2c_writeReg(I2C_ADDR << 1, ACCEL_OFFSET_X_LSB_ADDR, &offset_data[0], sizeof(offset_data), I2C_TIMEOUT);

  
  data = 0x20;
  /* Reset */
  i2c_writeReg(I2C_ADDR << 1, BNO055_SYS_TRIGGER_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  /* wait_ms incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  wait_ms(30);
  i2c_readReg(I2C_ADDR << 1, BNO055_CHIP_ID_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  while (data != BNO055_ID) {
    wait_ms(10);
  }
  wait_ms(50);

  data = POWER_MODE_NORMAL;
  /* Set to normal power mode */
  i2c_writeReg(I2C_ADDR << 1, BNO055_PWR_MODE_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  wait_ms(10);

  data = 0x00;
  i2c_writeReg(I2C_ADDR << 1, BNO055_PAGE_ID_ADDR, &data, sizeof(data), I2C_TIMEOUT);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  wait_ms(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  wait_ms(10);
  */

  data = 0x80;
  i2c_writeReg(I2C_ADDR << 1, BNO055_SYS_TRIGGER_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  wait_ms(10);
  data = OPERATION_MODE_NDOF;
  /* Set the requested operating mode (see section 3.3) */
  i2c_writeReg(I2C_ADDR << 1, BNO055_OPR_MODE_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  wait_ms(20);
  
  return true;
}

bool calibration_done(void) {
  uint8_t data = 0;
  i2c_readReg(I2C_ADDR << 1, BNO055_CALIB_STAT_ADDR, &data, sizeof(data), I2C_TIMEOUT);
  return data == 255;
}

//  *           possible vector type values
//  *           [BNO055_ACCEL_DATA_X_LSB_ADDR
//  *            BNO055_MAG_DATA_X_LSB_ADDR
//  *            BNO055_GYRO_DATA_X_LSB_ADDR
//  *            BNO055_EULER_H_LSB_ADDR
//  *            BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR
//  *            BNO055_GRAVITY_DATA_X_LSB_ADDR]
void retrieveVector(uint8_t vector_type) {
  uint8_t data[6];

  int16_t xt = 0;
  int16_t yt = 0;
  int16_t zt = 0;

  /* Read vector data (6 bytes) */
  i2c_readReg(I2C_ADDR << 1, vector_type, &data[0], sizeof(data), I2C_TIMEOUT);

  xt = ((int16_t)data[0]) | (((int16_t)data[1]) << 8);
  yt = ((int16_t)data[2]) | (((int16_t)data[3]) << 8);
  zt = ((int16_t)data[4]) | (((int16_t)data[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type) {
  case BNO055_MAG_DATA_X_LSB_ADDR:
    /* 1uT = 16 LSB */
    x = ((double)xt) / 16.0;
    y = ((double)yt) / 16.0;
    z = ((double)zt) / 16.0;
    break;
  case BNO055_GYRO_DATA_X_LSB_ADDR:
    /* 1dps = 16 LSB */
    x = ((double)xt) / 16.0;
    y = ((double)yt) / 16.0;
    z = ((double)zt) / 16.0;
    break;
  case BNO055_EULER_H_LSB_ADDR:
    /* 1 degree = 16 LSB */
    x = ((double)xt) / 16.0;
    y = ((double)yt) / 16.0;
    z = ((double)zt) / 16.0;
    break;
  case BNO055_ACCEL_DATA_X_LSB_ADDR:
    /* 1m/s^2 = 100 LSB */
    x = ((double)xt) / 100.0;
    y = ((double)yt) / 100.0;
    z = ((double)zt) / 100.0;
    break;
  case BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR:
    /* 1m/s^2 = 100 LSB */
    x = ((double)xt) / 100.0;
    y = ((double)yt) / 100.0;
    z = ((double)zt) / 100.0;
    break;
  case BNO055_GRAVITY_DATA_X_LSB_ADDR:
    /* 1m/s^2 = 100 LSB */
    x = ((double)xt) / 100.0;
    y = ((double)yt) / 100.0;
    z = ((double)zt) / 100.0;
    break;
  }
}

double getX(void) {
  return x;
}

double getY(void) {
  return y;
}

double getZ(void) {
  return z;
}

double getW(void) {
  return w;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
void retrieveQuat(void) {
  uint8_t data[8];

  int16_t xt = 0;
  int16_t yt = 0;
  int16_t zt = 0;
  int16_t wt = 0;

  /* Read quat data (8 bytes) */
  i2c_readReg(I2C_ADDR << 1, BNO055_QUATERNION_DATA_W_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  wt = (((uint16_t)data[1]) << 8) | ((uint16_t)data[0]);
  xt = (((uint16_t)data[3]) << 8) | ((uint16_t)data[2]);
  yt = (((uint16_t)data[5]) << 8) | ((uint16_t)data[4]);
  zt = (((uint16_t)data[7]) << 8) | ((uint16_t)data[6]);

  /*!
   * Assign to Quaternion
   * See
   * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));

  w = scale * wt;
  x = scale * xt;
  y = scale * yt;
  z = scale * zt;
}

void printCalibrationData(void) {
  uint8_t data[18];
  i2c_readReg(I2C_ADDR << 1, BNO055_CALIB_STAT_ADDR, &data[0], sizeof(uint8_t), I2C_TIMEOUT);
  
  print_val_hex8(data[0]);
  if (data[0] == 255)
  {  
    data[0] = OPERATION_MODE_CONFIG;
    /* Switch to config mode to read offset */
    i2c_writeReg(I2C_ADDR << 1, BNO055_OPR_MODE_ADDR, &data[0], sizeof(uint8_t), I2C_TIMEOUT);

  }
  i2c_readReg(I2C_ADDR << 1, ACCEL_OFFSET_X_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  // uint16_t ax = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
  // uint16_t ay = (((uint16_t)data[2]) << 8) | ((uint16_t)data[3]);
  // uint16_t az = (((uint16_t)data[4]) << 8) | ((uint16_t)data[5]);
  // uint16_t mx = (((uint16_t)data[6]) << 8) | ((uint16_t)data[7]);
  // uint16_t my = (((uint16_t)data[8]) << 8) | ((uint16_t)data[9]);
  // uint16_t mz = (((uint16_t)data[10]) << 8) | ((uint16_t)data[11]);
  // uint16_t gx = (((uint16_t)data[12]) << 8) | ((uint16_t)data[13]);
  // uint16_t gy = (((uint16_t)data[14]) << 8) | ((uint16_t)data[15]);
  // uint16_t gz = (((uint16_t)data[16]) << 8) | ((uint16_t)data[17]);
  // print_val_hex16(ax);
  // print_val_hex16(ay);
  // print_val_hex16(az);
  // print_val_hex16(mx);
  // print_val_hex16(my);
  // print_val_hex16(mz);
  // print_val_hex16(gx);
  // print_val_hex16(gy);
  // print_val_hex16(gz);
}
void getAngles(void)
{
  uint8_t data[6];
  i2c_readReg(I2C_ADDR << 1, BNO055_EULER_H_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  // int h = ((int)((((uint16_t)data[1]) << 8) | ((uint16_t)data[0]))) / 16;
  // int r = ((int)((((uint16_t)data[3]) << 8) | ((uint16_t)data[2]))) / 16;
  // int p = ((int)((((uint16_t)data[5]) << 8) | ((uint16_t)data[4]))) / 16;
  // print_val_decs(h);
  // print_val_decs(r);
  // print_val_decs(p);
}

void getAcc(void)
{
  uint8_t data[6];
  i2c_readReg(I2C_ADDR << 1, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  // int y = ((int)((((uint16_t)data[1]) << 8) | ((uint16_t)data[0]))) / 16;
  // int x = -((int)((((uint16_t)data[3]) << 8) | ((uint16_t)data[2]))) / 16;
  // int z = ((int)((((uint16_t)data[5]) << 8) | ((uint16_t)data[4]))) / 16;
  // print_val_decs(x);
  // print_val_decs(y);
  // print_val_decs(z);
}

euler_data getSensorData(void)
{
  uint8_t data[8];
  struct euler_data sensorData;
  i2c_readReg(I2C_ADDR << 1, BNO055_EULER_H_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  sensorData.h = ((int)((((uint16_t)data[1]) << 8) | ((uint16_t)data[0]))) / 16;
  sensorData.r = ((int)((((uint16_t)data[3]) << 8) | ((uint16_t)data[2]))) / 16;
  sensorData.p = ((int)((((uint16_t)data[5]) << 8) | ((uint16_t)data[4]))) / 16;
  // const double scale = (1.0 / (1 << 14));

  /* Read quat data (8 bytes) */
  // i2c_readReg(I2C_ADDR << 1, BNO055_QUATERNION_DATA_W_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  // w = ((((uint16_t)data[1]) << 8) | ((uint16_t)data[0])) * scale;
  // x = ((((uint16_t)data[3]) << 8) | ((uint16_t)data[2])) * scale;
  // y = ((((uint16_t)data[5]) << 8) | ((uint16_t)data[4])) * scale;
  // z = ((((uint16_t)data[7]) << 8) | ((uint16_t)data[6])) * scale;
  // double sqw = w * w;
  // double sqx = x * x;
  // double sqy = y * y;
  // double sqz = z * z;

  // sensorData.h = atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw)) * 180 / M_PI;
  // sensorData.r = asin(-2.0 * (x * z - y * w) / (sqx + sqy + sqz + sqw)) * 180 / M_PI;
  // sensorData.p = atan2(2.0 * (y * z + x * w), (-sqx - sqy + sqz + sqw)) *180 / M_PI;

  i2c_readReg(I2C_ADDR << 1, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, &data[0], sizeof(data), I2C_TIMEOUT);
  sensorData.y = ((int)((((uint16_t)data[1]) << 8) | ((uint16_t)data[0]))) / 16;
  sensorData.x = -((int)((((uint16_t)data[3]) << 8) | ((uint16_t)data[2]))) / 16;
  sensorData.z = ((int)((((uint16_t)data[5]) << 8) | ((uint16_t)data[4]))) / 16;
  
  return sensorData;
}