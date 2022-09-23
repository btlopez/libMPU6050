#include "MPU6050.h"
int Gscale = GFS_2000DPS;
int Ascale = AFS_8G;

float MPU6050lib::getGres() {
  switch (Gscale) {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
  // value:
  case GFS_250DPS:
    return 250.0 / 32768.0;
    break;
  case GFS_500DPS:
    return 500.0 / 32768.0;
    break;
  case GFS_1000DPS:
    return 1000.0 / 32768.0;
    break;
  case GFS_2000DPS:
    return 2000.0 / 32768.0;
    break;
  }
}

float MPU6050lib::getAres() {
  switch (Ascale) {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
  // value:
  case AFS_2G:
    return 2.0 / 32768.0;
    break;
  case AFS_4G:
    return 4.0 / 32768.0;
    break;
  case AFS_8G:
    return 8.0 / 32768.0;
    break;
  case AFS_16G:
    return 16.0 / 32768.0;
    break;
  }
}

void MPU6050lib::readAccelData(int16_t *destination) {
  uint8_t rawData[6]; // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6,
            &rawData[0]); // Read the six raw data registers into data array
  destination[0] =
      (int16_t)((rawData[0] << 8) |
                rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void MPU6050lib::readGyroData(int16_t *destination) {
  uint8_t rawData[6]; // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6,
            &rawData[0]); // Read the six raw data registers sequentially into
                          // data array
  destination[0] =
      (int16_t)((rawData[0] << 8) |
                rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

int16_t MPU6050lib::readTempData() {
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2,
            &rawData[0]); // Read the two raw data registers sequentially into
                          // data array
  return ((int16_t)rawData[0]) << 8 |
         rawData[1]; // Turn the MSB and LSB into a 16-bit value
}

void MPU6050lib::initMPU6050() {
  // wake up device-don't need this here if using calibration function below
  // Configure the MPU-6050
  delay(100);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1,
            0x00); // Clear sleep mode bit (6), enable all sensors

  // Reset gyro, accel, and temp sensors (in case we didn't power cycle)
  delay(100);
  writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07);

  // Get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1,
            0x02); // Set clock source to be PLL with x-axis gyroscope
                   // reference, bits 2:0 = 001

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // 1KHz gryo sample rate
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);

  // Set DLPF to 94 Hz
  writeByte(MPU6050_ADDRESS, CONFIG, 0x02);
  
  // Set DLPF to Default 260 Hz
  // writeByte(MPU6050_ADDRESS,CONFIG,0x00);

  // Set gyro scale to 2000 deg/s
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);

  // Set accel scale to 8 g's
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x16);
}

void MPU6050lib::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.write(data);                // Put data in Tx buffer
  Wire.endTransmission();          // Send the Tx buffer
}

uint8_t MPU6050lib::readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data;                    // `data` will store the register data
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.endTransmission(
      false); // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address,
                   (uint8_t)1); // Read one byte from slave register address
  data = Wire.read();           // Fill Rx buffer with result
  return data;                  // Return data read from slave register
}

void MPU6050lib::readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                           uint8_t *dest) {
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.endTransmission(
      false); // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count); // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
}
