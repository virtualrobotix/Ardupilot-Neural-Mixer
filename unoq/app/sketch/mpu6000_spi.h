#pragma once

// MPU6000 on the SPI header: SCK D13, MISO D12, MOSI D11, CS D10 (INT is not used).
// Bit-banged mode 3, as in the earlier SPI port of this firmware: the Zephyr SPI driver clocks
// write data as zeros. Same registers, ranges and axis mapping as the I2C driver (mpu6050.h),
// whose helpers it uses. Included by imu.h.
//
// MISO has a pull-up: with no sensor on the header every read returns 0xFF, which can never pass
// for a WHO_AM_I or for a sample.

constexpr uint8_t MPU_SPI_CS = 10;
constexpr uint8_t MPU_SPI_MOSI = 11;
constexpr uint8_t MPU_SPI_MISO = 12;
constexpr uint8_t MPU_SPI_SCK = 13;

static bool mpuSpiReady = false;

static uint8_t mpuSpiShift(uint8_t out) {
  uint8_t in = 0;
  for (int8_t bit = 7; bit >= 0; --bit) {
    digitalWrite(MPU_SPI_SCK, LOW);                       // both sides change their data line
    digitalWrite(MPU_SPI_MOSI, (out >> bit) & 1);
    digitalWrite(MPU_SPI_SCK, HIGH);                      // both sides sample
    in = (uint8_t)((in << 1) | (digitalRead(MPU_SPI_MISO) ? 1 : 0));
  }
  return in;
}

static void mpuSpiBytes(uint8_t *buf, uint8_t n) {
  digitalWrite(MPU_SPI_CS, LOW);
  for (uint8_t i = 0; i < n; ++i) buf[i] = mpuSpiShift(buf[i]);
  digitalWrite(MPU_SPI_CS, HIGH);
}

static void mpuSpiWrite(uint8_t reg, uint8_t value) {
  uint8_t buf[2] = {(uint8_t)(reg & 0x7F), value};
  mpuSpiBytes(buf, 2);
}

static void mpuSpiRead(uint8_t reg, uint8_t *data, uint8_t size) {
  uint8_t buf[16] = {};
  if (size + 1u > sizeof(buf)) size = sizeof(buf) - 1;
  buf[0] = (uint8_t)(reg | 0x80);
  mpuSpiBytes(buf, size + 1);
  memcpy(data, buf + 1, size);
}

// write a register and check that the sensor kept the value
static bool mpuSpiSet(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    uint8_t got = ~value;
    mpuSpiWrite(reg, value);
    mpuSpiRead(reg, &got, 1);
    if (got == value) return true;
    delay(2);
  }
  return false;
}

static bool mpuSpiInit() {
  mpuSpiReady = false;
  pinMode(MPU_SPI_CS, OUTPUT);
  digitalWrite(MPU_SPI_CS, HIGH);
  pinMode(MPU_SPI_SCK, OUTPUT);
  digitalWrite(MPU_SPI_SCK, HIGH);
  pinMode(MPU_SPI_MOSI, OUTPUT);
  pinMode(MPU_SPI_MISO, INPUT_PULLUP);

  uint8_t who = 0;
  for (uint8_t attempt = 0; attempt < 10 && who != 0x68; ++attempt) {
    mpuSpiWrite(0x6A, 0x10);                              // SPI only: I2C interface off
    mpuSpiRead(0x75, &who, 1);
    if (who != 0x68) delay(20);
  }
  imuSpiWho = who;
  if (who != 0x68) return false;

  mpuSpiWrite(0x6B, 0x80);                                // device reset
  delay(100);
  mpuSpiWrite(0x68, 0x07);                                // signal path reset, asked for on SPI
  delay(100);
  mpuSpiWrite(0x6A, 0x10);                                // the reset turned the I2C interface on again
  if (!mpuSpiSet(0x6B, 0x01)) return false;               // wake, X gyro PLL
  delay(50);
  static const uint8_t config[][2] = {
    {0x6C, 0x00}, // all accelerometer and gyro axes enabled
    {0x19, 0x04}, // 1 kHz / (1 + 4) = 200 Hz
    {0x1A, 0x02}, // DLPF: accel 94 Hz, gyro 98 Hz
    {0x1B, 0x10}, // gyro +/-1000 dps
    {0x1C, 0x00}, // accel +/-2 g, 16384 LSB/g
    {0x37, 0x00}, // status clears on INT_STATUS read
    {0x38, 0x01}  // data-ready status; the INT wire is not needed
  };
  for (const auto &entry : config) {
    if (!mpuSpiSet(entry[0], entry[1])) return false;
  }
  delay(50);
  mpuSpiReady = true;
  return true;
}

static bool mpuSpiUpdate() {
  imu.ok = 0;
  if (!mpuSpiReady) return false;

  // INT_STATUS, accel XYZ, temperature, gyro XYZ (big endian)
  uint8_t data[15];
  mpuSpiRead(0x3A, data, sizeof(data));
  bool allOnes = true, allZeros = true;
  for (uint8_t b : data) {
    allOnes = allOnes && b == 0xFF;
    allZeros = allZeros && b == 0x00;
  }
  if (allOnes || allZeros) return false;                  // nobody is driving MISO: unplugged or unpowered
  if (!(data[0] & 0x01)) return false;                    // no fresh sample
  mpuPublish(data);
  return true;
}
