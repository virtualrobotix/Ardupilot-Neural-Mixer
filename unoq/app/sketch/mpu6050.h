#pragma once

// MPU6000 / MPU6050 on I2C, from the "XGO Duck MPU6050 Operativo" package (driver written there
// as a drop-in for qmi8658.h and verified on the Uno Q on 25 October 2026). Included by imu.h.
// AD0 = GND selects 0x68, AD0 = 3V3 selects 0x69.
// Mount with sensor +X/+Y/+Z aligned to the original QMI8658.

static uint8_t mpuAddr = 0;
static bool mpuReady = false;

// Optional measured stationary offsets, in native MPU counts.
// These are NOT mounting-angle corrections.
static const int16_t GYRO_BIAS_RAW[3] = {0, 0, 0};

static bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, (int)size) != size) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < size; ++i) data[i] = (uint8_t)Wire.read();
  return true;
}

static int16_t mpuSignedBE(const uint8_t *p) {
  const uint16_t u = ((uint16_t)p[0] << 8) | p[1];
  return (int16_t)((u & 0x8000u) ? (int32_t)u - 65536 : u);
}

static int16_t mpuClamp(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static int16_t mpuGyro(int32_t raw) {
  // MPU at +/-1000 dps: 32.8 LSB/dps; XGO expects 32.
  const int32_t n = raw * 40;
  return mpuClamp((n >= 0 ? n + 20 : n - 20) / 41);
}

// INT_STATUS, accel XYZ, temperature, gyro XYZ as read from 0x3A -> imu, for both transports
static void mpuPublish(const uint8_t *data) {
  const int32_t ax = mpuSignedBE(data + 1);
  const int32_t ay = mpuSignedBE(data + 3);
  const int32_t az = mpuSignedBE(data + 5);
  const int32_t gx = mpuSignedBE(data + 9)  - GYRO_BIAS_RAW[0];
  const int32_t gy = mpuSignedBE(data + 11) - GYRO_BIAS_RAW[1];
  const int32_t gz = mpuSignedBE(data + 13) - GYRO_BIAS_RAW[2];

  // Preserve the EXISTING XGO protocol convention exactly.
  // The Linux code applies its additional body-frame mapping.
  imu.accRaw[0] = mpuClamp(az);
  imu.accRaw[1] = mpuClamp(ax);
  imu.accRaw[2] = mpuClamp(-ay);
  imu.gyroRaw[0] = mpuGyro(-gz);
  imu.gyroRaw[1] = mpuGyro(-gx);
  imu.gyroRaw[2] = mpuGyro(gy);
  imu.ok = 1;
}

static bool mpuInit() {
  mpuReady = false;
  mpuAddr = 0;
  imuMpuWho = 0;

  for (uint8_t addr = 0x68; addr <= 0x69; ++addr) {
    mpuAddr = addr;
    uint8_t who = 0;
    if (mpuRead(0x75, &who, 1)) {
      imuMpuWho = who;
      if (who == 0x68) break;
    }
    mpuAddr = 0;
  }
  if (!mpuAddr) return false;

  if (!mpuWrite(0x6B, 0x80)) { mpuAddr = 0; return false; } // Device reset
  delay(100);
  if (!mpuWrite(0x6B, 0x01)) { mpuAddr = 0; return false; } // Wake, X gyro PLL
  delay(50);

  static const uint8_t config[][2] = {
    {0x6C, 0x00}, // All accelerometer and gyro axes enabled
    {0x6A, 0x00}, // DMP, FIFO and auxiliary I2C master off
    {0x23, 0x00}, // No FIFO sources
    {0x19, 0x04}, // 1 kHz / (1 + 4) = 200 Hz
    {0x1A, 0x02}, // DLPF: accel 94 Hz, gyro 98 Hz
    {0x1B, 0x10}, // Gyro +/-1000 dps
    {0x1C, 0x00}, // Accel +/-2 g, 16384 LSB/g
    {0x37, 0x00}, // Status clears on INT_STATUS read
    {0x38, 0x01}  // Enable data-ready status; INT wire not needed
  };
  for (const auto &entry : config) {
    uint8_t value = 0;
    if (!mpuWrite(entry[0], entry[1]) ||
        !mpuRead(entry[0], &value, 1) || value != entry[1]) { mpuAddr = 0; return false; }
  }
  uint8_t power = 0;
  if (!mpuRead(0x6B, &power, 1) || power != 0x01) { mpuAddr = 0; return false; }
  delay(100);
  mpuReady = true;
  // Healthy only after mpuUpdate() obtains an actual sample.
  return true;
}

static bool mpuUpdate() {
  imu.ok = 0;
  if (!mpuReady) return false;

  // INT_STATUS, accel XYZ, temperature, gyro XYZ (big endian).
  uint8_t data[15];
  if (!mpuRead(0x3A, data, sizeof(data))) return false; // the IMU thread counts errors and re-probes
  if (!(data[0] & 0x01)) return false; // No fresh sample

  mpuPublish(data);
  return true;
}
