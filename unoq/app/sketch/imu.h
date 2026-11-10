#pragma once

#include <Arduino.h>
#include <Wire.h>

// IMU, found at start-up in this order:
//   MPU6000 on SPI (SCK D13, MISO D12, MOSI D11, CS D10), WHO_AM_I 0x68        -> mpu6000_spi.h
//   MPU6000 / MPU6050 on Wire (D20/SDA, D21/SCL) at 0x68 or 0x69 (AD0)         -> mpu6050.h
//   QMI8658 on Wire at 0x6A or 0x6B, WHO_AM_I 0x05 (the XGO Duck board)        -> qmi8658.h
// An MPU is probed first, so wiring one in replaces the on-board sensor. All drivers publish the
// same raw units (accelerometer 16384 LSB/g, gyro 32 LSB/dps) with the XGO axis convention, so the
// state packet is the same whichever sensor answers.

struct ImuSample {
  uint8_t ok;
  int16_t accRaw[3];
  int16_t gyroRaw[3];
};

struct ImuSnapshot {
  ImuSample sample;
  uint32_t seq, sampleUs, periodUs, workUs, overruns, errors, initAttempts;
};

enum ImuKind : uint8_t { IMU_NONE = 0, IMU_QMI8658 = 1, IMU_MPU60X0 = 2, IMU_MPU6000_SPI = 3 };

static ImuSample imu = {};
static volatile uint8_t imuKind = IMU_NONE;   // sensor in use
static volatile uint8_t imuAddr = 0;          // its I2C address
// what answered WHO_AM_I at the MPU addresses on the last probe, 0 if nothing did: a device that
// is wired but is not an MPU6000/6050 shows up here
static volatile uint8_t imuMpuWho = 0;
// what came back as WHO_AM_I on the SPI header: 0x68 an MPU6000, 0xFF nothing wired
static volatile uint8_t imuSpiWho = 0;

// When no IMU answers, the two I2C buses a breakout can be plugged into are scanned, so that the
// host can tell "nothing wired" from "wired somewhere else / other address / not an MPU":
//   bus 0 = Wire  (SDA/D20, SCL/D21, the pins above AREF)     bus 1 = Wire1 (Qwiic connector)
constexpr uint8_t IMU_SCAN_MAX = 6;
static volatile uint8_t imuScanFlags = 0;               // bit 0/2: bus 0/1 scanned, bit 1/3: bus 0/1 not usable
static volatile uint8_t imuScanCount[2] = {0, 0};       // devices that answered
static volatile uint8_t imuScanAddr[2][IMU_SCAN_MAX] = {};

#include "qmi8658.h"
#include "mpu6050.h"
#include "mpu6000_spi.h"

static void imuScanBus(arduino::ZephyrI2C &bus, uint8_t index) {
  uint8_t n = 0;
  bool stuck = false;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    const uint32_t t0 = micros();
    const bool present = bus.requestFrom((int)addr, 1) == 1;
    while (bus.available()) (void)bus.read();
    if (present) {
      if (n < IMU_SCAN_MAX) imuScanAddr[index][n] = addr;
      ++n;
    } else if (micros() - t0 > 20000u) {
      stuck = true;       // an address that nobody acknowledges fails at once on a healthy bus
      break;
    }
  }
  imuScanCount[index] = n;
  imuScanFlags = (imuScanFlags & ~(3u << (2 * index))) | ((1u | (stuck ? 2u : 0u)) << (2 * index));
}

static void imuScan() {
  static bool qwiicStarted = false;
  imuScanBus(Wire, 0);
  if (!qwiicStarted) { Wire1.begin(); Wire1.setClock(400000); qwiicStarted = true; }
  imuScanBus(Wire1, 1);
}

static bool initImu() {
  static uint8_t failures = 0;
  Wire.begin();
  Wire.setClock(400000);
  delay(20);
  imu = {};
  imuKind = IMU_NONE;
  imuAddr = 0;
  if (mpuSpiInit()) {
    imuKind = IMU_MPU6000_SPI;
    imuAddr = MPU_SPI_CS;
  } else if (mpuInit()) {
    imuKind = IMU_MPU60X0;
    imuAddr = mpuAddr;
  } else if (qmiInit()) {
    imuKind = IMU_QMI8658;
    imuAddr = qmiAddr;
  }
  if (imuKind != IMU_NONE) {
    failures = 0;
    return true;
  }
  if ((failures++ % 5) == 0) imuScan();     // at the first failure, then every 5 s
  return false;
}

static bool updateImu() {
  switch (imuKind) {
    case IMU_MPU6000_SPI: return mpuSpiUpdate();
    case IMU_MPU60X0: return mpuUpdate();
    case IMU_QMI8658: return qmiUpdate();
    default: imu.ok = 0; return false;
  }
}

// after repeated read errors: probe again from scratch
static void forgetImu() {
  imuKind = IMU_NONE;
  imuAddr = 0;
  qmiAddr = 0;
  mpuAddr = 0;
  mpuReady = false;
  mpuSpiReady = false;
}
