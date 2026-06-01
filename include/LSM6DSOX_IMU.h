// LSM6DSOX IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef LSM6DSOX_IMU_H
#define LSM6DSOX_IMU_H

#include <Arduino.h>
#include <Adafruit_LSM6DSOX.h> // Specific for LSM6DSOX
#include <Wire.h>
#include "IMUInterface.h"
#include "IMUCommon.h"
#include "StaticCovariances.h"
#include <string.h>

class LSM6DSOX_IMU : public IMUInterface
{
public:
    LSM6DSOX_IMU() : latestAccelerometerX(0.0), latestAccelerometerY(0.0), latestAccelerometerZ(0.0),
                     latestGyroscopeX(0.0), latestGyroscopeY(0.0), latestGyroscopeZ(0.0),
                     latestTemperature(0.0), latestTimestampMilliseconds(0),
                     sampleCount(0), staticCovarianceMode(false)
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        // Initialize static covariance matrices
        memcpy(accelCovMatrix, LSM6DSOX_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
        memcpy(gyroCovMatrix, LSM6DSOX_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
    }

    bool begin() override
    {
        Serial.println("LSM6DSOX: init -> calling begin_I2C()");

        // Wire.begin() is idempotent on most Arduino platforms (calling multiple times is safe).
        // It's already called in main.cpp, but we ensure it here for robustness in case this
        // driver is used standalone. Note: some custom boards may require synchronized Wire setup.
        Wire.begin();

        bool ok = sensorInstance.begin_I2C(); // Init I2C
        if (!ok)
        {
            Serial.println("LSM6DSOX: begin_I2C() returned false");
            Serial.println("LSM6DSOX: Scanning I2C bus for devices...");

            // Simple I2C scanner to list any responding addresses.
            for (uint8_t addr = 1; addr < 127; ++addr)
            {
                Wire.beginTransmission(addr);
                uint8_t err = Wire.endTransmission();
                if (err == 0)
                {
                    Serial.print("  Found device at 0x");
                    if (addr < 16)
                        Serial.print('0');
                    Serial.println(addr, HEX);
                }
            }
        }
        else
        {
            // Configure covariance accumulator defaults
            accelAccumulator.setWindowSize(200);
            gyroAccumulator.setWindowSize(200);
            accelAccumulator.setVarianceEpsilon(1e-9f);
            gyroAccumulator.setVarianceEpsilon(1e-9f);
        }

        return ok;
    }

    void readSensorData() override
    {
        sensors_event_t accelEvent, gyroEvent, tempEvent;
        sensorInstance.getEvent(&accelEvent, &gyroEvent, &tempEvent);

        latestAccelerometerX = accelEvent.acceleration.x;
        latestAccelerometerY = accelEvent.acceleration.y;
        latestAccelerometerZ = accelEvent.acceleration.z;

        latestGyroscopeX = gyroEvent.gyro.x;
        latestGyroscopeY = gyroEvent.gyro.y;
        latestGyroscopeZ = gyroEvent.gyro.z;

        latestTemperature = tempEvent.temperature;

        latestTimestampMilliseconds = millis();

        // Add filtered/raw samples to accumulators
        accelAccumulator.addSample(latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ);
        gyroAccumulator.addSample(latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ);
        sampleCount++;
    }

    float getAccelerometerX() const override { return latestAccelerometerX; }
    float getAccelerometerY() const override { return latestAccelerometerY; }
    float getAccelerometerZ() const override { return latestAccelerometerZ; }
    float getGyroscopeX() const override { return latestGyroscopeX; }
    float getGyroscopeY() const override { return latestGyroscopeY; }
    float getGyroscopeZ() const override { return latestGyroscopeZ; }
    float getTemperature() const override { return latestTemperature; }

    // Orientation not provided by LSM6DSOX (no magnetometer/quaternion).
    // Provide default quaternion (identity) so this class satisfies the
    // IMUInterface contract. Drivers that can supply orientation should
    // override hasOrientation() to return true and provide values.
    float getOrientationX() const override { return 0.0f; }
    float getOrientationY() const override { return 0.0f; }
    float getOrientationZ() const override { return 0.0f; }
    float getOrientationW() const override { return 1.0f; }
    bool hasOrientation() const override { return false; }
    const float *getOrientationCovMatrix() const override { return nullptr; }

    void computeCovariances() override
    {
        if (!staticCovarianceMode)
        {
            // Compute covariance matrices using the unbiased sample estimator
            // (cov = Σ(x-mean)(y-mean) / (n-1)). The accumulator uses sum and
            // cross-product accumulators to compute this efficiently.
            accelAccumulator.computeCovMatrix(accelCovMatrix);
            gyroAccumulator.computeCovMatrix(gyroCovMatrix);
        }
        // If staticCovarianceMode is true, keep the static values already in the matrices
    }

    float getAccelerometerCovariance() const override { return accelCovMatrix[0]; } // Example: return Var(X); update if needed
    float getGyroscopeCovariance() const override { return gyroCovMatrix[0]; }      // Example: return Var(X)

    // New getters for full matrices (used in JSON formatter)
    const float *getAccelCovMatrix() const { return accelCovMatrix; }
    const float *getGyroCovMatrix() const { return gyroCovMatrix; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    // Optional: Reset accumulators to prevent overflow in long runs
    void resetCovarianceAccumulators()
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        // Re-initialize static values
        memcpy(accelCovMatrix, LSM6DSOX_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
        memcpy(gyroCovMatrix, LSM6DSOX_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
    }

    // Static covariance mode control
    void setStaticCovarianceMode(bool enabled) override
    {
        staticCovarianceMode = enabled;
        if (enabled)
        {
            // Switch to static values
            memcpy(accelCovMatrix, LSM6DSOX_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
            memcpy(gyroCovMatrix, LSM6DSOX_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
        }
    }

    bool getStaticCovarianceMode() const override
    {
        return staticCovarianceMode;
    }

private:
    Adafruit_LSM6DSOX sensorInstance;

    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    // Accumulators for covariance
    // Accumulators (use helper to avoid duplicated code)
    CovarianceAccumulator accelAccumulator;
    CovarianceAccumulator gyroAccumulator;
    int sampleCount;

    // Covariance matrices (3x3, row-major)
    float accelCovMatrix[9];
    float gyroCovMatrix[9];

    // Static covariance mode flag
    bool staticCovarianceMode;
};

#endif // LSM6DSOX_IMU_H
