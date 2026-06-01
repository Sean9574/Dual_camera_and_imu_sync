// include/GroveBMI088_IMU.h
// Grove BMI088 IMU implementation with explicit BMI088->ENU axis mapping.
// Library frame per comment: right-handed, Z positive DOWN; we map to ENU: X fwd, Y left, Z up.

#ifndef GROVE_BMI088_IMU_H
#define GROVE_BMI088_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "IMUInterface.h"
#include "IMUCommon.h"
#include "StaticCovariances.h"
#include <string.h>

// Bolderflight BMI088 returns accel in m/s^2 and gyro in rad/s; library uses a right-handed frame with Z positive down.
// We expose a simple axis-sign mapping so the firmware publishes ENU by default (X=fwd, Y=left, Z=up).
class GroveBMI088_IMU : public IMUInterface
{
private:
    Bmi088 *bmi088; // dynamically allocated so we can probe addresses

public:
    GroveBMI088_IMU()
        : latestAccelerometerX(0.0f), latestAccelerometerY(0.0f), latestAccelerometerZ(0.0f),
          latestGyroscopeX(0.0f), latestGyroscopeY(0.0f), latestGyroscopeZ(0.0f),
          latestTemperature(0.0f), latestTimestampMilliseconds(0),
          offsetAccelX(0.0f), offsetAccelY(0.0f), offsetAccelZ(0.0f),
          offsetGyroX(0.0f), offsetGyroY(0.0f), offsetGyroZ(0.0f),
          sampleCount(0),
          // BMI088 (Bolderflight) is Z-down, Y-right; ENU needs Z-up, Y-left
          signAx(1.0f), signAy(-1.0f), signAz(-1.0f),
          signGx(1.0f), signGy(-1.0f), signGz(-1.0f),
          staticCovarianceMode(false)
    {
        bmi088 = new Bmi088(Wire, 0x18, 0x68);
        accelAccumulator.reset();
        gyroAccumulator.reset();
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        // Initialize static covariance matrices
        memcpy(accelCovMatrix, BMI088_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
        memcpy(gyroCovMatrix, BMI088_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
    }

    ~GroveBMI088_IMU()
    {
        if (bmi088)
        {
            delete bmi088;
            bmi088 = nullptr;
        }
    }

    // Optional: allow changing signs at runtime (e.g., via a simple serial command later)
    void setAccelSigns(float sx, float sy, float sz)
    {
        signAx = (sx >= 0 ? 1.0f : -1.0f);
        signAy = (sy >= 0 ? 1.0f : -1.0f);
        signAz = (sz >= 0 ? 1.0f : -1.0f);
    }
    void setGyroSigns(float sx, float sy, float sz)
    {
        signGx = (sx >= 0 ? 1.0f : -1.0f);
        signGy = (sy >= 0 ? 1.0f : -1.0f);
        signGz = (sz >= 0 ? 1.0f : -1.0f);
    }

    bool begin() override
    {
        // Wire.begin() is idempotent on most Arduino platforms. It's already called in main.cpp,
        // but called here for robustness in case this driver is used standalone.
        Wire.begin();

        // Probe common Grove address pairs. BMI088 has separate I2C addresses for accel and gyro.
        const uint8_t accel_addrs[] = {0x18, 0x19};
        const uint8_t gyro_addrs[] = {0x68, 0x69};

        Serial.println("BMI088: Starting probe for I2C addresses...");
        bool init_ok = false;
        for (size_t ai = 0; ai < sizeof(accel_addrs); ++ai)
        {
            for (size_t gi = 0; gi < sizeof(gyro_addrs); ++gi)
            {
                uint8_t a = accel_addrs[ai];
                uint8_t g = gyro_addrs[gi];
                Serial.print("BMI088: Trying accel=0x");
                Serial.print(a, HEX);
                Serial.print(" gyro=0x");
                Serial.println(g, HEX);

                delete bmi088;
                bmi088 = new Bmi088(Wire, a, g);
                int status = bmi088->begin();
                if (status >= 0)
                {
                    init_ok = true;
                    Serial.print("BMI088: Found device at accel=0x");
                    Serial.print(a, HEX);
                    Serial.print(" gyro=0x");
                    Serial.println(g, HEX);
                    break;
                }
                delay(10);
            }
            if (init_ok)
                break;
        }
        if (!init_ok)
        {
            Serial.println("BMI088: No device detected on I2C with common address pairs.");
            return false;
        }

        // Configure ranges and ODR to match ~100 Hz publishing rate
        bmi088->setRange(Bmi088::ACCEL_RANGE_24G, Bmi088::GYRO_RANGE_500DPS);
        bmi088->setOdr(Bmi088::ODR_400HZ);

        // Covariance accumulator defaults (rolling window)
        accelAccumulator.setWindowSize(200);
        gyroAccumulator.setWindowSize(200);
        accelAccumulator.setVarianceEpsilon(1e-9f);
        gyroAccumulator.setVarianceEpsilon(1e-9f);

        // Calibrate only gyro DC bias; keep accelerometer DC offsets at zero (preserve gravity)
        Serial.println("BMI088: Calibrating gyro offsets...");
        calibrateGyroOffsets();
        Serial.println("BMI088: Gyro calibration complete.");

        // Explicitly zero accel offsets to avoid gravity tampering
        offsetAccelX = 0.0f;
        offsetAccelY = 0.0f;
        offsetAccelZ = 0.0f;

        // Defaults already map BMI088 (Z-down/Y-right) -> ENU (Z-up/Y-left)
        // signAx=+1, signAy=-1, signAz=-1; signGx=+1, signGy=-1, signGz=-1

        return true;
    }

    void readSensorData() override
    {
        // Read synchronized sensor values
        bmi088->readSensor();

        float ax = bmi088->getAccelX_mss();
        float ay = bmi088->getAccelY_mss();
        float az = bmi088->getAccelZ_mss();

        // Apply sign mapping to ENU (X fwd, Y left, Z up)
        ax = signAx * ax;
        ay = signAy * ay;
        az = signAz * az;

        // Keep gravity; do NOT subtract accel DC offsets in firmware
        latestAccelerometerX = lowPassFilter(latestAccelerometerX, ax, 0.8f);
        latestAccelerometerY = lowPassFilter(latestAccelerometerY, ay, 0.8f);
        latestAccelerometerZ = lowPassFilter(latestAccelerometerZ, az, 0.8f);

        float gx = bmi088->getGyroX_rads();
        float gy = bmi088->getGyroY_rads();
        float gz = bmi088->getGyroZ_rads();

        // Subtract gyro DC bias only, then apply sign mapping to ENU
        gx = signGx * (gx - offsetGyroX);
        gy = signGy * (gy - offsetGyroY);
        gz = signGz * (gz - offsetGyroZ);

        latestGyroscopeX = lowPassFilter(latestGyroscopeX, gx, 0.8f);
        latestGyroscopeY = lowPassFilter(latestGyroscopeY, gy, 0.8f);
        latestGyroscopeZ = lowPassFilter(latestGyroscopeZ, gz, 0.8f);

        latestTemperature = bmi088->getTemperature_C();
        latestTimestampMilliseconds = millis();

        // Accumulate for covariance
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

    // BMI088 has no orientation output
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
            accelAccumulator.computeCovMatrix(accelCovMatrix);
            gyroAccumulator.computeCovMatrix(gyroCovMatrix);
        }
        // If staticCovarianceMode is true, keep the static values already in the matrices
    }

    float getAccelerometerCovariance() const override { return accelCovMatrix[0]; }
    float getGyroscopeCovariance() const override { return gyroCovMatrix[0]; }

    const float *getAccelCovMatrix() const override { return accelCovMatrix; }
    const float *getGyroCovMatrix() const override { return gyroCovMatrix; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    void resetCovarianceAccumulators()
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        // Re-initialize static values
        memcpy(accelCovMatrix, BMI088_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
        memcpy(gyroCovMatrix, BMI088_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
    }

    // Static covariance mode control
    void setStaticCovarianceMode(bool enabled) override
    {
        staticCovarianceMode = enabled;
        if (enabled)
        {
            // Switch to static values
            memcpy(accelCovMatrix, BMI088_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
            memcpy(gyroCovMatrix, BMI088_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
        }
    }

    bool getStaticCovarianceMode() const override
    {
        return staticCovarianceMode;
    }

    // Public for transparency
    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    float offsetAccelX, offsetAccelY, offsetAccelZ; // kept zero intentionally
    float offsetGyroX, offsetGyroY, offsetGyroZ;

    CovarianceAccumulator accelAccumulator;
    CovarianceAccumulator gyroAccumulator;
    int sampleCount;

    float accelCovMatrix[9];
    float gyroCovMatrix[9];

private:
    static float lowPassFilter(float prev, float curr, float alpha)
    {
        return prev * (1.0f - alpha) + curr * alpha;
    }

    void calibrateGyroOffsets()
    {
        offsetGyroX = offsetGyroY = offsetGyroZ = 0.0f;
        const int calSamples = 200;
        for (int i = 0; i < calSamples; ++i)
        {
            bmi088->readSensor();
            offsetGyroX += bmi088->getGyroX_rads();
            offsetGyroY += bmi088->getGyroY_rads();
            offsetGyroZ += bmi088->getGyroZ_rads();
            delay(5);
        }
        offsetGyroX /= calSamples;
        offsetGyroY /= calSamples;
        offsetGyroZ /= calSamples;

        // Leave accel offsets at zero so gravity is preserved in Imu
        offsetAccelX = 0.0f;
        offsetAccelY = 0.0f;
        offsetAccelZ = 0.0f;
    }

    // Axis sign mapping (BMI088 -> ENU)
    float signAx, signAy, signAz;
    float signGx, signGy, signGz;

    // Static covariance mode flag
    bool staticCovarianceMode;
};

#endif // GROVE_BMI088_IMU_H
