// BNO085 IMU implementation.
// Inherits from IMUInterface. All code inline for header-only.

#ifndef BNO085_IMU_H
#define BNO085_IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h> // Adafruit library for BNO085/BNO08x
#include "IMUInterface.h"
#include "IMUCommon.h"
#include "StaticCovariances.h"
#include <string.h> // For memset

class BNO085_IMU : public IMUInterface
{
public:
    BNO085_IMU() : latestAccelerometerX(0.0f), latestAccelerometerY(0.0f), latestAccelerometerZ(0.0f),
                   latestGyroscopeX(0.0f), latestGyroscopeY(0.0f), latestGyroscopeZ(0.0f),
                   latestTemperature(0.0f), latestTimestampMilliseconds(0),
                   latestOrientationX(0.0f), latestOrientationY(0.0f), latestOrientationZ(0.0f), latestOrientationW(1.0f),
                   sampleCount(0), staticCovarianceMode(false)
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        orientAccumulator.reset();
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        memset(orientCovMatrix, 0, sizeof(orientCovMatrix));
        // Initialize static covariance matrices
        memcpy(accelCovMatrix, BNO085_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
        memcpy(gyroCovMatrix, BNO085_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
        memcpy(orientCovMatrix, BNO085_StaticCovariances::ORIENT_COV, sizeof(orientCovMatrix));
    }

    bool begin() override
    {
        if (!sensorInstance.begin_I2C())
        { // Default address 0x4A; change to 0x4B if jumpered
            return false;
        }
        // Enable required reports
        sensorInstance.enableReport(SH2_LINEAR_ACCELERATION);
        sensorInstance.enableReport(SH2_GYROSCOPE_CALIBRATED);
        
        // SH2_ARVR_STABILIZED_RV (vs SH2_GAME_ROTATION_VECTOR):
        // =========================================================
        // ARVR provides superior performance for robotics/odometry applications:
        //  - 9-DOF fusion (Gyro + Accel + Magnetometer) vs Gyro+Accel only
        //  - Real-time stabilized update rate (~200Hz vs ~33-50Hz for GAME mode)
        //  - Magnetometer provides absolute heading reference for drift correction
        //  - Built-in Fast Magnetic Calibration (FMC) - auto-calibrates at boot & during operation
        //
        // Magnetometer Calibration Notes:
        //  - BNO085 performs automatic calibration during startup (500ms delay below)
        //  - Continuous online calibration during normal operation
        //  - Status byte available in sensorValue.status indicates calibration quality (0-3, where 3=fully calibrated)
        //  - For stable mounting (under lidar): calibration converges quickly, then remains stable
        //  - Magnetic field distortions (metal nearby) are handled by FMC algorithm
        //
        // Application Suitability (Educational Robot Odometry):
        //  - Ideal for flat-surface navigation (parking lot, slopes)
        //  - Heading stability prevents gyro drift accumulation
        //  - 200Hz quaternions exceed typical lidar/camera update rates for fusion
        
        sensorInstance.enableReport(SH2_ARVR_STABILIZED_RV, 5000);  // 200Hz
        sensorInstance.enableReport(SH2_TEMPERATURE);
        delay(500); // Allow initial calibration/stabilization (FMC runs here)

        // Configure covariance accumulator defaults
        accelAccumulator.setWindowSize(200);
        gyroAccumulator.setWindowSize(200);
        orientAccumulator.setWindowSize(200);
        accelAccumulator.setVarianceEpsilon(1e-9f);
        gyroAccumulator.setVarianceEpsilon(1e-9f);
        orientAccumulator.setVarianceEpsilon(1e-9f);
        return true;
    }

    void readSensorData() override
    {
        sh2_SensorValue_t sensorValue;

        // Poll for new data (BNO08x updates asynchronously)
        while (sensorInstance.getSensorEvent(&sensorValue))
        {
            switch (sensorValue.sensorId)
            {
            case SH2_LINEAR_ACCELERATION:
                latestAccelerometerX = sensorValue.un.linearAcceleration.x;
                latestAccelerometerY = sensorValue.un.linearAcceleration.y;
                latestAccelerometerZ = sensorValue.un.linearAcceleration.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                latestGyroscopeX = sensorValue.un.gyroscope.x;
                latestGyroscopeY = sensorValue.un.gyroscope.y;
                latestGyroscopeZ = sensorValue.un.gyroscope.z;
                break;
            case SH2_ARVR_STABILIZED_RV:
                latestOrientationX = sensorValue.un.arvrStabilizedRV.i;
                latestOrientationY = sensorValue.un.arvrStabilizedRV.j;
                latestOrientationZ = sensorValue.un.arvrStabilizedRV.k;
                latestOrientationW = sensorValue.un.arvrStabilizedRV.real;
                break;
            case SH2_TEMPERATURE:
                latestTemperature = sensorValue.un.temperature.value;
                break;
            default:
                break;
            }
        }

        latestTimestampMilliseconds = millis();

        // Accumulate sensor and orientation samples
        accelAccumulator.addSample(latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ);
        gyroAccumulator.addSample(latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ);
        // Track the vector part (i,j,k) of the quaternion for simple covariance
        orientAccumulator.addSample(latestOrientationX, latestOrientationY, latestOrientationZ);
        sampleCount++;
    }

    float getAccelerometerX() const override { return latestAccelerometerX; }
    float getAccelerometerY() const override { return latestAccelerometerY; }
    float getAccelerometerZ() const override { return latestAccelerometerZ; }
    float getGyroscopeX() const override { return latestGyroscopeX; }
    float getGyroscopeY() const override { return latestGyroscopeY; }
    float getGyroscopeZ() const override { return latestGyroscopeZ; }
    float getTemperature() const override { return latestTemperature; }
    float getOrientationX() const override { return latestOrientationX; }
    float getOrientationY() const override { return latestOrientationY; }
    float getOrientationZ() const override { return latestOrientationZ; }
    float getOrientationW() const override { return latestOrientationW; }

    void computeCovariances() override
    {
        if (!staticCovarianceMode)
        {
            // Compute covariance matrices using the unbiased sample estimator:
            //   mean_x = (1/n) * Σ x_i
            //   cov_xy = (1/(n-1)) * Σ (x_i - mean_x) (y_i - mean_y)
            // Implementation uses sum and cross-product accumulators for numerical efficiency.
            accelAccumulator.computeCovMatrix(accelCovMatrix);
            gyroAccumulator.computeCovMatrix(gyroCovMatrix);
            // Orientation covariance based on quaternion vector part (i,j,k). If there are
            // insufficient samples orientCovMatrix will be zeroed by the accumulator.
            orientAccumulator.computeCovMatrix(orientCovMatrix);
        }
        // If staticCovarianceMode is true, keep the static values already in the matrices
    }

    float getAccelerometerCovariance() const override { return accelCovMatrix[0]; } // Example: return Var(X); update if needed
    float getGyroscopeCovariance() const override { return gyroCovMatrix[0]; }      // Example: return Var(X)

    // New getters for full matrices (used in JSON formatter)
    const float *getAccelCovMatrix() const override { return accelCovMatrix; }
    const float *getGyroCovMatrix() const override { return gyroCovMatrix; }
    // Orientation is provided by this sensor (game rotation vector). Indicate support.
    bool hasOrientation() const override { return true; }
    // Provide a 3x3 orientation covariance matrix computed over the quaternion
    // vector part (i,j,k). This is a pragmatic choice: many drivers do not provide
    // a full orientation covariance; using the vector part gives some notion of
    // variability while remaining simple.
    const float *getOrientationCovMatrix() const override { return orientCovMatrix; }

    unsigned long getTimestampMilliseconds() const override { return latestTimestampMilliseconds; }

    // Optional: Reset accumulators to prevent overflow in long runs
    void resetCovarianceAccumulators()
    {
        accelAccumulator.reset();
        gyroAccumulator.reset();
        orientAccumulator.reset();
        sampleCount = 0;
        memset(accelCovMatrix, 0, sizeof(accelCovMatrix));
        memset(gyroCovMatrix, 0, sizeof(gyroCovMatrix));
        memset(orientCovMatrix, 0, sizeof(orientCovMatrix));
        // Re-initialize static values
        memcpy(accelCovMatrix, BNO085_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
        memcpy(gyroCovMatrix, BNO085_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
        memcpy(orientCovMatrix, BNO085_StaticCovariances::ORIENT_COV, sizeof(orientCovMatrix));
    }

    // Static covariance mode control
    void setStaticCovarianceMode(bool enabled) override
    {
        staticCovarianceMode = enabled;
        if (enabled)
        {
            // Switch to static values
            memcpy(accelCovMatrix, BNO085_StaticCovariances::ACCEL_COV, sizeof(accelCovMatrix));
            memcpy(gyroCovMatrix, BNO085_StaticCovariances::GYRO_COV, sizeof(gyroCovMatrix));
            memcpy(orientCovMatrix, BNO085_StaticCovariances::ORIENT_COV, sizeof(orientCovMatrix));
        }
    }

    bool getStaticCovarianceMode() const override
    {
        return staticCovarianceMode;
    }

private:
    Adafruit_BNO08x sensorInstance;

    float latestAccelerometerX, latestAccelerometerY, latestAccelerometerZ;
    float latestGyroscopeX, latestGyroscopeY, latestGyroscopeZ;
    float latestTemperature;
    unsigned long latestTimestampMilliseconds;

    float latestOrientationX, latestOrientationY, latestOrientationZ, latestOrientationW;

    // Accumulators for covariance (reuse common helper)
    CovarianceAccumulator accelAccumulator;
    CovarianceAccumulator gyroAccumulator;
    // Track quaternion vector part for orientation covariance (i,j,k)
    CovarianceAccumulator orientAccumulator;
    int sampleCount;

    // Covariance matrices (3x3, row-major)
    float accelCovMatrix[9];
    float gyroCovMatrix[9];
    float orientCovMatrix[9];

    // Static covariance mode flag
    bool staticCovarianceMode;
};

#endif // BNO085_IMU_H
