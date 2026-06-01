// Formatter for ROS IMU JSON.
// All code inline for header-only.

#ifndef IMU_MESSAGE_FORMATTER_H
#define IMU_MESSAGE_FORMATTER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "IMUInterface.h"

class IMUMessageFormatter
{
public:
    static String formatAsRosImuJson(const IMUInterface &imu)
    {
        JsonDocument doc;

        unsigned long ts = imu.getTimestampMilliseconds();
        doc["header"]["stamp"]["secs"] = ts / 1000;
        doc["header"]["stamp"]["nsecs"] = (ts % 1000) * 1000000;
        doc["header"]["frame_id"] = "imu_link";

        // Orientation (from game rotation vector)
        // Only populate orientation fields if the IMU reports having orientation.
        // Some sensors (e.g., raw gyro/accel-only) won't supply orientation.
        const auto &orient = doc["orientation"].to<JsonObject>();
        if (imu.hasOrientation()) {
            orient["x"] = imu.getOrientationX();
            orient["y"] = imu.getOrientationY();
            orient["z"] = imu.getOrientationZ();
            orient["w"] = imu.getOrientationW();

            // Orientation covariance: follow REP 145.
            // - If the driver provides a 3x3 orientation covariance matrix, use it.
            // - If the driver supports orientation but does not provide covariance,
            //   populate with zeros (meaningfully low covariance not specified).
            const auto &orientCov = doc["orientation_covariance"].to<JsonArray>();
            const float *orientMatrix = imu.getOrientationCovMatrix();
            if (orientMatrix) {
                for (int i = 0; i < 9; ++i)
                    orientCov.add(orientMatrix[i]);
            } else {
                // Known orientation but no covariance provided: set zeros per REP145
                for (int i = 0; i < 9; ++i)
                    orientCov.add(0.0);
            }
        } else {
            // IMU doesn't provide orientation. Per REP 145 the orientation covariance
            // should be set to [-1, 0, 0, 0, 0, 0, 0, 0, 0] to indicate orientation is
            // unknown / not provided.
            const auto &orientCov = doc["orientation_covariance"].to<JsonArray>();
            orientCov.add(-1.0);
            for (int i = 1; i < 9; ++i)
                orientCov.add(0.0);
        }

        // Linear acceleration
        const auto &linAccel = doc["linear_acceleration"].to<JsonObject>();
        linAccel["x"] = imu.getAccelerometerX();
        linAccel["y"] = imu.getAccelerometerY();
        linAccel["z"] = imu.getAccelerometerZ();

        const auto &linCov = doc["linear_acceleration_covariance"].to<JsonArray>();
        const float *accelMatrix = imu.getAccelCovMatrix();
        // Protective clamp: ensure diagonal elements are non-negative and not NaN.
        for (int i = 0; i < 9; ++i) {
            float v = accelMatrix[i];
            if (i == 0 || i == 4 || i == 8) {
                if (!isfinite(v) || v < 0.0f) v = 0.0f;
            } else {
                if (!isfinite(v)) v = 0.0f;
            }
            linCov.add(v);
        }

        // Angular velocity
        const auto &angVel = doc["angular_velocity"].to<JsonObject>();
        angVel["x"] = imu.getGyroscopeX();
        angVel["y"] = imu.getGyroscopeY();
        angVel["z"] = imu.getGyroscopeZ();

        const auto &angCov = doc["angular_velocity_covariance"].to<JsonArray>();
        const float *gyroMatrix = imu.getGyroCovMatrix();
        for (int i = 0; i < 9; ++i) {
            float v = gyroMatrix[i];
            if (i == 0 || i == 4 || i == 8) {
                if (!isfinite(v) || v < 0.0f) v = 0.0f;
            } else {
                if (!isfinite(v)) v = 0.0f;
            }
            angCov.add(v);
        }

        String json;
        serializeJson(doc, json);
        return json;
    }
};

#endif // IMU_MESSAGE_FORMATTER_H
