// Abstract base for IMU sensors.
// Common interface for data reading and covariances.

#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <Arduino.h>

class IMUInterface
{
public:
    virtual bool begin() = 0;
    virtual void readSensorData() = 0;
    virtual float getAccelerometerX() const = 0;
    virtual float getAccelerometerY() const = 0;
    virtual float getAccelerometerZ() const = 0;
    virtual float getGyroscopeX() const = 0;
    virtual float getGyroscopeY() const = 0;
    virtual float getGyroscopeZ() const = 0;
    virtual float getTemperature() const = 0;
    virtual void computeCovariances() = 0;
    virtual float getAccelerometerCovariance() const = 0;
    virtual float getGyroscopeCovariance() const = 0;
    virtual const float *getAccelCovMatrix() const = 0;
    virtual const float *getGyroCovMatrix() const = 0;
    virtual float getOrientationX() const = 0; // Quaternion X
    virtual float getOrientationY() const = 0; // Quaternion Y
    virtual float getOrientationZ() const = 0; // Quaternion Z
    virtual float getOrientationW() const = 0; // Quaternion W
    // Return true if this IMU implementation provides a valid orientation (quaternion).
    // Default: false so adding this method is non-breaking for existing drivers.
    virtual bool hasOrientation() const { return false; }
    // If the IMU can provide an orientation covariance matrix, return a pointer to
    // a 9-element row-major float array. Return nullptr if not available. Default
    // implementation returns nullptr to remain backward compatible.
    virtual const float *getOrientationCovMatrix() const { return nullptr; }
    virtual unsigned long getTimestampMilliseconds() const = 0;

    // Static covariance mode: if enabled, use fixed datasheet-based values instead of
    // computing covariances from sample statistics. Useful for robot_localization.
    // NOTE: Static mode assumes ideal operating conditions. If your IMUs are subject to
    // vibration, temperature drift, or other environmental factors, computed covariances
    // may be more appropriate. Use with caution in production systems where sensor behavior
    // may deviate from datasheet specs.
    // Default: false (use computed covariances)
    virtual void setStaticCovarianceMode(bool enabled) { (void)enabled; }
    virtual bool getStaticCovarianceMode() const { return false; }

    // TODO: Consider adding thermal monitoring in the future.
    // Sensors can degrade or fail if temperature exceeds specs. Applications should:
    //   - Call getTemperature() periodically and log warnings if T > 60°C
    //   - Implement thermal shutdown if T > 85°C (sensor-dependent)

    virtual ~IMUInterface() {}
};

#endif // IMU_INTERFACE_H
