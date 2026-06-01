// StaticCovariances.h
// Static covariance matrices for each IMU sensor based on datasheet specs.
// These values are derived from sensor noise densities and ODR.
// For robot_localization: use these as conservative, static measurement variances.

#ifndef STATIC_COVARIANCES_H
#define STATIC_COVARIANCES_H

// ============================================================================
// 1. BMI088 (Bosch) - Static Covariances
// ============================================================================
// Based on: ~100 Hz ODR, slightly inflated for robustness per datasheet noise specs
// Linear acceleration: ~5.0e-4 (m/s^2)^2
// Angular velocity: ~1.0e-5 (rad/s)^2
struct BMI088_StaticCovariances
{
    static constexpr float ACCEL_VAR = 5.0e-4f; // (m/s^2)^2
    static constexpr float GYRO_VAR = 1.0e-5f;  // (rad/s)^2

    // Symmetric 3x3 matrices (diagonal only for simplicity)
    static constexpr float ACCEL_COV[9] = {
        ACCEL_VAR, 0.0f, 0.0f,
        0.0f, ACCEL_VAR, 0.0f,
        0.0f, 0.0f, ACCEL_VAR};

    static constexpr float GYRO_COV[9] = {
        GYRO_VAR, 0.0f, 0.0f,
        0.0f, GYRO_VAR, 0.0f,
        0.0f, 0.0f, GYRO_VAR};
};

// ============================================================================
// 2. LSM6DSOX (ST) - Static Covariances
// ============================================================================
// Based on: ~100 Hz ODR, high-performance mode, lower noise than BMI088
// Linear acceleration: ~3.0e-5 (m/s^2)^2
// Angular velocity: ~2.0e-7 (rad/s)^2
struct LSM6DSOX_StaticCovariances
{
    static constexpr float ACCEL_VAR = 3.0e-5f; // (m/s^2)^2
    static constexpr float GYRO_VAR = 2.0e-7f;  // (rad/s)^2

    static constexpr float ACCEL_COV[9] = {
        ACCEL_VAR, 0.0f, 0.0f,
        0.0f, ACCEL_VAR, 0.0f,
        0.0f, 0.0f, ACCEL_VAR};

    static constexpr float GYRO_COV[9] = {
        GYRO_VAR, 0.0f, 0.0f,
        0.0f, GYRO_VAR, 0.0f,
        0.0f, 0.0f, GYRO_VAR};
};

// ============================================================================
// 3. BNO085 (Bosch/CEVA) - Static Covariances
// ============================================================================
// Based on: fusion-based sensor with on-chip processing
// Orientation: ~9e-4 rad^2 (from ~1-2° RMS error)
// Angular velocity: ~1e-6 (rad/s)^2 (mid-grade MEMS-like scale)
// Linear acceleration: ~1e-4 (m/s^2)^2 (conservative for raw accel if available)
struct BNO085_StaticCovariances
{
    static constexpr float ACCEL_VAR = 1.0e-4f;  // (m/s^2)^2 - conservative for raw accel
    static constexpr float GYRO_VAR = 1.0e-6f;   // (rad/s)^2
    static constexpr float ORIENT_VAR = 9.0e-4f; // rad^2 (orientation/quaternion)

    static constexpr float ACCEL_COV[9] = {
        ACCEL_VAR, 0.0f, 0.0f,
        0.0f, ACCEL_VAR, 0.0f,
        0.0f, 0.0f, ACCEL_VAR};

    static constexpr float GYRO_COV[9] = {
        GYRO_VAR, 0.0f, 0.0f,
        0.0f, GYRO_VAR, 0.0f,
        0.0f, 0.0f, GYRO_VAR};

    static constexpr float ORIENT_COV[9] = {
        ORIENT_VAR, 0.0f, 0.0f,
        0.0f, ORIENT_VAR, 0.0f,
        0.0f, 0.0f, ORIENT_VAR};
};

#endif // STATIC_COVARIANCES_H
