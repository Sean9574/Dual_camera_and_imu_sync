// src/main.cpp
// XIAO RP2350 / ESP32-S3 IMU -> COBS+CRC16 binary telemetry at 100 Hz.
// Packet: [0x31][seq u16][sensor_id u8][flags u8][t_ms u32]
//         [quat w,x,y,z f32][gyro x,y,z f32][acc x,y,z f32]
//         [diag_ori 3*f32][diag_gyro 3*f32][diag_acc 3*f32][crc16 u16] + COBS + 0x00.
//
// flags: bit0 = orientation_valid, bit1 = camera_triggered (this sample fired GPIO5)
//
// HARDWARE CAMERA-IMU SYNC:
// The camera trigger fires on every 4th IMU sample (seq % 4 == 0 = 25 fps),
// immediately after the IMU sample is read, so the global-shutter exposure and
// that IMU sample coincide in hardware. The packet for that sample sets flags
// bit1, telling the host exactly which IMU sample the camera frame belongs to.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "IMUInterface.h"
#include "BNO085_IMU.h"
#include "LSM6DSOX_IMU.h"
#include "GroveBMI088_IMU.h"
#include "IMUCommon.h"
#include "CameraManager.h"

// Camera trigger fires every 4th IMU sample (100 Hz / 4 = 25 fps = 40 ms).
static const uint8_t CAMERA_TRIGGER_EVERY_N = 4;   // every 4th sample = 25 fps

static const uint8_t PKT_IMU_V1 = 0x31;

static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
    uint8_t *out_start = output;
    const uint8_t *in_end = input + length;
    uint8_t *code_ptr = output++;
    uint8_t code = 1;
    while (input < in_end)
    {
        if (*input == 0)
        {
            *code_ptr = code;
            code_ptr = output++;
            code = 1;
            ++input;
        }
        else
        {
            *output++ = *input++;
            if (++code == 0xFF)
            {
                *code_ptr = code;
                code_ptr = output++;
                code = 1;
            }
        }
    }
    *code_ptr = code;
    return (size_t)(output - out_start);
}

enum SensorId : uint8_t
{
    SID_NONE = 0,
    SID_LSM6DSOX = 1,
    SID_BMI088 = 2,
    SID_BNO085 = 3
};

static IMUInterface *imu = nullptr;
static LSM6DSOX_IMU imu_lsm6;
static GroveBMI088_IMU imu_bmi;
static BNO085_IMU imu_bno;
static SensorId detected = SID_NONE;

static bool begin_first_available()
{
    if (imu_bno.begin())
    {
        imu = &imu_bno;
        detected = SID_BNO085;
        Serial.println("IMU: BNO085 detected");
        return true;
    }
    if (imu_lsm6.begin())
    {
        imu = &imu_lsm6;
        detected = SID_LSM6DSOX;
        Serial.println("IMU: LSM6DSOX detected");
        return true;
    }
    if (imu_bmi.begin())
    {
        imu = &imu_bmi;
        detected = SID_BMI088;
        Serial.println("IMU: BMI088 detected");
        return true;
    }
    imu = nullptr;
    detected = SID_NONE;
    Serial.println("IMU: none detected");
    return false;
}

#pragma pack(push, 1)
struct ImuPacketV1
{
    uint8_t type;
    uint16_t seq;
    uint8_t sensor_id;
    uint8_t flags; // bit0: orientation_valid, bit1: camera_triggered
    uint32_t t_ms;
    float qw, qx, qy, qz;
    float gx, gy, gz; // rad/s
    float ax, ay, az; // m/s^2
    float cov_ori_x, cov_ori_y, cov_ori_z;
    float cov_gyr_x, cov_gyr_y, cov_gyr_z;
    float cov_acc_x, cov_acc_y, cov_acc_z;
    uint16_t crc;
};
#pragma pack(pop)

#if defined(__cplusplus) && (__cplusplus >= 201103L)
static_assert(sizeof(ImuPacketV1) == (1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2), "Packet size mismatch");
static_assert((sizeof(ImuPacketV1) * 2 < 256), "COBS buffer too small - increase buffer or reduce packet size");
#endif

static const uint32_t publish_hz = 100;
static const uint32_t publish_dt_ms = 1000 / publish_hz;

#ifndef ENABLE_CRC_INJECTION
#define ENABLE_CRC_INJECTION 0
#endif
static bool inject_crc = (ENABLE_CRC_INJECTION != 0);
static bool send_test_once = false;

static bool USE_STATIC_COVARIANCE = true;

static inline float fclampnan(float v) { return isfinite(v) ? v : 0.0f; }

// Camera trigger manager (USB OV9281 cameras), GPIO5 on RP2350.
static CameraManager camera;

static void process_commands(float ax, float ay, float az)
{
    int safety = 16;
    while (Serial.available() > 0 && safety-- > 0)
    {
        int c = Serial.read();
        if (c == 'T' || c == 't')
        {
            send_test_once = true;
            Serial.println("CMD: one-shot test frame requested");
        }
        else if (c == 'C' || c == 'c')
        {
            #if ENABLE_CRC_INJECTION
            inject_crc = !inject_crc;
            Serial.print("CMD: CRC injection ");
            Serial.println(inject_crc ? "ON" : "OFF");
            #else
            Serial.println("CMD: CRC injection disabled (compile with -D ENABLE_CRC_INJECTION=1)");
            #endif
        }
        else if (c == 'R' || c == 'r')
        {
            Serial.println("CMD: reset covariances (if supported)");
        }
        else if (c == 'D' || c == 'd')
        {
            Serial.print("ACC mps2: ");
            Serial.print(ax, 6);
            Serial.print(", ");
            Serial.print(ay, 6);
            Serial.print(", ");
            Serial.println(az, 6);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.println("IMU binary v1 ready (COBS+CRC16), 115200 baud");
    Serial.println("Commands: T=test, C=CRC toggle, D=accel");
    Serial.println("Camera trigger: every 4th sample (25 fps), flags bit1 marks triggered sample");
    Wire.begin();
    begin_first_available();

    if (imu && USE_STATIC_COVARIANCE)
    {
        imu->setStaticCovarianceMode(true);
        Serial.println("Static covariance mode: ENABLED");
    }

    // Initialize camera trigger pin on GPIO5 (RP2350), 20 us pulse.
    // Interval arg unused now (we fire manually on every Nth sample), kept for pin setup.
    camera.begin(5, 20, 40);
}

void loop()
{
    static uint16_t seq = 0;
    static uint32_t last_ms = 0;
    static bool first_run = true;

    const uint32_t now_ms = millis();

    if (first_run)
    {
        last_ms = now_ms;
        first_run = false;
    }

    if ((now_ms - last_ms) < publish_dt_ms)
    {
        process_commands(0.0f, 0.0f, 0.0f);
        delay(1);
        return;
    }
    last_ms = now_ms;

    if (!imu)
    {
        begin_first_available();
        if (!imu)
        {
            delay(10);
            return;
        }
    }

    // Does THIS sample fire the camera trigger? (seq is this packet's number)
    const bool trigger_this = (seq % CAMERA_TRIGGER_EVERY_N == 0);

    // 1) Read one fresh sample
    float raw_qw = 1.0f, raw_qx = 0.0f, raw_qy = 0.0f, raw_qz = 0.0f;
    float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;
    float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
    bool has_ori = false;

    if (imu)
    {
        imu->readSensorData();

        // Fire camera trigger IMMEDIATELY after reading this IMU sample, so the
        // global-shutter exposure and this sample coincide in hardware.
        if (trigger_this)
        {
            camera.sendTrigger();
        }

        has_ori = imu->hasOrientation();
        if (has_ori)
        {
            raw_qx = imu->getOrientationX();
            raw_qy = imu->getOrientationY();
            raw_qz = imu->getOrientationZ();
            raw_qw = imu->getOrientationW();
            float qn = sqrtf(raw_qw * raw_qw + raw_qx * raw_qx + raw_qy * raw_qy + raw_qz * raw_qz);
            const float QUAT_MIN_NORM = 1e-6f;
            if (qn > QUAT_MIN_NORM)
            {
                raw_qw /= qn;
                raw_qx /= qn;
                raw_qy /= qn;
                raw_qz /= qn;
            }
            else
            {
                raw_qw = 1.0f;
                raw_qx = 0.0f;
                raw_qy = 0.0f;
                raw_qz = 0.0f;
            }
        }
        raw_gx = imu->getGyroscopeX();
        raw_gy = imu->getGyroscopeY();
        raw_gz = imu->getGyroscopeZ();
        raw_ax = imu->getAccelerometerX();
        raw_ay = imu->getAccelerometerY();
        raw_az = imu->getAccelerometerZ();

        // 2) Only now compute covariances (do not touch the snapshot)
        imu->computeCovariances();

        raw_qw = fclampnan(raw_qw);
        raw_qx = fclampnan(raw_qx);
        raw_qy = fclampnan(raw_qy);
        raw_qz = fclampnan(raw_qz);
        raw_gx = fclampnan(raw_gx);
        raw_gy = fclampnan(raw_gy);
        raw_gz = fclampnan(raw_gz);
        raw_ax = fclampnan(raw_ax);
        raw_ay = fclampnan(raw_ay);
        raw_az = fclampnan(raw_az);
    }

    process_commands(raw_ax, raw_ay, raw_az);

    // Build packet
    ImuPacketV1 p{};
    p.type = PKT_IMU_V1;
    p.seq = seq++;
    p.sensor_id = (uint8_t)detected;
    p.flags = (has_ori ? 0x01 : 0x00) | (trigger_this ? 0x02 : 0x00);
    p.t_ms = now_ms;

    p.qw = raw_qw;
    p.qx = raw_qx;
    p.qy = raw_qy;
    p.qz = raw_qz;
    p.gx = raw_gx;
    p.gy = raw_gy;
    p.gz = raw_gz;
    p.ax = raw_ax;
    p.ay = raw_ay;
    p.az = raw_az;

    const float *covG = (imu != nullptr) ? imu->getGyroCovMatrix() : nullptr;
    const float *covA = (imu != nullptr) ? imu->getAccelCovMatrix() : nullptr;
    const float *covO = (imu != nullptr && has_ori) ? imu->getOrientationCovMatrix() : nullptr;

    p.cov_gyr_x = covG ? fclampnan(covG[0]) : 0.0f;
    p.cov_gyr_y = covG ? fclampnan(covG[4]) : 0.0f;
    p.cov_gyr_z = covG ? fclampnan(covG[8]) : 0.0f;

    p.cov_acc_x = covA ? fclampnan(covA[0]) : 0.0f;
    p.cov_acc_y = covA ? fclampnan(covA[4]) : 0.0f;
    p.cov_acc_z = covA ? fclampnan(covA[8]) : 0.0f;

    if (covO)
    {
        p.cov_ori_x = fclampnan(covO[0]);
        p.cov_ori_y = fclampnan(covO[4]);
        p.cov_ori_z = fclampnan(covO[8]);
    }

    const size_t payload_len_wo_crc = sizeof(ImuPacketV1) - sizeof(uint16_t);
    p.crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(&p), payload_len_wo_crc);
    if (inject_crc && (p.seq % 100 == 0))
    {
        p.crc ^= 0xFFFF;
    }

    uint8_t enc[256];
    const size_t enc_len = cobs_encode(reinterpret_cast<const uint8_t *>(&p), sizeof(ImuPacketV1), enc);
    Serial.write(enc, enc_len);
    Serial.write((uint8_t)0x00);
}
