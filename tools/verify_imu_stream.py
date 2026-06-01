# tools/verify_imu_stream.py (final)
# Robust IMU stream verifier for COBS+CRC frames from firmware.
# Requires: pip install pyserial cobs

import argparse
import math
import struct
import time
from collections import deque

import serial
from cobs import cobs

PKT_IMU_V1 = 0x31


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def parse_frame(payload: bytes):
    expected_len = 1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2
    if len(payload) != expected_len or payload[0] != PKT_IMU_V1:
        return None
    if struct.unpack_from("<H", payload, expected_len - 2)[0] != crc16_ccitt(payload[:-2]):
        return None
    off = 0
    (pkt_type, seq, sensor_id, flags, t_ms) = struct.unpack_from("<B H B B I", payload, off)
    off += 1 + 2 + 1 + 1 + 4
    (qw, qx, qy, qz) = struct.unpack_from("<4f", payload, off)
    off += 16
    (gx, gy, gz) = struct.unpack_from("<3f", payload, off)
    off += 12
    (ax, ay, az) = struct.unpack_from("<3f", payload, off)
    off += 12
    (cov_ori_x, cov_ori_y, cov_ori_z) = struct.unpack_from("<3f", payload, off)
    off += 12
    (cov_gx, cov_gy, cov_gz) = struct.unpack_from("<3f", payload, off)
    off += 12
    (cov_ax, cov_ay, cov_az) = struct.unpack_from("<3f", payload, off)
    off += 12
    return {
        "seq": seq,
        "sid": sensor_id,
        "flags": flags,
        "t_ms": t_ms,
        "quat": (qw, qx, qy, qz),
        "gyro": (gx, gy, gz),
        "acc": (ax, ay, az),
        "cov_o": (cov_ori_x, cov_ori_y, cov_ori_z),
        "cov_g": (cov_gx, cov_gy, cov_gz),
        "cov_a": (cov_ax, cov_ay, cov_az),
    }


def var_diag(samples):
    n = len(samples)
    if n < 2:
        return (math.nan, math.nan, math.nan)
    sx = sy = sz = 0.0
    sxx = syy = szz = 0.0
    for x, y, z in samples:
        sx += x
        sy += y
        sz += z
        sxx += x * x
        syy += y * y
        szz += z * z
    mx, my, mz = sx / n, sy / n, sz / n
    vx = (sxx - n * mx * mx) / (n - 1)
    vy = (syy - n * my * my) / (n - 1)
    vz = (szz - n * mz * mz) / (n - 1)
    return (max(0.0, vx), max(0.0, vy), max(0.0, vz))


def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--window", type=int, default=200)
    ap.add_argument("--tol_ratio", type=float, default=2.0)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--trigger_test", action="store_true", help="Send 'T' to request one-shot synthetic frame")
    ap.add_argument("--toggle_crc", action="store_true", help="Toggle firmware CRC injection with 'C'")
    ap.add_argument("--skip_cov_check", action="store_true", help="Skip covariance validation (use when static covariances are enabled)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.1)
    if args.toggle_crc:
        ser.write(b"C")
    if args.trigger_test:
        ser.write(b"T")

    gyro_win = deque(maxlen=args.window)
    acc_win = deque(maxlen=args.window)
    t_win = deque(maxlen=args.window)

    frames = 0
    drops = 0
    last_seq = None
    bad_crc = 0
    bad_len = 0
    wrong_type = 0

    # Physics and stats
    quat_bad = 0
    grav_bad = 0
    gyro_bias_bad = 0
    cov_warn = 0
    rate_jitter_bad = 0

    start = time.time()

    while time.time() - start < args.seconds:
        raw = ser.read_until(b"\x00")
        if not raw:
            continue
        raw = raw[:-1] if raw.endswith(b"\x00") else raw
        try:
            payload = cobs.decode(raw)
        except Exception:
            continue
        expected_len = 1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2
        if len(payload) != expected_len:
            bad_len += 1
            continue
        if payload[0] != PKT_IMU_V1:
            wrong_type += 1
            continue
        if struct.unpack_from("<H", payload, expected_len - 2)[0] != crc16_ccitt(payload[:-2]):
            bad_crc += 1
            continue

        parsed = parse_frame(payload)
        if not parsed:
            continue
        frames += 1
        if last_seq is not None and ((last_seq + 1) & 0xFFFF) != parsed["seq"]:
            drops += 1
        last_seq = parsed["seq"]

        gyro_win.append(parsed["gyro"])
        acc_win.append(parsed["acc"])
        t_win.append(parsed["t_ms"])

        # Quaternion norm check if orientation_valid
        orientation_valid = (parsed["flags"] & 0x01) != 0
        if orientation_valid:
            qw, qx, qy, qz = parsed["quat"]
            qn = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if not (0.995 <= qn <= 1.005):
                quat_bad += 1

        # Stationary detection: low gyro magnitude window
        if len(gyro_win) == gyro_win.maxlen:
            gmag = sum(norm3(g) for g in gyro_win) / len(gyro_win)
            stationary = gmag < 0.02  # rad/s
            if stationary:
                amag = sum(norm3(a) for a in acc_win) / len(acc_win)
                if not (8.5 <= amag <= 10.8):  # wide band for commodity IMUs
                    grav_bad += 1
                # Gyro bias near zero at rest
                gx_mean = sum(g[0] for g in gyro_win) / len(gyro_win)
                gy_mean = sum(g[1] for g in gyro_win) / len(gyro_win)
                gz_mean = sum(g[2] for g in gyro_win) / len(gyro_win)
                if max(abs(gx_mean), abs(gy_mean), abs(gz_mean)) > 0.05:
                    gyro_bias_bad += 1
                # Variance agreement
                vg = var_diag(list(gyro_win))
                va = var_diag(list(acc_win))
                if not args.skip_cov_check:
                    for comp, tx in zip(vg, parsed["cov_g"]):
                        ratio = max(1e-12, comp) / max(1e-12, tx)
                        if ratio < 1 / args.tol_ratio or ratio > args.tol_ratio:
                            cov_warn += 1
                    for comp, tx in zip(va, parsed["cov_a"]):
                        ratio = max(1e-12, comp) / max(1e-12, tx)
                        if ratio < 1 / args.tol_ratio or ratio > args.tol_ratio:
                            cov_warn += 1
            # Rate jitter check from device timestamps
            dt = (t_win[-1] - t_win[0]) / max(1, (len(t_win) - 1))
            if not (8 <= dt <= 12):  # ms, expect ~10 ms
                rate_jitter_bad += 1

    elapsed = time.time() - start
    rate = frames / max(1e-6, elapsed)

    print(f"Frames: {frames}, Drops: {drops}, Rate: {rate:.1f} Hz")
    print(f"crc_err: {bad_crc}, len_err: {bad_len}, type_err: {wrong_type}")
    print(
        f"quat_bad: {quat_bad}, grav_bad: {grav_bad}, gyro_bias_bad: {gyro_bias_bad}, cov_warn: {cov_warn}, rate_jitter_bad: {rate_jitter_bad}"
    )

    # Simple pass/fail heuristic
    ok = drops == 0 and bad_crc == 0 and bad_len == 0 and wrong_type == 0
    ok = ok and (quat_bad == 0)  # ignore if orientation not provided
    ok = ok and (grav_bad == 0) and (gyro_bias_bad == 0)
    if args.skip_cov_check:
        ok = ok and (rate > 90.0)
    else:
        ok = ok and (cov_warn == 0) and (rate > 90.0)

    exit(0 if ok else 2)


if __name__ == "__main__":
    main()
