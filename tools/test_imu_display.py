#!/usr/bin/env python3
"""
test_imu_display.py
Real-time IMU data display tool for BNO085 sensor.
Displays acceleration, gyroscope, temperature, and orientation (quaternion) data.

Usage:
    python3 tools/test_imu_display.py [--port /dev/ttyUSB0] [--baud 115200]

Requirements:
    pip install pyserial cobs
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
    from cobs import cobs
except ImportError:
    print("Error: Required packages not found.")
    print("Install with: pip install pyserial cobs")
    sys.exit(1)

PKT_IMU_V1 = 0x31


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """Calculate CRC16-CCITT checksum."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def parse_frame(payload: bytes):
    """Parse IMU frame and return sensor data dictionary."""
    expected_len = 1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2
    if len(payload) != expected_len or payload[0] != PKT_IMU_V1:
        return None
    
    # Verify CRC
    if struct.unpack_from("<H", payload, expected_len - 2)[0] != crc16_ccitt(payload[:-2]):
        return None
    
    # Parse frame
    off = 0
    (pkt_type, seq, sensor_id, flags, t_ms) = struct.unpack_from("<B H B B I", payload, off)
    off += 1 + 2 + 1 + 1 + 4
    
    # Orientation (quaternion)
    (qw, qx, qy, qz) = struct.unpack_from("<4f", payload, off)
    off += 16
    
    # Gyroscope
    (gx, gy, gz) = struct.unpack_from("<3f", payload, off)
    off += 12
    
    # Accelerometer
    (ax, ay, az) = struct.unpack_from("<3f", payload, off)
    off += 12
    
    # Covariances
    (cov_ori_x, cov_ori_y, cov_ori_z) = struct.unpack_from("<3f", payload, off)
    off += 12
    (cov_gx, cov_gy, cov_gz) = struct.unpack_from("<3f", payload, off)
    off += 12
    (cov_ax, cov_ay, cov_az) = struct.unpack_from("<3f", payload, off)
    
    return {
        "seq": seq,
        "sid": sensor_id,
        "flags": flags,
        "time_ms": t_ms,
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
        "gx": gx, "gy": gy, "gz": gz,
        "ax": ax, "ay": ay, "az": az,
        "cov_ori": (cov_ori_x, cov_ori_y, cov_ori_z),
        "cov_gyro": (cov_gx, cov_gy, cov_gz),
        "cov_accel": (cov_ax, cov_ay, cov_az),
    }


def quaternion_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def format_data(data, show_covariances=False, show_euler=False, last_data=None):
    """Format sensor data for display."""
    # Calculate magnitude of acceleration and angular velocity
    accel_mag = math.sqrt(data["ax"]**2 + data["ay"]**2 + data["az"]**2)
    gyro_mag = math.sqrt(data["gx"]**2 + data["gy"]**2 + data["gz"]**2)
    
    # Check if quaternion changed from last frame
    quat_stale = False
    if last_data:
        quat_same = (
            abs(data["qw"] - last_data["qw"]) < 1e-6 and
            abs(data["qx"] - last_data["qx"]) < 1e-6 and
            abs(data["qy"] - last_data["qy"]) < 1e-6 and
            abs(data["qz"] - last_data["qz"]) < 1e-6
        )
        quat_stale = quat_same
    
    output = f"\nSeq: {data['seq']:5d} | Time: {data['time_ms']:10d} ms"
    if quat_stale:
        output += " | ⚠️ QUAT_STALE"
    output += "\n"
    
    # Accelerometer data
    output += f"Accel (m/s²): X={data['ax']:8.4f}  Y={data['ay']:8.4f}  Z={data['az']:8.4f}  |Mag|={accel_mag:.4f}\n"
    
    # Gyroscope data
    output += f"Gyro  (rad/s): X={data['gx']:8.4f}  Y={data['gy']:8.4f}  Z={data['gz']:8.4f}  |Mag|={gyro_mag:.4f}\n"
    
    # Orientation data
    output += f"Quat: W={data['qw']:8.4f}  X={data['qx']:8.4f}  Y={data['qy']:8.4f}  Z={data['qz']:8.4f}\n"
    
    # Euler angles only if requested (expensive calculation)
    if show_euler:
        roll, pitch, yaw = quaternion_to_euler(data["qw"], data["qx"], data["qy"], data["qz"])
        output += f"Euler (deg): Roll={roll:8.2f}°  Pitch={pitch:8.2f}°  Yaw={yaw:8.2f}°\n"
    # Covariances (if requested)
    if show_covariances:
        output += f"{'─'*70}\n"
        cov_o = data['cov_ori']
        output += f"Cov Ori: X={cov_o[0]:.6e}  Y={cov_o[1]:.6e}  Z={cov_o[2]:.6e}\n"
        cov_g = data['cov_gyro']
        output += f"Cov Gyro: X={cov_g[0]:.6e}  Y={cov_g[1]:.6e}  Z={cov_g[2]:.6e}\n"
        cov_a = data['cov_accel']
        output += f"Cov Accel: X={cov_a[0]:.6e}  Y={cov_a[1]:.6e}  Z={cov_a[2]:.6e}\n"
    
    return output


def main():
    parser = argparse.ArgumentParser(
        description="Display real-time IMU (BNO085) sensor data from serial connection."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port (default: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--cov",
        action="store_true",
        help="Show covariance matrices"
    )
    parser.add_argument(
        "--euler",
        action="store_true",
        help="Calculate and show Euler angles (slower, use sparingly)"
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of samples to display (0 = infinite)"
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Minimal output, one line per frame"
    )
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="Highlight stale quaternions (frames where quat didn't update)"
    )
    
    args = parser.parse_args()
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.01)
        print(f"Connected to {args.port} at {args.baud} baud")
        print("Waiting for IMU data...")
        
        # Frame synchronization buffer
        buffer = bytearray()
        frame_count = 0
        error_count = 0
        sync_loss_count = 0
        last_data = None
        quat_stale_count = 0
        
        while True:
            try:
                # Read all available data without blocking
                bytes_available = ser.in_waiting
                if bytes_available > 0:
                    data = ser.read(bytes_available)
                    buffer.extend(data)
                
                # Process frames: look for 0x00 delimiter
                while True:
                    delim_idx = buffer.find(0x00)
                    if delim_idx == -1:
                        # No complete frame yet
                        break
                    
                    if delim_idx == 0:
                        # Skip leading delimiter bytes (sync recovery)
                        buffer.pop(0)
                        continue
                    
                    # Try to decode frame (everything before the 0x00)
                    frame_data = bytes(buffer[:delim_idx])
                    buffer = buffer[delim_idx + 1:]  # Remove processed frame and delimiter
                    
                    try:
                        decoded = cobs.decode(frame_data)
                        data = parse_frame(decoded)
                        
                        if data:
                            frame_count += 1
                            
                            # Check if quaternion is stale
                            if last_data:
                                quat_same = (
                                    abs(data["qw"] - last_data["qw"]) < 1e-6 and
                                    abs(data["qx"] - last_data["qx"]) < 1e-6 and
                                    abs(data["qy"] - last_data["qy"]) < 1e-6 and
                                    abs(data["qz"] - last_data["qz"]) < 1e-6
                                )
                                if quat_same:
                                    quat_stale_count += 1
                            
                            if args.quiet:
                                stale_marker = " ⚠️ QUAT_STALE" if (args.diagnose and last_data and quat_same) else ""
                                print(
                                    f"Seq: {data['seq']:5d} | "
                                    f"A({data['ax']:7.3f}, {data['ay']:7.3f}, {data['az']:7.3f}) | "
                                    f"G({data['gx']:7.3f}, {data['gy']:7.3f}, {data['gz']:7.3f}){stale_marker}"
                                )
                            else:
                                print(format_data(data, show_covariances=args.cov, show_euler=args.euler, last_data=last_data))
                            
                            last_data = data.copy()
                            
                            if args.count > 0 and frame_count >= args.count:
                                print(f"\nDisplayed {frame_count} frames. Exiting.")
                                break
                        else:
                            error_count += 1
                    except Exception as e:
                        error_count += 1
                        # On decode error, skip 1 byte and try to resync
                        if len(buffer) > 0:
                            buffer.pop(0)
                            sync_loss_count += 1
                
                if args.count > 0 and frame_count >= args.count:
                    break
                        
            except KeyboardInterrupt:
                print(f"\n\nInterrupted by user.")
                print(f"Successfully displayed: {frame_count} frames")
                print(f"Stale quaternions: {quat_stale_count} ({100*quat_stale_count/max(1,frame_count):.1f}%)")
                print(f"Errors encountered: {error_count}")
                print(f"Sync losses: {sync_loss_count}")
                break
            except Exception as e:
                error_count += 1
                print(f"Error reading from serial: {e}")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error opening serial port {args.port}: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
