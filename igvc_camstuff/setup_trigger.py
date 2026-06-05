#!/usr/bin/env python3
"""
setup_trigger.py
Enables external hardware trigger snapshot mode on the ArduCam OV9281 cameras.
Auto-discovers them by V4L2 card name so it works regardless of /dev/videoN.
"""
import subprocess
import glob

CAMERA_NAME_MATCH = 'OV9281'
EXPOSURE = 200   # units of 100us; 200 = 20ms (< 40ms trigger period)

def card_name(dev):
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '--info'],
                             capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if 'Card type' in line:
                return line.split(':', 1)[1].strip()
    except Exception:
        pass
    return ''

def is_capture(dev):
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '--list-formats'],
                             capture_output=True, text=True, timeout=2).stdout
        return 'MJPG' in out or 'YUYV' in out or 'Video Capture' in out
    except Exception:
        return False

def discover():
    devs = []
    for dev in sorted(glob.glob('/dev/video*')):
        if CAMERA_NAME_MATCH in card_name(dev) and is_capture(dev):
            devs.append(dev)
    return devs

def set_ctrl(dev, ctrl, val):
    r = subprocess.run(['v4l2-ctl', '-d', dev, f'--set-ctrl={ctrl}={val}'],
                       capture_output=True, text=True)
    return r.returncode == 0

def setup_camera(dev):
    print(f'Configuring {dev} for external trigger...')
    ok = True
    ok &= set_ctrl(dev, 'auto_exposure', 1)
    ok &= set_ctrl(dev, 'exposure_time_absolute', EXPOSURE)
    ok &= set_ctrl(dev, 'exposure_dynamic_framerate', 1)
    print(f'  {"OK" if ok else "FAILED"}: {dev}')
    return ok

if __name__ == '__main__':
    print('ArduCam OV9281 External Trigger Setup (auto-discovery)')
    print('=' * 50)
    devs = discover()
    if not devs:
        print('No OV9281 cameras found! Check connections.')
    else:
        print(f'Found: {devs}')
        for d in devs:
            setup_camera(d)
    print('Done.')
