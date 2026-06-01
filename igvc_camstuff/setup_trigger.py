#!/usr/bin/env python3
"""
setup_trigger.py
Enables external hardware trigger snapshot mode on ArduCam OV9281 USB cameras.

KEY FINDING: trigger mode is a STANDARD V4L2 control, not a UVC extension unit.
ArduCam's 'low-brightness compensation' / 'exposure_auto_priority' maps to
'exposure_dynamic_framerate' (control 0x009a0903) on kernel 6.8.
Setting it to 1 ENABLES external trigger snapshot mode.

Both cameras share the XIAO GPIO5 pulse, so they expose simultaneously =
true hardware stereo sync.

Wiring: OV9281 Pin F -> XIAO GPIO5, Pin G -> GND
"""
import subprocess
import sys

CAMERAS    = ['/dev/video0', '/dev/video2']
EXPOSURE   = 200   # units of 100us; 200 = 20ms (must be < 40ms for 25fps)

def set_ctrl(device, ctrl, value):
    r = subprocess.run(
        ['v4l2-ctl', '-d', device, f'--set-ctrl={ctrl}={value}'],
        capture_output=True, text=True
    )
    return r.returncode == 0

def setup_camera(device):
    print(f"Configuring {device} for external trigger...")
    ok = True
    # 1) Manual exposure mode
    ok &= set_ctrl(device, 'auto_exposure', 1)
    # 2) Short fixed exposure (must be shorter than trigger period)
    ok &= set_ctrl(device, 'exposure_time_absolute', EXPOSURE)
    # 3) Enable external trigger snapshot mode (THE trigger switch)
    ok &= set_ctrl(device, 'exposure_dynamic_framerate', 1)

    if ok:
        print(f"  ✓ {device} in external trigger mode (exposure={EXPOSURE*100}us)")
    else:
        print(f"  ✗ {device} setup had errors")
    return ok

if __name__ == '__main__':
    print("ArduCam OV9281 External Trigger Setup")
    print("=" * 40)
    print("Wiring: Pin F -> GPIO5, Pin G -> GND")
    print("Both cameras share the trigger = hardware stereo sync\n")

    all_ok = True
    for dev in CAMERAS:
        all_ok &= setup_camera(dev)

    print()
    if all_ok:
        print("✓ Both cameras armed. They will now capture only on GPIO5 pulses.")
    else:
        print("⚠ Some cameras failed setup — check connections.")
    print("Launch ROS2 nodes now.")
