#!/usr/bin/env python3
"""
Safe EPOS init for EC-i40 (Hall + Encoder, 2048 CPR)
with live plotting of target vs. actual shaft velocity.
"""

import ctypes, time, sys
import matplotlib.pyplot as plt
from collections import deque

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"
PORT = b"USB0"
NODE_ID = 913

# Gear ratio (motor rotations : output shaft rotations)
GEAR_RATIO = 1

# Test profile
TARGET_RPM = 50          # target **shaft** RPM
ACC_RPM_S = 1000
DEC_RPM_S = 1000
RUN_TIME = 10.0          # seconds at target before going back to zero
MONITOR_TIME = 25.0      # total monitoring time

# ----------------------------------------------

# Load lib
try:
    epos = ctypes.cdll.LoadLibrary("libEposCmd.so")
except Exception as e:
    print("❌ Failed to load libEposCmd.so:", e)
    sys.exit(1)

error_code = ctypes.c_uint()

# --- Helper wrappers ---
def open_device():
    return epos.VCS_OpenDevice(DEVICE_NAME, PROTOCOL_STACK, INTERFACE, PORT, ctypes.byref(error_code))

def close_device(h):
    epos.VCS_CloseDevice(h, ctypes.byref(error_code))

def get_state(h):
    s = ctypes.c_ushort()
    if epos.VCS_GetState(h, NODE_ID, ctypes.byref(s), ctypes.byref(error_code)) != 0:
        return s.value
    return None

def get_fault(h):
    f = ctypes.c_uint()
    if epos.VCS_GetFaultState(h, NODE_ID, ctypes.byref(f), ctypes.byref(error_code)) != 0:
        return f.value
    return 0

def clear_fault(h):
    return epos.VCS_ClearFault(h, NODE_ID, ctypes.byref(error_code)) != 0

def set_velocity_mode(h):
    return epos.VCS_SetOperationMode(h, NODE_ID, ctypes.c_int(3), ctypes.byref(error_code)) != 0

def state_sequence_enable(h):
    epos.VCS_SetState(h, NODE_ID, 2, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetState(h, NODE_ID, 3, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetEnableState(h, NODE_ID, ctypes.byref(error_code))
    time.sleep(0.12)

def set_velocity_profile(h, acc, dec):
    return epos.VCS_SetVelocityProfile(h, NODE_ID, ctypes.c_uint(acc), ctypes.c_uint(dec), ctypes.byref(error_code)) != 0

def move_with_velocity(h, rpm):
    # Convert shaft RPM to motor RPM
    motor_rpm = int(rpm * GEAR_RATIO)
    return epos.VCS_MoveWithVelocity(h, NODE_ID, ctypes.c_long(motor_rpm), ctypes.byref(error_code)) != 0

def halt_velocity(h):
    if hasattr(epos, "VCS_HaltVelocityMovement"):
        epos.VCS_HaltVelocityMovement(h, NODE_ID, ctypes.byref(error_code))
    else:
        epos.VCS_MoveWithVelocity(h, NODE_ID, ctypes.c_long(0), ctypes.byref(error_code))

def get_velocity_is(h):
    v = ctypes.c_long()
    if epos.VCS_GetVelocityIs(h, NODE_ID, ctypes.byref(v), ctypes.byref(error_code)) != 0:
        return v.value / GEAR_RATIO  # convert motor RPM to output shaft RPM
    return None

# ----------------- Main flow -----------------
if __name__ == "__main__":
    h = open_device()
    if h == 0:
        print("❌ Could not open device (VCS_OpenDevice failed)")
        sys.exit(1)
    print("✅ Device opened")

    # 1) clear any fault
    f = get_fault(h)
    if f:
        print(f"⚠️ Fault present at start: 0x{f:08X}  — attempting to clear")
        if clear_fault(h):
            print("✅ Fault cleared")
            time.sleep(0.2)
        else:
            print("❌ Could not clear fault. Fix hardware and retry.")
            close_device(h)
            sys.exit(1)
    else:
        print("✅ No fault at start")

    # 2) Configure velocity mode & enable
    if set_velocity_mode(h):
        print("✅ Operation Mode set to Velocity (3)")
    else:
        print(f"❌ Failed to set operation mode (error {error_code.value})")

    state_sequence_enable(h)
    s = get_state(h)
    print(f"📟 State after enable attempts = {s}  (want 3 or 8 for Operation Enabled)")

    # 3) Set safe velocity profile
    if set_velocity_profile(h, ACC_RPM_S, DEC_RPM_S):
        print(f"✅ Velocity profile set (acc={ACC_RPM_S}, dec={DEC_RPM_S})")
    else:
        print("⚠️ Could not set velocity profile")

    # Safety: only run test if no fault active
    f = get_fault(h)
    if f:
        print(f"❌ Fault remains 0x{f:08X} — aborting test run")
        close_device(h)
        sys.exit(1)

    # --- setup live plot ---
    plt.ion()
    fig, ax = plt.subplots()

    time_window = 100
    times = deque(maxlen=time_window)
    target_vel = deque(maxlen=time_window)
    actual_vel = deque(maxlen=time_window)

    line_target, = ax.plot([], [], 'r--', label="Target Velocity [RPM]")
    line_actual, = ax.plot([], [], 'b-', label="Actual Velocity [RPM]")

    ax.set_xlabel("Sample")
    ax.set_ylabel("Velocity [RPM]")
    ax.legend()
    plt.title("Live EPOS Shaft Velocity")
    fig.tight_layout()

    print(f"⚡ Test sequence: {TARGET_RPM} rpm → 0 rpm → {TARGET_RPM} rpm → 0 rpm")

    # --- test loop ---
    t0 = time.time()
    sample = 0
    commanded_vel = TARGET_RPM
    sequence_steps = [(TARGET_RPM, RUN_TIME), (0, RUN_TIME), (TARGET_RPM, RUN_TIME), (0, RUN_TIME)]
    try:
        for cmd, duration in sequence_steps:
            move_with_velocity(h, cmd)
            step_start = time.time()
            while time.time() - step_start < duration:
                v = get_velocity_is(h)
                f = get_fault(h)

                times.append(sample)
                target_vel.append(cmd)
                actual_vel.append(v if v is not None else 0)

                line_target.set_data(times, target_vel)
                line_actual.set_data(times, actual_vel)

                ax.set_xlim(max(0, sample - time_window), sample)
                if actual_vel:
                    vmin = min(min(actual_vel), min(target_vel)) - 5
                    vmax = max(max(actual_vel), max(target_vel)) + 5
                    ax.set_ylim(vmin, vmax)

                fig.canvas.draw()
                fig.canvas.flush_events()

                print(f"📊 Target={cmd} rpm, Actual={v:.1f} rpm, fault=0x{f:08X}", flush=True)

                sample += 1
                if f:
                    print("⚠️ Fault triggered — halting")
                    halt_velocity(h)
                    break

                time.sleep(0.2)

    finally:
        halt_velocity(h)
        time.sleep(0.2)
        plt.ioff()
        plt.show()

    print("🛑 Test run finished")
    close_device(h)
    print("✅ Device closed")
