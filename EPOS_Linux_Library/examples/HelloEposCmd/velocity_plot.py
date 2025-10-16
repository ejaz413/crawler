#!/usr/bin/env python3
"""
Safe EPOS init for EC-i40 (Hall + Encoder, 2048 CPR)
with live plotting of target and actual velocity.
"""

import ctypes, time, sys
import matplotlib.pyplot as plt
from collections import deque

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"
PORT = b"USB0"
NODE_ID = 1

# Test profile
TARGET_RPM = 5000         # commanded speed
ACC_RPM_S = 1000
DEC_RPM_S = 1000

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
    return None

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
    return epos.VCS_MoveWithVelocity(h, NODE_ID, ctypes.c_long(int(rpm)), ctypes.byref(error_code)) != 0

def halt_velocity(h):
    if hasattr(epos, "VCS_HaltVelocityMovement"):
        epos.VCS_HaltVelocityMovement(h, NODE_ID, ctypes.byref(error_code))
    else:
        epos.VCS_MoveWithVelocity(h, NODE_ID, ctypes.c_long(0), ctypes.byref(error_code))

def get_velocity_is(h):
    v = ctypes.c_long()
    if epos.VCS_GetVelocityIs(h, NODE_ID, ctypes.byref(v), ctypes.byref(error_code)) != 0:
        return v.value
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

    # 3) Set safe velocity profile & gentle test run
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

    print(f"⚡ Test run at {TARGET_RPM} rpm for 10s (live monitoring + plot)")
    if not move_with_velocity(h, TARGET_RPM):
        print(f"❌ MoveWithVelocity failed (error {error_code.value}) — aborting")
        close_device(h)
        sys.exit(1)

    # --- setup live plot ---
    plt.ion()
    fig, ax = plt.subplots()

    time_window = 100   # number of samples to show
    times = deque(maxlen=time_window)
    target_vel = deque(maxlen=time_window)
    actual_vel = deque(maxlen=time_window)

    line_target, = ax.plot([], [], 'r--', label="Target Velocity [rpm]")
    line_actual, = ax.plot([], [], 'b-', label="Actual Velocity [rpm]")

    ax.set_xlabel("Sample")
    ax.set_ylabel("Velocity [rpm]")
    ax.legend()
    plt.title("Live EPOS Velocity")
    fig.tight_layout()

    # --- test loop with plotting ---
    t0 = time.time()
    sample = 0
    try:
        while time.time() - t0 < 10.0:
            v = get_velocity_is(h)
            f = get_fault(h)

            # store values
            times.append(sample)
            target_vel.append(TARGET_RPM)
            actual_vel.append(v if v is not None else 0)

            # update plot
            line_target.set_data(times, target_vel)
            line_actual.set_data(times, actual_vel)

            ax.set_xlim(max(0, sample - time_window), sample)
            if actual_vel:
                vmin = min(min(actual_vel), TARGET_RPM) - 10
                vmax = max(max(actual_vel), TARGET_RPM) + 10
                ax.set_ylim(vmin, vmax)

            fig.canvas.draw()
            fig.canvas.flush_events()

            print(f"📊 Target={TARGET_RPM} rpm, Actual={v} rpm, fault=0x{f:08X}", flush=True)

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

# End of script
