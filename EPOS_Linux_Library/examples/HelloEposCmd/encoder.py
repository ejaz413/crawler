#!/usr/bin/env python3
"""
Safe EPOS init for EC-i40 (Hall + Encoder, 2048 CPR)

- Clears faults
- Sets velocity mode and safe velocity profile
- Performs a gentle test run while monitoring faults/inputs

This version does NOT use SDO writes.
"""

import ctypes, time, sys

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"
PORT = b"USB0"
NODE_ID = 1

# Conservative test profile
TEST_RPM = 2000
ACC_RPM_S = 1000
DEC_RPM_S = 1000

MAX_SAFE_CURRENT_MA = 40000    # abort if actual current > this (mA)

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

def get_current_is(h):
    i = ctypes.c_short()
    if hasattr(epos, "VCS_GetCurrentIs"):
        if epos.VCS_GetCurrentIs(h, NODE_ID, ctypes.byref(i), ctypes.byref(error_code)) != 0:
            return i.value
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

    print(f"⚡ Gentle test run at {TEST_RPM} rpm for 2s (monitoring current/velocity)")
    if not move_with_velocity(h, TEST_RPM):
        print(f"❌ MoveWithVelocity failed (error {error_code.value}) — aborting")
        close_device(h)
        sys.exit(1)

    t0 = time.time()
    try:
        while time.time() - t0 < 10.0:
            v = get_velocity_is(h)
            i = get_current_is(h)
            f = get_fault(h)
            print(f"📊 v={v} rpm, i={i if i is not None else 'N/A'} mA, fault=0x{f:08X}")
            if f:
                print("⚠️ Fault triggered during test — halting")
                halt_velocity(h)
                break
            if i is not None and abs(i) > MAX_SAFE_CURRENT_MA:
                print("⚠️ Current exceeded safety limit — halting")
                halt_velocity(h)
                break
            time.sleep(0.2)
    finally:
        halt_velocity(h)
        time.sleep(0.2)

    print("🛑 Test run finished")
    close_device(h)
    print("✅ Device closed")

# End of script
