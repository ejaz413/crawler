#!/usr/bin/env python3
"""
Safe EPOS init for EC-i40 (Hall + Encoder, 2048 CPR) — POSITION CONTROL version

- Clears faults
- Sets current limits (conservative)
- Sets pole pairs
- Attempts to write encoder CPR (2048) via SDO if available
- Sets Profile Position Mode and safe position profile
- Performs a gentle test move while monitoring faults/inputs
"""

import ctypes, time, sys

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"
PORT = b"USB0"
NODE_ID = 1

# Motor parameters (EC-i 40, 24V)
CONTINUOUS_CURRENT_MA = 5360
PEAK_CURRENT_MA = 23000
POLE_PAIRS = 4
ENCODER_CPR = 2048

# Conservative test profile
TEST_POSITION = 500  # encoder counts
PROFILE_VELOCITY = 1000      # counts/sec
PROFILE_ACCELERATION = 1000  # counts/sec^2
PROFILE_DECELERATION = 1000  # counts/sec^2

MAX_SAFE_CURRENT_MA = 20000

# Object Dictionary indices
OD_IDX_CURRENT_LIMIT = 0x3001
OD_SUB_CONT_CURRENT = 1
OD_SUB_PEAK_CURRENT = 2
OD_IDX_POLE_PAIRS = 0x3003
OD_IDX_ENCODER_CPR = 0x3004
OD_SUB_ENCODER_CPR = 1

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

def set_position_mode(h):
    POSITION_MODE = 1  # Profile Position Mode
    return epos.VCS_SetOperationMode(h, NODE_ID, ctypes.c_int(POSITION_MODE), ctypes.byref(error_code)) != 0

def state_sequence_enable(h):
    epos.VCS_SetState(h, NODE_ID, 2, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetState(h, NODE_ID, 3, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetEnableState(h, NODE_ID, ctypes.byref(error_code))
    time.sleep(0.12)

def set_position_profile(h, acc, dec, vel):
    """Set acceleration, deceleration, and max velocity for Profile Position Mode"""
    epos.VCS_SetPositionProfile(h, NODE_ID, ctypes.c_uint(vel), ctypes.c_uint(acc), ctypes.c_uint(dec), ctypes.byref(error_code))

def move_to_position(h, position, absolute=True):
    return epos.VCS_MoveToPosition(h, NODE_ID, ctypes.c_long(position), ctypes.c_int(1 if absolute else 0), ctypes.byref(error_code)) != 0

def get_position_is(h):
    pos = ctypes.c_long()
    if epos.VCS_GetPositionIs(h, NODE_ID, ctypes.byref(pos), ctypes.byref(error_code)) != 0:
        return pos.value
    return None

def get_current_is(h):
    i = ctypes.c_short()
    if hasattr(epos, "VCS_GetCurrentIs"):
        if epos.VCS_GetCurrentIs(h, NODE_ID, ctypes.byref(i), ctypes.byref(error_code)) != 0:
            return i.value
    return None

# --- SDO helpers ---
def has_sdo():
    return hasattr(epos, "VCS_SDODownload") and hasattr(epos, "VCS_SDOUpload")

def sdo_write(h, index, subindex, value, size=4, timeout_ms=1000):
    if not hasattr(epos, "VCS_SDODownload"):
        return False
    try:
        buf = (ctypes.c_ubyte * size)()
        v = int(value) & ((1 << (8*size)) - 1)
        for i in range(size):
            buf[i] = (v >> (8 * i)) & 0xFF
        ret = epos.VCS_SDODownload(h,
                                   ctypes.c_uint(NODE_ID),
                                   ctypes.c_uint(index),
                                   ctypes.c_uint(subindex),
                                   ctypes.byref(buf),
                                   ctypes.c_uint(size),
                                   ctypes.c_uint(timeout_ms),
                                   ctypes.byref(error_code))
        return ret != 0
    except Exception:
        return False

# ----------------- Main flow -----------------
if __name__ == "__main__":
    h = open_device()
    if h == 0:
        print("❌ Could not open device (VCS_OpenDevice failed)")
        sys.exit(1)
    print("✅ Device opened")

    # Clear any fault
    f = get_fault(h)
    if f:
        print(f"⚠️ Fault present at start: 0x{f:08X} — attempting to clear")
        if clear_fault(h):
            print("✅ Fault cleared")
            time.sleep(0.2)
        else:
            print("❌ Could not clear fault. Fix hardware and retry.")
            close_device(h)
            sys.exit(1)
    else:
        print("✅ No fault at start")

    # SDO writes for current limits, pole pairs
    if has_sdo():
        sdo_write(h, OD_IDX_CURRENT_LIMIT, OD_SUB_CONT_CURRENT, CONTINUOUS_CURRENT_MA)
        sdo_write(h, OD_IDX_CURRENT_LIMIT, OD_SUB_PEAK_CURRENT, PEAK_CURRENT_MA)
        sdo_write(h, OD_IDX_POLE_PAIRS, 1, POLE_PAIRS)
        sdo_write(h, OD_IDX_ENCODER_CPR, OD_SUB_ENCODER_CPR, ENCODER_CPR)
        print("🔧 Attempted SDO writes for current/polepairs/encoder CPR")
    else:
        print("⚠️ SDO not available — set current/polepairs/encoder CPR manually in EPOS Studio")

    # Set Profile Position Mode
    if set_position_mode(h):
        print("✅ Operation Mode set to Profile Position (1)")
    else:
        print(f"❌ Failed to set operation mode (error {error_code.value})")

    state_sequence_enable(h)
    s = get_state(h)
    print(f"📟 State after enable = {s} (want 3 or 8)")

    # Set safe position profile
    set_position_profile(h, PROFILE_ACCELERATION, PROFILE_DECELERATION, PROFILE_VELOCITY)
    print(f"✅ Position profile set (acc={PROFILE_ACCELERATION}, dec={PROFILE_DECELERATION}, vel={PROFILE_VELOCITY})")

    # Gentle test move
    f = get_fault(h)
    if f:
        print(f"❌ Fault remains 0x{f:08X} — aborting test move")
        close_device(h)
        sys.exit(1)

    print(f"⚡ Gentle test move to {TEST_POSITION} counts")
    if not move_to_position(h, TEST_POSITION):
        print(f"❌ MoveToPosition failed (error {error_code.value}) — aborting")
        close_device(h)
        sys.exit(1)

    t0 = time.time()
    try:
        while time.time() - t0 < 5.0:
            pos = get_position_is(h)
            i = get_current_is(h)
            f = get_fault(h)
            print(f"📊 pos={pos}, current={i if i is not None else 'N/A'} mA, fault=0x{f:08X}")
            if f:
                print("⚠️ Fault triggered during move — stopping")
                break
            if i is not None and abs(i) > MAX_SAFE_CURRENT_MA:
                print("⚠️ Current exceeded safety limit — stopping")
                break
            time.sleep(0.2)
    finally:
        # Halt by moving to current position
        current_pos = get_position_is(h)
        move_to_position(h, current_pos)
        time.sleep(0.2)

    print("🛑 Test move finished")
    close_device(h)
    print("✅ Device closed")
