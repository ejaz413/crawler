#!/usr/bin/env python3
"""
Safe EPOS init for EC-i40 (2048 CPR) — POSITION CONTROL demo with automatic safety limits

- Clears faults
- Sets current limits, pole pairs, encoder CPR (if SDO available)
- Sets Profile Position Mode and safe position profile
- Performs back-and-forth test moves while checking:
  - Faults
  - Current
  - Software min/max position limits (skips unsafe moves)
"""

import ctypes, time, sys

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"
PORT = b"USB0"
NODE_ID = 1

CONTINUOUS_CURRENT_MA = 5360
PEAK_CURRENT_MA = 23000
POLE_PAIRS = 4
ENCODER_CPR = 2048

# Test positions (desired)
TEST_POSITIONS = [0, 500, 0, -500, 0]  # back-and-forth
PROFILE_VELOCITY = 1000
PROFILE_ACCELERATION = 1000
PROFILE_DECELERATION = 1000
MAX_SAFE_CURRENT_MA = 20000

# Safe default position limits (used if SDO not available)
DEFAULT_MIN_POSITION = 0
DEFAULT_MAX_POSITION = 500

# Object Dictionary indices
OD_IDX_CURRENT_LIMIT = 0x3001
OD_SUB_CONT_CURRENT = 1
OD_SUB_PEAK_CURRENT = 2
OD_IDX_POLE_PAIRS = 0x3003
OD_IDX_ENCODER_CPR = 0x3004
OD_SUB_ENCODER_CPR = 1

OD_IDX_MIN_POSITION = 0x607D  # Profile Position Min
OD_IDX_MAX_POSITION = 0x607E  # Profile Position Max

# Load lib
try:
    epos = ctypes.cdll.LoadLibrary("libEposCmd.so")
except Exception as e:
    print("❌ Failed to load libEposCmd.so:", e)
    sys.exit(1)

error_code = ctypes.c_uint()

# --- Helper wrappers ---
def open_device(): return epos.VCS_OpenDevice(DEVICE_NAME, PROTOCOL_STACK, INTERFACE, PORT, ctypes.byref(error_code))
def close_device(h): epos.VCS_CloseDevice(h, ctypes.byref(error_code))
def get_state(h):
    s = ctypes.c_ushort()
    return s.value if epos.VCS_GetState(h, NODE_ID, ctypes.byref(s), ctypes.byref(error_code)) != 0 else None
def get_fault(h):
    f = ctypes.c_uint()
    return f.value if epos.VCS_GetFaultState(h, NODE_ID, ctypes.byref(f), ctypes.byref(error_code)) != 0 else None
def clear_fault(h): return epos.VCS_ClearFault(h, NODE_ID, ctypes.byref(error_code)) != 0
def set_position_mode(h): return epos.VCS_SetOperationMode(h, NODE_ID, ctypes.c_int(1), ctypes.byref(error_code)) != 0
def state_sequence_enable(h):
    epos.VCS_SetState(h, NODE_ID, 2, ctypes.byref(error_code)); time.sleep(0.12)
    epos.VCS_SetState(h, NODE_ID, 3, ctypes.byref(error_code)); time.sleep(0.12)
    epos.VCS_SetEnableState(h, NODE_ID, ctypes.byref(error_code)); time.sleep(0.12)
def set_position_profile(h, acc, dec, vel):
    epos.VCS_SetPositionProfile(h, NODE_ID, ctypes.c_uint(vel), ctypes.c_uint(acc), ctypes.c_uint(dec), ctypes.byref(error_code))
def move_to_position(h, position, absolute=True):
    return epos.VCS_MoveToPosition(h, NODE_ID, ctypes.c_long(position), ctypes.c_int(1 if absolute else 0), ctypes.byref(error_code)) != 0
def get_position_is(h):
    pos = ctypes.c_long()
    return pos.value if epos.VCS_GetPositionIs(h, NODE_ID, ctypes.byref(pos), ctypes.byref(error_code)) != 0 else None
def get_current_is(h):
    i = ctypes.c_short()
    if hasattr(epos, "VCS_GetCurrentIs"):
        if epos.VCS_GetCurrentIs(h, NODE_ID, ctypes.byref(i), ctypes.byref(error_code)) != 0:
            return i.value
    return None
def has_sdo(): return hasattr(epos, "VCS_SDODownload") and hasattr(epos, "VCS_SDOUpload")
def sdo_write(h, index, subindex, value, size=4, timeout_ms=1000):
    if not hasattr(epos, "VCS_SDODownload"): return False
    try:
        buf = (ctypes.c_ubyte * size)()
        v = int(value) & ((1 << (8*size)) - 1)
        for i in range(size): buf[i] = (v >> (8*i)) & 0xFF
        ret = epos.VCS_SDODownload(h, ctypes.c_uint(NODE_ID), ctypes.c_uint(index), ctypes.c_uint(subindex),
                                   ctypes.byref(buf), ctypes.c_uint(size), ctypes.c_uint(timeout_ms), ctypes.byref(error_code))
        return ret != 0
    except Exception: return False
def sdo_read(h, index, subindex, size=4, timeout_ms=1000):
    if not hasattr(epos, "VCS_SDOUpload"): return None
    try:
        buf = (ctypes.c_ubyte * size)()
        ret = epos.VCS_SDOUpload(h, ctypes.c_uint(NODE_ID), ctypes.c_uint(index), ctypes.c_uint(subindex),
                                 ctypes.byref(buf), ctypes.c_uint(size), ctypes.c_uint(timeout_ms), ctypes.byref(error_code))
        if ret == 0: return None
        val = 0
        for i in range(size): val |= buf[i] << (8*i)
        return val
    except Exception: return None

def get_position_limits(h):
    if has_sdo():
        min_pos = sdo_read(h, OD_IDX_MIN_POSITION, 0) or DEFAULT_MIN_POSITION
        max_pos = sdo_read(h, OD_IDX_MAX_POSITION, 0) or DEFAULT_MAX_POSITION
    else:
        min_pos, max_pos = DEFAULT_MIN_POSITION, DEFAULT_MAX_POSITION
    return min_pos, max_pos

# ----------------- Main -----------------
if __name__ == "__main__":
    h = open_device()
    if h == 0: sys.exit("❌ Could not open device")
    print("✅ Device opened")

    # Clear fault
    f = get_fault(h)
    if f: clear_fault(h); time.sleep(0.2)

    # SDO writes
    if has_sdo():
        sdo_write(h, OD_IDX_CURRENT_LIMIT, OD_SUB_CONT_CURRENT, CONTINUOUS_CURRENT_MA)
        sdo_write(h, OD_IDX_CURRENT_LIMIT, OD_SUB_PEAK_CURRENT, PEAK_CURRENT_MA)
        sdo_write(h, OD_IDX_POLE_PAIRS, 1, POLE_PAIRS)
        sdo_write(h, OD_IDX_ENCODER_CPR, OD_SUB_ENCODER_CPR, ENCODER_CPR)
        print("🔧 SDO writes done")
    else:
        print("⚠️ SDO not available — set manually in EPOS Studio")

    # Enable device
    if not set_position_mode(h): print(f"❌ Failed to set mode ({error_code.value})")
    state_sequence_enable(h)
    print(f"📟 State after enable = {get_state(h)} (want 3 or 8)")

    # Set position profile
    set_position_profile(h, PROFILE_ACCELERATION, PROFILE_DECELERATION, PROFILE_VELOCITY)
    print(f"✅ Position profile set")

    # Get limits
    min_pos, max_pos = get_position_limits(h)
    print(f"🔒 Position limits = {min_pos} ... {max_pos}")

    # Safe back-and-forth test
    print("⚡ Starting safe back-and-forth test")
    for pos in TEST_POSITIONS:
        if pos < min_pos or pos > max_pos:
            print(f"⚠️ Skipping unsafe target {pos} (outside {min_pos}..{max_pos})")
            continue
        f = get_fault(h)
        i = get_current_is(h)
        if f or (i is not None and abs(i) > MAX_SAFE_CURRENT_MA):
            print(f"❌ Aborting move at pos {pos} due to fault/current")
            break
        print(f"➡️ Moving to {pos}")
        if not move_to_position(h, pos): break
        while True:
            current_pos = get_position_is(h)
            f = get_fault(h)
            i = get_current_is(h)
            if f or (i is not None and abs(i) > MAX_SAFE_CURRENT_MA): break
            if current_pos is not None and abs(current_pos - pos) < 5: break
            time.sleep(0.1)
        print(f"✅ Reached {pos}")

    print("🛑 Test sequence finished")
    close_device(h)
    print("✅ Device closed")
