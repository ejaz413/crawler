#!/usr/bin/env python3
import ctypes, time, sys

# --- EPOS definitions ---
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"
PORT = b"USB0"
NODE_ID = 1

# Motor parameters (EC-i 40, 24V, 5.36 A nominal)
NOMINAL_CURRENT = 5360      # mA
PEAK_CURRENT = 23000        # mA (short bursts)
CONT_CURRENT = 5000         # mA safe limit for startup
POLE_PAIRS = 4              # check datasheet / order code
ENCODER_COUNTS = 500        # only if encoder is present, else leave unused

# Load EPOS library
epos = ctypes.cdll.LoadLibrary("libEposCmd.so")
error_code = ctypes.c_uint()

def open_device():
    return epos.VCS_OpenDevice(
        DEVICE_NAME, PROTOCOL_STACK, INTERFACE, PORT, ctypes.byref(error_code)
    )

def close_device(h):
    epos.VCS_CloseDevice(h, ctypes.byref(error_code))

def clear_fault(h):
    return epos.VCS_ClearFault(h, NODE_ID, ctypes.byref(error_code)) != 0

def set_opmode_velocity(h):
    return epos.VCS_SetOperationMode(h, NODE_ID, ctypes.c_int(3), ctypes.byref(error_code)) != 0

def set_enable_state(h):
    epos.VCS_SetEnableState(h, NODE_ID, ctypes.byref(error_code))

def get_state(h):
    state = ctypes.c_ushort()
    epos.VCS_GetState(h, NODE_ID, ctypes.byref(state), ctypes.byref(error_code))
    return state.value

# --- Main ---
if __name__ == "__main__":
    h = open_device()
    if h == 0:
        print("❌ Could not open device")
        sys.exit(1)
    print("✅ Device opened")

    # 1) Clear faults
    if clear_fault(h):
        print("✅ Fault cleared")

    # 2) Set current limits
    if hasattr(epos, "VCS_SetCurrentMust"):
        # If your lib has direct current limit funcs, use them
        pass
    # Fallback via Object Dictionary (SDO write)
    # Indices for EPOS4:
    # 0x3001 sub 01h = Continuous current limit [mA]
    # 0x3001 sub 02h = Output current limit [mA]
    # 0x3003 sub 01h = Number of pole pairs
    if hasattr(epos, "VCS_SDODownload"):
        def sdo_write(index, sub, value):
            buf = (ctypes.c_ubyte * 4)()
            for i in range(4):
                buf[i] = (value >> (8 * i)) & 0xFF
            epos.VCS_SDODownload(
                h, NODE_ID,
                ctypes.c_uint(index), ctypes.c_uint(sub),
                ctypes.byref(buf), ctypes.c_uint(4),
                ctypes.c_uint(1000),
                ctypes.byref(error_code)
            )
        # Write motor limits
        sdo_write(0x3001, 1, CONT_CURRENT)  # continuous current
        sdo_write(0x3001, 2, PEAK_CURRENT)  # peak current
        sdo_write(0x3003, 1, POLE_PAIRS)    # pole pairs
        print(f"✅ Current limits set: {CONT_CURRENT} mA cont, {PEAK_CURRENT} mA peak")

    # 3) Set velocity mode
    if set_opmode_velocity(h):
        print("✅ Operation Mode set to Velocity (3)")

    # 4) Enable motor
    set_enable_state(h)
    state = get_state(h)
    print(f"📟 State after enable = {state}")

    # 5) Safe test run
    if hasattr(epos, "VCS_SetVelocityProfile"):
        epos.VCS_SetVelocityProfile(h, NODE_ID, 100, 100, ctypes.byref(error_code))
        epos.VCS_MoveWithVelocity(h, NODE_ID, 100, ctypes.byref(error_code))
        print("⚡ Motor test run at 20 rpm for 2s...")
        time.sleep(2)
        epos.VCS_HaltVelocityMovement(h, NODE_ID, ctypes.byref(error_code))
        print("🛑 Motor stopped")

    close_device(h)
    print("✅ Device closed")
