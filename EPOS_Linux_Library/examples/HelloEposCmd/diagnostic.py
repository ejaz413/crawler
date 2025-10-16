#!/usr/bin/env python3
import ctypes, time

# Load EPOS Command Library
epos = ctypes.cdll.LoadLibrary("libEposCmd.so")

DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK_NAME = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
PORT_NAME = b"USB0"
NODE_ID = 1

error_code = ctypes.c_uint()

# --- Helper functions ---
def get_state(key_handle):
    state = ctypes.c_ushort()
    if epos.VCS_GetState(key_handle, NODE_ID, ctypes.byref(state), ctypes.byref(error_code)) != 0:
        return state.value
    return None

def get_actual_velocity(key_handle):
    velocity = ctypes.c_long()
    if epos.VCS_GetVelocityIs(key_handle, NODE_ID, ctypes.byref(velocity), ctypes.byref(error_code)) != 0:
        return velocity.value
    return None

def get_fault_state(key_handle):
    fault = ctypes.c_uint()
    if epos.VCS_GetFaultState(key_handle, NODE_ID, ctypes.byref(fault), ctypes.byref(error_code)) != 0:
        return fault.value
    return None

def decode_fault_state(fault):
    messages = []
    if fault & 0x00000001: messages.append("Overcurrent")
    if fault & 0x00000002: messages.append("Overvoltage")
    if fault & 0x00000004: messages.append("Undervoltage")
    if fault & 0x00000008: messages.append("Overtemperature")
    if fault & 0x00000010: messages.append("Encoder error")
    if fault & 0x00000020: messages.append("Hall sensor error")
    if fault & 0x00000040: messages.append("Motor blocked")
    if fault & 0x00000080: messages.append("Command error")
    return messages

if __name__ == "__main__":
    # Open EPOS device
    key_handle = epos.VCS_OpenDevice(
        DEVICE_NAME, PROTOCOL_STACK_NAME, INTERFACE_NAME, PORT_NAME, ctypes.byref(error_code)
    )
    if key_handle == 0:
        print("❌ Could not open device")
        exit(1)
    print("✅ Device opened for diagnostics")

    try:
        while True:
            state = get_state(key_handle)
            fault = get_fault_state(key_handle)
            actual_vel = get_actual_velocity(key_handle)

            print("\n=== EPOS Diagnostic ===")
            print(f"📟 Current State = {state}")
            if fault != 0:
                print(f"⚠️ Fault State = 0x{fault:08X}")
                for msg in decode_fault_state(fault):
                    print(f"❌ {msg}")
            else:
                print("✅ No active faults detected")
            print(f"📊 Actual Velocity = {actual_vel} rpm")

            # Optionally, add a pause and repeat
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 Stopping diagnostics...")
        epos.VCS_CloseDevice(key_handle, ctypes.byref(error_code))
        print("✅ Device closed")
