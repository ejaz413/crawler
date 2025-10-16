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

def clear_fault(key_handle):
    if epos.VCS_ClearFault(key_handle, NODE_ID, ctypes.byref(error_code)) != 0:
        print("✅ Fault cleared")
        return True
    else:
        print("❌ Failed to clear fault")
        return False

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
    # --- 1) Open device ---
    key_handle = epos.VCS_OpenDevice(
        DEVICE_NAME, PROTOCOL_STACK_NAME, INTERFACE_NAME, PORT_NAME, ctypes.byref(error_code)
    )
    if key_handle == 0:
        print("❌ Could not open device")
        exit(1)
    print("✅ Device opened")

    # --- 2) Check and clear faults ---
    fault = get_fault_state(key_handle)
    if fault != 0:
        print(f"⚠️ Fault detected: 0x{fault:08X}")
        for msg in decode_fault_state(fault):
            print(f"❌ {msg}")
        clear_fault(key_handle)
        time.sleep(0.5)

    # --- 3) Follow state sequence ---
    print("🔄 Setting state: Ready to Switch On (2)")
    epos.VCS_SetState(key_handle, NODE_ID, 2, ctypes.byref(error_code))
    time.sleep(0.5)
    print("🔄 Setting state: Switched On (3)")
    epos.VCS_SetState(key_handle, NODE_ID, 3, ctypes.byref(error_code))
    time.sleep(0.5)
    print("🔄 Enabling motor (Operation Enabled)")
    epos.VCS_SetEnableState(key_handle, NODE_ID, ctypes.byref(error_code))
    time.sleep(0.5)

    # --- 4) Verify state ---
    state = get_state(key_handle)
    print(f"📟 Current State = {state}")
    if state in (3, 8):
        print("✅ Motor is fully Operation Enabled")
    else:
        print("⚠️ Motor still not in Operation Enabled")

    # --- 5) Set velocity profile ---
    accel = ctypes.c_uint(1000)
    decel = ctypes.c_uint(1000)
    max_velocity = 10000  # rpm
    if epos.VCS_SetVelocityProfile(key_handle, NODE_ID, accel, decel, ctypes.byref(error_code)) != 0:
        print(f"✅ Velocity profile set (acc={accel.value}, dec={decel.value})")
    else:
        print(f"❌ Failed to set velocity profile, error={error_code.value}")

    # --- 6) Move motor ---
    target_velocity = ctypes.c_long(max_velocity)
    if epos.VCS_MoveWithVelocity(key_handle, NODE_ID, target_velocity, ctypes.byref(error_code)) != 0:
        print(f"⚡ Motor running at {target_velocity.value} rpm")
    else:
        print(f"❌ Failed to move motor, error={error_code.value}")

    # --- 7) Monitor actual velocity for 5 seconds ---
    for i in range(10):
        time.sleep(0.5)
        actual_vel = get_actual_velocity(key_handle)
        print(f"📊 Actual velocity = {actual_vel} rpm")

    # --- 8) Stop motor ---
    epos.VCS_MoveWithVelocity(key_handle, NODE_ID, ctypes.c_long(0), ctypes.byref(error_code))
    print("🛑 Motor stopped")

    epos.VCS_CloseDevice(key_handle, ctypes.byref(error_code))
    print("✅ Device closed")
