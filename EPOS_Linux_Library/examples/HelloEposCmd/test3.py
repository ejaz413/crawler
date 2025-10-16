#!/usr/bin/env python3

import ctypes, time

epos = ctypes.cdll.LoadLibrary("libEposCmd.so")

DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK_NAME = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
PORT_NAME = b"USB0"
NODE_ID = 1

error_code = ctypes.c_uint()

def get_state(key_handle):
    state = ctypes.c_ushort()
    ret = epos.VCS_GetState(key_handle, NODE_ID, ctypes.byref(state), ctypes.byref(error_code))
    if ret != 0:
        return state.value
    return None

def get_actual_velocity(key_handle):
    velocity = ctypes.c_long()
    ret = epos.VCS_GetVelocityIs(key_handle, NODE_ID, ctypes.byref(velocity), ctypes.byref(error_code))
    if ret != 0:
        return velocity.value
    return None

def get_max_profile_velocity(key_handle):
    max_vel = ctypes.c_long()
    ret = epos.VCS_GetMaxProfileVelocity(key_handle, NODE_ID, ctypes.byref(max_vel), ctypes.byref(error_code))
    if ret != 0:
        return max_vel.value
    return None

def is_motor_powered(key_handle):
    # Read digital inputs (optional)
    # If you have hardware enable input, this can be read
    # For simplicity, we just return True (assume powered)
    return True

if __name__ == "__main__":
    key_handle = epos.VCS_OpenDevice(
        DEVICE_NAME,
        PROTOCOL_STACK_NAME,
        INTERFACE_NAME,
        PORT_NAME,
        ctypes.byref(error_code)
    )

    if key_handle == 0:
        print("❌ Could not open device")
        exit(1)

    # --- 1) Set operation mode to velocity (3) ---
    mode = ctypes.c_int(3)
    if epos.VCS_SetOperationMode(key_handle, NODE_ID, mode, ctypes.byref(error_code)) != 0:
        print("✅ Operation Mode set to Velocity (3)")
    else:
        print(f"❌ Failed to set operation mode, error={error_code.value}")

    # --- 2) Explicitly change state sequence ---
    if epos.VCS_SetState(key_handle, NODE_ID, 2, ctypes.byref(error_code)) != 0:  # 2 = Switched On
        print("✅ State set to Switched On (2)")
    else:
        print(f"❌ Could not set state=2, error={error_code.value}")

    time.sleep(0.2)

    if epos.VCS_SetEnableState(key_handle, NODE_ID, ctypes.byref(error_code)) != 0:
        print("⚡ Motor enable command sent...")
    else:
        print(f"❌ Failed to enable motor, error={error_code.value}")

    # --- 3) Check state ---
    time.sleep(0.2)
    state = get_state(key_handle)
    print(f"📟 Current State = {state}")
    if state in (3, 8):
        print("✅ Motor is fully Operation Enabled")

        # --- 4) Read MaxProfileVelocity ---
        max_vel = get_max_profile_velocity(key_handle)
        print(f"📌 MaxProfileVelocity = {max_vel} rpm")
        if max_vel == 0:
            print("⚠️ MaxProfileVelocity is 0! Motor cannot move.")
        
        # --- 5) Check if motor is powered ---
        if not is_motor_powered(key_handle):
            print("⚠️ Motor power not detected! Check power supply or enable input.")
        else:
            # --- 6) Set velocity profile ---
            accel = ctypes.c_uint(1000)
            decel = ctypes.c_uint(1000)
            if epos.VCS_SetVelocityProfile(key_handle, NODE_ID, accel, decel, ctypes.byref(error_code)) != 0:
                print(f"✅ Velocity profile set (acc={accel.value}, dec={decel.value})")
            else:
                print(f"❌ Failed to set velocity profile, error={error_code.value}")

            # --- 7) Command motor velocity ---
            target_velocity = ctypes.c_long(min(500, max_vel))  # limit by MaxProfileVelocity
            if epos.VCS_MoveWithVelocity(key_handle, NODE_ID, target_velocity, ctypes.byref(error_code)) != 0:
                print(f"⚡ Motor running at {target_velocity.value} rpm")
            else:
                print(f"❌ Failed to start motor, error={error_code.value}")

            # --- 8) Monitor actual velocity for 5 seconds ---
            for i in range(10):
                time.sleep(0.5)
                actual_vel = get_actual_velocity(key_handle)
                print(f"📊 Actual velocity = {actual_vel} rpm")

            # --- 9) Stop motor ---
            if epos.VCS_MoveWithVelocity(key_handle, NODE_ID, ctypes.c_long(0), ctypes.byref(error_code)) != 0:
                print("🛑 Motor stopped")
            else:
                print(f"❌ Failed to stop motor, error={error_code.value}")

    else:
        print("⚠️ Still not in Operation Enabled — check if homing is mandatory")

    epos.VCS_CloseDevice(key_handle, ctypes.byref(error_code))
