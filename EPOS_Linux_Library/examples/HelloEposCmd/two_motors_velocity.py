#!/usr/bin/env python3
"""
Safe EPOS init for 2x EC-i40 motors (each on its own USB port)
with live plotting of target and actual velocity for both motors.
"""

import ctypes, time, sys
import matplotlib.pyplot as plt
from collections import deque

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"

# Each motor on its own USB port
MOTORS = [
    {"port": b"USB0", "node_id": 1},
    {"port": b"USB1", "node_id": 1},
]

TARGET_RPM = 5000
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
def open_device(port):
    return epos.VCS_OpenDevice(DEVICE_NAME, PROTOCOL_STACK, INTERFACE, port, ctypes.byref(error_code))

def close_device(h):
    epos.VCS_CloseDevice(h, ctypes.byref(error_code))

def get_state(h, nid):
    s = ctypes.c_ushort()
    if epos.VCS_GetState(h, nid, ctypes.byref(s), ctypes.byref(error_code)) != 0:
        return s.value
    return None

def get_fault(h, nid):
    f = ctypes.c_uint()
    if epos.VCS_GetFaultState(h, nid, ctypes.byref(f), ctypes.byref(error_code)) != 0:
        return f.value
    return None

def clear_fault(h, nid):
    return epos.VCS_ClearFault(h, nid, ctypes.byref(error_code)) != 0

def set_velocity_mode(h, nid):
    return epos.VCS_SetOperationMode(h, nid, ctypes.c_int(3), ctypes.byref(error_code)) != 0

def state_sequence_enable(h, nid):
    epos.VCS_SetState(h, nid, 2, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetState(h, nid, 3, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetEnableState(h, nid, ctypes.byref(error_code))
    time.sleep(0.12)

def set_velocity_profile(h, nid, acc, dec):
    return epos.VCS_SetVelocityProfile(h, nid, ctypes.c_uint(acc), ctypes.c_uint(dec), ctypes.byref(error_code)) != 0

def move_with_velocity(h, nid, rpm):
    return epos.VCS_MoveWithVelocity(h, nid, ctypes.c_long(int(rpm)), ctypes.byref(error_code)) != 0

def halt_velocity(h, nid):
    if hasattr(epos, "VCS_HaltVelocityMovement"):
        epos.VCS_HaltVelocityMovement(h, nid, ctypes.byref(error_code))
    else:
        epos.VCS_MoveWithVelocity(h, nid, ctypes.c_long(0), ctypes.byref(error_code))

def get_velocity_is(h, nid):
    v = ctypes.c_long()
    if epos.VCS_GetVelocityIs(h, nid, ctypes.byref(v), ctypes.byref(error_code)) != 0:
        return v.value
    return None


# ----------------- Main flow -----------------
if __name__ == "__main__":
    # open both devices
    for m in MOTORS:
        m["handle"] = open_device(m["port"])
        if m["handle"] == 0:
            print(f"❌ Could not open device on {m['port'].decode()}")
            sys.exit(1)
        print(f"✅ Device opened on {m['port'].decode()}")

        nid = m["node_id"]
        h = m["handle"]

        # clear fault
        f = get_fault(h, nid)
        if f:
            print(f"⚠️ Fault on {m['port'].decode()}: 0x{f:08X} — clearing")
            clear_fault(h, nid)
            time.sleep(0.2)

        # set velocity mode
        set_velocity_mode(h, nid)
        state_sequence_enable(h, nid)
        set_velocity_profile(h, nid, ACC_RPM_S, DEC_RPM_S)

        print(f"✅ Motor on {m['port'].decode()} ready")

    # start both
    for m in MOTORS:
        move_with_velocity(m["handle"], m["node_id"], TARGET_RPM)

    print(f"\n⚡ Both motors running at {TARGET_RPM} rpm for 10s")

    # --- live plot ---
    plt.ion()
    fig, ax = plt.subplots()

    time_window = 100
    times = deque(maxlen=time_window)
    actual_vels = {m["port"]: deque(maxlen=time_window) for m in MOTORS}

    line_target, = ax.plot([], [], 'r--', label="Target Velocity [rpm]")
    lines_actual = {
        m["port"]: ax.plot([], [], label=f"{m['port'].decode()} Actual [rpm]")[0]
        for m in MOTORS
    }

    ax.set_xlabel("Sample")
    ax.set_ylabel("Velocity [rpm]")
    ax.legend()
    plt.title("Live EPOS Velocity (2 Motors / 2 USB ports)")
    fig.tight_layout()

    # --- loop ---
    t0 = time.time()
    sample = 0
    try:
        while time.time() - t0 < 10.0:
            times.append(sample)
            for m in MOTORS:
                v = get_velocity_is(m["handle"], m["node_id"])
                actual_vels[m["port"]].append(v if v is not None else 0)

            # plot
            line_target.set_data(times, [TARGET_RPM] * len(times))
            for m in MOTORS:
                lines_actual[m["port"]].set_data(times, actual_vels[m["port"]])

            ax.set_xlim(max(0, sample - time_window), sample)
            vmin = min(min(vals) for vals in actual_vels.values()) - 10
            vmax = max(max(vals) for vals in actual_vels.values()) + 10
            ax.set_ylim(vmin, vmax)

            fig.canvas.draw()
            fig.canvas.flush_events()

            print("📊 " + " | ".join([f"{p.decode()}: {actual_vels[p][-1]} rpm" for p in actual_vels]))

            sample += 1
            time.sleep(0.2)
    finally:
        for m in MOTORS:
            halt_velocity(m["handle"], m["node_id"])
            close_device(m["handle"])
        plt.ioff()
        plt.show()

    print("🛑 Test finished, all devices closed")
