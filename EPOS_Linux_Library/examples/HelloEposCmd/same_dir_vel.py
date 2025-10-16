#!/usr/bin/env python3
"""
Two-motor velocity control (EPOS4 via USB)
Crawler setup: one motor inverted.
Live plot: target vs actual velocity for both motors on same axes.
"""

import ctypes, time, sys
import matplotlib.pyplot as plt
from collections import deque

# ----------------- CONFIG -----------------
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE = b"USB"

# Motors: port, node_id, invert (for crawler), target rpm
MOTORS = [
    {"port": b"USB0", "node_id": 1, "invert": False, "target": 5000},  # Left motor
    {"port": b"USB1", "node_id": 1, "invert": True,  "target": 5000},  # Right motor (inverted)
]

ACC_RPM_S = 1000
DEC_RPM_S = 1000
TEST_DURATION = 10.0   # seconds
SAMPLE_PERIOD = 0.2    # seconds

# ----------------------------------------------

# Load library
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
    time.sleep(0.1)
    epos.VCS_SetState(h, nid, 3, ctypes.byref(error_code))
    time.sleep(0.1)
    epos.VCS_SetEnableState(h, nid, ctypes.byref(error_code))
    time.sleep(0.1)

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

        # clear fault if needed
        f = get_fault(m["handle"], m["node_id"])
        if f:
            print(f"⚠️ Fault on {m['port'].decode()}: 0x{f:08X} — clearing")
            clear_fault(m["handle"], m["node_id"])
            time.sleep(0.2)

        # setup
        set_velocity_mode(m["handle"], m["node_id"])
        state_sequence_enable(m["handle"], m["node_id"])
        set_velocity_profile(m["handle"], m["node_id"], ACC_RPM_S, DEC_RPM_S)

    # start both motors
    for m in MOTORS:
        rpm = m["target"] if not m["invert"] else -m["target"]
        move_with_velocity(m["handle"], m["node_id"], rpm)

    print(f"\n⚡ Both motors running (crawler mode) for {TEST_DURATION}s")

    # --- live plot setup ---
    plt.ion()
    fig, ax = plt.subplots()

    time_window = int(TEST_DURATION / SAMPLE_PERIOD)
    times = deque(maxlen=time_window)

    actual_vels = {m["port"]: deque(maxlen=time_window) for m in MOTORS}
    target_vels = {m["port"]: deque(maxlen=time_window) for m in MOTORS}

    # make line objects for each motor
    styles = ["r", "b"]  # red for left, blue for right
    lines = {}
    for idx, m in enumerate(MOTORS):
        port = m["port"]
        # dashed = target, solid = actual
        lines[(port, "target")], = ax.plot([], [], styles[idx] + "--", label=f"{port.decode()} Target [rpm]")
        lines[(port, "actual")], = ax.plot([], [], styles[idx] + "-",  label=f"{port.decode()} Actual [rpm]")

    ax.set_xlabel("Sample")
    ax.set_ylabel("Velocity [rpm]")
    ax.legend()
    plt.title("Live EPOS Velocity (2 Motors, crawler setup)")
    fig.tight_layout()

    # --- loop ---
    t0 = time.time()
    sample = 0
    try:
        while time.time() - t0 < TEST_DURATION:
            times.append(sample)
            for m in MOTORS:
                port = m["port"]
                nid = m["node_id"]

                v = get_velocity_is(m["handle"], nid)
                # adjust for inversion so crawler forward motion is positive
                v_adj = -v if (m["invert"] and v is not None) else v

                actual_vels[port].append(v_adj if v_adj is not None else 0)
                target_vels[port].append(m["target"])

            # update plot
            for m in MOTORS:
                port = m["port"]
                lines[(port, "target")].set_data(times, target_vels[port])
                lines[(port, "actual")].set_data(times, actual_vels[port])

            # only set x-limits if more than 1 sample
            if sample > 1:
                ax.set_xlim(max(0, sample - time_window), sample)

            if sample > 1:
                vmin = min(min(vals) for vals in actual_vels.values()) - 50
                vmax = max(max(vals) for vals in actual_vels.values()) + 50

                # sanity clamp (in case of bad first readings)
                if vmin < -10000: 
                    vmin = -6000
                if vmax > 10000:
                    vmax = 6000

                ax.set_ylim(vmin, vmax)


            fig.canvas.draw()
            fig.canvas.flush_events()

            # print console log
            status = " | ".join([f"{p.decode()} T={target_vels[p][-1]}, A={actual_vels[p][-1]}" for p in actual_vels])
            print("📊", status)

            sample += 1
            time.sleep(SAMPLE_PERIOD)
    finally:
        for m in MOTORS:
            halt_velocity(m["handle"], m["node_id"])
            close_device(m["handle"])
        plt.ioff()
        plt.show()

    print("🛑 Test finished, all devices closed")
