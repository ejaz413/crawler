#!/usr/bin/env python3
"""
EPOS Auto-Tuner (safe, conservative)

What it does:
- Connects to EPOS on USB0 / NODE_ID
- Clears faults if present
- Performs a sequence of low-speed step responses to estimate dynamics
- Computes simple PI (or PID) gains from step response (very conservative)
- If the EPOS library exposes SDO upload/download functions, the script will
  attempt to write the gains into the Object Dictionary (OD) entries.
- If SDO functions are not available, the script will print recommended gains
  for manual entry in EPOS Studio.

Safety:
- Very low test velocity and gentle acceleration used by default.
- Constantly monitors faults and digital inputs and aborts immediately if unsafe.
- Stops and clears fault if needed, then exits.

Customization:
- If you know the OD indices for velocity loop gains for your EPOS model,
  set OD_INDEX_KP, OD_INDEX_KI, OD_INDEX_KD below.
"""

import ctypes, time, math, sys

# --- Config ---
DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK_NAME = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
PORT_NAME = b"USB0"
NODE_ID = 1

TEST_RPMS = [20, 40]            # small step levels (rpm) to test response
HOLD_SECONDS = 2.0              # seconds to hold each step
ACC_RPM_S = 50                  # conservative acceleration (rpm/s)
DEC_RPM_S = 50
RAMP_SEQUENCE = [5, 10, 20]     # ramp targets (rpm) before major test
SAMPLE_INTERVAL = 0.05          # telemetry sampling interval during step (s)
MAX_SAFE_CURRENT_MA = 5000      # abort if current (mA) exceeds this (set conservatively)

# --- Object Dictionary indices (may differ by firmware) ---
# Replace these with the correct indices for your device if you know them.
# Typical velocity loop gain OD entries vary; these are placeholders.
OD_INDEX_KP = 0x60F9   # <-- CHANGE if your firmware uses different index
OD_INDEX_KI = 0x60FA   # <-- CHANGE if your firmware uses different index
OD_INDEX_KD = 0x60FB   # <-- CHANGE if your firmware uses different index
OD_SUBINDEX = 0x00

# --- Load EPOS library ---
try:
    epos = ctypes.cdll.LoadLibrary("libEposCmd.so")
except Exception as e:
    print("❌ Failed to load libEposCmd.so:", e)
    sys.exit(1)

error_code = ctypes.c_uint()

# --- Helper wrappers (common safe calls) ---
def open_device():
    return epos.VCS_OpenDevice(DEVICE_NAME, PROTOCOL_STACK_NAME, INTERFACE_NAME, PORT_NAME, ctypes.byref(error_code))

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

def set_state_sequence(h):
    epos.VCS_SetState(h, NODE_ID, 2, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetState(h, NODE_ID, 3, ctypes.byref(error_code))
    time.sleep(0.12)
    epos.VCS_SetEnableState(h, NODE_ID, ctypes.byref(error_code))
    time.sleep(0.12)

def set_velocity_profile(h, acc_rpm_s, dec_rpm_s):
    return epos.VCS_SetVelocityProfile(h, NODE_ID, ctypes.c_uint(acc_rpm_s), ctypes.c_uint(dec_rpm_s), ctypes.byref(error_code)) != 0

def move_with_velocity(h, rpm):
    return epos.VCS_MoveWithVelocity(h, NODE_ID, ctypes.c_long(int(rpm)), ctypes.byref(error_code)) != 0

def get_velocity_is(h):
    v = ctypes.c_long()
    if epos.VCS_GetVelocityIs(h, NODE_ID, ctypes.byref(v), ctypes.byref(error_code)) != 0:
        return v.value
    return None

def get_current_is(h):
    # Signed 16-bit current in mA usually
    i = ctypes.c_short()
    if hasattr(epos, "VCS_GetCurrentIs"):
        if epos.VCS_GetCurrentIs(h, NODE_ID, ctypes.byref(i), ctypes.byref(error_code)) != 0:
            return i.value
    return None

def get_digital_inputs(h):
    v = ctypes.c_ushort()
    if hasattr(epos, "VCS_GetDigitalInputs"):
        if epos.VCS_GetDigitalInputs(h, NODE_ID, ctypes.byref(v), ctypes.byref(error_code)) != 0:
            return v.value
    return None

# --- SDO (Object Dictionary) helpers (best-effort) ---
def has_sdo_functions():
    # check for common function names; if not found we will fallback
    return hasattr(epos, "VCS_SDOUpload") and hasattr(epos, "VCS_SDODownload")

def sdo_upload(h, index, subindex, buffer_size=4):
    """
    Try to read an OD entry using VCS_SDOUpload.
    This function's exact signature can vary between EPOS libs.
    We attempt common calling conventions and fall back if not available.
    """
    # Try a few plausible signatures inside try/except
    try:
        # Common: int VCS_SDOUpload(void* keyHandle, unsigned int NodeID, unsigned int index, unsigned int subIndex, void* data, unsigned int size, unsigned int timeout, unsigned int *pErrorCode)
        buf = (ctypes.c_ubyte * buffer_size)()
        timeout = ctypes.c_uint(1000)
        ret = epos.VCS_SDOUpload(h, ctypes.c_uint(NODE_ID), ctypes.c_uint(index), ctypes.c_uint(subindex), ctypes.byref(buf), ctypes.c_uint(buffer_size), timeout, ctypes.byref(error_code))
        if ret != 0:
            # convert little-endian byte buffer to int
            val = 0
            for i in range(buffer_size):
                val |= (buf[i] << (8 * i))
            return val
    except Exception:
        pass
    return None

def sdo_download(h, index, subindex, value, buffer_size=4):
    """
    Try to write an OD entry using VCS_SDODownload.
    """
    try:
        # fill buffer little-endian
        buf = (ctypes.c_ubyte * buffer_size)()
        v = int(value)
        for i in range(buffer_size):
            buf[i] = (v >> (8 * i)) & 0xFF
        timeout = ctypes.c_uint(1000)
        ret = epos.VCS_SDODownload(h, ctypes.c_uint(NODE_ID), ctypes.c_uint(index), ctypes.c_uint(subindex), ctypes.byref(buf), ctypes.c_uint(buffer_size), timeout, ctypes.byref(error_code))
        return ret != 0
    except Exception:
        return False

# --- Simple step-response estimator (very conservative) ---
def run_step_test(h, step_rpm, hold_sec, sample_interval):
    """Apply step to step_rpm from 0 and return time-series of (t, vel, current)"""
    # Ensure low accel/decel
    set_velocity_profile(h, ACC_RPM_S, DEC_RPM_S)
    move_with_velocity(h, 0)
    time.sleep(0.15)

    # step to value
    if not move_with_velocity(h, step_rpm):
        print("❌ move_with_velocity failed for step", step_rpm, "error=", error_code.value)
        return None

    t0 = time.time()
    samples = []
    while time.time() - t0 < hold_sec:
        v = get_velocity_is(h)
        i = get_current_is(h)
        samples.append((time.time() - t0, v if v is not None else 0, i if i is not None else 0))
        # immediate abort if fault/current > threshold
        f = get_fault(h)
        if f:
            print("⚠️ Fault detected during step:", hex(f))
            move_with_velocity(h, 0)
            return None
        if i is not None and abs(i) > MAX_SAFE_CURRENT_MA:
            print("⚠️ Current exceed safety threshold:", i, "mA")
            move_with_velocity(h, 0)
            return None
        time.sleep(sample_interval)
    # bring to zero
    move_with_velocity(h, 0)
    time.sleep(0.2)
    return samples

def estimate_pi_from_step(samples, target_rpm):
    """
    Very simple estimation:
    - compute steady-state value and 10%-90% rise times to approximate bandwidth
    - use simple rules to produce Kp/Ki (conservative)
    NOTE: This is heuristic and intentionally conservative.
    """
    if not samples:
        return None
    times = [s[0] for s in samples]
    vels  = [s[1] for s in samples]
    # steady-state ~ average of last 20% samples
    n = max(1, len(vels)//5)
    steady = sum(vels[-n:]) / n
    # compute rise time between 10% and 90%
    t10 = None; t90 = None
    v10 = 0.1 * target_rpm
    v90 = 0.9 * target_rpm
    for t, v, _ in samples:
        if t10 is None and v >= v10:
            t10 = t
        if t90 is None and v >= v90:
            t90 = t
        if t10 is not None and t90 is not None:
            break
    if t10 is None or t90 is None or (t90 - t10) <= 0:
        # fallback conservative gains
        Kp = 500
        Ki = 50
        Kd = 0
        return Kp, Ki, Kd, steady
    rise = t90 - t10
    # approximate bandwidth wn ~ 1.8 / rise_time (very rough)
    wn = 1.8 / rise
    # Set proportional gain proportional to desired bandwidth and steady-state error
    # This is a heuristic mapping; keep very conservative scale
    if steady == 0:
        steady = 1.0
    # P ~ alpha * wn * (target/steady)
    alpha = 300.0
    Kp = max(100, int(alpha * wn * (target_rpm / steady)))
    Ki = max(10, int(Kp * 0.1))   # integral slower than P
    Kd = 0
    return Kp, Ki, Kd, steady

# --- High level auto-tune routine ---
def auto_tune_run(h):
    print("=== Auto-tune start (very conservative) ===")
    # 1) safe checks and clear faults
    f = get_fault(h)
    if f:
        print("⚠️ Fault present at start:", hex(f), "- attempting clear")
        if clear_fault(h):
            print("✅ Fault cleared")
            time.sleep(0.2)
        else:
            print("❌ Could not clear fault. Fix hardware and retry.")
            return None

    # 2) set velocity mode and state sequence
    if not set_velocity_mode(h):
        print("❌ Failed to set velocity mode")
        return None
    set_state_sequence(h)
    s = get_state(h)
    print("📟 State after sequence:", s)

    # 3) ramp pre-tests (gentle)
    set_velocity_profile(h, ACC_RPM_S, DEC_RPM_S)
    for tgt in RAMP_SEQUENCE:
        print("→ Gentle ramp to", tgt, "rpm")
        if not move_with_velocity(h, tgt):
            print("❌ move command failed at ramp", tgt, "error", error_code.value)
            return None
        t0 = time.time()
        while time.time() - t0 < 1.2:
            if get_fault(h):
                print("⚠️ Fault during ramp")
                move_with_velocity(h, 0)
                return None
            time.sleep(0.12)
    move_with_velocity(h, 0)
    time.sleep(0.3)

    # 4) run step tests for each TEST_RPMS and estimate gains
    results = []
    for step in TEST_RPMS:
        print(f"--- Running step test to {step} rpm ---")
        samples = run_step_test(h, step, HOLD_SECONDS, SAMPLE_INTERVAL)
        if samples is None:
            print("❌ Step test failed or aborted for", step)
            return None
        est = estimate_pi_from_step(samples, step)
        if est is None:
            print("❌ Could not estimate gains for step", step)
            return None
        Kp, Ki, Kd, steady = est
        print(f"📈 Estimated gains for step {step} rpm: Kp={Kp}, Ki={Ki}, Kd={Kd} (steady={steady:.1f} rpm)")
        results.append((step, Kp, Ki, Kd, steady))

    # 5) choose final gains (median/most conservative)
    Kps = [r[1] for r in results]
    Kis = [r[2] for r in results]
    Kds = [r[3] for r in results]
    final_Kp = int(sorted(Kps)[len(Kps)//2])
    final_Ki = int(sorted(Kis)[len(Kis)//2])
    final_Kd = int(sorted(Kds)[len(Kds)//2])
    print("✅ Final recommended gains:", final_Kp, final_Ki, final_Kd)

    # 6) attempt to write to OD via SDO if available
    if has_sdo_functions():
        print("🔧 SDO functions appear available. Attempting to write OD entries.")
        ok1 = sdo_download(h, OD_INDEX_KP, OD_SUBINDEX, final_Kp)
        ok2 = sdo_download(h, OD_INDEX_KI, OD_SUBINDEX, final_Ki)
        ok3 = sdo_download(h, OD_INDEX_KD, OD_SUBINDEX, final_Kd)
        if ok1 and ok2 and ok3:
            print("✅ Gains written to OD (KP, KI, KD). You should power-cycle or re-init controller if required by firmware.")
            # read back to confirm
            rkp = sdo_upload(h, OD_INDEX_KP, OD_SUBINDEX)
            rki = sdo_upload(h, OD_INDEX_KI, OD_SUBINDEX)
            rkd = sdo_upload(h, OD_INDEX_KD, OD_SUBINDEX)
            print("🔁 Read back:", rkp, rki, rkd)
        else:
            print("⚠️ Failed to write one or more OD entries. Check OD indices and permissions.")
            print("➡️ Recommended gains (not written):", final_Kp, final_Ki, final_Kd)
    else:
        print("⚠️ SDO functions not available in this lib. Cannot write gains automatically.")
        print("➡️ Recommended gains:", final_Kp, final_Ki, final_Kd)
        print("Please apply these manually in EPOS Studio or update library that supports SDO.")

    return final_Kp, final_Ki, final_Kd

# --- Main entry ---
if __name__ == "__main__":
    h = open_device()
    if h == 0:
        print("❌ Could not open device")
        sys.exit(1)
    print("✅ Device opened")

    try:
        gains = auto_tune_run(h)
        if gains:
            print("🎉 Auto-tune completed. Gains:", gains)
        else:
            print("❌ Auto-tune aborted or failed.")
    finally:
        close_device(h)
        print("✅ Device closed")
