#!/usr/bin/env python3
import ctypes

# Load EPOS Command Library
epos = ctypes.cdll.LoadLibrary("libEposCmd.so")

DEVICE_NAME = b"EPOS4"
PROTOCOL_STACK_NAME = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
BAUDRATE = 1000000
NODE_ID_RANGE = range(1, 5)  # adjust if you use Node IDs >4

error_code = ctypes.c_uint()

def scan_port(port_name):
    """Scan one USB port for EPOS devices by Node ID."""
    print(f"\n🔍 Scanning {port_name.decode()}...")

    for node_id in NODE_ID_RANGE:
        key_handle = epos.VCS_OpenDevice(
            DEVICE_NAME,
            PROTOCOL_STACK_NAME,
            INTERFACE_NAME,
            port_name,
            ctypes.byref(error_code)
        )

        if key_handle == 0:
            continue

        # Try to get device state
        state = ctypes.c_ushort()
        ret = epos.VCS_GetState(
            key_handle, node_id,
            ctypes.byref(state),
            ctypes.byref(error_code)
        )

        if ret != 0:
            print(f"✅ Found EPOS4 on {port_name.decode()}, Node {node_id}, State={state.value}")

        epos.VCS_CloseDevice(key_handle, ctypes.byref(error_code))

if __name__ == "__main__":
    # Assume you have USB0 and USB1
    for port in [b"USB0", b"USB1"]:
        scan_port(port)
