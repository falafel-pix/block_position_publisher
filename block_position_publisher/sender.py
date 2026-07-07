#!/usr/bin/env python3
"""
Windows Controller Reader - Zero dependencies
Reads controller via Windows API (ctypes) and sends to WSL2 via UDP
"""
import ctypes
from ctypes import wintypes
import socket
import json
import time
import struct

# Windows API structures
class JOYINFOEX(ctypes.Structure):
    _fields_ = [
        ("dwSize", wintypes.DWORD),
        ("dwFlags", wintypes.DWORD),
        ("dwXpos", wintypes.DWORD),
        ("dwYpos", wintypes.DWORD),
        ("dwZpos", wintypes.DWORD),
        ("dwRpos", wintypes.DWORD),
        ("dwUpos", wintypes.DWORD),
        ("dwVpos", wintypes.DWORD),
        ("dwButtons", wintypes.DWORD),
        ("dwButtonNumber", wintypes.DWORD),
        ("dwPOV", wintypes.DWORD),
        ("dwReserved1", wintypes.DWORD),
        ("dwReserved2", wintypes.DWORD),
    ]

# Constants
JOY_RETURNX = 0x00000001
JOY_RETURNY = 0x00000002
JOY_RETURNZ = 0x00000004
JOY_RETURNR = 0x00000008
JOY_RETURNU = 0x00000010
JOY_RETURNV = 0x00000020
JOY_RETURNPOV = 0x00000040
JOY_RETURNBUTTONS = 0x00000080
JOY_RETURNALL = (JOY_RETURNX | JOY_RETURNY | JOY_RETURNZ |
                 JOY_RETURNR | JOY_RETURNU | JOY_RETURNV |
                 JOY_RETURNPOV | JOY_RETURNBUTTONS)

# WSL2 IP target IP of wsl here - 
WSL_IP = ""
UDP_PORT = 5555

def main():
    # Load WinMM DLL
    winmm = ctypes.windll.winmm
    
    # Check if joystick is connected
    joy_id = 0
    result = winmm.joyGetNumDevs()
    if result == 0:
        print("No joystick detected!")
        return
    
    print(f"Joystick devices available: {result}")
    
    # Setup JOYINFOEX
    joy_info = JOYINFOEX()
    joy_info.dwSize = ctypes.sizeof(JOYINFOEX)
    joy_info.dwFlags = JOY_RETURNALL
    
    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Sending controller data to {WSL_IP}:{UDP_PORT}")
    print("Press Ctrl+C to quit\n")
    
    prev_buttons = 0
    prev_axes = [0.0] * 6
    
    try:
        while True:
            result = winmm.joyGetPosEx(joy_id, ctypes.byref(joy_info))
            
            if result == 0:  # JOYERR_NOERROR
                # Extract buttons (up to 32 buttons)
                buttons = []
                for i in range(32):
                    buttons.append(1 if (joy_info.dwButtons & (1 << i)) else 0)
                
                # Convert axes to -1.0 to 1.0 range
                # dwXpos, dwYpos, etc. range from 0 to 65535
                axes = [
                    (joy_info.dwXpos - 32767) / 32767.0,
                    (joy_info.dwYpos - 32767) / 32767.0,
                    (joy_info.dwZpos - 32767) / 32767.0,
                    (joy_info.dwRpos - 32767) / 32767.0,
                    (joy_info.dwUpos - 32767) / 32767.0,
                    (joy_info.dwVpos - 32767) / 32767.0,
                ]
                
                # Detect button changes
                for i in range(32):
                    curr = buttons[i]
                    prev = (prev_buttons >> i) & 1
                    if curr != prev:
                        status = "PRESSED" if curr else "RELEASED"
                        print(f"Button {i}: {status}")
                
                # Detect axis changes
                for i, (curr, prev) in enumerate(zip(axes, prev_axes)):
                    if abs(curr - prev) > 0.05:
                        print(f"Axis {i}: {curr:.3f}")
                
                # Send via UDP
                data = json.dumps({"buttons": buttons, "axes": axes})
                sock.sendto(data.encode(), (WSL_IP, UDP_PORT))
                
                prev_buttons = joy_info.dwButtons
                prev_axes = axes
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
