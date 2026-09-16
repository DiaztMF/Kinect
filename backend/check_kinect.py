"""
Kinect for Windows v1 / Xbox 360 Hardware and SDK Diagnostics.
Checks connected USB devices and Microsoft Kinect SDK 1.8 installation status.
"""

import os
import platform
import subprocess
import sys

TARGET_DEVICES = {
    "USB\\VID_045E&PID_02B0": "Xbox NUI Motor",
    "USB\\VID_045E&PID_02AE": "Xbox NUI Camera",
    "USB\\VID_045E&PID_02AD": "Xbox NUI Audio",
}

SDK_PATHS = [
    r"C:\Windows\System32\Kinect10.dll",
    r"C:\Windows\SysWOW64\Kinect10.dll",
    r"C:\Program Files\Microsoft SDKs\Kinect\v1.8",
    r"C:\Program Files\Microsoft SDKs\Kinect\v1.8\Assemblies\Microsoft.Kinect.dll",
]

def check_usb_devices_windows():
    detected = {}
    for hw_id in TARGET_DEVICES:
        detected[hw_id] = False

    try:
        cmd = [
            "powershell",
            "-NoProfile",
            "-Command",
            "Get-PnpDevice | Select-Object -Property InstanceId, FriendlyName, Status | ConvertTo-Json"
        ]
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        output = res.stdout.upper()
        for hw_id in TARGET_DEVICES:
            if hw_id.upper() in output:
                detected[hw_id] = True
    except Exception as e:
        print(f"[WARN] Failed querying PnP devices: {e}", file=sys.stderr)

    return detected

def check_sdk_files():
    found = {}
    for p in SDK_PATHS:
        found[p] = os.path.exists(p)
    return found

def main():
    print("=" * 60)
    print(" KINECT HARDWARE & SDK DIAGNOSTIC REPORT")
    print("=" * 60)
    print(f"OS: {platform.system()} {platform.release()} ({platform.architecture()[0]})")
    print(f"Python: {sys.version.split()[0]}")
    print("-" * 60)

    # 1. Hardware Status
    print("[1] USB Device Detection:")
    devices = check_usb_devices_windows() if platform.system() == "Windows" else {k: False for k in TARGET_DEVICES}
    all_devices_connected = True
    for hw_id, name in TARGET_DEVICES.items():
        present = devices.get(hw_id, False)
        status_str = "[OK] CONNECTED" if present else "[MISSING] NOT FOUND"
        print(f"  - {name} ({hw_id}): {status_str}")
        if not present:
            all_devices_connected = False

    print("\n[2] SDK 1.8 Files & Libraries:")
    sdk_checks = check_sdk_files()
    kinect_dll_found = sdk_checks.get(r"C:\Windows\System32\Kinect10.dll", False) or sdk_checks.get(r"C:\Windows\SysWOW64\Kinect10.dll", False)
    sdk_dir_found = sdk_checks.get(r"C:\Program Files\Microsoft SDKs\Kinect\v1.8", False)

    for path, present in sdk_checks.items():
        status_str = "[OK] PRESENT" if present else "[MISSING]"
        print(f"  - {path}: {status_str}")

    print("-" * 60)
    print("DIAGNOSTIC SUMMARY:")
    if all_devices_connected:
        print("  - Hardware: All 3 Kinect endpoints detected.")
    else:
        print("  - Hardware: One or more Kinect endpoints missing.")
        print("    -> Ensure Kinect USB is plugged into USB 2.0/3.0 port and power adapter is ON.")

    if kinect_dll_found and sdk_dir_found:
        print("  - SDK 1.8: Runtime & SDK properly installed.")
    else:
        print("  - SDK 1.8: Missing or incomplete installation.")
        print("\nSETUP / INSTALLATION GUIDE:")
        print("  1. Download 'Kinect for Windows SDK v1.8':")
        print("     https://www.microsoft.com/en-us/download/details.aspx?id=40278")
        print("  2. (Optional) Download 'Kinect for Windows Developer Toolkit v1.8':")
        print("     https://www.microsoft.com/en-us/download/details.aspx?id=40276")
        print("  3. Run KinectSDK-v1.8-Setup.exe as Administrator.")
        print("  4. Unplug and reconnect Kinect USB cable.")
        print("  5. Re-run this check script.")
    print("=" * 60)

if __name__ == "__main__":
    main()
