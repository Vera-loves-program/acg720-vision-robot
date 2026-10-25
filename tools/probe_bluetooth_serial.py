"""Windows serial open/read diagnostic. Never sends bytes or motor commands.

Close VOFA/other serial clients before comparing their result with this probe.
Opening a Bluetooth COM may initiate the already-paired radio connection;
this tool does not pair, remove devices, change baud/line settings, or toggle
DTR/RTS. No pyserial, admin or third-party dependency is required.
"""
import argparse
import ctypes
from ctypes import wintypes
import os
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', help='Previously verified robot port, e.g.COM8')
    parser.add_argument('--open-only', action='store_true')
    args = parser.parse_args()
    if os.name != 'nt' or not re.fullmatch(r'COM[1-9]\d*', args.port.upper()):
        parser.error('Run on Windows with an explicit COMn port')
    port = args.port.upper()
    api = ctypes.WinDLL('kernel32', use_last_error=True)
    api.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
        wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    api.CreateFileW.restype = wintypes.HANDLE
    api.CloseHandle.argtypes = [wintypes.HANDLE]
    api.CloseHandle.restype = wintypes.BOOL
    handle = api.CreateFileW('\\\\.\\' + port, 0xC0000000, 0, None, 3, 0, None)
    if handle == wintypes.HANDLE(-1).value:
        code = ctypes.get_last_error()
        print(f'OPEN_FAILED {port}: Win32={code}; {ctypes.FormatError(code).strip()}')
        return 1
    try:
        print(f'OPEN_OK {port}; exclusive handle acquired. No bytes sent.')
        if args.open_only:
            return 0
        class Timeouts(ctypes.Structure):
            _fields_ = [(name, wintypes.DWORD) for name in (
                'ReadIntervalTimeout', 'ReadTotalTimeoutMultiplier',
                'ReadTotalTimeoutConstant', 'WriteTotalTimeoutMultiplier',
                'WriteTotalTimeoutConstant')]
        api.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(Timeouts)]
        api.SetCommTimeouts.restype = wintypes.BOOL
        deadlines = Timeouts(0xFFFFFFFF, 0, 1500, 0, 0)
        if not api.SetCommTimeouts(handle, ctypes.byref(deadlines)):
            code = ctypes.get_last_error()
            print(f'TIMEOUT_SETUP_FAILED Win32={code}; {ctypes.FormatError(code).strip()}')
            return 2
        api.ReadFile.argtypes = [wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
        api.ReadFile.restype = wintypes.BOOL
        buffer, received = ctypes.create_string_buffer(256), wintypes.DWORD()
        if not api.ReadFile(handle, buffer, len(buffer), ctypes.byref(received), None):
            code = ctypes.get_last_error()
            print(f'READ_FAILED Win32={code}; {ctypes.FormatError(code).strip()}')
            return 3
        print(f'READ_BYTES {received.value}: {bytes(buffer[:received.value])!r}')
        return 0
    finally:
        api.CloseHandle(handle)
        print('CLOSED; no robot commands were sent.')


if __name__ == '__main__':
    raise SystemExit(main())
