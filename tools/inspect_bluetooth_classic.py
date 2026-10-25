"""Read cached Windows Classic Bluetooth identity/installed-service state.

Uses actual Bluetooth radio handles. Never opens a COM port, performs inquiry,
authenticates, pairs/removes a device, changes a service, or resets an adapter.
Installed services are local enabled-service records, not a live SDP probe.
"""
from __future__ import annotations

import argparse
import ctypes as C
from ctypes import wintypes as W
import json
import os
import re
import sys
import uuid


class Address(C.Structure):
    _fields_ = [('ullLong', C.c_ulonglong)]


class SystemTime(C.Structure):
    _fields_ = [(field, W.WORD) for field in (
        'year', 'month', 'day_of_week', 'day', 'hour', 'minute', 'second', 'milliseconds')]


class DeviceInfo(C.Structure):
    _fields_ = [('dwSize', W.DWORD), ('Address', Address),
                ('ulClassofDevice', W.ULONG), ('fConnected', W.BOOL),
                ('fRemembered', W.BOOL), ('fAuthenticated', W.BOOL),
                ('stLastSeen', SystemTime), ('stLastUsed', SystemTime),
                ('szName', W.WCHAR * 248)]


class RadioInfo(C.Structure):
    _fields_ = [('dwSize', W.DWORD), ('address', Address),
                ('szName', W.WCHAR * 248), ('ulClassofDevice', W.ULONG),
                ('lmpSubversion', W.USHORT), ('manufacturer', W.USHORT)]


class FindRadioParams(C.Structure):
    _fields_ = [('dwSize', W.DWORD)]


class SearchParams(C.Structure):
    _fields_ = [('dwSize', W.DWORD), ('fReturnAuthenticated', W.BOOL),
                ('fReturnRemembered', W.BOOL), ('fReturnUnknown', W.BOOL),
                ('fReturnConnected', W.BOOL), ('fIssueInquiry', W.BOOL),
                ('cTimeoutMultiplier', W.BYTE), ('hRadio', W.HANDLE)]


class Guid(C.Structure):
    _fields_ = [('bytes', W.BYTE * 16)]


def sized(kind):
    value = kind()
    value.dwSize = C.sizeof(kind)
    return value


def error(code):
    return {'code': int(code), 'message': C.FormatError(code).strip() if code else ''}


def time_text(value):
    if not value.year:
        return None
    return (f'{value.year:04}-{value.month:02}-{value.day:02} '
            f'{value.hour:02}:{value.minute:02}:{value.second:02}')


def device_text(value):
    return {'address': f'{value.Address.ullLong:012X}', 'name': value.szName,
            # Win32 BOOL is nonzero, not necessarily exactly1.
            'authenticated': bool(value.fAuthenticated),
            'remembered': bool(value.fRemembered), 'connected': bool(value.fConnected),
            'raw_flags': [value.fAuthenticated, value.fRemembered, value.fConnected],
            'last_seen': time_text(value.stLastSeen), 'last_used': time_text(value.stLastUsed)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', default='04250224011F', help='Target Classic address (12 hex digits)')
    args = parser.parse_args(argv)
    target = args.address.replace(':', '').replace('-', '')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', target):
        parser.error('address must contain exactly12 hexadecimal digits')
    if os.name != 'nt':
        parser.error('Windows only')
    target_number = int(target, 16)
    api = C.WinDLL('bthprops.cpl', use_last_error=True)
    kernel = C.WinDLL('kernel32', use_last_error=True)
    api.BluetoothFindFirstRadio.argtypes = [C.POINTER(FindRadioParams), C.POINTER(W.HANDLE)]
    api.BluetoothFindFirstRadio.restype = W.HANDLE
    api.BluetoothFindNextRadio.argtypes = [W.HANDLE, C.POINTER(W.HANDLE)]
    api.BluetoothFindNextRadio.restype = W.BOOL
    api.BluetoothFindRadioClose.argtypes = [W.HANDLE]
    api.BluetoothFindRadioClose.restype = W.BOOL
    api.BluetoothGetRadioInfo.argtypes = [W.HANDLE, C.POINTER(RadioInfo)]
    api.BluetoothGetRadioInfo.restype = W.DWORD
    api.BluetoothGetDeviceInfo.argtypes = [W.HANDLE, C.POINTER(DeviceInfo)]
    api.BluetoothGetDeviceInfo.restype = W.DWORD
    api.BluetoothFindFirstDevice.argtypes = [C.POINTER(SearchParams), C.POINTER(DeviceInfo)]
    api.BluetoothFindFirstDevice.restype = W.HANDLE
    api.BluetoothFindNextDevice.argtypes = [W.HANDLE, C.POINTER(DeviceInfo)]
    api.BluetoothFindNextDevice.restype = W.BOOL
    api.BluetoothFindDeviceClose.argtypes = [W.HANDLE]
    api.BluetoothFindDeviceClose.restype = W.BOOL
    api.BluetoothEnumerateInstalledServices.argtypes = [
        W.HANDLE, C.POINTER(DeviceInfo), C.POINTER(W.DWORD), C.POINTER(Guid)]
    api.BluetoothEnumerateInstalledServices.restype = W.DWORD
    kernel.CloseHandle.argtypes = [W.HANDLE]
    kernel.CloseHandle.restype = W.BOOL
    output = {'target': target.upper(), 'read_only': True, 'active_inquiry': False,
              'structure_sizes': {kind.__name__: C.sizeof(kind) for kind in (DeviceInfo, RadioInfo, SearchParams)},
              'radios': []}
    # Detect layout errors before handing a structure to the OS.
    assert C.sizeof(W.BOOL) == 4 and C.sizeof(W.WCHAR) == 2
    assert C.sizeof(DeviceInfo) == 560 and DeviceInfo.Address.offset == 8
    assert DeviceInfo.fAuthenticated.offset == 28 and DeviceInfo.szName.offset == 64
    find_params, radio_handle = sized(FindRadioParams), W.HANDLE()
    C.set_last_error(0)
    radio_find = api.BluetoothFindFirstRadio(C.byref(find_params), C.byref(radio_handle))
    if not radio_find:
        output['radio_enumeration_error'] = error(C.get_last_error())
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 1
    try:
        while True:
            entry = {}
            try:
                radio = sized(RadioInfo)
                result = api.BluetoothGetRadioInfo(radio_handle, C.byref(radio))
                entry['radio_info_result'] = error(result)
                if not result:
                    entry.update(radio_name=radio.szName, radio_address=f'{radio.address.ullLong:012X}')
                queried = sized(DeviceInfo)
                queried.Address.ullLong = target_number
                result = api.BluetoothGetDeviceInfo(radio_handle, C.byref(queried))
                entry['target_query_result'] = error(result)
                if not result:
                    entry['target_device'] = device_text(queried)
                    count = W.DWORD(0)
                    first_result = api.BluetoothEnumerateInstalledServices(
                        radio_handle, C.byref(queried), C.byref(count), None)
                    entry['installed_services_count_result'] = error(first_result)
                    entry['installed_services_count'] = count.value
                    if first_result in (0, 234) and count.value <= 256:
                        services = (Guid * max(1, count.value))()
                        if count.value:
                            service_result = api.BluetoothEnumerateInstalledServices(
                                radio_handle, C.byref(queried), C.byref(count), services)
                        else:
                            service_result = first_result
                        entry['installed_services_result'] = error(service_result)
                        if not service_result:
                            guids = [str(uuid.UUID(bytes_le=bytes(value.bytes))) for value in services[:count.value]]
                            entry['enabled_service_guids'] = guids
                            entry['spp_enabled'] = '00001101-0000-1000-8000-00805f9b34fb' in guids
                search = sized(SearchParams)
                search.fReturnAuthenticated = search.fReturnRemembered = 1
                search.fReturnUnknown = search.fReturnConnected = 1
                search.fIssueInquiry = 0
                search.hRadio = radio_handle
                found = sized(DeviceInfo)
                C.set_last_error(0)
                device_find = api.BluetoothFindFirstDevice(C.byref(search), C.byref(found))
                entry['cached_classic_device_count'] = 0
                entry['cached_target_matches'] = []
                if not device_find:
                    entry['cached_enumeration_error'] = error(C.get_last_error())
                else:
                    try:
                        while True:
                            entry['cached_classic_device_count'] += 1
                            if found.Address.ullLong == target_number:
                                entry['cached_target_matches'].append(device_text(found))
                            found = sized(DeviceInfo)
                            C.set_last_error(0)
                            if not api.BluetoothFindNextDevice(device_find, C.byref(found)):
                                entry['cached_enumeration_end'] = error(C.get_last_error())
                                break
                    finally:
                        api.BluetoothFindDeviceClose(device_find)
            finally:
                kernel.CloseHandle(radio_handle)
                output['radios'].append(entry)
            radio_handle = W.HANDLE()
            C.set_last_error(0)
            if not api.BluetoothFindNextRadio(radio_find, C.byref(radio_handle)):
                output['radio_enumeration_end'] = error(C.get_last_error())
                break
    finally:
        api.BluetoothFindRadioClose(radio_find)
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
