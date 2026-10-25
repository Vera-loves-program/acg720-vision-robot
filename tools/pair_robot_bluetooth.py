"""Pair only the confirmed Classic AAA04250224011F and enable its SPP service.

This tool changes only that target's authentication/SPP state. It never removes
devices, edits the registry, resets Bluetooth, opens COM ports, or sends robot
bytes. One invocation makes at most one authentication request with one PIN.
The factory PIN is not assumed; supply the actual PIN explicitly with --pin.

API references:
https://learn.microsoft.com/windows/win32/api/bluetoothapis/nf-bluetoothapis-bluetoothauthenticatedevice
https://learn.microsoft.com/windows/win32/api/bluetoothapis/nf-bluetoothapis-bluetoothsetservicestate
"""
from __future__ import annotations

import argparse
import ctypes as C
from ctypes import wintypes as W
import json
import os
import sys
import uuid

from inspect_bluetooth_classic import DeviceInfo, FindRadioParams, Guid, device_text, error, sized

TARGET_ADDRESS = 0x04250224011F
TARGET_NAME = 'AAA'
SPP_UUID = uuid.UUID('00001101-0000-1000-8000-00805f9b34fb')
BLUETOOTH_SERVICE_ENABLE = 1


def pin_value(value):
    # HC-04 accepts at most16 characters; printable ASCII also avoids UTF-16
    # code-unit/PIN-byte ambiguity in the legacy transparent-mode API.
    if not 1 <= len(value) <= 16 or any(not 32 <= ord(char) <= 126 for char in value):
        raise argparse.ArgumentTypeError('PIN必须为1～16个可打印ASCII字符（不会输出PIN）')
    return value


def read_target(api, radio_handle):
    info = sized(DeviceInfo)
    info.Address.ullLong = TARGET_ADDRESS
    code = api.BluetoothGetDeviceInfo(radio_handle, C.byref(info))
    state = {'query_result': error(code)}
    if not code:
        state.update(device_text(info))
    return info, state


def read_services(api, radio_handle, info):
    count = W.DWORD(0)
    code = api.BluetoothEnumerateInstalledServices(radio_handle, C.byref(info), C.byref(count), None)
    if code not in (0, 234) or count.value > 256:
        return {'result': error(code), 'spp_enabled': None, 'count': count.value}
    values = (Guid * max(1, count.value))()
    if count.value:
        code = api.BluetoothEnumerateInstalledServices(radio_handle, C.byref(info), C.byref(count), values)
    result = {'result': error(code), 'count': count.value, 'spp_enabled': None}
    if not code:
        guids = [str(uuid.UUID(bytes_le=bytes(value.bytes))) for value in values[:count.value]]
        result.update(enabled_service_guids=guids, spp_enabled=str(SPP_UUID) in guids)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pin', type=pin_value, help='实际配对PIN；不输出，不自动尝试其他PIN')
    parser.add_argument('--enable-spp', action='store_true', help='仅为已认证AAA启用SPP，不进行配对')
    args = parser.parse_args(argv)
    if not args.enable_spp and args.pin is None:
        parser.error('单次配对必须明确提供--pin；已有认证时可用--enable-spp')
    if args.enable_spp and args.pin is not None:
        parser.error('--enable-spp模式不会使用PIN；请不要同时指定--pin')
    if os.name != 'nt':
        parser.error('Windows only')
    assert C.sizeof(DeviceInfo) == 560 and DeviceInfo.Address.offset == 8
    assert C.sizeof(W.BOOL) == 4 and C.sizeof(W.WCHAR) == 2
    api = C.WinDLL('bthprops.cpl', use_last_error=True)
    kernel = C.WinDLL('kernel32', use_last_error=True)
    api.BluetoothFindFirstRadio.argtypes = [C.POINTER(FindRadioParams), C.POINTER(W.HANDLE)]
    api.BluetoothFindFirstRadio.restype = W.HANDLE
    api.BluetoothFindNextRadio.argtypes = [W.HANDLE, C.POINTER(W.HANDLE)]
    api.BluetoothFindNextRadio.restype = W.BOOL
    api.BluetoothFindRadioClose.argtypes = [W.HANDLE]
    api.BluetoothFindRadioClose.restype = W.BOOL
    api.BluetoothGetDeviceInfo.argtypes = [W.HANDLE, C.POINTER(DeviceInfo)]
    api.BluetoothGetDeviceInfo.restype = W.DWORD
    api.BluetoothAuthenticateDevice.argtypes = [W.HWND, W.HANDLE, C.POINTER(DeviceInfo), W.LPWSTR, W.ULONG]
    api.BluetoothAuthenticateDevice.restype = W.DWORD
    api.BluetoothEnumerateInstalledServices.argtypes = [
        W.HANDLE, C.POINTER(DeviceInfo), C.POINTER(W.DWORD), C.POINTER(Guid)]
    api.BluetoothEnumerateInstalledServices.restype = W.DWORD
    api.BluetoothSetServiceState.argtypes = [W.HANDLE, C.POINTER(DeviceInfo), C.POINTER(Guid), W.DWORD]
    api.BluetoothSetServiceState.restype = W.DWORD
    kernel.CloseHandle.argtypes = [W.HANDLE]
    kernel.CloseHandle.restype = W.BOOL
    report = {'target_address': f'{TARGET_ADDRESS:012X}', 'required_name': TARGET_NAME,
              'mode': 'enable_spp_only' if args.enable_spp else 'single_pin_pair_then_enable_spp',
              'authentication_attempts': 0, 'radio_queries': [], 'success': False}
    radio_handle = W.HANDLE()
    params = sized(FindRadioParams)
    C.set_last_error(0)
    radio_find = api.BluetoothFindFirstRadio(C.byref(params), C.byref(radio_handle))
    if not radio_find:
        report['radio_enumeration_result'] = error(C.get_last_error())
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    try:
        while True:
            selected = False
            try:
                info, before = read_target(api, radio_handle)
                report['radio_queries'].append(before)
                if before['query_result']['code'] == 0:
                    if info.Address.ullLong != TARGET_ADDRESS or info.szName != TARGET_NAME:
                        report['error'] = '目标名称或地址不匹配；未进行认证或服务修改'
                        break
                    selected = True
                    report['before'] = before
                    if not info.fAuthenticated:
                        if args.enable_spp:
                            report['error'] = 'AAA尚未认证；--enable-spp不会发起配对'
                            break
                        pin_buffer = C.create_unicode_buffer(args.pin)
                        report['authentication_attempts'] = 1
                        try:
                            code = api.BluetoothAuthenticateDevice(
                                None, radio_handle, C.byref(info), pin_buffer, len(args.pin))
                        finally:
                            # Never print the PIN; clear its mutable API buffer.
                            C.memset(C.addressof(pin_buffer), 0, C.sizeof(pin_buffer))
                        report['authentication_result'] = error(code)
                        info, after_auth = read_target(api, radio_handle)
                        report['after_authentication'] = after_auth
                        if code not in (0, 259) or after_auth['query_result']['code'] or not info.fAuthenticated:
                            report['error'] = '单次认证未确认成功；已停止，不重试其他PIN，也不启用SPP'
                            break
                    else:
                        report['authentication_skipped'] = 'Already authenticated; no PIN attempt'
                    # Revalidate identity immediately before the only service mutation.
                    if info.Address.ullLong != TARGET_ADDRESS or info.szName != TARGET_NAME:
                        report['error'] = '认证后目标身份不匹配；未修改SPP'
                        break
                    report['services_before'] = read_services(api, radio_handle, info)
                    if report['services_before']['spp_enabled']:
                        report['spp_enable_skipped'] = 'SPP already enabled'
                    else:
                        guid = Guid()
                        guid.bytes[:] = SPP_UUID.bytes_le
                        code = api.BluetoothSetServiceState(
                            radio_handle, C.byref(info), C.byref(guid), BLUETOOTH_SERVICE_ENABLE)
                        report['spp_enable_result'] = error(code)
                    info, report['after'] = read_target(api, radio_handle)
                    report['services_after'] = read_services(api, radio_handle, info)
                    report['success'] = bool(
                        not report['after']['query_result']['code'] and info.fAuthenticated and
                        report['services_after']['spp_enabled'])
                    if not report['success']:
                        report['error'] = '未确认AAA认证与SPP均有效；请依据返回Win32码检查，不自动重试'
                    break
            finally:
                kernel.CloseHandle(radio_handle)
            if selected:
                break
            radio_handle = W.HANDLE()
            C.set_last_error(0)
            if not api.BluetoothFindNextRadio(radio_find, C.byref(radio_handle)):
                report['radio_enumeration_end'] = error(C.get_last_error())
                report['error'] = '所有可见radio均未找到已确认的Classic AAA；未修改设备'
                break
    except Exception as exc:
        # Exception text from ctypes can contain argument representations; do
        # not print it because the authentication argument includes the PIN.
        report['error'] = f'检查或调用异常（{type(exc).__name__}）；未自动重试'
        report['last_win32_result'] = error(C.get_last_error())
    finally:
        api.BluetoothFindRadioClose(radio_find)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
