#!/usr/bin/env python3
"""AirDAP Host 目标绑定工具（pyusb）。

用法：
    airdap_bind.py list
    airdap_bind.py [--serial UID] show
    airdap_bind.py [--serial UID] bind <uid>
    airdap_bind.py [--serial UID] forget
    airdap_bind.py [--serial UID] peers
    airdap_bind.py [--serial UID] exit-wireless

USB 序列号就是 Host 本机的 16 位十六进制 UID，用于在多个设备间选择目标。
"""
import argparse
import re
import sys

try:
    import usb.core
    import usb.util
except ImportError:  # pragma: no cover
    sys.exit("需要 pyusb：pip install pyusb")

VID = 0x1209
PID = 0x572D

REQ_MODE = 0x01
REQ_BIND = 0x03
REQ_PEERS = 0x04

MODE_LOCAL = 1

BIND_STATUS_SIZE = 32
BIND_UID_SIZE = 8
PEER_PAGE_SIZE = 32
PEER_PAGE_ITEMS = 2
PEER_PAGES = 4
PEER_ENTRY_SIZE = 12

FLAG_BOUND = 0x01
FLAG_PAIRED = 0x02
FLAG_SCAN_ONLY = 0x04

VENDOR_IN = 0xC0
VENDOR_OUT = 0x40


def find_devices():
    return list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID))


def device_label(index, dev):
    try:
        serial = dev.serial_number or "?"
    except usb.core.USBError:
        serial = "?"
    try:
        product = dev.product or "AirDAP"
    except usb.core.USBError:
        product = "AirDAP"
    return "%d: %s (%s)" % (index, serial, product)


def normalize_uid(text):
    digits = re.sub(r"[^0-9a-fA-F]", "", text)
    if len(digits) != 2 * BIND_UID_SIZE:
        raise ValueError("UID 需要 16 个十六进制字符（8 字节）")
    return digits.upper()


def open_device(args):
    devices = find_devices()
    if not devices:
        sys.exit("未找到 AirDAP（VID:PID = 1209:572D）")
    if args.serial:
        wanted = normalize_uid(args.serial)
        for index, dev in enumerate(devices):
            try:
                serial = (dev.serial_number or "").upper()
            except usb.core.USBError:
                continue
            if serial == wanted:
                return dev
        print("可用设备：")
        for index, dev in enumerate(devices):
            print("  " + device_label(index, dev))
        sys.exit("找不到序列号为 %s 的设备" % wanted)
    if args.index is not None:
        if args.index >= len(devices):
            sys.exit("设备序号超出范围（共 %d 台）" % len(devices))
        return devices[args.index]
    if len(devices) > 1:
        print("检测到多台 AirDAP，请用 --serial 或 --index 选择：")
        for index, dev in enumerate(devices):
            print("  " + device_label(index, dev))
        sys.exit(2)
    return devices[0]


def ctrl_in(dev, request, index, length, timeout):
    return bytes(dev.ctrl_transfer(VENDOR_IN, request, 0, index, length, timeout=timeout))


def ctrl_out(dev, request, index, data, timeout):
    return dev.ctrl_transfer(VENDOR_OUT, request, 0, index, data, timeout=timeout)


def format_uid(raw):
    return "".join("%02X" % value for value in raw)


def cmd_list(_args):
    devices = find_devices()
    if not devices:
        print("未找到 AirDAP")
        return 0
    for index, dev in enumerate(devices):
        print(device_label(index, dev))
    return 0


def cmd_show(args):
    dev = open_device(args)
    reply = ctrl_in(dev, REQ_BIND, 0, BIND_STATUS_SIZE, args.timeout)
    if len(reply) != BIND_STATUS_SIZE:
        sys.exit("状态响应长度错误：%d" % len(reply))
    flags = reply[0]
    bound_uid = format_uid(reply[8:16])
    own_uid = format_uid(reply[16:24])
    print("本机 UID : %s" % own_uid)
    print("绑定 UID : %s" % (bound_uid if flags & FLAG_BOUND else "(未绑定)"))
    print("状态     : bound=%d paired=%d scan-only=%d"
          % (1 if flags & FLAG_BOUND else 0,
             1 if flags & FLAG_PAIRED else 0,
             1 if flags & FLAG_SCAN_ONLY else 0))
    return 0


def cmd_bind(args):
    uid = normalize_uid(args.uid)
    dev = open_device(args)
    ctrl_out(dev, REQ_BIND, 0, bytes.fromhex(uid), args.timeout)
    print("已提交绑定：%s（保存完成前请勿断电）" % uid)
    return 0


def cmd_forget(args):
    dev = open_device(args)
    ctrl_out(dev, REQ_BIND, 0, None, args.timeout)
    print("已清除绑定，Host 进入 scan-only；用 peers 查看目标，或 bind 指定 UID。")
    return 0


def cmd_peers(args):
    dev = open_device(args)
    entries = []
    total = 0
    for page in range(PEER_PAGES):
        reply = ctrl_in(dev, REQ_PEERS, page, PEER_PAGE_SIZE, args.timeout)
        if len(reply) != PEER_PAGE_SIZE:
            sys.exit("发现列表响应长度错误：%d" % len(reply))
        total = reply[0]
        count = reply[2]
        if count > PEER_PAGE_ITEMS:
            count = PEER_PAGE_ITEMS
        for item in range(count):
            offset = 4 + item * PEER_ENTRY_SIZE
            uid = format_uid(reply[offset:offset + 8])
            last_seen = int.from_bytes(reply[offset + 8:offset + 12], "little")
            entries.append((uid, last_seen))
        if len(entries) >= total:
            break
    print("共 %d 个目标：" % total)
    for index, (uid, last_seen) in enumerate(entries):
        print("  %d: %s  last_seen=%d ms" % (index, uid, last_seen))
    return 0


def cmd_exit_wireless(args):
    dev = open_device(args)
    try:
        dev.ctrl_transfer(VENDOR_OUT, REQ_MODE, MODE_LOCAL, 0, None, timeout=args.timeout)
    except usb.core.USBError as error:
        # 角色切换会重新枚举，设备端可能先断开；我们无法区分成功和失败。
        print("已发送退出无线请求（设备可能已重新枚举）：%s" % error)
        return 0
    print("已切换到 Local；断电后仍按绑定恢复 Host。")
    return 0


def main():
    parser = argparse.ArgumentParser(description="AirDAP Host 目标绑定工具")
    parser.add_argument("--serial", help="Host 的 USB 序列号（16 位十六进制本机 UID）")
    parser.add_argument("--index", type=int, help="按 list 的顺序选择设备")
    parser.add_argument("--timeout", type=int, default=2000, help="USB 控制传输超时（ms）")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="列出所有 AirDAP Host").set_defaults(func=cmd_list)
    subparsers.add_parser("show", help="显示绑定状态").set_defaults(func=cmd_show)
    bind = subparsers.add_parser("bind", help="绑定指定 Target UID")
    bind.add_argument("uid", help="16 位十六进制 UID")
    bind.set_defaults(func=cmd_bind)
    subparsers.add_parser("forget", help="清除绑定并进入 scan-only").set_defaults(func=cmd_forget)
    subparsers.add_parser("peers", help="读取 scan-only 发现列表").set_defaults(func=cmd_peers)
    subparsers.add_parser("exit-wireless", help="切换到 Local（本次运行）").set_defaults(
        func=cmd_exit_wireless)

    args = parser.parse_args()
    try:
        return args.func(args)
    except usb.core.USBError as error:
        sys.exit("USB 错误：%s（Windows 请确认已安装 WinUSB/libusb 驱动）" % error)
    except ValueError as error:
        sys.exit("参数错误：%s" % error)


if __name__ == "__main__":
    sys.exit(main())
