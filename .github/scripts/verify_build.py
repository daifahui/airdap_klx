#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check native MRS products, project profiles and linked memory bounds."""
from pathlib import Path
import hashlib
import json
import re
import struct
import xml.etree.ElementTree as ET
import zlib

from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[2]
PROJECTS = ROOT / 'firmware/projects'


def symbols(path):
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        return {s.name: int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols()}


def bank(name):
    out = PROJECTS / name / 'obj'
    sym = symbols(out / (name + '.elf'))
    assert sym['__stack_size'] == 768, name
    assert sym['__stack_bottom'] == 0x20002D00, name
    assert sym['_eusrstack'] == 0x20003000, name
    assert sym['__ram_end'] <= sym['__stack_bottom'], name
    assert sym['MODE_mailbox'] == 0x20000090, name
    assert sym['__role_mailbox_end'] == 0x200000D0, name
    image = (out / (name + '.bin')).read_bytes()
    assert image[0x14:0x18] == bytes.fromhex('a9bdf9f3'), name
    return {'ram_including_stack': sym['__ram_end'] - 0x20000000 + 768,
            'ram_free': sym['__stack_bottom'] - sym['__ram_end'],
            'stack': 768, 'bin_bytes': len(image),
            'sha256': hashlib.sha256(image).hexdigest()}


def defines(name):
    values = set()
    for option in ET.parse(PROJECTS / name / '.cproject').iter('option'):
        if option.get('superClass', '').endswith(('.c.compiler.defs', '.assembler.defs')):
            values.update(v.get('value') for v in option)
    return values


def main():
    report = {'products': {}, 'banks': {}}
    config = (ROOT / 'firmware/src/app_config.h').read_text()
    ids = [int(re.search(r'^#define\s+' + name + r'\s+(0x[0-9a-fA-F]+)', config, re.M)[1], 16)
           for name in ('USB_VID', 'USB_PID')]
    expected_defines = {
        'Local': {'APP_MODE=0'},
        'Host': {'APP_MODE=2'},
        'Target': {'APP_MODE=3'},
        'DFU': set(),
        'Router': set(),
        'FullRole': set(),
        'Dongle': {'APP_MODE=2', 'BSP_USB_DONGLE=1'},
    }
    for name, expected in expected_defines.items():
        actual = defines(name)
        assert actual == expected, (name, actual)
    for name in ('Local', 'Host', 'Target'):
        report['banks'][name] = bank(name)

    dfu_sym = symbols(PROJECTS / 'DFU/obj/DFU.elf')
    dfu_image = (PROJECTS / 'DFU/obj/DFU.bin').read_bytes()
    assert len(dfu_image) == 8192
    assert int.from_bytes(dfu_image[-4:], 'little') == dfu_sym['dfu_enter']
    assert dfu_sym['__bss_end'] <= dfu_sym['__stack_bottom'] == 0x20002C00
    router_image = (PROJECTS / 'Router/obj/Router.bin').read_bytes()
    assert 0 < len(router_image) <= 0x1000

    # Dongle is one linked application since the HOST bank was merged into it, so
    # its own ELF carries the HOST-side RAM, stack, mailbox and Flash bounds.
    dongle_sym = symbols(PROJECTS / 'Dongle/obj/Dongle.elf')
    assert dongle_sym['__stack_size'] == 768
    assert dongle_sym['__stack_bottom'] == 0x20002D00
    assert dongle_sym['_eusrstack'] == 0x20003000
    assert dongle_sym['__ram_end'] <= dongle_sym['__stack_bottom']
    assert dongle_sym['MODE_mailbox'] == 0x20000090
    assert dongle_sym['__role_mailbox_end'] == 0x200000D0
    assert 0x10000 < dongle_sym['__flash_end'] <= 0x20000
    dongle_image = (PROJECTS / 'Dongle/obj/Dongle.bin').read_bytes()
    assert len(dongle_image) == 0x36000
    assert dongle_image[:len(router_image)] == router_image
    assert not any(v != 0xFF for v in dongle_image[len(router_image):0x1000])
    assert dongle_image[0x10014:0x10018] == bytes.fromhex('a9bdf9f3')
    assert not any(v != 0xFF for v in dongle_image[0x20000:0x34000])
    assert dongle_image[0x34000:0x36000] == dfu_image
    report['banks']['Dongle'] = {'ram_including_stack': dongle_sym['__ram_end'] - 0x20000000 + 768,
                                 'ram_free': dongle_sym['__stack_bottom'] - dongle_sym['__ram_end'],
                                 'stack': 768, 'bin_bytes': len(dongle_image),
                                 'sha256': hashlib.sha256(dongle_image).hexdigest()}

    for name in ('FullRole', 'Dongle'):
        out = PROJECTS / name / 'obj'
        payload = (out / (name + '.bin')).read_bytes()
        assert len(payload) == 0x36000
        if name == 'FullRole':
            expected = bytearray(b'\xff' * len(payload))
            banks = [('Router', 0), ('Local', 0x1000), ('Host', 0x10000),
                     ('Target', 0x20000), ('DFU', 0x34000)]
            for bank_name, offset in banks:
                chunk = (PROJECTS / bank_name / 'obj' / (bank_name + '.bin')).read_bytes()
                assert not any(v != 0xFF for v in expected[offset:offset + len(chunk)])
                expected[offset:offset + len(chunk)] = chunk
            assert payload == expected
        else:
            # The merged Dongle embeds the router and DFU images at their fixed
            # Flash addresses and leaves the unused target bank erased.
            assert payload[:len(router_image)] == router_image
            assert payload[0x34000:0x36000] == dfu_image
            assert not any(v != 0xFF for v in payload[0x20000:0x34000])
        package = (out / (name + '.dfu')).read_bytes()
        assert package[:-16] == payload and package[-8:-4] == b'UFD\x10', name
        did, pid, vid, spec = struct.unpack('<HHHH', package[-16:-8])
        assert (did, vid, pid, spec) == (0xFFFF, *ids, 0x0100), name
        assert struct.unpack('<I', package[-4:])[0] == zlib.crc32(package[:-4]) ^ 0xFFFFFFFF, name
        # Decode Intel HEX independently, including its checksums and padding.
        decoded = bytearray(b'\xff' * len(payload))
        upper = 0
        for line in (out / (name + '.hex')).read_text().splitlines():
            assert line.startswith(':')
            row = bytes.fromhex(line[1:])
            assert sum(row) & 255 == 0 and len(row) == row[0] + 5
            address = int.from_bytes(row[1:3], 'big')
            if row[3] == 2:
                upper = int.from_bytes(row[4:-1], 'big') << 4
            elif row[3] == 4:
                upper = int.from_bytes(row[4:-1], 'big') << 16
            elif row[3] == 0:
                address += upper
                assert address + row[0] <= len(payload)
                decoded[address:address + row[0]] = row[4:-1]
        assert decoded == payload, name
        report['products'][name] = {p.suffix: {'bytes': p.stat().st_size,
                                   'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                                   for p in (out / (name + s) for s in ('.hex', '.bin', '.dfu'))}
    out = ROOT / '.github/.logs'
    out.mkdir(exist_ok=True)
    (out / 'verified-products.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: FullRole/Dongle HEX/BIN/DFU, role profiles, RAM/stack bounds, DFU and bank placement')


if __name__ == '__main__':
    main()
