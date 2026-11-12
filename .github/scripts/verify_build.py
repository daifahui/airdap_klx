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


def bank(name, start, end, limit=None):
    out = PROJECTS / name / 'obj'
    sym = symbols(out / (name + '.elf'))
    assert sym['__stack_size'] == 768, name
    assert sym['__stack_bottom'] == 0x20002D00, name
    assert sym['_eusrstack'] == 0x20003000, name
    assert sym['__ram_end'] <= sym['__stack_bottom'], name
    assert sym['MODE_mailbox'] == 0x20000090, name
    assert sym['__role_mailbox_end'] == 0x200000D0, name
    assert sym['_start'] == start, name
    assert start < sym['__flash_end'] <= (limit if limit is not None else end), name
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
        'FullRole': set(),
        'Dongle': {'APP_MODE=2', 'BSP_USB_DONGLE=1'},
    }
    for name, expected in expected_defines.items():
        actual = defines(name)
        assert actual == expected, (name, actual)
    for name, start, end, limit in (('Local', 0x2000, 0x10000, None),
                                    ('Host', 0x10000, 0x20000, 0x1E000),
                                    ('Target', 0x20000, 0x3C000, None)):
        report['banks'][name] = bank(name, start, end, limit)

    dfu_sym = symbols(PROJECTS / 'DFU/obj/DFU.elf')
    dfu_image = (PROJECTS / 'DFU/obj/DFU.bin').read_bytes()
    assert len(dfu_image) == 8192
    assert dfu_sym['_start'] == 0
    assert 0 < dfu_sym['dfu_enter'] < dfu_sym['__ram_load']
    assert dfu_sym['__dfu_payload_end'] <= dfu_sym['__dfu_pointer'] == 0x1FFC
    assert dfu_image[0x14:0x18] == bytes.fromhex('a9bdf9f3')
    assert int.from_bytes(dfu_image[-4:], 'little') == dfu_sym['dfu_enter']
    assert dfu_sym['__bss_end'] <= dfu_sym['__stack_bottom'] == 0x20002C00
    with (PROJECTS / 'DFU/obj/DFU.elf').open('rb') as stream:
        elf = ELFFile(stream)
        for func in elf.get_section_by_name('.symtab').iter_symbols():
            if func['st_info']['type'] == 'STT_FUNC' and func['st_size'] and func.name != 'dfu_enter':
                assert 0x20000000 <= func['st_value'] < dfu_sym['__ram_initialized_end'], func.name
    report['dfu'] = {'reserved': len(dfu_image), 'payload_end': dfu_sym['__dfu_payload_end'],
                     'padding_before_pointer': 0x1FFC - dfu_sym['__dfu_payload_end'],
                     'entry': dfu_sym['dfu_enter'], 'stack': 1024,
                     'update_range': [0, 0x3C000],
                     'ram_free': dfu_sym['__stack_bottom'] - dfu_sym['__bss_end']}

    # Dongle is one linked application since the HOST bank was merged into it, so
    # its own ELF carries the HOST-side RAM, stack, mailbox and Flash bounds.
    dongle_sym = symbols(PROJECTS / 'Dongle/obj/Dongle.elf')
    assert dongle_sym['__stack_size'] == 768
    assert dongle_sym['__stack_bottom'] == 0x20002D00
    assert dongle_sym['_eusrstack'] == 0x20003000
    assert dongle_sym['__ram_end'] <= dongle_sym['__stack_bottom']
    assert dongle_sym['MODE_mailbox'] == 0x20000090
    assert dongle_sym['__role_mailbox_end'] == 0x200000D0
    assert 0x10000 < dongle_sym['__flash_end'] <= 0x1E000
    dongle_image = (PROJECTS / 'Dongle/obj/Dongle.bin').read_bytes()
    assert len(dongle_image) == dongle_sym['__flash_end']
    assert dongle_image[:0x2000] == dfu_image
    assert not any(v != 0xFF for v in dongle_image[0x2000:0x10000])
    assert dongle_image[0x10014:0x10018] == bytes.fromhex('a9bdf9f3')
    report['banks']['Dongle'] = {'ram_including_stack': dongle_sym['__ram_end'] - 0x20000000 + 768,
                                 'ram_free': dongle_sym['__stack_bottom'] - dongle_sym['__ram_end'],
                                 'stack': 768, 'bin_bytes': len(dongle_image),
                                 'sha256': hashlib.sha256(dongle_image).hexdigest()}

    for name in ('FullRole', 'Dongle'):
        out = PROJECTS / name / 'obj'
        payload = (out / (name + '.bin')).read_bytes()
        assert 0x2000 < len(payload) <= 0x3C000
        if name == 'FullRole':
            expected = bytearray(b'\xff' * len(payload))
            banks = [('DFU', 0), ('Local', 0x2000), ('Host', 0x10000), ('Target', 0x20000)]
            for bank_name, offset in banks:
                chunk = (PROJECTS / bank_name / 'obj' / (bank_name + '.bin')).read_bytes()
                # 0x1E000..0x20000 holds the Host peer-binding sectors.
                assert not chunk or offset + len(chunk) <= 0x1E000 or offset >= 0x20000, bank_name
                assert not any(v != 0xFF for v in expected[offset:offset + len(chunk)])
                expected[offset:offset + len(chunk)] = chunk
            assert not any(v != 0xFF for v in payload[0x1E000:0x20000])
            assert payload == expected
        else:
            # Dongle contains the same boot DFU and only a HOST application.
            assert payload[:0x2000] == dfu_image
            assert not any(v != 0xFF for v in payload[0x2000:0x10000])
            assert not any(v != 0xFF for v in payload[0x1E000:0x20000])
        package = (out / (name + '.dfu')).read_bytes()
        # DFU runs from SRAM and can update itself at address zero.
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
    out.mkdir(parents=True, exist_ok=True)
    (out / 'verified-products.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: FullRole/Dongle HEX/BIN/DFU, role profiles, RAM/stack bounds, DFU and bank placement')


if __name__ == '__main__':
    main()
