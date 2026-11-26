#!/usr/bin/env python3
"""Inspect an ATK-DLRK3568 RKFW image and build a separate boot FIT.

Only the original external-data FIT layout is supported. All original
subimage hashes are checked, the DTB and resource must remain identical,
and a FIT with any real signature is refused.
"""

import argparse
import hashlib
from pathlib import Path
import re
import struct


SECTOR_SIZE = 512
ALIGN_MASK = SECTOR_SIZE - 1
SUBIMAGES = ("fdt", "kernel", "resource")


def align_sector(size):
    return (size + ALIGN_MASK) & ~ALIGN_MASK


def read_at(stream, offset, size):
    stream.seek(offset)
    result = stream.read(size)
    if len(result) != size:
        raise ValueError("Truncated firmware member")
    return result


def read_rkfw(path):
    with path.open("rb") as firmware:
        outer = read_at(firmware, 0, 41)
        if outer[:4] != b"RKFW":
            raise ValueError("Expected an RKFW update image")
        archive_offset = struct.unpack_from("<I", outer, 33)[0]
        header = read_at(firmware, archive_offset, 2048)
        if header[:4] != b"RKAF" or header[8:42].strip(b" \0") != b"RK3568":
            raise ValueError("Expected an RK3568 RKAF archive")
        count = struct.unpack_from("<I", header, 0x88)[0]
        if not 1 <= count <= (len(header) - 0x8c) // 112:
            raise ValueError("Unexpected RKAF entry count")
        members = {}
        for index in range(count):
            start = 0x8c + index * 112
            name = header[start:start + 32].split(b"\0", 1)[0].decode("ascii")
            offset, sector, _, size = struct.unpack_from("<4I", header, start + 96)
            if name in members:
                raise ValueError(f"Duplicate RKAF member {name}")
            members[name] = (offset, sector, size)
        parameter_offset, _, parameter_size = members["parameter"]
        if not 8 <= parameter_size <= 4096:
            raise ValueError("Unexpected parameter member size")
        parameter = read_at(firmware, archive_offset + parameter_offset, parameter_size)
        if parameter[:4] != b"PARM" or b"TYPE: GPT\n" not in parameter:
            raise ValueError("Expected Rockchip GPT partition parameters")
        partition = re.search(rb"(0x[0-9a-fA-F]+)@(0x[0-9a-fA-F]+)\(boot\)", parameter)
        if partition is None:
            raise ValueError("Missing boot partition entry")
        partition_size, partition_sector = (int(value, 16) for value in partition.groups())
        boot_offset, boot_sector, boot_size = members["boot"]
        capacity = partition_size * SECTOR_SIZE
        if boot_sector != partition_sector or not 0 < boot_size <= capacity:
            raise ValueError("Boot partition does not match the RKAF table")
        stock = read_at(firmware, archive_offset + boot_offset, boot_size)
    return stock, capacity, boot_sector


def read_nodes(fit):
    header = struct.unpack_from(">10I", fit)
    magic, total, structure, strings, _, version, _, _, strings_size, structure_size = header
    if (magic != 0xd00dfeed or version != 17 or total > len(fit)
            or not 40 <= structure < structure + structure_size <= strings
            or strings + strings_size > total):
        raise ValueError("Invalid FIT header")
    cursor = structure
    stack = []
    nodes = {}
    while cursor < structure + structure_size:
        token = struct.unpack_from(">I", fit, cursor)[0]
        cursor += 4
        if token == 1:
            end = fit.index(0, cursor, structure + structure_size)
            stack.append(fit[cursor:end].decode("ascii"))
            cursor = (end + 4) & ~3
            node = "/".join(stack) or "/"
            if node in nodes:
                raise ValueError(f"Duplicate FIT node {node}")
            nodes[node] = {}
        elif token == 2:
            stack.pop()
        elif token == 3:
            length, relative = struct.unpack_from(">2I", fit, cursor)
            cursor += 8
            begin = strings + relative
            end = fit.index(0, begin, strings + strings_size)
            name = fit[begin:end].decode("ascii")
            node = "/".join(stack) or "/"
            if name in nodes[node] or cursor + length > structure + structure_size:
                raise ValueError(f"Invalid FIT property {name}")
            nodes[node][name] = (cursor, length)
            cursor = (cursor + length + 3) & ~3
        elif token == 4:
            continue
        elif token == 9 and not stack:
            return total, nodes
        else:
            raise ValueError("Invalid FIT structure token")
    raise ValueError("FIT structure does not terminate")


def inspect_fit(fit):
    header_size, nodes = read_nodes(fit)

    def value(node, name):
        offset, size = nodes[node][name]
        return fit[offset:offset + size]

    def u32(node, name):
        return struct.unpack(">I", value(node, name))[0]

    paths = {"/", "/images", "/configurations", "/configurations/conf",
             "/configurations/conf/signature"}
    paths.update(f"/images/{name}{tail}" for name in SUBIMAGES for tail in ("", "/hash"))
    if set(nodes) != paths:
        raise ValueError("Unexpected FIT image or configuration nodes")
    if u32("/", "totalsize") != len(fit) or value("/configurations", "default") != b"conf\0":
        raise ValueError("Unexpected FIT size or default configuration")
    config = "/configurations/conf"
    for name, expected in (
        ("fdt", b"fdt\0"), ("kernel", b"kernel\0"), ("multi", b"resource\0")
    ):
        if value(config, name) != expected:
            raise ValueError("Unexpected boot configuration")
    signature = config + "/signature"
    if set(nodes[signature]) != {"algo", "padding", "key-name-hint", "sign-images"}:
        raise ValueError("Unexpected signature metadata. A signed FIT requires factory signing.")
    parts = {}
    cursor = 0x800
    if header_size != 0x600 or any(fit[header_size:cursor]):
        raise ValueError("Unexpected FIT header alignment")
    for name in SUBIMAGES:
        node = f"/images/{name}"
        size = u32(node, "data-size")
        position = u32(node, "data-position")
        if position != cursor or not 0 < size <= len(fit) - position:
            raise ValueError(f"Invalid FIT position or size for {name}")
        if value(node, "arch") != b"arm64\0" or value(node, "compression") != b"none\0":
            raise ValueError(f"Unsupported architecture or compression for {name}")
        payload = fit[position:position + size]
        if (value(node + "/hash", "algo") != b"sha256\0"
                or value(node + "/hash", "value") != hashlib.sha256(payload).digest()):
            raise ValueError(f"SHA-256 check failed for {name}")
        cursor = align_sector(position + size)
        if any(fit[position + size:cursor]):
            raise ValueError(f"Unexpected nonzero padding after {name}")
        parts[name] = payload
    tail = len(fit) - cursor
    if tail != 2 * SECTOR_SIZE or any(fit[cursor:]):
        raise ValueError("Unexpected data at the end of boot FIT")
    return nodes, parts, tail


def replace_kernel(stock, nodes, stock_parts, tail, kernel, capacity):
    if len(kernel) < 64 or kernel[56:60] != b"ARMd":
        raise ValueError("New kernel is not an uncompressed arm64 Linux Image")
    header = bytearray(stock[:0x800])
    output = bytearray()

    def set_property(node, name, payload):
        offset, size = nodes[node][name]
        if len(payload) != size:
            raise ValueError(f"Unexpected length of {name}")
        header[offset:offset + size] = payload

    def set_u32(node, name, number):
        set_property(node, name, struct.pack(">I", number))

    for name in SUBIMAGES:
        node = f"/images/{name}"
        payload = kernel if name == "kernel" else stock_parts[name]
        set_u32(node, "data-position", len(header) + len(output))
        set_u32(node, "data-size", len(payload))
        set_property(node + "/hash", "value", hashlib.sha256(payload).digest())
        output.extend(payload)
        output.extend(bytes(align_sector(len(payload)) - len(payload)))
    output.extend(bytes(tail))
    total = len(header) + len(output)
    if total > capacity:
        raise ValueError(f"Candidate boot FIT is {total} bytes, exceeds {capacity}-byte boot partition")
    set_u32("/", "totalsize", total)
    return bytes(header + output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("firmware", type=Path, help="original RK3568 update.img")
    parser.add_argument("--kernel", type=Path, help="new uncompressed arm64 Linux Image")
    parser.add_argument("--dtb", type=Path, help="DTB expected to be identical to the stock DTB")
    parser.add_argument("--output", type=Path, help="new standalone boot.img")
    args = parser.parse_args()
    build = any((args.kernel, args.dtb, args.output))
    if build and not all((args.kernel, args.dtb, args.output)):
        parser.error("--kernel, --dtb, and --output must be provided together")
    try:
        stock, capacity, boot_sector = read_rkfw(args.firmware)
        nodes, parts, tail = inspect_fit(stock)
        reference = replace_kernel(stock, nodes, parts, tail, parts["kernel"], capacity)
        if reference != stock:
            raise ValueError("Stock round trip did not reproduce the original boot.img")
        print(f"Boot starts at sector 0x{boot_sector:x}, capacity {capacity} bytes")
        print(f"Stock boot FIT {len(stock)} bytes, SHA-256 {hashlib.sha256(stock).hexdigest()}")
        for name, payload in parts.items():
            print(f"  {name:8} {len(payload):9} bytes, SHA-256 {hashlib.sha256(payload).hexdigest()}")
        print("Stock FIT round trip is byte-identical. Signature node has no signature value.")
        if build:
            if args.dtb.read_bytes() != parts["fdt"]:
                raise ValueError("The supplied DTB does not exactly match the original")
            if args.kernel.stat().st_size > capacity:
                raise ValueError("The new kernel exceeds boot partition capacity")
            candidate = replace_kernel(stock, nodes, parts, tail, args.kernel.read_bytes(), capacity)
            inspect_fit(candidate)
            with args.output.open("xb") as output:
                output.write(candidate)
            print(f"{args.output}: {len(candidate)} bytes, SHA-256 {hashlib.sha256(candidate).hexdigest()}")
    except (KeyError, OSError, ValueError, struct.error) as exc:
        parser.exit(1, f"boot FIT: {exc}\n")


if __name__ == "__main__":
    main()
