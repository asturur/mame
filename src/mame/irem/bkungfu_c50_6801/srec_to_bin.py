#!/usr/bin/env python3
# license:BSD-3-Clause

"""Convert CRASM Motorola S-record output to one fixed-size ROM image."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
	parser = argparse.ArgumentParser()
	parser.add_argument("input", type=Path)
	parser.add_argument("output", type=Path)
	parser.add_argument("--base", type=lambda value: int(value, 0), default=0xF000)
	parser.add_argument("--size", type=lambda value: int(value, 0), default=0x1000)
	args = parser.parse_args()

	image = bytearray([0xFF] * args.size)
	address_bytes = {"1": 2, "2": 3, "3": 4}
	for line_number, text in enumerate(args.input.read_text().splitlines(), 1):
		line = text.strip()
		if not line or not line.startswith("S") or line[1] not in address_bytes:
			continue
		record = bytes.fromhex(line[2:])
		if len(record) != record[0] + 1 or (sum(record) & 0xFF) != 0xFF:
			raise ValueError(f"bad S-record checksum/length at line {line_number}")
		width = address_bytes[line[1]]
		address = int.from_bytes(record[1:1 + width], "big")
		payload = record[1 + width:-1]
		start = address - args.base
		end = start + len(payload)
		if start < 0 or end > len(image):
			raise ValueError(f"record outside output ROM at line {line_number}")
		image[start:end] = payload

	args.output.write_bytes(image)


if __name__ == "__main__":
	main()
