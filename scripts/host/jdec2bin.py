#!/usr/bin/env python3
"""
Script for extracting the flash data in a JDEC file to a raw binary file
that can be written to flash without any additional conversion.

Reads from stdin and writes to stdout by default. On Windows, use the -o
argument, as redirecting binary output can cause corruption related to
newlines (LF --> CRLF).

The script will extract all sections, one after another, and ignores offsets
(but warns if the offset isn't at the end of the previous section).


Copyright (c) 2016-2020, D3 Engineering.  All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; version 2 of the License.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
more details.

"""

import errno
import sys
import itertools
import argparse

parser = argparse.ArgumentParser(description="Options when converting JED files to BIN files")
parser.add_argument("--size", "-s", type=int, help="Number of bytes to write")
parser.add_argument("--input", "-i", type=argparse.FileType("rb"), help="Input file")
parser.add_argument("--output", "-o", type=argparse.FileType("wb"), help="Output file")
args = parser.parse_args()

infile = args.input
outfile = args.output

def by(amnt, it):
	while True:
		accum = ""
		for i in range(amnt):
			try:
				accum += next(it)
			except StopIteration:
				if i == 0:
					raise
				else:
					break
		yield accum

jdec = infile.read()
jdec = jdec.partition(b"\x02")[2]
jdec = jdec.partition(b"\x03")[0]

if not jdec:
	print("No JDEC section found", file=sys.stderr)
	sys.exit(1)

jdec = jdec.decode("ascii")

fields = jdec.split("*")
bitsWritten = 0

for field in fields:
	field = field.strip()

	if not field or field[0] != "L":
		continue

	addr, _, data = field[1:].partition("\n")
	addr = int(addr)
	if addr != bitsWritten:
		print("address mismatch: expected to be at %d bits (%d bytes), but actually at %d bits (%d bytes)" % (addr, addr/8, bitsWritten, bitsWritten/8), file=sys.stderr)
	data = filter(lambda ch: not ch.isspace(), data)
	for octetStr in by(8, iter(data)):
		if args.size and bitsWritten/8 == args.size:
			break
		octet = int(octetStr, 2).to_bytes(1, "big")
		outfile.write(octet)
		bitsWritten += 8

infile.close()
outfile.close()
