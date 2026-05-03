#!/usr/bin/env python3
"""
Byte-compare reference for the FrontierSat AX100 uplink framer.

Imports pycsp / pycsplink from the CalgaryToSpace CTS_SAT_1_COMMUNICATIONS
repo directly, so the hex it prints is the authoritative ground truth.
Compare this against `uplink_test --print-frame` output with the same
parameters.

Usage
-----
    # Default path assumes ~/src/CTS_SAT_1_COMMUNICATIONS/Working_Python_Files
    ./scripts/verify_uplink_frame.py

    # Or point at a different checkout:
    CTS_PATH=/path/to/Working_Python_Files ./scripts/verify_uplink_frame.py

Reed-Solomon is on by default (matches pycsplink uplink and our
uplink_test default). Install with:

    pip install reed-solomon-ccsds crc xtea

Pass --no-rs if you want to compare against a frame built without RS.
CRC and XTEA are still stubbed because we don't exercise them here.
"""
import argparse
import os
import sys
import types

for dep in ("xtea", "crc"):
    if dep not in sys.modules:
        sys.modules[dep] = types.ModuleType(dep)

ap = argparse.ArgumentParser()
ap.add_argument("--payload", default="ping",
                help="ASCII payload (default: ping)")
ap.add_argument("--no-rs", action="store_true",
                help="Disable Reed-Solomon (match uplink_test --no-reed-solomon)")
ap.add_argument("--no-hmac", action="store_true",
                help="Disable HMAC (match uplink_test --no-hmac)")
args = ap.parse_args()

CTS_PATH = os.environ.get(
    "CTS_PATH",
    os.path.expanduser("~/src/CTS_SAT_1_COMMUNICATIONS/Working_Python_Files"),
)
if not os.path.isdir(CTS_PATH):
    sys.stderr.write(
        f"CTS_PATH does not exist: {CTS_PATH}\n"
        "  Set CTS_PATH=/abs/path/to/Working_Python_Files\n"
    )
    sys.exit(1)
sys.path.insert(0, CTS_PATH)

import pycsp as csp            # noqa: E402
import pycsplink as csplink    # noqa: E402

# Test vector — must match uplink_test CLI invocation exactly.
HMAC_KEY_HEX = "00112233445566778899AABBCCDDEEFF"
CSP_ARGS = dict(src=1, dst=2, dport=3, sport=4, prio="norm")

pkt = csp.Packet(**CSP_ARGS, crc_endian=None)
pkt.payload = args.payload.encode("ascii")

uplink = csplink.AX100(
    hmac_key=None if args.no_hmac else bytes.fromhex(HMAC_KEY_HEX),
    crc=False,
    reed_solomon=not args.no_rs,
    randomize=True,
    len_field=True,
    syncword=True,
    prefill=32,
    tailfill=1,
)

frame = uplink.encode(pkt)
print(frame.hex().upper())
