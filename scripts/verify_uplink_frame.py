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

The CRC / XTEA / Reed-Solomon modules referenced by pycsp/pycsplink at
import time are stubbed below because we don't exercise them in this
test. If you ever enable CRC, RS, or XTEA, install the real modules
(`pip install crc xtea reed-solomon-ccsds` or equivalent) and drop the
stub block.
"""
import os
import sys
import types

for dep in ("xtea", "crc", "reed_solomon_ccsds"):
    if dep not in sys.modules:
        sys.modules[dep] = types.ModuleType(dep)

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
PAYLOAD = b"ping"
CSP_ARGS = dict(src=1, dst=2, dport=3, sport=4, prio="norm")

pkt = csp.Packet(**CSP_ARGS, crc_endian=None)
pkt.payload = PAYLOAD

uplink = csplink.AX100(
    hmac_key=bytes.fromhex(HMAC_KEY_HEX),
    crc=False,
    reed_solomon=False,
    randomize=True,
    len_field=True,
    syncword=True,
    prefill=32,
    tailfill=1,
)

frame = uplink.encode(pkt)
print(frame.hex().upper())
