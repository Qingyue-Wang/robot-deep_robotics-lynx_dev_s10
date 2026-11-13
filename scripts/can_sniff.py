#!/usr/bin/env python3
"""Sniff piper CAN commands on can_piper for N seconds: prints unique id/data pairs."""
import sys
import time
from collections import Counter

import can

IFACE = "can_piper"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
WANT = {0x150, 0x151, 0x155, 0x156, 0x157, 0x470, 0x471, 0x475}


def main() -> int:
    bus = can.interface.Bus(channel=IFACE, interface="socketcan")
    seen = Counter()
    t0 = time.time()
    while time.time() - t0 < DUR:
        msg = bus.recv(timeout=0.2)
        if msg is None or msg.arbitration_id not in WANT:
            continue
        seen[(msg.arbitration_id, msg.data.hex(" "))] += 1
    bus.shutdown()
    for (cid, data), n in sorted(seen.items(), key=lambda kv: -kv[1]):
        print(f"0x{cid:03X}  {data}   x{n}  ({n / DUR:.1f}/s)")
    if not seen:
        print(f"no frames among {[hex(i) for i in sorted(WANT)]} in {DUR}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
