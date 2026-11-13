#!/usr/bin/env python3
"""Send piper CAN frames: 0x150 MotionCtrl_1(emergency_stop, track_ctrl, grag_teach_ctrl)
and 0x151 MotionCtrl_2(ctrl_mode, move_mode, spd, is_mit_mode).
Usage: python3 /tmp/can_motion.py 150 02 00 00
       python3 /tmp/can_motion.py 151 01 01 50 00
Sends the frame 3 times, 50 ms apart."""
import sys
import time

import can

IFACE = "can_piper"


def main() -> int:
    cid = int(sys.argv[1], 16)
    payload = [int(x, 16) for x in sys.argv[2:]]
    if len(payload) > 8:
        print("payload too long")
        return 2
    data = bytes(payload + [0x00] * (8 - len(payload)))
    bus = can.interface.Bus(channel=IFACE, bustype="socketcan")
    for i in range(3):
        msg = can.Message(arbitration_id=cid, data=data, is_extended_id=False)
        bus.send(msg)
        print(f"sent {i + 1}: id=0x{cid:03X} data={data.hex(' ')}")
        time.sleep(0.05)
    bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
