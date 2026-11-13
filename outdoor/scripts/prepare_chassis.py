#!/usr/bin/env python3
"""Lynx S10 posture/navigation-mode setup over the plaintext UDP monitor
protocol (10.21.33.103:30004).

Usage:
  python3 scripts/prepare_chassis.py            # read BasicStatus only
  python3 scripts/prepare_chassis.py --mode 1   # usage mode = navigation
  python3 scripts/prepare_chassis.py --gait flat  # navigation gait 0x3002
  python3 scripts/prepare_chassis.py --stand    # stand (MotionParam 1 -> 17)
  python3 scripts/prepare_chassis.py --apply    # mode + gait + stand, verified
"""
import argparse
import json
import socket
import struct
import sys
import time

HOST = "10.21.33.103"
PORT = 30004
HEAD = b"\xeb\x91\xeb\x90"


def frame(payload: dict) -> bytes:
    body = json.dumps(payload).encode()
    return (
        HEAD
        + struct.pack("<HH", len(body), 0)
        + bytes([1, 0, 1])
        + b"\0" * 5
        + body
    )


def request(payload: dict, timeout=5.0):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((HOST, PORT))
        sock.settimeout(1.0)
        sock.send(frame(payload))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                raw = sock.recv(8192)
            except socket.timeout:
                continue
            if len(raw) < 16 or raw[:4] != HEAD:
                continue
            size = struct.unpack_from("<H", raw, 4)[0]
            if len(raw) < 16 + size:
                continue
            try:
                return json.loads(raw[16 : 16 + size]).get("PatrolDevice", {})
            except (ValueError, TypeError, AttributeError):
                continue
    return {}


def now():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def status():
    out = request(
        {"PatrolDevice": {
            "Type": 0x00100064,
            "Command": 5,
            "Time": now(),
            "Items": {},
        }}
    )
    return out.get("Items", {}).get("BasicStatus", {})


def send(kind, items):
    table = {
        "mode": (0x00100002, 0x00500002),
        "gait": (0x00100001, 0x00300002),
        "motion": (0x00100001, 0x00200002),
    }
    ty, cmd = table[kind]
    return request(
        {"PatrolDevice": {
            "Type": ty,
            "Command": cmd,
            "Time": now(),
            "Items": items,
        }}
    )


def wait_state(want, key, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        st = status()
        if st.get(key) == want:
            return st
        time.sleep(0.5)
    return st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", type=int)
    ap.add_argument("--gait", choices=("flat", "stairs"))
    ap.add_argument("--stand", action="store_true")
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()
    if args.apply:
        args.mode, args.gait, args.stand = 1, "flat", True
    if not (args.mode or args.gait or args.stand):
        st = status()
        print(json.dumps(st, ensure_ascii=False, indent=2) if st
              else "no BasicStatus reply")
        return 0
    if args.mode is not None:
        send("mode", {"Mode": args.mode})
        st = wait_state(args.mode, "ControlUsageMode")
        print("usage_mode ->", st.get("ControlUsageMode"))
    if args.gait:
        code = {"flat": 0x3002, "stairs": 0x3003}[args.gait]
        send("gait", {"GaitParam": code, "ActionParam": 0})
        st = wait_state(code, "Gait")
        print("gait ->", hex(st.get("Gait") or 0))
    if args.stand:
        # Standard flow: stand first, then RL control mode 17.
        send("motion", {"MotionParam": 1})
        st = wait_state(1, "MotionState", timeout_s=15.0)
        print("motion ->", st.get("MotionState"))
        if st.get("MotionState") == 1:
            send("motion", {"MotionParam": 17})
            st = wait_state(17, "MotionState", timeout_s=15.0)
            print("motion ->", st.get("MotionState"))
    st = status()
    print("final:", json.dumps({
        "MotionState": st.get("MotionState"),
        "Gait": hex(st.get("Gait") or 0),
        "ControlUsageMode": st.get("ControlUsageMode"),
    }, ensure_ascii=False))
    ok = (
        st.get("MotionState") == 17
        and st.get("Gait") == 0x3002
        and st.get("ControlUsageMode") == 1
    )
    print("READY" if ok else "NOT-READY")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
