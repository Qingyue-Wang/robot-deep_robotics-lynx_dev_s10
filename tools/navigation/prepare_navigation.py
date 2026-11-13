"""Explicitly select S10 navigation mode and flat gait; never send velocity."""
import argparse
import json
import socket
import struct
import time


def send(sock, kind, command, items):
    body = json.dumps({'PatrolDevice': {'Type': kind, 'Command': command,
        'Time': time.strftime('%Y-%m-%d %H:%M:%S'), 'Items': items}}).encode()
    sock.send(b'\xeb\x91\xeb\x90' + struct.pack('<HH', len(body), 0)
              + bytes([1, 0, 1]) + b'\0' * 5 + body)


def read_status(sock, timeout=3):
    send(sock, 0x00100064, 5, {})
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw = sock.recv(8192)
        except socket.timeout:
            continue
        if len(raw) < 16 or raw[:4] != b'\xeb\x91\xeb\x90':
            continue
        try:
            body = json.loads(raw[16:16 + struct.unpack_from('<H', raw, 4)[0]])
            status = body.get('PatrolDevice', {}).get('Items', {}).get('BasicStatus')
            if status:
                return status
        except (ValueError, TypeError):
            continue
    raise TimeoutError('No S10 BasicStatus feedback')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--host', default='10.21.33.103')
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((args.host, 30004))
        sock.settimeout(.25)
        before = read_status(sock)
        print('before', json.dumps(before), flush=True)
        if not args.apply:
            return
        if before.get('MotionState') != 17:
            raise RuntimeError('Robot must already be standing in RL state 17')
        send(sock, 0x00100002, 0x00500002, {'Mode': 1})
        deadline = time.monotonic() + 5
        while True:
            status = read_status(sock)
            if status.get('ControlUsageMode') == 1:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Navigation mode not confirmed: ' + str(status))
        send(sock, 0x00100001, 0x00300002, {'GaitParam': 0x3002, 'ActionParam': 0})
        deadline = time.monotonic() + 5
        while True:
            status = read_status(sock)
            if (status.get('MotionState') == 17 and status.get('ControlUsageMode') == 1
                    and status.get('Gait') == 0x3002):
                print('confirmed', json.dumps(status), flush=True)
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('Navigation gait not confirmed: ' + str(status))


if __name__ == '__main__':
    main()
