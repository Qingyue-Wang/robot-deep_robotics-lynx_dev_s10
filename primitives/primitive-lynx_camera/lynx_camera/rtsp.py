"""Bounded, interruptible RTSP reads using the robot's ffmpeg binary."""

import os
import select
import subprocess
import time


def jpeg_frames(url, stop, fps, timeout_s):
    command = [
        "ffmpeg", "-nostdin", "-loglevel", "error", "-rtsp_transport", "tcp",
        "-threads", "1", "-analyzeduration", "0", "-probesize", "32768",
        "-i", url, "-an", "-vf", f"fps={fps:g}",
        "-threads", "1", "-q:v", "3", "-f", "mjpeg", "pipe:1",
    ]
    with subprocess.Popen(command, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, bufsize=0) as process:
        buffer = bytearray()
        deadline = time.monotonic() + timeout_s
        try:
            while not stop.is_set():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"no complete frame within {timeout_s:g}s")
                if not select.select([process.stdout], [], [], 0.2)[0]:
                    continue
                chunk = os.read(process.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError("ffmpeg stream ended")
                buffer.extend(chunk)
                while True:
                    start = buffer.find(b"\xff\xd8")
                    end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                    if end < 0:
                        if len(buffer) > 8 * 1024 * 1024:
                            raise RuntimeError("ffmpeg frame exceeds 8 MiB")
                        break
                    frame = bytes(buffer[start:end + 2])
                    del buffer[:end + 2]
                    deadline = time.monotonic() + timeout_s
                    yield frame
        finally:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
