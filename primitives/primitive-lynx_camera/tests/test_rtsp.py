import subprocess
import sys
import threading

import pytest

from lynx_camera import rtsp


@pytest.fixture
def pipeline(monkeypatch):
    processes = []
    popen = subprocess.Popen

    def start(script):
        def spawn(_command, **kwargs):
            process = popen([sys.executable, "-u", "-c", script], **kwargs)
            processes.append(process)
            return process

        monkeypatch.setattr(rtsp.subprocess, "Popen", spawn)
        return processes

    yield start
    for process in processes:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=1)


def test_fragmented_frames_and_stop_reap_decoder(pipeline):
    processes = pipeline(
        "import os,time; os.write(1,b'noise\\xff'); time.sleep(.02); "
        "os.write(1,b'\\xd8one\\xff\\xd9\\xff\\xd8two\\xff\\xd9'); time.sleep(30)"
    )
    stop = threading.Event()
    frames = rtsp.jpeg_frames("rtsp://camera", stop, 5, 1)
    try:
        assert next(frames) == b"\xff\xd8one\xff\xd9"
        assert next(frames) == b"\xff\xd8two\xff\xd9"
        stop.set()
        with pytest.raises(StopIteration):
            next(frames)
        assert processes[0].poll() is not None
    finally:
        frames.close()


@pytest.mark.parametrize("ending", ["import time; time.sleep(30)", ""])
def test_incomplete_frame_times_out_or_reports_eof_and_reaps_decoder(pipeline, ending):
    processes = pipeline("import os; os.write(1,b'\\xff\\xd8partial'); " + ending)
    frames = rtsp.jpeg_frames("rtsp://camera", threading.Event(), 5, .15)
    with pytest.raises((TimeoutError, RuntimeError)):
        next(frames)
    assert processes[0].poll() is not None
