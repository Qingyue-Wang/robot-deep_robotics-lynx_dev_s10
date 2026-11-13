#!/usr/bin/env python3
"""向 liaison 提交一条任务并流式打印 pilot 事件。

用法: python3 scripts/submit_task.py "<任务文本>" [超时秒数]
"""
import sys
import time

sys.path.insert(0, "/tmp/pbgen")

import grpc  # noqa: E402

import pilot_pb2 as pilot_pb  # noqa: E402
from robonix_contracts_pb2_grpc import RobonixSystemLiaisonSubmitStub  # noqa: E402

TEXT = sys.argv[1] if len(sys.argv) > 1 else "抓萝卜"
TIMEOUT = int(sys.argv[2]) if len(sys.argv) > 2 else 300

stub = RobonixSystemLiaisonSubmitStub(grpc.insecure_channel("127.0.0.1:50081"))
task = pilot_pb.Task(task_id=f"t{int(time.time())}", session_id="s1", source=1, text=TEXT)

t0 = time.time()
try:
    for ev in stub.SubmitTask(task, timeout=TIMEOUT):
        dt = time.time() - t0
        kind = ev.event_kind
        body = ""
        if ev.final_text:
            body = ev.final_text
        elif ev.text_chunk:
            body = ev.text_chunk
        elif kind == "plan" or ev.HasField("plan"):
            try:
                ops = getattr(ev.plan, "operations", [])
                body = f"plan={ev.plan.plan_id} ops={len(ops)}"
            except Exception:
                body = "plan=<n/a>"
        elif ev.HasField("status"):
            body = f"state={ev.status.state} {ev.status.message}"
        elif ev.HasField("task_state"):
            body = f"task_state={ev.task_state}"
        print(f"[{dt:6.1f}s] {kind} {body}"[:600], flush=True)
except grpc.RpcError as e:
    print(f"[grpc] {e.code()}: {e.details()}", flush=True)

print("=== submit done ===", flush=True)
