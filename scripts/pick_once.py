#!/usr/bin/env python3
"""直接让 executor 跑一次抓取技能（不过 pilot、不过大模型）。

为什么需要它：走 pilot 时"抓一下桌上的胡萝卜"要先让大模型出一个 RTDL 计划，
计划偶尔解析失败（模型吐空白内容）或把上一轮的结论当成本轮已完成；而抓取动作
本身是确定性的。这个脚本把「一次抓取」直接派给 executor，省掉 LLM 这一跳。

用法（部署根目录下）：
    ./scripts/pick_once.py                          # 抓 carrot（默认目标）
    ./scripts/pick_once.py --target 胡萝卜
    ./scripts/pick_once.py --action put_down        # 放下
    ./scripts/pick_once.py --action observe         # 只看不抓
    ./scripts/pick_once.py --node-kind 3 --timeout 300

前置：protos 已编译到 /tmp/pbgen（和 scripts/submit_task.py 用同一份）：
    PROTO=rbnx-boot/cache/service-map-rbnx/rbnx-build/proto-staging
    python3 -m grpc_tools.protoc -I "$PROTO" --python_out=/tmp/pbgen \
        --grpc_python_out=/tmp/pbgen "$PROTO"/*.proto

退出码：0 = 计划跑完且该节点成功；1 = 跑完但失败；2 = 连不上/参数错。
"""
import argparse
import json
import sys
import time

sys.path.insert(0, "/tmp/pbgen")

import grpc                                        # noqa: E402

import pilot_pb2                                   # noqa: E402
import robonix_contracts_pb2_grpc                  # noqa: E402

# 每个能力**参数名不一样**（实测踩过）：pick 要 `object_name`（"target" 会被 MCP 的
# inputSchema 拒掉：1 validation error for pickArguments / object_name Field required），
# observe 用 `target`，put_down 不带参数。
CONTRACTS = {
    "pick": ("depth_grasp", "robonix/skill/pick/pick", "object_name"),
    "put_down": ("depth_grasp", "robonix/skill/pick/put_down", None),
    "observe": ("depth_grasp", "robonix/skill/pick/observe", "target"),
}
NODE_STATE_NAMES = {0: "PENDING", 1: "RUNNING", 2: "SUCCEEDED", 3: "FAILED",
                    4: "CANCELED", 5: "TIMEOUT", 6: "PAUSED", 7: "VERIFYING"}


def build_plan(action, target, node_kind, plan_id):
    provider, contract, arg_name = CONTRACTS[action]
    args = {arg_name: target} if arg_name else {}
    call = pilot_pb2.CapabilityCall(
        call_id=f"c{int(time.time() * 1000) % 1000000}",
        provider_id=provider,
        contract_id=contract,
        args_json=json.dumps(args, ensure_ascii=False))
    node = pilot_pb2.RtdlNode(node_kind=node_kind, call=call,
                              op_id="op1", description=f"{action} {target}")
    return pilot_pb2.Plan(plan_id=plan_id, session_id=f"s{int(time.time())}",
                          round=1, nodes=[node], root_index=0)


def main() -> int:
    parser = argparse.ArgumentParser(description="直接让 executor 跑一次抓取技能")
    parser.add_argument("--target", default="carrot", help="抓什么（默认 carrot）")
    parser.add_argument("--action", choices=sorted(CONTRACTS), default="pick")
    parser.add_argument("--node-kind", type=int, default=2,
                        help="RTDL 叶子节点类型（默认 2 = 叶子能力调用（实测：0=未指定空操作、1=parallel、2=leaf）；executor 对非法值会报 invalid node_kind）")
    parser.add_argument("--executor", default="127.0.0.1:50061")
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()

    plan = build_plan(args.action, args.target, args.node_kind, f"p{int(time.time())}")
    started = time.monotonic()
    print(f"[pick_once] {args.action} target={args.target!r} -> {args.executor} "
          f"(node_kind={args.node_kind}, timeout={args.timeout:.0f}s)", flush=True)

    stub = robonix_contracts_pb2_grpc.RobonixSystemExecutorExecuteStub(
        grpc.insecure_channel(args.executor))
    ok, any_failed = False, None
    try:
        for event in stub.Execute(plan, timeout=args.timeout):
            elapsed = time.monotonic() - started
            kind = event.event_kind
            if kind == 0:                                  # PLAN_STARTED
                print(f"[{elapsed:6.1f}s] plan started {event.plan_started.plan_id}",
                      flush=True)
            elif kind == 1:                                # NODE_STATE
                state = event.node_state
                name = NODE_STATE_NAMES.get(state.state, str(state.state))
                leaf = state.leaf_result
                extra = ""
                if leaf and (leaf.output or leaf.error):
                    extra = f" output={leaf.output[:400]!r} error={leaf.error[:200]!r}"
                if state.operator_detail:
                    extra += f" detail={state.operator_detail[:200]!r}"
                print(f"[{elapsed:6.1f}s] node {state.node_index} {name}{extra}", flush=True)
                if name == "SUCCEEDED":
                    ok = True
                elif name in ("FAILED", "CANCELED", "TIMEOUT"):
                    ok = False
            elif kind == 2:                                # PLAN_COMPLETE
                any_failed = event.plan_complete.any_failed
                print(f"[{elapsed:6.1f}s] plan complete any_failed={any_failed}",
                      flush=True)
    except grpc.RpcError as error:
        print(f"[pick_once] gRPC 失败：{error.code()} {error.details()}", flush=True)
        return 2

    elapsed = time.monotonic() - started
    good = bool(ok) and not any_failed
    print(f"[pick_once] 用时 {elapsed:.1f}s，"
          f"结果 {'成功' if good else '未成功（看 rbnx-boot/logs/depth_grasp.log）'}",
          flush=True)
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
