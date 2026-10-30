#!/usr/bin/env python3
# Description: 手动拖臂、按空格记关节角——给「多观察位扫描」的锚点（observe_scan.anchors_deg）用。
#
# 为什么单独一个脚本：示教/观察位是**人拖着臂摆**出来的。标定台那套要起整个网页和部署，
# 这里只干一件事——**只读**地看当前关节角，空格记一条，退出时把能直接贴进清单的
# anchors_deg YAML 打出来。
#
# **本脚本绝不使能、绝不切控制模式、绝不发运动指令。** 臂必须是"软的"（在臂上按示教按钮
# 进入拖动模式），你拖到位置、松手、按空格。读的是 CAN 上的状态回帧，跟 piper_sdk 的
# 只读接口一样；不需要 rbnx / 相机 / 视觉服务。
#
# 用法（在任意目录跑都行，默认把结果写当前目录）：
#     python3 record_poses.py                    # 交互：空格记录 / u 撤销 / q 退出
#     python3 record_poses.py --out poses.json   # 指定输出文件
#     python3 record_poses.py --port can2        # 指定 CAN 口（默认自动找 USB-CAN）
#     python3 record_poses.py --once             # 只记一条就退出（测通路/顺手抓一个位姿）
#
# 记完把退出时打印的 anchors_deg 块贴进部署清单的 observe_scan 里即可（6 个数一组，可多组）。
import argparse
import json
import os
import shutil
import sys
import time
import unicodedata

CAN_BITRATE = 1000000
FACTOR = 1000.0          # piper SDK 的定点系数：角度/位置都 ×1000 传输


# ---------------------------------------------------------------------------
# CAN 口：自动找 USB-CAN（和臂原语/标定工具同一套逻辑：接口名会漂，按驱动拓扑认口）
# ---------------------------------------------------------------------------

def list_can_interfaces():
    import glob
    return sorted(os.path.basename(path) for path in glob.glob("/sys/class/net/can*"))


def find_usb_can():
    """USB-CAN 适配器的接口名（如 "can2"）；找不到返回 None。只读探测，不动接口。"""
    import glob
    for path in sorted(glob.glob("/sys/class/net/can*")):
        try:
            device = os.path.realpath(os.path.join(path, "device"))
        except OSError:
            continue
        if "usb" in device.lower():
            return os.path.basename(path)
    return None


def is_up(iface):
    try:
        with open("/sys/class/net/%s/operstate" % iface, encoding="utf-8") as handle:
            return handle.read().strip() in ("up", "unknown")
    except OSError:
        return False


def resolve_port(configured):
    if configured:
        return configured
    detected = find_usb_can()
    if detected:
        return detected
    raise SystemExit("找不到 USB-CAN 口（当前 CAN 口：%s）。用 --port 指定，例如 --port can2。\n"
                     "口没起来的话先：sudo ip link set can2 type can bitrate %d && sudo ip link set can2 up"
                     % (", ".join(list_can_interfaces()) or "无", CAN_BITRATE))


# ---------------------------------------------------------------------------
# 只读地读臂（不 import DirectArm：它的构造函数会清一次错误码，这里连写命令都不发）
# ---------------------------------------------------------------------------

class ReadOnlyArm:
    """只读的 CAN 会话：读关节角/末端位姿/夹爪/状态，**不使能、不动臂**。"""

    def __init__(self, port: str):
        from piper_sdk import C_PiperInterface_V2
        self.port = port
        self.piper = C_PiperInterface_V2(port)
        self.piper.ConnectPort()

    def joints_deg(self):
        joint = self.piper.GetArmJointMsgs().joint_state
        return [joint.joint_1 / FACTOR, joint.joint_2 / FACTOR, joint.joint_3 / FACTOR,
                joint.joint_4 / FACTOR, joint.joint_5 / FACTOR, joint.joint_6 / FACTOR]

    def end_pose(self):
        """末端 [x, y, z]（米）与 euler zyx（度）——只作记录参考，不进锚点。"""
        pose = self.piper.GetArmEndPoseMsgs().end_pose
        position = [pose.X_axis / FACTOR / 1000.0, pose.Y_axis / FACTOR / 1000.0,
                    pose.Z_axis / FACTOR / 1000.0]
        from scipy.spatial.transform import Rotation
        euler = Rotation.from_euler(
            "xyz", [pose.RX_axis / FACTOR, pose.RY_axis / FACTOR, pose.RZ_axis / FACTOR],
            degrees=True).as_euler("zyx", degrees=True).tolist()
        return position, euler

    def gripper_open_0to1(self):
        grip = self.piper.GetArmGripperMsgs().gripper_state
        return max(0.0, min(1.0, grip.grippers_angle / FACTOR / 100.0))

    def status(self):
        return self.piper.GetArmStatus().arm_status


ARM_STATUS_TEXT = {
    0x00: "正常", 0x01: "急停", 0x02: "无解", 0x03: "奇异点", 0x04: "目标角度超过限",
    0x05: "关节通信异常", 0x06: "关节抱闸未打开", 0x07: "机械臂发生碰撞",
    0x08: "拖动示教时超速(拖慢一点)", 0x09: "关节状态异常", 0x0A: "其它异常",
    0x0B: "示教记录", 0x0C: "示教执行", 0x0D: "示教暂停", 0x0E: "主控NTC过温",
    0x0F: "释放电阻NTC过温",
}


# ---------------------------------------------------------------------------
# 记录
# ---------------------------------------------------------------------------

def now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def make_record(arm: ReadOnlyArm, index: int) -> dict:
    position, euler = arm.end_pose()
    return {
        "index": index,
        "ts": now_text(),
        "joints_deg": [round(value, 3) for value in arm.joints_deg()],
        "pos_m": [round(value, 4) for value in position],
        "euler_zyx_deg": [round(value, 2) for value in euler],
        "gripper": round(arm.gripper_open_0to1(), 3),
    }


def save(path: str, port: str, records: list) -> None:
    payload = {"recorded_at": now_text(), "port": port, "poses": records}
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


def yaml_block(records: list) -> str:
    if not records:
        return "（还没记录任何位姿）"
    lines = ["  anchors_deg:"]
    for record in records:
        lines.append("    - [%s]" % ", ".join("%.2f" % value for value in record["joints_deg"]))
    return "\n".join(lines)


def print_summary(path: str, records: list) -> None:
    print()
    print("=" * 78)
    print("记了 %d 个位姿，已存：%s" % (len(records), os.path.abspath(path)))
    for record in records:
        print("  #%d  [%s]" % (record["index"] + 1,
                              ", ".join("%.2f" % value for value in record["joints_deg"])))
    print()
    print("贴进部署清单 skill.config.observe_scan（多观察位扫描的锚点）时用这段：")
    print()
    print(yaml_block(records))
    print()
    print("=" * 78)


def terminal_columns() -> int:
    """终端一行能放多少列（留一列余量，别正好顶到折行点）。"""
    return max(20, shutil.get_terminal_size(fallback=(80, 24)).columns - 1)


def fit_to_width(text: str, max_columns: int) -> str:
    """按**显示宽度**截断一行（中文占 2 列）。

    为什么必须截：状态行只要折行，`\\r` 就回不到原来那一行的行首，屏幕上会变成一坨
    重复的"当前 [...]"（第一版就是折在宿主终端宽度上）。
    """
    kept, width = [], 0
    for char in text:
        char_width = 2 if unicodedata.east_asian_width(char) in ("W", "F") else 1
        if width + char_width > max_columns:
            break
        kept.append(char)
        width += char_width
    return "".join(kept)


def write_status(line: str) -> None:
    """原地刷新状态行：回到行首（CR）+ 清到行尾（ANSI EL），不留残影、不换行。

    这里用 chr() 而不是转义写法，免得文件里多/少一个反斜杠：少了刷新不了（会变成
    一行行往下打），多了会把字面量 "\\r" 打到屏幕上。
    """
    sys.stdout.write(chr(13) + line + chr(27) + "[K")
    sys.stdout.flush()


def run_interactive(arm: ReadOnlyArm, path: str, interval: float) -> list:
    """交互循环：TTY 用 cbreak 单键（空格立即生效）；管道输入退回按行读。"""
    records = []
    status = arm.status()
    print("只读模式：不使能、不动臂。先在臂上按示教按钮进入拖动模式，拖到位后按【空格】记录。")
    print("状态：%s。空格=记录  u=撤销上一条  q=退出（自动保存）"
          % ARM_STATUS_TEXT.get(status.arm_status, hex(status.arm_status)))

    tty_mode = sys.stdin.isatty()

    def clear_line():
        if tty_mode:                              # 管道输入时没有那行状态，别刷 \r 留脏字符
            sys.stdout.write("\r\x1b[K")
            sys.stdout.flush()

    def record():
        record = make_record(arm, len(records))
        records.append(record)
        save(path, arm.port, records)
        clear_line()
        print("#%d 已记录：[%s]" % (record["index"] + 1,
                                   ", ".join("%.2f" % value for value in record["joints_deg"])),
              flush=True)

    def undo():
        if not records:
            print("还没有记录，撤销什么也不做。", flush=True)
            return
        removed = records.pop()
        save(path, arm.port, records)
        clear_line()
        print("已撤销 #%d：[%s]" % (removed["index"] + 1,
                                   ", ".join("%.2f" % value for value in removed["joints_deg"])),
              flush=True)

    if not sys.stdin.isatty():
        # 管道/重定向：一行一个命令。空行/空格行 = 记录；u = 撤销；q = 退出。
        print("（stdin 不是终端：按行读，空行=记录，u=撤销，q=退出）", flush=True)
        for line in sys.stdin:
            command = line.strip()
            if command.lower() == "q":
                break
            if command.lower() == "u":
                undo()
                continue
            record()
        return records

    import select
    import termios
    import tty

    old_settings = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())        # 单键立即生效，不用回车
        while True:
            status = arm.status()
            joints = arm.joints_deg()
            line = "当前 [%s]  已记 %d 条  %s" % (
                ", ".join("%6.2f" % value for value in joints), len(records),
                ARM_STATUS_TEXT.get(status.arm_status, hex(status.arm_status)))
            write_status(fit_to_width(line, terminal_columns()))
            ready, _, _ = select.select([sys.stdin], [], [], interval)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key in (" ", "\n", "\r"):
                record()
            elif key.lower() == "u":
                undo()
            elif key.lower() == "q":
                break
            elif key == "\x03":                  # Ctrl-C
                break
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        sys.stdout.write("\r\x1b[K")
        sys.stdout.flush()
    return records


def main() -> int:
    parser = argparse.ArgumentParser(
        description="只读记录当前关节角（给多观察位扫描的 anchors_deg 用）。"
                    "空格记录 / u 撤销 / q 退出；不会使能、不会动臂。")
    parser.add_argument("--port", default="", help="CAN 口名（默认自动找 USB-CAN）")
    parser.add_argument("--out", default="", help="输出 JSON 路径（默认当前目录 recorded_poses_<时间>.json）")
    parser.add_argument("--interval", type=float, default=0.2, help="状态刷新间隔（秒，默认 0.2）")
    parser.add_argument("--once", action="store_true", help="只记一条就退出（测通路/快速抓一个位姿）")
    args = parser.parse_args()

    port = resolve_port(args.port)
    if not is_up(port):
        print("⚠️ %s 当前是 DOWN，先把 CAN 拉起来：\n"
              "    sudo ip link set %s type can bitrate %d && sudo ip link set %s up"
              % (port, port, CAN_BITRATE, port), file=sys.stderr)
        return 2
    print("CAN 口：%s（只读，不会使能/不会动臂）" % port)

    arm = ReadOnlyArm(port)
    time.sleep(0.4)                              # 等 CAN 回帧刷一轮
    path = args.out or ("recorded_poses_%s.json" % time.strftime("%Y%m%d-%H%M%S"))

    if args.once:
        records = [make_record(arm, 0)]
        save(path, port, records)
        print_summary(path, records)
        return 0

    try:
        records = run_interactive(arm, path, args.interval)
    except KeyboardInterrupt:
        records = []
        print("\nCtrl-C：退出（已记录的内容在每次记录时已经落盘）")
        try:
            with open(path, encoding="utf-8") as handle:
                records = json.load(handle).get("poses", [])
        except (OSError, json.JSONDecodeError):
            pass
    save(path, port, records)
    print_summary(path, records)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
