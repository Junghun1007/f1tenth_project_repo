#!/usr/bin/env python3
"""
8BitDo Ultimate 2.4GHz controller input scanner for D-input mode.

Run examples:
  python3 controller_input_test.py --list
  python3 controller_input_test.py
  python3 controller_input_test.py --map
  python3 controller_input_test.py /dev/input/js0

Set the controller/receiver to D-input mode before running this script.
Press one control at a time and record the printed axis/button numbers.
"""

import argparse
import glob
import os
import select
import struct
import sys
from dataclasses import dataclass, field


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT = struct.Struct("IhBB")
DEFAULT_KEYMAP_OUTPUT = "autodrive_ws/src/vehicle_bringup/config/manual_keymap.yaml"
DEFAULT_AXIS_THRESHOLD = 16000


MAPPING_STEPS = [
    ("left_stick_left", "왼쪽 조이스틱을 왼쪽으로 끝까지 밀기"),
    ("left_stick_right", "왼쪽 조이스틱을 오른쪽으로 끝까지 밀기"),
    ("left_stick_up", "왼쪽 조이스틱을 위로 끝까지 밀기"),
    ("left_stick_down", "왼쪽 조이스틱을 아래로 끝까지 밀기"),
    ("right_stick_left", "오른쪽 조이스틱을 왼쪽으로 끝까지 밀기"),
    ("right_stick_right", "오른쪽 조이스틱을 오른쪽으로 끝까지 밀기"),
    ("right_stick_up", "오른쪽 조이스틱을 위로 끝까지 밀기"),
    ("right_stick_down", "오른쪽 조이스틱을 아래로 끝까지 밀기"),
    ("button_x", "X 버튼 누르기"),
    ("button_y", "Y 버튼 누르기"),
    ("button_b", "B 버튼 누르기"),
    ("button_a", "A 버튼 누르기"),
    ("dpad_east", "동쪽 키 누르기. 일반적으로 십자키 오른쪽"),
    ("dpad_west", "서쪽 키 누르기. 일반적으로 십자키 왼쪽"),
    ("dpad_south", "남쪽 키 누르기. 일반적으로 십자키 아래"),
    ("dpad_north", "북쪽 키 누르기. 일반적으로 십자키 위"),
    ("button_minus", "- 버튼 누르기"),
    ("button_plus", "+ 버튼 누르기"),
    ("button_select", "select 버튼 누르기"),
    ("trigger_lt", "LT 누르기"),
    ("trigger_rt", "RT 누르기"),
    ("button_l4", "L4 버튼 누르기"),
    ("button_r4", "R4 버튼 누르기"),
    ("button_lb", "LB 버튼 누르기"),
    ("button_rb", "RB 버튼 누르기"),
    ("button_pr", "PR 버튼 누르기"),
    ("button_pl", "PL 버튼 누르기"),
]


@dataclass
class ControllerState:
    axes: dict[int, int] = field(default_factory=dict)
    buttons: dict[int, int] = field(default_factory=dict)
    printed_axes: dict[int, int] = field(default_factory=dict)


def device_name(device_path: str) -> str:
    base_name = os.path.basename(device_path)
    sysfs_name = f"/sys/class/input/{base_name}/device/name"
    try:
        with open(sysfs_name, "r", encoding="utf-8") as name_file:
            return name_file.read().strip()
    except OSError:
        return "unknown"


def joystick_devices() -> list[str]:
    return sorted(glob.glob("/dev/input/js*"))


def pick_device(devices: list[str]) -> str | None:
    if not devices:
        return None

    for device_path in devices:
        if "8bitdo" in device_name(device_path).lower():
            return device_path

    return devices[0]


def print_devices(devices: list[str]) -> None:
    if not devices:
        print("No joystick devices found under /dev/input/js*.")
        print("Check the 2.4GHz dongle, pairing state, and D-input mode.")
        return

    for device_path in devices:
        print(f"{device_path}: {device_name(device_path)}")


def warn_if_not_direct_input(name: str) -> None:
    lower_name = name.lower()
    if "xbox" in lower_name or "x-box" in lower_name or "xinput" in lower_name:
        print()
        print("WARNING: This device name looks like XInput, not D-input.")
        print("Set the 8BitDo controller/receiver to D-input mode and reconnect it.")
        print()


def normalized_axis(value: int) -> float:
    return max(-1.0, min(1.0, value / 32767.0))


def should_print_axis(
    state: ControllerState,
    axis_number: int,
    value: int,
    delta: int,
) -> bool:
    previous = state.printed_axes.get(axis_number)
    if previous is None:
        return True
    if abs(value - previous) >= delta:
        return True
    return value == 0 and previous != 0


def print_event(
    state: ControllerState,
    event_time: int,
    event_value: int,
    event_type: int,
    event_number: int,
    axis_delta: int,
    show_init: bool,
) -> None:
    is_init = bool(event_type & JS_EVENT_INIT)
    clean_type = event_type & ~JS_EVENT_INIT

    if is_init and not show_init:
        if clean_type == JS_EVENT_AXIS:
            state.axes[event_number] = event_value
        elif clean_type == JS_EVENT_BUTTON:
            state.buttons[event_number] = event_value
        return

    prefix = "INIT " if is_init else ""

    if clean_type == JS_EVENT_BUTTON:
        state.buttons[event_number] = event_value
        action = "DOWN" if event_value else "UP"
        print(f"{prefix}BUTTON {event_number}: {action} value={event_value}")
        return

    if clean_type == JS_EVENT_AXIS:
        state.axes[event_number] = event_value
        if not is_init and not should_print_axis(state, event_number, event_value, axis_delta):
            return

        state.printed_axes[event_number] = event_value
        print(
            f"{prefix}AXIS {event_number}: "
            f"value={event_value:+6d} normalized={normalized_axis(event_value):+.3f}"
        )
        return

    print(
        f"{prefix}UNKNOWN type=0x{event_type:02x} "
        f"number={event_number} value={event_value} time={event_time}"
    )


def read_js_events(fd: int) -> list[tuple[int, int, int, int]]:
    try:
        data = os.read(fd, JS_EVENT.size * 32)
    except BlockingIOError:
        return []

    events = []
    for offset in range(0, len(data), JS_EVENT.size):
        chunk = data[offset : offset + JS_EVENT.size]
        if len(chunk) == JS_EVENT.size:
            events.append(JS_EVENT.unpack(chunk))
    return events


def drain_events(fd: int) -> None:
    while True:
        readable, _, _ = select.select([fd], [], [], 0)
        if not readable:
            return
        read_js_events(fd)


def wait_for_mapping_event(
    fd: int,
    axis_threshold: int,
) -> dict[str, int | str]:
    while True:
        readable, _, _ = select.select([fd], [], [], 0.5)
        if not readable:
            continue

        for _, event_value, event_type, event_number in read_js_events(fd):
            clean_type = event_type & ~JS_EVENT_INIT
            if event_type & JS_EVENT_INIT:
                continue

            if clean_type == JS_EVENT_BUTTON and event_value == 1:
                return {
                    "type": "button",
                    "number": event_number,
                    "pressed_value": event_value,
                    "released_value": 0,
                }

            if clean_type == JS_EVENT_AXIS and abs(event_value) >= axis_threshold:
                return {
                    "type": "axis",
                    "number": event_number,
                    "direction": 1 if event_value > 0 else -1,
                    "value": event_value,
                    "threshold": axis_threshold,
                }


def format_mapping_value(mapping_value: dict[str, int | str]) -> str:
    if mapping_value["type"] == "button":
        return (
            "      type: button\n"
            f"      number: {mapping_value['number']}\n"
            f"      pressed_value: {mapping_value['pressed_value']}\n"
            f"      released_value: {mapping_value['released_value']}\n"
        )

    return (
        "      type: axis\n"
        f"      number: {mapping_value['number']}\n"
        f"      direction: {mapping_value['direction']}\n"
        f"      threshold: {mapping_value['threshold']}\n"
        f"      sample_value: {mapping_value['value']}\n"
    )


def write_manual_keymap(
    output_path: str,
    device_path: str,
    name: str,
    mapping: dict[str, dict[str, int | str]],
) -> None:
    lines = [
        "manual_keymap:",
        "  controller:",
        '    mode: "d_input"',
        '    connection: "2.4ghz_dongle"',
        f'    device: "{device_path}"',
        f'    name: "{name}"',
        "",
        "  controls:",
    ]

    for key, _ in MAPPING_STEPS:
        lines.append(f"    {key}:")
        lines.append(format_mapping_value(mapping[key]).rstrip())
        lines.append("")

    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write("\n".join(lines).rstrip() + "\n")


def run_guided_mapping(
    device_path: str,
    output_path: str,
    axis_threshold: int,
) -> int:
    name = device_name(device_path)
    print(f"Using {device_path}: {name}")
    warn_if_not_direct_input(name)
    print("Guided keymap mode.")
    print("각 단계마다 Enter를 누른 뒤 지시한 입력만 한 번 움직이거나 누르세요.")
    print("입력 후에는 손을 떼고 다음 단계로 넘어가면 됩니다.")
    print(f"Output: {output_path}")
    print()

    try:
        fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
    except PermissionError:
        print(f"Permission denied: {device_path}", file=sys.stderr)
        print("Try: sudo python3 controller_input_test.py --map", file=sys.stderr)
        print("Or add your user to the input group, then log out/in.", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"Failed to open {device_path}: {exc}", file=sys.stderr)
        return 1

    mapping: dict[str, dict[str, int | str]] = {}

    try:
        drain_events(fd)
        input("모든 버튼에서 손을 떼고 스틱을 중앙에 둔 뒤 Enter를 누르세요.")

        for index, (key, prompt) in enumerate(MAPPING_STEPS, start=1):
            print()
            input(f"[{index}/{len(MAPPING_STEPS)}] {prompt}. 준비되면 Enter.")
            drain_events(fd)
            print("입력 대기 중...")
            mapping[key] = wait_for_mapping_event(fd, axis_threshold)
            print(f"기록됨: {key} -> {mapping[key]}")
            input("손을 떼고 스틱을 중앙에 둔 뒤 Enter.")
            drain_events(fd)

        write_manual_keymap(output_path, device_path, name, mapping)
        print()
        print(f"Saved keymap: {output_path}")
        return 0
    except KeyboardInterrupt:
        print()
        print("Mapping cancelled.")
        return 130
    finally:
        os.close(fd)


def scan_device(device_path: str, axis_delta: int, show_init: bool) -> int:
    name = device_name(device_path)
    print(f"Using {device_path}: {name}")
    warn_if_not_direct_input(name)
    print("Press buttons, triggers, sticks, and D-pad one at a time.")
    print("Stop with Ctrl+C.")
    print()

    state = ControllerState()

    try:
        fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
    except PermissionError:
        print(f"Permission denied: {device_path}", file=sys.stderr)
        print("Try: sudo python3 controller_input_test.py", file=sys.stderr)
        print("Or add your user to the input group, then log out/in.", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"Failed to open {device_path}: {exc}", file=sys.stderr)
        return 1

    try:
        while True:
            readable, _, _ = select.select([fd], [], [], 0.5)
            if not readable:
                continue

            try:
                data = os.read(fd, JS_EVENT.size * 32)
            except BlockingIOError:
                continue

            for offset in range(0, len(data), JS_EVENT.size):
                chunk = data[offset : offset + JS_EVENT.size]
                if len(chunk) != JS_EVENT.size:
                    continue

                event = JS_EVENT.unpack(chunk)
                print_event(state, *event, axis_delta=axis_delta, show_init=show_init)
    except KeyboardInterrupt:
        print()
        print("Stopped.")
        return 0
    finally:
        os.close(fd)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read Linux joystick events from an 8BitDo controller in D-input mode."
    )
    parser.add_argument(
        "device",
        nargs="?",
        help="Joystick device path, for example /dev/input/js0.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List joystick devices and exit.",
    )
    parser.add_argument(
        "--map",
        action="store_true",
        help="Run guided mapping and write manual_keymap.yaml.",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_KEYMAP_OUTPUT,
        help=f"Output YAML path for --map. Default: {DEFAULT_KEYMAP_OUTPUT}",
    )
    parser.add_argument(
        "--show-init",
        action="store_true",
        help="Print initial axis/button states reported when the device opens.",
    )
    parser.add_argument(
        "--axis-threshold",
        type=int,
        default=DEFAULT_AXIS_THRESHOLD,
        help="Axis magnitude required to record an axis during --map.",
    )
    parser.add_argument(
        "--axis-delta",
        type=int,
        default=1024,
        help="Minimum axis value change before printing another axis event.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    devices = joystick_devices()

    if args.list:
        print_devices(devices)
        return 0

    device_path = args.device or pick_device(devices)
    if device_path is None:
        print_devices(devices)
        return 1

    if args.map:
        return run_guided_mapping(device_path, args.output, args.axis_threshold)

    return scan_device(device_path, args.axis_delta, args.show_init)


if __name__ == "__main__":
    raise SystemExit(main())
