# vehicle_bringup

`vehicle_bringup` owns the launch files and runtime configuration used to
start the F1TENTH vehicle. It replaces the former `vehicle_launcher` and
`vehicle_config` packages.

## Launch files

- `joy_test.launch.py`: starts only the joystick input path.
- `vesc_only.launch.py`: starts only `vesc_bridge_node`.
- `manual_drive.launch.py`: starts joystick input, direct manual actuator
  commands, and the VESC bridge.
- `auto_drive.launch.py`: starts BEV centerline generation, Stanley/PID
  autonomous control, and the VESC bridge.

```bash
ros2 launch vehicle_bringup joy_test.launch.py
ros2 launch vehicle_bringup vesc_only.launch.py
ros2 launch vehicle_bringup manual_drive.launch.py
ros2 launch vehicle_bringup auto_drive.launch.py
```

The VESC launch files default to the Jetson UART device `/dev/ttyTHS1`. A
different device path can be supplied without editing YAML:

```bash
ros2 launch vehicle_bringup vesc_only.launch.py \
  vesc_port:=/dev/serial/by-id/usb-your-adapter-id
```

## Configuration

- `config/vesc_config.yaml`: serial transport, command limits, and telemetry.
- `config/manual_vesc_config.yaml`: manual accelerator, brake, and steering.
- `auto_control/config/auto_control.yaml`: Stanley, speed PID, vehicle, and
  autonomous actuator parameters.
- `config/controller_keymap.yaml`: 8BitDo controller mapping.

The previous empty `joy.yaml` was removed because `joy_initializer` already
owns the joystick node defaults.

## Autonomous driving

The autonomous launch follows `/camera/image_bev_lane`, starts as soon as a
valid centerline and fresh VESC telemetry are present, and uses the same
`/vesc/duty` and `/vesc/servo_position` interface as manual driving. Never run
the manual and autonomous launches together.

By default the preview contains only the lane-recognition result. Disable the
window completely with:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py preview_enabled:=false
```

Start without automatic motion for a lifted-wheel check with
`auto_enabled:=false`. Controller parameters are installed from
`auto_control/config/auto_control.yaml`.

## 8BitDo input safety

The manual launch uses the project-owned `joy_input_node` for the 8BitDo
controller in D-input mode. It selects the exact SDL controller name, publishes
only `/joy`, and has no haptic subsystem, feedback subscription, or rumble API.
The actuator commander consumes `/joy` directly; no conversion/debug process
runs between the controller and actuator commands.

The initial `Opened joystick input` message is normal startup. If the kernel or
SDL actually removes the receiver, the node prints `Joystick disconnected by
SDL`, stops publishing immediately so the 0.30-second actuator watchdog can
stop the vehicle, and waits for the same named controller. Only a subsequent
physical reattachment prints `Reopened joystick after a real SDL disconnect`.

RB gear changes are edge-triggered and immediate: they are accepted only while
the commanded duty is already zero. A rejected change is not stored or replayed
later. `vesc_bridge` communicates through `/dev/ttyTHS1` and does not access the
8BitDo USB device.
