# vehicle_bringup

`vehicle_bringup` owns the launch files and runtime configuration used to
start the F1TENTH vehicle. It replaces the former `vehicle_launcher` and
`vehicle_config` packages.

## Launch files

- `joy_test.launch.py`: starts only the joystick input path.
- `vesc_only.launch.py`: starts only `vesc_bridge_node`.
- `manual_drive.launch.py`: starts joystick input, direct manual actuator
  commands, and the VESC bridge.
- `manual_drive_with_dynamics.launch.py`: starts manual driving plus the
  read-only vehicle dynamics/CAN monitor.
- `auto_drive.launch.py`: starts BEV centerline generation, Stanley/PID
  autonomous control, and the VESC bridge.

```bash
ros2 launch vehicle_bringup joy_test.launch.py
ros2 launch vehicle_bringup vesc_only.launch.py
ros2 launch vehicle_bringup manual_drive.launch.py
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py
ros2 launch vehicle_bringup auto_drive.launch.py
```

The dynamics launch defaults to the current UART VESC telemetry path. To use
VESC CAN STATUS broadcasts directly, bring up `can0` first and launch with
`input_mode:=socketcan`. See `vehicle_dynamics_monitor/README.md` for CAN and
accelerometer-compensation details.

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

Camera stabilization defaults to high-frequency-only correction at 3 Hz.
Compare correction on and off without changing the BEV path:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  imu_stabilization_enabled:=true
ros2 launch vehicle_bringup auto_drive.launch.py \
  imu_stabilization_enabled:=false
```

Tune the split with
`imu_stabilization_high_frequency_vibration_cutoff_hz:=3.0`. A larger value
passes more slow/medium motion through; a smaller value corrects a wider
frequency range.

Start without automatic motion for a lifted-wheel check with
`auto_enabled:=false`. Controller parameters are installed from
`auto_control/config/auto_control.yaml`.

## 8BitDo Bluetooth input safety

The manual launch uses the project-owned `joy_input_node` for the 8BitDo
controller in Bluetooth D-input mode. Remove the 2.4 GHz receiver before
launch. The node uses SDL GameController's standard layout and matches the
`8BitDo` name substring, publishes only `/joy`, and has no haptic subsystem,
feedback subscription, or rumble API. The actuator commander consumes `/joy`
directly; no conversion/debug process runs between the controller and actuator
commands.

The initial `Opened Bluetooth game controller` message is normal startup. If
BlueZ or SDL removes the controller, the node stops publishing immediately so
the 0.30-second actuator watchdog can stop the vehicle. Reconnection is
automatic after BlueZ restores the trusted Bluetooth link.

The manual mapping is `RT=axis5`, `LT=axis4`, left-stick X=`axis0`, and
`RB=button10`. Both triggers use `0.0` released and `1.0` fully pressed. Verify
these values with `joy_test.launch.py` while the drive wheels are lifted before
the first powered run.

If SDL exposes a generic name without `8BitDo`, clear the name filter and use
the first SDL-recognized controller:

```bash
ros2 launch vehicle_bringup joy_test.launch.py controller_name_contains:=""
```

RB gear changes are edge-triggered and immediate: they are accepted only while
the commanded duty is already zero. A rejected change is not stored or replayed
later. `vesc_bridge` communicates through `/dev/ttyTHS1` and does not access the
Bluetooth controller transport.
