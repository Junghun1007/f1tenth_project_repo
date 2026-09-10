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
  read-only vehicle dynamics/CAN monitor, including direct CANable 2 SLCAN
  telemetry when selected.
- `auto_drive.launch.py`: starts BEV images, ML lane/centerline detection, Stanley/PID
  autonomous control, the VESC bridge, and the vehicle dynamics/CAN monitor.

```bash
ros2 launch vehicle_bringup joy_test.launch.py
ros2 launch vehicle_bringup vesc_only.launch.py
ros2 launch vehicle_bringup manual_drive.launch.py
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py
ros2 launch vehicle_bringup auto_drive.launch.py
```

Manual driving defaults to the vehicle-unique `/autopilot03` namespace for
all three nodes and the `/autopilot03/joy`, `/autopilot03/manual/*`, and
`/autopilot03/vesc/*` topics. This prevents another vehicle on the same ROS
domain from publishing actuator commands into this vehicle. Override the
identifier when required:

```bash
ros2 launch vehicle_bringup manual_drive.launch.py \
  vehicle_namespace:=autopilot03
```

The dynamics launch defaults to the current UART VESC telemetry path. To use
VESC CAN STATUS broadcasts directly, bring up `can0` first and launch with
`input_mode:=socketcan`. See `vehicle_dynamics_monitor/README.md` for CAN and
accelerometer-compensation details.

On the Jetson L4T kernel without `slcan`, a CANable 2 at `/dev/ttyACM0` can be
read directly without creating `can1`:

```bash
python3 -m pip install 'python-can[serial]'
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

The VESC continues to receive actuator commands through the existing UART
bridge. The CANable path is receive-only and is used only for vehicle telemetry
and acceleration estimation.

The VESC launch files default to the Jetson UART device `/dev/ttyTHS1`. A
different device path can be supplied without editing YAML:

```bash
ros2 launch vehicle_bringup vesc_only.launch.py \
  vesc_port:=/dev/serial/by-id/usb-your-adapter-id
```

## Configuration

- `config/vesc_config.yaml`: serial transport, command limits, and telemetry.
- `config/manual_vesc_config.yaml`: manual accelerator, 0..8 A LT electrical
  brake ramp, and steering.
- `auto_control/config/auto_control.yaml`: Stanley, speed PID, vehicle, and
  autonomous duty/electrical-brake actuator parameters.
- `config/controller_keymap.yaml`: 8BitDo controller mapping.

The previous empty `joy.yaml` was removed because `joy_initializer` already
owns the joystick node defaults.

## Autonomous driving

The pipeline is `camera_driver -> bev_processor -> line_detactor -> auto_control`.
The controller consumes `/line_detactor/result` centerline points, keeping their
path order through corners. The preview displays detected lanes and the yellow
centerline. Rule-based sliding-window detection and `/camera/image_bev_lane`
publication have been removed.

After pulling `0906ML`, rebuild the changed packages on the Jetson before launch:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bev_processor line_detactor auto_control vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The existing configuration-file paths and CAN arguments remain usable:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  bev_params_file:=/home/autopilot03/Desktop/0822ver3/f1tenth_project_repo/bev_config_test.yaml \
  auto_control_params_file:=/home/autopilot03/Desktop/0822ver3/f1tenth_project_repo/auto_control_test.yaml \
  camera_params_file:=/home/autopilot03/Desktop/0822ver3/f1tenth_project_repo/camera_config_test.yaml \
  auto_enabled:=true \
  preview_enabled:=true \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

These are external YAML paths; replace them if the files move. To override ML
lane/centerline settings, add
`line_detactor_params_file:=/absolute/path/line_detactor_test.yaml`.
Use `src/line_detactor/config/line_detactor.yaml` as the template, with root
`line_detactor.ros__parameters`. Defaults retain 0.65m lane width, spatial smoothing,
outer-boundary corner weighting, and yellow rendering. Parameters load at startup.

`bev_params_file` remains necessary for projection geometry, startup measurement
and capture settings. Old `lane_*` BEV keys are ignored and can be deleted.
The launch derives ML input topic, physical extents, result topic, and controller
scale/frame consistently. It requires X=0 at the front axle, symmetric Y bounds,
and BEV dimensions matching the static model. It requires connection, centerline
and result publication enabled. Individual supported launch arguments override YAML.
Old controller `lane_topic`, `lane_pixel_threshold`, `path_local_smoothing_window_m`
and `path_outlier_threshold_m` no longer affect this pipeline.

SLCAN/SocketCAN inputs feed receive-only telemetry and camera acceleration
compensation. Actuation and controller ERPM still use `vesc_bridge` through UART
(`/dev/ttyTHS1`, overridable by `vesc_port`). `python-can[serial]` is required for SLCAN.

`auto_enabled:=true` arms motion once fresh valid path and VESC data arrive.
Use `auto_enabled:=false` to start disabled and `preview_enabled:=false` to hide the GUI.
The ML preview has no BEV Space-key shortcut; disable explicitly with:

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: false"
```

Invalid/stale path or ERPM and VESC disconnect send duty zero and centered steering.
Stop-line detection alone does not command stopping. Never run manual and auto launches together.
Dataset capture now saves only `origin_bev`; removed rule-based labels are not generated.

## ML pipeline diagnosis

Updating sources with `git pull` does not update an installed launch file.
Rebuild **vehicle_bringup** together with `line_detactor`, `auto_control` and
`bev_processor`; rebuilding only BEV does not install the ML launch integration.
Use a fresh terminal with the Humble environment and this workspace's
`install/local_setup.bash` when checking for an older workspace overlay.

The current launch prints `[ML auto drive]` with its launch path, selected YAML,
model path, topic chain, and preview selection. A loaded detector prints
`Line detector ready`. The default ML window is `BEV lane TensorRT preview`
and contains inference/FPS/lanes information below the image; the raw BEV preview
is disabled by the integrated launch. An ML window can still show just the image
if the model detects no lanes, so use the status logs to distinguish these cases.

Read-only inspection in the same ROS environment as the running launch:

```bash
ros2 pkg prefix vehicle_bringup
ros2 pkg prefix line_detactor
ros2 node list
ros2 param get /bev_processor preview_enabled
ros2 param get /line_detactor model_path
ros2 topic info /camera/image_bev
ros2 topic info /line_detactor/result
```

The pipeline needs `/bev_processor`, `/line_detactor`, `/auto_control` and
`/vesc_bridge_node`. `ros2 pkg prefix` should point into the intended 0906ML install.
The BEV topic needs a publisher and ML subscriber; the result topic needs an ML
publisher and controller subscriber. Extra diagnostic subscriptions may also appear.

`Auto status` now separates actuator stop state from ML reception:

- `lane_rx=0`: no LaneResult has reached this controller.
- `lane_status=no_centerline(state=...)`: ML messages arrive, but contain no centerline.
- `centerline_sample_limit`: the generator exceeded its sample budget.
- `insufficient_contiguous_path`: the received points do not meet the configured ROI,
  length, point-count or adjacent-gap requirements.
- `accepted`: the latest received path passed the controller's checks. Use
  `last_rx_age`, `capture_age_at_rx` and `state` to check subsequent staleness.
- `state=vesc_disconnected`, `waiting_for_erpm` or `erpm_timeout`: inspect UART VESC
  telemetry separately; SLCAN dynamics does not replace controller ERPM/connection.

The diagnostic output does not relax path freshness, validity or motor limits.

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
