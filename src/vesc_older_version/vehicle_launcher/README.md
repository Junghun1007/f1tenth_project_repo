# vehicle_launcher

## Joy Raw Input Test

This step only checks raw controller input:

```text
8BitDo Controller -> joy_initializer -> joy_node -> /joy
```

It does not run `joy_params_converter_node`, `actuator_commander_node`, or `vesc_initialize_node`.
`joy_initializer` wraps the upstream ROS `joy_node` with project defaults.

Build from the workspace root:

```bash
cd /home/ohslo/Desktop/ROS_Test/autodrive_ws
colcon build --packages-select joy_initializer vehicle_config vehicle_launcher manual_control
source install/setup.bash
```

Run `joy_initializer` for the 8BitDo controller on `/dev/input/js0`:

```bash
ros2 launch vehicle_launcher joy_test.launch.py
```

Check that `/joy` is published:

```bash
ros2 topic list
ros2 topic echo /joy
```

The `/joy` message type is `sensor_msgs/msg/Joy`. Check the `axes` and `buttons` arrays while moving the controller.

Current important mapping from `vehicle_config/config/controller_keymap.yaml`:

```text
left_stick.x: axis 0
left_stick.y: axis 1
right_stick.x: axis 2
right_stick.y: axis 3
RT trigger: axis 4, button 9
LT trigger: axis 5, button 8
dpad left/right: axis 6
dpad up/down: axis 7
A button: 0
B button: 1
X button: 3
Y button: 4
LB: 6
RB: 7
```
