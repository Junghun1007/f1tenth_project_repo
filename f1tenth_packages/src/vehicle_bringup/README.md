# vehicle_bringup

## Joy Raw Input Test

This step only checks raw controller input:

```text
8BitDo Controller -> joy_node -> /joy
```

It does not run `manual_control_node`, `vesc_interface_node`, duty conversion, or servo conversion.

Build from the workspace root:

```bash
cd /home/ohslo/Desktop/ROS_Test/autodrive_ws
colcon build --packages-select vehicle_bringup
source install/setup.bash
```

Run `joy_node` for the 8BitDo controller on `/dev/input/js1`:

```bash
ros2 launch vehicle_bringup joy_test.launch.py
```

Check that `/joy` is published:

```bash
ros2 topic list
ros2 topic echo /joy
```

The `/joy` message type is `sensor_msgs/msg/Joy`. Check the `axes` and `buttons` arrays while moving the controller.

Current important mapping from `config/manual_keymap.yaml`:

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
