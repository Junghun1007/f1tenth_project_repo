# joy_initializer

`joy_initializer` is the project-owned wrapper for the ROS `joy_linux` package.

It does not reimplement joystick device reading. It launches the proven
`joy_linux_node` with AutoDrive defaults, so other packages can depend on
`joy_initializer` instead of opening the controller themselves. The controller
is selected by its Linux name and the force-feedback device path is deliberately
empty, preventing rumble output to the 8BitDo dongle.

Run only the joystick input node:

```bash
ros2 launch joy_initializer joy.launch.py
```

Override the Linux joystick device path or name when needed:

```bash
ros2 launch joy_initializer joy.launch.py \
  device_path:=/dev/input/js0 device_name:="8BitDo controller name"
```
