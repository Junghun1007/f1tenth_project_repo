# joy_initializer

`joy_initializer` is the project-owned wrapper for the ROS `joy` package.

It does not reimplement joystick device reading. It launches the proven
`joy_node` from the upstream `joy` package with AutoDrive defaults, so other
packages can depend on `joy_initializer` instead of calling `joy_node` directly.

Run only the joystick input node:

```bash
ros2 launch joy_initializer joy.launch.py
```

Override the Linux joystick device id when needed:

```bash
ros2 launch joy_initializer joy.launch.py device_id:=0
```
