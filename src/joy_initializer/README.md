# joy_initializer

`joy_initializer` is the project-owned wrapper for the ROS `joy` package.

It does not reimplement joystick device reading. It launches the proven
`joy_node` with the original D-input defaults. The project publishes no
joystick feedback commands; it only consumes axes and buttons from `/joy`.

Run only the joystick input node:

```bash
ros2 launch joy_initializer joy.launch.py
```

Override the Linux joystick device number when needed. `/dev/input/js0` maps
to `device_id:=0`.

```bash
ros2 launch joy_initializer joy.launch.py device_id:=0
```
