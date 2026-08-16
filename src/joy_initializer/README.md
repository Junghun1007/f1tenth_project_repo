# joy_initializer

`joy_initializer` owns the project's SDL joystick input executable.

Unlike the upstream ROS `joy_node`, `joy_input_node` initializes only
`SDL_INIT_JOYSTICK`. It contains no haptic initialization, rumble API call, or
`/joy/set_feedback` subscription. It preserves the raw SDL joystick axes,
buttons, hats, deadzone, and `/joy` message format used by the existing 8BitDo
D-input keymap.

The configured controller is selected by its exact SDL name rather than only
by device index. `Opened joystick input` is printed once for the initial open.
`Joystick disconnected by SDL` followed by `Reopened joystick after a real SDL
disconnect` means that Linux/SDL actually removed and re-added the device; it
is not a periodic reopen performed by this node.

Run only the joystick input node:

```bash
ros2 launch joy_initializer joy.launch.py
```

Override the device name when the SDL-reported name differs. An empty name
falls back to the numeric SDL device index.

```bash
ros2 launch joy_initializer joy.launch.py \
  device_name:="8BitDo Ultimate 2 Wireless Controller for PC"
```
