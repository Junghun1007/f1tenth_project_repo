# manual_control

Translates raw joystick input from `/joy` into normalized manual-control topics.

Published topics:

```text
/manual/accelerator       std_msgs/msg/Float32  0.0 to 1.0
/manual/brake             std_msgs/msg/Float32  0.0 to 1.0
/manual/steering          std_msgs/msg/Float32  -1.0 to 1.0
/manual/gear_toggle       std_msgs/msg/Bool
/manual/current_erpm      std_msgs/msg/Int32
/vesc/measured_erpm       std_msgs/msg/Int32  VESC-estimated ERPM
/manual/gear              std_msgs/msg/String
/manual/controller_debug  std_msgs/msg/String  JSON controller state
```

Current mapping:

```text
RT trigger axis -> /manual/accelerator
LT trigger axis -> /manual/brake
left stick X -> /manual/steering
Y button -> /manual/gear_toggle
```

`actuator_commander_node` starts stopped in forward gear. RT increases the
command ERPM, releasing both pedals coasts toward zero, LT brakes more strongly,
and Y toggles forward/reverse while stopped. Current gear and command ERPM are
published on `/manual/gear` and
`/manual/current_erpm`. The VESC node publishes measured ERPM on
`/vesc/measured_erpm` and logs target and measured ERPM together.
