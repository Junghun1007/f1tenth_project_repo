# manual_control

Translates raw joystick input from `/joy` into normalized manual-control topics.

Published topics:

```text
/manual/throttle  std_msgs/msg/Float32  0.0 to 1.0
/manual/steering  std_msgs/msg/Float32  -1.0 to 1.0
/manual/controller_debug  std_msgs/msg/String  JSON controller state
```

Current mapping:

```text
RT trigger axis -> /manual/throttle
left stick X -> /manual/steering
```
