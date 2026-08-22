# manual_control

Reads SDL-standard Bluetooth controller input from `/joy` and sends manual
actuator commands.

Published topics:

```text
/manual/current_duty      std_msgs/msg/Float32
/vesc/duty                std_msgs/msg/Float32  -1.0 to 1.0
/vesc/measured_erpm       std_msgs/msg/Int32  VESC-estimated ERPM
/manual/gear              std_msgs/msg/String
```

Current mapping:

```text
RT axis 5 (0.0 released, 1.0 pressed) -> acceleration
LT axis 4 (0.0 released, 1.0 pressed) -> deceleration toward duty 0
left stick X axis 0 -> steering angle
RB button 10 -> forward/reverse toggle while stopped
```

`actuator_commander_node` starts stopped in forward gear. RT increases the
command duty, LT reduces duty toward zero, and RB toggles forward/reverse while
stopped. With the configured `immediate_stop_on_accelerator_release: false`,
releasing RT follows the coast deceleration rate. Current gear and command duty are
published on `/manual/gear` and
`/manual/current_duty`. The VESC node publishes measured ERPM on
`/vesc/measured_erpm` and logs target duty and measured ERPM together.

`actuator_commander_node` reads all four controls directly from the same `/joy`
frame. The former intermediate converter and four manual input topics are not
used by `manual_drive.launch.py`. RB changes gear only on its rising edge, so
holding the button does not repeat the change.

The manual controller runs at 80 Hz. Its configured duty limits and ramps are:

Controller state and VESC command topics use `KEEP_LAST(1)` best-effort QoS.
Holding RT or a steering input therefore replaces the pending state instead of
accumulating old commands. When `/joy` stops, the 0.30-second watchdog sends
duty 0 and centers steering. When serial I/O is temporarily delayed, the VESC
node processes only the newest waiting duty/ERPM/servo command.

```yaml
forward_max_duty: 0.10
reverse_max_duty: 0.08
start_duty: 0.05
reverse_start_duty: 0.05
acceleration_duty_per_sec: 0.03
coast_deceleration_duty_per_sec: 0.04
brake_duty_per_sec: 0.08
immediate_stop_on_accelerator_release: false
```

`coast_deceleration_duty_per_sec` is used only when
`immediate_stop_on_accelerator_release` is `false`. LT does not request VESC
brake current; it ramps the duty command toward zero.
