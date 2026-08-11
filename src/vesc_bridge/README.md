# vesc_bridge

`vesc_bridge` connects ROS 2 command and telemetry topics to a VESC over a
serial device. Both the VESC USB CDC port and a USB-to-UART adapter use the
same VESC packet protocol; only the serial device path and matching baudrate
change.

The installed executable is `vesc_bridge_node`. Runtime parameters are kept
in `vehicle_bringup/config/vesc_config.yaml`.

## Topics

- Subscribes: `/vesc/duty`, `/vesc/erpm`, `/vesc/servo_position`
- Publishes: `/vesc/measured_erpm`, `/vesc/connected`

Measured ERPM is selectively requested at 80 Hz by a dedicated UART worker.
Duty/ERPM and servo callbacks only replace the newest pending state, so a slow
telemetry response does not block the ROS executor or replay superseded control
commands. Drive and steering writes are handled before telemetry. The periodic
status log reports requested/actual telemetry Hz, UART round-trip average/max,
and failed queries for on-vehicle verification.

The 80 Hz setting matches the camera's nominal publish rate but does not
phase-lock UART replies to image exposure. Each transaction records its host
monotonic start, finish, and midpoint so a later speed/IMU fusion stage can
align samples by time instead of assuming matching sequence numbers.

The port defaults to the Jetson UART device `/dev/ttyTHS1`. Override the
`port` parameter when using a different serial device. Configure UART for
3.3 V logic, cross TX/RX, connect GND, and do not connect the UART power pin
to the VESC.
