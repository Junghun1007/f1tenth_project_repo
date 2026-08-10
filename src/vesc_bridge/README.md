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

For a CH341T adapter on Linux, the device normally appears as
`/dev/ttyUSB0`. Prefer a stable `/dev/serial/by-id/...` path when available.
Configure the adapter for UART and 3.3 V logic, cross TX/RX, connect GND, and
do not connect the adapter power pin to the VESC.
