# vehicle_test_drive

Directly publishes a steering-servo sweep and ERPM ramps to the VESC command
topics. Do not run `manual_control` command publishers at the same time.

Sequence:

1. Servo position: 0.5 down to 0.1, then 0.1 up to 0.9 in 0.1 steps.
2. ERPM: 0 to 5000 over 3 seconds.
3. Hold 5000 for 0.25 seconds, then ERPM 0 for 0.5 seconds.
4. ERPM: 0 to -5000 over 3 seconds, then hold for 0.25 seconds.
5. Stop at ERPM 0 and return the servo to 0.5.

Run:

```bash
ros2 run vehicle_test_drive vehicle_test_drive
```
