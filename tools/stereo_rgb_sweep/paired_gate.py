# Runs INSIDE the OAK Script node. Both camera frames must have the same sequence.
# Python runner inserts LOW_FPS and POLICY before this script.
import time

left = None
right = None
active = 0
next_capture = None
while True:
    control = node.io['control'].tryGet()
    if control is not None:
        active = int(control.getData()[0])
        next_capture = None
    packet = node.io['left'].tryGet()
    if packet is not None:
        left = packet
    packet = node.io['right'].tryGet()
    if packet is not None:
        right = packet
    if left is None or right is None:
        time.sleep(0.0005)
        continue
    if left.getSequenceNum() < right.getSequenceNum():
        left = None
        continue
    if right.getSequenceNum() < left.getSequenceNum():
        right = None
        continue
    capture = left.getTimestamp().total_seconds()
    rate = 40.0 if POLICY == 'fps_burst' and active else LOW_FPS
    if next_capture is None:
        next_capture = capture
    if capture + 0.0001 >= next_capture:
        branch = 'high' if POLICY == 'resolution_burst' and active else 'low'
        node.io['left_' + branch].send(left)
        node.io['right_' + branch].send(right)
        next_capture += 1.0 / rate
        if capture - next_capture > 1.0 / rate:
            next_capture = capture + 1.0 / rate
    left = None
    right = None
