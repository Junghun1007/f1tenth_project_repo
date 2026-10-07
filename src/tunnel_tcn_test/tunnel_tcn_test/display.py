"""ROS-independent preview rendering and mono8 message decoding."""
import cv2
import numpy as np


def mono_image(message):
    if message.encoding != 'mono8' or message.width <= 0 or message.height <= 0:
        raise ValueError('Expected mono8 stereo image')
    if message.step < message.width or len(message.data) < message.step * message.height:
        raise ValueError('Invalid image stride/buffer length')
    return np.frombuffer(message.data, np.uint8, count=message.step * message.height).reshape(
        message.height, message.step)[:, :message.width].copy()


def footer(gray, probabilities, state, last_event, status, fps, infer_ms, total_ms, dropped, settings):
    picture = cv2.cvtColor(cv2.resize(gray, (960, 600), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
    canvas = cv2.copyMakeBorder(picture, 0, 160, 0, 0, cv2.BORDER_CONSTANT, value=(22, 24, 28))
    lines = [
        f'PARTIAL INT8 | {status} | state: {state.upper()}',
        'NONE %.3f   ENTRY %.3f   EXIT %.3f' % tuple(probabilities),
        f'Last event: {last_event} | source FPS {fps:.1f} | missed frames ~{dropped}',
        f'TRT+transfer {infer_ms:.2f} ms | frame processing {total_ms:.2f} ms | R: new drive/reset  Q: quit',
        f'Camera: {settings}  (edit in OAK IR live controls)',
    ]
    for j, line in enumerate(lines):
        color = (100, 230, 120) if j == 0 and status == 'LIVE' else (225, 225, 225)
        cv2.putText(canvas, line, (14, 624 + 29 * j), cv2.FONT_HERSHEY_SIMPLEX, .54, color, 1, cv2.LINE_AA)
    return canvas
