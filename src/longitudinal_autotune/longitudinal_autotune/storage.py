"""Atomic campaign/checkpoint writes and nonblocking trace logging."""
import json
import os
from pathlib import Path
import queue
import tempfile
import threading
import yaml


def clean(value):
    import math
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(v) for v in value]
    return value


def atomic_write(path, value, yaml_file=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            data = clean(value)
            stream.write(yaml.safe_dump(data, sort_keys=True) if yaml_file else json.dumps(data, indent=2, allow_nan=False))
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class Trace:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.queue = queue.Queue(maxsize=4096)
        self.dropped = 0
        self.failed = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, record):
        try:
            self.queue.put_nowait(clean(record))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        try:
            with self.path.open('x') as stream:
                while True:
                    item = self.queue.get()
                    if item is None:
                        break
                    if isinstance(item, tuple):
                        stream.flush(); os.fsync(stream.fileno()); item[0].set()
                        continue
                    stream.write(json.dumps(item, allow_nan=False)+'\n')
                    stream.flush()
        except Exception:
            self.failed = True

    def barrier(self):
        event = threading.Event()
        try: self.queue.put_nowait((event,))
        except queue.Full: self.dropped += 1
        return event

    def close(self):
        if self.thread.is_alive():
            try: self.queue.put(None, timeout=1.)
            except queue.Full: self.failed = True
            self.thread.join(timeout=5.)
        if self.thread.is_alive():
            self.failed = True
