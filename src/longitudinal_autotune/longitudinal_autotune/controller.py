"""Restart the unchanged production controller only while the output gate holds."""
import os
from pathlib import Path
import signal
import subprocess
import time
from .config import PREFIX
from .storage import atomic_write


class Controller:
    def __init__(self, root):
        self.root = Path(root)
        self.child = self.log = None
        self.pending = None
        self.stopping_at = None
        self.generation = 0
        self.started = 0.

    def request(self, parameters):
        self.pending = dict(parameters)
        if self.child and self.child.poll() is None and self.stopping_at is None:
            os.killpg(self.child.pid, signal.SIGINT)
            self.stopping_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.child and self.child.poll() is None:
            if self.stopping_at is not None and now-self.stopping_at > 2.:
                os.killpg(self.child.pid, signal.SIGKILL)
            return False
        if self.log:
            self.log.close(); self.log = None
        if self.pending is None:
            return False
        self.generation += 1
        self.started = now
        p, self.pending = self.pending, None
        p['driving_log_directory'] = str(self.root/f'controller_{self.generation}')
        path = self.root/f'controller_{self.generation}.yaml'
        atomic_write(path, {'/**': {'ros__parameters': p}}, True)
        self.log = (self.root/f'controller_{self.generation}.log').open('x')
        self.child = subprocess.Popen(['ros2', 'run', 'auto_control', 'auto_control_node', '--ros-args',
            '-r', '__ns:='+PREFIX, '-r', f'__node:=autotune_controller_{self.generation}',
            '-r', '/auto/maximum_speed_request:='+PREFIX+'/controller/maximum_speed_request',
            '-r', '/auto/maximum_speed_state:='+PREFIX+'/controller/maximum_speed_state',
            '--params-file', str(path)],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.stopping_at = None
        return True

    def alive(self):
        return self.child is not None and self.child.poll() is None and self.stopping_at is None

    def close(self):
        self.pending = None
        if self.child and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try: self.child.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGKILL); self.child.wait(timeout=1.)
        if self.log: self.log.close(); self.log = None
