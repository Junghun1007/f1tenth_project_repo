"""Restart the unchanged production controller only while the output gate holds."""
import os
from pathlib import Path
import signal
import subprocess
import time
from .config import PREFIX
from .storage import atomic_write


def controller_executable():
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix('auto_control'))/'lib/auto_control/auto_control_node'
    if not path.is_file():
        raise RuntimeError('Installed auto_control executable not found: '+str(path))
    return str(path)


class Controller:
    def __init__(self, root):
        self.root = Path(root)
        self.child = self.log = None
        self.pending = None
        self.stopping_at = None
        self.generation = 0
        self.started = 0.

    def _signal_group(self, sig):
        if self.child is None:
            return
        try:
            os.killpg(self.child.pid, sig)
        except ProcessLookupError:
            pass  # The process group already exited normally.

    def request(self, parameters):
        self.pending = dict(parameters)
        if self.child and self.child.poll() is None and self.stopping_at is None:
            self._signal_group(signal.SIGINT)
            self.stopping_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.child and self.child.poll() is None:
            if self.stopping_at is not None and now-self.stopping_at > 2.:
                self._signal_group(signal.SIGKILL)
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
        # Own the actual executable, not a ros2-run wrapper which can exit first.
        self.child = subprocess.Popen([controller_executable(), '--ros-args',
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
        try:
            if self.child is not None:
                # Signal the group even when the recorded leader already exited;
                # older ros2-run sessions could otherwise leave its child alive.
                self._signal_group(signal.SIGINT)
                if self.child.poll() is None:
                    try:
                        self.child.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        self._signal_group(signal.SIGKILL)
                        self.child.wait(timeout=1.)
                self._signal_group(signal.SIGKILL)
        finally:
            self.child = None
            if self.log:
                self.log.close(); self.log = None
