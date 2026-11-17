"""기구학 자전거 모델 (설계 문서 6장). 상태 기준점 = 뒷차축 중심."""
import math
import numpy as np

from .config import VehicleParams
from .geometry import rect_corners


class Vehicle:
    def __init__(self, params: VehicleParams, x, y, psi, v=0.0):
        self.p = params
        self.x, self.y, self.psi = float(x), float(y), float(psi)
        self.v = float(v)
        self.delta = 0.0
        self.yaw_rate = 0.0

    @classmethod
    def from_front_axle(cls, params: VehicleParams, fx, fy, psi, v=0.0):
        return cls(params, fx - params.wheelbase * math.cos(psi),
                   fy - params.wheelbase * math.sin(psi), psi, v)

    def step(self, delta_cmd: float, v_target: float, dt: float):
        p = self.p
        delta_cmd = max(-p.max_steer, min(p.max_steer, delta_cmd))
        max_step = math.radians(p.steer_rate_deg_s) * dt
        self.delta += max(-max_step, min(max_step, delta_cmd - self.delta))
        accel = (v_target - self.v) / p.speed_tau
        accel = max(-p.max_decel, min(p.max_accel, accel))
        self.v = max(0.0, self.v + accel * dt)
        self.yaw_rate = self.v / p.wheelbase * math.tan(self.delta)
        self.x += self.v * math.cos(self.psi) * dt
        self.y += self.v * math.sin(self.psi) * dt
        self.psi += self.yaw_rate * dt

    def front_axle(self):
        L = self.p.wheelbase
        return np.array([self.x + L * math.cos(self.psi), self.y + L * math.sin(self.psi)])

    def body_center(self):
        off = self.p.length / 2 - self.p.rear_overhang
        return np.array([self.x + off * math.cos(self.psi), self.y + off * math.sin(self.psi)])

    def footprint(self):
        c = self.body_center()
        return rect_corners(c[0], c[1], self.psi, self.p.length, self.p.width)


def body_center_from_front_axle(params: VehicleParams):
    """앞차축 기준 차체 중심의 x (음수)."""
    return params.length / 2 - params.rear_overhang - params.wheelbase
