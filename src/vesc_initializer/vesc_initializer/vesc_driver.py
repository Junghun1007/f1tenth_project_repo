#!/usr/bin/env python3

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class VescCommandIds:
    # 현재는 비활성화. 실제 제어 경로는 ERPM과 servo만 사용한다.
    set_duty: int = 5
    set_current: int = 6
    set_current_brake: int = 7

    # 현재 활성화된 명령.
    set_erpm: int = 8
    set_servo_pos: int = 12


@dataclass(frozen=True)
class VescScales:
    # 현재는 비활성화. 나중에 다시 켤 때 설정 구조를 유지하려고 남겨둔다.
    duty: int = 100000
    current: int = 1000
    brake_current: int = 1000

    # 현재 활성화된 명령 스케일.
    servo: int = 1000


class VescDriverError(RuntimeError):
    pass


class VescDriver:
    """VESC 시리얼 포트에 명령 패킷을 쓰는 하위 드라이버.

    이 클래스는 ROS2 의존성을 갖지 않는다. ROS 노드는 토픽을 구독한 뒤
    이 메서드들을 호출하고, 이 파일은 패킷 인코딩과 시리얼 쓰기만 맡는다.
    """

    START_BYTE = 2
    END_BYTE = 3
    MAX_SHORT_PAYLOAD_SIZE = 255

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 0.1,
        write_timeout: float = 2.0,
        startup_delay: float = 0.2,
        command_ids: VescCommandIds | None = None,
        scales: VescScales | None = None,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.startup_delay = startup_delay
        self.command_ids = command_ids or VescCommandIds()
        self.scales = scales or VescScales()
        self._serial_factory = serial_factory
        self._serial: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    def open(self) -> None:
        if self.is_open:
            return

        factory = self._serial_factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise VescDriverError(
                    "pyserial is required to open a VESC serial port."
                ) from exc

            factory = serial.Serial

        try:
            self._serial = factory(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
        except Exception as exc:
            raise VescDriverError(f"Failed to open VESC port {self.port}: {exc}") from exc

        if self.startup_delay > 0.0:
            time.sleep(self.startup_delay)

    def close(self) -> None:
        if self._serial is None:
            return

        try:
            self._serial.close()
        finally:
            self._serial = None

    def __enter__(self) -> "VescDriver":
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def set_duty(self, duty: float) -> None:
        # 현재 duty 명령은 비활성화한다. 모터 속도 제어는 ERPM 명령만 사용한다.
        # 기존 구현은 나중에 다시 켤 수 있도록 주석으로 남겨둔다.
        # duty = self._clamp(duty, -1.0, 1.0)
        # value = int(duty * self.scales.duty)
        # payload = bytes([self.command_ids.set_duty]) + struct.pack(">i", value)
        # self.write_payload(payload)
        raise VescDriverError("Duty 명령은 비활성화되어 있습니다. ERPM 명령을 사용하세요.")

    def set_current(self, current_amps: float) -> None:
        # 현재 current 명령은 비활성화한다. 안전 정책이 정해진 뒤 다시 켠다.
        # 기존 구현은 나중에 다시 켤 수 있도록 주석으로 남겨둔다.
        # value = int(current_amps * self.scales.current)
        # payload = bytes([self.command_ids.set_current]) + struct.pack(">i", value)
        # self.write_payload(payload)
        raise VescDriverError("Current 명령은 비활성화되어 있습니다. ERPM 명령을 사용하세요.")

    def set_brake_current(self, brake_current_amps: float) -> None:
        # 현재 brake current 명령은 비활성화한다.
        # 정지는 우선 ERPM 0 또는 상위 안전 정책에서 처리한다.
        # 기존 구현은 나중에 다시 켤 수 있도록 주석으로 남겨둔다.
        # value = int(brake_current_amps * self.scales.brake_current)
        # payload = bytes([self.command_ids.set_current_brake]) + struct.pack(">i", value)
        # self.write_payload(payload)
        raise VescDriverError(
            "Brake current 명령은 비활성화되어 있습니다. ERPM 명령을 사용하세요."
        )

    def set_erpm(self, erpm: int | float) -> None:
        # VESC set_rpm expects ERPM, not wheel RPM. ERPM is electrical RPM.
        payload = bytes([self.command_ids.set_erpm]) + struct.pack(">i", int(erpm))
        self.write_payload(payload)

    def set_servo_position(self, position: float) -> None:
        position = self._clamp(position, 0.0, 1.0)
        value = int(position * self.scales.servo)
        payload = bytes([self.command_ids.set_servo_pos]) + struct.pack(">h", value)
        self.write_payload(payload)

    def write_payload(self, payload: bytes) -> None:
        self.write_packet(self.make_packet(payload))

    def write_packet(self, packet: bytes) -> None:
        self.open()
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        try:
            self._serial.write(packet)
        except Exception as exc:
            raise VescDriverError(f"Failed to write VESC packet: {exc}") from exc

    @classmethod
    def make_packet(cls, payload: bytes) -> bytes:
        if len(payload) > cls.MAX_SHORT_PAYLOAD_SIZE:
            raise VescDriverError(
                f"Payload too large for short VESC packet: {len(payload)} bytes"
            )

        crc = cls.crc16_xmodem(payload)
        return (
            bytes([cls.START_BYTE, len(payload)])
            + payload
            + struct.pack(">H", crc)
            + bytes([cls.END_BYTE])
        )

    @staticmethod
    def crc16_xmodem(data: bytes) -> int:
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, float(value)))
