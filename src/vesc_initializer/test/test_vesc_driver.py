from __future__ import annotations

import unittest

from vesc_initializer.vesc_driver import VescDriver, VescDriverError


class FakeSerial:
    def __init__(self, response: bytes = b"") -> None:
        self.is_open = True
        self.response = bytearray(response)
        self.writes: list[bytes] = []

    def write(self, packet: bytes) -> int:
        self.writes.append(packet)
        return len(packet)

    def read(self, size: int) -> bytes:
        chunk = bytes(self.response[:size])
        del self.response[:size]
        return chunk

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


def make_driver(fake_serial: FakeSerial) -> VescDriver:
    return VescDriver(
        startup_delay=0.0,
        serial_factory=lambda **_kwargs: fake_serial,
    )


class VescDriverTest(unittest.TestCase):
    def test_get_firmware_version_requires_a_valid_vesc_response(self) -> None:
        response = VescDriver.make_packet(bytes([0, 6, 5]))
        fake_serial = FakeSerial(response=response)
        driver = make_driver(fake_serial)

        self.assertEqual(driver.get_firmware_version(), (6, 5))
        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([0]))],
        )

    def test_get_firmware_version_rejects_invalid_crc(self) -> None:
        response = bytearray(VescDriver.make_packet(bytes([0, 6, 5])))
        response[-2] ^= 0xFF
        driver = make_driver(FakeSerial(response=bytes(response)))

        with self.assertRaisesRegex(VescDriverError, "CRC"):
            driver.get_firmware_version()

    def test_get_firmware_version_times_out_without_a_response(self) -> None:
        driver = make_driver(FakeSerial())

        with self.assertRaisesRegex(VescDriverError, "Timed out"):
            driver.get_firmware_version()


if __name__ == "__main__":
    unittest.main()
