from __future__ import annotations

import unittest

from manual_control.erpm_command_profile import (
    ErpmCommandProfile,
    ErpmProfileConfig,
    Gear,
)


def make_profile() -> ErpmCommandProfile:
    return ErpmCommandProfile(
        ErpmProfileConfig(
            forward_max_erpm=8000,
            reverse_max_erpm=5000,
            start_erpm=1000,
            acceleration_erpm_per_sec=1200.0,
            coast_deceleration_erpm_per_sec=600.0,
            brake_erpm_per_sec=5000.0,
            pedal_deadzone=0.03,
        )
    )


class ErpmCommandProfileTest(unittest.TestCase):
    def test_starts_stopped_in_forward_gear(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.gear, Gear.FORWARD)
        self.assertEqual(profile.command_erpm, 0)

    def test_accelerator_starts_at_1000_erpm_then_uses_ramp_rate(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.update(1.0, 0.0, 1.0), 1000)
        self.assertEqual(profile.update(1.0, 0.0, 1.0), 2200)

    def test_accelerator_is_proportional_and_clamped_to_forward_limit(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.update(0.5, 0.0, 1.0), 1000)
        self.assertEqual(profile.update(1.0, 0.0, 100.0), 8000)

    def test_releasing_both_pedals_coasts_toward_zero(self) -> None:
        profile = make_profile()
        profile.update(1.0, 0.0, 1.0)

        self.assertEqual(profile.update(0.0, 0.0, 0.5), 700)
        self.assertEqual(profile.update(0.0, 0.0, 5.0), 0)

    def test_brake_has_priority_and_decreases_erpm(self) -> None:
        profile = make_profile()
        profile.update(1.0, 0.0, 2.0)
        profile.update(1.0, 0.0, 1.0)

        self.assertEqual(profile.update(1.0, 0.5, 0.5), 950)
        self.assertEqual(profile.update(0.0, 1.0, 1.0), 0)

    def test_y_toggle_changes_to_reverse_only_while_stopped(self) -> None:
        profile = make_profile()

        self.assertTrue(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)
        profile.update(1.0, 0.0, 1.0)
        self.assertFalse(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)

    def test_reverse_accelerator_is_clamped_to_negative_5000_erpm(self) -> None:
        profile = make_profile()
        profile.toggle_gear()

        self.assertEqual(profile.update(1.0, 0.0, 1.0), -1000)
        self.assertEqual(profile.update(1.0, 0.0, 100.0), -5000)

    def test_brake_moves_reverse_erpm_toward_zero(self) -> None:
        profile = make_profile()
        profile.toggle_gear()
        profile.update(1.0, 0.0, 2.0)
        profile.update(1.0, 0.0, 1.0)

        self.assertEqual(profile.update(0.0, 0.5, 0.5), -950)
        self.assertEqual(profile.update(0.0, 1.0, 1.0), 0)

    def test_coasting_moves_reverse_erpm_toward_zero(self) -> None:
        profile = make_profile()
        profile.toggle_gear()
        profile.update(1.0, 0.0, 1.0)

        self.assertEqual(profile.update(0.0, 0.0, 0.5), -700)

    def test_pedal_deadzone_ignores_small_input(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.update(0.029, 0.0, 1.0), 0)


if __name__ == "__main__":
    unittest.main()
