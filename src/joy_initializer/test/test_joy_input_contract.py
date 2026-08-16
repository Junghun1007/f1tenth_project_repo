from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_input_node_has_no_force_feedback_api() -> None:
    source = (PACKAGE_ROOT / "src" / "joy_input_node.cpp").read_text(
        encoding="utf-8"
    )
    forbidden_symbols = (
        "SDL_INIT_HAPTIC",
        "SDL_Haptic",
        "JoyFeedback",
        "set_feedback",
        "GameControllerRumble",
        "HapticRumble",
    )

    for symbol in forbidden_symbols:
        assert symbol not in source


def test_launch_uses_project_input_node_and_exact_controller_name() -> None:
    launch_source = (PACKAGE_ROOT / "launch" / "joy.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'package="joy_initializer"' in launch_source
    assert 'executable="joy_input_node"' in launch_source
    assert 'package="joy"' not in launch_source
    assert "8BitDo Ultimate 2 Wireless Controller for PC" in launch_source
