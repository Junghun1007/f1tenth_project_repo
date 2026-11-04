from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(REPO/'src/auto_control'))

@pytest.fixture
def document():
    from longitudinal_autotune.config import load
    return load(ROOT/'config/experiment.yaml')

@pytest.fixture
def baseline(document):
    from longitudinal_autotune.config import ros_parameters, control_parameters
    return control_parameters(ros_parameters(REPO/'src/auto_control/config/auto_control.yaml','auto_control'),document)
