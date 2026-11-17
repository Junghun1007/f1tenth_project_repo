"""시나리오 정의 (설계 문서 10장)."""
from dataclasses import dataclass
import numpy as np

from .track import Track, stage1_cones, stage2_cones, STAGE1_GAP_DEFAULT, STAGE1_SPACING_DEFAULT

_TR = Track()


@dataclass
class Scenario:
    name: str
    start_s: float
    distance: float
    v0: float
    obstacles: str                 # none | stage1 | stage2 | both
    view: tuple                    # 화면 확대 영역 (xmin, xmax, ymin, ymax)
    stage1_gap: float = STAGE1_GAP_DEFAULT   # 1단계 콘 지점에서 남는 차선 폭
    stage1_spacing: float = STAGE1_SPACING_DEFAULT   # 1단계 묶음 사이 앞뒤 간격
    custom_cones: np.ndarray = None          # 에디터/레이아웃 파일로 지정한 콘 (있으면 우선)

    def cones(self) -> np.ndarray:
        if self.custom_cones is not None:
            return np.asarray(self.custom_cones, float).reshape(-1, 2)
        parts = []
        if self.obstacles in ("stage1", "both"):
            parts.append(stage1_cones(self.stage1_gap, self.stage1_spacing))
        if self.obstacles in ("stage2", "both"):
            parts.append(stage2_cones())
        return np.concatenate(parts) if parts else np.zeros((0, 2))


SCENARIOS = {
    "stage0_lap": Scenario("stage0_lap", _TR.mat_entry_s(30) + 0.5, _TR.length, 0.0, "none",
                           (-0.1, 13.1, -0.1, 4.1)),
    "stage1_cones": Scenario("stage1_cones", 13.0, _TR.mat_entry_s(9) + 1.0 - 13.0, 1.0, "stage1",
                             (7.6, 12.9, 1.8, 4.1)),
    "stage2_guide": Scenario("stage2_guide", 18.3, _TR.mat_entry_s(14) + 1.0 - 18.3, 1.0, "stage2",
                             (2.4, 9.4, 1.6, 4.1)),
    "full_lap": Scenario("full_lap", _TR.mat_entry_s(30) + 0.5, _TR.length, 0.0, "both",
                         (-0.1, 13.1, -0.1, 4.1)),
}

STAGE1_ZONE = (_TR.mat_entry_s(7), _TR.mat_entry_s(9) + 0.6)   # 7번 진입 ~ 9번 초입


def with_layout(base: Scenario, cones) -> Scenario:
    """기존 시나리오 구간에 사용자 콘 배치를 넣는다. 1단계 차선 이탈 지표는 1단계 구간일 때만."""
    import dataclasses
    obstacles = "stage1" if base.obstacles in ("stage1", "both") else "custom"
    return dataclasses.replace(base, custom_cones=np.asarray(cones, float).reshape(-1, 2), obstacles=obstacles)


def load_layout(path):
    import json
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    return np.asarray(data["cones"] if isinstance(data, dict) else data, float).reshape(-1, 2)


def save_layout(path, cones):
    import json
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"cones": [[round(float(x), 3), round(float(y), 3)] for x, y in np.asarray(cones).reshape(-1, 2)]},
                  fh, ensure_ascii=False, indent=1)
