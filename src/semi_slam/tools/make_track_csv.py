"""정답 트랙 CSV 생성 (사용자가 준 4 m × 4 m 트랙 그림에서 읽은 치수).

좌표: 트랙 영역 왼쪽 아래 모서리 = 원점, x = 오른쪽, y = 위 (m). 주행 방향은 반시계.
  선 위치 = 테이프 가운데 (차선 인식도 테이프 가운데를 선으로 잡는다)
  실측: 차선 폭 = 테이프 안쪽 끼리 0.64 m, 테이프 두께 0.03 m → 테이프 가운데 간격 0.67 m
  도로 가운데(가장자리에서 0.50 m)는 그림 그대로 두고 두 선을 양쪽으로 0.335 m씩
  바깥 흰 선: 가장자리에서 0.165 m, 코너 반지름 0.62 m (그림의 대각선 50.0 cm에서 구한 코너 중심 유지)
  안쪽 흰 선: 가장자리에서 0.835 m, 코너 반지름 0.174 m (그림의 대각선 124.3 cm에서 구한 코너 중심 유지) 중앙 점선은 그림에 표시만 한 것 → 화면 참고용으로만 저장 (매핑·보정에 쓰지 않음)
  정지선 S: x = 1.97 m, 출발선 T1: x = 2.03 m (아래쪽 도로를 가로지름)

    python3 tools/make_track_csv.py maps/track_4x4.csv
"""
import math
import sys

SIZE = 4.0
LANE_INNER_EDGE, TAPE = 0.64, 0.03
HALF = (LANE_INNER_EDGE + TAPE) / 2                      # 0.335
OUTER_CORNER_CENTER, INNER_CORNER_CENTER = 0.785, 1.009  # 그림에서 구한 코너 중심 (가장자리에서)
OUTER_INSET, INNER_INSET = 0.50 - HALF, 0.50 + HALF
OUTER_R, INNER_R = OUTER_CORNER_CENTER - OUTER_INSET, INNER_CORNER_CENTER - INNER_INSET
STOP_X, START_X = 1.97, 2.03
SPACING = 0.01


def rounded_square(inset, r, spacing=SPACING):
    """반시계 방향 닫힌 둘레 (아래 변 가운데에서 시작)."""
    a, b = inset, SIZE - inset
    pts = []
    def straight(p0, p1):
        n = max(1, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / spacing))
        for k in range(n):
            pts.append((p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n))
    def arc(cx, cy, a0):
        n = max(2, int(r * math.pi / 2 / spacing))
        for k in range(n):
            t = a0 + (math.pi / 2) * k / n
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    straight((SIZE / 2, a), (b - r, a)); arc(b - r, a + r, -math.pi / 2)
    straight((b, a + r), (b, b - r)); arc(b - r, b - r, 0.0)
    straight((b - r, b), (a + r, b)); arc(a + r, b - r, math.pi / 2)
    straight((a, b - r), (a, a + r)); arc(a + r, a + r, math.pi)
    straight((a + r, a), (SIZE / 2, a))
    return pts


def midline(outer, inner):
    out = []
    for (x, y) in outer:
        j = min(inner, key=lambda q: (q[0] - x) ** 2 + (q[1] - y) ** 2)
        out.append(((x + j[0]) / 2, (y + j[1]) / 2))
    return out[::3]


def main(path):
    outer, inner = rounded_square(OUTER_INSET, OUTER_R), rounded_square(INNER_INSET, INNER_R)
    rows = [("outer", p) for p in outer] + [("inner", p) for p in inner] + [("center", p) for p in midline(outer, inner)]
    for name, x in (("stop_S", STOP_X), ("start_T1", START_X)):
        rows += [(name, (x, OUTER_INSET)), (name, (x, INNER_INSET))]
    with open(path, "w") as f:
        f.write("line,x_m,y_m\n")
        for name, (x, y) in rows:
            f.write(f"{name},{x:.4f},{y:.4f}\n")
    print(f"{len(rows)} rows -> {path}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "track_4x4.csv")
