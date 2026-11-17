"""콘 배치 에디터 (GUI).

    python -m avoidance_sim.editor                    # 1·2단계 규격 배치로 시작
    python -m avoidance_sim.editor --layout my.json   # 저장한 배치 불러오기

마우스
  왼쪽 클릭(빈 곳)   콘 추가 ('묶음' 켜면 차선 안쪽으로 2개 붙여서)
  왼쪽 드래그(콘)     콘 이동 (1 cm 단위)
  오른쪽 클릭(콘)     콘 삭제
키
  ctrl+z 되돌리기, r 실행, s 저장
"""
import argparse
import copy
import math
import numpy as np

from .config import SimParams
from .geometry import square_corners
from .scenarios import SCENARIOS, with_layout, save_layout, load_layout
from .sim import run
from .track import Track, stage1_cones, stage2_cones, CONE_SIZE, STAGE1_GAP_DEFAULT, STAGE1_SPACING_DEFAULT
from . import viewer as V

SECTIONS = {"1단계 구간": "stage1_cones", "2단계 구간": "stage2_guide", "한 바퀴": "full_lap"}
PICK_RADIUS = 0.09


class Editor:
    def __init__(self, cones=None, layout_path="cone_layout.json"):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, RadioButtons, CheckButtons, Slider, TextBox
        V._font()
        self.plt = plt
        self.tr = Track()
        self.cones = (np.concatenate([stage1_cones(), stage2_cones()]) if cones is None
                      else np.asarray(cones, float).reshape(-1, 2)).copy()
        self.history = []
        self.drag = None
        self.layout_path = layout_path
        self.viewers = []

        self.fig = plt.figure(figsize=(15, 7.6), facecolor=V.BG)
        self.fig.canvas.manager.set_window_title("콘 배치 에디터") if self.fig.canvas.manager else None
        self.ax = self.fig.add_axes([0.03, 0.27, 0.94, 0.66])
        ax = self.ax
        ax.set_facecolor(V.BG)
        ax.set_aspect("equal")
        ax.set_xlim(-0.1, 13.1)
        ax.set_ylim(-0.1, 4.1)
        ax.tick_params(colors="#888", labelsize=7)
        V._mats(ax, self.tr)
        V._mat_labels(ax, self.tr)
        from .track import guide_centerline
        g = guide_centerline()
        ax.plot(g[:, 0], g[:, 1], ls=(0, (4, 4)), color="#666", lw=0.8, zorder=1)
        self.section_patch = None
        self.cone_patches = []
        self.status = self.fig.text(0.03, 0.955, "", color="white", fontsize=10)
        self.fig.text(0.55, 0.955, "왼클릭: 추가 · 드래그: 이동 · 우클릭: 삭제 · ctrl+z: 되돌리기 · r: 실행",
                      color="#aaa", fontsize=9)

        # --- 위젯 ---
        def axw(rect):
            a = self.fig.add_axes(rect)
            a.set_facecolor("#22252b")
            return a
        self.w_section = RadioButtons(axw([0.03, 0.03, 0.12, 0.18]), list(SECTIONS), active=0)
        self.w_planner = CheckButtons(axw([0.16, 0.03, 0.11, 0.18]), ["기준안", "제안안", "제안안(차선 기억)"],
                                      [True, True, False])
        self.w_opts = CheckButtons(axw([0.28, 0.03, 0.12, 0.18]), ["노이즈", "카메라 모델", "묶음(2개)"],
                                   [True, True, False])
        for w in (self.w_section, self.w_planner, self.w_opts):
            for lbl in w.labels:
                lbl.set_color("white")
                lbl.set_fontsize(9)
        self.w_gap = Slider(axw([0.45, 0.17, 0.2, 0.03]), "1단계 폭", 0.40, 0.65, valinit=STAGE1_GAP_DEFAULT,
                            valstep=0.01, color="#3d8bfd")
        self.w_gap.label.set_color("white"); self.w_gap.valtext.set_color("white")
        self.w_spacing = Slider(axw([0.45, 0.21, 0.2, 0.03]), "1단계 묶음 간격", 0.45, 0.90,
                                valinit=STAGE1_SPACING_DEFAULT, valstep=0.01, color="#3d8bfd")
        self.w_spacing.label.set_color("white"); self.w_spacing.valtext.set_color("white")
        self.w_seed = TextBox(axw([0.45, 0.115, 0.06, 0.035]), "시드 ", initial="0", color="#22252b",
                              hovercolor="#2b2f36")
        self.w_seed.label.set_color("white"); self.w_seed.text_disp.set_color("white")
        self.w_file = TextBox(axw([0.45, 0.06, 0.16, 0.035]), "파일 ", initial=layout_path, color="#22252b",
                              hovercolor="#2b2f36")
        self.w_file.label.set_color("white"); self.w_file.text_disp.set_color("white")

        buttons = [
            ("실행 ▶  (r)", self.on_run, [0.72, 0.13, 0.12, 0.08], "#1f6f4a"),
            ("1단계 규격 배치", self.on_preset1, [0.72, 0.03, 0.12, 0.08], "#2b2f36"),
            ("2단계 규격 배치", self.on_preset2, [0.85, 0.13, 0.12, 0.08], "#2b2f36"),
            ("모두 지우기", self.on_clear, [0.85, 0.03, 0.12, 0.08], "#5a2b2b"),
            ("저장 (s)", self.on_save, [0.62, 0.055, 0.045, 0.045], "#2b2f36"),
            ("열기", self.on_load, [0.67, 0.055, 0.04, 0.045], "#2b2f36"),
        ]
        self.buttons = []
        for label, cb, rect, color in buttons:
            b = Button(self.fig.add_axes(rect), label, color=color, hovercolor="#3a3f48")
            b.label.set_color("white")
            b.label.set_fontsize(9)
            b.on_clicked(cb)
            self.buttons.append(b)
        self.w_section.on_clicked(lambda _: self.redraw())

        c = self.fig.canvas
        c.mpl_connect("button_press_event", self.on_press)
        c.mpl_connect("motion_notify_event", self.on_motion)
        c.mpl_connect("button_release_event", self.on_release)
        c.mpl_connect("key_press_event", self.on_key)
        self.redraw()

    # ------------------------------------------------------------------
    def section(self):
        return SCENARIOS[SECTIONS[self.w_section.value_selected]]

    def redraw(self):
        for p in self.cone_patches:
            p.remove()
        self.cone_patches = []
        from matplotlib.patches import Polygon, Rectangle
        for c in self.cones:
            p = Polygon(square_corners(c, CONE_SIZE), closed=True, facecolor=V.CONE, edgecolor="#552a00",
                        lw=0.5, zorder=3)
            self.ax.add_patch(p)
            self.cone_patches.append(p)
        if self.section_patch is not None:
            self.section_patch.remove()
        sc = self.section()
        x0, x1, y0, y1 = sc.view
        self.section_patch = Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls="--", lw=1.0,
                                       edgecolor="#36e0c8", zorder=6)
        self.ax.add_patch(self.section_patch)
        sx, sy, _ = self.tr.pose_at(sc.start_s)
        if hasattr(self, "_start_mark"):
            self._start_mark.remove()
        self._start_mark = self.ax.scatter([sx], [sy], marker=">", s=90, color="#36e0c8", zorder=7)
        self.status.set_text(f"콘 {len(self.cones)}개 · 구간: {self.w_section.value_selected} (청록 점선, ▶ 출발)")
        self.fig.canvas.draw_idle()

    def _push(self):
        self.history.append(self.cones.copy())
        self.history = self.history[-50:]

    def _pick(self, x, y):
        if not len(self.cones):
            return None
        d = np.hypot(self.cones[:, 0] - x, self.cones[:, 1] - y)
        i = int(np.argmin(d))
        return i if d[i] <= PICK_RADIUS else None

    # ------------------------------------------------------------------
    def on_press(self, ev):
        if ev.inaxes is not self.ax or ev.xdata is None:
            return
        if self.fig.canvas.toolbar is not None and getattr(self.fig.canvas.toolbar, "mode", ""):
            return            # 확대/이동 도구 사용 중
        x, y = round(ev.xdata, 2), round(ev.ydata, 2)
        i = self._pick(x, y)
        if ev.button == 3 and i is not None:
            self._push()
            self.cones = np.delete(self.cones, i, axis=0)
            self.redraw()
        elif ev.button == 1:
            if i is not None:
                self._push()
                self.drag = (i, x - self.cones[i, 0], y - self.cones[i, 1])
            else:
                self._push()
                self.cones = np.vstack([self.cones, self._new_cones(x, y)])
                self.redraw()

    def _new_cones(self, x, y):
        if not self.w_opts.get_status()[2]:
            return np.array([[x, y]])
        # 묶음: 차선 중심 쪽으로 콘 하나 더 (1단계 규격처럼 가로로 붙임)
        s, l = self.tr.project((x, y))
        sign = -1.0 if l > 0 else 1.0
        p2 = self.tr.point_at(s, l + sign * CONE_SIZE)
        return np.array([[x, y], [round(p2[0], 2), round(p2[1], 2)]])

    def on_motion(self, ev):
        if self.drag is None or ev.inaxes is not self.ax or ev.xdata is None:
            return
        i, ox, oy = self.drag
        self.cones[i] = (round(ev.xdata - ox, 2), round(ev.ydata - oy, 2))
        self.cone_patches[i].set_xy(square_corners(self.cones[i], CONE_SIZE))
        self.fig.canvas.draw_idle()

    def on_release(self, ev):
        if self.drag is not None:
            self.drag = None
            self.redraw()

    def on_key(self, ev):
        if ev.key in ("ctrl+z", "cmd+z") and self.history:
            self.cones = self.history.pop()
            self.redraw()
        elif ev.key == "r":
            self.on_run(None)
        elif ev.key == "s":
            self.on_save(None)

    # ------------------------------------------------------------------
    def on_preset1(self, _):
        self._push()
        keep = self.cones[~self._in_stage1(self.cones)] if len(self.cones) else self.cones
        self.cones = np.vstack([keep.reshape(-1, 2), stage1_cones(self.w_gap.val, self.w_spacing.val)])
        self.redraw()

    def on_preset2(self, _):
        self._push()
        keep = self.cones[~self._in_stage2(self.cones)] if len(self.cones) else self.cones
        self.cones = np.vstack([keep.reshape(-1, 2), stage2_cones()])
        self.redraw()

    @staticmethod
    def _in_stage1(c):
        c = np.asarray(c).reshape(-1, 2)
        return (c[:, 0] > 8.2) & (c[:, 0] < 11.0) & (c[:, 1] > 3.0)   # 묶음 간격 최대 0.90 m까지

    @staticmethod
    def _in_stage2(c):
        c = np.asarray(c).reshape(-1, 2)
        return (c[:, 0] >= 3.9) & (c[:, 0] <= 7.1) & (c[:, 1] > 2.0)

    def on_clear(self, _):
        self._push()
        self.cones = np.zeros((0, 2))
        self.redraw()

    def on_save(self, _):
        path = self.w_file.text.strip() or self.layout_path
        save_layout(path, self.cones)
        self.status.set_text(f"저장했습니다: {path} (콘 {len(self.cones)}개)")
        self.fig.canvas.draw_idle()

    def on_load(self, _):
        path = self.w_file.text.strip() or self.layout_path
        try:
            cones = load_layout(path)
        except OSError as exc:
            self.status.set_text(f"열기 실패: {exc}")
            self.fig.canvas.draw_idle()
            return
        self._push()
        self.cones = cones
        self.redraw()

    # ------------------------------------------------------------------
    def build_runs(self):
        planners = [n for n, on in zip(("baseline", "proposed", "proposed_lane"), self.w_planner.get_status()) if on]
        noise, camera_model, _ = self.w_opts.get_status()
        try:
            seed = int(self.w_seed.text)
        except ValueError:
            seed = 0
        sc = with_layout(self.section(), self.cones)
        params = []
        for _ in planners:
            P = SimParams()
            P.perception.noise = noise
            P.perception.camera_model = camera_model
            params.append(P)
        return sc, planners, params, seed

    def on_run(self, _):
        sc, planners, params, seed = self.build_runs()
        if not planners:
            self.status.set_text("기준안/제안안 중 하나 이상 선택하세요")
            self.fig.canvas.draw_idle()
            return
        self.status.set_text("계산 중… (구간 길이에 따라 수 초~수십 초)")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        results = [run(sc, pl, P, seed=seed) for pl, P in zip(planners, params)]
        summary = " / ".join(f"{r.planner}: 접촉 {r.metrics['contacts']}회, 최소여유 {r.metrics['min_clearance_m']} m"
                             for r in results)
        self.status.set_text(summary)
        print(summary)
        v = V.Viewer(results, f"사용자 배치 · {self.w_section.value_selected} · 시드 {seed}")
        self.viewers.append(v)
        v.start()
        self.fig.canvas.draw_idle()
        return results

    def show(self):
        self.plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser(description="콘 배치 에디터")
    ap.add_argument("--layout", help="불러올 배치 JSON")
    args = ap.parse_args(argv)
    cones = load_layout(args.layout) if args.layout else None
    Editor(cones, args.layout or "cone_layout.json").show()


if __name__ == "__main__":
    main()
