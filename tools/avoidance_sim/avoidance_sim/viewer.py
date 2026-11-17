"""matplotlib GUI (설계 문서 12장).

왼쪽: 트랙 위에서 보기 / 오른쪽: 앞차축 기준 차량 시점 (앞이 위쪽)
키: 스페이스 일시정지, ←/→ 한 프레임 이동(일시정지 중), ↑/↓ 재생 속도
"""
import math
import numpy as np
import matplotlib
from matplotlib.patches import Polygon, Rectangle
from matplotlib.lines import Line2D

from .geometry import to_world, to_local, square_corners
from .track import Track, LANE_HALF_TAPE, guide_centerline
from .scenarios import SCENARIOS

BG = "#16181d"
MAT = "#000000"
GRID = "#3a3d45"
TAPE = "#f2f2f2"
CONE = "#ff8a1f"
CAR = "#3d8bfd"
HIT = "#ff3b3b"
PATH = "#36e0c8"
DET = "#ffe14d"
MEM = "#e75cff"
FOV = "#7ad3ff"


def _mats(ax, tr: Track):
    # 트랙 매트 30장 + 보조 A~C
    cells = [(x, 0) for x in range(13)] + [(x, 3) for x in range(13)] + [(0, 1), (0, 2), (12, 1), (12, 2)]
    cells += [(4, 2), (5, 2), (6, 2)]
    for cx, cy in cells:
        ax.add_patch(Rectangle((cx, cy), 1, 1, facecolor=MAT, edgecolor=GRID, lw=0.6, zorder=0))
    for x, y, label in [(5.5, 2.5, "보조 B"), (6.5, 2.5, "보조 A"), (4.5, 2.5, "보조 C")]:
        ax.text(x, y - 0.35, label, color="#666", ha="center", va="center", fontsize=7, zorder=1)
    for l in (LANE_HALF_TAPE, -LANE_HALF_TAPE):
        _, pts = tr.sample(0, tr.length, 0.02, l)
        ax.plot(pts[:, 0], pts[:, 1], color=TAPE, lw=1.4, zorder=1, solid_capstyle="butt")


def _mat_labels(ax, tr: Track):
    for m in list(range(21, 31)) + [1] + [3, 4] + list(range(6, 17)) + [18, 19]:
        x, y = tr.point_at(tr.mat_entry_s(m) + 0.5, 0.0)
        ax.text(x, y, str(m), color="#555", fontsize=7, ha="center", va="center", zorder=1)


def _font():
    for name in ("NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "Malgun Gothic", "AppleGothic"):
        try:
            from matplotlib import font_manager
            font_manager.findfont(name, fallback_to_default=False)
            matplotlib.rcParams["font.family"] = name
            return
        except Exception:
            continue


class Viewer:
    def __init__(self, results, title=None):
        """results: Result 하나 또는 여러 개(나란히 비교)."""
        import matplotlib.pyplot as plt
        _font()
        matplotlib.rcParams["axes.unicode_minus"] = False
        self.plt = plt
        self.results = results if isinstance(results, (list, tuple)) else [results]
        self.tr = Track()
        n = len(self.results)
        self.fig = plt.figure(figsize=(15, min(6.0 * n, 13.5)), facecolor=BG)   # 3개 비교도 노트북 화면에 들어가게
        gs = self.fig.add_gridspec(n, 2, width_ratios=[2.3, 1], left=0.03, right=0.99, top=0.93 - 0.02 * (n - 1),
                                   bottom=0.04, wspace=0.08, hspace=0.18)
        self.panels = [self._make_panel(r, gs[k, 0], gs[k, 1]) for k, r in enumerate(self.results)]
        self.title = self.fig.suptitle(title or "", color="white", fontsize=12)
        self.n_frames = max(len(r.frames) for r in self.results)
        self.i = 0
        self.paused = False
        self.step = 1

    # ------------------------------------------------------------------
    def _make_panel(self, res, gs_world, gs_local):
        sc = SCENARIOS.get(res.scenario, SCENARIOS["full_lap"])
        axw = self.fig.add_subplot(gs_world)
        axw.set_facecolor(BG)
        axw.set_aspect("equal")
        view = getattr(res, "view", None) or sc.view
        axw.set_xlim(view[0], view[1])
        axw.set_ylim(view[2], view[3])
        axw.tick_params(colors="#888", labelsize=7)
        for s in axw.spines.values():
            s.set_color("#444")
        _mats(axw, self.tr)
        _mat_labels(axw, self.tr)
        if sc.obstacles in ("stage2", "both") or getattr(res, "show_guide", False):
            g = guide_centerline()
            axw.plot(g[:, 0], g[:, 1], ls=(0, (4, 4)), color="#888", lw=0.8, zorder=1)
        from matplotlib.collections import PolyCollection
        axw.add_collection(PolyCollection([square_corners(c, 0.13) for c in res.cones], facecolors=CONE,
                                          edgecolors="#552a00", linewidths=0.5, zorder=3))
        hit_cones = PolyCollection([], facecolors=HIT, edgecolors="white", linewidths=0.6, zorder=3.5)
        axw.add_collection(hit_cones)
        trail, = axw.plot([], [], color=CAR, lw=1.0, alpha=0.6, zorder=2)
        path_w, = axw.plot([], [], color=PATH, lw=1.6, zorder=4)
        det_w = axw.scatter([], [], marker="x", s=22, color=DET, zorder=6, linewidths=1.2)
        mem_w = axw.scatter([], [], marker="o", s=40, facecolors="none", edgecolors=MEM, zorder=6, linewidths=1.2)
        car_w = Polygon(np.zeros((4, 2)), closed=True, facecolor=CAR, edgecolor="white", lw=0.8, alpha=0.9, zorder=5)
        axw.add_patch(car_w)
        fov_w = Polygon(np.zeros((4, 2)), closed=True, facecolor=FOV, alpha=0.10, edgecolor=FOV, lw=0.6, zorder=2)
        axw.add_patch(fov_w)
        label = axw.text(0.01, 0.98, "", transform=axw.transAxes, color="white", fontsize=9, va="top", zorder=10,
                         bbox=dict(facecolor="#000", alpha=0.6, edgecolor="none"))

        sub = gs_local.subgridspec(2, 1, height_ratios=[1, 2.1], hspace=0.22)
        axc = self.fig.add_subplot(sub[0])
        axc.set_facecolor("#23262d")
        axc.set_xticks([]); axc.set_yticks([])
        axc.set_title("카메라 영상 (가림·YOLO 박스)", color="#ccc", fontsize=9)
        from .config import CameraParams
        cpar = CameraParams()
        W, H = int(cpar.image_width * cpar.raster_scale), int(cpar.image_height * cpar.raster_scale)
        cam_img = axc.imshow(np.zeros((H, W, 3)), extent=(0, W, H, 0), interpolation="nearest")
        roi_y = H * (1 - cpar.roi_height_percent / 100)
        axc.axhline(roi_y, color="#ffd400", lw=0.7, ls="--")
        axc.set_xlim(0, W); axc.set_ylim(H, 0)
        cam_boxes, = axc.plot([], [], color=DET, lw=1.0)
        cam_ghost, = axc.plot([], [], color=HIT, lw=1.2)
        axl = self.fig.add_subplot(sub[1])
        axl.set_facecolor(MAT)
        axl.set_aspect("equal")
        axl.set_xlim(-1.1, 1.1)
        axl.set_ylim(-0.75, 3.25)
        axl.tick_params(colors="#888", labelsize=7)
        axl.set_title("차량 시점 (앞차축 기준, 앞=위)", color="#ccc", fontsize=9)
        axl.add_patch(Rectangle((-0.6, 0.0), 1.2, 3.0, facecolor=FOV, alpha=0.08, edgecolor=FOV, lw=0.8))
        axl.add_patch(Polygon(self._car_local(res), closed=True, facecolor=CAR, edgecolor="white", lw=0.8, zorder=5))
        true_l = axl.scatter([], [], marker="s", s=30, facecolors="none", edgecolors=CONE, linewidths=0.8, zorder=3)
        tapes_l = axl.scatter([], [], s=2, color=TAPE, zorder=2)
        ref_l, = axl.plot([], [], color="#aaa", lw=0.8, ls="--", zorder=3)
        path_l, = axl.plot([], [], color=PATH, lw=1.8, zorder=4)
        det_l = axl.scatter([], [], marker="x", s=40, color=DET, zorder=6, linewidths=1.4)
        mem_l = axl.scatter([], [], marker="o", s=70, facecolors="none", edgecolors=MEM, zorder=6, linewidths=1.4)
        axl.legend(handles=[
            Line2D([], [], marker="s", ls="", mfc="none", mec=CONE, label="실제 콘"),
            Line2D([], [], marker="x", ls="", color=DET, label="이번 관측"),
            Line2D([], [], marker="o", ls="", mfc="none", mec=MEM, label="기억 중"),
            Line2D([], [], color=PATH, label="계획 경로"),
            Line2D([], [], color="#aaa", ls="--", label="차선 중심 추정")],
            loc="lower right", fontsize=7, facecolor="#111", edgecolor="#333", labelcolor="white")
        return dict(res=res, axw=axw, hit_cones=hit_cones, trail=trail, path_w=path_w, det_w=det_w, mem_w=mem_w,
                    car_w=car_w, fov_w=fov_w, label=label, true_l=true_l, tapes_l=tapes_l, ref_l=ref_l,
                    path_l=path_l, det_l=det_l, mem_l=mem_l, axc=axc, cam_img=cam_img, cam_boxes=cam_boxes,
                    cam_ghost=cam_ghost, cam_wh=(W, H))

    DYNAMIC = ("hit_cones", "trail", "path_w", "det_w", "mem_w", "car_w", "fov_w", "label", "true_l", "tapes_l",
               "ref_l", "path_l", "det_l", "mem_l", "cam_img", "cam_boxes", "cam_ghost")

    def artists(self):
        return [p[k] for p in self.panels for k in self.DYNAMIC]

    @staticmethod
    def _car_local(res):
        from .config import VehicleParams
        vp = VehicleParams()
        x0, x1 = -vp.front_axle_to_rear_bumper, vp.front_overhang
        hw = vp.width / 2
        return np.array([[-hw, x0], [hw, x0], [hw, x1], [-hw, x1]])

    # ------------------------------------------------------------------
    def draw(self, i):
        for pnl in self.panels:
            res = pnl["res"]
            if not res.frames:
                continue
            f = res.frames[min(i, len(res.frames) - 1)]
            hit = bool(f.contacts)
            pnl["car_w"].set_xy(f.footprint)
            pnl["car_w"].set_facecolor(HIT if hit else CAR)
            pnl["hit_cones"].set_verts([square_corners(res.cones[k], 0.13) for k in f.contacts])
            k = min(i, len(res.trail) - 1)
            pnl["trail"].set_data(res.trail[:k + 1, 0], res.trail[:k + 1, 1])
            _set_line(pnl["path_w"], f.path_world)
            _set_scatter(pnl["det_w"], f.detections_world)
            _set_scatter(pnl["mem_w"], f.memory_world)
            x, y, psi = f.pose
            from .config import VehicleParams
            L = VehicleParams().wheelbase
            fa = (x + L * math.cos(psi), y + L * math.sin(psi))
            pnl["fov_w"].set_xy(to_world(np.array([[0, 0.6], [3, 0.6], [3, -0.6], [0, -0.6]]), fa, psi))
            m = res.metrics
            contacts_so_far = sum(1 for c in m.get("contact_detail", []) if c["t"] <= f.t + 1e-6)
            pnl["label"].set_text(
                f"[{res.planner}] t={f.t:5.2f}s  v={f.v:4.2f} (목표 {f.v_target:4.2f}) m/s  "
                f"조향={math.degrees(f.delta):+5.1f}°\n접촉 {contacts_so_far}회  {f.status}")
            # 차량 시점: (x 앞, y 왼쪽) → 화면 (−y, x)
            tl = to_local(res.cones, fa, psi) if len(res.cones) else np.zeros((0, 2))
            _set_scatter(pnl["true_l"], _scr(tl))
            tapes = np.vstack([a for a in f.lane_local if len(a)]) if any(len(a) for a in f.lane_local) else np.zeros((0, 2))
            _set_scatter(pnl["tapes_l"], _scr(tapes))
            _set_line(pnl["ref_l"], _scr(f.reference_local))
            _set_line(pnl["path_l"], _scr(f.path_local))
            _set_scatter(pnl["det_l"], _scr(f.detections_local))
            _set_scatter(pnl["mem_l"], _scr(f.memory_local))
            self._draw_camera(pnl, f)
        return self.artists()

    def _draw_camera(self, pnl, f):
        W, H = pnl["cam_wh"]
        img = np.empty((H, W, 3))
        img[:] = (0.16, 0.17, 0.19)
        horizon = 0
        img[:, :] = (0.09, 0.09, 0.10)
        if f.cam_image is None:
            pnl["cam_img"].set_data(img)
            pnl["cam_boxes"].set_data([], [])
            pnl["cam_ghost"].set_data([], [])
            return
        own = f.cam_image
        mask = own >= 0
        # 콘마다 약간 다른 주황색 (겹침 경계가 보이게)
        shade = 0.75 + 0.25 * ((own % 3) / 2.0)
        img[mask] = np.stack([shade[mask], 0.45 * shade[mask], 0.08 * shade[mask]], axis=1)
        pnl["cam_img"].set_data(img)
        normal, ghost = [], []
        for idx, (u0, v0, u1, v1), src in (f.cam_boxes or []):
            (ghost if src != idx else normal).append([(u0, v0), (u1, v0), (u1, v1), (u0, v1), (u0, v0),
                                                      (np.nan, np.nan)])
        for line, boxes in ((pnl["cam_boxes"], normal), (pnl["cam_ghost"], ghost)):
            pts = np.array(boxes, float).reshape(-1, 2) if boxes else np.zeros((0, 2))
            line.set_data(pts[:, 0], pts[:, 1])

    # ------------------------------------------------------------------
    def save(self, out_path, fps=15, stride=2, dpi=80):
        from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
        idx = list(range(0, self.n_frames, stride)) + [self.n_frames - 1] * int(fps * 1.5)
        anim = FuncAnimation(self.fig, lambda k: self.draw(k), frames=idx, blit=False)
        writer = PillowWriter(fps=fps) if str(out_path).endswith(".gif") else FFMpegWriter(fps=fps, bitrate=2400)
        anim.save(out_path, writer=writer, dpi=dpi)

    def snapshot(self, out_path, t, dpi=90):
        res = self.results[0]
        i = int(np.argmin([abs(f.t - t) for f in res.frames]))
        self.draw(i)
        self.fig.savefig(out_path, dpi=dpi, facecolor=BG)

    def start(self):
        """이미 GUI 루프가 도는 상태(에디터)에서 새 창으로 재생."""
        self.show(block=False)

    def show(self, block=True):
        """실시간 속도로 재생. 바뀌는 그림만 다시 그린다(blit)."""
        import time
        from matplotlib.animation import FuncAnimation
        self.speed = 1.0
        self._t0 = time.perf_counter()
        self._i0 = 0

        def restart_clock():
            self._t0 = time.perf_counter()
            self._i0 = self.i

        def on_key(ev):
            if ev.key == " ":
                self.paused = not self.paused
                restart_clock()
            elif ev.key in ("right", "left"):
                self.paused = True
                self.i = max(0, min(self.n_frames - 1, self.i + (1 if ev.key == "right" else -1)))
            elif ev.key == "up":
                self.speed = min(8.0, self.speed * 2)
                restart_clock()
            elif ev.key == "down":
                self.speed = max(0.125, self.speed / 2)
                restart_clock()

        def tick(_):
            if not self.paused:
                # 녹화는 30 Hz → 경과 시간 기준으로 프레임 선택 (그리기가 느려도 실시간 유지)
                k = self._i0 + int((time.perf_counter() - self._t0) * 30.0 * self.speed)
                if k >= self.n_frames:
                    k = 0
                    self._t0, self._i0 = time.perf_counter(), 0
                self.i = k
            return self.draw(self.i)

        for a in self.artists():
            a.set_animated(True)
        self.fig.canvas.mpl_connect("key_press_event", on_key)
        self.fig.text(0.99, 0.005, "스페이스: 일시정지 · ←/→: 프레임 · ↑/↓: 재생 속도", color="#888", fontsize=8,
                      ha="right")
        self._anim = FuncAnimation(self.fig, tick, interval=15, blit=True, cache_frame_data=False)
        if block:
            self.plt.show()
        else:
            self.fig.show()


def _scr(pts):
    pts = np.asarray(pts).reshape(-1, 2)
    return np.stack([-pts[:, 1], pts[:, 0]], axis=1) if len(pts) else pts


def _set_line(line, pts):
    pts = np.asarray(pts).reshape(-1, 2)
    line.set_data(pts[:, 0], pts[:, 1])


def _set_scatter(sc, pts):
    pts = np.asarray(pts).reshape(-1, 2)
    sc.set_offsets(pts if len(pts) else np.zeros((0, 2)))
