#!/usr/bin/env python3
"""녹화 BEV로 차선 검출·경로 생성을 다시 돌려 보는 웹뷰 (표준 라이브러리만 사용).

  python3 src/auto_control/tools/lane_path_lab/server.py --data <세션 폴더들의 상위 폴더>

세션 폴더 = rgb_bev/ (120×300 BEV), rgb/ (원본, 선택) 를 가진 폴더.
처음 실행 때 모델 추론 결과(logit)를 --cache 에 저장하고, 이후 파라미터 변경은 경로 생성만 다시 돈다.
"""
import argparse
import json
import re
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
PKG = HERE.parent.parent              # src/auto_control
SRC = PKG.parent                      # src
REPO = SRC.parent

# 웹에 노출할 그룹 (정지선·오버레이·좌우 추적처럼 경로에 안 쓰이거나 시간축이 필요한 값은 제외).
# 코너·평활화는 새 경로 계획으로 대체할 예정이라 숨기고, "기존 경로" 비교용으로 차량 현재값을 그대로 넣는다.
GROUPS = [
    ("BEV_LANE_DETECTION_SETTINGS", "검출", {"left_mask_threshold", "right_mask_threshold", "stop_line_mask_threshold", "bridge_centerline"}, False),
    ("BEV_LANE_CONNECTION_SETTINGS", "차선 연결", None, False),
    ("BEV_LANE_CENTERLINE_SETTINGS", "중앙선", None, False),
    ("BEV_LANE_CORNER_SETTINGS", "코너 바깥 차선", None, True),
    ("BEV_LANE_SMOOTHING_SETTINGS", "경로 평활화", None, True),
]
ITEM = re.compile(r'X\((\w+),\s*(\w+),\s*(\w+),\s*"([^"]*)",\s*"([^"]*)",\s*([-\d.]+),\s*([-\d.]+),\s*([\w.\-]+),\s*([\w.\-]+)\)')


def num(text):
    return {"true": 1.0, "false": 0.0}.get(text, None) if text in ("true", "false") else float(text)


def load_schema(tuning_hpp, yaml_path):
    text = Path(tuning_hpp).read_text()
    car = {}
    for line in Path(yaml_path).read_text().splitlines():
        m = re.match(r"\s*lane\.(\w+):\s*([-\d.]+)", line)
        if m:
            car[m.group(1)] = float(m.group(2))
    groups = []
    for macro, title, only, hidden in GROUPS:
        body = text.split(f"#define {macro}(X)", 1)[1].split("\n\n", 1)[0]
        items = []
        for t, key, _member, label, unit, lo, hi, rgb, _ir in ITEM.findall(body):
            if only is not None and key not in only:
                continue
            default = car.get(key, num(rgb))
            items.append({"key": key, "type": t, "label": label, "unit": unit, "min": float(lo), "max": float(hi),
                          "default": default, "header_default": num(rgb)})
        groups.append({"title": title, "items": items, "hidden": hidden})
    return groups


class Engine:
    def __init__(self, binary):
        self.proc = subprocess.Popen([binary, "serve"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lock = threading.Lock()

    def run(self, logit_path, params):
        line = str(logit_path) + "".join(f" {k}={v}" for k, v in params.items()) + "\n"
        with self.lock:
            self.proc.stdin.write(line)
            self.proc.stdin.flush()
            return self.proc.stdout.readline()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="세션 폴더들이 들어 있는 폴더")
    ap.add_argument("--cache", default=str(REPO / "build/lane_path_lab/cache"))
    ap.add_argument("--bin", default=str(REPO / "build/lane_path_lab/lane_path_lab"))
    ap.add_argument("--model", default=str(PKG / "models/lane_c3_120x300.onnx"))
    ap.add_argument("--yaml", default=str(PKG / "config/auto_drive.yaml"))
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()

    data, cache = Path(a.data), Path(a.cache)
    sessions = {}
    for bev in sorted(data.glob("**/rgb_bev")):
        name = bev.parent.name
        out = cache / name
        frames = sorted(p.stem for p in bev.glob("*.jpg"))
        if not out.exists() or len(list(out.glob("*.bin"))) < len(frames):
            print(f"[추론] {name}: {len(frames)}프레임 → {out}")
            subprocess.run([a.bin, "infer", a.model, str(bev), str(out)], check=True, stdout=subprocess.DEVNULL)
        sessions[name] = {"dir": bev.parent, "frames": frames, "has_rgb": (bev.parent / "rgb").is_dir()}
    if not sessions:
        raise SystemExit(f"{data} 아래에 rgb_bev 폴더가 없습니다")
    full = load_schema(SRC / "bev_handoff/include/bev_handoff/lane_tuning.hpp", a.yaml)
    schema = [g for g in full if not g["hidden"]]
    hidden = {i["key"]: i["default"] for g in full if g["hidden"] for i in g["items"]}
    engine = Engine(a.bin)
    model_name = Path(a.model).name

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, body, ctype="application/json; charset=utf-8", code=200):
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def params(self, q):
            keys = {i["key"] for g in schema for i in g["items"]}
            return {**hidden, **{k: float(v[0]) for k, v in q.items() if k in keys}}

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path in ("/", "/index.html"):
                return self.send((HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
            if u.path == "/planner.js":
                return self.send((HERE / "planner.js").read_bytes(), "text/javascript; charset=utf-8")
            if u.path == "/api/meta":
                return self.send(json.dumps({"model": model_name, "schema": schema, "sessions": {
                    k: {"frames": v["frames"], "has_rgb": v["has_rgb"]} for k, v in sessions.items()}}, ensure_ascii=False))
            m = re.fullmatch(r"/img/([\w.-]+)/(rgb|rgb_bev)/(\w+)\.jpg", u.path)
            if m and m.group(1) in sessions:
                f = sessions[m.group(1)]["dir"] / m.group(2) / f"{m.group(3)}.jpg"
                if f.is_file():
                    return self.send(f.read_bytes(), "image/jpeg")
            if u.path in ("/api/run", "/api/batch"):
                s = q.get("session", [""])[0]
                if s not in sessions:
                    return self.send('{"error":"세션 없음"}', code=404)
                p = self.params(q)
                if u.path == "/api/run":
                    fr = q.get("frame", [""])[0]
                    if fr not in sessions[s]["frames"]:
                        return self.send('{"error":"프레임 없음"}', code=404)
                    return self.send(engine.run(cache / s / f"{fr}.bin", p))
                out = []
                for fr in sessions[s]["frames"]:
                    r = json.loads(engine.run(cache / s / f"{fr}.bin", p))
                    out.append({"frame": fr, "error": r.get("error"), "centerline": r.get("centerline", []),
                                "reference": r.get("reference", []), "observed": r.get("observed", [[], []]),
                                "state": r.get("state", 0)})
                return self.send(json.dumps(out))
            self.send('{"error":"not found"}', code=404)

    print(f"http://127.0.0.1:{a.port}  (세션 {len(sessions)}개, 모델 {model_name})")
    ThreadingHTTPServer(("127.0.0.1", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
