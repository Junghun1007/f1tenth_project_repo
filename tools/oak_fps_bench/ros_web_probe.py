#!/usr/bin/env python3
"""auto_drive 웹 텔레메트리에서 단계별 연산(ms/FPS)을 일정 시간 모아 평균을 낸다.

auto_drive.launch.py가 떠 있는 상태에서 실행한다. 미리보기 클라이언트로 등록해야
stages(cam_depth, lane_latency, yolo_* ...)가 텔레메트리에 실린다.
"""
import argparse
import json
import statistics
import threading
import time
import urllib.request


def post(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=2) as r:
        return r.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    base = f"http://127.0.0.1:{a.port}"
    client = "oak_fps_bench"
    stop = threading.Event()

    def keepalive():
        while not stop.is_set():
            try:
                post(base, "/api/view", {"client": client, "preview": True, "bev": False, "camera": False})
                post(base, "/api/heartbeat", {"client": client})
            except Exception:
                pass
            stop.wait(0.5)

    threading.Thread(target=keepalive, daemon=True).start()
    time.sleep(1.5)
    stages, yolo_fps, usage, frames = {}, [], {"cpu": [], "proc_cpu": [], "gpu": []}, 0
    state = "-"
    t_end = time.time() + a.seconds
    try:
        with urllib.request.urlopen(base + "/api/events", timeout=5) as r:
            for raw in r:
                if time.time() > t_end:
                    break
                line = raw.decode(errors="replace").strip()
                if not line.startswith("data: "):
                    continue
                try:
                    j = json.loads(line[6:])
                except ValueError:
                    continue
                state = j.get("state", state)
                if "stages" not in j:
                    continue
                frames += 1
                for s in j["stages"]:
                    d = stages.setdefault(s["name"], {"ms": [], "fps": []})
                    d["ms"].append(float(s["ms"]))
                    d["fps"].append(float(s["fps"]))
                y = j.get("yolo") or {}
                if "fps" in y:
                    yolo_fps.append(float(y["fps"]))
                for k in usage:
                    v = (j.get("usage") or {}).get(k)
                    if v is not None:
                        usage[k].append(float(v))
    except Exception as e:  # 웹이 아직 안 떴거나 끊김
        print(f"web probe error: {e}")
    finally:
        stop.set()

    def avg(v):
        return f"{statistics.mean(v):.2f}" if v else "-"

    lines = [f"web telemetry frames={frames} state={state} seconds={a.seconds:.0f}",
             "stage                  avg_ms   avg_fps   (fps = 1초당 처리 횟수)"]
    for name, d in stages.items():
        lines.append(f"{name:<20} {avg(d['ms']):>8} {avg(d['fps']):>9}")
    lines.append(f"yolo_worker_fps        {avg(yolo_fps)}")
    lines.append(f"host cpu% avg={avg(usage['cpu'])} process cpu% avg={avg(usage['proc_cpu'])} gpu% avg={avg(usage['gpu'])}")
    text = "\n".join(lines)
    print(text)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    main()
