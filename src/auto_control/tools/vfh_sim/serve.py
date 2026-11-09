"""Serve the existing course editor with the real VFH planner on localhost."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import tempfile
from threading import Lock

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]


def build_planner(binary):
    subprocess.run([
        "c++", "-std=c++17", "-O3", "-DNDEBUG",
        "-I" + str(HERE / "policy_include"),
        "-I" + str(SRC / "roi_vfh_test/include"),
        "-I" + str(SRC / "auto_control/include"),
        str(HERE / "native_planner.cpp"),
        str(SRC / "auto_control/src/control_core.cpp"),
        "-o", str(binary),
    ], check=True)


def page():
    fragment = (HERE / "view.html").read_text()
    scripts = {
        "CORE": (HERE / "core.js").read_text() + "\n" + (HERE / "native.js").read_text(),
        "UI": (HERE / "ui.js").read_text(),
    }
    for key, script in scripts.items():
        fragment = fragment.replace(f"/*__SIM_{key}__*/", script.replace("</script", "<\\/script"))
    return ('<!doctype html><html lang="ko"><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>ROS VFH+ 로컬 시뮬레이션</title>'
            '<style>:root{--foreground:#202632;--background:#fff;--muted:#aab3bd;'
            '--muted-foreground:#526070;--border:#cad2dd;--viz-series-1:#146ed0;'
            '--viz-series-2:#d04435;--viz-series-3:#dd8a24;--destructive:#af1624}'
            'body{font:15px system-ui;margin:24px}button,input,select{font:inherit}'
            'button{padding:5px 9px}td,th{padding:4px 10px;text-align:left}</style>'
            '<h1>ROS VFH+ 로컬 시뮬레이션</h1>' + fragment + '</html>').encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    document = page()
    with tempfile.TemporaryDirectory(prefix="vfh-sim-") as temp:
        binary = Path(temp) / "planner"
        build_planner(binary)
        with subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE) as worker:
            worker_lock = Lock()

            class Handler(BaseHTTPRequestHandler):
                def reply(self, status, kind, data):
                    self.send_response(status)
                    self.send_header("Content-Type", kind)
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)

                def do_GET(self):
                    if self.path == "/":
                        self.reply(200, "text/html; charset=utf-8", document)
                    else:
                        self.send_error(404)

                def do_POST(self):
                    if self.path != "/plan":
                        self.send_error(404)
                        return
                    size = self.headers.get("Content-Length", "")
                    if not size.isdecimal() or not 0 < int(size) <= 65536:
                        self.send_error(400)
                        return
                    payload = self.rfile.read(int(size))
                    if b"\n" in payload or b"\r" in payload:
                        self.send_error(400)
                        return
                    try:
                        with worker_lock:
                            worker.stdin.write(payload + b"\n")
                            worker.stdin.flush()
                            result = worker.stdout.readline()
                        if not result:
                            raise BrokenPipeError
                    except OSError:
                        self.send_error(503, "Native planner unavailable")
                        return
                    self.reply(200, "application/json", result)

                def log_message(self, format, *args):
                    if self.path != "/plan":
                        super().log_message(format, *args)

            with ThreadingHTTPServer(("127.0.0.1", args.port), Handler) as server:
                print(f"http://127.0.0.1:{args.port}/", flush=True)
                try:
                    server.serve_forever()
                except KeyboardInterrupt:
                    pass
                finally:
                    worker.stdin.close()


if __name__ == "__main__":
    main()
