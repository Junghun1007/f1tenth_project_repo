#!/usr/bin/env bash
# OAK RGB·스테레오 수신 FPS/지연 진단을 한 번에 돌린다 (Jetson에서 실행).
#   1) 시스템 정보 (Jetson 전원 모드·클럭·USB 연결 상태·온도)
#   2) 카메라 단독 벤치마크: 설정을 하나씩 바꿔 가며 FPS·순번 누락·지연·장치 CPU 측정 (ROS 없음)
#   3) (--ros) 실제 auto_drive launch를 detect 모드로 띄워 camera_driver [PERF] 로그와 웹 단계별 연산 수집
# 결과: ~/oak_fps_bench_<시각>/summary.txt (이 파일 내용을 그대로 붙여 주면 됨) + 같은 이름의 .tar.gz
#
# 사용 예:
#   bash tools/oak_fps_bench/run_all.sh                      # 1+2 (약 6~7분)
#   bash tools/oak_fps_bench/run_all.sh --ros --ws ~/1113ML/f1tenth_project_repo
#   bash tools/oak_fps_bench/run_all.sh --ros --ros-tunnel --ws <워크스페이스>   # 터널 켠 실제 구성도
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SECONDS_PER_CONFIG=12
RUN_BENCH=1
RUN_ROS=0
RUN_ROS_TUNNEL=0
ROS_SECONDS=30
ROS_STARTUP_MAX=240
WS=""
ONLY=""
FORCE=0
while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) SECONDS_PER_CONFIG="$2"; shift ;;
    --only) ONLY="$2"; shift ;;
    --skip-bench) RUN_BENCH=0 ;;
    --ros) RUN_ROS=1 ;;
    --ros-tunnel) RUN_ROS_TUNNEL=1 ;;
    --ros-seconds) ROS_SECONDS="$2"; shift ;;
    --ws) WS="$2"; shift ;;
    --force) FORCE=1 ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
  shift
done
if [ -z "$WS" ]; then WS="$(cd "$HERE/../.." && pwd)"; fi

# ------------------------------------------------------------------ 다른 OAK 사용자 확인 (로그 tee를 띄우기 전에)
others=$(pgrep -af "component_container|ros2 launch|camera_driver|bev_processor|/build/oak_fps_bench" | grep -v -E "pgrep|run_all\.sh" || true)
if [ -n "$others" ]; then
  echo "OAK를 쓰는 다른 프로세스가 떠 있습니다 (측정이 틀어집니다):"
  echo "$others"
  if [ "$FORCE" -ne 1 ]; then echo "끄고 다시 실행하거나 --force를 붙이세요."; exit 1; fi
fi

OUT="$HOME/oak_fps_bench_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
exec > >(tee -a "$OUT/run.log") 2>&1
echo "결과 폴더: $OUT"

sudo_n() { sudo -n "$@" 2>/dev/null; }
section() { echo; echo "##### $*"; }

# ------------------------------------------------------------------ 1) 시스템 정보
{
  section "date / kernel / board"
  date; uname -a
  cat /etc/nv_tegra_release 2>/dev/null
  [ -r /proc/device-tree/model ] && { tr -d '\0' < /proc/device-tree/model; echo; }
  section "power mode / clocks"
  (nvpmodel -q 2>/dev/null || sudo_n nvpmodel -q) || echo "nvpmodel: 권한 없음/없음"
  sudo_n jetson_clocks --show || echo "jetson_clocks --show: sudo 비밀번호 필요 (생략)"
  for c in /sys/devices/system/cpu/cpu[0-9]*; do
    [ -r "$c/cpufreq/scaling_cur_freq" ] && echo "$(basename "$c") gov=$(cat "$c/cpufreq/scaling_governor") cur=$(cat "$c/cpufreq/scaling_cur_freq") max=$(cat "$c/cpufreq/cpuinfo_max_freq") online=$(cat "$c/online" 2>/dev/null || echo 1)"
  done
  for f in /sys/devices/platform/*gpu*/devfreq/*/cur_freq /sys/class/devfreq/*gpu*/cur_freq; do [ -r "$f" ] && echo "gpu cur_freq $f = $(cat "$f")"; done
  section "load / memory / temperature"
  uptime; free -h
  for z in /sys/class/thermal/thermal_zone*; do [ -r "$z/temp" ] && echo "$(cat "$z/type") $(( $(cat "$z/temp") / 1000 ))C"; done
  section "USB: OAK (Movidius 03e7) 연결 속도·전원 관리"
  for d in /sys/bus/usb/devices/*; do
    [ -r "$d/idVendor" ] || continue
    if [ "$(cat "$d/idVendor")" = "03e7" ]; then
      echo "$(basename "$d") id=03e7:$(cat "$d/idProduct") speed=$(cat "$d/speed")Mbps version=$(cat "$d/version" | tr -d ' ') product=$(cat "$d/product" 2>/dev/null) power/control=$(cat "$d/power/control" 2>/dev/null) autosuspend_ms=$(cat "$d/power/autosuspend_delay_ms" 2>/dev/null)"
      p="$d"; while p=$(dirname "$p"); [ "$p" != "/sys/bus/usb/devices" ] && [ -r "$p/speed" ]; do echo "  parent $(basename "$p") speed=$(cat "$p/speed")Mbps product=$(cat "$p/product" 2>/dev/null)"; done
    fi
  done
  echo "usbcore.autosuspend=$(cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null)"
  lsusb 2>/dev/null
  lsusb -t 2>/dev/null
  section "dmesg (USB/xhci 최근)"
  (dmesg 2>/dev/null || sudo_n dmesg) | grep -iE "usb|xhci|03e7|movidius" | tail -n 40 || echo "dmesg: 권한 없음"
} > "$OUT/sysinfo.txt" 2>&1
echo "[1/3] 시스템 정보 저장: $OUT/sysinfo.txt"

TEGRA_PID=""
if command -v tegrastats >/dev/null 2>&1; then
  tegrastats --interval 1000 > "$OUT/tegrastats.log" 2>&1 &
  TEGRA_PID=$!
fi
cleanup() { [ -n "$TEGRA_PID" ] && kill "$TEGRA_PID" 2>/dev/null; }
trap cleanup EXIT

# ------------------------------------------------------------------ 2) 카메라 단독 벤치마크
if [ "$RUN_BENCH" -eq 1 ]; then
  echo "[2/3] 벤치마크 빌드"
  if cmake -S "$HERE" -B "$OUT/build" -DCMAKE_BUILD_TYPE=Release > "$OUT/build.log" 2>&1 &&
     cmake --build "$OUT/build" -j4 >> "$OUT/build.log" 2>&1; then
    ldd "$OUT/build/oak_fps_bench" | grep -i depthai >> "$OUT/sysinfo.txt"
    echo "[2/3] 벤치마크 실행 (설정당 약 $((SECONDS_PER_CONFIG + 8))초, 차를 세워 두고 카메라 앞 조명은 평소대로)"
    args=(--seconds "$SECONDS_PER_CONFIG" --csv "$OUT/bench.csv")
    [ -n "$ONLY" ] && args+=(--only "$ONLY")
    LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}" "$OUT/build/oak_fps_bench" "${args[@]}" 2>&1 | tee "$OUT/bench.txt"
  else
    echo "빌드 실패: $OUT/build.log 마지막 줄:"; tail -n 30 "$OUT/build.log"
  fi
fi

# ------------------------------------------------------------------ 3) 실제 launch (detect 모드)
run_ros() {
  local tag="$1" tunnel="$2"
  echo "[3/3] 실제 launch 측정: $tag (tunnel_enabled=$tunnel, mode=detect: 자동 명령 없음)"
  set +u   # ROS setup 스크립트는 정의 안 된 변수를 참조한다
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  if ! source "$WS/install/setup.bash"; then echo "워크스페이스를 찾을 수 없음: $WS (--ws로 지정)"; set -u; return; fi
  set -u
  local cam_yaml
  cam_yaml="$(ros2 pkg prefix camera_driver)/share/camera_driver/config/camera_config.yaml"
  python3 - "$cam_yaml" "$OUT/camera_perf.yaml" <<'EOF'
import sys, yaml
src, dst = sys.argv[1], sys.argv[2]
d = yaml.safe_load(open(src)) or {}
d.setdefault("camera_driver", {}).setdefault("ros__parameters", {})["perf_log_enabled"] = True
yaml.safe_dump(d, open(dst, "w"), allow_unicode=True, sort_keys=False)
EOF
  local log="$OUT/ros_launch_$tag.log"
  setsid ros2 launch vehicle_bringup auto_drive.launch.py mode:=detect camera_params_file:="$OUT/camera_perf.yaml" \
    tunnel_enabled:="$tunnel" > "$log" 2>&1 &
  local lpid=$!
  local waited=0
  # 카메라 [PERF] 로그와 웹 서버가 모두 뜰 때까지 (첫 실행은 TensorRT 엔진 생성으로 수 분)
  until grep -q "\[PERF\]\[CAMERA\]" "$log" 2>/dev/null && curl -s -o /dev/null "http://127.0.0.1:8765/api/params"; do
    sleep 5; waited=$((waited + 5))
    if ! kill -0 "$lpid" 2>/dev/null; then echo "launch가 종료됨. $log 확인"; break; fi
    if [ "$waited" -ge "$ROS_STARTUP_MAX" ]; then echo "시작 대기 시간 초과 (${ROS_STARTUP_MAX}s)"; break; fi
  done
  sleep 15   # 시작 측정·모델 로드 직후 과도 구간을 넘긴다
  local mark
  mark=$(wc -l < "$log")
  python3 "$HERE/ros_web_probe.py" --seconds "$ROS_SECONDS" --out "$OUT/ros_web_$tag.txt" &
  local probe=$!
  local cpid
  cpid=$(pgrep -f "bev_processor_container" | head -n 1)
  if [ -n "$cpid" ]; then
    sleep 5
    top -H -b -n 2 -d 5 -p "$cpid" | awk 'BEGIN{n=0} /^top -/{n++} n==2' | head -n 45 > "$OUT/ros_threads_$tag.txt"
  fi
  wait "$probe"
  tail -n +"$mark" "$log" | grep -E "\[PERF\]\[CAMERA\]|dropped=|Depth input|capture_fps" | tail -n 40 > "$OUT/ros_camera_$tag.txt"
  grep -E "OAK: full sensor|USB=|Shared RGB/depth|Tunnel IR undistort|Depth input: first|First frame|Options:" "$log" | head -n 20 >> "$OUT/ros_camera_$tag.txt"
  grep -iE "error|exception|fail" "$log" | grep -v "errors(capture" | tail -n 20 > "$OUT/ros_errors_$tag.txt"
  kill -INT -- -"$lpid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 "$lpid" 2>/dev/null || break; sleep 1; done
  kill -TERM -- -"$lpid" 2>/dev/null; sleep 2; kill -KILL -- -"$lpid" 2>/dev/null
  sleep 3
}
if [ "$RUN_ROS" -eq 1 ]; then
  run_ros "tunnel_off" false
  [ "$RUN_ROS_TUNNEL" -eq 1 ] && run_ros "tunnel_on" true
fi

# ------------------------------------------------------------------ 요약
cleanup
{
  echo "===== oak_fps_bench summary ($(date '+%F %T'))"
  cat "$OUT/sysinfo.txt"
  echo
  if [ -f "$OUT/bench.txt" ]; then grep -vE "^\[bench\] running" "$OUT/bench.txt"
  elif [ -f "$OUT/build.log" ]; then echo "----- 벤치마크 빌드 실패 (build.log 끝부분)"; tail -n 40 "$OUT/build.log"; fi
  for f in "$OUT"/ros_camera_*.txt "$OUT"/ros_web_*.txt "$OUT"/ros_threads_*.txt "$OUT"/ros_errors_*.txt; do
    [ -f "$f" ] || continue
    echo; echo "----- $(basename "$f")"; cat "$f"
  done
  if [ -s "$OUT/tegrastats.log" ]; then echo; echo "----- tegrastats (처음·중간·끝)"; n=$(wc -l < "$OUT/tegrastats.log"); sed -n "1p;$((n / 2))p;${n}p" "$OUT/tegrastats.log"; fi
} > "$OUT/summary.txt"
tar --exclude="$(basename "$OUT")/build" -czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
echo
echo "완료. 이 파일 내용을 그대로 붙여 주세요: $OUT/summary.txt"
echo "전체 로그 묶음: $OUT.tar.gz"
