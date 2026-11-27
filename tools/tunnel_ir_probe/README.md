# tunnel_ir_probe (일회용 측정 도구)

터널에서 IR 차선이 늦게 들어와 차가 멈춘 원인이 **IMU 영상 안정화 보정**인지 확정하기 위한 측정 스크립트.
측정이 끝나면 이 폴더를 통째로 지운다 (`git rm -r tools/tunnel_ir_probe`). 다른 패키지는 건드리지 않으며
`COLCON_IGNORE`로 빌드에서도 빠진다.

## 가설

IMU 보정이 끝나기 전(시작 직후·웹 REFIT 직후, 정지 상태로 약 4 s 필요)에는 터널 IR 프레임이 IMU 보정값을
기다리다 모두 버려진다 → 진입 판정은 났는데 IR 차선이 한참 뒤(보정 완료 뒤)에야 들어온다.

## 시험

터널 없이 웹 "터널 IR 강제"로 진입 뒤 경로(IR 게이트 → 노출 → IMU 동기화 → IR BEV → IR 검출 → 선택기)를 실제로 돌린다.
실내라 IR 차선이 무효여도 상관없다 (도착 여부·시각만 본다).

| 시험 | 동작 | 가설이 맞다면 |
|---|---|---|
| T1 기준 | 보정 완료 → IR ON | IR이 1 s 안에 도착 |
| T2 REFIT 직후 | REFIT → 바로 IR ON | 첫 IR ≈ 보정 완료 시각 |
| T3 REFIT+흔들기 | T2 + 안내에 따라 흔든 뒤 내려놓기 | 흔드는 동안 IR 없음 |
| T4 대조 | 보정 완료 → IR ON + 흔들기 | IR 계속 도착 |

## 실행 (Jetson)

1. 290514c 이후 빌드로 실행 (웹 텔레메트리에 IMU 보정 상태가 있어야 한다). **반드시 `mode:=detect`**:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py tunnel_enabled:=true mode:=detect
```

2. VESC 연결, 차는 바닥에 정지, 조이스틱 수동(안전 MANUAL). 다른 터미널에서:

```bash
source install/setup.bash && python3 tools/tunnel_ir_probe/probe.py --trials 5
```

흔들기 시험까지: `--with-motion` (화면 안내 "지금 차를 손으로 계속 흔드세요" → "내려놓고 가만히 두세요"를 따른다).

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--tests` | `T1,T2` | 실행할 시험 |
| `--with-motion` | 끔 | T3, T4 추가 |
| `--trials` | 5 | 시험마다 반복 수 |
| `--max-wait` | 25 | IR ON 뒤 첫 IR을 기다릴 최대 시간 (s) |
| `--hold` | 3 | 첫 IR 뒤 계속 볼 시간 (s) |
| `--motion-sec` | 8 | 흔드는 시간 (s) |
| `--port` | 8765 | auto_drive 웹 포트 |

## 안전

- `mode:=detect`가 아니면 시작하지 않는다 (자동 주행 명령이 나가지 않는 상태에서만).
- 강제 IR을 켤 때마다 웹과 같은 조건(READY/MANUAL·정지·최신 ERPM·안전 MANUAL)을 직접 확인한다.
- 상태가 DRIVING/ARMING/STOPPING이 되거나, 흔들기 안내 밖에서 속도가 0.05 m/s를 넘거나, 텔레메트리가 2 s 끊기면 중단한다.
- 끝날 때(Ctrl+C·중단 포함) 항상 강제 IR을 끈다.

## 결과 (`tunnel_ir_probe_<시각>/`, 이 폴더를 보내면 됨)

- `summary.txt`: 시험별 지연 중앙값·최대, 판정(지지/불일치/판단불가)
- `trials.csv`: 시험 1회당 한 줄. 시각은 IR ON 기준 초
  - `imu_calibrated_s` 보정 완료, `first_ir_bev_s` 첫 IR BEV, `first_ir_raw_s` 첫 IR 검출 결과,
    `first_ir_valid_s` 첫 유효 IR, `first_ir_fwd_s` 선택기를 지나 auto_drive로 간 첫 IR
  - `imu_wait_expired`, `imu_initialized_log`: 카메라 드라이버 `[IR CAMERA]` 로그 (IR 켠 동안 2 s마다)
- `events.csv`: 상태·터널 상태·보정 상태 변화 원본 (monotonic 시각)
- `rosout.log`: 관련 로그 원문

판정 규칙: T2/T3는 첫 IR이 보정 완료 0.3 s 전보다 빠르면 불일치, 보정 뒤 1 s 안이면 지지.
REFIT이 보정을 다시 시작하지 않은 회차는 판단불가로 따로 센다.
