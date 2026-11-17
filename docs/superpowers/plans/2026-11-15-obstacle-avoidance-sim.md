# 라바콘 회피 시뮬레이터 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 대회 트랙에서 기준/제안 회피 알고리즘을 비교하는 Python 2D 시뮬레이터 + GUI

**Architecture:** 순수 함수 모듈(트랙·기하·차량·인지·제어·플래너)을 시뮬 루프가 묶고, 결과 프레임 기록을 GUI(matplotlib)와 배치 지표가 소비한다.

**Tech Stack:** Python 3.10+, numpy, matplotlib, pyyaml, pytest

**Spec:** `docs/superpowers/specs/2026-11-15-obstacle-avoidance-sim-design.md`

## Global Constraints

- 코드 위치 `tools/avoidance_sim/`, 패키지 `avoidance_sim`, 테스트 `tools/avoidance_sim/tests/`
- 차량 좌표는 앞차축 기준 x 앞, y 왼쪽 (auto_control과 동일)
- 차량: 휠베이스 0.33, 전장 0.50, 전폭 0.30, 앞 오버행 0.05, 뒤 오버행 0.12, 최대 조향 30°
- BEV 시야 x∈[0,3.0], y∈[−0.6,0.6]
- Stanley gain 1.3, softening 0.3, heading lookahead 0.05; 속도 1.0~1.8, 횡가속 1.6, 감속 0.5, 장애물 상한 1.0
- 기본 설정은 노이즈·가림·지연 ON

## Review Focus

- 경로 점열이 짧거나 비어 있을 때(차선 끝, 시야 밖) → 마지막 유효 경로 유지, 예외로 죽지 않음
- 콘이 정확히 차선 중앙/시야 경계에 있을 때 → 좌우 선택 결정적(오른쪽 회피)
- 트랙 위치 투영이 코너 경계(직선↔원호)에서 끊기지 않음
- 시드 고정 시 같은 결과 재현
- GUI 없는 환경(Agg)에서도 GIF/MP4 저장 동작

---

### Task 1: 트랙 기하 (`track.py`, `geometry.py`)
- Produces: `Track.centerline(n)->(N,2)`, `Track.project(xy)->(s,l)`, `Track.pose_at(s)->(x,y,psi)`, `stage1_cones()`, `stage2_cones()` → `(N,2)` 중심, `rect_corners`, `rects_overlap`(SAT)
- Tests: 한 바퀴 29.14±0.01, 1단계 남는 폭 0.47, 2단계 간격 0.186±0.002, s=1.5 오른쪽 줄 y=3.16

### Task 2: 차량 모델 (`vehicle.py`)
- Produces: `Vehicle.step(delta_cmd, v_target, dt)`, `front_axle_pose()`, `footprint()`
- Tests: 직진 1 s 이동 거리, 일정 조향 회전 반지름 = L/tanδ

### Task 3: 인지 (`perception.py`)
- Produces: `observe_lane(track, pose, rng)`, `observe_cones(cones, pose, rng, cfg)` (가림 포함), `Odometry`
- Tests: 일렬 콘에서 뒤 콘 미관측, 시야 밖 미관측

### Task 4: 제어 (`control.py`)
- Produces: `stanley(path, v, cfg)`, `plan_speed(path, cfg, obstacle_limit)`
- Tests: 직선 경로 오프셋에서 조향 부호, 곡률 2(반지름 0.5) → 1.0 하한

### Task 5: 플래너 (`planners.py`)
- Produces: `BaselinePlanner.plan(obs)`, `ProposedPlanner.plan(obs)` → `PlanResult(path, obstacles_active, debug)`
- Tests: 메모리 콘이 차량 이동 후 위치 유지 및 뒤로 지나가면 삭제, DP가 차선 막힘 시 차선 밖 선택

### Task 6: 시뮬 루프·지표·시나리오 (`sim.py`, `scenarios.py`, `metrics`)
- Produces: `run(scenario, planner, seed) -> Result(frames, metrics)`
- Tests: stage0 접촉 0 완주

### Task 7: GUI·배치 (`viewer.py`, `run.py`, `batch.py`)
- `python -m avoidance_sim.run --scenario ... --planner ... [--save out.gif]`, `python -m avoidance_sim.batch ...`
- 확인: GIF 저장 성공
