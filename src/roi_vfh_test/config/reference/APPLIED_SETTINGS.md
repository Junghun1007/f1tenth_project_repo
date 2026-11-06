# 첨부 YAML 적용 범위

이 디렉터리의 YAML은 사용자 첨부 설정을 기준으로 한다. roi_lidar_test.yaml은 후속 요청의 2026-11-06 16:32 스크린샷 기본값으로 갱신했으며, line_detactor_test.yaml의 centerline_corner_outward_offset_m은 후속 요청대로 0.0m로 변경했다. 나머지 7개 YAML은 첨부 원본을 보관한다. launch 기본 경로는 설치된 패키지 내부이므로 다른 컴퓨터의 절대 경로에 의존하지 않는다.

| 파일 | VFH 실행에 적용하는 내용 |
|---|---|
| auto_control_test.yaml | 차량 조향·서보·ERPM 환산, Stanley, 입력 유효성, 구동 PID/FF·브레이크·복구, 속도 상한, 로그 |
| obstacles_test.yaml | 차폭·후방 검사 길이·패딩·불확실성, 최대 곡률·회피 횡가속·감속도·표본 간격·계획 주기 |
| bev_config_test.yaml | 카메라 위치·차량 좌표, 시작 지면/자세 측정의 전체 measurement 설정, 차선 BEV 형상 |
| camera_config_test.yaml | 공용 RGB 크기(1280×800), CAM_A/CROP/왜곡 보정 조건, IMU 400Hz·큐 80·최대 배치 5 |
| line_detactor_test.yaml | 전체 차선 인식 모델·후처리·중앙선 설정 |
| roi_lidar_test.yaml | depth 60FPS, ROI/지면/스캔/클러스터/프리뷰, terrain 폭 1.5m·전방 4m |
| centerline_preview_test.yaml | 별도 중앙선 프리뷰 프로필. 원본 보관만 하며 전체 line_detactor 설정을 덮지 않음 |
| depth_lidar_test.yaml | 별도 depth 파이프라인. 원본 보관만 함 |
| traffic_light_test.yaml | 이번 범위에서 제외한 신호등 파이프라인. 원본 보관만 함 |

## 우선순위와 의도적인 차이

- ROI 패키지 기본값에 첨부 ROI 값을 덮는다. 첨부에 없는 최신 RANSAC·최소 장애물 높이·빈 셀 보정 옵션은 현재 기본값을 유지한다.
- 카메라 위치는 BEV의 X=-0.09m가 우선한다. 시작 측정도 BEV 프로필의 ROI 456×228, 아래 80px, 프리뷰 ON을 따른다. 측정 완료 후 측정 창은 닫힌다.
- 카메라는 ROI 하나가 소유한다. 별도 camera_driver/BEV/depth 노드를 추가 실행하지 않는다. RGB 15FPS·depth 60FPS는 ROI 설정을 쓰므로 standalone camera의 80FPS를 복사하지 않는다. 별도 영상 안정화 기능은 이식하지 않는다.
- line_detactor에는 전체 첨부 YAML을 전달한다. 입력 토픽·공용 BEV 사용·출력 활성화·BEV 치수만 공용 ROI 파이프라인에 맞춘다. 차선 BEV는 1.2×3m/120×300으로 terrain 크기와 별개다.
- VFH 기본 목표 속도는 사용자 선택대로 최신 사진대로 0.8m/s를 사용한다. GUI에서 auto_control의 실제 maximum_speed_mps=1.8까지 선택할 수 있다. 곡률·검사 길이·정지 거리 제한이 최종 속도를 낮출 수 있다. 기존 1.0m/s 최소 속도는 적용하지 않는다.
- 앞차축부터 앞범퍼 0.05m, 후방 검사 길이 0.25m, 폭 0.30m를 사용한다. 최신 사진의 패딩 0.03m와 불확실성 0.01m를 분리 적용하고 셀과 차체의 실제 교차를 검사한다.
- 차선 추종은 공유 Stanley 함수(게인 1.3, soften 0.3, 접선 lookahead 0.05m), 시간 보정된 조향 필터 0.4와 240°/s를 사용한다. 차선 최소 점수 8개, 최소 길이 0.12m, 간격 0.15m, 기하 창 0.16m도 동일하다.
- 계획 20Hz/액추에이터 100Hz, ERPM 수명 0.15초를 적용한다. 후속 사용자 선택으로 VFH 차선/terrain 촬영 수명과 차선 수신 수명은 0.30초로 변경했다. 입력 유실 후 정상 입력·유효 경로가 0.50초 유지되면 자동 재출발하며, Stop은 이를 취소한다. 차량 좌표로 변환된 ROI IMU는 Z축 yaw를 사용한다.
- 구동은 기존 auto_control의 공통 PID·브레이크·서보 함수를 사용한다. 신호등·정지선·전체 코너 상태기와 별도 성능측정 모드는 실행하지 않는다. 전기 브레이크 OFF 설정도 비상 정지 제동을 해제하지 않는다.
- CSV는 첨부대로 driving_logs/30Hz 설정으로 비동기 기록한다. 파일 메타데이터에 적용 파라미터와 참조 프로필을 남긴다.
- Start 수동 게이트, 출발 후 파라미터 잠금, 차선 우선/VFH+ 회피, 회색 미관측 통과 허용, terrain memory 없음, 일반 회피 차선 이탈 기본값은 후속 요청대로 최신 사진대로 0.40m이며, 출발 전 설정 범위는 0~1.0m다.
- 계획 격자는 ROI 프리뷰와 동일한 장애물 표본 임계값을 사용한다. 표본 수가 부족한 한 점을 계획 전용 빨간 셀로 승격하지 않는다. 해당 셀은 회색이며 바닥 보정 대상에서는 계속 제외된다.

이번 반영에서는 빌드·테스트를 실행하지 않았다. 참고용 테스트 기대값만 새 프로필에 맞췄다.

VFH 실행에서는 중복 ROI RGB BEV창만 숨긴다(preview.show_bev_window=false). 뎁스 선택/ROI 파라미터 창과 센서·차선 출력은 유지한다.

IMU는 camera_driver와 같은 calibrated gyro 및 보정 회전을 사용하되, ROI에서는 시작 측정으로 차량 좌표까지 변환한다. 따라서 /camera/imu의 optical Y축 대신 /roi_lidar/imu의 차량 Z축을 yaw로 사용한다. ROI의 IMU 큐 수신을 depth/RANSAC/고립점 처리와 분리된 전용 스레드로 변경했다. ROS IMU 송수신 큐는 200개로 제한하며, 실제 측정 timestamp와 0.10초 IMU 만료 검사는 유지한다. 장치 재시작·예외 시 전용 스레드를 먼저 종료하고 장치를 해제한다.

VFH launch는 vesc_bridge_node를 기본으로 함께 실행하며 vehicle_bringup/config/vesc_config.yaml을 재사용한다.
포트·namespace·5개 VESC 토픽을 VFH/실측속도 설정과 일치시킨다. 외부 bridge 사용 시 vesc_bridge_enabled:=false로 끈다.

주행은 차선 추종 → 막히면 회피 → 충돌 없는 복귀 궤적이 열리면 차선 추종으로 통합했다.
유도 미션/입구/양쪽 콘 통로 판정 및 통로에서의 차선 이탈 제한 해제는 제거했다.
모든 회피는 동일한 차선 이탈 한도와 격자·차체 검사를 따른다.
VFH 마스크가 모두 막혀도 별도 곡선을 차체 검사하며, 관련 샘플 수·길이 여유·복귀/연속성 가중치는 VFH 설정창에서 출발 전에 조절한다.

최신 사진 요청의 VFH step 0.03m·감속도 0.4m/s²·응답 지연 0.1초·hazard 4회/8셀을 반영했다. 격자 바깥 통과를 허용하고, horizon 전체의 차선 주행 차체·패딩과 빨간 셀 충돌만 회피를 활성화한다. 현재 속도의 정지 거리로 회피 진입을 제한하는 안은 사용자 후속 선택으로 사용하지 않는다.
