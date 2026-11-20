// Copied from src/cone_drive_test/include/cone_drive_test/core/lane_post.hpp (1113ML) — auto_control로 옮김
// 차선 라벨(120×300, 1=왼쪽 2=오른쪽) → 성분 정리·경계 보간(lane_connector) → 중앙선(centerline) → 앞차축 기준 m 좌표.
// line_detactor_node의 후처리 순서와 설정(line_detactor.yaml) 그대로.
#pragma once
#include <opencv2/core.hpp>
#include "auto_control/drive/copied/centerline.hpp"
#include "auto_control/drive/copied/lane_connector.hpp"
#include "auto_control/drive/copied/stop_line_distance.hpp"
#include "auto_control/drive/core/lane_side_tracker.hpp"
#include "auto_control/drive/core/types.hpp"

namespace auto_control::drive {

struct LanePostConfig {
  LaneConnectionConfig connection;
  CenterlineConfig centerline;
  double x_max_m{3.0}, y_max_m{0.6}, meter_per_pixel{0.01};
  bool side_tracking{true};
  StopLineDistanceConfig stop_line;   // line_detactor 기본값   // 차선 좌우 판정 고정 (lane.side_tracking)
  LanePostConfig();   // line_detactor.yaml 값으로 초기화
};

class LanePostprocessor {
public:
  explicit LanePostprocessor(const LanePostConfig & c);
  void set_config(const LanePostConfig & c);
  // labels: 모델 입력 크기(가로 120, 세로 300) mono8. want_reference: 근처에 콘이 있을 때만 바깥 이동 없는 중앙선 추가 계산
  // stop_mask: 정지선 채널 (모델 크기, 비어 있으면 정지선 계산 생략)
  LaneObservation process(const cv::Mat & labels, double capture_time, bool want_reference, const cv::Mat & stop_mask = cv::Mat()) const;
  Vec2 pixel_to_vehicle(const cv::Point2f & p_padded) const;

private:
  LanePostConfig c_;
  CenterlineConfig reference_;
  mutable LaneSideTracker tracker_;   // 차선 스레드 전용
};

}  // namespace auto_control::drive
