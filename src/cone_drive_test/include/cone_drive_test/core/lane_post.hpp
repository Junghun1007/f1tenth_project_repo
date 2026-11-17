// 차선 라벨(120×300, 1=왼쪽 2=오른쪽) → 성분 정리·경계 보간(lane_connector) → 중앙선(centerline) → 앞차축 기준 m 좌표.
// line_detactor_node의 후처리 순서와 설정(line_detactor.yaml) 그대로.
#pragma once
#include <opencv2/core.hpp>
#include "cone_drive_test/copied/centerline.hpp"
#include "cone_drive_test/copied/lane_connector.hpp"
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct LanePostConfig {
  LaneConnectionConfig connection;
  CenterlineConfig centerline;
  double x_max_m{3.0}, y_max_m{0.6}, meter_per_pixel{0.01};
  LanePostConfig();   // line_detactor.yaml 값으로 초기화
};

class LanePostprocessor {
public:
  explicit LanePostprocessor(const LanePostConfig & c);
  void set_config(const LanePostConfig & c);
  // labels: 모델 입력 크기(가로 120, 세로 300) mono8. want_reference: 근처에 콘이 있을 때만 바깥 이동 없는 중앙선 추가 계산
  LaneObservation process(const cv::Mat & labels, double capture_time, bool want_reference) const;
  Vec2 pixel_to_vehicle(const cv::Point2f & p_padded) const;

private:
  LanePostConfig c_;
  CenterlineConfig reference_;
};

}  // namespace cone_drive_test
