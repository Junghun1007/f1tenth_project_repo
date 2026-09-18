#pragma once
#include "bev_handoff/avoidance_handoff.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace line_detactor
{
inline std::string drawAvoidancePreview(cv::Mat & image,const std_msgs::msg::Header & header,
  int width,int height,int padding,double max_delta,double max_age)
{
  if (!bev_handoff::avoidancePreviewEnabled()) {return {};}
  const auto plan=bev_handoff::latestAvoidancePreview();
  if (!plan) {return "TEST WAIT: lane/depth";}
  const auto now=std::chrono::steady_clock::now();
  const double age=std::chrono::duration<double>(now-plan->lane_received_at).count();
  const double depth_age=std::chrono::duration<double>(now-plan->depth_captured_at).count();
  const double delta=std::abs(double(header.stamp.sec)-plan->header.stamp.sec+
    (double(header.stamp.nanosec)-plan->header.stamp.nanosec)*1e-9);
  const double display_delta=std::min(max_age,std::max(max_delta,plan->display_delta_sec));
  if (plan->header.frame_id!=header.frame_id || age<0 || age>max_age || depth_age<0 || depth_age>max_age || delta>display_delta) {
    return "TEST WAIT: stale/unsynced";
  }
  if (plan->width!=width || plan->height!=height || image.cols!=width+2*padding || image.rows!=height) {
    return "TEST WAIT: geometry";
  }
  auto plot=image(cv::Rect(padding,0,width,height));
  const auto pixel=[&](double x,double y) {
    return cv::Point(static_cast<int>(std::lround((plan->y_max-y)/plan->meter_per_pixel-.5)),
      static_cast<int>(std::lround((plan->x_max-x)/plan->meter_per_pixel-.5)));
  };
  for (const auto & box:plan->boxes) {
    cv::rectangle(plot,pixel(box.x0,box.y0),pixel(box.x1,box.y1),cv::Scalar(70,70,180),1);
  }
  const auto path=[&](const std::vector<cv::Point2d> & points,const cv::Scalar & color,int thickness) {
    for (std::size_t i=1;i<points.size();++i) {
      cv::line(plot,pixel(points[i-1].x,points[i-1].y),pixel(points[i].x,points[i].y),color,thickness,cv::LINE_AA);
    }
  };
  for (const auto & c:plan->candidates) {path(c.path,c.valid?cv::Scalar(110,150,110):cv::Scalar(65,65,95),1);}
  path(plan->original,cv::Scalar(0,255,255),1);
  path(plan->selected,cv::Scalar(255,255,0),2);
  // Detailed rejection reason is available on /auto/avoidance_preview/status.
  return std::string(plan->control_requested?"APPLY ":"TEST ")+plan->status.substr(0,plan->status.find(':'))+
    cv::format(" v%.2f k%.1f",plan->recommended_speed,plan->max_curvature);
}
}
