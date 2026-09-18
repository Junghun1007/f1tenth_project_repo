#include "roi_lidar/preview.hpp"
#include <opencv2/imgproc.hpp>
#include <iomanip>
#include <sstream>
namespace roi_lidar
{
cv::Point mapPixel(double x, double y, const Options & o, int size)
{
  const double scale=(size-80)/(2.0*o.max_range);
  return {static_cast<int>(std::lround(size/2.0-y*scale)),
    static_cast<int>(std::lround(size*0.5+40-x*scale))};
}
cv::Mat mapPreview(const Scan & scan, const Options & o, int size, const std::string & status, const cv::Mat & background)
{
  const int height=static_cast<int>(size*.5)+140;
  cv::Mat canvas(height,size,CV_8UC3,cv::Scalar(245,245,245));
  if (!background.empty()) {background.copyTo(canvas);}
  cv::rectangle(canvas,{0,0,size,80},{245,245,245},cv::FILLED);
  const int footer=static_cast<int>(size*.5)+44;
  cv::rectangle(canvas,{0,footer,size,height-footer},{245,245,245},cv::FILLED);
  const double step=o.max_range<=15 ? 1.0 : 5.0;
  for (double x=0;x<=o.max_range;x+=step) {
    cv::line(canvas,mapPixel(x,-o.max_range,o,size),mapPixel(x,o.max_range,o,size),{220,220,220});
    cv::putText(canvas,std::to_string(static_cast<int>(x))+"m",mapPixel(x,-o.max_range,o,size)+cv::Point(-30,0),
      cv::FONT_HERSHEY_SIMPLEX,.35,{180,180,180},1,cv::LINE_AA);
  }
  for (double y=-o.max_range;y<=o.max_range;y+=step) {
    cv::line(canvas,mapPixel(0,y,o,size),mapPixel(o.max_range,y,o,size),{225,225,225});
  }
  const auto origin=mapPixel(0,0,o,size);
  for (const double angle : {o.angle_min,o.angle_max}) {
    cv::line(canvas,origin,mapPixel(o.max_range*std::cos(angle*radians),
      o.max_range*std::sin(angle*radians),o,size),{190,180,120},1,cv::LINE_AA);
  }
  float closest=std::numeric_limits<float>::infinity();
  cv::Point nearest;
  for (std::size_t i=0;i<scan.ranges.size();++i) {
    const float r=scan.ranges[i];
    if (!std::isfinite(r) || r<o.min_range || r>o.max_range) {continue;}
    const double a=(o.angle_min+i*(o.angle_max-o.angle_min)/(o.bins-1))*radians;
    const auto p=mapPixel(r*std::cos(a),r*std::sin(a),o,size);
    cv::circle(canvas,p,3,{30,85,225},-1,cv::LINE_AA);
    if (r<closest) {closest=r; nearest=p;}
  }
  cv::arrowedLine(canvas,origin,origin+cv::Point(0,-22),{80,150,30},3,cv::LINE_AA);
  cv::putText(canvas,"FRONT AXLE | +X forward / +Y left",origin+cv::Point(-140,28),cv::FONT_HERSHEY_SIMPLEX,.45,{50,50,50},1,cv::LINE_AA);
  if (std::isfinite(closest)) {
    std::ostringstream label; label<<std::fixed<<std::setprecision(2)<<closest<<" m";
    cv::putText(canvas,label.str(),nearest+cv::Point(8,-8),cv::FONT_HERSHEY_SIMPLEX,.5,{245,245,245},3,cv::LINE_AA);
    cv::putText(canvas,label.str(),nearest+cv::Point(8,-8),cv::FONT_HERSHEY_SIMPLEX,.5,{20,60,190},1,cv::LINE_AA);
  }
  cv::putText(canvas,"ROI LIDAR | current obstacle surface returns",{20,30},cv::FONT_HERSHEY_SIMPLEX,.55,{40,40,40},1,cv::LINE_AA);
  cv::putText(canvas,status,{20,55},cv::FONT_HERSHEY_SIMPLEX,.4,{50,50,50},1,cv::LINE_AA);
  cv::putText(canvas,"Unobserved area is UNKNOWN | Q: quit | R: full ROI",{20,height-25},cv::FONT_HERSHEY_SIMPLEX,.4,{65,65,65},1,cv::LINE_AA);
  return canvas;
}
void BevProjector::configure(int width, int height, const point_cloud::Intrinsics & k,
  const point_cloud::RigidTransform & t, const Options & o, int size)
{
  if (width<1 || height<1 || size<240 || size>1600 || k.fx<=0 || k.fy<=0) {
    throw std::invalid_argument("Invalid RGB BEV geometry");
  }
  map_x_=cv::Mat(static_cast<int>(size*.5)+140,size,CV_32F,cv::Scalar(-1));
  map_y_=cv::Mat(static_cast<int>(size*.5)+140,size,CV_32F,cv::Scalar(-1));
  const double scale=(size-80)/(2.0*o.max_range);
  for (int row=0;row<map_x_.rows;++row) {
    for (int col=0;col<size;++col) {
      const double x=(size*.5+40-row)/scale, y=(size/2.0-col)/scale;
      if (x<0 || x>o.max_range || std::abs(y)>o.max_range) {continue;}
      const point_cloud::Vector3 delta{x-t.translation[0],y-t.translation[1],-t.translation[2]};
      point_cloud::Vector3 camera{};
      for (int axis=0;axis<3;++axis) {
        for (int j=0;j<3;++j) {camera[axis]+=t.rotation[j*3+axis]*delta[j];}
      }
      if (camera[2]<=0) {continue;}
      const double u=k.fx*camera[0]/camera[2]+k.cx, v=k.fy*camera[1]/camera[2]+k.cy;
      if (u<0 || v<0 || u>=width-1 || v>=height-1) {continue;}
      map_x_.at<float>(row,col)=u; map_y_.at<float>(row,col)=v;
    }
  }
}
cv::Mat BevProjector::render(const cv::Mat & bgr) const
{
  if (map_x_.empty() || bgr.type()!=CV_8UC3) {throw std::invalid_argument("Unconfigured RGB BEV preview");}
  cv::Mat out;
  cv::remap(bgr,out,map_x_,map_y_,cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(35,35,35));
  return out;
}
cv::Mat roiPreview(const cv::Mat & input, const Options & o, bool depth)
{
  cv::Mat gray, image;
  if (depth) {
    input.convertTo(gray,CV_8U,255.0/o.max_depth/1000.0);
    cv::applyColorMap(gray,image,cv::COLORMAP_TURBO);
    image.setTo(cv::Scalar(0,0,0),input==0);
  } else {cv::cvtColor(input,image,cv::COLOR_GRAY2BGR);}
  const auto roi=imageRoi(image.cols,image.rows,o);
  cv::rectangle(image,{roi.x,roi.y,roi.width,roi.height},{40,240,40},2);
  cv::putText(image,"Drag ROI | R: reset",{10,22},cv::FONT_HERSHEY_SIMPLEX,.5,{40,255,40},1,cv::LINE_AA);
  return image;
}
}
