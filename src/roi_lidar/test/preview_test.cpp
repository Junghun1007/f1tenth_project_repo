#include "roi_lidar/preview.hpp"
#include <iostream>
using namespace roi_lidar;
static void check(bool ok,const char * text) {if (!ok) {throw std::runtime_error(text);}}
int main()
{
  Options o; o.max_range=4;
  const auto origin=mapPixel(0,0,o,800),front=mapPixel(1,0,o,800),left=mapPixel(1,1,o,800);
  check(front.y<origin.y && left.x<front.x,"BEV axes mirrored");
  BevProjector bev;
  // Down-facing pinhole: world ground -> center pixel; RGB is already undistorted.
  point_cloud::RigidTransform t{{1,0,0,0,-1,0,0,0,-1},{1,0,1}};
  point_cloud::Intrinsics k{20,20,50,50};
  bev.configure(100,100,k,t,o,800);
  cv::Mat rgb(100,100,CV_8UC3,cv::Scalar(10,80,160));
  const auto background=bev.render(rgb);
  check(background.at<cv::Vec3b>(front)==cv::Vec3b(10,80,160),"RGB ground projection misaligned");
  check(background.at<cv::Vec3b>(0,0)==cv::Vec3b(35,35,35),"Outside RGB FOV not masked");
  auto scan=emptyScan(o); scan.ranges[o.bins/2]=1;
  const auto overlay=mapPreview(scan,o,800,"TEST",background);
  check(overlay.at<cv::Vec3b>(front)==cv::Vec3b(30,85,225),"Obstacle not overlaid at metric position");
  // Non-symmetric rotation and translated camera: verify inverse ground mapping.
  const auto tilted=point_cloud::vehicleFromRgb(-.16,.03,.2,3,15,7);
  const point_cloud::Intrinsics intrinsics{180,180,160,100};
  bev.configure(320,200,intrinsics,tilted,o,800);
  cv::Mat gradient(200,320,CV_8UC3);
  for (int row=0;row<200;++row) {for (int col=0;col<320;++col) {
    gradient.at<cv::Vec3b>(row,col)={static_cast<unsigned char>(col/2),static_cast<unsigned char>(row),100};
  }}
  const auto pixel=mapPixel(1,.2,o,800);
  const double scale=(800-80)/(2.0*o.max_range);
  const point_cloud::Vector3 delta{(800*.5+40-pixel.y)/scale-tilted.translation[0],
    (400-pixel.x)/scale-tilted.translation[1],-tilted.translation[2]};
  point_cloud::Vector3 optical{};
  for (int axis=0;axis<3;++axis) {for (int j=0;j<3;++j) {
    optical[axis]+=tilted.rotation[j*3+axis]*delta[j];
  }}
  const double expected_u=intrinsics.fx*optical[0]/optical[2]+intrinsics.cx;
  const double expected_v=intrinsics.fy*optical[1]/optical[2]+intrinsics.cy;
  check(expected_u>0 && expected_u<319 && expected_v>0 && expected_v<199,"Invalid test geometry");
  const auto color=bev.render(gradient).at<cv::Vec3b>(pixel);
  check(std::abs(color[0]-expected_u/2)<2 && std::abs(color[1]-expected_v)<2,
    "Tilted/translated RGB BEV mapping incorrect");
  cv::Mat depth(40,60,CV_16UC1,cv::Scalar(1000));
  auto roi=roiPreview(depth,o,true);
  check(roi.rows==40 && roi.cols==60 && roi.type()==CV_8UC3,"Depth ROI preview malformed");
  cv::Mat gray(40,60,CV_8UC1,cv::Scalar(100));
  check(roiPreview(gray,o,false).type()==CV_8UC3,"Camera ROI preview malformed");
  std::cout<<"RGB BEV preview tests passed: metric projection, axes, FOV mask, obstacle overlay, ROI\n";
}
