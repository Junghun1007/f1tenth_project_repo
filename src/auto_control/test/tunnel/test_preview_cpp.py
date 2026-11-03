"""Compile the real HUD renderer and preview handoff without ROS/GPU/GUI."""
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[3]


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    # Default array parameter has {} before the function body.
    if source[opening:opening+2] == '{}':
        opening = source.index('{', opening+2)
    depth, end = 1, opening+1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class PreviewNativeTests(unittest.TestCase):
    def test_real_renderer_and_owning_latest_frame_handoff(self):
        compiler, pkg = shutil.which('c++'), shutil.which('pkg-config')
        if not compiler or not pkg:
            self.skipTest('C++ compiler/OpenCV development files unavailable')
        config = subprocess.run([pkg, '--cflags', '--libs', 'opencv4'],
                                text=True, capture_output=True)
        if config.returncode:
            self.skipTest('OpenCV development files unavailable')
        source = (SOURCE / 'line_detactor/src/line_detactor_node.cpp').read_text()
        renderer = function(source, '  cv::Mat preview_canvas(')
        code = r'''
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "bev_handoff/direct_bev_handoff.hpp"
namespace bev_handoff {bool avoidanceControlRequested() {return false;}}
constexpr int kBannerHeight=144;
struct Harness {
  bool obstacle_overlay_enabled_{true},tunnel_enabled_{true};
  struct {bool enabled{true};} connection_;
RENDERER
};
void require(bool condition) {
  if (!condition) {throw std::runtime_error("Preview handoff/HUD regression");}
}
int main() {
  require(!bev_handoff::latestIrLanePreview());
  {
    auto frame=std::make_shared<bev_handoff::IrLanePreview>();
    frame->image=cv::Mat::zeros(300,180,CV_8UC3);
    frame->image.at<cv::Vec3b>(0,0)={1,2,3};
    frame->generation=7;
    bev_handoff::publishIrLanePreview(frame);
  }
  auto retained=bev_handoff::latestIrLanePreview();
  require(retained && retained->generation==7);
  require(retained->image.at<cv::Vec3b>(0,0)==cv::Vec3b(1,2,3));
  bev_handoff::publishIrLanePreview(nullptr);
  require(!bev_handoff::latestIrLanePreview());
  require(retained->image.at<cv::Vec3b>(0,0)==cv::Vec3b(1,2,3));
  cv::Mat image=cv::Mat::zeros(300,180,CV_8UC3);
  cv::line(image,{50,299},{35,0},{255,0,0},2);
  cv::line(image,{130,299},{145,0},{0,0,255},2);
  cv::line(image,{90,299},{90,0},{0,255,255},2);
  Harness preview;
  const std::array<std::string,3> hud{
    "Tunnel: ENTERING","TCN avg 1234.5FPS","TCN LIVE E:0.75 X:0.05"};
  const auto canvas=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,.075,"","",hud);
  require(canvas.cols==180 && canvas.rows==300+144+28+14);
  require(cv::countNonZero(canvas(cv::Rect(0,300+14*5,180,14)).reshape(1))>0);
  cv::Mat enlarged;
  cv::resize(canvas,enlarged,{},3.,3.,cv::INTER_NEAREST);
  cv::imwrite("/tmp/f1tenth_tunnel_preview_hud.png",enlarged);
  preview.tunnel_enabled_=false;
  const auto legacy=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,.075);
  require(legacy.rows==300+144+14);
  const auto unknown=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,
    std::numeric_limits<double>::quiet_NaN());
  const cv::Rect duty_row(0,300+14*8,180,14);
  require(cv::norm(legacy(duty_row),unknown(duty_row),cv::NORM_L1)>0);
  require(cv::norm(legacy(cv::Rect(0,300,180,14*8)),
    unknown(cv::Rect(0,300,180,14*8)),cv::NORM_L1)==0);
}
'''.replace('RENDERER', renderer)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / 'std_msgs/msg/header.hpp'
            header.parent.mkdir(parents=True)
            header.write_text('#pragma once\nnamespace std_msgs {namespace msg {struct Header {};}}\n')
            file, binary = root / 'preview.cpp', root / 'preview_test'
            file.write_text(code)
            compiled = subprocess.run([compiler, '-std=c++17', '-O2', '-DNDEBUG', '-Wall', '-Wextra',
                            '-Werror', '-I'+str(root), '-I'+str(SOURCE/'bev_handoff/include'),
                            str(file), str(SOURCE/'bev_handoff/src/direct_bev_handoff.cpp'),
                            *shlex.split(config.stdout), '-o', str(binary)],
                           capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
