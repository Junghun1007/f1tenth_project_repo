#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "manual_capture/capture_logic.hpp"
#include "manual_capture/capture_writer.hpp"

using namespace manual_capture;

TEST(ExposureFeedback, RequiresAFrameCapturedAfterTheRequest)
{
  const ExposureSetting manual{false, 5000, 800};
  const ExposureReading previous{5000, 800, 9.99};
  const auto waiting = exposureFeedback(manual, 10.0, 10.2, 30.0, previous);
  EXPECT_EQ(waiting.status, ExposureStatus::WAITING_FOR_FRAME);
  ASSERT_TRUE(waiting.actual);
  EXPECT_EQ(waiting.actual->exposure_us, 5000);

  const auto missing = exposureFeedback(manual, 10.0, 10.9, 30.0, previous);
  EXPECT_EQ(missing.status, ExposureStatus::FRAME_MISSING);
  EXPECT_FALSE(missing.actual);
  EXPECT_EQ(
    exposureFeedback(manual, 10.0, 10.2, 30.0, ExposureReading{5000, 800, 10.1}).status,
    ExposureStatus::CONFIRMED);
}

TEST(ExposureFeedback, ShowsLensMismatchAndDetectsStaleFrames)
{
  const ExposureSetting manual{false, 5000, 800};
  const ExposureReading different{5000, 720, 10.15};
  EXPECT_EQ(
    exposureFeedback(manual, 10.0, 10.2, 30.0, different).status,
    ExposureStatus::WAITING_FOR_FRAME);

  const auto mismatch = exposureFeedback(manual, 10.0, 10.85, 30.0, different);
  EXPECT_EQ(mismatch.status, ExposureStatus::MISMATCH);
  ASSERT_TRUE(mismatch.actual);
  EXPECT_EQ(mismatch.actual->iso, 720);
  EXPECT_EQ(
    exposureFeedback(manual, 10.0, 11.0, 30.0, different).status,
    ExposureStatus::FRAME_MISSING);
}

TEST(ExposureFeedback, HandlesAutoAndSlowOrInvalidReadback)
{
  const ExposureSetting automatic{true, 5000, 800};
  EXPECT_EQ(
    exposureFeedback(automatic, 10.0, 11.5, 1.0, std::nullopt).status,
    ExposureStatus::WAITING_FOR_FRAME);
  EXPECT_EQ(
    exposureFeedback(automatic, 10.0, 11.5, 1.0, ExposureReading{4350, 700, 11.0}).status,
    ExposureStatus::AUTOMATIC);
  const auto invalid = exposureFeedback(automatic, 10.0, 13.1, 1.0, ExposureReading{0, 700, 13.0});
  EXPECT_EQ(invalid.status, ExposureStatus::FRAME_MISSING);
  EXPECT_FALSE(invalid.actual);
}

TEST(ViewSelection, StartsWithColorOnly)
{
  ViewSelection selection;
  EXPECT_TRUE(selection.preview(View::RGB));
  EXPECT_FALSE(selection.preview(View::LEFT));
  EXPECT_FALSE(selection.preview(View::CENTER));
  ASSERT_EQ(selection.captureTargets().size(), 1U);
  EXPECT_EQ(selection.captureTargets()[0], View::RGB);
  EXPECT_FALSE(dotProjectorRequired(selection));
}

TEST(ViewSelection, CheckFollowsPreviewAndCanBeCleared)
{
  ViewSelection selection;
  EXPECT_FALSE(selection.setChecked(View::LEFT, true));  // preview off
  selection.setPreview(View::LEFT, true);
  EXPECT_TRUE(selection.checked(View::LEFT));            // auto checked
  EXPECT_TRUE(selection.setChecked(View::LEFT, false));
  EXPECT_EQ(selection.captureTargets().size(), 1U);
  selection.setPreview(View::LEFT, false);
  EXPECT_FALSE(selection.checked(View::LEFT));
}

TEST(ViewSelection, LeftOnlyLocksRightAndCenter)
{
  ViewSelection selection;
  selection.setPreview(View::RIGHT, true);
  selection.setPreview(View::CENTER, true);
  EXPECT_TRUE(dotProjectorRequired(selection));
  EXPECT_FALSE(selection.setPreview(View::LEFT_BEV, true));  // locked in LEFT_RIGHT

  selection.setMode(StereoMode::LEFT_ONLY);
  EXPECT_FALSE(selection.preview(View::RIGHT));
  EXPECT_FALSE(selection.preview(View::CENTER));
  EXPECT_FALSE(selection.setPreview(View::RIGHT, true));
  EXPECT_TRUE(selection.setPreview(View::LEFT_BEV, true));
  EXPECT_FALSE(dotProjectorRequired(selection));
  for (const auto view : selection.captureTargets()) {
    EXPECT_NE(view, View::RIGHT);
    EXPECT_NE(view, View::CENTER);
  }
}

TEST(CaptureController, SingleModeCapturesOnEveryTrigger)
{
  CaptureController controller;
  EXPECT_EQ(controller.trigger(), CaptureAction::CAPTURE_ONCE);
  EXPECT_EQ(controller.trigger(), CaptureAction::CAPTURE_ONCE);
  EXPECT_FALSE(controller.recording());
  EXPECT_TRUE(controller.singleViewCaptureAllowed());
}

TEST(CaptureController, AutoModeToggleForbidsPerViewCapture)
{
  CaptureController controller;
  ASSERT_TRUE(controller.setMode(CaptureMode::AUTO));
  EXPECT_EQ(controller.trigger(), CaptureAction::START_AUTO);
  EXPECT_TRUE(controller.recording());
  EXPECT_FALSE(controller.singleViewCaptureAllowed());
  EXPECT_FALSE(controller.setMode(CaptureMode::SINGLE));  // locked while recording
  EXPECT_EQ(controller.start(), CaptureAction::NONE);
  EXPECT_EQ(controller.trigger(), CaptureAction::STOP_AUTO);
  EXPECT_TRUE(controller.singleViewCaptureAllowed());
  EXPECT_EQ(controller.stop(), CaptureAction::NONE);
}

TEST(ButtonEdge, FiresOnlyOnRisingEdge)
{
  ButtonEdge edge;
  EXPECT_FALSE(edge.update(true));  // held at startup
  EXPECT_FALSE(edge.update(true));
  EXPECT_FALSE(edge.update(false));
  EXPECT_TRUE(edge.update(true));
  EXPECT_FALSE(edge.update(true));
}

TEST(Refit, RejectedWhileMoving)
{
  EXPECT_FALSE(decideRefit(3000, 0.1, 100, 1.0).allowed);
  EXPECT_FALSE(decideRefit(-500, 0.1, 100, 1.0).allowed);
  EXPECT_TRUE(decideRefit(40, 0.1, 100, 1.0).allowed);
  EXPECT_TRUE(decideRefit(std::nullopt, 0.0, 100, 1.0).allowed);  // no telemetry
  EXPECT_TRUE(decideRefit(3000, 5.0, 100, 1.0).allowed);          // stale telemetry
}

TEST(Helpers, ExposureNamesAndScheduler)
{
  EXPECT_EQ(maximumExposureUs(30.0), 33333);
  EXPECT_EQ(frameFileName(42), "000042.jpg");
  EXPECT_EQ(sessionDirectoryName(0).rfind("session_", 0), 0U);

  IntervalScheduler scheduler;
  scheduler.reset(10.0, 5.0);
  EXPECT_TRUE(scheduler.due(10.0));
  EXPECT_FALSE(scheduler.due(10.1));
  EXPECT_TRUE(scheduler.due(10.2));
  EXPECT_TRUE(scheduler.due(15.0));   // after a stall: one capture, no burst
  EXPECT_FALSE(scheduler.due(15.1));
}

TEST(CaptureWriter, WritesJpegsPerViewAndMeta)
{
  const auto session = std::filesystem::temp_directory_path() / "manual_capture_writer_test";
  std::filesystem::remove_all(session);
  {
    CaptureWriter writer("event,views", 4);
    CaptureEvent event;
    event.session_directory = session / "manual";
    event.index = 7;
    event.images.push_back({View::RGB, cv::Mat(8, 8, CV_8UC3, cv::Scalar(10, 20, 30))});
    event.images.push_back({View::LEFT, cv::Mat(8, 8, CV_8UC1, cv::Scalar(99))});
    event.meta_row = "7,rgb;stereo_left";
    ASSERT_TRUE(writer.enqueue(event));
    auto auto_event = event;
    auto_event.session_directory = session / "auto" / "first";
    auto_event.images = {{View::RGB_BEV, cv::Mat(8, 8, CV_8UC3, cv::Scalar(10, 20, 30))}};
    auto_event.meta_row = "7,rgb_bev";
    ASSERT_TRUE(writer.enqueue(auto_event));
    auto_event.session_directory = session / "auto" / "second";
    ASSERT_TRUE(writer.enqueue(std::move(auto_event)));
    event.index = 8;
    event.meta_row = "8,rgb;stereo_left";
    ASSERT_TRUE(writer.enqueue(std::move(event)));
    writer.flush();
    EXPECT_EQ(writer.written(View::RGB), 2U);
    EXPECT_EQ(writer.written(View::LEFT), 2U);
    EXPECT_EQ(writer.written(View::RGB_BEV), 2U);
    EXPECT_EQ(writer.writtenTotal(), 6U);
  }
  EXPECT_TRUE(std::filesystem::exists(session / "manual" / "rgb" / "000007.jpg"));
  EXPECT_TRUE(std::filesystem::exists(session / "manual" / "stereo_left" / "000007.jpg"));
  EXPECT_TRUE(std::filesystem::exists(session / "auto" / "first" / "rgb_bev" / "000007.jpg"));
  EXPECT_TRUE(std::filesystem::exists(session / "auto" / "second" / "rgb_bev" / "000007.jpg"));
  const auto check_meta = [](const std::filesystem::path & directory,
      const std::vector<std::string> & expected) {
      std::ifstream meta(directory / "meta.csv");
      std::vector<std::string> rows;
      for (std::string row; std::getline(meta, row);) {
        rows.push_back(row);
      }
      EXPECT_EQ(rows, expected);
    };
  check_meta(session / "manual", {"event,views", "7,rgb;stereo_left", "8,rgb;stereo_left"});
  check_meta(session / "auto" / "first", {"event,views", "7,rgb_bev"});
  check_meta(session / "auto" / "second", {"event,views", "7,rgb_bev"});
  std::filesystem::remove_all(session);
}
