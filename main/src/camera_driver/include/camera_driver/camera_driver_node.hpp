#ifndef CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
#define CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_

#include <memory>

#include "camera_driver/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camera_driver
{

class CAMERA_DRIVER_PUBLIC CameraDriverNode : public rclcpp::Node
{
public:
  explicit CameraDriverNode(const rclcpp::NodeOptions & options);
  ~CameraDriverNode() override;

  CameraDriverNode(const CameraDriverNode &) = delete;
  CameraDriverNode & operator=(const CameraDriverNode &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace camera_driver

#endif  // CAMERA_DRIVER__CAMERA_DRIVER_NODE_HPP_
