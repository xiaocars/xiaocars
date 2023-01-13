#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>


namespace xiaocars
{
class XiaoDuckie : public rclcpp::Node
{
  public:
    XiaoDuckie();

  private:
    size_t count_;
};
} // namespace xiaocars
