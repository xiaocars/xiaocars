#include <iostream>
#include <memory>
#include <functional>
#include <JetsonGPIO.h>

#include <sensors/wheel_encoder.h>

namespace sensors {

EncoderCallable::EncoderCallable(
  const std::string& topic_name,
  int& ticks,
  DIRECTION& direction,
  std::shared_ptr<rclcpp::Node> & nh,
  int queue_limit)
  : topic_name_(topic_name),
  ticks_(ticks),
  direction_(direction),
  nh_(nh),
  queue_limit_(queue_limit)
{
  RCLCPP_INFO(nh_->get_logger(), ": Wheel Encoder Callback: Creating publisher for topic '%s' ", topic_name.c_str());
  publisher_ = nh_->create_publisher<std_msgs::msg::Int32>(topic_name_, queue_limit);
}

void EncoderCallable::operator()(const std::string& channel)
{
  auto message = std_msgs::msg::Int32();
  ticks_ += direction_;
  message.data  = ticks_;
  RCLCPP_DEBUG(
    nh_->get_logger(),
    "Publishing wheel encoder count '%d' on channel '%s'",
    message.data,
    channel.c_str());
  publisher_->publish(message);
}

WheelEncoder::WheelEncoder(
  std::shared_ptr<rclcpp::Node> & nh,
  const WheelEncoderConfig& config)
  : nh_(nh),
  config_(config),
  ticks_(0),
  direction_(DIRECTION::BACKWARD)
{
  callback_= std::make_shared<EncoderCallable>(
    config_.topic_name_,
    ticks_,
    direction_,
    nh_,
    config_.queue_limit
  );

  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(config_.gpio_pin_, GPIO::IN);
  GPIO::add_event_detect(config_.gpio_pin_, GPIO::RISING, *callback_);

  // initialized_ = true;
  // return true;
}


} // namespace sensors
