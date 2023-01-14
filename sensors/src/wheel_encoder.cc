#include "sensors/wheel_encoder.h"

#include <JetsonGPIO.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <string>

namespace sensors {

EncoderCallable::EncoderCallable(
  const std::string& topic_name,
  int& ticks,
  DIRECTION& direction,
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& publisher,
  int queue_limit)
  : topic_name_(topic_name),
  ticks_(ticks),
  direction_(direction),
  publisher_(publisher),
  queue_limit_(queue_limit) {}

void EncoderCallable::operator()(const std::string& channel)
{
  auto message = std_msgs::msg::Int32();
  ticks_ += direction_;
  message.data  = ticks_;
  publisher_->publish(message);
}

WheelEncoder::WheelEncoder(const WheelEncoderConfig& config)
  : Node(strcat("WheelEncoder-", config.orientation_.c_str())),
  config_(config),
  ticks_(0),
  direction_(DIRECTION::BACKWARD)
{

  CreatePublisher();

  callback_= std::make_shared<EncoderCallable>(
    config_.topic_name_,
    ticks_,
    direction_,
    publisher_,
    config_.queue_limit
  );

  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(config_.gpio_pin_, GPIO::IN);
  GPIO::add_event_detect(config_.gpio_pin_, GPIO::RISING, *callback_);

}

void WheelEncoder::CreatePublisher()
{
  RCLCPP_DEBUG(
    this->get_logger(),
    ": Creating publisher for topic '%s' ",
    config_.topic_name_.c_str());
  
  // creating publisher for the encoder total count messages
  publisher_ = this->create_publisher<std_msgs::msg::Int32>(config_.topic_name_, config_.queue_limit);
}

} // namespace sensors
