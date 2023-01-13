#pragma once

#include <iostream>
#include <memory>
#include <functional>
#include <JetsonGPIO.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace sensors {

/*!
  * @brief ENUM that represents the direction of motion for the robot.
  * This value is updated when the controller makes a decision about
  * direction.
  * The value is used to update the global tick count on the encoder.
  * @ingroup Sensors
  */
enum DIRECTION {
  FORWARD = 1,
  BACKWARD = -1
};

struct WheelEncoderConfig {
  int gpio_pin_;
  int resolution_;
  int publish_frequency_;
  int queue_limit;
  std::string orientation_;
  std::string topic_name_;
};

/*!
  * @brief This class is used by the Jetson GPIO c++ library as a callback
  * for the event detect on a GPIO pin.
  * We cannot use a regular lambda bcause this Jetson GPIO library requires
  * the callbacks must be equaility-comparable.
  * More details: https://github.com/pjueon/JetsonGPIO/issues/55#issuecomment-1059888835
  * @ingroup Sensors
  */
class EncoderCallable
{
public:
  EncoderCallable() = delete;
  /*!
    * @brief Default constructor.
    * @param name Reference to the name assigned to this callback.
    * @param ticks Pointer referring to total ticks in the WheelEncoderSensor.
    * @param callback The external callback passed to WheelEncoderSensor.
    * @param direction Pointer to the DIRECTION enum in WheelEncoderSensor.
    */
  EncoderCallable(
    const std::string & topic_name,
    int & ticks,
    DIRECTION & direction,
    std::shared_ptr<rclcpp::Node> & nh,
    int queue_limit);

  /*!
    * @brief Callable executed by the GPIO library upon event detection.
    * Used to update the global tick count and execute the callback provided
    * to the Encoder through its constructor.
    * @param channel Reference to the GPIO pin passed in by the library.
    */
  void operator()(const std::string& channel);
  // Callable with one string type argument
  // {
  //   // std::cout << "A callback named " << topic_name;
  //   // std::cout << " called from channel " << channel << std::endl;
  //   ticks_ += direction_;
  // }

  /*!
    * @brief equality operator used by GPIO library to remove callbacks.
    * @param other Reference to the object being compared.
    */
  bool operator==(const EncoderCallable& other) const // Equality-comparable
  {
    return topic_name_ == other.topic_name_;
  }

  /*!
    * @brief in-equality operator used by GPIO library to remove callbacks.
    * @param other Reference to the object being compared.
    */
  bool operator!=(const EncoderCallable& other) const 
  {
    return !(*this == other);
  }
    
private:
  const std::string& topic_name_;
  int& ticks_;
  DIRECTION& direction_;
  std::shared_ptr<rclcpp::Node> & nh_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  int & queue_limit_;
};


/*!
 * @brief This class manages the WheelEncoder sensor connected to the GPIO
 * pins in the NVIDIA Jetson Nano Board
 * @ingroup Sensors
*/
class WheelEncoder
{
public:
  /*!
    * @brief Default constructor.
    */
  WheelEncoder() = delete;

  WheelEncoder(
    std::shared_ptr<rclcpp::Node> & nh,
    const WheelEncoderConfig & config);
private:
  std::shared_ptr<rclcpp::Node> & nh_;
  const WheelEncoderConfig & config_;
  int ticks_;
  DIRECTION direction_;
  bool initialized_ = false;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std::shared_ptr<EncoderCallable> callback_ = nullptr;
};


} // namespace sensors
