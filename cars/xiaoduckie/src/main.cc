#include <rclcpp/rclcpp.hpp>
#include <xiaoduckie/xiaoduckie.h>

int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);
  // xiaocars::XiaoDuckie  xiaoduckie = xiaocars::XiaoDuckie();
  rclcpp::spin(std::make_shared<xiaocars::XiaoDuckie>());
  rclcpp::shutdown();
  return 0;
}