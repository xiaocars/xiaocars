#include <xiaoduckie/create_nodes.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  std::string cmd_vel_topic_name("/xiaoduckie/cmd_vel");
  std::string wheels_vel_topic_name("/xiaoduckie/wheels_vel");

  auto hat =  Build_HAT();

  auto driver = Build_DifferentialDriveWheelsDriver(
    options,
    hat,
    wheels_vel_topic_name);

  auto controller_node = Build_DifferentialDriveController(
    options,
    cmd_vel_topic_name,
    wheels_vel_topic_name);

  exec.add_node(controller_node);
  exec.add_node(driver);
  
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
