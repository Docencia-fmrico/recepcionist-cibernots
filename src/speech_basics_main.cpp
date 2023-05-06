#include "gb_dialog/DialogInterfaces.hpp"
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto forwarder = std::make_shared<gb_dialog::DialogInterface>();
  forwarder->listen();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(forwarder);
  executor.spin();

  return 0;
}