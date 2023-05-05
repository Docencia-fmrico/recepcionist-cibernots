#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto forwarder = std::make_shared<gb_dialog::Speech>();
  forwarder->listen();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(forwarder);
  executor.spin();

  return 0;
}

