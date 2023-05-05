#ifndef SPEECH_BASICS_HPP_
#define SPEECH_BASICS_HPP_

#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <mutex>

namespace gb_dialog
{

template<typename T>
class SharedVariable {
public:
  void set(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
  }

  T get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

private:
  mutable std::mutex mutex_;
  T value_;
};

class SpeechBasics : public rclcpp::Node {
public:
  SpeechBasics();

  void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void name(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void askdrink(dialogflow_ros2_interfaces::msg::DialogflowResult result);

private:
  std::shared_ptr<SharedVariable<std::string>> str_var_;
};

}  // namespace gb_dialog

#endif  // SPEECH_BASICS_HPP_
