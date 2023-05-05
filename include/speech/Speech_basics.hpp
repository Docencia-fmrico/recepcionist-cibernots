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

class SpeechBasics : public DialogInterface {
public:
  SpeechBasics() : str_var_(std::make_shared<SharedVariable<std::string>>()) {
    this->registerCallback(std::bind(&SpeechBasics::noIntentCB, this, ph::_1));
    this->registerCallback(std::bind(&SpeechBasics::welcomeIntentCB, this, ph::_1), "Default Welcome Intent");
    this->registerCallback(std::bind(&SpeechBasics::drinks, this, ph::_1), "Drinks");
    this->registerCallback(std::bind(&SpeechBasics::name, this, ph::_1), "Name");
    this->registerCallback(std::bind(&SpeechBasics::askdrink, this, ph::_1), "Ask Drink");
    this->registerCallback(std::bind(&SpeechBasics::introduce, this, ph::_1), "Introduce Guest");
  }

  void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void name(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result);
  void askdrink(dialogflow_ros2_interfaces::msg::DialogflowResult result);

private:
  void speak(const std::string& phrase);

  std::shared_ptr<SharedVariable<std::string>> str_var_;
};

}  // namespace gb_dialog

#endif  // SPEECH_BASICS_HPP_
