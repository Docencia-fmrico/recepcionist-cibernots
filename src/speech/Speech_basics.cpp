#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <mutex>

//#include "speech/Speech_basics.hpp"

using namespace std::placeholders;  
namespace ph = std::placeholders;

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

  void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] noIntentCB: intent [%s]", result.intent.c_str());
  }

  void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] welcomeIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
  }

  void drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] drinks: intent [%s]", result.intent.c_str());
    str_var_->set(result.intent.param_);
    speak(result.fulfillment_text);
  }

  void name(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] name: intent [%s]", result.intent.c_str());
    str_var_->set(result.intent.param_);
    speak(result.fulfillment_text);
  }

  void introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
    std::ostringstream oss;
    std::string phrase;

    std::string name = str_var_->get();
    oss << "Their name is " << name;
    phrase = oss.str();
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] introduce: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    speak(phrase);
  }

  void askdrink(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
    std::ostringstream oss;
    std::string phrase;

    std::string name = str_var_->get();
    oss << "I want a " << name << " for our guest";
    phrase = oss.str();
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] askdrink: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    speak(phrase);
  }

private:
  std::shared_ptr<SharedVariable<std::string>> str_var_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto forwarder = std::make_shared<gb_dialog::ExampleDF>();
  forwarder->listen();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(forwarder);
  executor.spin();

  return 0;
}
}  // namespace gb_dialog