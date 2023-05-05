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

SpeechBasics::SpeechBasics()
: Node("Speech_basics")
{
SpeechBasics() : str_var_(std::make_shared<SharedVariable<std::string>>()) {
  this->registerCallback(std::bind(&SpeechBasics::noIntentCB, this, ph::_1));
  this->registerCallback(std::bind(&SpeechBasics::welcomeIntentCB, this, ph::_1), "Default Welcome Intent");
  this->registerCallback(std::bind(&SpeechBasics::drinks, this, ph::_1), "Drinks");
  this->registerCallback(std::bind(&SpeechBasics::name, this, ph::_1), "Name");
  this->registerCallback(std::bind(&SpeechBasics::askdrink, this, ph::_1), "Ask Drink");
  this->registerCallback(std::bind(&SpeechBasics::introduce, this, ph::_1), "Introduce Guest");
}
}

void SpeechBasics::noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
  RCLCPP_INFO(this->get_logger(), "[SpeechBasics] noIntentCB: intent [%s]", result.intent.c_str());
}

void SpeechBasics::welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
  RCLCPP_INFO(this->get_logger(), "[SpeechBasics] welcomeIntentCB: intent [%s]", result.intent.c_str());
  speak(result.fulfillment_text);
}

void SpeechBasics::drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
  RCLCPP_INFO(this->get_logger(), "[SpeechBasics] drinks: intent [%s]", result.intent.c_str());
  str_var_->set(result.intent.param_);
  speak(result.fulfillment_text);
}

void SpeechBasics::name(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
  RCLCPP_INFO(this->get_logger(), "[SpeechBasics] name: intent [%s]", result.intent.c_str());
  str_var_->set(result.intent.param_);
  speak(result.fulfillment_text);
}

void SpeechBasics::introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
  std::ostringstream oss;
  std::string phrase;

  std::string name = str_var_->get();
  oss << "Their name is " << name;
  phrase = oss.str();
  RCLCPP_INFO(this->get_logger(), "[SpeechBasics] introduce: intent [%s]", result.intent.c_str());
  speak(result.fulfillment_text);
  speak(phrase);
}

void SpeechBasics::askdrink(dialogflow_ros2_interfaces::msg::DialogflowResult result) {
  std::ostringstream oss;
  std::string phrase;

  std::string name = str_var_->get();
  oss << "I want a " << name << " for our guest";
  phrase = oss.str();
  RCLCPP_INFO(this->get_logger(), "[SpeechBasics] askdrink: intent [%s]", result.intent.c_str());
  speak(result.fulfillment_text);
  speak(phrase);
}
}  // namespace gb_dialog