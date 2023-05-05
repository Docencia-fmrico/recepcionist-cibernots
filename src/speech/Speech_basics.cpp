#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace gb_dialog
{

class SpeechBasics: public DialogInterface
{
  public:
    SpeechBasics()
    {
      this->registerCallback(std::bind(&SpeechBasics::noIntentCB, this, ph::_1));
      this->registerCallback(std::bind(&SpeechBasics::welcomeIntentCB, this, ph::_1),"Default Welcome Intent");
      this->registerCallback(std::bind(&SpeechBasics::drinks, this, ph::_1),"Drinks");
      this->registerCallback(std::bind(&SpeechBasics::name, this, ph::_1),"Name");
      this->registerCallback(std::bind(&SpeechBasics::askdrink, this, ph::_1),"Ask Drink");
      this->registerCallback(std::bind(&SpeechBasics::introduce, this, ph::_1),"Introduce Guest");
    }

    void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[SpeechBasics] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[SpeechBasics] welcomeIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[SpeechBasics] drinks: intent [%s]", result.intent.c_str());
      setOutput("str", result.intent.param_);
      speak(result.fulfillment_text);
    }

    void name(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[SpeechBasics] name: intent [%s]", result.intent.c_str());
      setOutput("str", result.intent.param_);
      speak(result.fulfillment_text);
    }

    void introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
        std::ostringstream oss;
        std::string phrase;
        std::string str;

        getInput("Info", str);

        oss << "Their name is " << str.name;
        phrase = oss.str();
        RCLCPP_INFO(this->get_logger(), "[SpeechBasics] introduce: intent [%s]", result.intent.c_str());
        speak(result.fulfillment_text);
        speak(phrase);
    }

    void askdrink(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
        std::ostringstream oss;
        std::string phrase;
        std::string str;

        getInput("Info", str);

        oss << "I want a " << str.name << " for our guest";
        phrase = oss.str();
        RCLCPP_INFO(this->get_logger(), "[SpeechBasics] askdrink: intent [%s]", result.intent.c_str());
        speak(result.fulfillment_text);
        speak(phrase);
    }

  private:
};
}  // namespace gb_dialog