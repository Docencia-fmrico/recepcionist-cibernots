#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace gb_dialog
{

class ExampleDF: public DialogInterface
{
  public:
    ExampleDF()
    {
      this->registerCallback(std::bind(&ExampleDF::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&ExampleDF::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");
      this->registerCallback(
        std::bind(&ExampleDF::drinks, this, ph::_1),
        "Drinks");
      this->registerCallback(
        std::bind(&ExampleDF::name, this, ph::_1),
        "Name");
      this->registerCallback(
        std::bind(&ExampleDF::askdrink, this, ph::_1),
        "Ask Drink");
      this->registerCallback(
        std::bind(&ExampleDF::introduce, this, ph::_1),
        "Introduce Guest");
    }

    void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[ExampleDF] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[ExampleDF] welcomeIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[ExampleDF] drinks: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void name(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[ExampleDF] name: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[ExampleDF] introduce: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void askdrink(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(this->get_logger(), "[ExampleDF] askdrink: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

  private:
};
}  // namespace gb_dialog