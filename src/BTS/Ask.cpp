#include "gb_dialog/speech_basics_node.hpp"
#include "sound_play.hpp"
#include <string>
#include <sstream>

using namespace std::placeholders;
namespace ph = std::placeholders;

namespace gb_dialog
{

    SpeechBasicsNode::SpeechBasicsNode(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config),
    firsttick_(true)
    {
    node_ = rclcpp::Node::make_shared("speech_basics_node");
    dialog_interface_ = std::make_shared<DialogInterface>(node_);
    dialog_interface_->registerCallback(std::bind(&SpeechBasicsNode::noIntentCB, this, ph::_1));
    dialog_interface_->registerCallback(std::bind(&SpeechBasicsNode::welcomeIntentCB, this, ph::_1), "Default Welcome Intent");
    dialog_interface_->registerCallback(std::bind(&SpeechBasicsNode::drinks, this, ph::_1), "Drinks");
    dialog_interface_->registerCallback(std::bind(&SpeechBasicsNode::name, this, ph::_1), "Name");
    dialog_interface_->registerCallback(std::bind(&SpeechBasicsNode::askdrink, this, ph::_1), "Ask Drink");
    dialog_interface_->registerCallback(std::bind(&SpeechBasicsNode::introduce, this, ph::_1), "Introduce Guest");

    failed_ = false;

    forwarder = std::make_shared<gb_dialog::Speech>();
    forwarder->listen();
    forwarder->done();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(forwarder);
    }


    
// BT::PortsList SpeechBasicsNode::providedPorts()

// {
// return{};
// }


void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] noIntentCB: intent [%s]", result.intent.c_str());
    failed_ = true;
}

void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] welcomeIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    failed_ = false;
}

void drinks(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] drinks: intent [%s]", result.intent.c_str());
    setOutput("str", result.intent.param_);
    speak(result.fulfillment_text);
    failed_ = false;
}

void name(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] name: intent [%s]", result.intent.c_str());
    setOutput("str", result.intent.param_);
    speak(result.fulfillment_text);
    failed_ = false;
}

void introduce(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
    std::ostringstream oss;
    std::string phrase;
    std::string str;

    getInput("Info", str);

    oss << "This is " << str.name;
    phrase = oss.str();
    RCLCPP_INFO(this->get_logger(), "[SpeechBasics] introduce: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    speak(phrase);
    failed_ = false;
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
    failed_ = false;
}

BT::NodeStatus SpeechBasicsNode::tick()
{
    if (failed_)
    {
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        return BT::NodeStatus::SUCCESS;
    }
}

}  // namespace gb_dialog

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<gb_dialog::SpeechBasicsNode>(name, config);
  };

  factory.registerBuilder<gb_dialog::SpeechBasicsNode>("SpeechBasics", builder);
}
