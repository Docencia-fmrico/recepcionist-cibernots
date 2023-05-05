#include <gb_dialog/DialogInterface.h>
#include <string>
#include <speech/speech.h>

namespace ph = std::placeholders;

namespace gb_dialog
{
class Speech: public DialogInterface
{
  public:
    Speech(): nh_()
    {
      this->registerCallback(std::bind(&Speech::noIntentCB, this, ph::_1), "Default Fallback Intent");
      this->registerCallback(
        std::bind(&Speech::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");
      this->registerCallback(std::bind(&Speech::luggageIntentCB, this, ph::_1), "Luggage Intent");

      service_ = nh_.advertiseService("speech", &gb_dialog::Speech::startDialog, this);
      //response_.query_text = '';
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Speech] noIntentCB: intent [%s]", result.intent.c_str());
      response_ = result;
      speak(result.fulfillment_text);
      listen();
    }

    void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Speech] welcomeIntentCB: intent [%s]", result.intent.c_str());
      response_ = result;
      speak(result.fulfillment_text);
    }

    void luggageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std::string color;

      for (const auto & param : result.parameters) {
          for (const auto & value : param.value) {
            color = value;
          }
      }

      ROS_INFO("luggageIntentCB: intent [%s]", result.intent.c_str());
      ROS_INFO("Color: %s", color.c_str());
      response_ = result;
      speak(result.fulfillment_text);
      if (color.empty()) {
        listen();
      }
    }

    bool startDialog(speech::speech::Request &req, speech::speech::Response &res){
      ros::Time stamp;

      speak(req.speak_text);
      stamp = ros::Time::now();
      while ((ros::Time::now() - stamp).sec != static_cast<int>(req.speak_text.length() * 0.10) ) {}

      if (req.action == 1) {
        listen();
        //while (response_.query_text == ''){}
        for (const auto & param : response_.parameters) {
          for (const auto & value : param.value) {
            res.response = value;
          }
        }
      }

      return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    dialogflow_ros_msgs::DialogflowResult response_;
};
}  // namespace gb_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speech_srv_node"); 
  gb_dialog::Speech forwarder;

  ROS_INFO("speech ready");
  
  ros::spin();
  return 0;
}