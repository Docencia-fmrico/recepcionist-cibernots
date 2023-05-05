#ifndef SPEECH_CHAT_H
#define SPEECH_CHAT_H

#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"

namespace ph = std::placeholders;

namespace speech
{
class Chat : public gb_dialog::DialogInterface
{
  public:
    Chat();

    void askNameCB(dialogflow_ros_msgs::DialogflowResult result);

    void askDrinkCB(dialogflow_ros_msgs::DialogflowResult result);

    bool done_;
    std::string param_;
    std::string response_;
  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_param_;
};
};  // namespace speech

#endif  // SPEECH_CHAT_H