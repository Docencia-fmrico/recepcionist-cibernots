#include <behaviortree_cpp_v3/action_node.h>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "gb_dialog/SpeechBasics.hpp"

namespace gb_dialog {

class SpeechNode : public BT::ActionNode {
 public:
  SpeechNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ActionNode(name, config) {
    // Inicializar variables compartidas
    //this->getInput<std::string>("name", name_);
    //this->getInput<std::string>("drink", drink_);

    // Crear objeto rclcpp::Node
    node_ = rclcpp::Node::make_shared("speech_node");
  }

  static BT::PortsList providedPorts() {
    /*// Definir puertos de entrada y salida
    return {
        BT::InputPort<std::string>("name", "Name of the guest"),
        BT::InputPort<std::string>("drink", "Drink of the guest"),
    };*/
  }

  BT::NodeStatus tick() override {
    // Crear suscripción al topic "/speech"
    auto subscriber = node_->create_subscription<gb_dialog::Speech>(
        "/speech",
        10,
        [this](const gb_dialog::Speech::SharedPtr msg) {
          // Actualizar variables compartidas
          if (msg->intent == "Drinks") {
            drink_ = msg->param;
          } else if (msg->intent == "Name") {
            name_ = msg->param;
          } else if(msg->intent == NULL) {
                return BT::NodeStatus::RUNNING;
          }
        });

    // Esperar hasta que se reciba un mensaje
    rclcpp::spin_until_future_complete(node_->get_node_base_interface()->get_context()->get_default_context().get(),
                                       subscriber->get_subscription_wait_set()->wait());

    return BT::NodeStatus::SUCCESS;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::string drink_;
};

}  // namespace gb_dialog
