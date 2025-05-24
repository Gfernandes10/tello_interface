#include <rclcpp/rclcpp.hpp>
// Inclua as mensagens, serviços e actions necessários
// #include "std_msgs/msg/string.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "example_interfaces/action/fibonacci.hpp"

class TelloInterfaceNode : public rclcpp::Node {
public:
  TelloInterfaceNode() : Node("tello_interface_node") {
    // === Placeholders de parâmetros ===
    // this->declare_parameter<std::string>("param_name", "default_value");

    // === Placeholders de publishers ===
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", 10);

    // === Placeholders de subscribers ===
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic_name", 10,
    //   std::bind(&TelloInterfaceNode::subscriber_callback, this, std::placeholders::_1));

    // === Placeholders de serviços ===
    // service_ = this->create_service<std_srvs::srv::Empty>(
    //   "service_name",
    //   std::bind(&TelloInterfaceNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // === Placeholders de actions ===
    // action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
    //   this, "action_name",
    //   std::bind(&TelloInterfaceNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    //   std::bind(&TelloInterfaceNode::cancel_callback, this, std::placeholders::_1),
    //   std::bind(&TelloInterfaceNode::accept_goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Tello Interface Node started!");
  }

private:
  // === Placeholders de membros para publishers, subscribers, serviços e actions ===
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  // rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr action_server_;

  // === Placeholders de callbacks ===
  // void subscriber_callback(const std_msgs::msg::String::SharedPtr msg) {
  //   // Lógica do subscriber
  // }

  // void service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  //                       std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  //   // Lógica do serviço
  // }

  // rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid,
  //                                           std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal) {
  //   // Lógica do goal
  //   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  // }

  // rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
  //   // Lógica do cancelamento
  //   return rclcpp_action::CancelResponse::ACCEPT;
  // }

  // void accept_goal_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
  //   // Lógica de execução do goal
  // }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TelloInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
