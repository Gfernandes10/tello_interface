#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tello_msgs/srv/tello_action.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <cmath>
#include <chrono>
#include <mutex>
#include <condition_variable>

// Serviço customizado para comando senoidal
#include "tello_interface/srv/sine_command.hpp"

class TelloInterfaceNode : public rclcpp::Node {
public:
  TelloInterfaceNode() : Node("tello_interface_node"), running_(true), sine_running_(false) {
    // === Placeholders de parâmetros ===
    // this->declare_parameter<std::string>("param_name", "default_value");
    
    

    // === Placeholders de publishers ===
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/drone1/cmd_vel", 10);
    
    // === Placeholders de subscribers ===
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic_name", 10,
    //   std::bind(&TelloInterfaceNode::subscriber_callback, this, std::placeholders::_1));

    // === Placeholders de serviços ===
    sine_service_ = this->create_service<tello_interface::srv::SineCommand>(
      "sine_command",
      std::bind(&TelloInterfaceNode::handle_sine_command, this, std::placeholders::_1, std::placeholders::_2)
    );

    // === Placeholders de actions ===
    // action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
    //   this, "action_name",
    //   std::bind(&TelloInterfaceNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    //   std::bind(&TelloInterfaceNode::cancel_callback, this, std::placeholders::_1),
    //   std::bind(&TelloInterfaceNode::accept_goal_callback, this, std::placeholders::_1));

    // Cliente para takeoff/land
    tello_action_client_ = this->create_client<tello_msgs::srv::TelloAction>("/drone1/tello_action");

    // Thread para ler teclado
    keyboard_thread_ = std::thread(&TelloInterfaceNode::keyboard_loop, this);
    RCLCPP_INFO(this->get_logger(), "Tello Interface Node started! Use WASD para mover, T para decolar, L para pousar, Q para sair.");
  }

  ~TelloInterfaceNode() override {
    running_ = false;
    stop_sine();
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
  }

private:
  // === Variables ===
  std::thread keyboard_thread_;
  std::atomic<bool> running_;
  std::thread sine_thread_;
  std::atomic<bool> sine_running_;
  std::mutex sine_mutex_;
  std::condition_variable sine_cv_;
  std::string sine_axis_ = "x"; // "x", "y", "z", "angular_z"
  int sine_type_ = 1;

  // === Parameters ===

  // === Publishers ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // === Subscribers ===
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // === Services and Actions ===
  rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr tello_action_client_;
  rclcpp::Service<tello_interface::srv::SineCommand>::SharedPtr sine_service_;

  /**
   * @brief Thread loop to read keyboard input and control the drone.
   *        Publishes Twist messages to /drone1/cmd_vel and calls takeoff/land services.
   *        Exits on 'q'.
   * @note No arguments. Runs as a member thread.
   * @return void
   */
  void keyboard_loop() {
    set_terminal_mode(false);
    geometry_msgs::msg::Twist current_twist;
    while (running_) {
      fd_set set;
      struct timeval timeout;
      FD_ZERO(&set);
      FD_SET(STDIN_FILENO, &set);
      timeout.tv_sec = 0;
      timeout.tv_usec = 1000;
      int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
      current_twist = geometry_msgs::msg::Twist();
      if (rv > 0) {
        char c = getchar();
        // Interrompe qualquer senoide ao pressionar qualquer tecla
        stop_sine();
        if (c == 'w') { current_twist.linear.x = 0.1; }
        else if (c == 's') { current_twist.linear.x = -0.1; }
        else if (c == 'a') { current_twist.linear.y = 0.1;  }
        else if (c == 'd') { current_twist.linear.y = -0.1; }
        else if (c == 'i') { current_twist.linear.z = 0.1; }
        else if (c == 'k') { current_twist.linear.z = -0.1; }
        else if (c == 'j') { current_twist.angular.z = 0.1;}
        else if (c == 'l') { current_twist.angular.z = -0.1;}
        else if (c == 'q') { running_ = false; break; }
        else if (c == 'e') { call_tello_action("takeoff"); }
        else if (c == 'c') { call_tello_action("land"); }
      }
      else if (rv == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading from stdin");
        break;
      }
      else if (rv == 0) {
        // Timeout, no input
        current_twist = geometry_msgs::msg::Twist();
      }
      send_command(current_twist);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    set_terminal_mode(true);
  }
  /**
   * @brief Sends a command to the drone by publishing a Twist message.
   * @param twist The Twist message containing the command.
   * @return void
   */
  void send_command(const geometry_msgs::msg::Twist &twist) {
    cmd_vel_pub_->publish(twist);
  }

  /**
   * @brief Calls the Tello action service with the given command ("takeoff" or "land").
   * @param cmd The command string to send to the service (e.g., "takeoff", "land").
   * @return void
   */
  void call_tello_action(const std::string &cmd) {
    if (!tello_action_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Tello action service not available");
      return;
    }
    auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
    request->cmd = cmd;
    auto result = tello_action_client_->async_send_request(request);
  }

  /**
   * @brief Sets the terminal mode to raw (no echo, no canonical mode) or restores it.
   * @param reset If true, restores the terminal to its original mode. If false, sets to raw mode.
   * @return void
   */
  void set_terminal_mode(bool reset) {
    static struct termios oldt, newt;
    if (!reset) {
      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
  }

  void handle_sine_command(const std::shared_ptr<tello_interface::srv::SineCommand::Request> req,
                          std::shared_ptr<tello_interface::srv::SineCommand::Response> res) {
    std::lock_guard<std::mutex> lock(sine_mutex_);
    stop_sine();
    sine_axis_ = req->axis;
    sine_type_ = req->sine_id;
    sine_running_ = true;
    sine_thread_ = std::thread(&TelloInterfaceNode::sine_loop, this);
    res->success = true;
  }

  void stop_sine() {
    sine_running_ = false;
    sine_cv_.notify_all();
    if (sine_thread_.joinable()) sine_thread_.join();
    send_command(geometry_msgs::msg::Twist());
  }

  void sine_loop() {
    int freq = 100;
    double t = 0.0;
    double dt = 1.0 / freq;
    rclcpp::Rate rate(freq);
    while (sine_running_) {
      geometry_msgs::msg::Twist twist;
      double value = 0.0;
      if (sine_type_ == 1) value = (0.1/4.5) * (3*std::sin(0.2*M_PI*t) + std::sin(0.6*M_PI*t) + 0.5*std::sin(M_PI*t));
      else if (sine_type_ == 2) value = 0.2 * std::sin(2*t);
      else if (sine_type_ == 3) value = 0.2 * std::sin(t) + 0.1 * std::sin(3*t);

      if (sine_axis_ == "x") twist.linear.x = value;
      else if (sine_axis_ == "y") twist.linear.y = value;
      else if (sine_axis_ == "z") twist.linear.z = value;
      else if (sine_axis_ == "angular_z") twist.angular.z = value;
      send_command(twist);
      t += dt;
      rate.sleep();
    }
    send_command(geometry_msgs::msg::Twist());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TelloInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
