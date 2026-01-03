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
#include "tello_interface/srv/sine_command.hpp"
#include "../ros_utils/csv_logger.h"
#include <QApplication>
#include <QWidget>
#include <QKeyEvent>
#include <QLabel>
#include <QVBoxLayout>
#include "tello_interface/teleop_widget.h"
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

class TelloInterfaceNode : public rclcpp::Node {
  public:
  // === Variables ===
  std::thread keyboard_thread_;
  std::atomic<bool> running_;
  std::thread sine_thread_;
  std::atomic<bool> sine_running_;
  std::mutex sine_mutex_;
  std::condition_variable sine_cv_;
  std::string sine_axis_ = "x"; // "x", "y", "z", "angular_z"
  std::string command_topic_ = "/tello/cmd_vel";
  std::string filtered_pose_topic_ = "/tello/filtered_pose";
  int sine_type_ = 1;
  bool enable_keyboard_control = true;

  // Service name parameters (defaults can be overridden via ROS2 parameters / launch files)
  std::string takeoff_service_name_ = "/tello/takeoff";
  std::string land_service_name_ = "/tello/land";
  std::string tello_action_service_name_ = "/drone1/tello_action";
  
  // === Parameters ===
  
  // === Publishers ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // === Subscribers ===
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_pose_sub_;
  
  // === Services and Actions ===
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
  rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr tello_action_client_;
  rclcpp::Service<tello_interface::srv::SineCommand>::SharedPtr sine_service_;

  // === Loggers ===
  std::unique_ptr<CSVLogger> csv_logger_u_control_;
  std::unique_ptr<CSVLogger> csv_logger_states_;
  std::unique_ptr<CSVLogger> csv_logger_filtered_pose_;

public:
  TelloInterfaceNode(bool use_gui = false) : Node("tello_interface_node"), running_(true), sine_running_(false) {
    const char* workspace_name = std::getenv("MY_WORKSPACE_NAME");
    if (workspace_name == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Environment variable MY_WORKSPACE_NAME is not set.");
        return;
    }

    // === Placeholders de parâmetros ===
    // this->declare_parameter<std::string>("param_name", "default_value");

    // declare topic parameters so they can be set from launch files
    this->declare_parameter<std::string>("command_topic", command_topic_);
    this->declare_parameter<std::string>("filtered_pose_topic", filtered_pose_topic_);
    // declare parameters for service names so they can be set via launch files
    this->declare_parameter<std::string>("takeoff_service_name", takeoff_service_name_);
    this->declare_parameter<std::string>("land_service_name", land_service_name_);
    this->declare_parameter<std::string>("tello_action_service_name", tello_action_service_name_);

    // get parameter values (these will override the defaults above if provided)
    this->get_parameter("command_topic", command_topic_);
    this->get_parameter("filtered_pose_topic", filtered_pose_topic_);
    this->get_parameter("takeoff_service_name", takeoff_service_name_);
    this->get_parameter("land_service_name", land_service_name_);
    this->get_parameter("tello_action_service_name", tello_action_service_name_);
    

    // === Placeholders de publishers ===
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);
    
    // === Placeholders de subscribers ===
    filtered_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      filtered_pose_topic_, 10,
      std::bind(&TelloInterfaceNode::filtered_pose_topic_callback, this, std::placeholders::_1));

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

    // === Placeholders de clientes ===


    takeoff_client_ = this->create_client<std_srvs::srv::Trigger>(takeoff_service_name_);
    land_client_ = this->create_client<std_srvs::srv::Trigger>(land_service_name_);
    tello_action_client_ = this->create_client<tello_msgs::srv::TelloAction>(tello_action_service_name_);

    // === Placeholders de loggers ===
    std::vector<std::string> header_u_control = std::vector<std::string>{"timestamp", 
                                                                                "topic", 
                                                                                "ux",  
                                                                                "uy",  
                                                                                "uz", 
                                                                                "uyaw"};
    csv_logger_u_control_ = std::make_unique<CSVLogger>(workspace_name, 
                                                        "tello_interface", 
                                                        "u_control", 
                                                        header_u_control);
    std::vector<std::string> header_filtered_pose_ = std::vector<std::string>{"timestamp", 
                                                                        "topic", 
                                                                        "x",  
                                                                        "dx",  
                                                                        "y", 
                                                                        "dy",
                                                                        "z",
                                                                        "dz",
                                                                        "roll",
                                                                        "pitch",
                                                                        "yaw",
                                                                        "p",  
                                                                        "q",
                                                                        "r"};
    csv_logger_filtered_pose_ = std::make_unique<CSVLogger>(workspace_name,
                                                        "tello_interface", 
                                                        "filtered_pose", 
                                                        header_filtered_pose_);                                                                    

    // === Placeholders de timers ===
    if (!use_gui) {
      keyboard_thread_ = std::thread(&TelloInterfaceNode::keyboard_loop, this);
      RCLCPP_INFO(this->get_logger(), "Tello Interface Node started! Use WASD para mover, T para decolar, L para pousar, Q para sair.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Tello Interface Node started em modo GUI! Use o teclado na janela para controlar.");
    }
  }

  ~TelloInterfaceNode() override {
    running_ = false;
    stop_sine();
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
  }

public:


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
    current_twist = geometry_msgs::msg::Twist();
    double speed = 0.1;
    while (running_) {
      fd_set set;
      struct timeval timeout;
      FD_ZERO(&set);
      FD_SET(STDIN_FILENO, &set);
      timeout.tv_sec = 0;
      timeout.tv_usec = 1000;
      int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
      if (rv > 0) {
        char c = getchar();
        // Interrompe qualquer senoide ao pressionar qualquer tecla
        if (sine_running_){ stop_sine(); }
        if (c == 'w') { current_twist.linear.x = speed; }
        else if (c == 's') { current_twist.linear.x = -speed; }
        else if (c == 'a') { current_twist.linear.y = speed;  }
        else if (c == 'd') { current_twist.linear.y = -speed; }
        else if (c == 'i') { current_twist.linear.z = speed; }
        else if (c == ',') { current_twist.linear.z = -speed; }
        else if (c == 'j') { current_twist.angular.z = speed;}
        else if (c == 'l') { current_twist.angular.z = -speed;}
        else if (c == 'q') { running_ = false; break; }
        else if (c == 'e') { call_tello_action("takeoff"); }
        else if (c == 'c') { call_tello_action("land"); }
        else if (c == 'k') {current_twist = geometry_msgs::msg::Twist(); }
        enable_keyboard_control = true;
      }
      else if (rv == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading from stdin");
        break;
      }
      if (enable_keyboard_control) {
        // Publish the current twist command
        // RCLCPP_INFO(this->get_logger(), "keyboard loop");
        send_command(current_twist);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    set_terminal_mode(true);
  }
  /**
   * @brief Sends a command to the drone by publishing a Twist message.
   * @param twist The Twist message containing the command.
   * @return void
   */
  void send_command(const geometry_msgs::msg::Twist &twist) {
    std::ostringstream timestamp_stream;
    timestamp_stream << std::fixed << std::setprecision(6) << this->now().seconds();
    std::string timestamp = timestamp_stream.str();
    std::vector<std::variant<std::string, double>> data_u_control = {timestamp, 
                                                                    command_topic_, 
                                                                    twist.linear.x, 
                                                                    twist.linear.y,
                                                                    twist.linear.z,
                                                                    twist.angular.z};
    csv_logger_u_control_->writeCSV(data_u_control);
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
    tello_action_client_->async_send_request(request);
    // if (cmd == "takeoff") {
    //   if (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_WARN(this->get_logger(), "Takeoff service not available");
    //     return;
    //   }
    //   auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    //   takeoff_client_->async_send_request(request);
    // } else if (cmd == "land") {
    //   if (!land_client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_WARN(this->get_logger(), "Land service not available");
    //     return;
    //   }
    //   auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    //   land_client_->async_send_request(request);
    // } else {
    //   if (!tello_action_client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_WARN(this->get_logger(), "Tello action service not available");
    //     return;
    //   }
    //   auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
    //   request->cmd = cmd;
    //   tello_action_client_->async_send_request(request);
    // }
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
    enable_keyboard_control = false;
    sine_thread_ = std::thread(&TelloInterfaceNode::sine_loop, this);
    res->success = true;
  }

  void stop_sine() {
    sine_running_ = false;
    sine_cv_.notify_all();
    if (sine_thread_.joinable()) sine_thread_.join();
    // RCLCPP_INFO(this->get_logger(), "stop sine loop");
    send_command(geometry_msgs::msg::Twist());
  }

  void sine_loop() {
    int freq = 50;
    double t = 0.0;
    double dt = 1.0 / freq;
    rclcpp::Rate rate(freq);
    double value = 0.0;
    while (sine_running_) {
      geometry_msgs::msg::Twist twist;
      if (sine_type_ == 1) value = (0.4/4.5) * (3*std::sin(0.2*M_PI*t) + std::sin(0.6*M_PI*t) + 0.5*std::sin(M_PI*t));
      else if (sine_type_ == 2) value = (0.2)*(std::sin(0.2*M_PI*t) + std::sin(0.4*M_PI*t));
      else if (sine_type_ == 3) value = (std::sin(t) >= 0 ? 0.3 : -0.3);

      if (sine_axis_ == "x") twist.linear.x = value;
      else if (sine_axis_ == "y") twist.linear.y = value;
      else if (sine_axis_ == "z") twist.linear.z = value;
      else if (sine_axis_ == "angular_z") twist.angular.z = value;
      // RCLCPP_INFO(this->get_logger(), "Sine loop");
      send_command(twist);
      t += dt;
      rate.sleep();
    }
    // send_command(geometry_msgs::msg::Twist());
  }

  /**
   * @brief Callback function for the filtered pose topic (Odometry).
   * @param msg The message received from the filtered pose topic.
   * @return void
   */
  void filtered_pose_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::vector<std::variant<std::string, double>> data_filtered_pose = {
      std::to_string(this->now().seconds()),
      filtered_pose_topic_,
      msg->pose.pose.position.x,
      msg->twist.twist.linear.x,
      msg->pose.pose.position.y,
      msg->twist.twist.linear.y,
      msg->pose.pose.position.z,
      msg->twist.twist.linear.z,
      0.0, // roll (placeholder)
      0.0, // pitch (placeholder)
      0.0, // yaw (placeholder)
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z
    };
    // Extrai yaw, pitch, roll dos quaternions
    const auto& q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = std::abs(sinp) >= 1 ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    data_filtered_pose[8] = roll;
    data_filtered_pose[9] = pitch;
    data_filtered_pose[10] = yaw;
    csv_logger_filtered_pose_->writeCSV(data_filtered_pose);
  }
};

int main(int argc, char ** argv)
{
  bool use_gui = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--gui") use_gui = true;
  }
  rclcpp::init(argc, argv);
  if (use_gui) {
    QApplication app(argc, argv);
    auto node = std::make_shared<TelloInterfaceNode>(true);
    TeleopWidget widget(
      [node](const geometry_msgs::msg::Twist& t){ node->send_command(t); },
      [node](const std::string& cmd){ node->call_tello_action(cmd); },
      [node](){ node->stop_sine(); },
      [node](){ return node->sine_running_.load(); }
    );
    widget.show();
    std::thread spin_thread([&](){ rclcpp::spin(node); });
    int ret = app.exec();
    node->running_ = false;
    spin_thread.join();
    rclcpp::shutdown();
    return ret;
  } else {
    auto node = std::make_shared<TelloInterfaceNode>(false);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
}
