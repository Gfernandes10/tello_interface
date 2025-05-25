#ifndef TELLO_INTERFACE_TELEOP_WIDGET_H
#define TELLO_INTERFACE_TELEOP_WIDGET_H

#include <QWidget>
#include <QKeyEvent>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>

class TeleopWidget : public QWidget {
  Q_OBJECT
public:
  TeleopWidget(std::function<void(const geometry_msgs::msg::Twist&)> send_cmd,
               std::function<void(const std::string&)> call_action,
               std::function<void()> stop_sine,
               std::function<bool()> is_sine_running,
               QWidget* parent = nullptr);
protected:
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
private:
  QTimer* cmd_timer_ = nullptr;
  geometry_msgs::msg::Twist current_twist_;
  bool key_active_ = false;
  void send_current_twist();
public:
  std::function<void(const geometry_msgs::msg::Twist&)> send_command_;
  std::function<void(const std::string&)> call_action_;
  std::function<void()> stop_sine_;
  std::function<bool()> is_sine_running_;
};

#endif // TELLO_INTERFACE_TELEOP_WIDGET_H
