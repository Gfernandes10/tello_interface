#include "tello_interface/teleop_widget.h"
#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>

TeleopWidget::TeleopWidget(std::function<void(const geometry_msgs::msg::Twist&)> send_cmd,
               std::function<void(const std::string&)> call_action,
               std::function<void()> stop_sine,
               std::function<bool()> is_sine_running,
               QWidget* parent)
    : QWidget(parent), send_command_(send_cmd), call_action_(call_action), stop_sine_(stop_sine), is_sine_running_(is_sine_running) {
    setWindowTitle("Tello Teleop - Keyboard");
    setFixedSize(350, 180);
    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* label = new QLabel("Use WASD to move, I/K to go up/down, J/L to rotate,\nE: takeoff, C: land, Q: quit\n\nClick this window and use the keyboard.", this);
    label->setWordWrap(true);
    layout->addWidget(label);
    setLayout(layout);
    setFocusPolicy(Qt::StrongFocus);
    cmd_timer_ = new QTimer(this);
    connect(cmd_timer_, &QTimer::timeout, this, &TeleopWidget::send_current_twist);
    cmd_timer_->setInterval(50); // 20Hz
}

void TeleopWidget::keyPressEvent(QKeyEvent* event) {
    double speed = 0.2;
    bool changed = false;
    switch (event->key()) {
        case Qt::Key_W: current_twist_.linear.x = speed; changed = true; break;
        case Qt::Key_S: current_twist_.linear.x = -speed; changed = true; break;
        case Qt::Key_A: current_twist_.linear.y = speed; changed = true; break;
        case Qt::Key_D: current_twist_.linear.y = -speed; changed = true; break;
        case Qt::Key_I: current_twist_.linear.z = speed; changed = true; break;
        case Qt::Key_K: current_twist_.linear.z = -speed; changed = true; break;
        case Qt::Key_J: current_twist_.angular.z = 2* speed; changed = true; break;
        case Qt::Key_L: current_twist_.angular.z = -2* speed; changed = true; break;
        case Qt::Key_E: call_action_("takeoff"); break;
        case Qt::Key_C: call_action_("land"); break;
        case Qt::Key_Q: QApplication::quit(); break;
        default: break;
    }
    if (changed) {
        if (is_sine_running_()) stop_sine_();
        if (!cmd_timer_->isActive()) cmd_timer_->start();
        send_current_twist();
        key_active_ = true;
    }
}

void TeleopWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->isAutoRepeat()) return;
    bool relevant = false;
    switch (event->key()) {
      case Qt::Key_W:
      case Qt::Key_S:
        current_twist_.linear.x = 0.0; relevant = true; break;
      case Qt::Key_A:
      case Qt::Key_D:
        current_twist_.linear.y = 0.0; relevant = true; break;
      case Qt::Key_I:
      case Qt::Key_K:
        current_twist_.linear.z = 0.0; relevant = true; break;
      case Qt::Key_J:
      case Qt::Key_L:
        current_twist_.angular.z = 0.0; relevant = true; break;
      default: break;
    }
    if (relevant) {
        send_current_twist();
        // Se todas as componentes forem zero, para o timer
        if (current_twist_.linear.x == 0.0 && current_twist_.linear.y == 0.0 && current_twist_.linear.z == 0.0 && current_twist_.angular.z == 0.0) {
            cmd_timer_->stop();
        }
    }
}

void TeleopWidget::send_current_twist() {
    send_command_(current_twist_);
}
