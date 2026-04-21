#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointDegGui : public QMainWindow
{
public:
  JointDegGui(std::shared_ptr<rclcpp::Node> node, QWidget * parent = nullptr)
  : QMainWindow(parent), node_(node)
  {
    publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    joint_names_ = {
      "joint11", "joint12", "joint13", "joint14", "joint15",
      "joint16", "joint17", "joint18", "joint19", "joint20"
    };

    // 每个关节的角度范围，单位：deg
    joint_limits_deg_ = {
      {-180, 180}, {-180, 180}, {-180, 180}, {-180, 180}, {-180, 180},
      {-180, 180}, {-180, 180}, {-180, 180}, {-180, 180}, {-180, 180}
    };

    current_deg_.resize(joint_names_.size(), 0.0);

    auto * central = new QWidget(this);
    auto * main_layout = new QVBoxLayout(central);

    auto * button_layout = new QHBoxLayout();
    auto * center_button = new QPushButton("Center");
    button_layout->addWidget(center_button);
    main_layout->addLayout(button_layout);

    auto * group_box = new QGroupBox("Joint Angles (deg)");
    auto * grid = new QGridLayout(group_box);

    sliders_.resize(joint_names_.size());
    value_boxes_.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto * name_label = new QLabel(QString::fromStdString(joint_names_[i]));
      auto * slider = new QSlider(Qt::Horizontal);
      auto * value_box = new QLineEdit("0.0");

      slider->setMinimum(joint_limits_deg_[i].first);
      slider->setMaximum(joint_limits_deg_[i].second);
      slider->setValue(0);

      value_box->setReadOnly(false);
      value_box->setFixedWidth(120);

      sliders_[i] = slider;
      value_boxes_[i] = value_box;

      connect(slider, &QSlider::valueChanged, this, [this, i](int value) {
        current_deg_[i] = static_cast<double>(value);
        value_boxes_[i]->setText(QString::number(current_deg_[i], 'f', 1));
      });

      connect(value_box, &QLineEdit::editingFinished, this, [this, i]() {
        bool ok = false;
        double val = value_boxes_[i]->text().toDouble(&ok);
        if (!ok) {
          value_boxes_[i]->setText(QString::number(current_deg_[i], 'f', 1));
          return;
        }

        const int min_deg = joint_limits_deg_[i].first;
        const int max_deg = joint_limits_deg_[i].second;
        if (val < min_deg) val = min_deg;
        if (val > max_deg) val = max_deg;

        current_deg_[i] = val;
        sliders_[i]->setValue(static_cast<int>(std::round(val)));
        value_boxes_[i]->setText(QString::number(current_deg_[i], 'f', 1));
      });

      int row = static_cast<int>(i);
      grid->addWidget(name_label, row, 0);
      grid->addWidget(slider, row, 1);
      grid->addWidget(value_box, row, 2);
    }

    group_box->setLayout(grid);
    main_layout->addWidget(group_box);

    setCentralWidget(central);
    setWindowTitle("角度控制 GUI");

    connect(center_button, &QPushButton::clicked, this, [this]() {
      for (size_t i = 0; i < sliders_.size(); ++i) {
        sliders_[i]->setValue(0);
        current_deg_[i] = 0.0;
        value_boxes_[i]->setText("0.0");
      }
    });

    // Qt 定时器：20 Hz 发布 joint_states
    auto * qt_timer = new QTimer(this);
    connect(qt_timer, &QTimer::timeout, this, &JointDegGui::publishJointStates);
    qt_timer->start(50);
  }

private:
  void publishJointStates()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = joint_names_;
    msg.position.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      msg.position[i] = degToRad(current_deg_[i]);
    }

    publisher_->publish(msg);
  }

  static double degToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

  std::vector<std::string> joint_names_;
  std::vector<std::pair<int, int>> joint_limits_deg_;
  std::vector<double> current_deg_;

  std::vector<QSlider *> sliders_;
  std::vector<QLineEdit *> value_boxes_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("joint_deg_gui_cpp");

  QApplication app(argc, argv);

  // ROS spin 放到单独线程
  std::thread ros_thread([&]() {
    rclcpp::spin(node);
  });

  JointDegGui window(node);
  window.resize(700, 500);
  window.show();

  int ret = app.exec();

  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  return ret;
}