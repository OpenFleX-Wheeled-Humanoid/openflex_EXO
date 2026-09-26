#pragma once

#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float32.hpp>

namespace openflex_teleop_exo_speed_panel {

class SpeedScalePanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit SpeedScalePanel(QWidget *parent = nullptr);
  ~SpeedScalePanel() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void onSliderChanged(int value);
  void onPreset10();
  void onPreset25();
  void onPreset50();
  void publishCurrentScale();

 private:
  void setupUi();
  void setupRos();
  void setScale(double scale, bool publish);
  void updateLabel();
  void publishScale(double scale);
  void scaleCallback(const std_msgs::msg::Float32::SharedPtr msg);

  static constexpr int kSliderScale = 1000;
  static constexpr double kMaxScale = 0.5;

  QSlider *scale_slider_{nullptr};
  QLabel *scale_label_{nullptr};
  QLabel *topic_label_{nullptr};
  QLabel *status_label_{nullptr};
  QPushButton *preset_10_{nullptr};
  QPushButton *preset_25_{nullptr};
  QPushButton *preset_50_{nullptr};
  QTimer *ros_spin_timer_{nullptr};
  QTimer *publish_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scale_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;

  std::string topic_{"/openflex_teleop_exo/speed_scale"};
  double scale_{0.1};
  bool suppress_slider_events_{false};
};

}  // namespace openflex_teleop_exo_speed_panel
