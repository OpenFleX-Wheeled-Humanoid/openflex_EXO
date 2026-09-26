#include "openflex_teleop_exo_speed_panel/speed_scale_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace openflex_teleop_exo_speed_panel {

namespace {
double clampScale(double value, double max_scale) {
  return std::max(0.0, std::min(max_scale, value));
}
}  // namespace

SpeedScalePanel::SpeedScalePanel(QWidget *parent) : rviz_common::Panel(parent) {
  setupUi();
}

void SpeedScalePanel::setupUi() {
  auto *root = new QVBoxLayout();
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  auto *title = new QLabel(QStringLiteral("<h3>OpenFlex EXO 速度倍率</h3>"));
  title->setAlignment(Qt::AlignCenter);
  root->addWidget(title);

  topic_label_ = new QLabel(QString::fromStdString(topic_));
  topic_label_->setAlignment(Qt::AlignCenter);
  topic_label_->setStyleSheet("color:#666;font-size:11px;");
  root->addWidget(topic_label_);

  auto *group = new QGroupBox(QStringLiteral("倍率 0.00 - 0.50"));
  auto *group_layout = new QVBoxLayout();

  scale_slider_ = new QSlider(Qt::Horizontal);
  scale_slider_->setRange(0, static_cast<int>(std::round(kMaxScale * kSliderScale)));
  scale_slider_->setTickInterval(100);
  scale_slider_->setTickPosition(QSlider::TicksBelow);
  scale_slider_->setValue(static_cast<int>(std::round(scale_ * kSliderScale)));
  group_layout->addWidget(scale_slider_);

  scale_label_ = new QLabel();
  scale_label_->setAlignment(Qt::AlignCenter);
  scale_label_->setStyleSheet("font-size:24px;font-weight:bold;color:#1f5f8b;");
  group_layout->addWidget(scale_label_);

  auto *preset_row = new QHBoxLayout();
  preset_10_ = new QPushButton(QStringLiteral("10%"));
  preset_25_ = new QPushButton(QStringLiteral("25%"));
  preset_50_ = new QPushButton(QStringLiteral("50%"));
  preset_row->addWidget(preset_10_);
  preset_row->addWidget(preset_25_);
  preset_row->addWidget(preset_50_);
  group_layout->addLayout(preset_row);

  group->setLayout(group_layout);
  root->addWidget(group);

  status_label_ = new QLabel(QStringLiteral("状态: 等待 RViz 初始化"));
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("background:#f3f4f6;color:#374151;padding:6px;border-radius:4px;");
  root->addWidget(status_label_);

  root->addStretch();
  setLayout(root);

  connect(scale_slider_, &QSlider::valueChanged, this, &SpeedScalePanel::onSliderChanged);
  connect(preset_10_, &QPushButton::clicked, this, &SpeedScalePanel::onPreset10);
  connect(preset_25_, &QPushButton::clicked, this, &SpeedScalePanel::onPreset25);
  connect(preset_50_, &QPushButton::clicked, this, &SpeedScalePanel::onPreset50);

  updateLabel();
}

void SpeedScalePanel::onInitialize() {
  setupRos();
  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() {
    if (node_) {
      rclcpp::spin_some(node_);
    }
  });
  ros_spin_timer_->start(30);

  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &SpeedScalePanel::publishCurrentScale);
  publish_timer_->start(500);

  publishScale(scale_);
}

void SpeedScalePanel::setupRos() {
  node_ = std::make_shared<rclcpp::Node>("openflex_teleop_exo_speed_panel");
  scale_pub_ = node_->create_publisher<std_msgs::msg::Float32>(topic_, 10);
  scale_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    topic_, 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { scaleCallback(msg); });
  status_label_->setText(QStringLiteral("状态: 已连接，发布速度倍率"));
}

void SpeedScalePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  float stored_scale = static_cast<float>(scale_);
  if (config.mapGetFloat("speed_scale", &stored_scale)) {
    setScale(stored_scale, false);
  }
}

void SpeedScalePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("speed_scale", static_cast<float>(scale_));
}

void SpeedScalePanel::onSliderChanged(int value) {
  if (suppress_slider_events_) {
    return;
  }
  setScale(static_cast<double>(value) / static_cast<double>(kSliderScale), true);
}

void SpeedScalePanel::onPreset10() { setScale(0.10, true); }
void SpeedScalePanel::onPreset25() { setScale(0.25, true); }
void SpeedScalePanel::onPreset50() { setScale(0.50, true); }

void SpeedScalePanel::publishCurrentScale() {
  publishScale(scale_);
}

void SpeedScalePanel::setScale(double scale, bool publish) {
  scale_ = clampScale(scale, kMaxScale);
  suppress_slider_events_ = true;
  scale_slider_->setValue(static_cast<int>(std::round(scale_ * kSliderScale)));
  suppress_slider_events_ = false;
  updateLabel();
  if (publish) {
    publishScale(scale_);
  }
}

void SpeedScalePanel::updateLabel() {
  const int percent = static_cast<int>(std::round(scale_ * 100.0));
  scale_label_->setText(QStringLiteral("%1%  (%2x)").arg(percent).arg(scale_, 0, 'f', 2));
}

void SpeedScalePanel::publishScale(double scale) {
  if (!scale_pub_) {
    return;
  }
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(clampScale(scale, kMaxScale));
  scale_pub_->publish(msg);
}

void SpeedScalePanel::scaleCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  const double incoming = clampScale(msg->data, kMaxScale);
  if (std::abs(incoming - scale_) < 1e-4) {
    return;
  }
  setScale(incoming, false);
}

}  // namespace openflex_teleop_exo_speed_panel

PLUGINLIB_EXPORT_CLASS(
  openflex_teleop_exo_speed_panel::SpeedScalePanel,
  rviz_common::Panel)
