// src/console_panel.cpp
#include "include/console_panel.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <rviz_common/display_context.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace rviz_plugins
{

ConsolePanel::ConsolePanel(QWidget* parent) : rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout(this);

  output_text_ = new QPlainTextEdit;
  output_text_->setReadOnly(true);
  layout->addWidget(output_text_);

  QHBoxLayout * input_layout = new QHBoxLayout;
  input_line_ = new QLineEdit;
  send_button_ = new QPushButton("Send");
  input_layout->addWidget(input_line_);
  input_layout->addWidget(send_button_);
  layout->addLayout(input_layout);

  connect(input_line_, &QLineEdit::returnPressed, this, &ConsolePanel::onEnterCommand);
  connect(send_button_, &QPushButton::clicked, this, &ConsolePanel::onEnterCommand);

  process_ = new QProcess(this);
  connect(process_, &QProcess::readyReadStandardOutput, this, &ConsolePanel::handleProcessOutput);
  connect(process_, &QProcess::readyReadStandardError, this, &ConsolePanel::handleProcessOutput);
}

ConsolePanel::~ConsolePanel()
{
  if (process_->state() != QProcess::NotRunning) {
    process_->kill();
  }
}

void ConsolePanel::onInitialize()
{
  ros_node_abstraction_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = ros_node_abstraction_->get_raw_node();

  rclcpp::QoS qos(rclcpp::KeepLast(1000));
  qos.reliable().transient_local(); 
  log_sub_ = node->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", qos,
      std::bind(&ConsolePanel::rosoutCallback, this, std::placeholders::_1));
}
void ConsolePanel::rosoutCallback(const rcl_interfaces::msg::Log::SharedPtr msg)
{
  std::string level_str;
  switch (msg->level) {
    case rcl_interfaces::msg::Log::DEBUG: level_str = "[DEBUG]"; break;
    case rcl_interfaces::msg::Log::INFO:  level_str = "[INFO] "; break;
    case rcl_interfaces::msg::Log::WARN:  level_str = "[WARN] "; break;
    case rcl_interfaces::msg::Log::ERROR: level_str = "[ERROR]"; break;
    case rcl_interfaces::msg::Log::FATAL: level_str = "[FATAL]"; break;
    default: level_str = "[UNK]  "; break;
  }
  double time_sec = msg->stamp.sec + msg->stamp.nanosec / 1e9;
  QString text = QString("[%1] %2 %3: %4")
                   .arg(time_sec, 0, 'f', 3)
                   .arg(level_str.c_str())
                   .arg(QString::fromStdString(msg->name))
                   .arg(QString::fromStdString(msg->msg)); 

  QMetaObject::invokeMethod(output_text_, [this, text]() {
      output_text_->appendPlainText(text);
      output_text_->ensureCursorVisible();
  }, Qt::AutoConnection);
}
void ConsolePanel::onEnterCommand()
{
  QString cmd = input_line_->text().trimmed();
  if (cmd.isEmpty()) {
    return;
  }

  output_text_->appendPlainText(QString("$ %1").arg(cmd));
  output_text_->ensureCursorVisible();

  input_line_->clear();

  if (process_->state() != QProcess::NotRunning) {
    process_->kill();
  }
  process_->start(QString::fromLatin1("bash"), QStringList() << "-c" << cmd);
}
void ConsolePanel::handleProcessOutput()
{
  QByteArray outBytes = process_->readAllStandardOutput();
  QByteArray errBytes = process_->readAllStandardError();
  QString output;
  if (!outBytes.isEmpty()) {
    output.append(QString::fromLocal8Bit(outBytes));
  }
  if (!errBytes.isEmpty()) {
    QString errText = QString::fromLocal8Bit(errBytes);
    QStringList lines = errText.split('\n', QString::SkipEmptyParts);
    for (const QString &line : lines) {
      output.append(QString("[stderr] %1\n").arg(line));
    }
  }
  if (!output.isEmpty()) {
    output_text_->insertPlainText(output);
    output_text_->ensureCursorVisible();
  }
}

}  // namespace aviz_plugins
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ConsolePanel, rviz_common::Panel)
