#ifndef AUTOWARE_RVIZ_CONSOLE__CONSOLE_PANEL_HPP_
#define AUTOWARE_RVIZ_CONSOLE__CONSOLE_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QProcess>

namespace rviz_plugins
{

class ConsolePanel : public rviz_common::Panel
{
  Q_OBJECT  

public:
  explicit ConsolePanel(QWidget * parent = nullptr);
  ~ConsolePanel() override;

  void onInitialize() override;  

private Q_SLOTS:
  void onEnterCommand();       
  void handleProcessOutput();  

private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> ros_node_abstraction_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
  void rosoutCallback(const rcl_interfaces::msg::Log::SharedPtr msg);  

  QPlainTextEdit * output_text_;  
  QLineEdit * input_line_;        
  QPushButton * send_button_;     

  QProcess * process_;            
};

}  // namespace autoware_rviz_console

#endif  // AUTOWARE_RVIZ_CONSOLE__CONSOLE_PANEL_HPP_
