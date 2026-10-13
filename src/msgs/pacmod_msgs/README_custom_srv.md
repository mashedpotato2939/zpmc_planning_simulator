# 服务

服务名称：/srv_upper_test

服务类型：pacmod_msgs::srv::UpperTest

字段含义：
1. index: 0表示正常4ws控制， 1表示寸动功能
2. is_new: 未使用
3. enable: 0 表示失能， 1表示使能
4. bool_cmd: 未使用
5. int_cmd: 未使用
6. float_cmd: 在4ws时未使用，寸动时表示移动距离（无正负号，正负通过正反向标志位）
7. result: 返回响应请求的次数

回调函数：
```sh
void MotionController::srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request, srvUpperTest::Response::SharedPtr response)
{
  // 4WS控制的启停命令和模式切换
  // 只有在当前模式失能时才能切换模式，或者在同一模式下切换启停状态；当切换模式时，如果新模式为启用，则更新对应的控制命令

  static size_t response_count = 0;
  upper_test.is_new = (upper_test.index != request->index);

  if(upper_test.is_new && (!upper_test.enable))
  {
    upper_test.index = request->index;
    upper_test.enable = request->enable;
    if(upper_test.index == JOG_MOVE && upper_test.enable){
      upper_test.float_cmd = request->float_cmd;
      precise_positioning_->setTargetDistance(static_cast<double>(upper_test.float_cmd));
    }
  } else if(upper_test.is_new && upper_test.enable)
  {
    RCLCPP_WARN(this->get_logger(), "Please disable the previous command before switching a new mode.");
  } else if(!upper_test.is_new)
  {
    upper_test.enable = request->enable;
    if(upper_test.index == JOG_MOVE && upper_test.enable){
      upper_test.float_cmd = request->float_cmd;
      precise_positioning_->setTargetDistance(static_cast<double>(upper_test.float_cmd));
    }
  }

  response->result = response_count ++;
}
```

主要逻辑：由一个模式切换向另一个模式，需要保证已经原来模式已经disable


