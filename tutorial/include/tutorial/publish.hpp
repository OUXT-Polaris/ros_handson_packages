#ifndef TUTORIAL__PUBLISH_HPP_
#define TUTORIAL__PUBLISH_HPP_

#include "tutorial/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial
{

// rclcpp::Nodeを継承することでROS2ノード実装に必要な関数をかんたんに容易することができる。
class Publish : public rclcpp::Node
{
public:
  // マルチプラットフォーム対応のためのマクロ
  TUTORIAL_PUBLIC
  // コンストラクタの引数は必ず「const rclcpp::NodeOptions & options」でなければならない
  explicit Publish(const rclcpp::NodeOptions & options);

  // virtualなデストラクタを定義（参照：https://www.yunabe.jp/docs/cpp_virtual_destructor.html）
  virtual ~Publish();

private:
  // Publisher（データ送信器）の共有ポインタ
  // データ型はstd_msgs/msg/String型（https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/String.msg）
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tutorial

#endif  // TUTORIAL__PUBLISH_HPP_
