#ifndef TUTORIAL__SUBSCRIBE_HPP_
#define TUTORIAL__SUBSCRIBE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "tutorial/visibility_control.h"

namespace tutorial
{

// rclcpp::Nodeを継承することでROS2ノード実装に必要な関数をかんたんに容易することができる。
class Subscribe : public rclcpp::Node
{
public:
  // マルチプラットフォーム対応のためのマクロ
  TUTORIAL_PUBLIC
  // コンストラクタの引数は必ず「const rclcpp::NodeOptions & options」でなければならない
  explicit Subscribe(const rclcpp::NodeOptions & options);

  // virtualなデストラクタを定義（参照：https://www.yunabe.jp/docs/cpp_virtual_destructor.html）
  virtual ~Subscribe();

private:
  // Subscriber（データ受信器）の共有ポインタ
  // データ型はstd_msgs/msg/String型（https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/String.msg）
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace tutorial

#endif  // TUTORIAL__SUBSCRIBE_HPP_
