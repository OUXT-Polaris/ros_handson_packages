#ifndef TUTORIAL__SUBSCRIBE_HPP_
#define TUTORIAL__SUBSCRIBE_HPP_

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

  virtual ~Publish();

private:

};

}  // namespace tutorial

#endif  // TUTORIAL__SUBSCRIBE_HPP_
