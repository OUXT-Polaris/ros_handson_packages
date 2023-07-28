// Copyright (c) 2023 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TUTORIAL__SUBSCRIBE_HPP_
#define TUTORIAL__SUBSCRIBE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "tutorial/visibility_control.h"

namespace tutorial
{

// rclcpp::Nodeを継承することでROS2ノード実装に必要な関数を容易に用意することができる。
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
