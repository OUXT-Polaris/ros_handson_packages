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

#include "tutorial/subscribe.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace tutorial
{

// コンストラクタ
Subscribe::Subscribe(const rclcpp::NodeOptions & options)
: rclcpp::Node("subscribe", options)  // メンバ初期化子リストで基底型であるrclcpp::Node型を初期化
{
  RCLCPP_INFO_STREAM(get_logger(), "start initializing subscriber");
  // ラムダ式で定義したコールバック関数を引き渡し、subscriberをインスタンス化したのちその共有ポインタを返すcreate_subscription関数を実行
  sub_ = create_subscription<std_msgs::msg::String>(
    // 第一引数はトピック名、第二引数は受信バッファのサイズ
    // 第三引数はコールバックのラムダ式、thisキャプチャしておけばこのクラスのメンバ変数、メンバ関数にアクセスできる
    "chatter", 1, [this](const std_msgs::msg::String & msg) {
      // ROS2のロギング用マクロ、ターミナルにログ出力しつつログファイルにも吐き出してくれる便利なやつ
      RCLCPP_INFO_STREAM(get_logger(), msg.data);
    });
}

// デストラクタ
Subscribe::~Subscribe() {}
}  // namespace tutorial

// turtorial::Subscribeクラスをコンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(tutorial::Subscribe)
