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

// rclcpp::shutodown/rclcpp::init等を使用するためのヘッダ
#include <rclcpp/rclcpp.hpp>

// componentクラス群を定義したヘッダファイルをinclude
#include <tutorial/publish.hpp>
#include <tutorial/subscribe.hpp>

// main文 (C++のエントリポイント)
int main(int argc, char * argv[])
{
  // ROS2の初期化処理
  rclcpp::init(argc, argv);
  // Executor(ノード実行器)のインスタンスを作成
  rclcpp::executors::MultiThreadedExecutor exec;
  // Publishノードの実体をインスタンス化し、その共有ポインタを取得
  const auto publish = std::make_shared<tutorial::Publish>(rclcpp::NodeOptions());
  // Subscribeノードの実態をインスタンス化し、その共有ポインタを取得
  const auto subscribe = std::make_shared<tutorial::Subscribe>(rclcpp::NodeOptions());
  // Executorに各ノードを登録する。
  // exec.add_node(std::make_shared<tutorial::Publish>(rclcpp::NodeOptions()));
  // とするとスッキリかけて良さそうに見えるが、add_nodeの処理が終わった瞬間に参照カウントが0になってノードの実体が破棄されてしまうのでやらないこと
  exec.add_node(publish);
  exec.add_node(subscribe);
  // spin（callbackの実行フラグとなっている各種イベントが実行されるまで待機する）処理
  exec.spin();
  // ROS2の終了処理
  rclcpp::shutdown();
}
