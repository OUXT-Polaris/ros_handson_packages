#include "tutorial/publish.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace tutorial
{

Publish::Publish(const rclcpp::NodeOptions & options)
: rclcpp::Node("publish", options) // メンバ初期化子リストで基底型であるrclcpp::Node型を初期化
{
  RCLCPP_INFO_STREAM(get_logger(), "start initializing publisher");
  // create_publisher関数でpublisherをインスタンス化、第一引数がトピック名、第二引数が送信バッファのサイズ
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 1);
  // std::chronoライブラリのリテラルを有効化（https://onihusube.hatenablog.com/entry/2018/06/01/010851）
  // この行があると、100msと書けば100 millisecondsを表現できる
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(
    100ms, [this]() {
      // Publishするデータを格納する変数を定義
      std_msgs::msg::String data;
      // 変数にデータの値を代入
      data.data = "Hello";
      // Publishする中身をターミナルに出力
      RCLCPP_INFO_STREAM(get_logger(), data.data);
      // データをpublish
      pub_->publish(data);
    });
}

Publish::~Publish()
{
}

}  // namespace tutorial

RCLCPP_COMPONENTS_REGISTER_NODE(tutorial::Publish)
