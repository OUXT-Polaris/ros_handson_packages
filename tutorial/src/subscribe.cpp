#include "tutorial/subscribe.hpp"

namespace tutorial
{

Subscribe::Subscribe(const rclcpp::NodeOptions & options)
: rclcpp::Node("subscribe", options) // メンバ初期化子リストで基底型であるrclcpp::Node型を初期化
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

Subscribe::~Subscribe()
{
}

}  // namespace tutorial
