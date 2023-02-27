#include "tutorial/publish.hpp"

namespace tutorial
{

Publish::Publish(const rclcpp::NodeOptions & options)
: rclcpp::Node("publish", options)
{
  // publisherの
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 1);
  // std::chronoライブラリのリテラルを有効化（https://onihusube.hatenablog.com/entry/2018/06/01/010851）
  // この行があると、100msと書けば100 millisecondsを表現できる
  using namespace std::chrono_literals;
  timer_ =
    this->create_wall_timer(
    100ms, [this]() {
      std_msgs::msg::String data;
      data.data = "Hello";
      pub_->publish(data);
    });
}

Publish::~Publish()
{
}

}  // namespace tutorial
