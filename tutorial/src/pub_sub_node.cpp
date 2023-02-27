// rclcpp::shutodown/rclcpp::init等を使用するためのヘッダ
#include <rclcpp/rclcpp.hpp>

// componentクラス群を定義したヘッダファイルをinclude
#include <tutorial/publish.hpp>
#include <tutorial/subscribe.hpp>

// main文 (C++のエントリポイント)
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(std::make_shared<tutorial::Publish>(rclcpp::NodeOptions()));
  exec.add_node(std::make_shared<tutorial::Subscribe>(rclcpp::NodeOptions()));
  exec.spin();
  rclcpp::shutdown();
}
