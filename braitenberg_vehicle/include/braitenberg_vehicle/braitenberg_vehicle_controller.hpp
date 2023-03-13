#ifndef BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
#define BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <optional>
#include <mutex>

#include "braitenberg_vehicle/visibility_control.h"

namespace braitenberg_vehicle
{
class BraitenbergVehicleController : public rclcpp::Node
{
public:
  // マルチプラットフォーム対応用マクロ
  BRAITENBERG_VEHICLE_PUBLIC
  explicit BraitenbergVehicleController(const rclcpp::NodeOptions & options);

  virtual ~BraitenbergVehicleController();

private:
  // パラメータを取得するためのテンプレート関数、第一引数はパラメータ名、第二引数はデフォルト値
  template<typename T>
  auto get_parameter(const std::string & name, const T & default_value) -> T
  {
    // 取得したいパラメータが宣言済みかを確認
    if (!has_parameter(name)) {
      // 宣言済み出なかった場合、宣言しつつデフォルト値をセット
      declare_parameter(name, default_value);
    }
    T value;
    // パラメータの値を取得
    get_parameter(name, value);
    return value;
  }
  // cmd_velに対して速度司令を出すPublisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  // /scanトピック経由でLiDARによるスキャン情報を取得するsubscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // scanトピック経由で点群を受け取るコールバック関数
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  // ゴール地点を受信するsubscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  // ゴール地点を受信するコールバック関数
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  // ゴール地点を記録するメンバ変数
  // ゴール地点受信前の状態を表現するためstd::optionalを用いて無効値を表現
  // https://cpprefjp.github.io/reference/optional/optional.html
  std::optional<geometry_msgs::msg::Pose> goal_pose_;
  // ホイールの半径を記録するメンバ変数
  double wheel_radius_;
  // ベースリンクのframe_idを記録するメンバ変数
  std::string base_link_frame_id_;
  // 制御コマンド更新のためのタイマー
  rclcpp::TimerBase::SharedPtr timer_;
  // 一定時間ごとに刻まれるタイマーのコールバック
  void timer_callback();
  // データ同期用のmutex
  std::mutex mutex_;
};

}  // namespace braitenberg_vehicle

#endif  // BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
