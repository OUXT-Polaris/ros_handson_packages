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

#ifndef BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
#define BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <braitenberg_vehicle/motion_model.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

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
  template <typename T>
  auto get_ros2_parameter(const std::string & name, const T & default_value) -> T
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
  std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
  // LaserScanで得られた点群情報を保存するための変数
  std::vector<geometry_msgs::msg::PointStamped> scan_points_;
  // ベースリンクのframe_id
  const std::string base_link_frame_id_;
  // オドメトリのframe_id
  const std::string odom_frame_id_;
  // 仮想光センサの取り付け位置のX座標
  const double virtual_light_sensor_position_x_offset_;
  // 仮想光センサの取り付け位置のY座標
  const double virtual_light_sensor_position_y_offset_;
  // 仮想光センサが正面から何度ずれて設置されているか(rad)
  const double virtual_light_sensor_angle_offset_;
  // 仮想光センサ出力をモータ回転数に変換するときの係数
  const double virtual_light_sensor_gain_;
  // 仮想光センサの視角（rad）
  const double virtual_light_sensor_viewing_angle_;
  // 仮想超音波センサのレンジ
  const double virtual_ultrasonic_sensor_range_;
  // 仮想超音波センサの取り付け位置のX座標
  const double virtual_ultrasonic_sensor_position_x_offset_;
  // 仮想超音波センサの取り付け位置のY座標
  const double virtual_ultrasonic_sensor_position_y_offset_;
  // 仮想超音波センサの視野角（rad）
  const double virtual_ultrasonic_viewing_angle_;
  // 仮想超音波センサ出力をモータ回転数に変換するときの係数
  const double virtual_ultrasonic_sensor_gain_;
  // 制御コマンド更新のためのタイマー
  rclcpp::TimerBase::SharedPtr timer_;
  // 一定時間ごとに刻まれるタイマーのコールバック
  void timer_callback();
  // データ同期用のmutex
  std::mutex mutex_;
  // 運動モデルを計算するクラス
  MotionModel motion_model_;
  // ゴール地点を光源として扱うための仮想光センサ入力を計算するための関数
  double emulate_light_sensor(
    double x_offset, double y_offset, double angle_offset,
    const geometry_msgs::msg::Point & goal_point) const;
  // LaserScan結果を仮想超音波センサ出力に変換する関数
  double emulate_ultrasonic_sensor(double x_offset, double y_offset) const;
  // tf(座標系解決のためのトピック)のデータを一体時間バッファするためのクラス
  tf2_ros::Buffer buffer_;
  // tf(座標系解決のためのトピック)を受信するためのクラス
  tf2_ros::TransformListener listener_;
};

}  // namespace braitenberg_vehicle

#endif  // BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
