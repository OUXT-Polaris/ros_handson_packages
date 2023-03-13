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

#include "braitenberg_vehicle/braitenberg_vehicle_controller.hpp"

#include <limits>

namespace braitenberg_vehicle
{

BraitenbergVehicleController::BraitenbergVehicleController(const rclcpp::NodeOptions & options)
: rclcpp::Node("braitenberg_vehicle_controller", options),
  // "base_link_frame_id"パラメータを読み込み、メンバ変数にセット
  base_link_frame_id_(get_parameter("base_link_frame_id", std::string("base_link"))),
  motion_model_(
    // "wheel_radius"パラメータを読み込みメンバ変数にセット、デフォルト値はTurtlebot3 burgerの.xacroファイルより計算
    get_parameter("wheel_radius", 0.033),
    // "wheel_base"パラメータを読み込みメンバ変数二セット、デフォルト値はTurtlebot3 burgerの.xacroファイルより計算
    get_parameter("wheel_base", 0.16))
{
  // publisherを作る関数、テンプレート引数はメッセージ型、第一引数はtopic名、第二引数は送信バッファサイズ
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  // subscriptionを作る関数、テンプレート引数はメッセージ型、第一引数はtopic名、第二引数は受信バッファサイズ、第三引数にコールバック関数を登録
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 1, [this](const sensor_msgs::msg::LaserScan::SharedPtr scan) { scan_callback(scan); });
  // subscriptionを作る関数、テンプレート引数はメッセージ型、第一引数はtopic名、第二引数は受信バッファサイズ、第三引数にコールバック関数を登録
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) { goal_pose_callback(pose); });
}

BraitenbergVehicleController::~BraitenbergVehicleController() {}

void BraitenbergVehicleController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  mutex_.lock();
  mutex_.unlock();
}

void BraitenbergVehicleController::goal_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  mutex_.lock();
  // 受信したゴール姿勢のframe_id(座標系の名前)が適切なものかを確認
  if (pose->header.frame_id == base_link_frame_id_) {
    // 適切な場合は、goal_pose_に受信したposeを代入
    goal_pose_ = pose->pose;
  } else {
    // 不適切な場合は、goal_pose_に無効値を表現するstd::nulloptをセット
    goal_pose_ = std::nullopt;
  }
  mutex_.unlock();
}

void BraitenbergVehicleController::timer_callback()
{
  mutex_.lock();
  if (goal_pose_) {
  } else {
    twist_pub_->publish(geometry_msgs::msg::Twist());
  }
  mutex_.unlock();
}

double BraitenbergVehicleController::emulate_light_sensor(double x_offset, double y_offset) const
{
  constexpr auto e = std::numeric_limits<double>::epsilon();
  if (goal_pose_) {
    if (const auto distance =
          std::hypot(goal_pose_->position.x - x_offset, goal_pose_->position.y - y_offset);
        std::abs(distance) < e)
      return 1 / distance;
    else {
      return 1;
    }
  } else {
    return 0;
  }
}
}  // namespace braitenberg_vehicle
