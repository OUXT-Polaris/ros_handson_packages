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

#include <bits/stdc++.h>

#include <braitenberg_vehicle/motion_model.hpp>

namespace braitenberg_vehicle
{
// Motion Modelクラスのコンストラクタ、メンバイニシャライザリストでconstなメンバ変数を初期化
MotionModel::MotionModel(double wheel_radius, double wheel_base)
: wheel_radius(wheel_radius), wheel_base(wheel_base)
{
}

// 左右の車輪速を引数に受けてロボットの速度を出力に入れる関数
geometry_msgs::msg::Twist MotionModel::get_twist(
  double left_wheel_rotational_speed, double right_wheel_rotational_speed) const
{
  geometry_msgs::msg::Twist twist;
  // 回転数から左右の速度を計算するラムダ式を定義
  auto get_speed = [this](double rotational_speed) {
    return rotational_speed / (2 * M_PI) * wheel_radius;
  };
  // 左右の速度を計算し、そこからロボット全体の速度を計算
  twist.linear.x =
    (get_speed(right_wheel_rotational_speed) + get_speed(left_wheel_rotational_speed)) * 0.5;
  twist.angular.z =
    (get_speed(right_wheel_rotational_speed) - get_speed(left_wheel_rotational_speed)) * wheel_base;
  return twist;
}
}  // namespace braitenberg_vehicle
