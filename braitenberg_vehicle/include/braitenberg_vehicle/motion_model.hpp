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

#ifndef BRAITENBERG_VEHICLE__MOTION_MODEL_HPP_
#define BRAITENBERG_VEHICLE__MOTION_MODEL_HPP_

#include <geometry_msgs/msg/twist.hpp>

namespace braitenberg_vehicle
{
class MotionModel
{
public:
  explicit MotionModel(double wheel_radius, double wheel_base);
  // ホイール半径、constなメンバ変数なのでメンバ初期化子リストで初期化後うわがくことはできない
  const double wheel_radius;
  // ホイールベース、constなメンバ変数なのでメンバ初期化子リストで初期化後うわがくことはできない
  const double wheel_base;
  // 車輪の回転速を入力すると速度を返す関数、関数の後についているconstはこの関数が副作用が無いことをコンパイラに伝えている。
  geometry_msgs::msg::Twist get_twist(
    double left_wheel_rotational_speed, double right_wheel_rotational_speed) const;
};
}  // namespace braitenberg_vehicle

#endif  // BRAITENBERG_VEHICLE__MOTION_MODEL_HPP_
