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

#include <algorithm>
#include <limits>
#include <rclcpp_components/register_node_macro.hpp>

// tf2::doTransform関数をエラーなく実行するためにはこのヘッダが必要
// これをincludeし忘れると実行時にクラッシュします。
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace braitenberg_vehicle
{

BraitenbergVehicleController::BraitenbergVehicleController(const rclcpp::NodeOptions & options)
: rclcpp::Node("braitenberg_vehicle_controller", options),
  // "base_link_frame_id"パラメータを読み込み、メンバ変数にセット
  base_link_frame_id_(get_ros2_parameter("base_link_frame_id", std::string("base_link"))),
  // "odom_frame_id"パラメータを読み込み、メンバ変数にセット
  odom_frame_id_(get_ros2_parameter("odom_frame_id", std::string("odom"))),
  //　パラメータから仮想光センサの取り付け位置を読み込み
  virtual_light_sensor_position_x_offset_(
    get_ros2_parameter("virtual_light_sensor_position_x_offset", 0.1)),
  virtual_light_sensor_position_y_offset_(
    get_ros2_parameter("virtual_light_sensor_position_y_offset", 3.0)),
  motion_model_(
    // "wheel_radius"パラメータを読み込みメンバ変数にセット、デフォルト値はTurtlebot3 burgerの.xacroファイルより計算
    get_ros2_parameter("wheel_radius", 0.033),
    // "wheel_base"パラメータを読み込みメンバ変数二セット、デフォルト値はTurtlebot3 burgerの.xacroファイルより計算
    get_ros2_parameter("wheel_base", 0.16)),
  buffer_(get_clock()),
  listener_(buffer_)
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
  // std::chronoに登録されたリテラルを用いて時間を人間的な書式で記載できるようにする
  using namespace std::chrono_literals;
  // BraitenbergVehicleController::timer_callback関数を1msに一回実行する関数として登録
  timer_ = create_wall_timer(1ms, [this]() { timer_callback(); });
}

BraitenbergVehicleController::~BraitenbergVehicleController() {}

void BraitenbergVehicleController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // クリティカルセクション開始
  mutex_.lock();
  // クリティカルセクション終了
  mutex_.unlock();
}

void BraitenbergVehicleController::goal_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // クリティカルセクション開始
  mutex_.lock();
  // 受信したゴール姿勢のframe_id(座標系の名前)が適切なものかを確認
  if (pose->header.frame_id == odom_frame_id_) {
    // 適切な場合は、goal_pose_に受信したposeを代入
    goal_pose_ = *pose;
  } else {
    // 不適切な場合は、odom座標系に座標変換
    try {
      // 座標変換結果を格納する一時変数
      geometry_msgs::msg::PoseStamped p;
      // 座標変換を実行
      tf2::doTransform(
        // 第一引数は変換したい姿勢を入力、第二引数には変換結果を格納する姿勢を入力
        *pose, p,
        // 同次変換行列計算に必要な情報をtfのバッファから計算
        // pose->header.frame_id(受信したゴール地点の座標系)-> odom_frame_id_(オドメトリ座標系)の相対座標系を計算
        // 第三引数はいつの時点の座標変換を取得したいか、rclcpp::Time(0)とすると最新の相対座標系が計算される
        // 第四引数はウェイトの最大時間、tfは分散座標系管理を行うためウェイトが短すぎると受信に失敗する可能性がある
        buffer_.lookupTransform(
          odom_frame_id_, pose->header.frame_id, rclcpp::Time(0), tf2::durationFromSec(1.0)));
      goal_pose_ = p;
    }
    // 座標変換が失敗したときの例外処理
    catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      // 座標変換に失敗した場合、ゴール地点をキャンセル
      goal_pose_ = std::nullopt;
    }
  }
  // クリティカルセクション終了
  mutex_.unlock();
}

void BraitenbergVehicleController::timer_callback()
{
  // クリティカルセクション開始
  mutex_.lock();
  if (goal_pose_) {
    // 座標変換結果を格納する一時変数
    geometry_msgs::msg::PoseStamped p;
    try {
      // 座標変換を実行
      tf2::doTransform(
        // 第一引数は変換したい姿勢を入力、第二引数には変換結果を格納する姿勢を入力
        goal_pose_.value(), p,
        // 同次変換行列計算に必要な情報をtfのバッファから計算
        // odom_frame_id_(オドメトリ座標系)->base_link_frame_id_(ロボット座標系)の相対座標系を計算
        // 第三引数はいつの時点の座標変換を取得したいか、rclcpp::Time(0)とすると最新の相対座標系が計算される
        // 第四引数はウェイトの最大時間、tfは分散座標系管理を行うためウェイトが短すぎると受信に失敗する可能性がある
        buffer_.lookupTransform(
          base_link_frame_id_, odom_frame_id_, rclcpp::Time(0), tf2::durationFromSec(1.0)));
      // RCLCPP_ERROR_STREAM(
      //   get_logger(), emulate_light_sensor(
      //                   virtual_light_sensor_position_x_offset_,
      //                   virtual_light_sensor_position_y_offset_ * -1, p.pose.position));
      twist_pub_->publish(
        // モーションモデルを計算して、速度司令を計算
        motion_model_.get_twist(
          // 左側の車輪には右側の仮想光センサの出力を入力
          // ROSの座標系は前方がX軸、左がY軸、上がZ軸の正方向
          100 * emulate_light_sensor(
                  virtual_light_sensor_position_x_offset_,
                  virtual_light_sensor_position_y_offset_ * -1, p.pose.position),
          // 右側の車輪には左側の仮想光センサの出力を入力
          // ROSの座標系は前方がX軸、左がY軸、上がZ軸の正方向
          100 * emulate_light_sensor(
                  virtual_light_sensor_position_x_offset_, virtual_light_sensor_position_y_offset_,
                  p.pose.position)));
    }
    // 座標変換が失敗したときの例外処理
    catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      // 座標変換に失敗した場合、その場で停止
      twist_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }
  } else {
    // 有効なゴール指定がされていない場合、その場で停止
    twist_pub_->publish(geometry_msgs::msg::Twist());
  }
  // クリティカルセクション終了
  mutex_.unlock();
}

// ゴール地点を光源として扱うための仮想光センサ入力を計算するための関数
// 第一引数のx_offsetはbase_linkからの仮想光センサのx座標
// 第二引数のy_offsetはbase_linkからの仮想光センサのy座標
// 第三引数のgoal_pointはbase_linkでみたときのゴール地点の座標
double BraitenbergVehicleController::emulate_light_sensor(
  double x_offset, double y_offset, const geometry_msgs::msg::Point & goal_point) const
{
  // constexprをつけることで変数eはコンパイル時に計算され定数となるので実行時の計算コストを減らすことができる（https://cpprefjp.github.io/lang/cpp11/constexpr.html）
  // std::numeric_limits<double>::epsilon();はdouble型の計算機イプシロン（処理系が取り扱える浮動小数点の誤差幅）である（https://cpprefjp.github.io/reference/limits/numeric_limits/epsilon.html）
  // 浮動小数点型に対して一致を計算する場合には「==演算子」を用いてはならない、計算機イプシロンを考慮して計算する必要がある
  // 例えば、float a = 3.0;したときにa=3であることを確認するには以下のように記述する必要がある
  // constexpr auto e = std::numeric_limits<float>::epsilon();
  // float a = 3.0;
  // if(std::abs(a - 3.0) <= e) { /* a = 3.0のときの処理 */ }
  constexpr auto e = std::numeric_limits<double>::epsilon();
  // 有効なゴール姿勢がある場合
  // (goal_point.x - x_offset)^2 + (goal_point.y - y_offset)^2の平方根を計算して二点間の距離を求める
  if (const auto distance = std::hypot(goal_point.x - x_offset, goal_point.y - y_offset);
      // 距離の絶対値が計算機イプシロンより小さい、つまりdistance = 0.0の場合
      std::abs(distance) <= e)
    // ゼロ割を回避しつつセンサの出力値を最大にしたいので1を返す
    return 1;
  else {
    // センサの出力は距離に反比例する。
    return std::clamp(1 / distance, 0.0, 1.0);
  }
}
}  // namespace braitenberg_vehicle

// braitenberg_vehicle::BraitenbergVehicleControllerクラスをコンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(braitenberg_vehicle::BraitenbergVehicleController)
