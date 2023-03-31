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
  virtual_light_sensor_angle_offset_(get_ros2_parameter("virtual_light_sensor_angle_offset", 0.5)),
  // パラメータから仮想光センサ出力をモータ回転数に変換するときの係数を読み込み
  virtual_light_sensor_gain_(get_ros2_parameter("virtual_light_sensor_gain", 100.0)),
  // パラメータから仮想光センサの視野角を読み込み
  virtual_light_sensor_viewing_angle_(
    get_ros2_parameter("virtual_light_sensor_viewing_angle", M_PI)),
  //　パラメータから仮想超音波センサの計測距離を読み込み
  virtual_ultrasonic_sensor_range_(get_ros2_parameter("virtual_ultrasonic_sensor_range", 1.0)),
  //　パラメータから仮想超音波センサの取り付け位置を読み込み
  virtual_ultrasonic_sensor_position_x_offset_(
    get_ros2_parameter("virtual_ultrasonic_sensor_position_x_offset", 0.1)),
  virtual_ultrasonic_sensor_position_y_offset_(
    get_ros2_parameter("virtual_ultrasonic_sensor_position_y_offset", 0.1)),
  // パラメータから仮想超音波センサの視野角を読み込み
  virtual_ultrasonic_viewing_angle_(
    get_ros2_parameter("virtual_ultrasonic_viewing_angle", 0.523599)),
  // パラメータから仮想超音波センサ出力をモータ回転数に変換するときの係数を読み込み
  virtual_ultrasonic_sensor_gain_(get_ros2_parameter("virtual_ultrasonic_sensor_gain", 100.0)),
  // motiom_modelクラスを初期化
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
  timer_ = create_wall_timer(10ms, [this]() { timer_callback(); });
}

BraitenbergVehicleController::~BraitenbergVehicleController() {}

void BraitenbergVehicleController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // クリティカルセクション開始
  mutex_.lock();
  scan_points_.clear();
  size_t index = 0;
  // 計測された各点に対して座標変換を実行
  for (const auto range : scan->ranges) {
    geometry_msgs::msg::PointStamped p;
    p.header = scan->header;
    // 各計測点のタイムスタンプを取得
    p.header.stamp = static_cast<rclcpp::Time>(p.header.stamp) +
                     rclcpp::Duration(std::chrono::nanoseconds(
                       static_cast<int>(std::round(scan->time_increment * std::pow(10, 9)))));
    // 各計測点の角度を計算
    double angle = scan->angle_min + index * scan->angle_increment;
    // 計測点のX座標、Y座標を計算
    p.point.x = range * std::sin(angle);
    p.point.y = range * std::cos(angle);
    try {
      tf2::doTransform(
        p, p,
        buffer_.lookupTransform(
          base_link_frame_id_, p.header.frame_id, rclcpp::Time(0), tf2::durationFromSec(1.0)));
      scan_points_.emplace_back(p);
    }
    // 座標変換が失敗したときの例外処理
    catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
    }
    // indexをインクリメント
    index++;
  }
  // クリティカルセクション終了
  mutex_.unlock();
}

void BraitenbergVehicleController::goal_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  RCLCPP_INFO_STREAM(get_logger(), "recieve goal pose\n" + geometry_msgs::msg::to_yaml(*pose));
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
      twist_pub_->publish(
        // モーションモデルを計算して、速度司令を計算
        motion_model_.get_twist(
          // 左側の車輪には右側の仮想光センサの出力を入力
          // ROSの座標系は前方がX軸、左がY軸、上がZ軸の正方向
          virtual_light_sensor_gain_ * emulate_light_sensor(
                                         virtual_light_sensor_position_x_offset_,
                                         virtual_light_sensor_position_y_offset_ * -1,
                                         virtual_light_sensor_angle_offset_ * -1, p.pose.position) -
            // 左側の車輪には左側の仮想超音波センサの出力を入力
            virtual_ultrasonic_sensor_gain_ * emulate_ultrasonic_sensor(
                                                virtual_ultrasonic_sensor_position_x_offset_,
                                                virtual_ultrasonic_sensor_position_y_offset_),
          // 右側の車輪には左側の仮想光センサの出力を入力
          // ROSの座標系は前方がX軸、左がY軸、上がZ軸の正方向
          virtual_light_sensor_gain_ * emulate_light_sensor(
                                         virtual_light_sensor_position_x_offset_,
                                         virtual_light_sensor_position_y_offset_,
                                         virtual_light_sensor_angle_offset_, p.pose.position) -
            // 左側の車輪には右側の仮想超音波センサの出力を入力
            virtual_ultrasonic_sensor_gain_ *
              emulate_ultrasonic_sensor(
                virtual_ultrasonic_sensor_position_x_offset_,
                virtual_ultrasonic_sensor_position_y_offset_ * -1)));
    }
    // 座標変換が失敗したときの例外処理
    catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      // 座標変換に失敗した場合、その場で停止
      twist_pub_->publish(geometry_msgs::msg::Twist());
      // クリティカルセクション終了
      mutex_.unlock();
      return;
    }
  } else {
    // 有効なゴール指定がされていない場合、その場で停止
    twist_pub_->publish(geometry_msgs::msg::Twist());
  }
  // クリティカルセクション終了
  mutex_.unlock();
}

// LaserScan結果を仮想超音波センサ出力に変換する関数
// 最小値は0,最大値は1
double BraitenbergVehicleController::emulate_ultrasonic_sensor(
  double x_offset, double y_offset) const
{
  // 点群をフィルタするラムダ式
  auto filter_scan = [this, x_offset, y_offset](float angle_min, float angle_max, float range_max) {
    std::vector<geometry_msgs::msg::Point> filtered_points;
    for (const auto & point : scan_points_) {
      // 仮想超音波センサから見た点に座標変換
      geometry_msgs::msg::Point p;
      p.x = point.point.x - x_offset;
      p.y = point.point.y - y_offset;
      p.z = point.point.z;
      // 仮想超音波センサから見た角度を計算
      double angle = std::atan2(p.y, p.x);
      if (
        // 角度の上限加減範囲に入っているかを検証
        angle_min <= angle && angle <= angle_max &&
        // 距離が範囲内に入っているかを検証
        std::hypot(p.x, p.y, p.z) <= range_max) {
        // 角度、距離が両方範囲内であれば点群を残す
        filtered_points.emplace_back(p);
      }
    }
    return filtered_points;
  };
  // 点群をフィルタ
  const auto filtered = filter_scan(
    -0.5 * virtual_ultrasonic_viewing_angle_, 0.5 * virtual_ultrasonic_viewing_angle_,
    virtual_ultrasonic_sensor_range_);
  // フィルタした結果何も残らなかった場合0を出力
  if (filtered.empty()) {
    return 0;
  }
  // フィルタした結果点群が残った時は最近傍点との距離から出力を計算
  else {
    // 距離情報を保管するstd::vector
    std::vector<double> distance;
    // 点群情報から距離情報を計算
    std::transform(
      filtered.begin(), filtered.end(), std::back_inserter(distance),
      [](const auto p) { return std::hypot(p.x, p.y, p.z); });
    // 最も小さい距離を計算
    double min_distance = *std::min_element(distance.begin(), distance.end());
    // 距離の二乗からセンサ出力を計算、1を超えている場合は1を出力
    return std::clamp(1 / (min_distance * min_distance), 0.0, 1.0);
  }
}

// ゴール地点を光源として扱うための仮想光センサ入力を計算するための関数
// 最小値は0,最大値は1
// 第一引数のx_offsetはbase_linkからの仮想光センサのx座標
// 第二引数のy_offsetはbase_linkからの仮想光センサのy座標
// 第三引数のangle_ofsetはbase_linkから見た時仮想光センサの角度のズレ
// 第四引数のgoal_pointはbase_linkでみたときのゴール地点の座標
double BraitenbergVehicleController::emulate_light_sensor(
  double x_offset, double y_offset, double angle_offset,
  const geometry_msgs::msg::Point & goal_point) const
{
  // arc tangentを使うときはatan2を使いましょう、ロボットでstd::atanを使うと突然解が飛んで事故ることがあります。
  const auto theta = std::atan2(goal_point.y - y_offset, goal_point.x - x_offset);
  // goal_pointが視界に入っているかを判定、入っていない場合は0を出力
  if (
    theta >= (angle_offset + virtual_light_sensor_viewing_angle_ * 0.5) ||
    (angle_offset - virtual_light_sensor_viewing_angle_ * 0.5) >= theta) {
    return 0;
  }
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
    return 1.0;
  else {
    // センサの出力は距離の二乗に反比例する。
    return std::clamp(1 / (distance * distance), 0.0, 1.0);
  }
}
}  // namespace braitenberg_vehicle

// braitenberg_vehicle::BraitenbergVehicleControllerクラスをコンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(braitenberg_vehicle::BraitenbergVehicleController)
