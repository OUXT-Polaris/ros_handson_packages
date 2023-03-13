#ifndef BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
#define BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
  template<typename T>
  auto get_parameter(const std::string & name, const T & default_value) -> T
  {
    if (!has_parameter(name)) {
      declare_parameter(name, default_value);
    }
    T value;
    get_parameter(name, value);
    return value;
  }
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  // cmd_velに対して速度司令を出すPublisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  // /scanトピック経由でLiDARによるスキャン情報を取得するsubscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // ゴール地点を受信するsubscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  double wheel_radius_;
};

}  // namespace braitenberg_vehicle

#endif  // BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
