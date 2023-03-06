#ifndef BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
#define BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_

#include "braitenberg_vehicle/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  // cmd_velに対して速度司令を出すPublisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  // /scanトピック経由でLiDARによるスキャン情報を取得するsubscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace braitenberg_vehicle

#endif  // BRAITENBERG_VEHICLE__BRAITENBERG_VEHICLE_CONTROLLER_HPP_
