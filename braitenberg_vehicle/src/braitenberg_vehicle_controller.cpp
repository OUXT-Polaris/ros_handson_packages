#include "braitenberg_vehicle/braitenberg_vehicle_controller.hpp"

namespace braitenberg_vehicle
{

BraitenbergVehicleController::BraitenbergVehicleController(const rclcpp::NodeOptions & options)
: rclcpp::Node("braitenberg_vehicle_controller", options)
{
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 1, [this](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      scan_callback(scan);
    });
}

BraitenbergVehicleController::~BraitenbergVehicleController()
{
}

void BraitenbergVehicleController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr /*scan*/)
{
}

}  // namespace braitenberg_vehicle
