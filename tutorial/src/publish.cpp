#include "tutorial/publish.hpp"

namespace tutorial
{

Publish::Publish(const rclcpp::NodeOptions & options)
: rclcpp::Node("publish", options)
{
}

Publish::~Publish()
{
}

}  // namespace tutorial
