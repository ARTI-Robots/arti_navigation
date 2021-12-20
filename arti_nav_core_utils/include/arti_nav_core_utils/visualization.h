/// \file
/// \author Alexander Buchegger
/// \date 2021-04-27
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#ifndef ARTI_NAV_CORE_UTILS_VISUALIZATION_H
#define ARTI_NAV_CORE_UTILS_VISUALIZATION_H

#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <geometry_msgs/Vector3.h>
#include <limits>
#include <ros/node_handle.h>
#include <std_msgs/ColorRGBA.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace arti_nav_core_utils
{

class TrajectoryWithVelocityMarkersPublisher
{
public:
  TrajectoryWithVelocityMarkersPublisher() = default;
  TrajectoryWithVelocityMarkersPublisher(ros::NodeHandle node_handle, const std::string& topic);

  void publish(
    const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory,
    size_t highlighted_index = std::numeric_limits<size_t>::max());

  geometry_msgs::Vector3 pose_marker_scale;
  double highlighted_marker_scale_factor{1.25};
  std_msgs::ColorRGBA pose_marker_color;
  std_msgs::ColorRGBA highlighted_pose_marker_color;
  geometry_msgs::Vector3 velocity_marker_scale;
  std_msgs::ColorRGBA velocity_marker_color;
  std_msgs::ColorRGBA highlighted_velocity_marker_color;

protected:
  ros::Publisher publisher_;
  size_t previous_trajectory_size_{0};
};

}

#endif // ARTI_NAV_CORE_UTILS_VISUALIZATION_H
