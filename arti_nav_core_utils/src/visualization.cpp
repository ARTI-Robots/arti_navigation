/// \file
/// \author Alexander Buchegger
/// \date 2021-04-27
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#include <arti_nav_core_utils/visualization.h>
#include <arti_nav_core_utils/conversions.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace arti_nav_core_utils
{

TrajectoryWithVelocityMarkersPublisher::TrajectoryWithVelocityMarkersPublisher(
  ros::NodeHandle node_handle, const std::string& topic)
  : publisher_(node_handle.advertise<visualization_msgs::MarkerArray>(topic, 1, true))
{
  pose_marker_scale.x = 0.3;
  pose_marker_scale.y = 0.1;
  pose_marker_scale.z = 0.1;

  pose_marker_color.r = 0.1;
  pose_marker_color.g = 1.0;
  pose_marker_color.b = 0.0;
  pose_marker_color.a = 1.0;

  highlighted_pose_marker_color.r = 0.9;
  highlighted_pose_marker_color.g = 1.0;
  highlighted_pose_marker_color.b = 0.0;
  highlighted_pose_marker_color.a = 1.0;

  velocity_marker_scale.x = 1.0;
  velocity_marker_scale.y = 0.1;
  velocity_marker_scale.z = 0.1;

  velocity_marker_color.r = 1.0;
  velocity_marker_color.g = 0.1;
  velocity_marker_color.b = 0.0;
  velocity_marker_color.a = 1.0;

  highlighted_velocity_marker_color.r = 1.0;
  highlighted_velocity_marker_color.g = 0.9;
  highlighted_velocity_marker_color.b = 0.0;
  highlighted_velocity_marker_color.a = 1.0;
}

void TrajectoryWithVelocityMarkersPublisher::publish(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory, const size_t highlighted_index)
{
  if (!publisher_)
  {
    return;
  }

  const nav_msgs::Path path
    = arti_nav_core_utils::convertToPath(trajectory, arti_nav_core_utils::non_finite_values::REPLACE_BY_0);

  visualization_msgs::MarkerArray markers;
  markers.markers.reserve(std::max(path.poses.size(), previous_trajectory_size_) * 2);
  for (size_t i = 0; i < path.poses.size(); ++i)
  {
    const geometry_msgs::PoseStamped& pose = path.poses[i];
    const double v_x = trajectory.movements.at(i).twist.x.value;

    {
      markers.markers.emplace_back();
      visualization_msgs::Marker& marker = markers.markers.back();
      marker.header = pose.header;
      marker.ns = "pose";
      marker.id = i;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = pose.pose;
      marker.scale = pose_marker_scale;
      if (i == highlighted_index)
      {
        marker.scale.y *= highlighted_marker_scale_factor;
        marker.scale.z *= highlighted_marker_scale_factor;
        marker.color = highlighted_pose_marker_color;
      }
      else
      {
        marker.color = pose_marker_color;
      }
    }

    {
      markers.markers.emplace_back();
      visualization_msgs::Marker& marker = markers.markers.back();
      marker.header = pose.header;
      marker.ns = "velocity";
      marker.id = i;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = pose.pose.position;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose.pose.orientation) + M_PI_2);
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale = velocity_marker_scale;
      marker.scale.x *= std::copysign(std::max(std::abs(v_x), 0.01), v_x);
      if (i == highlighted_index)
      {
        marker.scale.y *= highlighted_marker_scale_factor;
        marker.scale.z *= highlighted_marker_scale_factor;
        marker.color = highlighted_velocity_marker_color;
      }
      else
      {
        marker.color = velocity_marker_color;
      }
    }
  }

  for (size_t i = path.poses.size(); i < previous_trajectory_size_; ++i)
  {
    {
      markers.markers.emplace_back();
      visualization_msgs::Marker& marker = markers.markers.back();
      marker.ns = "pose";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
    }

    {
      markers.markers.emplace_back();
      visualization_msgs::Marker& marker = markers.markers.back();
      marker.ns = "velocity";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }
  previous_trajectory_size_ = path.poses.size();

  publisher_.publish(markers);
}

}
