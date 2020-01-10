/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_path_follower_wrapper.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <angles/angles.h>

namespace arti_wrap_old_nav_core
{
BasePathFollowerWrapper::BasePathFollowerWrapper() : plugin_loader_("nav_core", "nav_core::BaseLocalPlanner")
{ }

void BasePathFollowerWrapper::initialize(std::string name, tf::TransformListener* tf,
                                         costmap_2d::Costmap2DROS* costmap_ros)
{
  ros::NodeHandle private_nh("~/" + name);
  std::string ros_path_follower_name;
  if (!private_nh.getParam("path_follower_name", ros_path_follower_name))
    throw std::runtime_error("no local planner specified to wrap");

  try
  {
    planner_ = plugin_loader_.createInstance(ros_path_follower_name);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Failed to load the " << ros_path_follower_name << " with fault " << ex.what());
    throw ex;
  }

  std::string plugin_name = plugin_loader_.getName(ros_path_follower_name);
  planner_->initialize(plugin_name, tf, costmap_ros);

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&BasePathFollowerWrapper::odomCB, this, _1));

}

bool BasePathFollowerWrapper::setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  if (trajectory.movements.empty())
  {
    ROS_ERROR("trajectory is empty");
    return false;
  }

  current_trajectory_ = trajectory;
  current_index_ = 0;

  std::vector<geometry_msgs::PoseStamped> planner_trajectory(current_trajectory_.movements.size());
  for (size_t i = 0; i < current_trajectory_.movements.size(); ++i)
  {
    planner_trajectory[i].header = current_trajectory_.header;
    planner_trajectory[i].pose.position.x = current_trajectory_.movements[i].pose.point.x.value;
    planner_trajectory[i].pose.position.y = current_trajectory_.movements[i].pose.point.y.value;
    planner_trajectory[i].pose.orientation =
        tf::createQuaternionMsgFromYaw(current_trajectory_.movements[i].pose.theta.value);
  }

  return planner_->setPlan(planner_trajectory);
}

arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum BasePathFollowerWrapper::computeVelocityCommands(
  geometry_msgs::Twist& next_command)
{
  calcualteCurrentIndex();

  arti_nav_core_msgs::Twist2DWithLimits expected_twist;
  if (!current_trajectory_.movements.empty())
  {
    expected_twist = current_trajectory_.movements[current_index_].twist;
  }
  else
  {
    expected_twist.x.value = std::numeric_limits<double>::infinity();
    expected_twist.y.value = std::numeric_limits<double>::infinity();
    expected_twist.theta.value = std::numeric_limits<double>::infinity();
  }

  // check if a twist is set by the trajectory we should use
  if (!current_trajectory_.movements.empty() && (current_index_ < (current_trajectory_.movements.size() - 1)))
  {
    if ((!expected_twist.theta.has_limits) && (!expected_twist.x.has_limits) && (!expected_twist.y.has_limits) &&
        std::isfinite(expected_twist.theta.value) && std::isfinite(expected_twist.x.value) &&
        std::isfinite(expected_twist.y.value))
    {
      next_command.linear.x = expected_twist.x.value;
      next_command.linear.y = expected_twist.y.value;

      next_command.angular.z = expected_twist.theta.value;
      return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::COMMAND_FOUND;
    }
  }

  if (!planner_->computeVelocityCommands(next_command))
  {
    ROS_ERROR("can not compute velocity");
    next_command = geometry_msgs::Twist();
    return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
  }

  // check limits if exist
  if (expected_twist.theta.has_limits)
  {
    if (!withinTolerance(next_command.angular.z, expected_twist.theta.value, expected_twist.theta.upper_limit,
                        expected_twist.theta.lower_limit, true))
    {
      ROS_ERROR("angular rotation velocity violates limits");
      next_command = geometry_msgs::Twist();
      return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
    }
  }

  if (expected_twist.x.has_limits)
  {
    if (!withinTolerance(next_command.linear.x, expected_twist.x.value, expected_twist.x.upper_limit,
                         expected_twist.x.lower_limit))
    {
      ROS_ERROR("x velocity violates limits");
      next_command = geometry_msgs::Twist();
      return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
    }
  }

  if (expected_twist.y.has_limits)
  {
    if (!withinTolerance(next_command.linear.y, expected_twist.y.value, expected_twist.y.upper_limit,
                         expected_twist.y.lower_limit))
    {
      ROS_ERROR("y velocity rotation violates limits");
      next_command = geometry_msgs::Twist();
      return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
    }
  }

  ROS_DEBUG_STREAM("found command: " << next_command);
  return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::COMMAND_FOUND;
}

bool BasePathFollowerWrapper::isGoalReached()
{
  return planner_->isGoalReached();
}

void BasePathFollowerWrapper::calcualteCurrentIndex()
{
  double min_distance = std::numeric_limits<double>::max();
  size_t best_index = current_index_;

  for (size_t i = current_index_; i < current_trajectory_.movements.size(); ++i)
  {
    const arti_nav_core_msgs::Pose2DWithLimits current_point = current_trajectory_.movements[i].pose;
    double current_distance = std::hypot(current_position_.position.x - current_point.point.x.value,
                                         current_position_.position.y - current_point.point.y.value);

    if (current_distance <= min_distance)
    {
      min_distance = current_distance;
      best_index = i;
    }
    else
      break;
  }

  current_index_ = best_index;
}

void BasePathFollowerWrapper::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> odom_guard(odometry_mutex_);

  current_position_ = msg->pose.pose;
}

bool BasePathFollowerWrapper::withinTolerance(double current_value, double goal_value, double uper_bound,
                                              double lower_bound, bool is_angular)
{
  double distance = current_value - goal_value;
  if (is_angular)
    distance = angles::shortest_angular_distance(current_value, goal_value);

  if (distance > uper_bound)
    return false;

  if (distance < lower_bound)
    return false;

  return true;
}
}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BasePathFollowerWrapper, arti_nav_core::BasePathFollower)
