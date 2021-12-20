/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_path_follower_wrapper.h>
#include <arti_nav_core_utils/conversions.h>
#include <angles/angles.h>
#include <arti_wrap_old_nav_core/utils.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <tf/transform_datatypes.h>

namespace arti_wrap_old_nav_core
{

const char BasePathFollowerWrapper::LOGGER_NAME[] = "base_path_follower_wrapper";

BasePathFollowerWrapper::BasePathFollowerWrapper()
  : plugin_loader_("nav_core", "nav_core::BaseLocalPlanner")
{
}

void BasePathFollowerWrapper::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros)
{
  private_nh_ = ros::NodeHandle("~/" + name);
  const std::string wrapped_type = private_nh_.param<std::string>("wrapped_type", {});
  if (wrapped_type.empty())
  {
    throw std::runtime_error("no path follower (local planner) specified to wrap");
  }

  try
  {
    planner_ = plugin_loader_.createInstance(wrapped_type);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM_NAMED(LOGGER_NAME, "Failed to load the '" << wrapped_type << "' plugin: " << ex.what());
    throw;
  }

  planner_->initialize(name, transformer, costmap_ros);

  odom_sub_ = private_nh_.subscribe("/odom", 1, &BasePathFollowerWrapper::odomCB, this);
}

bool BasePathFollowerWrapper::setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  if (trajectory.movements.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "trajectory is empty");
    return false;
  }

  current_trajectory_ = trajectory;
  current_index_ = 0;

  const nav_msgs::Path planner_plan
    = arti_nav_core_utils::convertToPath(current_trajectory_, arti_nav_core_utils::non_finite_values::PASS_THROUGH);
  return planner_->setPlan(planner_plan.poses);
}

arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum BasePathFollowerWrapper::computeVelocityCommands(
  geometry_msgs::Twist& next_command)
{
  calculateCurrentIndex();

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
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can not compute velocity");
    next_command = {};
    return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
  }

  // check limits if exist
  // Note that an angular velocity should not be angle-normalized (-pi/s is very different from pi/s)
  if (expected_twist.theta.has_limits && !withinTolerance(next_command.angular.z, expected_twist.theta))
  {
    enforceLimit(next_command, next_command.angular.z, expected_twist.theta, "angular");
  }

  if (expected_twist.x.has_limits && !withinTolerance(next_command.linear.x, expected_twist.x))
  {
    enforceLimit(next_command, next_command.linear.x, expected_twist.x, "x");
  }

  if (expected_twist.y.has_limits && !withinTolerance(next_command.linear.y, expected_twist.y))
  {
    enforceLimit(next_command, next_command.linear.y, expected_twist.y, "y");
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "found command: " << next_command);
  return arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::COMMAND_FOUND;
}

bool BasePathFollowerWrapper::isGoalReached()
{
  return planner_->isGoalReached();
}

void BasePathFollowerWrapper::calculateCurrentIndex()
{
  std::unique_lock<std::mutex> odom_guard(odometry_mutex_);
  const geometry_msgs::Pose current_pose = current_pose_;
  odom_guard.unlock();

  double min_distance = std::numeric_limits<double>::max();
  size_t best_index = current_index_;

  for (size_t i = current_index_; i < current_trajectory_.movements.size(); ++i)
  {
    const arti_nav_core_msgs::Pose2DWithLimits& current_point = current_trajectory_.movements[i].pose;
    const double current_distance = calculateDistance(current_point, current_pose);

    if (current_distance <= min_distance)
    {
      min_distance = current_distance;
      best_index = i;
    }
    else
    {
      break;
    }
  }

  current_index_ = best_index;
}

void BasePathFollowerWrapper::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> odom_guard(odometry_mutex_);
  current_pose_ = msg->pose.pose;
}

bool BasePathFollowerWrapper::withinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value)
{
  const double distance = current_value - goal_value.value;
  return goal_value.lower_limit <= distance && distance <= goal_value.upper_limit;
}

void BasePathFollowerWrapper::enforceLimit(
  geometry_msgs::Twist& next_command, double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value,
  const char* name)
{
  if (!std::isfinite(current_value))
  {
    ROS_WARN_STREAM("current " << name << " velocity is non-finite");
    return;
  }

  if (current_value == 0.0)
  {
    // Cannot compute a ratio.
    return;
  }

  const double min = goal_value.value + goal_value.lower_limit;
  const double max = goal_value.value + goal_value.upper_limit;
  double ratio = 1.0;
  ROS_WARN_STREAM_NAMED(LOGGER_NAME,
                        name << " velocity violates limits: goal: " << goal_value.value << ", min: " << min
                             << ", current: " << current_value << ", max: " << max);

  if (max < current_value)
  {
    ratio = max / current_value;
  }
  else if (min > current_value)
  {
    ratio = min / current_value;
  }

  next_command.angular.z *= ratio;
  next_command.linear.x *= ratio;
  next_command.linear.y *= ratio;
  ROS_WARN_STREAM_NAMED(LOGGER_NAME, "resulting " << name << " velocity: " << current_value * ratio);
}

}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BasePathFollowerWrapper, arti_nav_core::BasePathFollower
)
