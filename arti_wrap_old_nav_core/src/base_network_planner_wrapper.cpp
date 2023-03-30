/*
Created by clemens on 27.09.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_network_planner_wrapper.h>
#include <arti_nav_core_utils/transformer.h>
#include <arti_wrap_old_nav_core/utils.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <memory>

namespace arti_wrap_old_nav_core
{

const char BaseNetworkPlannerWrapper::LOGGER_NAME[] = "base_network_planner_wrapper";

BaseNetworkPlannerWrapper::BaseNetworkPlannerWrapper()
  : plugin_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{
}

void BaseNetworkPlannerWrapper::initialize(std::string name, arti_nav_core::Transformer* transformer)
{
  nh_ = ros::NodeHandle("~/" + name);

  corridor_width_ = nh_.param<double>("corridor_width", 1.);

  transformer_ = transformer;
  costmap_.reset(new costmap_2d::Costmap2DROS("network_costmap", *transformer_));

  const std::string wrapped_type = nh_.param<std::string>("wrapped_type", {});
  if (wrapped_type.empty())
  {
    throw std::runtime_error("no global planner specified to wrap");
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

  planner_->initialize(name, costmap_.get());
}

bool BaseNetworkPlannerWrapper::setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal)
{
  goal_ = goal;

  return true;
}

arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum BaseNetworkPlannerWrapper::makePlan(
  arti_nav_core_msgs::Movement2DGoalWithConstraints& plan)
{
  geometry_msgs::PoseStamped planner_goal;
  planner_goal.header = goal_.header;
  if (std::isfinite(goal_.pose.theta.value))
  {
    planner_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.pose.theta.value);
  }
  else
  {
    planner_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.);
  }
  planner_goal.pose.position.x = goal_.pose.point.x.value;
  planner_goal.pose.position.y = goal_.pose.point.y.value;

  std::string tf_error_message;
  const auto planner_goal_tfd = arti_nav_core_utils::tryToTransform(*transformer_, planner_goal,
                                                                    costmap_->getGlobalFrameID(), ros::Duration(1.0),
                                                                    &tf_error_message);
  if (!planner_goal_tfd)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "failed to transform goal: " << tf_error_message);
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  plan.path_limits.poses.clear();

#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  geometry_msgs::PoseStamped start_pose_msg;
  if (!costmap_->getRobotPose(start_pose_msg))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't get current robot pose for global planer");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }
#else // if KINETIC (v12)
  // Get current pose of robot as start pose of plan:
  tf::Stamped<tf::Pose> start_pose;
  if (!costmap_->getRobotPose(start_pose))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't get current robot pose for global planer");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }
  geometry_msgs::PoseStamped start_pose_msg;
  tf::poseStampedTFToMsg(start_pose, start_pose_msg);
#endif

  std::vector<geometry_msgs::PoseStamped> planner_plan;
  if (!planner_->makePlan(start_pose_msg, *planner_goal_tfd, planner_plan))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't find path from " << start_pose_msg << " to " << goal_);
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  if (planner_plan.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "empty plan for " << start_pose_msg << " to " << goal_);
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  plan.path_limits.header = planner_plan.front().header;
  plan.path_limits.poses.resize(planner_plan.size());
  for (size_t i = 0; i < planner_plan.size(); ++i)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseNetworkPlannerWrapper::makePlan planner_plan[i]: " << planner_plan[i]);

    plan.path_limits.poses[i].point.x.value = planner_plan[i].pose.position.x;
    plan.path_limits.poses[i].point.x.has_limits = true;
    plan.path_limits.poses[i].point.x.lower_limit = -corridor_width_;
    plan.path_limits.poses[i].point.x.upper_limit = corridor_width_;

    plan.path_limits.poses[i].point.y.value = planner_plan[i].pose.position.y;
    plan.path_limits.poses[i].point.y.has_limits = true;
    plan.path_limits.poses[i].point.y.lower_limit = -corridor_width_;
    plan.path_limits.poses[i].point.y.upper_limit = corridor_width_;

    plan.path_limits.poses[i].theta.value = tf::getYaw(planner_plan[i].pose.orientation);
    plan.path_limits.poses[i].theta.has_limits = true;
    plan.path_limits.poses[i].theta.lower_limit = -M_PI_4;
    plan.path_limits.poses[i].theta.upper_limit = +M_PI_4;
  }

  // ensuring goal constraints
  plan.path_limits.poses.back().point.x.has_limits = goal_.pose.point.x.has_limits;
  plan.path_limits.poses.back().point.x.lower_limit = goal_.pose.point.x.lower_limit;
  plan.path_limits.poses.back().point.x.upper_limit = goal_.pose.point.x.upper_limit;

  plan.path_limits.poses.back().point.y.has_limits = goal_.pose.point.y.has_limits;
  plan.path_limits.poses.back().point.y.lower_limit = goal_.pose.point.y.lower_limit;
  plan.path_limits.poses.back().point.y.upper_limit = goal_.pose.point.y.upper_limit;

  if (std::isfinite(goal_.pose.theta.value))
  {
    plan.path_limits.poses.back().theta.has_limits = goal_.pose.theta.has_limits;
    plan.path_limits.poses.back().theta.lower_limit = goal_.pose.theta.lower_limit;
    plan.path_limits.poses.back().theta.upper_limit = goal_.pose.theta.upper_limit;
  }
  else
  {
    // as we want to next planer to give a hint how the orientation should be we apply a very lose constraint here instead of removing the orientation
    plan.path_limits.poses.back().theta.has_limits = true;
    plan.path_limits.poses.back().theta.lower_limit = -M_PI;
    plan.path_limits.poses.back().theta.upper_limit = M_PI;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseNetworkPlannerWrapper::makePlan goal_limits: " << goal_);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseNetworkPlannerWrapper::makePlan plan: " << plan);

  return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::PLAN_FOUND;
}

}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BaseNetworkPlannerWrapper, arti_nav_core::BaseNetworkPlanner)
