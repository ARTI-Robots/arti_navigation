/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_global_planner_wrapper.h>
#include <arti_nav_core_utils/transformer.h>
#include <arti_wrap_old_nav_core/utils.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

namespace arti_wrap_old_nav_core
{

const char BaseGlobalPlannerWrapper::LOGGER_NAME[] = "base_global_planner_wrapper";

BaseGlobalPlannerWrapper::BaseGlobalPlannerWrapper()
  : plugin_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{
}

void BaseGlobalPlannerWrapper::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros)
{
  transformer_ = transformer;
  costmap_ = costmap_ros;

  const ros::NodeHandle private_nh("~/" + name);
  const std::string wrapped_type = private_nh.param<std::string>("wrapped_type", {});
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

  planner_->initialize(name, costmap_ros);
}

bool BaseGlobalPlannerWrapper::setGoal(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& goal, const arti_nav_core_msgs::Path2DWithLimits& path_limits)
{
  geometry_msgs::PoseStamped planner_goal;
  planner_goal.header = goal.header;
  if (std::isfinite(goal.pose.theta.value))
  {
    planner_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal.pose.theta.value);
  }
  else
  {
    planner_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.);
  }
  planner_goal.pose.position.x = goal.pose.point.x.value;
  planner_goal.pose.position.y = goal.pose.point.y.value;

  std::string tf_error_message;
  const auto planner_goal_tfd = arti_nav_core_utils::tryToTransform(*transformer_, planner_goal,
                                                                    costmap_->getGlobalFrameID(), ros::Duration(1.0),
                                                                    &tf_error_message);
  if (!planner_goal_tfd)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "failed to transform goal: " << tf_error_message);
    return false;
  }
  goal_ = *planner_goal_tfd;

  // TODO: transform path limits:
  path_limits_ = path_limits;

  // Ensure goal constraints are applied at end of path:
  if (path_limits_.poses.empty())
  {
    path_limits_.poses.push_back(goal.pose);
  }
  else
  {
    path_limits_.poses.back() = goal.pose;
  }

  return true;
}

arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum BaseGlobalPlannerWrapper::makePlan(
  arti_nav_core_msgs::Path2DWithLimits& plan)
{
  plan.poses.clear();

#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  geometry_msgs::PoseStamped start_pose_msg;
  if (!costmap_->getRobotPose(start_pose_msg))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't get current robot pose for global planer");
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }
#else // if KINETIC (v12)
  // Get current pose of robot as start pose of plan:
  tf::Stamped<tf::Pose> start_pose;
  if (!costmap_->getRobotPose(start_pose))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't get current robot pose for global planer");
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }
  geometry_msgs::PoseStamped start_pose_msg;
  tf::poseStampedTFToMsg(start_pose, start_pose_msg);
#endif

  std::vector<geometry_msgs::PoseStamped> planner_plan;
  if (!planner_->makePlan(start_pose_msg, goal_, planner_plan))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't find path from " << start_pose_msg << " to " << goal_);
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  if (planner_plan.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "empty plan for " << start_pose_msg << " to " << goal_);
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  plan.header = planner_plan.front().header;
  plan.poses.resize(planner_plan.size());
  size_t path_limits_index = 0;
  for (size_t i = 0; i < planner_plan.size(); ++i)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseGlobalPlannerWrapper::makePlan planner_plan[i]: " << planner_plan[i]);

    // check if next path index better fit the current path pose
    boost::optional<size_t> new_index = boost::make_optional<size_t>(false, 0);

    double current_distance = calculateDistance(path_limits_.poses[path_limits_index], planner_plan[i].pose);
    for (size_t j = path_limits_index + 1; j < path_limits_.poses.size(); ++j)
    {
      double new_distance = calculateDistance(path_limits_.poses[j], planner_plan[i].pose);
      if (new_distance <= current_distance)
      {
        new_index = j;
      }
      else
      {
        break;
      }
    }

    if (new_index)
    {
      path_limits_index = *new_index;
    }

    plan.poses[i].point.x.value = planner_plan[i].pose.position.x;
    plan.poses[i].point.x.has_limits = path_limits_.poses[path_limits_index].point.x.has_limits;
    plan.poses[i].point.x.lower_limit = path_limits_.poses[path_limits_index].point.x.lower_limit;
    plan.poses[i].point.x.upper_limit = path_limits_.poses[path_limits_index].point.x.upper_limit;

    plan.poses[i].point.y.value = planner_plan[i].pose.position.y;
    plan.poses[i].point.y.has_limits = path_limits_.poses[path_limits_index].point.y.has_limits;
    plan.poses[i].point.y.lower_limit = path_limits_.poses[path_limits_index].point.y.lower_limit;
    plan.poses[i].point.y.upper_limit = path_limits_.poses[path_limits_index].point.y.upper_limit;

    plan.poses[i].theta.value = tf::getYaw(planner_plan[i].pose.orientation);
    if (std::isfinite(path_limits_.poses[path_limits_index].theta.value))
    {
      plan.poses[i].theta.has_limits = path_limits_.poses[path_limits_index].theta.has_limits;
      plan.poses[i].theta.lower_limit = path_limits_.poses[path_limits_index].theta.lower_limit;
      plan.poses[i].theta.upper_limit = path_limits_.poses[path_limits_index].theta.upper_limit;
    }
    else
    {
      // as we want to next planer to give a hint how the orientation should be we apply a very lose constraint here instead of removing the orientation
      plan.poses[i].theta.has_limits = true;
      plan.poses[i].theta.lower_limit = -M_PI;
      plan.poses[i].theta.upper_limit = M_PI;
    }
  }

  // ensuring goal constraints
  plan.poses.back().point.x.has_limits = path_limits_.poses.back().point.x.has_limits;
  plan.poses.back().point.x.lower_limit = path_limits_.poses.back().point.x.lower_limit;
  plan.poses.back().point.x.upper_limit = path_limits_.poses.back().point.x.upper_limit;

  plan.poses.back().point.y.has_limits = path_limits_.poses.back().point.y.has_limits;
  plan.poses.back().point.y.lower_limit = path_limits_.poses.back().point.y.lower_limit;
  plan.poses.back().point.y.upper_limit = path_limits_.poses.back().point.y.upper_limit;

  if (std::isfinite(path_limits_.poses.back().theta.value))
  {
    plan.poses.back().theta.has_limits = path_limits_.poses.back().theta.has_limits;
    plan.poses.back().theta.lower_limit = path_limits_.poses.back().theta.lower_limit;
    plan.poses.back().theta.upper_limit = path_limits_.poses.back().theta.upper_limit;
  }
  else
  {
    // as we want to next planer to give a hint how the orientation should be we apply a very lose constraint here instead of removing the orientation
    plan.poses.back().theta.has_limits = true;
    plan.poses.back().theta.lower_limit = -M_PI;
    plan.poses.back().theta.upper_limit = M_PI;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseGlobalPlannerWrapper::makePlan path_limits: " << path_limits_);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseGlobalPlannerWrapper::makePlan goal_limits: " << plan.poses.back());
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseGlobalPlannerWrapper::makePlan plan: " << plan);

  return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::PLAN_FOUND;
}

}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BaseGlobalPlannerWrapper, arti_nav_core::BaseGlobalPlanner)
