/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_global_planner_wrapper.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <tf/exceptions.h>

namespace arti_wrap_old_nav_core
{
BaseGlobalPlannerWrapper::BaseGlobalPlannerWrapper()
  : plugin_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    tf_(nullptr), costmap_(nullptr)
{}

bool BaseGlobalPlannerWrapper::setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits &goal)
{
  geometry_msgs::PoseStamped planner_goal;
  planner_goal.header = goal.header;
  planner_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal.pose.theta.value);
  planner_goal.pose.position.x = goal.pose.point.x.value;
  planner_goal.pose.position.y = goal.pose.point.y.value;

  const std::string target_frame = costmap_->getGlobalFrameID();
  if (!tf_->waitForTransform(target_frame, planner_goal.header.frame_id, planner_goal.header.stamp, ros::Duration(1.)))
  {
    ROS_ERROR_STREAM("Can not transform from: " << planner_goal.header.frame_id << " to " << target_frame);
    return false;
  }

  try
  {
    tf_->transformPose(target_frame, planner_goal, goal_);
  }
  catch (const tf::TransformException &exeption)
  {
    ROS_ERROR_STREAM("Can not transform from: " << planner_goal.header.frame_id << " to " << target_frame);
    return false;
  }

  goal_limits_ = goal;

  return true;
}

arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum BaseGlobalPlannerWrapper::makePlan(
    arti_nav_core_msgs::Path2DWithLimits &plan)
{
  plan.poses.clear();
  // Get current pose of robot as start pose of plan:
  tf::Stamped<tf::Pose> start_pose;
  if (!costmap_->getRobotPose(start_pose))
  {
    ROS_ERROR_STREAM("can't get current robot pose for global planer");
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }
  geometry_msgs::PoseStamped start_pose_msg;
  tf::poseStampedTFToMsg(start_pose, start_pose_msg);

  std::vector<geometry_msgs::PoseStamped> planner_plan;
  if (!planner_->makePlan(start_pose_msg, goal_, planner_plan))
  {
    ROS_ERROR_STREAM("can't find path from " << start_pose_msg << " to " << goal_);
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  if (planner_plan.empty())
  {
    ROS_ERROR_STREAM("empty plan for " << start_pose_msg << " to " << goal_);
    return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  plan.header = planner_plan.front().header;
  plan.poses.resize(planner_plan.size());
  for (size_t i = 0; i < planner_plan.size(); ++i)
  {
    plan.poses[i].point.x.value = planner_plan[i].pose.position.x;
    plan.poses[i].point.y.value = planner_plan[i].pose.position.y;

    plan.poses[i].theta.value = tf::getYaw(planner_plan[i].pose.orientation);
  }

  plan.poses.back().point.x.has_limits = goal_limits_.pose.point.x.has_limits;
  plan.poses.back().point.x.lower_limit = goal_limits_.pose.point.x.lower_limit;
  plan.poses.back().point.x.upper_limit = goal_limits_.pose.point.x.upper_limit;

  plan.poses.back().point.y.has_limits = goal_limits_.pose.point.y.has_limits;
  plan.poses.back().point.y.lower_limit = goal_limits_.pose.point.y.lower_limit;
  plan.poses.back().point.y.upper_limit = goal_limits_.pose.point.y.upper_limit;

  plan.poses.back().theta.has_limits = goal_limits_.pose.theta.has_limits;
  plan.poses.back().theta.lower_limit = goal_limits_.pose.theta.lower_limit;
  plan.poses.back().theta.upper_limit = goal_limits_.pose.theta.upper_limit;

  ROS_DEBUG_STREAM("BaseGlobalPlannerWrapper::makePlan goal_limits: " << goal_limits_);
  ROS_DEBUG_STREAM("BaseGlobalPlannerWrapper::makePlan plan: " << plan);

  return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::PLAN_FOUND;
}

void BaseGlobalPlannerWrapper::initialize(std::string name,
                                          tf::TransformListener *tf,
                                          costmap_2d::Costmap2DROS *costmap_ros)
{
  tf_ = tf;
  costmap_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);
  std::string ros_global_planer_name;
  if (!private_nh.getParam("global_planer_name", ros_global_planer_name))
    throw std::runtime_error("no local planner specified to wrap");

  try
  {
    planner_ = plugin_loader_.createInstance(ros_global_planer_name);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Failed to load the " << ros_global_planer_name << " with fault " << ex.what());
    throw ex;
  }

  std::string plugin_name = plugin_loader_.getName(ros_global_planer_name);
  planner_->initialize(plugin_name, costmap_ros);
}
}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BaseGlobalPlannerWrapper, arti_nav_core::BaseGlobalPlanner)
