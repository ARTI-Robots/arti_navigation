/*
Created by clemens on 22.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_local_planner_wrapper.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>

namespace arti_wrap_old_nav_core
{
BaseLocalPlannerWrapper::BaseLocalPlannerWrapper() : plugin_loader_("nav_core", "nav_core::BaseLocalPlanner"),
                                                     private_nh_("~")
{}

void BaseLocalPlannerWrapper::initialize(std::string name,
                                         tf::TransformListener *tf,
                                         costmap_2d::Costmap2DROS *costmap_ros)
{
  private_nh_ = ros::NodeHandle("~/" + name);
  std::string ros_local_planer_name;
  if (!private_nh_.getParam("local_planer_name", ros_local_planer_name))
    throw std::runtime_error("no local planner specified to wrap");

  ROS_INFO_STREAM("local planner wrapper global frame id: " << costmap_ros->getGlobalFrameID());
  ROS_INFO_STREAM("local planner wrapper robot footprint polygon: " << costmap_ros->getRobotFootprintPolygon());

  try
  {
    planner_ = plugin_loader_.createInstance(ros_local_planer_name);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Failed to load the " << ros_local_planer_name << " with fault " << ex.what());
    throw ex;
  }

  std::string plugin_name = plugin_loader_.getName(ros_local_planer_name);
  planner_->initialize(plugin_name, tf, costmap_ros);

  std::string plan_topic_name = private_nh_.param<std::string>("plan_topic_name", "local_plan");

  private_nh_ = ros::NodeHandle("~/" + plugin_name);
  trajectory_sub_ = private_nh_.subscribe<nav_msgs::Path>(plan_topic_name, 1, &BaseLocalPlannerWrapper::localPlanCB, this);
}

bool BaseLocalPlannerWrapper::isGoalReached()
{
  return planner_->isGoalReached();
}

arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum BaseLocalPlannerWrapper::makeTrajectory(
    arti_nav_core_msgs::Trajectory2DWithLimits &trajectory)
{
  if (!planner_)
  {
    ROS_WARN_STREAM("planner not initialized");
    return BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
  }

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::makeTrajectory: 1");
  std::unique_lock<std::mutex> trajectory_lock(trajectory_mutex_);
  received_trajectory_ = false;

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::makeTrajectory: 2");

  if (planner_->isGoalReached())
  {
    ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "no plan needed goal is reached");
    return BaseLocalPlanner::BaseLocalPlannerErrorEnum::GOAL_REACHED;
  }

  geometry_msgs::Twist next_cmd_vel;
  if (!planner_->computeVelocityCommands(next_cmd_vel))
  {
    ROS_WARN_STREAM("failed to calculate trajectory");
    return BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
  }

  if (planner_->isGoalReached())
  {
    ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "goal is reached wtf");
    return BaseLocalPlanner::BaseLocalPlannerErrorEnum::GOAL_REACHED;
  }

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "found next command: " << next_cmd_vel);

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::makeTrajectory: 3");

  while (!received_trajectory_)
    trajectory_condition_variable_.wait(trajectory_lock);

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::makeTrajectory: 4");

  trajectory = trajectory_;

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::makeTrajectory: 5");

  return arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::TRAJECTORY_FOUND;
}

bool BaseLocalPlannerWrapper::setPlan(const arti_nav_core_msgs::Path2DWithLimits &plan)
{
  if (plan.poses.empty())
    return planner_->setPlan(std::vector<geometry_msgs::PoseStamped>());

  std::vector<geometry_msgs::PoseStamped> planner_plan(plan.poses.size());
  for (size_t i = 0; i < planner_plan.size(); ++i)
  {
    planner_plan[i].header = plan.header;
    planner_plan[i].pose.position.x = plan.poses[i].point.x.value;
    planner_plan[i].pose.position.y = plan.poses[i].point.y.value;
    planner_plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(plan.poses[i].theta.value);
  }

  return planner_->setPlan(planner_plan);
}

bool BaseLocalPlannerWrapper::setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist)
{
  final_twist_ = final_twist;

  return true;
}

void BaseLocalPlannerWrapper::localPlanCB(const nav_msgs::Path::ConstPtr &local_path)
{
  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::localPlanCB: 1");

  std::unique_lock<std::mutex> trajectory_lock(trajectory_mutex_);

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::localPlanCB: 2");

  if (received_trajectory_)
    return;

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::localPlanCB: 3");

  received_trajectory_ = true;

  ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::localPlanCB: 4");

  trajectory_.movements.clear();
  if (!local_path->poses.empty())
  {
    trajectory_.header = local_path->poses.front().header;

    trajectory_.movements.resize(local_path->poses.size());
    for (size_t i = 0; i < local_path->poses.size(); ++i)
    {
      trajectory_.movements[i].pose.point.x.value = local_path->poses[i].pose.position.x;
      trajectory_.movements[i].pose.point.y.value = local_path->poses[i].pose.position.y;
      trajectory_.movements[i].pose.theta.value = tf::getYaw(local_path->poses[i].pose.orientation);

      // set twist to be invalid indication no constraint at all
      trajectory_.movements[i].twist.x.value = std::numeric_limits<double>::infinity();
      trajectory_.movements[i].twist.y.value = std::numeric_limits<double>::infinity();
      trajectory_.movements[i].twist.theta.value = std::numeric_limits<double>::infinity();
    }

    ROS_DEBUG_STREAM_NAMED("base_local_planner_wrapper", "BaseLocalPlannerWrapper::localPlanCB: 5");

    trajectory_.movements.back().twist = final_twist_;
  }

  trajectory_condition_variable_.notify_all();
}
}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BaseLocalPlannerWrapper, arti_nav_core::BaseLocalPlanner)
