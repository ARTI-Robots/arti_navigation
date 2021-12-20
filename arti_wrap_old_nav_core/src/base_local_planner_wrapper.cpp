/*
Created by clemens on 22.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_wrap_old_nav_core/base_local_planner_wrapper.h>
#include <arti_nav_core_utils/conversions.h>
#include <arti_wrap_old_nav_core/utils.h>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

namespace arti_wrap_old_nav_core
{

const char BaseLocalPlannerWrapper::LOGGER_NAME[] = "base_local_planner_wrapper";

BaseLocalPlannerWrapper::BaseLocalPlannerWrapper()
  : plugin_loader_("nav_core", "nav_core::BaseLocalPlanner"), private_nh_("~")
{
}

void BaseLocalPlannerWrapper::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros)
{
  private_nh_ = ros::NodeHandle("~/" + name);
  const std::string wrapped_type = private_nh_.param<std::string>("wrapped_type", {});
  if (wrapped_type.empty())
  {
    throw std::runtime_error("no local planner specified to wrap");
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "local planner wrapper global frame id: " << costmap_ros->getGlobalFrameID());
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME,
                         "local planner wrapper robot footprint polygon: " << costmap_ros->getRobotFootprintPolygon());

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

  const std::string local_plan_topic = private_nh_.param<std::string>("local_plan_topic", "local_plan");
  local_plan_subscriber_ = private_nh_.subscribe(local_plan_topic, 1, &BaseLocalPlannerWrapper::localPlanCB, this);
}

bool BaseLocalPlannerWrapper::isGoalReached()
{
  return planner_->isGoalReached();
}

arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum BaseLocalPlannerWrapper::makeTrajectory(
  arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  if (!planner_)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "planner not initialized");
    return BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseLocalPlannerWrapper::makeTrajectory: 1");
  std::unique_lock<std::mutex> local_plan_lock(local_plan_mutex_);
  local_plan_.reset();

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "BaseLocalPlannerWrapper::makeTrajectory: 2");

  if (planner_->isGoalReached())
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "no plan needed goal is reached");
    return BaseLocalPlannerErrorEnum::GOAL_REACHED;
  }

  geometry_msgs::Twist next_cmd_vel;
  if (!planner_->computeVelocityCommands(next_cmd_vel))
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "failed to calculate trajectory");
    return BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
  }

  if (planner_->isGoalReached())
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "goal is reached after computing velocity commands, this is strange");
    return BaseLocalPlannerErrorEnum::GOAL_REACHED;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "found next command: " << next_cmd_vel << ", now waiting for local plan");

  const auto timeout = std::chrono::system_clock::now() + std::chrono::milliseconds(1000);
  while (!local_plan_)
  {
    if (local_plan_condition_variable_.wait_until(local_plan_lock, timeout) == std::cv_status::timeout)
    {
      if (!local_plan_)
      {
        ROS_WARN_STREAM_NAMED(LOGGER_NAME, "timed out waiting for local plan from planner, maybe we subscribed to the"
                                           " wrong topic");
        return BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
      }
    }
  }

  nav_msgs::Path::ConstPtr local_plan;
  local_plan.swap(local_plan_);
  local_plan_lock.unlock();

  convertToTrajectory(*local_plan, trajectory);
  return BaseLocalPlannerErrorEnum::TRAJECTORY_FOUND;
}

bool BaseLocalPlannerWrapper::setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan)
{
  global_plan_ = plan;

  const nav_msgs::Path planner_plan
    = arti_nav_core_utils::convertToPath(plan, arti_nav_core_utils::non_finite_values::PASS_THROUGH);
  return planner_->setPlan(planner_plan.poses);
}

bool BaseLocalPlannerWrapper::setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist)
{
  final_twist_ = final_twist;

  return true;
}

void BaseLocalPlannerWrapper::localPlanCB(const nav_msgs::Path::ConstPtr& local_plan)
{
  std::unique_lock<std::mutex> local_plan_lock(local_plan_mutex_);
  if (!local_plan_)
  {
    local_plan_ = local_plan;
    local_plan_condition_variable_.notify_all();
  }
}

void BaseLocalPlannerWrapper::convertToTrajectory(
  const nav_msgs::Path& local_plan, arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) const
{
  trajectory.movements.clear();
  if (!local_plan.poses.empty())
  {
    trajectory.header = local_plan.poses.front().header;

    trajectory.movements.resize(local_plan.poses.size());

    size_t global_plan_index = 0;
    for (size_t i = 0; i < local_plan.poses.size(); ++i)
    {
      const auto& local_plan_pose = local_plan.poses[i];
      auto& movement = trajectory.movements[i];

      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "local plan pose: " << local_plan_pose);

      movement.pose.point.x.value = local_plan_pose.pose.position.x;
      movement.pose.point.y.value = local_plan_pose.pose.position.y;
      movement.pose.theta.value = tf::getYaw(local_plan_pose.pose.orientation);

      if (!global_plan_.poses.empty())
      {
        const double current_distance = calculateDistance(global_plan_.poses[global_plan_index], local_plan_pose.pose);
        ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "i: " << i << ", global_plan_index: " << global_plan_index
                                                  << ", current_distance: " << current_distance);
        for (size_t j = global_plan_index + 1; j < global_plan_.poses.size(); ++j)
        {
          const double new_distance = calculateDistance(global_plan_.poses[j], local_plan_pose.pose);
          ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "new_distance: " << new_distance);
          if (new_distance <= current_distance)
          {
            global_plan_index = j;
          }
          else
          {
            break;
          }
        }

        const auto& global_plan_pose = global_plan_.poses[global_plan_index];

        ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "i: " << i << ", used global_plan_index: " << global_plan_index
                                                  << ", global plan pose: " << global_plan_pose);

        movement.pose.point.x.has_limits = global_plan_pose.point.x.has_limits;
        movement.pose.point.x.lower_limit = global_plan_pose.point.x.lower_limit;
        movement.pose.point.x.upper_limit = global_plan_pose.point.x.upper_limit;

        movement.pose.point.y.has_limits = global_plan_pose.point.y.has_limits;
        movement.pose.point.y.lower_limit = global_plan_pose.point.y.lower_limit;
        movement.pose.point.y.upper_limit = global_plan_pose.point.y.upper_limit;

        movement.pose.theta.has_limits = global_plan_pose.theta.has_limits;
        movement.pose.theta.lower_limit = global_plan_pose.theta.lower_limit;
        movement.pose.theta.upper_limit = global_plan_pose.theta.upper_limit;
      }

      // Set twist to be non-finite to indicate no constraint at all:
      movement.twist.x.value = std::numeric_limits<double>::infinity();
      movement.twist.y.value = std::numeric_limits<double>::infinity();
      movement.twist.theta.value = std::numeric_limits<double>::infinity();
    }

    trajectory.movements.back().twist = final_twist_;
  }
}

}  // namespace arti_wrap_old_nav_core

PLUGINLIB_EXPORT_CLASS(arti_wrap_old_nav_core::BaseLocalPlannerWrapper, arti_nav_core::BaseLocalPlanner)
