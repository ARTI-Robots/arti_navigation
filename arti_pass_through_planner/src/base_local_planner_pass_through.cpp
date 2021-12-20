/*
Created by clemens on 11.10.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_pass_through_planner/base_local_planner_pass_through.h>
#include <angles/angles.h>
#include <arti_nav_core_utils/transformer.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arti_pass_through_planner
{
bool BaseLocalPlannerPassThrough::setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan)
{
  trajectory_.header = plan.header;

  trajectory_.movements.resize(plan.poses.size());
  for (size_t i = 0; i < plan.poses.size(); ++i)
  {
    trajectory_.movements[i].pose = plan.poses[i];
    trajectory_.movements[i].twist.x.value = std::numeric_limits<double>::infinity();
    trajectory_.movements[i].twist.y.value = std::numeric_limits<double>::infinity();
    trajectory_.movements[i].twist.theta.value = std::numeric_limits<double>::infinity();
  }

  return true;
}

bool BaseLocalPlannerPassThrough::setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist)
{
  if (!trajectory_.movements.empty())
  {
    trajectory_.movements.back().twist = final_twist;
  }

  return true;
}

arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum BaseLocalPlannerPassThrough::makeTrajectory(
  arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  trajectory = trajectory_;

  return arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::TRAJECTORY_FOUND;
}

void BaseLocalPlannerPassThrough::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* /*costmap_ros*/)
{
  private_nh_ = ros::NodeHandle("~/" + name);
  transformer_ = transformer;

  robot_information_ = std::make_shared<RobotInformation>(private_nh_);

  xy_close_tolerance_increase_ = private_nh_.param<double>("xy_close_tolerance_increase", 1.0);
  yaw_close_tolerance_increase_ = private_nh_.param<double>("yaw_close_tolerance_increase", 1.0);
}

bool BaseLocalPlannerPassThrough::isGoalReached()
{
  if (!robot_information_)
  {
    return false;
  }

  if (trajectory_.movements.empty())
  {
    return true;
  }

  boost::optional<geometry_msgs::PoseStamped> current_pose_tfd;
  {
    std::unique_lock<RobotInformation> information_guard(*robot_information_);
    const geometry_msgs::PoseStamped current_pose = robot_information_->getRobotPose();
    information_guard.unlock();

    std::string tf_error_message;
    current_pose_tfd = arti_nav_core_utils::tryToTransform(*transformer_, current_pose, trajectory_.header.frame_id,
                                                           ros::Duration(1.0), &tf_error_message);
    if (!current_pose_tfd)
    {
      ROS_ERROR_STREAM("failed to transform current pose to trajectory frame: " << tf_error_message);
      return false;
    }
  }

  const arti_nav_core_msgs::Pose2DWithLimits& goal = trajectory_.movements.back().pose;

  return xValueWithinTolerance(current_pose_tfd->pose.position.x, goal.point.x)
         && yValueWithinTolerance(current_pose_tfd->pose.position.y, goal.point.y)
         && thetaValueWithinTolerance(tf::getYaw(current_pose_tfd->pose.orientation), goal.theta);
}

bool BaseLocalPlannerPassThrough::xValueWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const
{
  return withinTolerance(current_value, goal_value.value, goal_value.upper_limit, goal_value.lower_limit,
                         xy_close_tolerance_increase_);
}

bool BaseLocalPlannerPassThrough::yValueWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const
{
  return withinTolerance(current_value, goal_value.value, goal_value.upper_limit, goal_value.lower_limit,
                         xy_close_tolerance_increase_);
}

bool BaseLocalPlannerPassThrough::thetaValueWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const
{
  return withinTolerance(current_value, goal_value.value, goal_value.upper_limit, goal_value.lower_limit,
                         yaw_close_tolerance_increase_, true);
}

bool BaseLocalPlannerPassThrough::withinTolerance(
  double current_value, double goal_value, double upper_bound, double lower_bound, double tolerance_increase,
  bool is_angular)
{
  if (!std::isfinite(goal_value))
  {
    return true;
  }

  double distance = current_value - goal_value;
  if (is_angular)
  {
    distance = angles::shortest_angular_distance(current_value, goal_value);
  }

  distance /= tolerance_increase;

  return (lower_bound <= distance) && (distance <= upper_bound);
}
}

PLUGINLIB_EXPORT_CLASS(arti_pass_through_planner::BaseLocalPlannerPassThrough, arti_nav_core::BaseLocalPlanner)
