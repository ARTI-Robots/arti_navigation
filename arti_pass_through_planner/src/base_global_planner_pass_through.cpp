/*
Created by clemens on 11.10.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_pass_through_planner/base_global_planner_pass_through.h>
#include <arti_nav_core_utils/tf2_arti_nav_core_msgs.h>
#include <arti_nav_core_utils/transformer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

namespace arti_pass_through_planner
{

void BaseGlobalPlannerPassThrough::initialize(
  std::string /*name*/, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* /*costmap_ros*/)
{
  transformer_ = transformer;
}

bool BaseGlobalPlannerPassThrough::setGoal(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& goal, const arti_nav_core_msgs::Path2DWithLimits& path_limits)
{
  if (path_limits.poses.empty())
  {
    plan_.header = goal.header;
    plan_.poses = {goal.pose};
  }
  else
  {
    plan_ = path_limits;

    std::string tf_error_message;
    const auto goal_tfd = arti_nav_core_utils::tryToTransform(*transformer_, goal, path_limits.header.frame_id,
                                                              ros::Duration(0.1), &tf_error_message);
    if (!goal_tfd)
    {
      ROS_ERROR_STREAM("failed to transform global planner goal: " << tf_error_message);
      return false;
    }

    if (!equal(plan_.poses.back(), goal_tfd->pose))
    {
      // only add the goal if not already in the path
      plan_.poses.push_back(goal_tfd->pose);
    }
    else
    {
      ROS_WARN_STREAM("skipped goal pose, because its already in the plan");
    }
  }

  return true;
}

arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum BaseGlobalPlannerPassThrough::makePlan(
  arti_nav_core_msgs::Path2DWithLimits& plan)
{
  plan = plan_;

  return arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::PLAN_FOUND;
}


bool BaseGlobalPlannerPassThrough::equal(
  const arti_nav_core_msgs::Pose2DWithLimits& pose_a, const arti_nav_core_msgs::Pose2DWithLimits& pose_b)
{
  return pose_a.point.x.value == pose_b.point.x.value
         && pose_a.point.y.value == pose_b.point.y.value
         && pose_a.theta.value == pose_b.theta.value;
}

}

PLUGINLIB_EXPORT_CLASS(arti_pass_through_planner::BaseGlobalPlannerPassThrough, arti_nav_core::BaseGlobalPlanner)
