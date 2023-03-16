/*
Created by abuchegger on 2021-01-23.
This file is part of the software provided by ARTI
Copyright (c) 2021, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_pass_through_planner/base_network_planner_pass_through.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

namespace arti_pass_through_planner
{

void BaseNetworkPlannerPassThrough::initialize(std::string /*name*/, arti_nav_core::Transformer* /*transformer*/)
{
  // Nothing to do here.
}

bool BaseNetworkPlannerPassThrough::setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal)
{
  path_.emplace();
  path_->header = goal.header;
  path_->poses.emplace_back(goal.pose);
  return true;
}

arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum BaseNetworkPlannerPassThrough::makePlan(arti_nav_core_msgs::Movement2DGoalWithConstraints& plan)
{
  if (path_->poses.empty())
  {
    ROS_ERROR_STREAM("BaseNetworkPlannerPassThrough::makePlan called before goal was set");
    return BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  plan.path_limits = *path_;
  return BaseNetworkPlannerErrorEnum::PLAN_FOUND;
}

void BaseNetworkPlannerPassThrough::handlePlannerError(
  const arti_nav_core_msgs::Pose2DWithLimits& /*error_pose_a*/,
  const arti_nav_core_msgs::Pose2DWithLimits& /*error_pose_b*/)
{
  // Ignored.
}

}

PLUGINLIB_EXPORT_CLASS(arti_pass_through_planner::BaseNetworkPlannerPassThrough, arti_nav_core::BaseNetworkPlanner)
