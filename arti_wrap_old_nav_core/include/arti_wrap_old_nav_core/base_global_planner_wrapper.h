/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_WRAP_OLD_NAV_CORE_BASE_GLOBAL_PLANNER_WRAPPER_H
#define ARTI_WRAP_OLD_NAV_CORE_BASE_GLOBAL_PLANNER_WRAPPER_H

#include <arti_nav_core/base_global_planner.h>
#include <string>
#include <vector>

#ifdef __GNUC__
#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <pluginlib/class_loader.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>

#pragma GCC diagnostic pop
#endif


namespace arti_wrap_old_nav_core
{
class BaseGlobalPlannerWrapper : public arti_nav_core::BaseGlobalPlanner
{
public:
  BaseGlobalPlannerWrapper();

  bool setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) override;

  arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum makePlan(
      arti_nav_core_msgs::Path2DWithLimits& plan) override;

  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

private:
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> plugin_loader_;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;

  geometry_msgs::PoseStamped goal_;
  arti_nav_core_msgs::Pose2DStampedWithLimits goal_limits_;

  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS* costmap_;
};
}  // namespace arti_wrap_old_nav_core

#endif  // ARTI_WRAP_OLD_NAV_CORE_BASE_GLOBAL_PLANNER_WRAPPER_H
