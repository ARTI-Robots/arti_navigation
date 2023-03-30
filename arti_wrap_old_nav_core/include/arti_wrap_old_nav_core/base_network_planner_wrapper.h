/*
Created by clemens on 27.09.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_WRAP_OLD_NAV_CORE_BASE_NETWORK_PLANNER_WRAPPER_H
#define ARTI_WRAP_OLD_NAV_CORE_BASE_NETWORK_PLANNER_WRAPPER_H


#include <arti_nav_core/base_network_planner.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include <nav_core/base_global_planner.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_wrap_old_nav_core
{

class BaseNetworkPlannerWrapper : public arti_nav_core::BaseNetworkPlanner
{
public:
  BaseNetworkPlannerWrapper();

  void initialize(std::string name, arti_nav_core::Transformer* transformer) override;

  bool setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) override;

  arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum makePlan(
    arti_nav_core_msgs::Movement2DGoalWithConstraints& plan) override;

private:
  static const char LOGGER_NAME[];

  ros::NodeHandle nh_;

  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> plugin_loader_;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;

  arti_nav_core_msgs::Pose2DStampedWithLimits goal_;

  arti_nav_core::Transformer* transformer_{nullptr};
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap_;

  double corridor_width_;
};

}  // namespace arti_wrap_old_nav_core


#endif //ARTI_WRAP_OLD_NAV_CORE_BASE_NETWORK_PLANNER_WRAPPER_H
