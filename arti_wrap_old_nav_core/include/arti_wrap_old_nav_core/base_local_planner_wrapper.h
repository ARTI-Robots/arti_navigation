/*
Created by clemens on 22.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_WRAP_OLD_NAV_CORE_BASE_LOCAL_PLANNER_WRAPPER_H
#define ARTI_WRAP_OLD_NAV_CORE_BASE_LOCAL_PLANNER_WRAPPER_H

#include <arti_nav_core/base_local_planner.h>
#include <condition_variable>
#include <mutex>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

namespace arti_wrap_old_nav_core
{

class BaseLocalPlannerWrapper : public arti_nav_core::BaseLocalPlanner
{
public:
  BaseLocalPlannerWrapper();

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan) override;

  bool setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist) override;

  arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum makeTrajectory(
    arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  bool isGoalReached() override;

private:
  static const char LOGGER_NAME[];

  void localPlanCB(const nav_msgs::Path::ConstPtr& local_plan);

  /**
   * Convert a local plan to a trajectory with the help of the global plan and final twist.
   */
  void convertToTrajectory(
    const nav_msgs::Path& local_plan, arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) const;

  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> plugin_loader_;
  boost::shared_ptr<nav_core::BaseLocalPlanner> planner_;

  ros::NodeHandle private_nh_;
  ros::Subscriber local_plan_subscriber_;
  std::mutex local_plan_mutex_;
  std::condition_variable local_plan_condition_variable_;
  nav_msgs::Path::ConstPtr local_plan_;

  arti_nav_core_msgs::Twist2DWithLimits final_twist_;
  arti_nav_core_msgs::Path2DWithLimits global_plan_;
};

}  // namespace arti_wrap_old_nav_core

#endif  // ARTI_WRAP_OLD_NAV_CORE_BASE_LOCAL_PLANNER_WRAPPER_H
