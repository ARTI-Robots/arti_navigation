/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_WRAP_OLD_NAV_CORE_BASE_PATH_FOLLOWER_WRAPPER_H
#define ARTI_WRAP_OLD_NAV_CORE_BASE_PATH_FOLLOWER_WRAPPER_H

#include <arti_nav_core/base_path_follower.h>
#include <arti_nav_core_msgs/ValueWithLimits.h>
#include <mutex>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>

namespace arti_wrap_old_nav_core
{

class BasePathFollowerWrapper : public arti_nav_core::BasePathFollower
{
public:
  BasePathFollowerWrapper();

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum computeVelocityCommands(
    geometry_msgs::Twist& next_command) override;

  bool isGoalReached() override;

  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

private:
  static const char LOGGER_NAME[];

  void calculateCurrentIndex();

  static bool withinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value);
  void enforceLimit(
    geometry_msgs::Twist& next_command, double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value,
    const char* name);
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> plugin_loader_;
  boost::shared_ptr<nav_core::BaseLocalPlanner> planner_;

  ros::NodeHandle private_nh_;

  ros::Subscriber odom_sub_;

  std::mutex odometry_mutex_;
  geometry_msgs::Pose current_pose_;

  size_t current_index_{0};
  arti_nav_core_msgs::Trajectory2DWithLimits current_trajectory_;
};

}  // namespace arti_wrap_old_nav_core

#endif  // ARTI_WRAP_OLD_NAV_CORE_BASE_PATH_FOLLOWER_WRAPPER_H
