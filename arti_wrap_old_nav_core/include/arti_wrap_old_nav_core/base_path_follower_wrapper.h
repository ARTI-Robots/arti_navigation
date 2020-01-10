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
#include <string>
#include <vector>

#ifdef __GNUC__
#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <pluginlib/class_loader.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>

#pragma GCC diagnostic pop
#endif

#include <mutex>

namespace arti_wrap_old_nav_core
{
class BasePathFollowerWrapper : public arti_nav_core::BasePathFollower
{
public:
  BasePathFollowerWrapper();

  bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum computeVelocityCommands(
    geometry_msgs::Twist& next_command) override;

  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool isGoalReached() override;

  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

private:
  void calcualteCurrentIndex();

  bool withinTolerance(double current_value, double goal_value, double uper_bound, double lower_bound,
                       bool is_angular = false);

  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> plugin_loader_;
  boost::shared_ptr<nav_core::BaseLocalPlanner> planner_;

  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;

  std::mutex odometry_mutex_;
  geometry_msgs::Pose current_position_;

  size_t current_index_;
  arti_nav_core_msgs::Trajectory2DWithLimits current_trajectory_;
};
}  // namespace arti_wrap_old_nav_core

#endif  // ARTI_WRAP_OLD_NAV_CORE_BASE_PATH_FOLLOWER_WRAPPER_H
