/*
Created by clemens on 24.06.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_ABSTRACTION_IMPL_H
#define ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_ABSTRACTION_IMPL_H

#include <arti_path_follower_abstraction/path_follower_abstraction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <costmap_2d/costmap_2d_ros.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_path_follower_abstraction
{
template<class O, class P, class E>
class PathFollowerAbstractionImpl : public PathFollowerAbstraction
{
public:
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  PathFollowerAbstractionImpl(
    const ros::NodeHandle &nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros, const std::string& base_class_name);
#else // if KINETIC (v12)
  PathFollowerAbstractionImpl(
    const ros::NodeHandle &nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros, const std::string& base_class_name);

#endif

  bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  PathFollowerAbstractionErrorEnum computeCommand() override;
  bool nonZeroCommandTimedOut() override;
  void resetZeroCommandTime() override;
  bool isGoalReached() override;

protected:
  PathFollowerAbstractionErrorEnum computeCommand(O& next_command);
  virtual PathFollowerAbstractionErrorEnum convert(const E& error) = 0;
  virtual bool nonZeroCommand(const O& next_command) = 0;
  void publishCommand(const O& command);
  void publishCommandDirect(const O& command);

  ros::NodeHandle nh_;
  boost::optional<O> last_send_command_;

private:
  ros::Publisher cmd_vel_pub_;

  pluginlib::ClassLoader<P> plugin_loader_;
  boost::shared_ptr<P> planner_;

  ros::Time last_non_zero_command_;
  ros::Duration max_zero_command_duration_;
};

template<class O, class P, class E>
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
PathFollowerAbstractionImpl<O, P, E>::PathFollowerAbstractionImpl(
  const ros::NodeHandle &nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
  costmap_2d::Costmap2DROS* costmap_ros, const std::string& base_class_name)
  : nh_(nh), plugin_loader_("arti_nav_core", base_class_name)
#else // if KINETIC (v12)
PathFollowerAbstractionImpl<O, P, E>::PathFollowerAbstractionImpl(
  const ros::NodeHandle &nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
  costmap_2d::Costmap2DROS* costmap_ros, const std::string& base_class_name)
  : nh_(nh), plugin_loader_("arti_nav_core", base_class_name)
#endif
{
  try
  {
    planner_ = plugin_loader_.createInstance(plugin_name);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Failed to load the " << plugin_name << " path follower: " << ex.what());
    throw;
  }

  planner_->initialize(plugin_loader_.getName(plugin_name), tf_listener, costmap_ros);

  cmd_vel_pub_ = nh_.advertise<O>("cmd_vel", 1);
  max_zero_command_duration_ = ros::Duration(nh_.param<double>("max_zero_command_duration", 1.0));
}

template<class O, class P, class E>
bool PathFollowerAbstractionImpl<O, P, E>::setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  return planner_->setTrajectory(trajectory);
}

template<class O, class P, class E>
PathFollowerAbstraction::PathFollowerAbstractionErrorEnum PathFollowerAbstractionImpl<O, P, E>::computeCommand()
{
  O next_command;
  PathFollowerAbstractionErrorEnum result = computeCommand(next_command);
  if (result == PathFollowerAbstractionErrorEnum::COMMAND_FOUND)
  {
    ROS_DEBUG_STREAM("found command publich command: " << next_command);
    publishCommand(next_command);
  }
  else
  {
    ROS_DEBUG_STREAM("no command found");
  }

  return result;
}

template<class O, class P, class E>
bool PathFollowerAbstractionImpl<O, P, E>::nonZeroCommandTimedOut()
{
  ROS_DEBUG_STREAM("last_non_zero_command: " << last_non_zero_command_);
  ROS_DEBUG_STREAM("max_zero_command_duration_: " << max_zero_command_duration_);
  ROS_DEBUG_STREAM("(ros::Time::now() - last_non_zero_command_): " << (ros::Time::now() - last_non_zero_command_));
  return (!last_non_zero_command_.isZero()
          && ((ros::Time::now() - last_non_zero_command_) > max_zero_command_duration_));
}

template<class O, class P, class E>
void PathFollowerAbstractionImpl<O, P, E>::resetZeroCommandTime()
{
  last_non_zero_command_ = ros::Time::now();
  ROS_DEBUG_STREAM("resetZeroCommandTime-> last_non_zero_command: " << last_non_zero_command_);
}

template<class O, class P, class E>
bool PathFollowerAbstractionImpl<O, P, E>::isGoalReached()
{
  return planner_->isGoalReached();
}

template<class O, class P, class E>
typename PathFollowerAbstraction::PathFollowerAbstractionErrorEnum PathFollowerAbstractionImpl<O, P, E>::computeCommand(
  O& next_command)
{
  E error = planner_->computeVelocityCommands(next_command);

  return convert(error);
}

template<class O, class P, class E>
void PathFollowerAbstractionImpl<O, P, E>::publishCommand(const O& command)
{
  if (nonZeroCommand(command))
  {
    last_non_zero_command_ = ros::Time::now();
  }

  publishCommandDirect(command);
}

template<class O, class P, class E>
void PathFollowerAbstractionImpl<O, P, E>::publishCommandDirect(const O& command)
{
  cmd_vel_pub_.publish(command);

  last_send_command_ = command;
}


}

#endif //ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_ABSTRACTION_IMPL_H
