/*
Created by clemens on 24.06.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_path_follower_abstraction/path_follower_command_velocities.h>

namespace arti_path_follower_abstraction
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
PathFollowerCommandVelocities::PathFollowerCommandVelocities(
  const ros::NodeHandle& nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
  costmap_2d::Costmap2DROS* costmap_ros)
  : PathFollowerAbstractionImpl(nh, plugin_name, tf_listener, costmap_ros, "arti_nav_core::BasePathFollower")
{
  min_translation_vel_ = nh_.param<double>("min_translation_vel", 0.05);
  min_rotation_vel_ = nh_.param<double>("min_rotation_vel", 0.1);
}
#else // if KINETIC (v12)
PathFollowerCommandVelocities::PathFollowerCommandVelocities(
  const ros::NodeHandle& nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
  costmap_2d::Costmap2DROS* costmap_ros)
  : PathFollowerAbstractionImpl(nh, plugin_name, tf_listener, costmap_ros, "arti_nav_core::BasePathFollower")
{
  min_translation_vel_ = nh_.param<double>("min_translation_vel", 0.05);
  min_rotation_vel_ = nh_.param<double>("min_rotation_vel", 0.1);
}
#endif

void PathFollowerCommandVelocities::sendStopCommand()
{
  geometry_msgs::Twist command;
  publishCommandDirect(command);
}

PathFollowerAbstraction::PathFollowerAbstractionErrorEnum PathFollowerCommandVelocities::convert(
  const arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum& error)
{
  switch (error)
  {
    case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::GOAL_REACHED:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::GOAL_REACHED;
    case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::COMMAND_FOUND:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::COMMAND_FOUND;
    case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::OBSTACLE_CLOSE:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::OBSTACLE_CLOSE;
    case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::OBSTACLE_IN_FRONT:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::OBSTACLE_IN_FRONT;
    case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::FAR_FROM_PATH:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::FAR_FROM_PATH;
    case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::NO_COMMAND_POSSIBLE;
  }
  ROS_FATAL_STREAM("bug: unknown path follower error");
  return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::NO_COMMAND_POSSIBLE;
}

bool PathFollowerCommandVelocities::nonZeroCommand(const geometry_msgs::Twist& next_command)
{
  if ((std::abs(next_command.linear.x) >= min_translation_vel_)
      || (std::abs(next_command.linear.y) >= min_translation_vel_)
      || (std::abs(next_command.angular.z) >= min_rotation_vel_))
  {
    return true;
  }

  return false;
}
}
