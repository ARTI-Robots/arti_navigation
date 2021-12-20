/*
Created by clemens on 25.06.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_path_follower_abstraction/path_follower_command_ackermann.h>
#include <angles/angles.h>

namespace arti_path_follower_abstraction
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
PathFollowerCommandAckermann::PathFollowerCommandAckermann(
  const ros::NodeHandle& nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
  costmap_2d::Costmap2DROS* costmap_ros)
  : PathFollowerAbstractionImpl(nh, plugin_name, tf_listener, costmap_ros, "arti_nav_core::BasePathFollowerAckermann")
{
  min_translation_vel_ = nh_.param<double>("min_translation_vel", 0.05);
  min_angular_change_ = nh_.param<double>("min_angular_change", 0.01);

  exected_command_sub_ = nh_.subscribe("exected_command", 1, &PathFollowerCommandAckermann::executedCommandCB, this);
}
#else // if KINETIC (v12)
PathFollowerCommandAckermann::PathFollowerCommandAckermann(
  const ros::NodeHandle& nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
  costmap_2d::Costmap2DROS* costmap_ros)
  : PathFollowerAbstractionImpl(nh, plugin_name, tf_listener, costmap_ros, "arti_nav_core::BasePathFollowerAckermann")
{
  min_translation_vel_ = nh_.param<double>("min_translation_vel", 0.05);
  min_angular_change_ = nh_.param<double>("min_angular_change", 0.01);

  exected_command_sub_ = nh_.subscribe("exected_command", 1, &PathFollowerCommandAckermann::executedCommandCB, this);
}
#endif

void PathFollowerCommandAckermann::sendStopCommand()
{
  ackermann_msgs::AckermannDrive command;
  if (executed_command_)
  {
    command.steering_angle = executed_command_->steering_angle;
  }
  else if (last_send_command_)
  {
    command.steering_angle = last_send_command_->steering_angle;
  }

  publishCommandDirect(command);
}

PathFollowerAbstraction::PathFollowerAbstractionErrorEnum PathFollowerCommandAckermann::convert(
  const arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum& error)
{
  switch (error)
  {
    case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::GOAL_REACHED:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::GOAL_REACHED;
    case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::COMMAND_FOUND:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::COMMAND_FOUND;
    case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::OBSTACLE_CLOSE:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::OBSTACLE_CLOSE;
    case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::OBSTACLE_IN_FRONT:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::OBSTACLE_IN_FRONT;
    case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::FAR_FROM_PATH:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::FAR_FROM_PATH;
    case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::NO_COMMAND_POSSIBLE:
      return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::NO_COMMAND_POSSIBLE;
  }
  ROS_FATAL_STREAM("bug: unknown path follower error");
  return PathFollowerAbstraction::PathFollowerAbstractionErrorEnum::NO_COMMAND_POSSIBLE;
}

bool PathFollowerCommandAckermann::nonZeroCommand(const ackermann_msgs::AckermannDrive& next_command)
{
  if (next_command.speed > min_translation_vel_)
  {
    return true;
  }

  if (executed_command_)
  {
    if (std::abs(angles::shortest_angular_distance(executed_command_->steering_angle, next_command.steering_angle))
        > min_angular_change_)
    {
      return true;
    }
    if (std::abs(angles::shortest_angular_distance(next_command.steering_angle, executed_command_->steering_angle))
        > min_angular_change_)
    {
      return true;
    }
  }

  if (last_send_command_)
  {
    if (std::abs(angles::shortest_angular_distance(last_send_command_->steering_angle, next_command.steering_angle))
        > min_angular_change_)
    {
      return true;
    }
    if (std::abs(angles::shortest_angular_distance(next_command.steering_angle, last_send_command_->steering_angle))
        > min_angular_change_)
    {
      return true;
    }
  }

  ROS_DEBUG_STREAM("zero command: " << next_command);
  if (executed_command_)
  {
    ROS_DEBUG_STREAM("executed command: " << *executed_command_);
  }

  if (last_send_command_)
  {
    ROS_DEBUG_STREAM("last send command: " << *last_send_command_);
  }

  return false;
}

void PathFollowerCommandAckermann::executedCommandCB(const ackermann_msgs::AckermannDrive& next_command)
{
  executed_command_ = next_command;
}

}
