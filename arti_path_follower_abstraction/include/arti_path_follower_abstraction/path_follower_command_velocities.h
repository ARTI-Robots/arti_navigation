/*
Created by clemens on 24.06.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_COMMAND_VELOCITIES_H
#define ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_COMMAND_VELOCITIES_H

#include <arti_path_follower_abstraction/path_follower_abstraction_impl.h>
#include <arti_nav_core/base_path_follower.h>
#include <geometry_msgs/Twist.h>

namespace arti_path_follower_abstraction
{
class PathFollowerCommandVelocities
  : public PathFollowerAbstractionImpl<geometry_msgs::Twist, arti_nav_core::BasePathFollower, arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum>
{
public:
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  PathFollowerCommandVelocities(
    const ros::NodeHandle& nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros);
#else // if KINETIC (v12)
  PathFollowerCommandVelocities(
    const ros::NodeHandle& nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros);
#endif
  void sendStopCommand() override;
protected:
  PathFollowerAbstractionErrorEnum convert(
    const arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum& error) override;
  bool nonZeroCommand(const geometry_msgs::Twist& next_command) override;

  double min_translation_vel_;
  double min_rotation_vel_;
};
}

#endif //ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_COMMAND_VELOCITIES_H
