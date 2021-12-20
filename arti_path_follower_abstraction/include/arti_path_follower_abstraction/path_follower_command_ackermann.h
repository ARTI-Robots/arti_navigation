/*
Created by clemens on 25.06.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_COMMAND_ACKERMANN_H
#define ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_COMMAND_ACKERMANN_H

#include <arti_path_follower_abstraction/path_follower_abstraction_impl.h>
#include <arti_nav_core/base_path_follower_ackermann.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace arti_path_follower_abstraction
{
class PathFollowerCommandAckermann
  : public PathFollowerAbstractionImpl<ackermann_msgs::AckermannDrive, arti_nav_core::BasePathFollowerAckermann, arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum>
{
public:
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  PathFollowerCommandAckermann(
    const ros::NodeHandle& nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros);
#else // if KINETIC (v12)
  PathFollowerCommandAckermann(
    const ros::NodeHandle& nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros);
#endif
  void sendStopCommand() override;
protected:
  PathFollowerAbstractionErrorEnum convert(
    const arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum& error) override;
  bool nonZeroCommand(const ackermann_msgs::AckermannDrive& next_command) override;
  void executedCommandCB(const ackermann_msgs::AckermannDrive& next_command);

  double min_translation_vel_;
  double min_angular_change_;
  boost::optional<ackermann_msgs::AckermannDrive> executed_command_;

  ros::Subscriber exected_command_sub_;
};
}


#endif //ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_COMMAND_ACKERMANN_H
