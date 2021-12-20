/*
Created by clemens on 03.05.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PURE_PURSUIT_ROBOT_INFORMATION_H
#define ARTI_PURE_PURSUIT_ROBOT_INFORMATION_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>

namespace arti_pass_through_planner
{
class RobotInformation
{
public:
  explicit RobotInformation(const ros::NodeHandle& nh);

  geometry_msgs::PoseStamped getRobotPose();
  geometry_msgs::Twist getTwist();

  void lock();
  void unlock();

  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

private:
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;

  std::mutex odometry_mutex_;
  geometry_msgs::PoseStamped current_position_;
  geometry_msgs::Twist current_twist_;
};
}

#endif //ARTI_PURE_PURSUIT_ROBOT_INFORMATION_H
