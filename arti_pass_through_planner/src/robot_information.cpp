/*
Created by clemens on 03.05.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_pass_through_planner/robot_information.h>

namespace arti_pass_through_planner
{
RobotInformation::RobotInformation(const ros::NodeHandle& nh) : nh_(nh)
{
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&RobotInformation::odomCB, this, _1));
}

geometry_msgs::PoseStamped RobotInformation::getRobotPose()
{
  return current_position_;
}

geometry_msgs::Twist RobotInformation::getTwist()
{
  return current_twist_;
}

void RobotInformation::lock()
{
  odometry_mutex_.lock();
}

void RobotInformation::unlock()
{
  odometry_mutex_.unlock();
}

void RobotInformation::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> odom_guard(odometry_mutex_);

  current_twist_ = msg->twist.twist;
  current_position_.header = msg->header;
  current_position_.pose = msg->pose.pose;
}
}
