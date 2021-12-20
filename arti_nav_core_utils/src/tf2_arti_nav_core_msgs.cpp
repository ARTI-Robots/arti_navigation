/*
Created by Alexander Buchegger on 2020-12-08.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_nav_core_utils/tf2_arti_nav_core_msgs.h>
#include <arti_nav_core_utils/transformations.h>

namespace tf2
{

template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Movement2DStampedWithLimits& movement)
{
  return movement.pose.header.stamp;
}

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Movement2DStampedWithLimits& movement)
{
  return movement.pose.header.frame_id;
}

template<>
void doTransform(
  const arti_nav_core_msgs::Movement2DStampedWithLimits& movement_in,
  arti_nav_core_msgs::Movement2DStampedWithLimits& movement_out, const geometry_msgs::TransformStamped& transform)
{
  movement_out = arti_nav_core_utils::transformMovement(movement_in, transform);
}


template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Path2DWithLimits& path)
{
  return path.header.stamp;
}

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Path2DWithLimits& path)
{
  return path.header.frame_id;
}

template<>
void doTransform(
  const arti_nav_core_msgs::Path2DWithLimits& path_in, arti_nav_core_msgs::Path2DWithLimits& path_out,
  const geometry_msgs::TransformStamped& transform)
{
  path_out = arti_nav_core_utils::transformPath(path_in, transform);
}


template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Pose2DStampedWithLimits& pose)
{
  return pose.header.stamp;
}

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Pose2DStampedWithLimits& pose)
{
  return pose.header.frame_id;
}

template<>
void doTransform(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose_in, arti_nav_core_msgs::Pose2DStampedWithLimits& pose_out,
  const geometry_msgs::TransformStamped& transform)
{
  pose_out = arti_nav_core_utils::transformPose(pose_in, transform);
}


template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  return trajectory.header.stamp;
}

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  return trajectory.header.frame_id;
}

template<>
void doTransform(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory_in,
  arti_nav_core_msgs::Trajectory2DWithLimits& trajectory_out, const geometry_msgs::TransformStamped& transform)
{
  trajectory_out = arti_nav_core_utils::transformTrajectory(trajectory_in, transform);
}

}
