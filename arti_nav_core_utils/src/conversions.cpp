/*
Created by Alexander Buchegger on 2020-12-08.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_nav_core_utils/conversions.h>
#include <cmath>
#include <ros/console.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf/transform_datatypes.h>

#define CHECK_FOR_NON_FINITE_VALUE(var, nfvs) arti_nav_core_utils::non_finite_values::check(var, #var, nfvs)

namespace arti_nav_core_utils
{
namespace non_finite_values
{
const Strategy THROW{[](const double value, const char* const name) -> double {
  std::ostringstream s;
  s << "failed to convert " << name << " because it is " << value;
  throw std::invalid_argument(s.str());
  return value;
}};

const Strategy PASS_THROUGH{[](const double value, const char* const name) -> double {
  ROS_WARN_NAMED("conversions", "conversions: %s is %f; passed through", name, value);
  return value;
}};

const Strategy REPLACE_BY_0{[](const double value, const char* const name) -> double {
  ROS_WARN_NAMED("conversions", "conversions: %s is %f; replaced by 0", name, value);
  return 0;
}};

static inline double check(const double value, const char* const name, const Strategy& strategy)
{
  return std::isfinite(value) ? value : strategy(value, name);
}
}

nav_msgs::Odometry convertToOdometry(
  const arti_nav_core_msgs::Movement2DStampedWithLimits& movement, const non_finite_values::Strategy& nfvs)
{
  nav_msgs::Odometry result;
  result.header = movement.pose.header;
  result.pose.pose = convertToPose(movement.pose.pose, nfvs);
  result.twist.twist = convertToTwist(movement.twist, nfvs);
  return result;
}

nav_msgs::Path convertToPath(
  const arti_nav_core_msgs::Path2DWithLimits& path_2d_with_limits, const non_finite_values::Strategy& nfvs)
{
  nav_msgs::Path result;
  result.header = path_2d_with_limits.header;
  result.poses.reserve(path_2d_with_limits.poses.size());
  for (const auto& pose : path_2d_with_limits.poses)
  {
    result.poses.emplace_back(convertToPoseStamped(path_2d_with_limits.header, pose, nfvs));
  }
  return result;
}

nav_msgs::Path convertToPath(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory, const non_finite_values::Strategy& nfvs)
{
  nav_msgs::Path result;
  result.header = trajectory.header;
  result.poses.reserve(trajectory.movements.size());
  for (const auto& movement : trajectory.movements)
  {
    result.poses.emplace_back(convertToPoseStamped(trajectory.header, movement.pose, nfvs));
  }
  return result;
}

geometry_msgs::Point convertToPoint(
  const arti_nav_core_msgs::Point2DWithLimits& point_2d_with_limits, const non_finite_values::Strategy& nfvs)
{
  geometry_msgs::Point result;
  result.x = CHECK_FOR_NON_FINITE_VALUE(point_2d_with_limits.x.value, nfvs);
  result.y = CHECK_FOR_NON_FINITE_VALUE(point_2d_with_limits.y.value, nfvs);
  return result;
}

geometry_msgs::PointStamped convertToPointStamped(
  const std_msgs::Header& header, const arti_nav_core_msgs::Point2DWithLimits& point_2d_with_limits,
  const non_finite_values::Strategy& nfvs)
{
  geometry_msgs::PointStamped result;
  result.header = header;
  result.point = convertToPoint(point_2d_with_limits, nfvs);
  return result;
}

geometry_msgs::Pose convertToPose(
  const arti_nav_core_msgs::Pose2DWithLimits& pose_2d_with_limits, const non_finite_values::Strategy& nfvs)
{
  geometry_msgs::Pose result;
  result.position = convertToPoint(pose_2d_with_limits.point, nfvs);
  result.orientation
    = tf::createQuaternionMsgFromYaw(CHECK_FOR_NON_FINITE_VALUE(pose_2d_with_limits.theta.value, nfvs));
  return result;
}

geometry_msgs::Pose2D convertToPose2D(
  const arti_nav_core_msgs::Pose2DWithLimits& pose_2d_with_limits, const non_finite_values::Strategy& nfvs)
{
  geometry_msgs::Pose2D result;
  result.x = CHECK_FOR_NON_FINITE_VALUE(pose_2d_with_limits.point.x.value, nfvs);
  result.y = CHECK_FOR_NON_FINITE_VALUE(pose_2d_with_limits.point.y.value, nfvs);
  result.theta = CHECK_FOR_NON_FINITE_VALUE(pose_2d_with_limits.theta.value, nfvs);
  return result;
}

geometry_msgs::PoseStamped convertToPoseStamped(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose_2d_with_limits, const non_finite_values::Strategy& nfvs)
{
  return convertToPoseStamped(pose_2d_with_limits.header, pose_2d_with_limits.pose, nfvs);
}

geometry_msgs::PoseStamped convertToPoseStamped(
  const std_msgs::Header& header, const arti_nav_core_msgs::Pose2DWithLimits& pose_2d_with_limits,
  const non_finite_values::Strategy& nfvs)
{
  geometry_msgs::PoseStamped result;
  result.header = header;
  result.pose = convertToPose(pose_2d_with_limits, nfvs);
  return result;
}

geometry_msgs::Twist convertToTwist(
  const arti_nav_core_msgs::Twist2DWithLimits& twist_2d_with_limits, const non_finite_values::Strategy& nfvs)
{
  geometry_msgs::Twist result;
  result.linear.x = CHECK_FOR_NON_FINITE_VALUE(twist_2d_with_limits.x.value, nfvs);
  result.linear.y = CHECK_FOR_NON_FINITE_VALUE(twist_2d_with_limits.y.value, nfvs);
  result.angular.z = CHECK_FOR_NON_FINITE_VALUE(twist_2d_with_limits.theta.value, nfvs);
  return result;
}

}
