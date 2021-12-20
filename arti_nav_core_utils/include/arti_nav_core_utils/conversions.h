/*
Created by Alexander Buchegger on 2020-12-08.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_NAV_CORE_UTILS_CONVERSIONS_H
#define ARTI_NAV_CORE_UTILS_CONVERSIONS_H

#include <arti_nav_core_msgs/Movement2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Path2DWithLimits.h>
#include <arti_nav_core_msgs/Point2DWithLimits.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Pose2DWithLimits.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <arti_nav_core_msgs/Twist2DWithLimits.h>
#include <functional>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

namespace arti_nav_core_utils
{
namespace non_finite_values
{
using Strategy = std::function<double(double, const char*)>;

extern const Strategy THROW;
extern const Strategy PASS_THROUGH;
extern const Strategy REPLACE_BY_0;
}

nav_msgs::Odometry convertToOdometry(
  const arti_nav_core_msgs::Movement2DStampedWithLimits& movement,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

nav_msgs::Path convertToPath(
  const arti_nav_core_msgs::Path2DWithLimits& path_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

nav_msgs::Path convertToPath(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::Point convertToPoint(
  const arti_nav_core_msgs::Point2DWithLimits& point_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::PointStamped convertToPointStamped(
  const std_msgs::Header& header, const arti_nav_core_msgs::Point2DWithLimits& point_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::Pose convertToPose(
  const arti_nav_core_msgs::Pose2DWithLimits& pose_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::Pose2D convertToPose2D(
  const arti_nav_core_msgs::Pose2DWithLimits& pose_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::PoseStamped convertToPoseStamped(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::PoseStamped convertToPoseStamped(
  const std_msgs::Header& header, const arti_nav_core_msgs::Pose2DWithLimits& pose_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

geometry_msgs::Twist convertToTwist(
  const arti_nav_core_msgs::Twist2DWithLimits& twist_2d_with_limits,
  const non_finite_values::Strategy& nfvs = non_finite_values::THROW);

}

#endif //ARTI_NAV_CORE_UTILS_CONVERSIONS_H
