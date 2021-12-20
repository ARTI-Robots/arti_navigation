/*
Created by Alexander Buchegger on 2020-09-22.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_NAV_CORE_UTILS_TRANSFORMATIONS_H
#define ARTI_NAV_CORE_UTILS_TRANSFORMATIONS_H

#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <arti_nav_core_msgs/Movement2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Movement2DWithLimits.h>
#include <arti_nav_core_msgs/Path2DWithLimits.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Pose2DWithLimits.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <arti_nav_core_msgs/Twist2DWithLimits.h>
#include <arti_nav_core_msgs/ValueWithLimits.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

namespace arti_nav_core_utils
{

arti_nav_core_msgs::Trajectory2DWithLimits transformTrajectory(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Path2DWithLimits transformPath(
  const arti_nav_core_msgs::Path2DWithLimits& path, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Path2DWithLimits transformPath(
  const arti_nav_core_msgs::Path2DWithLimits& path, const tf2::Stamped<tf2::Transform>& transform);

arti_nav_core_msgs::Movement2DGoalWithConstraints transformMovementGoal(
  const arti_nav_core_msgs::Movement2DGoalWithConstraints& movement_goal,
  const geometry_msgs::TransformStamped& goal_transform, const geometry_msgs::TransformStamped& path_limits_transform);

arti_nav_core_msgs::Movement2DStampedWithLimits transformMovement(
  const arti_nav_core_msgs::Movement2DStampedWithLimits& movement, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Movement2DWithLimits transformMovement(
  const arti_nav_core_msgs::Movement2DWithLimits& movement, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Movement2DWithLimits transformMovement(
  const arti_nav_core_msgs::Movement2DWithLimits& movement, const tf2::Stamped<tf2::Transform>& transform);

arti_nav_core_msgs::Pose2DStampedWithLimits transformPose(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Pose2DWithLimits transformPose(
  const arti_nav_core_msgs::Pose2DWithLimits& pose, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Pose2DWithLimits transformPose(
  const arti_nav_core_msgs::Pose2DWithLimits& pose, const tf2::Stamped<tf2::Transform>& transform);

arti_nav_core_msgs::Twist2DWithLimits transformTwist(
  const arti_nav_core_msgs::Twist2DWithLimits& twist, const geometry_msgs::TransformStamped& transform);

arti_nav_core_msgs::Twist2DWithLimits transformTwist(
  const arti_nav_core_msgs::Twist2DWithLimits& twist, const tf2::Stamped<tf2::Transform>& transform);

}

#endif //ARTI_NAV_CORE_UTILS_TRANSFORMATIONS_H
