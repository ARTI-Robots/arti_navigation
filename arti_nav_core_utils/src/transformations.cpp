/*
Created by Alexander Buchegger on 2020-09-22.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_nav_core_utils/transformations.h>
#include <stdexcept>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define CHECK_FRAME_ID(frame_id, transform) checkFrameId(frame_id, transform, __func__)

namespace arti_nav_core_utils
{

static void checkFrameId(
  const std::string& frame_id, const geometry_msgs::TransformStamped& transform, const char* function_name)
{
  if (frame_id != transform.child_frame_id)
  {
    throw std::logic_error(std::string(function_name) + ": header.frame_id != transform.child_frame_id");
  }
}

arti_nav_core_msgs::Trajectory2DWithLimits transformTrajectory(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory, const geometry_msgs::TransformStamped& transform)
{
  CHECK_FRAME_ID(trajectory.header.frame_id, transform);

  tf2::Stamped<tf2::Transform> tf2_transform;
  tf2::convert(transform, tf2_transform);

  arti_nav_core_msgs::Trajectory2DWithLimits result;
  result.header = transform.header;

  result.movements.reserve(trajectory.movements.size());
  for (const auto& movement : trajectory.movements)
  {
    result.movements.emplace_back(transformMovement(movement, tf2_transform));
  }

  return result;
}

arti_nav_core_msgs::Path2DWithLimits transformPath(
  const arti_nav_core_msgs::Path2DWithLimits& path, const geometry_msgs::TransformStamped& transform)
{
  CHECK_FRAME_ID(path.header.frame_id, transform);
  tf2::Stamped<tf2::Transform> tf2_transform;
  tf2::convert(transform, tf2_transform);
  return transformPath(path, tf2_transform);
}

arti_nav_core_msgs::Path2DWithLimits transformPath(
  const arti_nav_core_msgs::Path2DWithLimits& path, const tf2::Stamped<tf2::Transform>& transform)
{
  arti_nav_core_msgs::Path2DWithLimits result;
  result.header.frame_id = transform.frame_id_;
  result.header.stamp = transform.stamp_;

  result.poses.reserve(path.poses.size());
  for (const auto& pose : path.poses)
  {
    result.poses.emplace_back(transformPose(pose, transform));
  }

  return result;
}

arti_nav_core_msgs::Movement2DGoalWithConstraints transformMovementGoal(
  const arti_nav_core_msgs::Movement2DGoalWithConstraints& movement_goal,
  const geometry_msgs::TransformStamped& goal_transform, const geometry_msgs::TransformStamped& path_limits_transform)
{
  arti_nav_core_msgs::Movement2DGoalWithConstraints result;
  result.goal = transformMovement(movement_goal.goal, goal_transform);
  result.path_limits = transformPath(movement_goal.path_limits, path_limits_transform);
  return result;
}

arti_nav_core_msgs::Movement2DStampedWithLimits transformMovement(
  const arti_nav_core_msgs::Movement2DStampedWithLimits& movement, const geometry_msgs::TransformStamped& transform)
{
  CHECK_FRAME_ID(movement.pose.header.frame_id, transform);
  arti_nav_core_msgs::Movement2DStampedWithLimits result;
  result.pose = transformPose(movement.pose, transform);
  // As the twist is considered relative to the pose, it doesn't change when the movement is transformed:
  result.twist = movement.twist;
  return result;
}

arti_nav_core_msgs::Movement2DWithLimits transformMovement(
  const arti_nav_core_msgs::Movement2DWithLimits& movement, const geometry_msgs::TransformStamped& transform)
{
  tf2::Stamped<tf2::Transform> tf2_transform;
  tf2::convert(transform, tf2_transform);
  return transformMovement(movement, tf2_transform);
}

arti_nav_core_msgs::Movement2DWithLimits transformMovement(
  const arti_nav_core_msgs::Movement2DWithLimits& movement, const tf2::Stamped<tf2::Transform>& transform)
{
  arti_nav_core_msgs::Movement2DWithLimits result;
  result.pose = transformPose(movement.pose, transform);
  // As the twist is considered relative to the pose, it doesn't change when the movement is transformed:
  result.twist = movement.twist;
  return result;
}

arti_nav_core_msgs::Pose2DStampedWithLimits transformPose(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose, const geometry_msgs::TransformStamped& transform)
{
  CHECK_FRAME_ID(pose.header.frame_id, transform);
  arti_nav_core_msgs::Pose2DStampedWithLimits result;
  result.header = transform.header;
  result.pose = transformPose(pose.pose, transform);
  return result;
}

arti_nav_core_msgs::Pose2DWithLimits transformPose(
  const arti_nav_core_msgs::Pose2DWithLimits& pose, const geometry_msgs::TransformStamped& transform)
{
  tf2::Stamped<tf2::Transform> tf2_transform;
  tf2::convert(transform, tf2_transform);
  return transformPose(pose, tf2_transform);
}

arti_nav_core_msgs::Pose2DWithLimits transformPose(
  const arti_nav_core_msgs::Pose2DWithLimits& pose, const tf2::Stamped<tf2::Transform>& transform)
{
  arti_nav_core_msgs::Pose2DWithLimits result;
  const tf2::Vector3 position_vec(pose.point.x.value, pose.point.y.value, 0.0);
  const tf2::Vector3 result_position = transform * position_vec;
  result.point.x.value = result_position.x();
  result.point.y.value = result_position.y();

  result.point.x.has_limits = pose.point.x.has_limits;
  result.point.y.has_limits = pose.point.y.has_limits;
  if (pose.point.x.has_limits || pose.point.y.has_limits)
  {
    const tf2::Vector3 position_lower_lower(pose.point.x.has_limits ? pose.point.x.lower_limit : 0.0,
                                            pose.point.y.has_limits ? pose.point.y.lower_limit : 0.0,
                                            0.0);

    const tf2::Vector3 result_position_lower_lower = transform.getBasis() * position_lower_lower;

    const tf2::Vector3 position_lower_upper(pose.point.x.has_limits ? pose.point.x.lower_limit : 0.0,
                                            pose.point.y.has_limits ? pose.point.y.upper_limit : 0.0,
                                            0.0);

    const tf2::Vector3 result_position_lower_upper = transform.getBasis() * position_lower_upper;

    const tf2::Vector3 position_upper_lower(pose.point.x.has_limits ? pose.point.x.upper_limit : 0.0,
                                            pose.point.y.has_limits ? pose.point.y.lower_limit : 0.0,
                                            0.0);

    const tf2::Vector3 result_position_upper_lower = transform.getBasis() * position_upper_lower;

    const tf2::Vector3 position_upper_upper(pose.point.x.has_limits ? pose.point.x.upper_limit : 0.0,
                                            pose.point.y.has_limits ? pose.point.y.upper_limit : 0.0,
                                            0.0);

    const tf2::Vector3 result_position_upper_upper = transform.getBasis() * position_upper_upper;

    if (pose.point.x.has_limits)
    {
      result.point.x.lower_limit = std::min(std::min(result_position_lower_lower.x(), result_position_lower_upper.x()),
                                            std::min(result_position_upper_lower.x(), result_position_upper_upper.x()));
      result.point.x.upper_limit = std::max(std::max(result_position_lower_lower.x(), result_position_lower_upper.x()),
                                            std::max(result_position_upper_lower.x(), result_position_upper_upper.x()));
    }
    if (pose.point.y.has_limits)
    {
      result.point.y.lower_limit = std::min(std::min(result_position_lower_lower.y(), result_position_lower_upper.y()),
                                            std::min(result_position_upper_lower.y(), position_upper_upper.y()));
      result.point.y.upper_limit = std::max(std::max(result_position_lower_lower.y(), result_position_lower_upper.y()),
                                            std::max(result_position_upper_lower.y(), result_position_upper_upper.y()));
    }
  }

  double rot_yaw, _pitch, _roll;
  transform.getBasis().getEulerYPR(rot_yaw, _pitch, _roll);

  result.theta.value = tf2NormalizeAngle(pose.theta.value + rot_yaw);
  result.theta.has_limits = pose.theta.has_limits;
  // Note: because limits are relative to value, we don't need to transform them:
  result.theta.lower_limit = pose.theta.lower_limit;
  result.theta.upper_limit = pose.theta.upper_limit;

  return result;
}

arti_nav_core_msgs::Twist2DWithLimits transformTwist(
  const arti_nav_core_msgs::Twist2DWithLimits& twist, const geometry_msgs::TransformStamped& transform)
{
  tf2::Stamped<tf2::Transform> tf2_transform;
  tf2::convert(transform, tf2_transform);
  return transformTwist(twist, tf2_transform);
}

arti_nav_core_msgs::Twist2DWithLimits transformTwist(
  const arti_nav_core_msgs::Twist2DWithLimits& twist, const tf2::Stamped<tf2::Transform>& transform)
{
  arti_nav_core_msgs::Twist2DWithLimits result;
  const tf2::Vector3 linear_velocity_transformed = transform * tf2::Vector3(twist.x.value, twist.y.value, 0.0);
  result.x.value = linear_velocity_transformed.x();
  result.y.value = linear_velocity_transformed.y();

  result.x.has_limits = twist.x.has_limits;
  result.y.has_limits = twist.y.has_limits;
  if (twist.x.has_limits || twist.y.has_limits)
  {
    const tf2::Vector3 position_lower_vec(twist.x.has_limits ? twist.x.lower_limit : 0.0,
                                          twist.y.has_limits ? twist.y.lower_limit : 0.0,
                                          0.0);
    const tf2::Vector3 result_position_lower = transform.getBasis() * position_lower_vec;

    const tf2::Vector3 position_upper_vec(twist.x.has_limits ? twist.x.upper_limit : 0.0,
                                          twist.y.has_limits ? twist.y.upper_limit : 0.0,
                                          0.0);
    const tf2::Vector3 result_position_upper = transform.getBasis() * position_upper_vec;

    if (twist.x.has_limits)
    {
      result.x.lower_limit = result_position_lower.x();
      result.x.upper_limit = result_position_upper.x();
    }
    if (twist.y.has_limits)
    {
      result.y.lower_limit = result_position_lower.y();
      result.y.upper_limit = result_position_upper.y();
    }
  }

  // Angular velocity doesn't change when transformed:
  result.theta = twist.theta;

  return result;
}

}
