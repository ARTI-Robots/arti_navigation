/*
Created by Alexander Buchegger on 2020-12-08.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_NAV_CORE_UTILS_TF2_ARTI_NAV_CORE_MSGS_H
#define ARTI_NAV_CORE_UTILS_TF2_ARTI_NAV_CORE_MSGS_H

#include <arti_nav_core_msgs/Movement2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Path2DWithLimits.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <tf2/convert.h>

/**
 * \file
 * This file contains specializations of tf2 function templates for arti_nav_core_msgs so that these messages can be
 * transformed easily using a tf2::BufferCore.
 *
 * Note that Movement2DGoalWithConstraints cannot be transformed in this way because its parts contain multiple time
 * stamps and frame IDs. Also, only messages with time stamps and frame IDs can be transformed.
 */

namespace tf2
{

template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Movement2DStampedWithLimits& movement);

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Movement2DStampedWithLimits& movement);

template<>
void doTransform(
  const arti_nav_core_msgs::Movement2DStampedWithLimits& movement_in,
  arti_nav_core_msgs::Movement2DStampedWithLimits& movement_out, const geometry_msgs::TransformStamped& transform);


template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Path2DWithLimits& path);

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Path2DWithLimits& path);

template<>
void doTransform(
  const arti_nav_core_msgs::Path2DWithLimits& path_in, arti_nav_core_msgs::Path2DWithLimits& path_out,
  const geometry_msgs::TransformStamped& transform);


template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Pose2DStampedWithLimits& pose);

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Pose2DStampedWithLimits& pose);

template<>
void doTransform(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose_in, arti_nav_core_msgs::Pose2DStampedWithLimits& pose_out,
  const geometry_msgs::TransformStamped& transform);


template<>
const ros::Time& getTimestamp(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory);

template<>
const std::string& getFrameId(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory);

template<>
void doTransform(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory_in,
  arti_nav_core_msgs::Trajectory2DWithLimits& trajectory_out, const geometry_msgs::TransformStamped& transform);

}

#endif //ARTI_NAV_CORE_UTILS_TF2_ARTI_NAV_CORE_MSGS_H
