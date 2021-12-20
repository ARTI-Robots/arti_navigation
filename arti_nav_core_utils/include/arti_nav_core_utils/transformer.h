/*
Created by Alexander Buchegger on 2020-12-28.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_NAV_CORE_UTILS_TRANSFORMER_H
#define ARTI_NAV_CORE_UTILS_TRANSFORMER_H

#include <arti_nav_core/transformer.h>
#include <boost/optional.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace arti_nav_core_utils
{

std::shared_ptr<arti_nav_core::Transformer> createTransformer(
  const boost::optional<ros::Duration>& max_cache_time = boost::none, bool spin_thread = true);

std::shared_ptr<arti_nav_core::Transformer> createTransformer(
  const ros::NodeHandle& node_handle, const boost::optional<ros::Duration>& max_cache_time = boost::none,
  bool spin_thread = true);

tf2_ros::Buffer& getBuffer(tf::TransformListener& transformer);
const tf2_ros::Buffer& getBuffer(const tf::TransformListener& transformer);

// Trivial implementations for consistent interface for different versions of arti_nav_core::Transformer:
tf2_ros::Buffer& getBuffer(tf2_ros::Buffer& transformer);
const tf2_ros::Buffer& getBuffer(const tf2_ros::Buffer& transformer);

boost::optional<geometry_msgs::TransformStamped> tryToLookupTransform(
  const tf::TransformListener& transformer, const std::string& target_frame, const std::string& source_frame,
  const ros::Time& time, const ros::Duration& timeout, std::string* error_message = nullptr);

boost::optional<geometry_msgs::TransformStamped> tryToLookupTransform(
  const tf2_ros::BufferInterface& transformer, const std::string& target_frame, const std::string& source_frame,
  const ros::Time& time, const ros::Duration& timeout, std::string* error_message = nullptr);

template<typename T, typename Transformer>
boost::optional<T> tryToTransform(
  const Transformer& transformer, const T& in, const std::string& target_frame, const ros::Duration& timeout,
  std::string* error_message = nullptr)
{
  const boost::optional<geometry_msgs::TransformStamped> transform
    = tryToLookupTransform(transformer, target_frame, tf2::getFrameId(in), tf2::getTimestamp(in), timeout,
                           error_message);
  if (transform)
  {
    T out;
    tf2::doTransform(in, out, *transform);
    return out;
  }
  return boost::none;
}

}

#endif //ARTI_NAV_CORE_UTILS_TRANSFORMER_H
