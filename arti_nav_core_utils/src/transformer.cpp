/*
Created by Alexander Buchegger on 2020-12-28.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_nav_core_utils/transformer.h>
#include <ros/common.h>
#include <tf2_ros/transform_listener.h>

namespace arti_nav_core_utils
{
namespace detail
{

#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
struct TransformListener : public tf2_ros::Buffer
{
  TransformListener(const ros::Duration& cache_time, const bool spin_thread)
    : Buffer(cache_time), tf2_listener_(*this, spin_thread)
  {
  }

  TransformListener(const ros::NodeHandle& node_handle, const ros::Duration& cache_time, const bool spin_thread)
    : Buffer(cache_time), tf2_listener_(*this, node_handle, spin_thread)
  {
  }

  tf2_ros::TransformListener tf2_listener_;
};
#else // if KINETIC (v12)
struct TransformListener : public tf::TransformListener
{
  // Trick to access protected member of tf::TransformListener:
  using tf::TransformListener::tf2_buffer_;
};
#endif

static void handleError(
  const std::string& target_frame, const std::string& source_frame, const char* const reason,
  std::string* const error_message)
{
  std::ostringstream ss;
  ss << "failed to lookup transform between '" << target_frame << "' and '" << source_frame << "': " << reason;
  std::string s = ss.str();
  ROS_DEBUG_STREAM(s);
  if (error_message != nullptr)
  {
    *error_message = std::move(s);
  }
}

}

std::shared_ptr<arti_nav_core::Transformer> createTransformer(
  const boost::optional<ros::Duration>& max_cache_time, const bool spin_thread)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  return std::make_shared<detail::TransformListener>(
    max_cache_time.get_value_or(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)), spin_thread);
#else // if KINETIC (v12)
  return std::make_shared<tf::TransformListener>(
    max_cache_time.get_value_or(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME)), spin_thread);
#endif
}

std::shared_ptr<arti_nav_core::Transformer> createTransformer(
  const ros::NodeHandle& node_handle, const boost::optional<ros::Duration>& max_cache_time, const bool spin_thread)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  return std::make_shared<detail::TransformListener>(
    node_handle, max_cache_time.get_value_or(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)), spin_thread);
#else // if KINETIC (v12)
  return std::make_shared<tf::TransformListener>(
    node_handle, max_cache_time.get_value_or(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME)), spin_thread);
#endif
}

tf2_ros::Buffer& getBuffer(tf::TransformListener& transformer)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  return *transformer.getTF2BufferPtr();
#else // if KINETIC (v12)
  return static_cast<detail::TransformListener&>(transformer).tf2_buffer_;
#endif
}

const tf2_ros::Buffer& getBuffer(const tf::TransformListener& transformer)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  return *const_cast<tf::TransformListener&>(transformer).getTF2BufferPtr();
#else // if KINETIC (v12)
  return static_cast<const detail::TransformListener&>(transformer).tf2_buffer_;
#endif
}

tf2_ros::Buffer& getBuffer(tf2_ros::Buffer& transformer)
{
  return transformer;
}

const tf2_ros::Buffer& getBuffer(const tf2_ros::Buffer& transformer)
{
  return transformer;
}

boost::optional<geometry_msgs::TransformStamped> tryToLookupTransform(
  const tf::TransformListener& transformer, const std::string& target_frame, const std::string& source_frame,
  const ros::Time& time, const ros::Duration& timeout, std::string* error_message)
{
  return tryToLookupTransform(getBuffer(transformer), target_frame, source_frame, time, timeout, error_message);
}

boost::optional<geometry_msgs::TransformStamped> tryToLookupTransform(
  const tf2_ros::BufferInterface& transformer, const std::string& target_frame, const std::string& source_frame,
  const ros::Time& time, const ros::Duration& timeout, std::string* error_message)
{
  try
  {
    return transformer.lookupTransform(target_frame, source_frame, time, timeout);
  }
  catch (const tf2::TransformException& ex)
  {
    detail::handleError(target_frame, source_frame, ex.what(), error_message);
  }
  return boost::none;
}

}
