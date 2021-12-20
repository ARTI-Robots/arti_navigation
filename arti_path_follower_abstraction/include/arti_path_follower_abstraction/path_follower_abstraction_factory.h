/*
Created by clemens on 25.06.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_ABSTRACTION_FACTORY_H
#define ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_ABSTRACTION_FACTORY_H

#include <memory>
#include <arti_path_follower_abstraction/path_follower_abstraction.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <costmap_2d/costmap_2d_ros.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_path_follower_abstraction
{
class PathFollowerAbstractionFactory
{
public:
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  static std::shared_ptr<PathFollowerAbstraction> createPathFollower(
    const ros::NodeHandle& nh, const std::string& plugin_name, tf2_ros::Buffer* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros, bool ackermann);
#else // if KINETIC (v12)
  static std::shared_ptr<PathFollowerAbstraction> createPathFollower(
    const ros::NodeHandle& nh, const std::string& plugin_name, tf::TransformListener* tf_listener,
    costmap_2d::Costmap2DROS* costmap_ros, bool ackermann);
#endif
};
}


#endif //ARTI_PATH_FOLLOWER_ABSTRACTION_PATH_FOLLOWER_ABSTRACTION_FACTORY_H
