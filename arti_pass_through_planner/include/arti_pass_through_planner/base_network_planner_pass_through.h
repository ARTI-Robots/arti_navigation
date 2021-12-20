/*
Created by abuchegger on 2021-01-23.
This file is part of the software provided by ARTI
Copyright (c) 2021, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PASS_THROUGH_PLANNER_BASE_NETWORK_PLANNER_PASS_THROUGH_H
#define ARTI_PASS_THROUGH_PLANNER_BASE_NETWORK_PLANNER_PASS_THROUGH_H

#include <arti_nav_core/base_network_planner.h>
#include <boost/optional.hpp>

namespace arti_pass_through_planner
{

class BaseNetworkPlannerPassThrough : public arti_nav_core::BaseNetworkPlanner
{
public:
  BaseNetworkPlannerPassThrough() = default;

  void initialize(std::string name, arti_nav_core::Transformer* transformer) override;

  bool setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) override;

  BaseNetworkPlannerErrorEnum makePlan(arti_nav_core_msgs::Path2DWithLimits& path) override;

  void handlePlannerError(
    const arti_nav_core_msgs::Pose2DWithLimits& error_pose_a,
    const arti_nav_core_msgs::Pose2DWithLimits& error_pose_b) override;

protected:
  boost::optional<arti_nav_core_msgs::Path2DWithLimits> path_;
};

}

#endif //ARTI_PASS_THROUGH_PLANNER_BASE_NETWORK_PLANNER_PASS_THROUGH_H
