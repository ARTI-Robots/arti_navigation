/*
Created by clemens on 11.10.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PASS_THROUGH_PLANNER_BASE_GLOBAL_PLANNER_PASS_THROUGH_H
#define ARTI_PASS_THROUGH_PLANNER_BASE_GLOBAL_PLANNER_PASS_THROUGH_H

#include <arti_nav_core/base_global_planner.h>

namespace arti_pass_through_planner
{

class BaseGlobalPlannerPassThrough : public arti_nav_core::BaseGlobalPlanner
{
public:
  BaseGlobalPlannerPassThrough() = default;

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setGoal(
    const arti_nav_core_msgs::Pose2DStampedWithLimits& goal,
    const arti_nav_core_msgs::Path2DWithLimits& path_limits) override;

  arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum makePlan(
    arti_nav_core_msgs::Path2DWithLimits& plan) override;

protected:
  static bool equal(
    const arti_nav_core_msgs::Pose2DWithLimits& pose_a, const arti_nav_core_msgs::Pose2DWithLimits& pose_b);

  arti_nav_core::Transformer* transformer_{nullptr};
  arti_nav_core_msgs::Path2DWithLimits plan_;
};

}

#endif //ARTI_PASS_THROUGH_PLANNER_BASE_GLOBAL_PLANNER_PASS_THROUGH_H
