/*
Created by clemens on 11.10.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PASS_THROUGH_PLANNER_BASE_LOCAL_PLANNER_PASS_THROUGH_H
#define ARTI_PASS_THROUGH_PLANNER_BASE_LOCAL_PLANNER_PASS_THROUGH_H

#include <arti_nav_core/base_local_planner.h>
#include <arti_pass_through_planner/robot_information.h>
#include <memory>
#include <ros/node_handle.h>

namespace arti_pass_through_planner
{

class BaseLocalPlannerPassThrough : public arti_nav_core::BaseLocalPlanner
{
public:
  BaseLocalPlannerPassThrough() = default;

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan) override;

  bool setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist) override;

  arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum makeTrajectory(
    arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  bool isGoalReached() override;

private:
  static bool withinTolerance(
    double current_value, double goal_value, double upper_bound, double lower_bound, double tolerance_increase,
    bool is_angular = false);

  bool xValueWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const;
  bool yValueWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const;
  bool thetaValueWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const;

  ros::NodeHandle private_nh_;

  arti_nav_core::Transformer* transformer_{nullptr};

  std::shared_ptr<RobotInformation> robot_information_;

  arti_nav_core_msgs::Trajectory2DWithLimits trajectory_;

  double xy_close_tolerance_increase_{1.0};
  double yaw_close_tolerance_increase_{1.0};
};

}

#endif //ARTI_PASS_THROUGH_PLANNER_BASE_LOCAL_PLANNER_PASS_THROUGH_H
