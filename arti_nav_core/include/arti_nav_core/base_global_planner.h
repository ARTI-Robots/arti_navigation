/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_NAV_CORE_BASE_GLOBAL_PLANNER_H
#define ARTI_NAV_CORE_BASE_GLOBAL_PLANNER_H

#ifdef __GNUC__
#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Path2DWithLimits.h>
#include <costmap_2d/costmap_2d_ros.h>

#pragma GCC diagnostic pop
#endif

namespace arti_nav_core
{
class BaseGlobalPlanner
{
public:
  virtual ~BaseGlobalPlanner() = default;

  /*!
   * set the goal which is used as bases to calculate a path from the current position considering the robots kinematic
   * and the environment.
   * The plan will reach the given goal considering the kinematic constraints of the robot
   * @param goal the goal to plan to
   * @return true of the goal update did not cause any error
   */
  virtual bool setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) = 0;

  /*!
   * enum to describe the result of the command calculation
   */
  enum class BaseGlobalPlannerErrorEnum
  {
    PLAN_FOUND,
    FAR_FROM_PATH,
    NO_PATH_POSSIBLE
  };

  /*!
   * calculates the plan which reaches the goal considering the kinematic of the robot and the environment
   * @param plan the plan which was calculated
   * @return if a plan was found or which fault occurred
   */
  virtual BaseGlobalPlannerErrorEnum makePlan(arti_nav_core_msgs::Path2DWithLimits& plan) = 0;

  /**
   * @brief  Constructs the global planner
   * @param name The name to give this instance of the global planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs for a plan
   */
  virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;
};
}

#endif //ARTI_NAV_CORE_BASE_GLOBAL_PLANNER_H
