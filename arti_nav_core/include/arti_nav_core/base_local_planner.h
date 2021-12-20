/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_NAV_CORE_BASE_LOCAL_PLANNER_H
#define ARTI_NAV_CORE_BASE_LOCAL_PLANNER_H

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Path2DWithLimits.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <string>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_nav_core
{
class BaseLocalPlanner
{
public:
  virtual ~BaseLocalPlanner() = default;

  /**
   * @brief  Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param transformer A pointer to a transformer
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  virtual void initialize(std::string name, Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) = 0;

  /*!
   * set the plan which is used as bases to calculate a local plan.
   * The local planner should follow this path as close as possible considering the dynamic constraints of the robot
   * @param plan the plan to use for the local planner
   * @return true if the plan update did not cause any error
   */
  virtual bool setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan) = 0;

  /*!
   * set the final velocity the robot should have at the last point of the plan
   * The local planner should create a trajectory which results in the final twist as possible considering the dynamic
   * constraints of the robot
   * @param final_twist the velocity the robot should have at its goal position
   * @return true if the final twist update did not cause any error
   */
  virtual bool setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist) = 0;

  /*!
   * enum to describe the result of the trajectory calculation
   */
  enum class BaseLocalPlannerErrorEnum
  {
    GOAL_REACHED,
    TRAJECTORY_FOUND,
    OBSTACLE_TO_CLOSE,
    FAR_FROM_PATH,
    NO_TRAJECTORY_POSSIBLE
  };

  /*!
   * calculates a trajectory which allows to follow the path close as possible but avoids obstacles and considers
   * robots dynamics
   * @param trajectory the trajectory which was calculated
   * @return if a plan was found or which fault occurred
   */
  virtual BaseLocalPlannerErrorEnum makeTrajectory(arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) = 0;

  /**
   * @brief  Check if the goal pose has been achieved by the local planner
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached() = 0;
};
}


#endif //ARTI_NAV_CORE_BASE_LOCAL_PLANNER_H
