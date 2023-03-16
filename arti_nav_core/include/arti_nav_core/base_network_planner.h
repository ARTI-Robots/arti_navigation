/*
Created by clemens on 27.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_NAV_CORE_BASE_NETWORK_PLANNER_H
#define ARTI_NAV_CORE_BASE_NETWORK_PLANNER_H

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <arti_nav_core_msgs/Path2DWithLimits.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <string>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_nav_core
{
class BaseNetworkPlanner
{
public:
  virtual ~BaseNetworkPlanner() = default;

  /**
   * @brief  Constructs the network planner
   * @param name The name to give this instance of the network planner
   * @param transformer A pointer to a transformer
   */
  virtual void initialize(std::string name, Transformer* transformer) = 0;

  /*!
   * set the goal which is used as bases to calculate a path within the network.
   * The closed node within the network is used for entry and exit. All paths are along the network edges.
   * @param goal the goal to plan to
   * @return true of the goal update did not cause any error
   */
  virtual bool setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) = 0;

  /*!
   * enum to describe the result of the command calculation
   */
  enum class BaseNetworkPlannerErrorEnum
  {
    PLAN_FOUND,
    FAR_FROM_PATH,
    NO_PATH_POSSIBLE
  };

  /*!
   * calculates the plan which reaches the goal considering the network defined in the environment
   * @param plan the plan which was calculated
   * @return if a plan was found or which fault occurred
   */
  virtual BaseNetworkPlannerErrorEnum makePlan(arti_nav_core_msgs::Movement2DGoalWithConstraints& plan) = 0;

  /*!
   * changes the network to avoid using navigation between error_pose_a and error_pose_b.
   * This may cause that the connection between two nodes is permanently removed.
   * \param error_pose_a pose which was reached but caused no further progress
   * \param error_pose_b pose which was not reached
   */
  virtual void handlePlannerError(
    const arti_nav_core_msgs::Pose2DWithLimits& /*error_pose_a*/,
    const arti_nav_core_msgs::Pose2DWithLimits& /*error_pose_b*/)
  {
  }
};
}

#endif //ARTI_NAV_CORE_BASE_NETWORK_PLANNER_H
