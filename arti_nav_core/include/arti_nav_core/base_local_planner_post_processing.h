/*
Created by clemens on 20.04.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_NAV_CORE_BASE_LOCAL_PLANNER_POST_PROCESSING_H
#define ARTI_NAV_CORE_BASE_LOCAL_PLANNER_POST_PROCESSING_H

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
class BaseLocalPlannerPostProcessing
{
public:
  virtual ~BaseLocalPlannerPostProcessing() = default;

  /**
   * @brief  Constructs the local planner post processing
   * @param name The name to give this instance of the local planner post processing
   * @param transformer A pointer to a transformer
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  virtual void initialize(std::string name, Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) = 0;

  /*!
   * enum to describe the result of the trajectory post processing
   */
  enum class BaseLocalPlannerPostProcessingErrorEnum
  {
    TRAJECTORY_POST_PROCESSED,
    OBSTACLE_TO_CLOSE,
    FAR_FROM_PATH,
    POST_PROCESSING_NOT_POSSIBLE
  };

  /*!
   * post processes the given trajectory and creates a new trajectory
   * @param old_trajectory old trajectory created till now
   * @param final_twist final twist to be applied on the trajectory
   * @param new_trajectory the trajectory after post processing
   * @return if the post processing was successfully or which fault occurred
   */
  virtual BaseLocalPlannerPostProcessingErrorEnum makeTrajectory(
    const arti_nav_core_msgs::Trajectory2DWithLimits& old_trajectory,
    const arti_nav_core_msgs::Twist2DWithLimits& final_twist,
    arti_nav_core_msgs::Trajectory2DWithLimits& new_trajectory) = 0;
};
}

#endif //ARTI_NAV_CORE_BASE_LOCAL_PLANNER_POST_PROCESSING_H
