/*
Created by clemens on 26.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_NAV_CORE_BASE_PATH_FOLLOWER_H
#define ARTI_NAV_CORE_BASE_PATH_FOLLOWER_H

#ifdef __GNUC__
#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>

#pragma GCC diagnostic pop
#endif


namespace arti_nav_core
{
class BasePathFollower
{
public:
  virtual ~BasePathFollower() = default;

  /*!
   * set the trajectory which is used as bases to calculate the next best command.
   * The next best command should follow this trajectory as close as possible considering the dynamic constraints of the robot
   * @param trajectory the trajectory to use for the path follower
   * @return true of the trajectory update did not cause any error
   */
  virtual bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) = 0;

  /*!
   * enum to describe the result of the command calculation
   */
  enum class BasePathFollowerErrorEnum
  {
    GOAL_REACHED,
    COMMAND_FOUND,
    OBSTACLE_CLOSE,
    OBSTACLE_IN_FRONT,
    FAR_FROM_PATH,
    NO_COMMAND_POSSIBLE
  };

  /*!
   * calculates the next best command which allows to follow the trajectory as close as possible but avoids obstacles
   * and considers robots dynamics
   * @param next_command the command which was calculated
   * @return if a command was found or which fault occurred
   */
  virtual BasePathFollowerErrorEnum computeVelocityCommands(geometry_msgs::Twist& next_command) = 0;

  /**
   * @brief  Constructs the path follower
   * @param name The name to give this instance of the path follower
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to commands
   */
  virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached() = 0;
};
}

#endif //ARTI_NAV_CORE_BASE_PATH_FOLLOWER_H
