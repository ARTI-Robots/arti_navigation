//
// \author Raphael Hoheneder
// \date 19.04.21
// \Copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
//

#include <arti_nav_core_utils/nurbs_trajectory_converter.h>
#include <angles/angles.h>
#include <cmath>
#include <ros/console.h>
#include <tinynurbs/tinynurbs.h>
#include <boost/optional.hpp>

namespace arti_nav_core_utils
{

NurbsTrajectoryConverter::NurbsTrajectoryConverter(double step_distance, double min_vel, double
max_lateral_acceleration, double max_speed, double max_delta_omega, double wheelbase)
: step_distance_(step_distance), min_vel_(min_vel) , max_lateral_acceleration_(max_lateral_acceleration),
  max_speed_(max_speed), max_delta_omega_(max_delta_omega), wheelbase_(wheelbase)
{
}

NurbsTrajectoryConverter::~NurbsTrajectoryConverter() = default;

arti_nav_core_msgs::Trajectory2DWithLimits NurbsTrajectoryConverter::convert()
{
  arti_nav_core_msgs::Trajectory2DWithLimits trajectories;
  //arti_nav_core_msgs::Movement2DWithLimits
  // speed ramp

  double accumulated_distance = 0.0;
  //int speedup = 0;
  //int speeddown = 0;
/*  if (max_speed_ > 0.0)
  {
    double curr_speed = max_speed_;

    double break_dist_start = getBrakingDistance(speed_prev_, curr_speed, acceleration_);
    ROS_DEBUG_STREAM("acc distance: " << break_dist_start);
    //speedup = std::ceil(break_dist_start / step_distance_);
    //ROS_DEBUG_STREAM("speedstep up: " << speedup << " s_ac: " << break_dist_start << " step_dist: " <<
                                      step_distance_);
    if (speed_prev_ - curr_speed >= 0.0)
    {
      //speedup = 0;
    }

    double break_dist_end = getBrakingDistance(curr_speed, speed_next_, deceleration_);
    ROS_DEBUG_STREAM("breakdistance: " << break_dist_end);
    //if(break_dist_end > distance_)
    //  break_dist_end = distance_;
    //speeddown = std::ceil(break_dist_end / step_distance_);

    if (speed_next_ - curr_speed >= 0.0)
    {
      //speeddown = 0;
    }
    //ROS_DEBUG_STREAM("speedstep down: " << speeddown << " s_de: " << break_dist_end << " step_dist: " <<
    //                                    step_distance_);
  }*/
  double nr_interval = std::floor(distance_ / step_distance_); //(0.1m)
  double distance_to_goal = getDistanceToGoal( 0, nr_interval);
  glm::vec<3, double_t> old_pt;

  boost::optional<double> old_yaw = boost::make_optional(false, double());

  // iterate over curve with nr_interval steps
  for(int i = 0; i <= nr_interval; i++)
  {
    double val = double(i)/double(nr_interval);
    if(i == nr_interval)
    {
      val = 1.0;
    }
    glm::vec<3, double_t> point = tinynurbs::curvePoint(crv_, val);
    glm::vec<3, double_t> orientation = tinynurbs::curveTangent(crv_, val);
    ROS_DEBUG_STREAM("Curve Point u: " << val << " x: " << point.x << " y: " << point.y << " z: " << point.z <<
                                       " | Orientation x: " << orientation.x << " y:" << orientation.y << " z:" <<
                                       orientation.z << " b:" << orientation.b << " length:" << orientation.length());
    std::vector<glm::vec<3, double_t>> deriv = tinynurbs::curveDerivatives(crv_, 2, val);

    double yaw = std::atan2(orientation.y, orientation.x);

    ROS_DEBUG_STREAM("yaw: " << yaw);

    arti_nav_core_msgs::Movement2DWithLimits move;
    move.pose.point.x.value = point.x;
    move.pose.point.x.has_limits = false;

    move.pose.point.y.value = point.y;
    move.pose.point.y.has_limits = false;

    // TODO Add thresholds xy, theta
    //if(threshold_xy_ )

    move.pose.theta.value = yaw;
    if (orientation_ != 0.0)
    {
      move.pose.theta.value = angles::normalize_angle(yaw - orientation_);
    }
    move.pose.theta.has_limits = false;
    double real_distance = 0.0;
    if(i > 0)
    {
      double val_0 = double(i-1) / double(nr_interval);

      //glm::vec<3, double_t> first_point = tinynurbs::curvePoint(crv_, val_0);
      real_distance = std::sqrt(((old_pt.x - point.x) * (old_pt.x - point.x)) +
                                ((old_pt.y - point.y) * (old_pt.y - point.y)));
    }
    glm::vec<3, double_t> end_point = tinynurbs::curvePoint(crv_, 1.0);

    accumulated_distance += real_distance;
    distance_to_goal -= real_distance;
    if (max_speed_ > 0.0)
    {
      move.twist.x.has_limits = true;
      double speed = max_speed_;

      // Speed ramp up
      //if ((speed_prev_ <= speed) && (i < speedup) && (speedup != 0)) // omit zero value at start
      {

        double new_speed = getVelFromAcceleration( accumulated_distance, speed_prev_, acceleration_);
        //if(i == 0)
        //  new_speed = getVelFromDecelaration((speedup - i+1) * step_distance_, max_speed_, acceleration_);
        ROS_DEBUG_STREAM("speedup: " << new_speed << " distance: " << accumulated_distance);//i * step_distance_);
        speed = (speed < new_speed) ? speed : new_speed;
      }
      //Speed ramp down
      //if ((speed_next_ <= speed) && ((i) >= (nr_interval - speeddown)) && (speeddown != 0))
      double breaking_d = getBrakingDistance(speed_prev_, speed_next_, deceleration_) - distance_to_goal;
      if(getBrakingDistance(speed_prev_, speed_next_, deceleration_) <= distance_to_goal)
      {
        //int s = nr_interval - i;
        double new_speed = getVelFromAcceleration(distance_to_goal, speed_next_, deceleration_);
        /*
        ROS_DEBUG_STREAM("nr_interval: " << nr_interval << " i: " << i << " speed_down: " << speeddown
                                         << "calc_speed: " << new_speed
                                         << " soll: " << speed << "  s: " << s);
        ROS_DEBUG_STREAM("new_speed: " << new_speed); //<< "  speed form interpolation: " << speed_new);
        */
        speed = (speed < new_speed) ? speed : new_speed;
      }

      if (speed < 0.0)
      {
        speed = 0.0;
      }
      if (speed < min_vel_)
      {
        speed = min_vel_;
      }

      ROS_DEBUG_STREAM("speed: " << speed);

      move.twist.x.value = speed;
      move.twist.x.upper_limit = 0.0; // relative limits to target speed aka x.value
      move.twist.x.lower_limit = -speed; // relative limits to target speed aka x.value
      if (std::abs(orientation_) >= M_PI_2) // reverse drive
      {
        move.twist.x.value = -speed;
        move.twist.x.upper_limit = speed; // relative limits to target speed aka x.value
        move.twist.x.lower_limit = 0.0;
      }

      //move.twist.theta.value = delta_theta?

    }

    glm::vec<3, double_t> t = deriv[0];

    //t = deriv[1];
    //t = glm::normalize(t);
    if (deriv[2].x != 0.0 && deriv[2].y != 0.0)
    {
      ROS_DEBUG_STREAM("Orientation x: " << orientation.x << " y:" << orientation.y);
      // 1st derivative is the tangent or Orientation of a evaluation point on the curve.
      ROS_DEBUG_STREAM(
        "derivative[1] x: " << deriv[1].x << " y:" << deriv[1].y );//<< " | norm x:" << t.x << " y:" << t.y);
      // 2nd derivative is the curvature -> use to calculate the radius at evaluation point on the curve.
      t = glm::normalize(deriv[2]);
      ROS_DEBUG_STREAM(
        "derivative[2] x: " << deriv[2].x << " y:" << deriv[2].y );//<< " | norm x:" << t.x << " y:" << t.y);
      //double k2 = std::hypot(deriv[2].x, deriv[2].y);

      double k = std::abs((deriv[1].x *deriv[2].y) - (deriv[2].x * deriv[1].y)) / std::pow((std::pow(deriv[1].x,2) +
        std::pow(deriv[1].y, 2)),(3.0/2.0));
      //if(std::abs(k - k2) > 0.02)
      //{
      //  ROS_WARN_STREAM("Radius missmatch k: " << k << " k2: " << k2);
      //}
      double r = k < 0.0001 ? 10000 : 1 / k; // avoid numerical instability
      ROS_DEBUG_STREAM("K: " << k << " | r: " << r);
      double v_theta = 0.0; //yaw * r; // angular velocity
      if(old_yaw.is_initialized())
      {
        // FIX ME real_distance is far from the step distance in curves, is there something better than euclidean
        // distance? maybe oversampling?
        double delta_t = real_distance / std::abs(move.twist.x.value);
        v_theta = angles::normalize_angle(old_yaw.value() - yaw) * r/delta_t;
      }
      old_yaw = yaw;
      double v_curve_max = std::sqrt(r * max_lateral_acceleration_);
      v_curve_max = v_curve_max > min_vel_ ? v_curve_max : min_vel_;
      if(std::abs(move.twist.x.value) > std::abs(v_curve_max))
      {
        ROS_DEBUG_STREAM("Max curve speed reduction, v: " <<  move.twist.x.value << " | v_curve_max: " << v_curve_max);
        move.twist.x.value = std::copysign(v_curve_max,  move.twist.x.value);

      }
      ROS_DEBUG_STREAM("yaw: " << yaw << "  r: " << r << " v_theta: " << v_theta);
      move.twist.theta.value = v_theta;

      //if( wheelbase_ != 0.0 && max_delta_omega_ != 0.0 &&

      {
        double r2 = 100000;
        if((i-1) >= 0 )
        {
          double val_0 = double(i - 1) / double(nr_interval);
          std::vector<glm::vec<3, double_t>> deriv_0 = tinynurbs::curveDerivatives(crv_, 2, val_0);
          //std::vector<glm::vec<3, double_t>> pos_0 = tinynurbs::curveDerivatives(crv_, 0, val);


          // https://en.wikipedia.org/wiki/Curvature#In_terms_of_a_general_parametrization
          double k2 = std::abs((deriv_0[1].x * deriv_0[2].y) - (deriv_0[2].x * deriv_0[1].y)) /
                          std::pow((std::pow(deriv_0[1].x,2)+ std::pow(deriv_0[1].y,2)),(3.0 / 2.0));

          r2 = k2 < 0.0001 ? 10000 : 1 / k2;
        }// avoid numerical instability
        double omega_end = std::atan2(wheelbase_ , r);
        double omega_start = std::atan2(wheelbase_ ,r2);
        if( std::abs(nr_interval - i ) < 1.0)
        {
          ROS_INFO("end of edge reached, set end angle to 0.0");
          omega_end = 0.0;
        }

        double delta_t = std::abs(omega_start - omega_end) / max_delta_omega_;
        double v_max = real_distance / delta_t;
        if((real_distance < 0.001) )
        {
          glm::vec<3, double_t> point2 = tinynurbs::curvePoint(crv_, (double(i+1.0)/nr_interval));
          double dist = std::sqrt(((point.x -point2.x)*(point.x-point2.x))+((point.y -point2.y)*(point.y-point2.y)));
          v_max = dist/delta_t;
        }

        ROS_DEBUG("d_omega: %f, v_max [%f]", (omega_start - omega_end)*180.0/M_PI, v_max);
        ROS_DEBUG("index: [%d], steering_angle: [%f], r: [%f], prev_angle [%f]", i , omega_end*180.0/M_PI, r, omega_start*180.0/M_PI);

        if( (!trajectories.movements.empty()) && !( std::abs(nr_interval - i ) < 1.0))
        {
          if (std::abs(trajectories.movements.back().twist.x.value) >= v_max)
          {
            ROS_INFO(
              "MAX velocity reduced to [%f] from [%f], because of max steering angle change limit, index[%d]  ",
              v_max, trajectories.movements.back().twist.x.value, i);
            if( v_max < min_vel_)
            {
              v_max = min_vel_;
            }
            trajectories.movements.back().twist.x.value = std::copysign(v_max, move.twist.x.value);
          }
        }
        else
        {
          if (std::abs(move.twist.x.value) >= v_max)
          {
          ROS_INFO("first/last movement, MAX velocity reduced to [%f] from [%f], because of max steering angle "
                   "change limit, index[%d]  ",
          v_max, move.twist.x.value, i);
          move.twist.x.value = std::copysign(v_max, move.twist.x.value);
          }
        }
      }

      ROS_DEBUG_STREAM("\n #############################################################################");
    }
    trajectories.movements.push_back(move);
    old_pt = point;
  }
  return trajectories;
}


void NurbsTrajectoryConverter::addControlPts(double x, double y, double z)
{
  control_points_.emplace_back(x, y, z);
  crv_.control_points.emplace_back(x, y, z);
}

unsigned int NurbsTrajectoryConverter::getControlPtsSize()
{
  return control_points_.size();
}

void NurbsTrajectoryConverter::clearControlPts()
{
  control_points_.clear();
  crv_.control_points.clear();
}

void NurbsTrajectoryConverter::addKnot(double knot)
{
  knots_.push_back(knot);
  crv_.knots.push_back(knot);
}

void NurbsTrajectoryConverter::clearKnots()
{
  knots_.clear();
  crv_.knots.clear();
}

unsigned int NurbsTrajectoryConverter::getKnotSize()
{
  return crv_.knots.size();
}

void NurbsTrajectoryConverter::setDegree(int degree)
{
  crv_.degree = degree;
}

int NurbsTrajectoryConverter::getDegree()
{
  return crv_.degree;
}

void NurbsTrajectoryConverter::setMaxSpeed(double max_speed)
{
  max_speed_ = max_speed;
}

void NurbsTrajectoryConverter::setSpeedPrevious(double speed)
{
  speed_prev_ = speed;
}

void NurbsTrajectoryConverter::setSpeedNext(double speed)
{
  speed_next_ = speed;
}

void NurbsTrajectoryConverter::setStepDistance(double step_distance)
{
  step_distance_ = step_distance;
}

bool NurbsTrajectoryConverter::checkValidity()
{
  return curveIsValid(crv_);
}

void NurbsTrajectoryConverter::setAcceleration(double acceleration)
{
  acceleration_ = acceleration;
}

void NurbsTrajectoryConverter::setDeceleration(double deceleration)
{
  deceleration_ = deceleration;
}

double NurbsTrajectoryConverter::getBrakingDistance(double v_prev, double v_next, double acceleration)
{
  if (acceleration < 0.001)
  {
    ROS_WARN_STREAM("acceleration limit 0.001, given value: " << acceleration << " overridden");
    acceleration = 0.001;
  }
  double breaking_distance = std::abs(std::pow(v_prev, 2) - std::pow(v_next, 2)) / (2 * acceleration);
  return breaking_distance;
}

double NurbsTrajectoryConverter::getVelFromAcceleration(double dist_traveled, double vel_curr, double acceleration)
{
  if (std::abs(acceleration) < 0.001)
  {
    ROS_WARN_STREAM("acceleration limit |0.001|, given value: " << acceleration << " overridden");
    acceleration = 0.001;
  }

  double resulting_vel = std::sqrt(std::pow(vel_curr, 2) + (2 * std::abs(acceleration) * dist_traveled));

  if ((resulting_vel < 0.0) || !std::isfinite(resulting_vel))
  {
    resulting_vel = 0.0;
    ROS_WARN_STREAM("getVelFromAcceleration: " << resulting_vel);
  }
  return resulting_vel;
}

double NurbsTrajectoryConverter::getVelFromDecelaration(double dist_traveled, double vel_curr, double decelaration)
{
  if (std::abs(decelaration) < 0.001)
  {
    ROS_WARN_STREAM("decelaration limit |0.001|, given value: " << decelaration << " overridden");
    decelaration = 0.001;
  }

  double resulting_vel = std::sqrt(std::pow(vel_curr, 2) - (2 * std::abs(decelaration) * dist_traveled));

  if ((resulting_vel < 0.0) || !std::isfinite(resulting_vel))
  {
    resulting_vel = 0.0;
    ROS_WARN_STREAM("getVelFromDecelaration: " << resulting_vel);
  }
  //ROS_DEBUG_STREAM("NURBS_TRAJECTORY: vel_curr: " << vel_curr << " diff: " << delta_vel << " s: " << dist_traveled);*/
  return resulting_vel;
}

void NurbsTrajectoryConverter::setDistance(double distance)
{
  distance_ = distance;
}

void NurbsTrajectoryConverter::setUseEuclideanDistance()
{
  ROS_WARN_STREAM("use only the euclidean distance between points");
  int s = control_points_.size() - 1;
  for (size_t i = 1; i < control_points_.size(); i++)
  {
    distance_ += std::sqrt(
      std::pow((control_points_[0].x - control_points_[s].x), 2) +
      std::pow((control_points_[0].y - control_points_[s].y), 2));
  }
}

void NurbsTrajectoryConverter::setOrientation(double orientation)
{
  orientation_ = orientation;
}

std::vector<double> NurbsTrajectoryConverter::getKnots()
{
  return knots_;
}

double NurbsTrajectoryConverter::getDistanceToGoal(int i, double interval)
{
  double u = (double)i / interval;
  glm::vec<3, double> p1 =  tinynurbs::curvePoint(crv_, u);
  double dist = 0.0;
  for(int run = i+1; (double)run <= interval; run++)
  {
    u = (double)run / interval;
    if(u > 1.0)
    {
      ROS_WARN("goal-distance overshoot: u[%f], correct to 1.0 (100pct)", u);
      u = 1.0;
    }
    glm::vec<3, double> p2 =  tinynurbs::curvePoint(crv_, u);
    dist += std::hypot((p1.x-p2.x), (p1.y-p2.y));
    p1 = p2;
  }
  return dist;
}

}
