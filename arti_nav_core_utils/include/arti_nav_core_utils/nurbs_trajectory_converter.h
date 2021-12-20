//
// \author Raphael Hoheneder
// \date 19.04.21
// \Copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
//

#ifndef ARTI_NAV_CORE_UTILS_NURBS_TRAJECTORY_CONVERTER_H
#define ARTI_NAV_CORE_UTILS_NURBS_TRAJECTORY_CONVERTER_H

#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <tinynurbs/core/curve.h>
#include <vector>

namespace arti_nav_core_utils
{

class NurbsTrajectoryConverter
{
public:
  explicit NurbsTrajectoryConverter(double step_distance, double min_vel = 0.01, double
  max_lateral_acceleration = 0.1, double max_speed = -1.0f);
  ~NurbsTrajectoryConverter();

  arti_nav_core_msgs::Trajectory2DWithLimits convert();
  void addControlPts(double x, double y, double z);
  unsigned int getControlPtsSize();
  void clearControlPts();
  void addKnot(double knot);
  void clearKnots();
  unsigned int getKnotSize();
  void setDegree(int degree);
  int getDegree();
  void setMaxSpeed(double max_speed);
  void setSpeedPrevious(double speed);
  void setSpeedNext(double speed);
  void setStepDistance(double step_distance);
  void setDistance(double distance);
  void setUseEuclideanDistance();
  void setOrientation(double orientation);
  void setAcceleration(double acceleration);
  void setDeceleration(double deceleration);
  bool checkValidity();
  double getBrakingDistance(double v_prev, double v_next, double acceleration);
  double getVelFromAcceleration(double dist_traveled, double vel_curr, double acceleration);
  double getVelFromDecelaration(double dist_traveled, double vel_curr, double decelaration);

private:
  double step_distance_;
  double min_vel_;
  double max_lateral_acceleration_;
  double max_speed_;
  double distance_{0.0};
  double orientation_{0.0};
  double acceleration_{0.0};
  double deceleration_{0.0};
  double speed_prev_{0.0};
  double speed_next_{0.0};
  tinynurbs::Curve<double> crv_;
  std::vector<glm::vec<3, double>> control_points_;
  std::vector<double> knots_;
};

}

#endif //ARTI_NAV_CORE_UTILS_NURBS_TRAJECTORY_CONVERTER_H
