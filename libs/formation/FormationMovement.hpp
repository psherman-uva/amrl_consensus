/*
  @file:      RobotFormation.hpp
  @author:    psherman
  @date       Nov. 2024
  
  @brief Class defining robots desired position/velocity around formation center
*/

#pragma once

#include <Eigen/Dense>


namespace amrl {

class FormationMovement
{
public:

  FormationMovement(void);
  ~FormationMovement(void);

  double vel_desired(double t);
  double vel_int_desired(double t);
  double vel_dot_desired(double t);

private:



};


}