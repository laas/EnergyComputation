#ifndef MOTORS_HH
#define MOTORS_HH

#include <iostream>
#include <string>
#include <fstream>      // std::ifstream
#include <sstream>
#include <vector>
#include <cmath>
#include <commonTools.hh>

class Motors
{
public:
  Motors(path_t robotFile);

  int readData(path_t robotFile);

  int DisplayJointI(int i);
  int DisplayAll();
  int ComputeConstantForEnergy();

private:
  std::vector<std::string> titleRobotConfig_ ;
public:
  std::vector<double> torque_constant_ ;
  std::vector<double> resistor_ ;
  std::vector<double> R_K2 ; // R/Kc^2 * tau^2
  std::vector<std::string> joint_name_;
  std::vector<double> gear_ratio_;
};

#endif // MOTORS_HH
