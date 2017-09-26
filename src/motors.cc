#include "motors.hh"

Motors::Motors(path_t robotFile)
{
    readData(robotFile);
    //DisplayAll();
    ComputeConstantForEnergy();
}

int Motors::readData(path_t robotFile){
    std::string RobotConfig = robotFile.string() ;
    std::ifstream dataStream ;
    dataStream.open(RobotConfig.c_str(),std::ifstream::in);

    // skipping header
    titleRobotConfig_.resize(4);
    std::vector< std::vector<double> > robotConfig ;
    for (unsigned int i = 0 ; i < 4 ; ++i)
        dataStream >> titleRobotConfig_[i];

    while (dataStream.good()) {
        std::string Jointname ;
        double value ;

        dataStream >> Jointname ;
        joint_name_.push_back(Jointname);

        dataStream >> value ;
        torque_constant_.push_back(value);
        dataStream >> value ;
        resistor_.push_back(value);
        dataStream >> value ;
        gear_ratio_.push_back(value);
    }

    dataStream.close();
    DisplayAll();
    return 0;
}

int Motors::DisplayJointI(int i)
{
    std::cout << joint_name_[i].c_str() << " " ;
    std::cout << torque_constant_[i] << " " ;
    std::cout << resistor_[i] << " " ;
    std::cout << gear_ratio_[i] << " " ;
    std::cout << std::endl ;
    return 0;
}

int Motors::DisplayAll()
{
    for (unsigned int i = 0 ; i < titleRobotConfig_.size() ; ++i)
        std::cout << titleRobotConfig_[i] << " ";
    std::cout << std::endl ;

    for (unsigned int i = 0 ; i < joint_name_.size() ; ++i)
        DisplayJointI(i);
    std::cout << std::endl ;

    return 0;
}

int Motors::ComputeConstantForEnergy()
{
    R_K2.resize(joint_name_.size());
    for (unsigned int i = 0 ; i < R_K2.size() ; ++i)
    {
        R_K2[i] = resistor_[i] / (torque_constant_[i]*torque_constant_[i]);
        //std::cout << R_K2[i]  << " " ;
    }
    //std::cout << std::endl;
}
