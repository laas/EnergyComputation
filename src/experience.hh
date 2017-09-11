#include "explorefolder.hh"
#include "motors.hh"
#include "commonTools.hh"

#ifndef EXPERIENCE_HH
#define EXPERIENCE_HH

class Experience
{
public: // methods
    Experience(Motors * hrp2motors,
               se3::Model * hrp2model,
               path_t input_state_path,
               path_t input_ref_path,
               path_t rootFolder);

    // handle the data
    int handleData();

    // getter & setter
    std::string name()
    {return experienceName_ ;}
    double walkedDistanced()
    {return walkedDistanced_ ;}
    double EnergyOfMotor()
    {return EnergyOfMotor_J_m_ ;}
    double EnergyOfWalking()
    {return EnergyOfWalking_J_m_ ;}
    double TimeTravelled()
    {return (0.005*(endData_-beginData_)) ;}

private : // methods
    int setExperienceName(path_t rootFolder);
    int readData();
    int filterTheData();
    int defineBeginEndIndexes();
    int computeTheEnergy();
    int compareRefMeasure();

private : // attributes
    path_t input_astate_path_ ;
    path_t input_ref_path_ ;
    std::string experienceName_ ;

    int beginData_ ;
    int endData_ ;
    unsigned int ddl_ ;
    Motors * hrp2motors_ ;
    se3::Model * robotModel_ ;
    se3::Data * robotData_ ;
    std::vector< std::string > titleRobotConfig_ ;
    std::vector< std::vector<double> > data_astate_ ;
    std::vector< std::vector<double> > data_ref_ ;

    // value computed from filtered signal
    // q_,dq_,ddq_ are evaluated as the motor rotation
    std::vector< std::vector<double> > q_,dq_,ddq_ ;
    // q_j_,dq_j_,dq_j_,
    std::vector< std::vector<double> > q_joint_ ;
    // reference torques at the motor level
    std::vector< std::vector<double> > torques_ ;
    // rnea at the joint level
    std::vector< std::vector<double> > rnea_torques_ ;
    // raw data
    std::vector< std::vector<double> > q_astate_ ;
    std::vector< std::vector<double> > q_ref_ ;

    std::vector< std::vector<double> > powerOutputMotors_ ;
    std::vector< std::vector<double> > powerOfWalk_ ;
    std::vector< std::vector<double> > errMeasureReference_ ;

    double walkedDistanced_ ;
    double EnergyOfMotor_J_m_ ;
    double EnergyOfWalking_J_m_ ;

    InfiniteImpendanceFilter IIF ;
};

#endif // EXPERIENCE_HH
