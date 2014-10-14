#include "explorefolder.hh"

#ifndef EXPERIENCE_HH
#define EXPERIENCE_HH

class Experience
{
public:
    Experience(Motors * hrp2motors, path_t input_path, path_t rootFolder);

    // getters & setters
    std::vector< std::vector<double> > * data()
    {return &data_ ;}

    // setOutPutPath
    int setOutputPath(path_t rootFolder);

    // read the data
    int readData();
    int treatData();

private :
    path_t input_path_ ;
    path_t output_path_ ;

    int beginData_ ;
    int endData_ ;

    Motors * hrp2motors_ ;
    std::vector<std::string> titleRobotConfig_ ;
    std::vector< std::vector<double> > data_ ;
    std::vector< std::vector<double> > energyOfMotors_ ;
    std::vector< std::vector<double> > energyOfWalking_ ;
};

#endif // EXPERIENCE_HH
