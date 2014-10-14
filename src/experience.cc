#include "experience.hh"

Experience(Motors * hrp2motors, path_t input_path, path_t rootFolder)
{
    input_path_ = input_path ;
    hrp2motors_ = hrp2motors ;

    beginData_ = 0 ;
    endData_ = 0 ;
    data_.clear() ;
    energyOfMotors_.clear() ;
    energyOfWalking_.clear() ;
    titleRobotConfig_.clear() ;

    setOutputPath(rootFolder);
}

int Experience::setOutputPath(path_t rootFolder)
{
    return 0 ;
}

int Experience::readData()
{
    data_.clear() ;
    std::string astateFile = input_path_.string() ;
    std::ifstream dataStream ;
    dataStream.open(astateFile.c_str(),std::ifstream::in);

    // skipping header
    titleRobotConfig_.resize(166) ;
    for (unsigned int i = 0 ; i < titleRobotConfig_.size() ; ++i)
        dataStream >> titleRobotConfig_[i];

        for (unsigned int i = 0 ; i < titleRobotConfig_.size() ; ++i)
            cout << titleRobotConfig_[i] << " ";
        cout << endl;

    while (dataStream.good()) {
        vector<double> oneLine(176) ;
        for (unsigned int i = 0 ; i < oneLine.size() ; ++i)
            dataStream >> oneLine[i];
        data.push_back(oneLine);
    }
    dataStream.close();
    return 0;
}



int Experience::treatData();