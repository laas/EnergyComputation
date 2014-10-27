#include "experience.hh"

using namespace std ;

Experience::Experience(Motors * hrp2motors, path_t input_path, path_t rootFolder)
{
    input_path_ = input_path ;
    hrp2motors_ = hrp2motors ;

    ddl_ = hrp2motors->gear_ratio_.size();

    beginData_ = 0 ;
    endData_ = 0 ;

    data_.clear() ;

    powerOutputMotors_.clear() ;
    powerOfWalk_.clear() ;

    EnergyOfMotor_J_m_ = 0.0 ;
    EnergyOfWalking_J_m_ = 0.0 ;
    walkedDistanced_ = -1.0 ;

    titleRobotConfig_.clear() ;

    q_.clear() ;
    dq_.clear() ;
    torques_.clear() ;

    setExperienceName(rootFolder);
}

int Experience::setExperienceName(path_t rootFolder)
{
    string rootFolder_st = rootFolder.string();
    string input_path_st = input_path_.string();

    experienceName_ = input_path_st.replace(0,rootFolder_st.length()+1,"") ;

    string asate_log = "-astate.log" ;
    size_t found = experienceName_.find(asate_log);
    experienceName_ = experienceName_.replace(found,asate_log.length()+1,"") ;

    bool ok = true ;
    while(ok)
    {
        found = experienceName_.find("/");
        if (found==std::string::npos)
            ok = false ;
        else
        {
            ok = true ;
            experienceName_ = experienceName_.replace(found,1,"_") ;
        }
    }

    string astring = "gravles" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 1.4 ;

    astring = "slipFloor" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 3.6 ;

    astring = "stepOver" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 0.4149695915 ;

    astring = "slopes" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 1.4 ;

    astring = "stairs" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 0.9 ;

    astring = "steppingStones" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 1.4 ;

    return 0 ;
}

int Experience::handleData()
{
    readData();
    defineBeginEndIndexes();
    computeTheEnergy();
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

    // reading all the data file
    while (dataStream.good()) {
        vector<double> oneLine(176) ;
        for (unsigned int i = 0 ; i < oneLine.size() ; ++i)
            dataStream >> oneLine[i];
        data_.push_back(oneLine);
    }
    dataStream.close();

    // save the data wanted in separated buffers for computation
    int N = data_.size() ;
    q_.resize( N , vector<double>(ddl_,0) ) ;
    dq_.resize( N , vector<double>(ddl_,0) ) ;
    torques_.resize( N , vector<double>(ddl_,0) ) ;

    for (unsigned int i = 0 ; i < N ; ++i )
    {
        for (unsigned int j = 0 ; j < 30 ; ++j )
        {
            q_[i][j] = hrp2motors_->gear_ratio_[j] * data_[i][j];
        }
        for (unsigned int j = 40 ; j < 40+30 ; ++j )
        {
            torques_[i][j-40] = data_[i][j];
        }
    }

    //    // use an order 1 finite low pass filter to clean the data
    for (unsigned int j = 0 ; j < ddl_ ; ++j )
        for (unsigned int i = 0 ; i < N ; ++i )
            lowpass(torques_[i][j],torques_[i][j],i,0.03);

//    for (unsigned int j = 0 ; j < ddl_ ; ++j )
//        for (unsigned int i = 0 ; i < N ; ++i )
//            lowpass(q_[i][j],q_[i][j],i,0.01);

    // compute the velocity of the joints by finite differenziation
    for (unsigned int j = 0 ; j < ddl_ ; ++j )
        for (unsigned int i = 0 ; i < N ; ++i )
            derivation(q_[i][j],dq_[i][j],i);

    for (unsigned int j = 0 ; j < ddl_ ; ++j )
        for (unsigned int i = 0 ; i < N ; ++i )
            lowpass(dq_[i][j],dq_[i][j],i,0.03);

    return 0;
}

int Experience::defineBeginEndIndexes()
{
    int N = data_.size() ;
    unsigned int i = 0 ;
    bool found = false ;
    while ( !found && i < N )
    {
        int test = 0 ;
        for (unsigned int j = 0 ; j < ddl_ ; ++j )
            test += abs(dq_[i][j]) > 0.5 ;
        if (test >= (int)(ddl_/5) )
            found = true ;
        else
            found = false ;

        beginData_ = i ;
        ++i;
    }
    if(i == N-1)
        cout << "Failure to find the beggining of the motion\n" ;

    found = false ;
    while ( !found && i < N )
    {
        int test = 0 ;
        for (unsigned int j = 0 ; j < ddl_ ; ++j )
            test += abs(dq_[i][j]) < 0.5 ;
        if (test >= ddl_)
            found = true ;
        else
            found = false ;

        endData_ = i ;
        ++i;
    }

    vector< vector<double> > tmp_torques = torques_ ;
    vector< vector<double> > tmp_q = q_ ;
    vector< vector<double> > tmp_dq = dq_ ;

    torques_.resize(endData_-beginData_);
    q_.resize(endData_-beginData_);
    dq_.resize(endData_-beginData_);

    for(unsigned int k = 0 ; k < torques_.size() ; ++k)
    {
        torques_[k] = tmp_torques[k+beginData_] ;
        q_[k] = tmp_q[k+beginData_] ;
        dq_[k] = tmp_dq[k+beginData_] ;
    }
    return 0 ;
}

int Experience::computeTheEnergy()
{
    unsigned int N = dq_.size();
    powerOutputMotors_.resize( N , vector<double> (ddl_,0) );
    powerOfWalk_.resize( N , vector<double> (ddl_,0) );
    for (unsigned int i = 0 ; i < N ; ++i )
    {
        for (unsigned int j = 0 ; j < ddl_ ; ++j )
        {
            powerOutputMotors_[i][j] = abs(dq_[i][j]) * abs(torques_[i][j]) ;
            powerOfWalk_[i][j] = hrp2motors_->R_K2[j] * torques_[i][j] * torques_[i][j] +
                    powerOutputMotors_[i][j] ;
        }
    }

    vector< vector<double> > energyOfMotors ( N , vector<double> (ddl_,0) );
    vector< vector<double> > energyOfWalk ( N , vector<double> (ddl_,0) );
    double dt = 0.005 ;
    for (unsigned int i = 1 ; i < energyOfMotors.size() ; ++i )
    {
        for (unsigned int j = 0 ; j < ddl_ ; ++j )
        {
            energyOfMotors[i][j] = energyOfMotors[i-1][j] + powerOutputMotors_[i][j]*dt ;
            energyOfWalk[i][j] = energyOfWalk[i-1][j] + powerOfWalk_[i][j]*dt ;
        }
    }

    EnergyOfMotor_J_m_ = 0.0;
    EnergyOfWalking_J_m_ = 0.0;
    for (unsigned int j = 0 ; j < ddl_ ; ++j )
    {
        EnergyOfMotor_J_m_ += energyOfMotors.back()[j] ;
        EnergyOfWalking_J_m_ += energyOfWalk.back()[j] ;
    }
    EnergyOfMotor_J_m_ = EnergyOfMotor_J_m_/walkedDistanced_ ;
    EnergyOfWalking_J_m_ = EnergyOfWalking_J_m_/walkedDistanced_;

    return 0 ;
}

