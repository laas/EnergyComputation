#include "experience.hh"

using namespace std ;

Experience::Experience(Motors * hrp2motors,
           //se3::Model * hrp2model,
           path_t input_state_path,
           path_t input_ref_path,
           path_t rootFolder)
{
    input_astate_path_ = input_state_path ;
    input_ref_path_ = input_ref_path ;
    hrp2motors_ = hrp2motors ;
//    robotModel_ = hrp2model ;
//    robotData_ = new se3::Data(*robotModel_);

    ddl_ = hrp2motors->gear_ratio_.size();

    beginData_ = 0 ;
    endData_ = 0 ;

    data_astate_.clear() ;

    powerOutputMotors_.clear() ;
    powerOfWalk_.clear() ;

    EnergyOfMotor_J_m_s_ = 0.0 ;
    EnergyOfWalking_J_m_s_ = 0.0 ;
    walkedDistanced_ = -1.0 ;
    MechaCostOfTransport_ = 0.0 ;
    CostOfTransport_ = 0.0 ;
    FroudeNumber_ = 0.0 ;

    WeightOfRobot_ = 560 ; // N
    LegLenght_ = 0.7 ; // m //TO BE VERIFIED
    Gravity_ = 9.81 ;  // m.s-2

    titleRobotConfig_.clear() ;

    q_.clear() ;
    dq_.clear() ;
    torques_.clear() ;
    q_ref_.clear();
    q_astate_.clear();

    setExperienceName(rootFolder);
}

int Experience::setExperienceName(path_t rootFolder)
{
    string rootFolder_st = rootFolder.string();
    string input_path_st = input_astate_path_.string();

    experienceName_ = input_path_st.replace(0,rootFolder_st.length(),"") ;

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

    astring = "slope" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 1.4 ;

    astring = "10cm" ; //stairs 10cm up
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 0.9 ;

    astring = "15cm" ; //stairs 15cm up multicontact
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 0.6 ;

    astring = "steppingStones" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 1.4 ;

    astring = "hwalk" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
         walkedDistanced_ = 2.0 ;

    astring = "Beam" ;
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 3.2 ;

    astring = "npg" ; // non linear PG with joystick
    found = experienceName_.find(astring);
    if ( found != std::string::npos)
        walkedDistanced_ = 3.5 ; //approximately

    return 0 ;
}

int Experience::handleData()
{
    readData();
    if (defineBeginEndIndexes() == -1)
    {
      return -1;
    }
    computeTheEnergy();
    compareRefMeasure();
    return 0 ;
}

int Experience::readData()
{
    data_astate_.clear() ;
    std::string astateFile = input_astate_path_.string() ;
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
        data_astate_.push_back(oneLine);
    }
    dataStream.close();

    data_ref_.clear() ;
    std::string rstateFile = input_ref_path_.string() ;
    dataStream.open(rstateFile.c_str(),std::ifstream::in);

    // skipping header
    titleRobotConfig_.resize(95) ;
    for (unsigned int i = 0 ; i < titleRobotConfig_.size() ; ++i)
        dataStream >> titleRobotConfig_[i];

    // reading all the data file
    while (dataStream.good()) {
        vector<double> oneLine(100) ;
        for (unsigned int i = 0 ; i < oneLine.size() ; ++i)
            dataStream >> oneLine[i];
        data_ref_.push_back(oneLine);
    }
    dataStream.close();

    // save the data wanted in separated buffers for computation
    int N = data_astate_.size() ;
    q_.resize( N , vector<double>(ddl_,0) ) ;
    q_ref_.resize( N , vector<double>(ddl_,0) ) ;
    q_astate_.resize( N , vector<double>(ddl_,0) );
    dq_.resize( N , vector<double>(ddl_,0) ) ;
    ddq_.resize( N , vector<double>(ddl_,0) ) ;
    torques_.resize( N , vector<double>(ddl_,0) ) ;
    rnea_torques_.resize( N , vector<double>(ddl_,0) ) ;

    for (unsigned int i = 0 ; i < N ; ++i )
    {
        for (unsigned int j = 0 ; j < 30 ; ++j )
        {
            q_[i][j] = hrp2motors_->gear_ratio_[j] * data_astate_[i][j];
        }
        for (unsigned int j = 0 ; j < 30 ; ++j )
        {
            q_astate_[i][j] = data_astate_[i][j];
            q_ref_[i][j] = data_ref_[i][j];
        }
        for (unsigned int j = 40 ; j < 40+30 ; ++j )
        {
            torques_[i][j-40] = data_astate_[i][j] / hrp2motors_->gear_ratio_[j-40];
        }
    }
    // Create a temporary vector q_motor
    std::vector< std::vector<double> > q_tmp = q_ ;
    IIF.filter(q_,q_tmp);
    std::vector< std::vector<double> > torques_tmp = torques_ ;
    IIF.filter(torques_,torques_tmp);

    // compute the velocity of the joints by finite differenziation
    derivation(q_tmp,dq_);

    // compute the acceleration of the joints by finite differenziation
    derivation(dq_,ddq_);

    // dump all the files
//    string dump = experienceName_+"_q.dat" ;
//    dumpData( dump , q_ ) ;
//    dump = experienceName_ + "_dq_.dat" ;
//    dumpData( dump , dq_ ) ;
//    dump = experienceName_ + "_ddq_.dat" ;
//    dumpData( dump , ddq_ ) ;
//    dump = experienceName_ + "_torques_filter.dat" ;
//    dumpData( dump , torques_tmp ) ;
//    dump = experienceName_ + "_torques.dat" ;
//    dumpData( dump , torques_ ) ;
//    dump = experienceName_ + "_q_filter.dat" ;
//    dumpData( dump , q_tmp ) ;
//    dump = experienceName_ + "_q_ref.dat" ;
//    dumpData( dump , q_ref_ ) ;
//    dump = experienceName_ + "_q_astate.dat" ;
//    dumpData( dump , q_astate_ ) ;
    return 0;
}

int Experience::defineBeginEndIndexes()
{
    int N = data_astate_.size() ;
    unsigned int i = 0 ;
    bool found = false ;
    while ( !found && i < N )
    {
        int test = 0 ;
        for (unsigned int j = 0 ; j < ddl_ ; ++j )
            test += (abs(dq_[i][j]) > 5) ;
        if (test >= 6 )
            found = true ;
        else
            found = false ;

        beginData_ = i ;
        ++i;
    }
    //beginData_ = 0 ;
    if(i == N)
    {
        cout << "Failure to find the beginning of the motion\n" ;
        return -1 ;
    }

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
    endData_ = dq_.size()-1 ;
    cout << beginData_ << " " << endData_ << endl ;

    vector< vector<double> > tmp_torques = torques_ ;
    vector< vector<double> > tmp_q = q_ ;
    vector< vector<double> > tmp_dq = dq_ ;

    torques_.resize(endData_-beginData_);
    q_.resize(endData_-beginData_);
    dq_.resize(endData_-beginData_);

    vector< vector<double> > tmp_q_ref = q_ref_ ;
    vector< vector<double> > tmp_q_astate = q_astate_ ;

    q_ref_.resize(endData_-beginData_);
    q_astate_.resize(endData_-beginData_);
    for(unsigned int k = 0 ; k < torques_.size() ; ++k)
    {
        torques_[k] = tmp_torques[k+beginData_] ;
        q_[k] = tmp_q[k+beginData_] ;
        dq_[k] = tmp_dq[k+beginData_] ;

        q_ref_[k] = tmp_q_ref[k+beginData_] ;
        q_astate_[k] = tmp_q_astate[k+beginData_] ;
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

    EnergyOfMotor_J_m_s_ = 0.0;
    EnergyOfWalking_J_m_s_ = 0.0;
    double EnergyOfMotorDDL = 0.0;
    double EnergyOfWalkingDDL = 0.0;
    for (unsigned int j = 0 ; j < ddl_ ; ++j )
    {
        EnergyOfMotorDDL += energyOfMotors.back()[j] ;
        EnergyOfWalkingDDL += energyOfWalk.back()[j] ;
    }

    EnergyOfMotor_J_m_s_   = EnergyOfMotorDDL   /walkedDistanced_  /(energyOfMotors.size()*0.005) ;
    EnergyOfWalking_J_m_s_ = EnergyOfWalkingDDL /walkedDistanced_  /(energyOfMotors.size()*0.005) ;

    double MeanVelocity = (walkedDistanced_  /(energyOfMotors.size()*0.005));

    MechaCostOfTransport_ = EnergyOfMotorDDL / (WeightOfRobot_*MeanVelocity) ;
    CostOfTransport_ = EnergyOfWalkingDDL / (WeightOfRobot_*MeanVelocity) ;

    FroudeNumber_ = MeanVelocity / sqrt(Gravity_ * LegLenght_) ;

    return 0 ;
}

int Experience::compareRefMeasure()
{
    unsigned int N = q_ref_.size() ;
    errMeasureReference_.resize(N) ;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
        errMeasureReference_[i] = vector<double>( ddl_ , 0.0) ;
    }

    for (unsigned int i = 0 ; i < N ; ++i)
    {
        for (unsigned int j = 0 ; j < ddl_ ; ++j)
        {
            errMeasureReference_[i][j] = sqrt( (q_ref_[i][j]-q_astate_[i][j])*(q_ref_[i][j]-q_astate_[i][j]) );
        }
    }

    vector<double> avrg_joints (ddl_,0.0);
    for (unsigned int j = 0 ; j < ddl_ ; ++j)
    {
        avrg_joints[j] = 0.0 ;
        for (unsigned int i = 0 ; i < N ; ++i)
        {
            avrg_joints[j] += errMeasureReference_[i][j] ;
        }
        avrg_joints[j] = avrg_joints[j] / N ;
    }

    vector<double> variance_joints (ddl_,0.0);
    for (unsigned int j = 0 ; j < ddl_ ; ++j)
    {
        variance_joints[j] = 0.0 ;
        for (unsigned int i = 0 ; i < N ; ++i)
        {
            variance_joints[j] += (errMeasureReference_[i][j] - avrg_joints[j]) *
                                  (errMeasureReference_[i][j] - avrg_joints[j]) ;
        }
        variance_joints[j] / N ;
    }

    vector<double> fair_die_joints (ddl_,0.0);
    for (unsigned int j = 0 ; j < ddl_ ; ++j)
    {
        fair_die_joints[j] = sqrt(variance_joints[j]) ;
    }

//    cout << "average per joints :\n" ;
//    for (unsigned int j = 0 ; j < ddl_ ; ++j)
//    {
//        cout << avrg_joints[j] << " " ;
//    }
//    cout << endl ;

//    cout << "variance per joints :\n" ;
//    for (unsigned int j = 0 ; j < ddl_ ; ++j)
//    {
//        cout << variance_joints[j] << " " ;
//    }
//    cout << endl ;

//    cout << "fair die per joints :\n" ;
//    for (unsigned int j = 0 ; j < ddl_ ; ++j)
//    {
//        cout << fair_die_joints[j] << " " ;
//    }
//    cout << endl ;
}
