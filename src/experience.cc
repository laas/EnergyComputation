#include "experience.hh"
#ifdef PINOCCHIO
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#endif
using namespace std ;

Experience::Experience(Motors * hrp2motors,
#ifdef PINOCCHIO
                       se3::Model * hrp2model,
#endif
                       path_t input_state_path,
                       path_t input_ref_path,
                       path_t rootFolder)
{
  input_astate_path_ = input_state_path ;
  input_ref_path_ = input_ref_path ;
  hrp2motors_ = hrp2motors ;
#ifdef PINOCCHIO
  robotModel_ = hrp2model ;
  robotData_ = new se3::Data(*robotModel_);
  q_odo_.resize(robotModel_->nq);
  dq_odo_.resize(robotModel_->nv);

  jac_lf_.resize(6,robotModel_->nv);
  jac_rf_.resize(6,robotModel_->nv);
#endif
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
  LegLenght_ = 0.605 ; // m //TO BE VERIFIED
  Gravity_ = 9.81 ;  // m.s-2

  titleRobotConfig_.clear();

  q_.clear() ;
  dq_.clear() ;
  torques_.clear() ;
  q_ref_.clear();
  q_astate_.clear();

  avrg_joints_    .clear();
  variance_joints_.clear();
  fair_die_joints_.clear();
  fall_fifo_      .clear();

  ignore_ref_ = false ;
  has_fallen_ = false ;
  maxTrackingError_ = 0.0 ;

  setExperienceName(rootFolder);
#ifdef PINOCCHIO
  left_foot_wrench_ .clear();
  right_foot_wrench_.clear();
  left_hand_wrench_ .clear();
  right_hand_wrench_.clear();
  left_foot_isInContact_ .clear();
  right_foot_isInContact_.clear();
#endif
}

int Experience::setExperienceName(path_t rootFolder)
{
  string rootFolder_st = rootFolder.string();
  string input_path_st = input_astate_path_.string();

  experienceName_ = input_path_st.replace(0,rootFolder_st.length(),"") ;

  string asate_log = "-astate.log" ;
  size_t found = experienceName_.find(asate_log);
  experienceName_ = experienceName_.replace(found,asate_log.length(),"") ;

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
  if(experienceName_.compare(0,1,"_")==0)
  {
    experienceName_.erase(0,1);
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

  astring = "ClimbingWithTools" ; //stairs 15cm up tools
  found = experienceName_.find(astring);
  if ( found != std::string::npos)
    walkedDistanced_ = 0.9 ;

  astring = "StepStairsDownSeq" ; // stairs 15cm
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
    walkedDistanced_ = 7.0 ; //approximately

  return 0 ;
}

int Experience::handleData()
{
  readData();
  if (defineBeginEndIndexes() == -1)
  {
    return -1;
  }
  if (compareRefMeasure() == -1)
  {
    has_fallen_ = true ;
//    return -1 ;
  }
#ifdef PINOCCHIO
  odometrie();
#endif
  computeTheEnergy();

  return 0 ;
}

int Experience::readFile(std::vector< std::vector<double> > & data,
                         path_t file, int number_of_column)
{
  cout << "reading :" << file << endl ;
  // clean the data
  data.clear();
  // open the file
  std::ifstream dataStream ;
  dataStream.open(file.c_str(),std::ifstream::in);
  // reading all the data file
  vector<double> oneLine ;
  string line ;
  double value ;
  int count_line = 0 ;
  while (getline(dataStream,line)){
    ++count_line ;
    if (line.find("%JA0 JA1 JA2 JA3 JA4 JA5 JA6 JA7")!= std::string::npos)
    {
      //        cout << "found \"%JA0 JA1 JA2 JA3 JA4 JA5 JA6 JA7\", in line "
      //             << count_line << ", line ignored" << endl ;
      line.clear();
      continue ;
    }
    oneLine.clear();
    istringstream line_stream(line);
    while(line_stream >> value)
    {
      oneLine.push_back(value);
    }
    //      if(oneLine.size() != number_of_column)
    //      {
    //        cout << "number of column " << oneLine.size()
    //             << " of line " << count_line
    //             << " is not standard. It should be " << number_of_column << endl ;
    //        continue ;
    //      }
    data.push_back(oneLine);
    line.clear();
  }
  // close the file
  dataStream.close();

  bool same_number_column = true;
  for(unsigned row=0 ; row< data.size() ; ++row)
  {
    same_number_column &= (data[row].size() == number_of_column);
    if (!same_number_column)
    {
      //      cout << data[row].size() << " " << number_of_column << endl ;
      //      for (unsigned col=0 ; col<data[row].size() ; ++col)
      //        cout << "col " << col << " : " << data[row][col] << endl;
      break ;
      //assert (same_number_column);
    }
  }

}

int Experience::readData()
{
  readFile(data_astate_,input_astate_path_,176);
  readFile(data_ref_,input_ref_path_,100);

  if ( data_astate_.size() != data_ref_.size() )
  {
    cout << "data_astate_.size():" << data_astate_.size()
         << " ; data_ref_.size():" << data_ref_.size() << endl ;
    ignore_ref_ = true ;
    std::cout << "-rstate and -astate files does not have the same number of"
              << " data" << std::endl;
  }
  assert(data_astate_.size() == data_ref_.size());

  // save the data wanted in separated buffers for computation
  int N = data_astate_.size() ;
  q_                .resize( N , vector<double>(ddl_,0.0) ) ;
  q_ref_            .resize( N , vector<double>(ddl_,0.0) ) ;
  q_astate_         .resize( N , vector<double>(ddl_,0.0) ) ;
  dq_               .resize( N , vector<double>(ddl_,0.0) ) ;
  ddq_              .resize( N , vector<double>(ddl_,0.0) ) ;
  torques_          .resize( N , vector<double>(ddl_,0.0) ) ;
  rnea_torques_     .resize( N , vector<double>(ddl_,0.0) ) ;
#ifdef PINOCCHIO
  left_foot_wrench_      .resize( N , vector<double>(6,0.0) ) ;
  right_foot_wrench_     .resize( N , vector<double>(6,0.0) ) ;
  left_hand_wrench_      .resize( N , vector<double>(6,0.0) ) ;
  right_hand_wrench_     .resize( N , vector<double>(6,0.0) ) ;
  left_foot_isInContact_ .resize( N , 0.0 ) ;
  right_foot_isInContact_.resize( N , 0.0 ) ;
#endif
  for (unsigned int i = 0 ; i < N ; ++i )
  {
    int index=0 ;
    for (unsigned int j = index ; j < index+30 ; ++j )
    {
      q_[i][j-index] = hrp2motors_->gear_ratio_[j] * data_astate_[i][j];
    }
    index=0;
    for (unsigned int j = index ; j < index+30 ; ++j )
    {
      q_astate_[i][j-index] = data_astate_[i][j];
      if(!ignore_ref_)
        q_ref_[i][j-index] = data_ref_[i][j];
    }
    index=40;
    for (unsigned int j = index ; j < index+30 ; ++j )
    {
      torques_[i][j-index] =
          data_astate_[i][j] / hrp2motors_->gear_ratio_[j-index];
    }
#ifdef PINOCCHIO
    index=120+0*6;
    for (unsigned int j = index ; j < index+6 ; ++j )
    {
      left_foot_wrench_[i][j-index] = data_astate_[i][j];
    }
    index=120+1*6;
    for (unsigned int j = index ; j < index+6 ; ++j )
    {
      right_foot_wrench_[i][j-index] = data_astate_[i][j];
    }
    index=120+2*6;
    for (unsigned int j = index ; j < index+6 ; ++j )
    {
      left_hand_wrench_[i][j-index] = data_astate_[i][j];
    }
    index=120+3*6;
    for (unsigned int j = index ; j < index+6 ; ++j )
    {
      right_hand_wrench_[i][j-index] = data_astate_[i][j];
    }
#endif
  }
  // Filter q, torques, external wrench with the same filter
  std::vector< std::vector<double> > q_tmp = q_ ;
  IIF.filter(q_tmp,q_);
  std::vector< std::vector<double> > torques_tmp = torques_ ;
  IIF.filter(torques_tmp,torques_);

#ifdef PINOCCHIO
  std::vector< std::vector<double> > left_foot_wrench_tmp = left_foot_wrench_ ;
  std::vector< std::vector<double> > right_foot_wrench_tmp = right_foot_wrench_ ;
  std::vector< std::vector<double> > left_hand_wrench_tmp = left_hand_wrench_ ;
  std::vector< std::vector<double> > right_hand_wrench_tmp = right_hand_wrench_ ;
  IIF.filter(left_foot_wrench_tmp,left_foot_wrench_);
  IIF.filter(right_foot_wrench_tmp,right_foot_wrench_);
  IIF.filter(left_hand_wrench_tmp,left_hand_wrench_);
  IIF.filter(right_hand_wrench_tmp,right_hand_wrench_);
#endif
  // compute the velocity of the joints by finite differentiation
  derivation(q_,dq_);
  // filter the velocity
  std::vector< std::vector<double> > dq_tmp = dq_ ;
  IIF.filter(dq_tmp,dq_);
  dq_tmp = dq_ ;
  IIF.filter(dq_tmp,dq_);

  // compute the acceleration of the joints by finite differentiation
  derivation(dq_,ddq_);
  // filter the acceleration
  std::vector< std::vector<double> > ddq_tmp = ddq_ ;
  IIF.filter(ddq_tmp,ddq_);
  // filter the acceleration
  ddq_tmp = ddq_ ;
  IIF.filter(ddq_tmp,ddq_);

  // dump all the files
  string dump ;
  dump = experienceName_+"_q.dat" ;
  dump_.dump( dump , q_ ) ;

  dump = experienceName_ + "_dq.dat" ;
  dump_.dump( dump , dq_ ) ;

  dump = experienceName_ + "_ddq.dat" ;
  dump_.dump( dump , ddq_ ) ;

  dump = experienceName_ + "_torques_raw.dat" ;
  dump_.dump( dump , torques_tmp ) ;

  dump = experienceName_ + "_torques.dat" ;
  dump_.dump( dump , torques_ ) ;

  dump = experienceName_ + "_q_raw.dat" ;
  dump_.dump( dump , q_tmp ) ;

  dump = experienceName_ + "_q_ref.dat" ;
  dump_.dump( dump , q_ref_ ) ;

  dump = experienceName_ + "_q_astate.dat" ;
  dump_.dump( dump , q_astate_ ) ;

  dump = experienceName_+"_left_foot_wrench.dat" ;
  dump_.dump( dump , left_foot_wrench_   ) ;

  dump = experienceName_+"_left_foot_wrench_raw.dat" ;
  dump_.dump( dump , left_foot_wrench_tmp   ) ;

  dump = experienceName_+"_right_foot_wrench.dat" ;
  dump_.dump( dump , right_foot_wrench_   ) ;

  dump = experienceName_+"_right_foot_wrench_raw.dat" ;
  dump_.dump( dump , right_foot_wrench_tmp   ) ;

  data_astate_.clear();
  data_ref_.clear();
  return 0;
}

int Experience::defineBeginEndIndexes()
{
  int N = dq_.size() ;
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
  cout << "BeginDataIndex : " << beginData_
       << " ; EndingDataIndex : " << endData_ << endl ;

  vector< vector<double> > tmp_torques = torques_ ;
  vector< vector<double> > tmp_q = q_ ;
  vector< vector<double> > tmp_dq = dq_ ;
#ifdef PINOCCHIO
  vector< vector<double> > tmp_left_foot_wrench  = left_foot_wrench_  ;
  vector< vector<double> > tmp_right_foot_wrench = right_foot_wrench_ ;
  vector< vector<double> > tmp_left_hand_wrench  = left_hand_wrench_  ;
  vector< vector<double> > tmp_right_hand_wrench = right_hand_wrench_ ;
#endif
  torques_          .resize(endData_-beginData_);
  q_                .resize(endData_-beginData_);
  dq_               .resize(endData_-beginData_);
#ifdef PINOCCHIO
  left_foot_wrench_ .resize(endData_-beginData_);
  right_foot_wrench_.resize(endData_-beginData_);
  left_hand_wrench_ .resize(endData_-beginData_);
  right_hand_wrench_.resize(endData_-beginData_);
#endif
  vector< vector<double> > tmp_q_ref = q_ref_ ;
  vector< vector<double> > tmp_q_astate = q_astate_ ;

  if(!ignore_ref_)
    q_ref_.resize(endData_-beginData_);
  q_astate_.resize(endData_-beginData_);
  for(unsigned int k = 0 ; k < torques_.size() ; ++k)
  {
    torques_[k] = tmp_torques[k+beginData_] ;
    q_[k] = tmp_q[k+beginData_] ;
    dq_[k] = tmp_dq[k+beginData_] ;
    if(!ignore_ref_)
      q_ref_[k] = tmp_q_ref[k+beginData_] ;
    q_astate_[k] = tmp_q_astate[k+beginData_] ;
#ifdef PINOCCHIO
    left_foot_wrench_ [k] = tmp_left_foot_wrench [k+beginData_];
    right_foot_wrench_[k] = tmp_right_foot_wrench[k+beginData_];
    left_hand_wrench_ [k] = tmp_left_hand_wrench [k+beginData_];
    right_hand_wrench_[k] = tmp_right_hand_wrench[k+beginData_];
#endif
  }
  // dump all the files
  string dump ;
  dump = experienceName_+"_q_cut.dat" ;
  dump_.dump( dump , q_ ) ;
  dump = experienceName_ + "_dq_cut.dat" ;
  dump_.dump( dump , dq_ ) ;
  dump = experienceName_ + "_torques_cut.dat" ;
  dump_.dump( dump , torques_ ) ;
#ifdef PINOCCHIO
  dump = experienceName_+"_left_foot_wrench_cut.dat" ;
  dump_.dump( dump , left_foot_wrench_   ) ;
  dump = experienceName_+"_right_foot_wrench_cut.dat" ;
  dump_.dump( dump , right_foot_wrench_   ) ;
#endif
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
      powerOfWalk_[i][j] =
          hrp2motors_->R_K2[j] * torques_[i][j] * torques_[i][j] +
          powerOutputMotors_[i][j] ;
    }
  }

  vector< vector<double> > energyOfMotors ( N , vector<double> (ddl_,0) );
  integration(powerOutputMotors_,energyOfMotors);
  vector< vector<double> > energyOfWalk ( N , vector<double> (ddl_,0) );
  integration(powerOfWalk_,energyOfWalk);

  EnergyOfMotor_J_m_s_ = 0.0;
  EnergyOfWalking_J_m_s_ = 0.0;
  double EnergyOfMotorDDL = 0.0;
  double EnergyOfWalkingDDL = 0.0;
  for (unsigned int j = 0 ; j < ddl_ ; ++j )
  {
    EnergyOfMotorDDL += energyOfMotors.back()[j] ;
    EnergyOfWalkingDDL += energyOfWalk.back()[j] ;
  }

  EnergyOfMotor_J_m_s_   = EnergyOfMotorDDL   /walkedDistanced_
                           /(energyOfMotors.size()*0.005) ;
  EnergyOfWalking_J_m_s_ = EnergyOfWalkingDDL /walkedDistanced_
                           /(energyOfMotors.size()*0.005) ;

  double MeanVelocity = (walkedDistanced_  /(energyOfMotors.size()*0.005));

  MechaCostOfTransport_ = EnergyOfMotorDDL / (WeightOfRobot_*MeanVelocity) ;
  CostOfTransport_ = EnergyOfWalkingDDL / (WeightOfRobot_*MeanVelocity) ;

  FroudeNumber_ = MeanVelocity / sqrt(Gravity_ * LegLenght_) ;

  return 0 ;
}

int Experience::compareRefMeasure()
{
  if(ignore_ref_)
    return 0 ;

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
      errMeasureReference_[i][j] = sqrt( (q_ref_[i][j]-q_astate_[i][j])*
                                         (q_ref_[i][j]-q_astate_[i][j]) );
    }
  }

  avrg_joints_.resize(ddl_,0.0);
  for (unsigned int j = 0 ; j < ddl_ ; ++j)
  {
    avrg_joints_[j] = 0.0 ;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
      avrg_joints_[j] += errMeasureReference_[i][j] ;
    }
    avrg_joints_[j] = avrg_joints_[j] / N ;
  }

  variance_joints_.resize(ddl_,0.0);
  for (unsigned int j = 0 ; j < ddl_ ; ++j)
  {
    variance_joints_[j] = 0.0 ;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
      variance_joints_[j] += (errMeasureReference_[i][j] - avrg_joints_[j]) *
                             (errMeasureReference_[i][j] - avrg_joints_[j]) ;
    }
    variance_joints_[j] / N ;
  }

  fair_die_joints_.resize(ddl_,0.0);
  for (unsigned int j = 0 ; j < ddl_ ; ++j)
  {
    fair_die_joints_[j] = sqrt(variance_joints_[j]) ;
  }
  return detectFall() ;
}

int Experience::detectFall()
{
  int ret = 0 ;
  maxTrackingError_ = 0.0 ;
  double time_max_error = 0.0 ;
  unsigned int N = q_ref_.size() ;
  fall_fifo_.resize( (int)round(0.1/0.005) , vector<double>(ddl_,0.0) );
  for (unsigned i=0 ; i<fall_fifo_.size()&i<errMeasureReference_.size() ; ++i )
  {
    for (unsigned int j = 0 ; j < ddl_ ; ++j)
    {
      fall_fifo_[i][j] = errMeasureReference_[i][j];
    }
  }

//  cout << "err sum per ddl : " ;
  double avrg_err_window = 0.0 ;
  for(int n=0 ; n<N-fall_fifo_.size() ; ++n)
  {
    // update the fifo :
    if(n!=0)
    {
      fall_fifo_.pop_front();
      if(n+fall_fifo_.size()<=errMeasureReference_.size())
        fall_fifo_.push_back(errMeasureReference_[n+fall_fifo_.size()-1]);
    }
    // compute the sum of the error per time and per ddl
    double err_sum = 0.0 ;
    for (unsigned i=0 ; i<fall_fifo_.size() ; ++i)
      for (unsigned j=0 ; j<ddl_ ; ++j)
        err_sum += fall_fifo_[i][j] ;

    avrg_err_window = err_sum/(fall_fifo_.size()*ddl_) ;
    if (maxTrackingError_ < avrg_err_window)
    {
      maxTrackingError_ = avrg_err_window ;
      time_max_error = (beginData_+n)*0.005 ;
    }
    //assert(avrg_err_window<0.024);
  }

  if (maxTrackingError_>0.007)
  {
//    cout << "max_error = " << max_error
//         << " between time=[" << time_max_error << ", "
//         << time_max_error + fall_fifo_.size()*0.005  << "]" << endl ;
//    cout << "################################################################"
//         << "################################################################"
//         << endl ;
    ret = -1 ;
    //break ;
  }
//  cout << "final err sum = " << avrg_err_window << endl ;
//  cout << "final max err = " << max_error << endl ;
//  cout << "return = " << ret << endl ;
  return ret ;
}

#ifdef PINOCCHIO
int Experience::contact_detection(const std::vector< std::vector<double> > & wrench,
                                  std::vector< int > & isInContact)
{
  isInContact.resize(wrench.size());
  // detect if foot is in contact :
  double mg = WeightOfRobot_;
  // Fz > 25% RobotWeight N
  isInContact[0] = wrench[0][2] > 40.0/100.0 * mg ;
  for (unsigned n=1 ; n<isInContact.size() ;++n)
  {
    double evolution = wrench[n][2]-wrench[n-1][2];
    if (isInContact[n-1] == 1)
    {
      if (evolution < 0 && wrench[n][2] < 40.0/100.0 * mg)
      {
        isInContact[n] = 0 ;
      }else{
        isInContact[n] = 1 ;
      }
    }else if (isInContact[n-1] == 0){
      if (evolution > 0 && wrench[n][2] > 25.0/100.0 * mg)
      {
        isInContact[n] = 1 ;
      }else{
        isInContact[n] = 0 ;
      }
    }
    else{
      isInContact[n] = 0 ;
    }
//    std::cout << wrench[n][2] << " "
//              << isInContact [n] << " "
//              << std::endl ;
  }
  return 0;
}

Eigen::VectorXd Experience::fromVrmlMotor2PinocchioJoint(std::vector<double> vrml)
{
  assert (vrml.size() == hrp2motors_->joint_name_.size());
  Eigen::VectorXd pin ;
  pin.resize(hrp2motors_->joint_name_.size());
  for(unsigned i=0 ; i<hrp2motors_->joint_name_.size() ; ++i)
  {
    se3::JointIndex pin_index = robotModel_->getJointId(
                                  hrp2motors_->joint_name_[i]) - 2 ;
    pin(pin_index) = vrml[i] / hrp2motors_->gear_ratio_[i];
  }
  return pin ;
}

int Experience::odometrie()
{
  unsigned int N = q_ref_.size() ;
  // detect if foot is in contact :
  contact_detection(right_foot_wrench_,right_foot_isInContact_);
  contact_detection(left_foot_wrench_,left_foot_isInContact_);
  string dump2 ;
  dump2 = experienceName_+"_lf_isInContact_cut.dat" ;
  dump_.dump( dump2, left_foot_isInContact_);
  dump2 = experienceName_+"_rf_isInContact_cut.dat" ;
  dump_.dump( dump2 , right_foot_isInContact_);

  // reset config vectors
  world_M_base_.resize(N);
  world_V_base_.resize(N);
  q_odo_.setZero();  jac_lf_.setZero();
  dq_odo_.setZero(); jac_rf_.setZero();
  // setup some constants
  se3::Model::FrameIndex lf_id = robotModel_->getFrameId("l_ankle");
  se3::Model::FrameIndex rf_id = robotModel_->getFrameId("r_ankle");
  Eigen::Matrix<double,6,1> world_V_lf_base, world_V_rf_base ;
  double fz_rf = 0.0 ;
  double fz_lf = 0.0 ;
  world_M_base_[0] = se3::SE3::Identity() ;
  world_V_base_[0].setZero();
  // compute the velocity of the feet on the base frame
  for(unsigned n=1 ; n<N ; ++n)
  {
    // update the configuration
    q_odo_.segment(7,ddl_) = fromVrmlMotor2PinocchioJoint(q_[n]);
    q_odo_.head<3>() = world_M_base_[n-1].translation();
    q_odo_.segment<4>(3) =
        Eigen::Quaternion<double>(world_M_base_[n-1].rotation()).coeffs() ;
    // update the velocity
    dq_odo_.head<6>().fill(0.) ;
    dq_odo_.segment(6,ddl_) = fromVrmlMotor2PinocchioJoint(dq_[n]);
    // update the z forces
    fz_rf = right_foot_wrench_[n][2] ;
    fz_lf = right_foot_wrench_[n][2] ;
    // compute the jacobians
    se3::framesForwardKinematics(*robotModel_,*robotData_,q_odo_);
    se3::computeJacobians(*robotModel_,*robotData_,q_odo_) ;
    se3::getFrameJacobian<false>(*robotModel_,*robotData_,lf_id,jac_lf_);
    se3::getFrameJacobian<false>(*robotModel_,*robotData_,rf_id,jac_rf_);
    se3::FrameIndex lf_id = robotModel_->getFrameId("l_ankle");
    se3::FrameIndex rf_id = robotModel_->getFrameId("r_ankle");
    se3::SE3 world_M_lf_ = robotData_->oMf[lf_id] ;
    se3::SE3 world_M_rf_ = robotData_->oMf[rf_id] ;
    // compute the velocity of the base relatif to the stance foot
    world_V_lf_base = /*world_M_lf_.toActionMatrix() * */ (- jac_lf_ * dq_odo_) ;
    world_V_rf_base = /*world_M_rf_.toActionMatrix() * */ (- jac_rf_ * dq_odo_) ;

    if (left_foot_isInContact_[n] == 1)
    {
      //cout << "left support" << endl ;
      world_V_base_[n] = world_V_lf_base ;
    }
    else if (right_foot_isInContact_[n] == 1)
    {
      //cout << "right support" << endl ;
      world_V_base_[n] = world_V_rf_base ;
    }else
    {
      //cout << "ARGH NO SUPPORT" << endl ;
      world_V_base_[n].setZero() ;
    }
    world_M_base_[n] = se3::exp6(world_V_base_[n]*0.005).act(world_M_base_[n-1]) ;
  }
  string dump ;
  dump = experienceName_+"_world_V_base.dat" ;
  dump_.dump( dump , world_V_base_ ) ;
  dump = experienceName_+"_world_M_base.dat" ;
  dump_.dump( dump , world_M_base_ ) ;

//  world_V_base_filtered_ = world_V_base_ ;
//  IIF.filter(world_V_base_,world_V_base_filtered_);
//  dump = experienceName_+"_world_V_basef.dat" ;
//  dump_.dump( dump , world_V_base_filtered_ ) ;
//  for(unsigned n=1 ; n<N ; ++n)
//  {
//    // update the configuration
//    for (unsigned ddl=0 ; ddl<ddl_ ; ++ddl)
//    {
//      q_odo_(7+ddl) = q_[n][ddl] ;
//    }
//    q_odo_.head<3>() = world_M_base_[n-1].translation();
////    q_odo_.segment<4>(3) =
////        Eigen::Quaternion<double>(world_M_base_[n-1].rotation()).coeffs() ;
////    dq_odo_.head<6>() = world_V_base_filtered_[n] ;
//    for (unsigned ddl=0 ; ddl<ddl_ ; ++ddl)
//    {
//      dq_odo_(6+ddl) = dq_[n][ddl] ;
//    }
//    Eigen::VectorXd q = se3::integrate(*robotModel_,q_odo_,dq_odo_*0.005);
//    Eigen::Vector3d p = q.head<3>() ;
//    Eigen::Matrix3d R = Eigen::Quaternion<double>(q.segment<4>(3))
//                          .toRotationMatrix() ;
//    world_M_base_[n].translation(p);
//    world_M_base_[n].rotation(R);
//    cout << world_V_base_filtered_[n].transpose() << endl ;
//    //cout << world_M_base_[n] << endl ;
//  }

  return 0;
}
#endif
