#include "main.hh"

using namespace std ;
namespace fs = boost::filesystem;

void nice_dump(path_t outputFile, vector<Experience> & Xp)
{
  // open/create and output file
  ofstream dumpStream ;
  dumpStream.open(outputFile.c_str(),ofstream::out);
  // save the important values extracted from the experiment
  vector<string> names ;
  names.push_back("ExperienceName "            );
  names.push_back("WalkedDistance(m) "         );
  names.push_back("has fallen? "               );
  names.push_back("MaxTrackingError(rad)"      );
  names.push_back("DurationOfTheExperiment(s) ");
  names.push_back("EnergyOfMotors(J.m-1.s-1) " );
  names.push_back("EnergyOfWalking(J.m-1.s-1) ");
  names.push_back("CostOfTransport "           );
  names.push_back("MechaCostOfTransport "      );
  names.push_back("FroudeNumber "              );
  names.push_back("baseTrajLenght "            );
  int maxwidth=names[0].size() ;
  for (unsigned i=0 ; i<Xp.size() ; ++i)
  {maxwidth = max<int>(maxwidth,Xp[i].name().size());}
  maxwidth += 2 ;

  dumpStream << std::left
             << std::setw(maxwidth)
             << std::setfill(' ')
             << names[0] ;
  for (unsigned i=1 ; i<names.size() ; ++i)
  {
    dumpStream << std::left
               << std::setw(names[i].size())
               << std::setfill(' ')
               << names[i] ;
  }
  dumpStream << endl ;

  for (unsigned int i = 0 ; i < Xp.size() ; ++i)
  {
    unsigned index=0;
    dumpStream
        << std::left << std::setw(maxwidth)
        << std::setfill(' ') << Xp[i].name();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].walkedDistanced();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].hasFallen();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].maxTrackingError();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].TimeTravelled();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].EnergyOfMotor();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].EnergyOfWalking();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].CostOfTransport();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].MechaCostOfTransport();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].FroudeNumber();
    ++index ;
    dumpStream
        << std::left << std::setw(names[index].size())
        << std::setfill(' ') << Xp[i].baseTrajLenght();
    ++index ;
    dumpStream
        << endl ;
  }
  dumpStream.close();
  cout << "results dumped in " << outputFile.c_str() << endl;
  return ;
}

int main(int argc, char *argv[])
{
  path_t dataRootPath("");
  path_t robotRootPath("");
  path_t outputFile("");
  path_t robotUrdfPath("");
#ifdef PINOCCHIO
  getOptions(argc,argv,dataRootPath,robotRootPath,robotUrdfPath,outputFile);
#else
  getOptions(argc,argv,dataRootPath,robotRootPath/*,robotUrdfPath*/,outputFile);
#endif

#ifdef PINOCCHIO
  // create a Pinocchio model
  se3::Model robotModel ;
  se3::urdf::buildModel(
        robotUrdfPath.string(),
        se3::JointModelFreeFlyer(),
        robotModel);
#endif

  // create a motor model
  Motors HRP2motors (robotRootPath) ;

  // explore the folder architecture and find all files inside
  ExploreFolder explore ;
  explore.findLogInFolder(dataRootPath);

  // get all the files name
  vector<result_set_t> * files = explore.files() ;

  // create the set of experiments
  vector<Experience> Xp ;

  // iterate over all the file "*_astate.log" and "*_rstate.log"
  result_set_t::iterator it_file_astate = (*files)[0].begin();
  result_set_t::iterator it_file_ref = (*files)[1].begin();
  for (unsigned int i = 0 ; i < (*files)[0].size() ;
       ++i, ++it_file_astate, ++it_file_ref)
  {
    // print the file currently analyzed
    cout << "file " << i <<  " " << endl ;
    // create an "Experience" object for each log
    Experience tmp_Xp = Experience (&HRP2motors,
#ifdef PINOCCHIO
                                    &robotModel,
#endif
                                    *it_file_astate,
                                    *it_file_ref,
                                    dataRootPath);
    // treat the data log
    if (tmp_Xp.handleData()==-1)
    {
      std::cout<< "wrong computation of data, log ignored" << std::endl;
      continue;
    }
    // save the object in the "Xp" vector
    Xp.push_back(tmp_Xp);
    //break;
  }
  nice_dump(outputFile, Xp);
  return 0 ;
}
