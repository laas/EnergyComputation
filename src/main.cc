#include "main.hh"

using namespace std ;
namespace fs = boost::filesystem;

int main(int argc, char *argv[])
{
    path_t dataRootPath("");
    path_t robotRootPath("");
    path_t outputFile("");
    path_t robotUrdfPath("");
    getOptions(argc,argv,dataRootPath,robotRootPath,robotUrdfPath,outputFile);

    // create a pionocchio model
    se3::Model robotModel ;
    se3::urdf::buildModel(robotUrdfPath.string(),
                          se3::JointModelFreeFlyer(),
                          robotModel);

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
        cout << "file " << i <<  " " ;
        cout << it_file_astate->string() << endl ;
        // create an "Experience" object for each log
        Experience tmp_Xp = Experience (&HRP2motors,
                                        &robotModel,
                                        *it_file_astate,
                                        *it_file_ref,
                                        dataRootPath);
        // treat the data log
        tmp_Xp.handleData();
        // save the object in the "Xp" vector
        Xp.push_back(tmp_Xp);
    }

    // open/create and output file
    ofstream dumpStream ;
    dumpStream.open(outputFile.c_str(),ofstream::out);
    // save the important values extracted from the experiment
    dumpStream << "ExperienceName\tWalkedDistance\tEnergyOfMotors\tEnergyOfWalking\tDurationOfTheExperiment\n" ;
    for (unsigned int i = 0 ; i < Xp.size() ; ++i)
    {
        dumpStream << Xp[i].name()            << "\t"
                   << Xp[i].walkedDistanced() << "\t"
                   << Xp[i].EnergyOfMotor()   << "\t"
                   << Xp[i].EnergyOfWalking() << "\t"
                   << Xp[i].TimeTravelled()   << endl ;
    }
    dumpStream.close();
    cout << "dumped" << endl;
    return 0 ;
}
