#include "ComputeEnergy.hh"

using namespace std ;
namespace fs = boost::filesystem;

int main(int argc, char *argv[])
{
    path_t dataRootPath("");
    path_t outputFile("");
    getOptions(argc,argv,dataRootPath,outputFile);

    Motors HRP2motors ;
    ExploreFolder explore ;
    explore.findLogInFolder(dataRootPath);

    vector<result_set_t> * files = explore.files() ;

    vector<Experience> Xp ;

    result_set_t::iterator it_file_astate = (*files)[0].begin();
    result_set_t::iterator it_file_ref = (*files)[1].begin();
    for (unsigned int i = 0 ; i < (*files)[0].size() ; ++i, ++it_file_astate, ++it_file_ref)
    {
        cout << "file " << i <<  " " ;
        cout << it_file_astate->string() << endl ;

        Experience tmp_Xp = Experience (&HRP2motors,*it_file_astate, *it_file_ref,dataRootPath);
        tmp_Xp.handleData();
        Xp.push_back(tmp_Xp);
    }

    ofstream dumpStream ;
    dumpStream.open(outputFile.c_str(),ofstream::out);

    dumpStream << "ExperienceName\tWalkedDistance\tEnergyOfMotors\tEnergyOfWalking\n" ;
    for (unsigned int i = 0 ; i < Xp.size() ; ++i)
    {
        dumpStream << Xp[i].name() << "\t"
                   << Xp[i].walkedDistanced() << "\t"
                   << Xp[i].EnergyOfMotor() << "\t"
                   << Xp[i].EnergyOfWalking() << endl ;
    }
    dumpStream.close();
    cout << "dumped" << endl;
    return 0 ;
}