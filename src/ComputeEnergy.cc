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

    result_set_t * files = explore.files() ;
    vector<string> fileNames (files->size());
    vector<Experience> Xp ;

    result_set_t::iterator it_file = files->begin();
    for (unsigned int i = 0 ; i < files->size() ; ++i, ++it_file)
    {
        cout << it_file->string() << endl ;
        Experience tmp_Xp = Experience (&HRP2motors,*it_file,dataRootPath);
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