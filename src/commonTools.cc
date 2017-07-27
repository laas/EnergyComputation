#include "commonTools.hh"
using namespace std ;
namespace fs = boost::filesystem;


// Return RC low-pass filter output samples, given input samples,
// time interval dt, and time constant RC
int lowpass( double x, double & y, double i, double a)
{
    static double y_1 = 0.0 ;
    if (i==0)
    {
        y = x ;
        y_1 = y ;
        return 0;
    }
    else
    {
        y = a * x + (1-a) * y_1 ;
        y_1 = y ;
    }
    return 0 ;
}

int derivation( double x, double & dx, double i)
{
    static double x_1 = 0.0 ;
    if (i==0)
    {
        dx = 0.0 ;
        x_1 = x ;
        return 0;
    }
    else
    {
        double dt = 0.005 ;
        dx = (x - x_1)/dt ;
        x_1 = x ;
    }
    return 0 ;
}

int dumpData(string fileName, vector< vector<double> >& data)
{
    ofstream dumpStream ;
    dumpStream.open(fileName.c_str(),ofstream::out);
    int N = data.size()-1;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
        for (unsigned int j = 0 ; j < data[0].size() ; ++j)
        {
            dumpStream << data[i][j] << " " ;
        }
        dumpStream << endl ;
    }
    dumpStream.close();
    cout << "dumped" << endl;
    return 0 ;
}

void getOptions(int argc,
                char *argv[],
                path_t & dataRootPath,
                path_t & robotRootPath,
                path_t & outputFile)
{
  //std::cout << "argc:" << argc << std::endl;
  if (argc < 3)
  {
    cerr << " This program takes 1 arguments: " << endl;
    cerr << "./hrp2energy DATA_PATH ROBOT_FILE_FULL_PATH" << endl;
    exit(-1);
  }
  else
  {
    dataRootPath=path_t(argv[1]);
    robotRootPath=path_t(argv[2]);
    boost::gregorian::date d = boost::gregorian::day_clock::universal_day();
    ostringstream fileName ("") ;
    fileName << dataRootPath.string() << "/results_" << d.year() << "_" << d.month() << "_" << d.day() << ".txt" ;
    outputFile = path_t(fileName.str());
    cout << outputFile.string() << endl ;
  }
}
