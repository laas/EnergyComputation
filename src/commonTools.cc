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

int derivation( const std::vector< std::vector<double> > & x,
                std::vector< std::vector<double> > & dx)
{
  double dt = 0.005 ;
  assert(x.size()!=0 && "No data to derive");
  assert(x[0].size()!=0 &&
         "Correct number of degree of freedom but no data to derive");
  dx .resize(x.size());
  unsigned rows = x.size();
  unsigned cols = x[0].size();
  for (unsigned int j = 0 ; j < cols ; ++j )
  {
    double x_1=x[0][j];
    dx[0][j]=0.0;
    for (unsigned int i = 1 ; i < rows ; ++i )
    {
      dx[i][j] = (x[i][j] - x_1)/dt ;
      x_1=x[i][j];
    }
  }
  return 0 ;
}

int integration( const std::vector< std::vector<double> > & dx,
                 std::vector< std::vector<double> > & x)
{
  double dt = 0.005 ;
  assert(dx.size()!=0 && "No data to integrate");
  assert(dx[0].size()!=0 &&
         "Correct number of degree of freedom but no data to integrate");
  unsigned rows = dx.size();
  unsigned cols = dx[0].size();
  x .resize(dx.size());
  for (unsigned int j = 0 ; j < cols ; ++j )
  {
    x[0][j]=0.0;
    for (unsigned int i = 1 ; i < rows ; ++i )
    {
      x[i][j] = x[i-1][j] + dx[i][j] * dt ;
    }
  }
  return 0 ;
}

int dumpData::dump(string &fileName, vector< vector<double> >& data)
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
#ifdef PINOCCHIO
int dumpData::dump(string &fileName, vector< se3::SE3 >& data)
{
  ofstream dumpStream ;
  dumpStream.open(fileName.c_str(),ofstream::out);
  int N = data.size()-1;
  for (unsigned int i = 0 ; i < N ; ++i)
  {
    this->dump(dumpStream,data[i].translation());
    this->dump(dumpStream,data[i].rotation());
    dumpStream << endl ;
  }
  dumpStream.close();
  cout << "dumped" << endl;
  return 0 ;
}
#endif
template<typename Matrix>
int dumpData::dump(ofstream &dumpStream, Matrix &data)
{
  for (unsigned int row = 0 ; row < data.rows() ; ++row)
    for (unsigned int col = 0 ; col < data.cols() ; ++col)
      dumpStream << data(row,col) << " " ;
  return 0 ;
}
#ifdef PINOCCHIO
int dumpData::dump(std::string &fileName,
                   std::vector<
                     Eigen::Matrix<double, 6, 1, 0, 6, 1>,
                     Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1, 0, 6, 1> >
                   > &data)
{
  ofstream dumpStream ;
  dumpStream.open(fileName.c_str(),ofstream::out);
  int N = data.size()-1;
  for (unsigned int i = 0 ; i < N ; ++i)
  {
    this->dump(dumpStream,data[i]);
    dumpStream << endl ;
  }
  dumpStream.close();
  cout << "dumped" << endl;
  return 0 ;
}
#endif
void getOptions(int argc,
                char *argv[],
                path_t & dataRootPath,
                path_t & robotRootPath,
#ifdef PINOCCHIO
                path_t & robotUrdfPath,
#endif
                path_t & outputFile)
{
  //std::cout << "argc:" << argc << std::endl;
#ifdef PINOCCHIO
  if (argc < 4)
  {
    cerr << " This program takes 1 arguments: " << endl;
    cerr << "./hrp2energy DATA_PATH ROBOT_FILE_PATH ROBOT_URDF_PATH" << endl;
    exit(-1);
  }
#else
  if (argc < 3)
  {
    cerr << " This program takes 1 arguments: " << endl;
    cerr << "./hrp2energy DATA_PATH ROBOT_FILE_PATH ROBOT_URDF_PATH" << endl;
    exit(-1);
  }
#endif
  else
  {
    dataRootPath=path_t(argv[1]);
    robotRootPath=path_t(argv[2]);
#ifdef PINOCCHIO
    robotUrdfPath=path_t(argv[3]);
#endif
    boost::gregorian::date d = boost::gregorian::day_clock::universal_day();
    ostringstream fileName ("") ;
    fileName << "."
             << "/results_"
             << d.year()
             << "_"
             << d.month()
             << "_" << d.day()
             << ".txt" ;
    outputFile = path_t(fileName.str());
    cout << outputFile.string() << endl ;
  }
}
