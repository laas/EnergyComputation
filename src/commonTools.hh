#ifndef COMMONTOOLS_HH
#define COMMONTOOLS_HH
//#include "pinocchio/parsers/urdf.hpp"
#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <sstream>
#include <fstream>
#include <cmath>

typedef boost::filesystem::path path_t;
typedef std::list<path_t> result_set_t;

int lowpass(double x, double &y, double i, double a);
int derivation(const std::vector<std::vector<double> > &x,
               std::vector<std::vector<double> > &dx);
int integration( const std::vector< std::vector<double> > & dx,
                std::vector< std::vector<double> > & x);
int dumpData(std::string fileName, std::vector< std::vector<double> >& data);

void getOptions(int argc,
                char *argv[],
                path_t &dataRootPath,
                path_t &robotRootPath, //path_t &robotUrdfPath,
                path_t &outputFile);


class InfiniteImpendanceFilter
{
public :
  InfiniteImpendanceFilter()
  {
    // Create the window for the filter.
    m_SamplingPeriod = 0.005 ;
    double T=0.05; // Arbritrary from Kajita's San Matlab files.
    int n=0;
    double sum=0,tmp=0;

    assert(m_SamplingPeriod > 0);
    n = (int)floor(T/m_SamplingPeriod);
    m_filterWindow.resize(n+1);
    for(int i=0;i<n+1;i++)
      {
        tmp =sin((M_PI*i)/n);
        m_filterWindow[i]=tmp*tmp;
      }

    for(int i=0;i<n+1;i++)
      sum+= m_filterWindow[i];

    for(int i=0;i<n+1;i++)
      m_filterWindow[i]/= sum;
  }

  void filter(std::vector< std::vector<double> > & in,
              std::vector< std::vector<double> > & out)
  {
    for(unsigned j=0 ; j<in[0].size() ; ++j)
    {
      int n=0;
      double T=0.050; // Arbritraty fixed from Kajita's San matlab files.

      assert(out.size()==in.size()&&out[0].size()==in[0].size());

      // Creates window.
      assert(m_SamplingPeriod > 0);
      n = (int)floor(T/m_SamplingPeriod);

      // Filter vector.
      for(int i=0;i<n+1;i++)
      {
        out[i][j] = in[i][j];
      }

      for(unsigned int i=n+1;i<in.size();i++)
      {
        double ltmp = 0.0 ;
        for(unsigned int k=0;k<m_filterWindow.size();k++)
        {
          ltmp += m_filterWindow[k]*in[i-k][j];
        }
        out[i][j] = ltmp ;
      }
    }
  }
private :
  std::vector<double> m_filterWindow;
  double m_SamplingPeriod ;
};

#endif // COMMONTOOLS_HH
