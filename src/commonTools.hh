#ifndef COMMONTOOLS_HH
#define COMMONTOOLS_HH

#ifdef PINOCCHIO
#include "pinocchio/parsers/urdf.hpp"
#endif
#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <sstream>
#include <fstream>
#include <cmath>

typedef boost::filesystem::path path_t;
typedef std::list<path_t> result_set_t;
#ifdef PINOCCHIO
typedef Eigen::Matrix<double,6,1> Vector6d ;
#endif
int lowpass(double x, double &y, double i, double a);
int derivation(const std::vector<std::vector<double> > &x,
               std::vector<std::vector<double> > &dx);
int integration( const std::vector< std::vector<double> > & dx,
                 std::vector< std::vector<double> > & x);

struct dumpData{
  int dump(std::string &fileName, std::vector< std::vector<double> >& data);
  int dump(std::string &fileName, std::vector< int >& data);
#ifdef PINOCCHIO
  int dump(std::string &fileName, std::vector< se3::SE3 >& data) ;
  int dump(std::string &fileName,
           std::vector<Eigen::MatrixXd,
                       Eigen::aligned_allocator<Eigen::MatrixXd> > &data) ;
  int dump(std::string &fileName,
           std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > &data) ;
#endif
  template<typename Matrix>
  int dump(std::ofstream &dumpStream, Matrix & data) ;
};

void getOptions(int argc,
                char *argv[],
                path_t &dataRootPath,
                path_t &robotRootPath,
#ifdef PINOCCHIO
                path_t & robotUrdfPath,
#endif
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

  void filter(const std::vector< std::vector<double> > & in,
              std::vector< std::vector<double> > & out)
  {
    out = in ;
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
    std::vector< std::vector<double> > out_shifted = out ;
    shift_trajectory(out_shifted,out,-5);
  }

  void shift_trajectory(const std::vector< std::vector<double> > & in,
                        std::vector< std::vector<double> > & out,
                        int shift=0)
  {
    out = in ;
    shift *= -1 ;
    for(unsigned j=0 ; j<out[0].size() ; ++j)
    {
      for(int i=0 ; i<out.size() ; ++i)
      {
        int is = i + shift;
        if (is < 0)
          is = 0;
        if (is >= out.size())
          is = out.size() - 1 ;

        out[i] = in[is] ;
      }
    }
  }

#ifdef PINOCCHIO
  void filter(std::vector< Eigen::Matrix<double,6,1> ,
                           Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > > & in,
              std::vector< Eigen::Matrix<double,6,1> ,
                           Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > > & out)
  {
    std::vector< std::vector<double> > in_vec ( in.size() , std::vector<double>(6,0.0) );
    std::vector< std::vector<double> > out_vec ( out.size() , std::vector<double>(6,0.0) );
    for(unsigned i=0 ; i<in.size() ; ++i)
    {
      for(unsigned j=0 ; j<6 ; ++j)
        in_vec[i][j] = in[i](j) ;
    }
    this->filter(in_vec,out_vec);
    for(unsigned i=0 ; i<in.size() ; ++i)
    {
      for(unsigned j=0 ; j<6 ; ++j)
        out[i](j) = out_vec[i][j] ;
    }
    return ;
  }
  #endif
private :
  std::vector<double> m_filterWindow;
  double m_SamplingPeriod ;
};

#endif // COMMONTOOLS_HH
