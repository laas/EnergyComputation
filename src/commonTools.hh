#ifndef COMMONTOOLS_HH
#define COMMONTOOLS_HH

#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <sstream>
#include <fstream>
#include <cmath>

typedef boost::filesystem::path path_t;
typedef std::list<path_t> result_set_t;

int lowpass(double x, double &y, double i, double a);
int derivation( double x, double & dx, double i);

int dumpData(std::string fileName, std::vector< std::vector<double> >& data);

void getOptions(int argc,
                char *argv[],
                path_t &dataRootPath,
                path_t &outputFile);

#endif // COMMONTOOLS_HH
