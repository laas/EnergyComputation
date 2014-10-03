#include "motors.hh"

int readData(int indice, std::vector< std::vector<double> > & data);
int treatingData(int experiment_index, Motors & hrp2motors,
                 std::vector< std::vector<double> > & data,
                 std::vector< std::vector<double> > & energyOfMotors,
                 std::vector<std::vector<double> > &energyOfWalk);
int dumpData(std::string fileName, std::vector< std::vector<double> >& data);
int dumpEnergyConsummed(std::string fileName, std::vector<double> & data);
int lowpass(double x, double &y, double i, double a);
int derivation( double x, double & dx, double i);
