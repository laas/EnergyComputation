#include "commonTools.hh"
#include "motors.hh"
#include "explorefolder.hh"
#include "experience.hh"

int readData(int indice, std::vector< std::vector<double> > & data);
int treatingData(int experiment_index, Motors & hrp2motors,
                 std::vector< std::vector<double> > & data,
                 std::vector< std::vector<double> > & energyOfMotors,
                 std::vector<std::vector<double> > &energyOfWalk);

int dumpEnergyConsummed(std::string fileName, std::vector<double> & data);
int main2();
