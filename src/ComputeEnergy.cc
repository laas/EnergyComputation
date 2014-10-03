#include "ComputeEnergy.hh"

//void getOptions(int argc,
//                char *argv[],
//                string & dataPath)
//{
//  std::cout << "argc:" << argc << std::endl;
//  if (argc!=2)
//  {
//    cerr << " This program takes 1 arguments: " << endl;
//    cerr << "./TestFootPrintPGInterface \n
//        DATA_PATH" << endl;
//    exit(-1);
//  }
//  else
//  {
//    dataPath=argv[1];
//  }
//}
using namespace std ;

vector<string> paths ;
vector<string> filenames ;
vector< vector<int> > beging_end ;
vector<double> walked_distance ;

int main(int argc, char *argv[])
{

    main2();
    return 0;
}

int testBoost()
{

}


int main2()
{
    // reading the datas of the robot
    Motors HRP2motors ;
    vector<int> tmp_begin_end_distance(2);

    // preparing the data path
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/8_12/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/8_12/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/8_12/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/8_12/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/8_12/");

    filenames.push_back("walkOnGravlesForward_2014_09_12_11_04-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walkOnGravlesForward_2014_09_12_11_06-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walkOnGravlesForward_2014_09_12_11_17-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walkOnGravlesForward_2014_09_12_11_20-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walkOnGravlesForward_2014_09_12_11_22-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);

    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/12_24/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/12_24/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/gravles/12_24/");

    filenames.push_back("walkOnGravlesForward_2014_09_12_11_35-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walkOnGravlesForward_2014_09_12_11_39-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walkOnGravlesForward_2014_09_12_11_41-astate.log");
    tmp_begin_end_distance[0] = 2000 ; tmp_begin_end_distance[1] = 3800 ; walked_distance.push_back(1.4) ; beging_end.push_back(tmp_begin_end_distance);

    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/backCarpet/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/backCarpet/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/backCarpet/");

    filenames.push_back("walk5timesLegLength_2014_09_12_13_16-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walk5timesLegLength_2014_09_12_13_17-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walk5timesLegLength_2014_09_12_13_19-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);

    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/greenCarpet/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/greenCarpet/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/greenCarpet/");

    filenames.push_back("walk5timesLegLength_2014_09_12_13_07-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walk5timesLegLength_2014_09_12_13_09-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walk5timesLegLength_2014_09_12_13_11-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);

    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/normal_floor/");

    filenames.push_back("walk5timesLegLength_2014_09_12_13_01-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ;  beging_end.push_back(tmp_begin_end_distance);


    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/greenCarpet/");
    paths.push_back("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/slipFloor/greenCarpet/");

    filenames.push_back("walk5timesLegLength_2014_09_12_13_07-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walk5timesLegLength_2014_09_12_13_09-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);
    filenames.push_back("walk5timesLegLength_2014_09_12_13_11-astate.log");
    tmp_begin_end_distance[0] = 200 ; tmp_begin_end_distance[1] = 3700 ; walked_distance.push_back(3.6) ; beging_end.push_back(tmp_begin_end_distance);



    // reading the data of the manipulation
    vector< vector<double> > data ;
    vector< vector<double> > energyOfMotors ;
    vector< vector<double> > energyOfWalking ;

    for (unsigned int i = 0 ; i < beging_end.size() ; ++i)
    {
        readData(i,data);
        treatingData(i,HRP2motors,data,energyOfMotors,energyOfWalking);
        dumpData("energyOfMotors.dat",energyOfMotors);
        dumpData("energyOfWalking.dat",energyOfWalking);
        vector<double> totalEnergyOfMotors = energyOfMotors.back();
        vector<double> totalEnergyOfWalking = energyOfWalking.back();
        for (unsigned int j = 0 ; j < totalEnergyOfMotors.size() ; ++j)
        {
            totalEnergyOfMotors[j] = totalEnergyOfMotors[j]/walked_distance[i];
            totalEnergyOfWalking[j] = totalEnergyOfWalking[j]/walked_distance[i];
        }
        string path_dump = "/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/experience" ;
        ostringstream oss ;
        oss.str("");
        oss << path_dump <<  i << "_totalEnergyOfWalking.dat" ;
        cout << oss.str() << endl ;
        dumpEnergyConsummed(oss.str(),totalEnergyOfWalking);

        oss.str("/home/mnaveau/devel/ComputeEnergyFromOpenHRPLogs/data/KPI/experience");
        oss << i << "_totalEnergyOfMotors.dat" ;
        dumpEnergyConsummed(oss.str(),totalEnergyOfMotors);
    }
    return 0;
}

int readData(int indice , vector< vector<double> > & data)
{
    data.clear();
    std::string astateFile = paths[indice] + filenames[indice] ;
    std::ifstream dataStream ;
    dataStream.open(astateFile.c_str(),std::ifstream::in);

    // skipping header
    vector<string> titleRobotConfig_(166) ;
    std::vector< std::vector<double> > robotConfig ;
    for (unsigned int i = 0 ; i < titleRobotConfig_.size() ; ++i)
        dataStream >> titleRobotConfig_[i];

    //    for (unsigned int i = 0 ; i < titleRobotConfig_.size() ; ++i)
    //        cout << titleRobotConfig_[i] << " ";
    //    cout << endl;

    while (dataStream.good()) {
        vector<double> oneLine(176) ;
        for (unsigned int i = 0 ; i < oneLine.size() ; ++i)
            dataStream >> oneLine[i];
        data.push_back(oneLine);
    }
    dataStream.close();
    return 0;
}

int treatingData(int experiment_index,
                 Motors & hrp2motors,
                 vector< vector<double> > & data,
                 vector<vector<double> > & energyOfMotors,
                 vector<vector<double> > & energyOfWalk)
{
    int N = beging_end[experiment_index][1] ;
    int N_1 = beging_end[experiment_index][0] ;
    vector< vector<double> > q (N-N_1,vector<double>(30,0)) ;
    vector< vector<double> > dq (N-N_1,vector<double>(30,0)) ;
    vector< vector<double> > torques (N-N_1,vector<double>(30,0)) ;
    vector< vector<double> > powerOfWalk (N-N_1,vector<double>(30,0)) ;
    vector< vector<double> > powerOutputMotors (N-N_1,vector<double>(30,0)) ;
    energyOfMotors.resize(N-N_1,vector<double>(31));
    energyOfWalk.resize(N-N_1,vector<double>(31));
    for (unsigned int i = N_1 ; i < N ; ++i )
    {
        for (unsigned int j = 0 ; j < 30 ; ++j )
        {
            q[i-N_1][j] = hrp2motors.gear_ratio_[j] * data[i][j];
        }
        for (unsigned int j = 40 ; j < 40+30 ; ++j )
        {
            torques[i-N_1][j-40] = data[i][j];
        }
    }


    for (unsigned int j = 0 ; j < torques[0].size() ; ++j )
        for (unsigned int i = 0 ; i < N-N_1 ; ++i )
            derivation(q[i][j],dq[i][j],i);

    dumpData("dq_unfiltered.dat", dq);
    for (unsigned int j = 0 ; j < torques[0].size() ; ++j )
        for (unsigned int i = 0 ; i < N-N_1 ; ++i )
            lowpass(torques[i][j],torques[i][j],i,0.03);

    for (unsigned int j = 0 ; j < torques[0].size() ; ++j )
        for (unsigned int i = 0 ; i < N-N_1 ; ++i )
            lowpass(dq[i][j],dq[i][j],i,0.03);

    for (unsigned int i = 0 ; i < N-N_1 ; ++i )
    {
        for (unsigned int j = 0 ; j < torques[0].size() ; ++j )
        {
            powerOutputMotors[i][j] = abs(dq[i][j]) * abs(torques[i][j]) ;
            powerOfWalk[i][j] = hrp2motors.R_K2[j] * torques[i][j] * torques[i][j] +
                    powerOutputMotors[i][j] ;
        }
    }

    dumpData("data.dat", data);
    dumpData("torques.dat", torques);
    dumpData("powerOutputMotors.dat", powerOutputMotors);
    dumpData("powerOfWalk.dat", powerOfWalk);
    dumpData("dq.dat", dq);
    dumpData("q.dat", q);
    double dt = 0.005;
    for (unsigned int j = 0 ; j < energyOfMotors[0].size() ; ++j )
    {
        energyOfMotors[0][j] = 0.0 ;
        energyOfWalk[0][j] = 0.0 ;
    }
    for (unsigned int i = 1 ; i < energyOfMotors.size() ; ++i )
    {
        for (unsigned int j = 0 ; j < powerOutputMotors[i].size() ; ++j )
        {
            energyOfMotors[i][j+1] = energyOfMotors[i-1][j+1] + powerOutputMotors[i][j]*dt ;
            energyOfWalk[i][j+1] = energyOfWalk[i-1][j+1] + powerOfWalk[i][j]*dt ;
        }

        double sum1 = 0.0;
        double sum2 = 0.0;
        for (unsigned int j = 0 ; j < torques.size() ; ++j )
        {
            sum1 += energyOfMotors[i][j+1] ;
            sum2 += energyOfWalk[i][j+1] ;
        }
        energyOfMotors[i][0] = sum1 ;
        energyOfWalk[i][0] = sum2 ;
    }

    return 0;
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

int dumpEnergyConsummed(string fileName, vector<double> & data)
{
    ofstream dumpStream ;
    dumpStream.open(fileName.c_str(),ofstream::out);
    dumpStream << "globalEnergy(J/m)    " ;
    for (unsigned int j = 1 ; j < data.size()-1 ; ++j)
        dumpStream << "motor" << j << "Energy(J/m)    " ;
    dumpStream << "motor" << data.size()-1 << "Energy(J/m)" ;

    dumpStream << endl;
    for (unsigned int j = 0 ; j < data.size() ; ++j)
    {
        dumpStream << data[j] << " " ;
    }
    dumpStream << endl ;
    dumpStream.close();
    cout << "dumped" << endl;
    return 0 ;
}

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