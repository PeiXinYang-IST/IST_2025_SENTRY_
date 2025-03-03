#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Eigen"
#include "tf2/utils.h"
using namespace std;
class Mpc
{
private:
    
public:
    Mpc(/* args */);
    ~Mpc();

    vector<double> solve(Eigen::Vector3d state, Eigen::MatrixXd desired_state,double dist,Eigen::Vector2d gradient,double safe_distance);
};



#endif