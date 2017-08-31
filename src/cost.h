#ifndef COST_H_
#define COST_H_


#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>

using namespace std;


// The speed in the slow lane will be penalized
const double SLOW = pow(10.0, 6.0);
// The short distance with the car ahead in the lane will be penalized
const double DISTANCE = pow(10.0, 8.5);
// Changing lane frequently should be penalized
const double LANE_CHANGE = pow(10.0, 5.0);




double slow_cost(double speed){return SLOW*abs(speed-(50/2.24));};

double distance_cost(double s1){return DISTANCE/s1;};

double lane_change_cost(){return LANE_CHANGE;};

#endif /*COST_H_*/
