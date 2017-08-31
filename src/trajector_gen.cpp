#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "trajector_gen.h"
using namespace std;


double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}





T_GEN::T_GEN(){};

/**
* Destructor
*/
T_GEN::~T_GEN(){};

vector<vector<double>> T_GEN::Solve(double ref_v, int lane_num, double x, double y, double s, double d, double yaw,
                                    double speed, vector<double> prev_path_x, vector<double> prev_path_y, double end_s, double end_d,
                                    vector<double> mp_s, vector<double> mp_x, vector<double> mp_y){


    int prev_size = prev_path_x.size();

    /*
    if (prev_size > 0){
        car_s = end_path_s;
    }
    */

    // Create a vector of (x,y) waypoints
    vector<double>pt_x;
    vector<double>pt_y;

    // Reference x, y and yaw value
    double ref_x = x;
    double ref_y = y;
    double ref_yaw = deg2rad(yaw);


    // If previous size is very small, use the car as starting reference
    if (prev_size < 2){
      double prev_x = x - cos(yaw);
      double prev_y = y - sin(yaw);

      pt_x.push_back(prev_x);
      pt_x.push_back(x);

      pt_y.push_back(prev_y);
      pt_y.push_back(y);
    }

    else{

      ref_x = prev_path_x[prev_size - 1];
      ref_y = prev_path_y[prev_size - 1];

      double ref_x_prev = prev_path_x[prev_size - 2];
      double ref_y_prev = prev_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

      pt_x.push_back(ref_x_prev);
      pt_x.push_back(ref_x);

      pt_y.push_back(ref_y_prev);
      pt_y.push_back(ref_y);

    }

    vector<double> next_wp0 = getXY(s + 30, (2 + 4*lane_num), mp_s, mp_x, mp_y);
    vector<double> next_wp1 = getXY(s + 60, (2 + 4*lane_num), mp_s, mp_x, mp_y);
    vector<double> next_wp2 = getXY(s + 90, (2 + 4*lane_num), mp_s, mp_x, mp_y);

    pt_x.push_back(next_wp0[0]);
    pt_x.push_back(next_wp1[0]);
    pt_x.push_back(next_wp2[0]);

    pt_y.push_back(next_wp0[1]);
    pt_y.push_back(next_wp1[1]);
    pt_y.push_back(next_wp2[1]);

    //Transform the global coordinate into the car's
    for (int i = 0; i < pt_x.size(); i++){

      double shift_x = pt_x[i] - ref_x;
      double shift_y = pt_y[i] - ref_y;

      pt_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
      pt_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    }

    // Create a spline
    tk::spline sp;

    // Set (x, y) to the spline
    sp.set_points(pt_x, pt_y);

    // Define the actual x, y used in the next waypoints status
    vector<double> next_x_pts;
    vector<double> next_y_pts;


    // The next waypoints start with previous path points
    for (int i = 0; i < prev_path_x.size(); i++){

      next_x_pts.push_back(prev_path_x[i]);
      next_y_pts.push_back(prev_path_y[i]);

    }


    //Break the spline points based on the referenced velocity
    double target_x = 30.0;
    double target_y = sp(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    // Fill up the rest of the waypoints
    for (int i = 1; i <= 50 - prev_path_x.size(); i++){
      double N = (target_dist/(0.02*ref_v/2.24));
      double x_point = x_add_on + (target_x)/N;
      double y_point = sp(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      // Now transform the coordinate to the global
      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      next_x_pts.push_back(x_point);
      next_y_pts.push_back(y_point);

    }

    vector<vector<double>> result;
    result.push_back(next_x_pts);
    result.push_back(next_y_pts);

    return result;

};



