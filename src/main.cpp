#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "trajector_gen.h"
#include "cost.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}



// set the lane num
int lane = 1;

// set the reference velocity
double ref_vel = 0; // unit in mph

int count_number = 0;

double d_error = 0;
double p_error = 0;
double i_error = 0;




int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }





  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;



    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();

          	double current_s = car_s;

          	count_number++;


          	if (prev_size > 0){
                car_s = end_path_s;
          	}

            // Set up the trajector generator class
          	T_GEN trajector_generator;

            // Set up the total cost for each lane, totally 3 lanes
            vector<double> total_cost = {0.0, 0.0, 0.0};

            // Loop across the lanes and score their costs
          	for (int new_lane = 0; new_lane < 3; new_lane++){

                // Define a minimum cost number, and flag the car id as -1 at the beginning
                // Also define the slowest speed, and flag the car id as -1 at the beginning
                int slowest_ahead_car_id = -1;
                int closest_ahead_car_id = -1;
                double closest_ahead_distance = 1000000;
                double slowest_ahead_car_speed = 100;

                for (int i = 0; i < sensor_fusion.size(); i++){
                    float d = sensor_fusion[i][6];

                    // Identify the cars in the target lane
                    if (d < (2 + 4 * new_lane + 2) && d > (2 + 4* new_lane -2)){

                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx*vx + vy*vy);
                        double check_car_s = sensor_fusion[i][5];

                        // Find out the closest car in front of the ego car
                        // Find out the slowest car in front of the ego car
                        if (check_car_s > current_s){
                            if((check_car_s - current_s) < closest_ahead_distance){
                                closest_ahead_car_id = i;
                                closest_ahead_distance = check_car_s - current_s;
                            }
                            if (check_speed < slowest_ahead_car_speed){
                                slowest_ahead_car_id = i;
                                slowest_ahead_car_speed = check_speed;
                            }
                        };
                    };
                };

                // Calculate the distance cost of the closest car, and the speed cost of the slowest car
                // The cost function could be found at cost.h
                // If the target lane and current lane are not the same, a lane change cost should be added to the total cost
                if (closest_ahead_distance < 300){
                    total_cost[new_lane] += distance_cost(closest_ahead_distance);
                }
                if (slowest_ahead_car_speed != 100){
                    total_cost[new_lane] += slow_cost(slowest_ahead_car_speed);
                }
                if (new_lane != lane){
                    total_cost[new_lane] += lane_change_cost();
                }
          	}


            // Loop over the total costs of the three lanes and find out the one with minimum cost
            // and define it as lane_decision
            double min_total_cost = total_cost[0];
          	int lane_decision = 0;

          	for (int i = 0; i < 3; i++){
                if (total_cost[i] < min_total_cost){
                    min_total_cost = total_cost[i];
                    lane_decision = i;
                }
            }

            // Define the target lane as 0 at the beginning
            int target_lane = 0;

            // Get the potential lane by comparing the lane_decision with lane. Only single lane change is allowed,
            // so the car will just change one lane even when the current lane is two lanes away from lane_decision.
            if (lane > lane_decision && lane > 0){
                target_lane = lane - 1;
            }
            else if (lane < lane_decision && lane < 2){
                target_lane = lane + 1;
            }
            else{
                target_lane = lane;
            }



            bool lane_change_feasible = true;

            // Check the feasibility of changing to the target lane without collision
            if (target_lane != lane){

                // Define the hypothetical (x, y) values for the target lane trajectory
                vector<double> h_next_x_vals;
                vector<double> h_next_y_vals;
                vector<double> s_and_d;

                // Define the hypothetical target lane trajectory
                vector<vector<double>> h_traj;

                h_traj = trajector_generator.Solve(ref_vel, target_lane, car_x, car_y, car_s, car_d, car_yaw,
                                                   car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d,
                                                   map_waypoints_s, map_waypoints_x, map_waypoints_y);

                h_next_x_vals = h_traj[0];
                h_next_y_vals = h_traj[1];

                int traj_size = h_next_x_vals.size();

                double p_x = h_next_x_vals[traj_size - 1];
                double p_y = h_next_y_vals[traj_size - 1];
                double p_x_prev = h_next_x_vals[traj_size - 2];
                double p_y_prev = h_next_y_vals[traj_size - 2];
                double p_yaw = atan2(p_y - p_y_prev, p_x - p_x_prev);

                s_and_d = getFrenet(p_x, p_y, p_yaw, map_waypoints_x, map_waypoints_y);
                double p_s = s_and_d[0];
                double p_d = s_and_d[1];

                for (int i = 0; i < sensor_fusion.size(); i++){
                    float d = sensor_fusion[i][6];

                    // Identify the cars in the target lane
                    if (d < (2 + 4 * target_lane + 2) && d > (2 + 4* target_lane -2)){

                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx*vx + vy*vy);
                        double check_car_s = sensor_fusion[i][5];
                        double orig_check_car_s = check_car_s;

                        check_car_s += ((double)traj_size * 0.02 * check_speed);

                        // Check if any car exists within 12 m away from the ego car,
                        // either by the predicted s at the end of the trajectory,
                        // or by the current s of the checked car and ego car.
                        // If it does, the lane change is not feasible.
                        if (abs(check_car_s - p_s) < 12 || abs(orig_check_car_s - current_s) < 12){
                            lane_change_feasible = false;
                        }
          	        }
                }
          	}


          	// If the lane change is feasible, car speed is higher than 30 and count number larger than 50,
          	// change the lane, and return count number to zero.
          	// The purpose of the count number is to prevent the double lane change which might induce jerk.
          	if (lane_change_feasible && car_speed > 30 && count_number > 50){
                lane = target_lane;
                count_number = 0;
          	}

            // Regulate the ego car speed by checking the distance with the front car
            double acceleration = 0.224;
            double closest_distance = 10000;
            int closest_speed = -1;

          	for (int i = 0; i < sensor_fusion.size(); i++){
                float d = sensor_fusion[i][6];
                if (d < (2 + 4 * lane + 2) && d > (2 + 4* lane -2)){
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    double orig_check_car_s = check_car_s;

                    // Find out the speed and distance of the car right in front of ego car
                    check_car_s += ((double)prev_size * 0.02 * check_speed);
                    if ((check_car_s > car_s) && ((check_car_s - car_s) < closest_distance)){
                        closest_distance = check_car_s - car_s;
                        closest_speed = check_speed;
                    }
                }
          	}

            // If the front car is within 35 m and its speed is smaller than the ego car,
            // adjust the speed of the ego car close to the front car
          	if(closest_distance < 35 && closest_speed != -1 && (ref_vel/2.24) > closest_speed){
               ref_vel -= 0.224;
          	}
          	// Otherwise keep the ego car speed at 49.5
          	else if (closest_distance > 35 && ref_vel < 49.5){
              ref_vel += acceleration;
          	}


            // Define the actual x, y used in the next waypoints status
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            vector<vector<double>> traj;

            // Generate the values for the next trajectory
            traj = trajector_generator.Solve(ref_vel, lane, car_x, car_y, car_s, car_d, car_yaw,
                                             car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d,
                                             map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals = traj[0];
            next_y_vals = traj[1];

          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

