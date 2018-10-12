#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "constants.h"
#include "Cost.h"
#include "Vehicle.h"
#include "jmt.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

double mag(double X, double Y)
{
	return sqrt(pow(X,2) + pow(Y,2));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
	const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, 
	const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
	const vector<double> &maps_x, const vector<double> &maps_y)
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
	const vector<double> &maps_x, const vector<double> &maps_y)
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
  	double s;
  	double d_x;
  	double d_y;
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

  Vehicle vehicle = Vehicle();
  vehicle.lane = MID_LANE;

// Needed for backwards compatibility with older uWebSockets version
#ifdef UWS_VCPKG
  h.onMessage([&vehicle,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
	  &map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> *ws, 
		  char *data, size_t length, uWS::OpCode opCode) {
#else
  h.onMessage([&vehicle,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
	  &map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, 
		  char *data, size_t length, uWS::OpCode opCode) {
#endif
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
          
			/*
			SIMULATOR INPUT: Get all data from the simulator
			*/

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];

			//TODO: fix the assertion error when the track loops back to the start (s_new < s_old)

          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"] / 2.24;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
			int prev_size = previous_path_x.size();

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

			// Sensor Fusion Data, a list of all other cars on the 
			// same side of the road.
			auto sensor_fusion = j[1]["sensor_fusion"];

			/*
			SENSOR FUSION: Determine the current and future state 
			of other road users
			*/

			// A map of predictions for non-ego vehicles
			map<int, vector<Vehicle>> predictions;

			for (int i = 0; i < sensor_fusion.size(); i++) {

				int id = sensor_fusion[i][0];
				double x = sensor_fusion[i][1];
				double y = sensor_fusion[i][2];
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double s = sensor_fusion[i][5];
				double d = sensor_fusion[i][6];

				Vehicle road_vehicle;
				road_vehicle.v = mag(vx, vy);
				road_vehicle.s = s;
				road_vehicle.d = d;
				road_vehicle.lane = int(floor((d / LANE_WIDTH)));
				road_vehicle.a = 0; // Assume no acceleration

				//A vector of predictions for a non-ego vehicle
				vector<Vehicle> prediction = 
					road_vehicle.generate_predictions();

				predictions.insert(pair<int, vector<Vehicle>>(id, prediction));
			} 
			
			/*
			VEHICLE STATE: Determine vehicle kinematics for 
			initial state estimation
			*/

			// Reference states
			double ref_yaw = deg2rad(car_yaw);

			double ref_x1, ref_x2, ref_x3;
			double ref_y1, ref_y2, ref_y3;
			double ref_v1;
			double ref_s, ref_d;
			double ref_v2 = 0;
			double ref_a1 = 0;

			// Create a list of widely spaced (x,y) waypoints
			vector<double> ptsx, ptsy;

			// If there is no trajectory, use the car's position 
			// and yaw rate to create anchor points
			if (prev_size < 2) {
				ref_x1 = car_x;
				ref_y1 = car_y;

				ref_x2 = ref_x1 - cos(ref_yaw);
				ref_y2 = ref_y1 - sin(ref_yaw);

				ref_v1 = car_speed;

				ref_s = car_s;
				ref_d = car_d;
			}
			// Otherwise use the last two points to create anchor points
			else {
				ref_x1 = previous_path_x[prev_size - 1];
				ref_y1 = previous_path_y[prev_size - 1];

				ref_x2 = previous_path_x[prev_size - 2];
				ref_y2 = previous_path_y[prev_size - 2];

				ref_v1 = mag(ref_x1 - ref_x2, ref_y1 - ref_y2) / DT;

				ref_yaw = atan2(ref_y1 - ref_y2, ref_x1 - ref_x2);

				ref_s = end_path_s;
				ref_d = end_path_d;
			}

			double ref_lane = int(floor((ref_d / LANE_WIDTH)));

			// Set the anchor points
			ptsx.push_back(ref_x2);
			ptsx.push_back(ref_x1);
			ptsy.push_back(ref_y2);
			ptsy.push_back(ref_y1);

			// Find the acceleration at the end of the last trajectory
			if (prev_size > 2) {
				ref_x3 = previous_path_x[prev_size - 3];
				ref_y3 = previous_path_y[prev_size - 3];

				ref_v2 = mag(ref_x2- ref_x3, ref_y2 - ref_y3) / DT;

				ref_a1 = (ref_v1 - ref_v2) / DT;
			}

			/*
			MOTION CONTROL 1: Load the historical data
			*/

			// Define the actual (x,y) points we will use for the planner
			vector<double> next_x_vals, next_y_vals, next_v_vals, next_a_vals;

			// Start with all the previous points from  the last time
			for (int i = 0; i < prev_size; ++i) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			/*
			BEHAVIOR GENERATION:	Select the lowest cost trajectory from a 
			list of randomly generated trajectories
			*/

			// Run trajectory planner on a fixed update rate
			if ((N_CYCLES - prev_size) * DT > UPDATE_RATE) {
				// Vehicle data
				vehicle.s = ref_s;
				vehicle.d = ref_d;
				vehicle.v = ref_v1;
				vehicle.a = ref_a1;
				vehicle.yaw = ref_yaw;
				vehicle.lane = ref_lane;
				vehicle.prev_size = prev_size;

				// Select the best trajectory
				vector<Vehicle> best_trajectory = 
					vehicle.choose_next_state(predictions);

				double next_s = best_trajectory.back().s;
				double next_d = best_trajectory.back().d;
				double next_v = best_trajectory.back().v;
				double next_a = best_trajectory.back().a;
				double next_lane = best_trajectory.back().lane;
				double delta_s = next_s - car_s;
				double delta_d = next_d - car_d;

				// Update State
				if (next_lane > ref_lane) {
					vehicle.state = "LCR";
				}
				else if (next_lane < ref_lane) {
					vehicle.state = "LCL";
				}
				else {
					vehicle.state = "KL";
				}

				// This spline adjustment factor smooths out a lane change 
				// maneuver
				double s_adjust_lc = 0;
				if (vehicle.state.compare("KL") == 1) {
					s_adjust_lc = 20;
				}

/*
				cout << "Current State: " << vehicle.state << endl;

				cout << "CURRENT s: " << car_s << "   d: " << car_d << "   v: " << car_speed << "   lane: " << vehicle.lane << endl;
				cout << "NEXT   s: " << next_s << "   d: " << next_d << "   v: " << next_v << "   a: " << next_a << "   lane: " << next_lane << endl;
				cout << "REF1   s: " << ref_s << "   x1: " << ref_x1 << "   y1: " << ref_y1 << "   v1: " << ref_v1 << "   a1: " << ref_a1 << endl;
				cout << "REF2   s: " << ref_s << "   x2: " << ref_x2 << "   y2: " << ref_y2 << "   v2: " << ref_v2 << endl;
				cout << "GOAL   ds: " << delta_s << "   dd: " << delta_d << endl;
				cout << "previous size: " << prev_size << endl;

*/

				vector<double> next_wp0 = getXY(car_s + 30 + s_adjust_lc, 
					next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				vector<double> next_wp1 = getXY(car_s + 60 + s_adjust_lc, 
					next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				vector<double> next_wp2 = getXY(car_s + 90 + s_adjust_lc,
					next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

				ptsx.push_back(next_wp0[0]);
				ptsx.push_back(next_wp1[0]);
				ptsx.push_back(next_wp2[0]);

				ptsy.push_back(next_wp0[1]);
				ptsy.push_back(next_wp1[1]);
				ptsy.push_back(next_wp2[1]);

				for (int i = 0; i < ptsx.size(); ++i) {
					// Shift car reference point to 0
					double shift_x = ptsx[i] - ref_x1;
					double shift_y = ptsy[i] - ref_y1;

					// Shift car reference angle to 0
					ptsx[i] = (shift_x*cos(-ref_yaw) - shift_y * sin(-ref_yaw));
					ptsy[i] = (shift_x*sin(-ref_yaw) + shift_y * cos(-ref_yaw));
				}

				// Create a spline
				tk::spline s;

				// Set (x,y) points to the spline
				cout << "PTSX: " << ptsx[0] << "; " << ptsx[1] << "; " << ptsx[2] << "; " << ptsx[3] << "; " << ptsx[4] << endl;
				s.set_points(ptsx, ptsy);

				// Calculate the JMT to smoothly move to the next point
				vector<double> start = { 0, ref_v1, ref_a1 };
				vector<double> end = { next_s - ref_s, next_v, next_a };
				double t = DT * (N_CYCLES - prev_size);
				vector<double> a_vals = JMT(start, end, t);

				// Append the JMT to the end of the current trajectory
				for (int i = 1; i <= (N_CYCLES - prev_size); ++i) {
					double t = i * DT;
					double x_point = a_vals[0] + a_vals[1] * t 
						+ a_vals[2] * pow(t, 2) + a_vals[3] * pow(t, 3) 
						+ a_vals[4] * pow(t, 4) + a_vals[5] * pow(t, 5);
					double y_point = s(x_point);
					//double v_point = a_vals[1] + 2 * a_vals[2] * t + 3 * a_vals[3] * pow(t, 2) + 4 * a_vals[4] * pow(t, 3) + 5 * a_vals[5] * pow(t, 4);
					//double a_point = 2 * a_vals[2] + 6 * a_vals[3] * t + 12 * a_vals[4] * pow(t, 2) + 20 * a_vals[5] * pow(t, 3);

					double x_ref = x_point;
					double y_ref = y_point;

					// Un-shift back to normal
					x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
					y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

					x_point += ref_x1;
					y_point += ref_y1;

					next_x_vals.push_back(x_point);
					next_y_vals.push_back(y_point);
					//next_v_vals.push_back(v_point);
					//next_a_vals.push_back(a_point);
				}

			}
			json msgJson;
			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;

			auto msg = "42[\"control\"," + msgJson.dump() + "]";

// Needed for backwards compatibility with older uWebSockets version
#ifdef UWS_VCPKG
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
	  }	else {
		// Manual driving
		std::string msg = "42[\"manual\",{}]";
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
			//this_thread::sleep_for(chrono::milliseconds(1000));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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

// Needed for backwards compatibility with older uWebSockets version
#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port)) {
	  std::cout << "Listening to port " << port << std::endl;
  } else {
	  std::cerr << "Failed to listen to port" << std::endl;
      return -1;
  }
#else
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
  }
  else {
	  std::cerr << "Failed to listen to port" << std::endl;
	  return -1;
  }
#endif
  h.run();
}
