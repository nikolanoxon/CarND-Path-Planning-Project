#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "constants.h"

using namespace std;

class Vehicle {
public:

	map<string, int> lane_direction = { {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, 
	{"PLCR", 1} };

	vector<int> lane_position = { LEFT_LANE_CENTER, MID_LANE_CENTER, 
		RIGHT_LANE_CENTER };

	int lane;

	double s;

	double d;

	double yaw;

	double v;

	double a;

	double j;

	int prev_size;

	double target_speed = V_MAX; //	m/s

	string state = "KL";


	/**
	* Constructor
	*/
	Vehicle();
	Vehicle(int lane, double s, double d, double theta, double v, double a, 
		string state);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

	vector<string> successor_states();

	vector<Vehicle> generate_trajectory(string state, map<int, 
		vector<Vehicle>> predictions);

	vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

	vector<Vehicle> lane_change_trajectory(string state, map<int, 
		vector<Vehicle>> predictions);

	Vehicle increment(double dt);

	vector<double> gaussian_move(void);

	vector<Vehicle> generate_predictions(int horizon = 1);
};

#endif /* VEHICLE */