#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

	map<string, int> lane_direction = { {"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1} };

	float lane_width = 4;

	float dt = 0.02; // cycle time (sec)

	int preferred_buffer = 6; // meters

	int lane;

	float s;

	float d;

	float yaw;

	float v;

	float a;

	float j;

	int lanes_available = 3;

	float v_max = 50.0 / 2.24;

	float a_max = 10; //	m/s^2

	float j_max = 10; //	m/s^3;

	float target_speed = v_max; //	m/s

	string state = "KL";

	/**
	* Constructor
	*/
	Vehicle();
	Vehicle(int lane, float s, float d, float theta, float v, float a, string state);

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

	vector<string> successor_states();

	vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

	vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

	vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

	vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

	Vehicle increment(float dt);

	bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

	bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

	vector<Vehicle> generate_predictions(int horizon = 2);

	void realize_next_state(vector<Vehicle> trajectory);

	void configure(vector<int> road_data);

};

#endif