#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float s, float d, float yaw, float v, float a, string state) {
/*
A vehicle object with lane and kinematic data
*/
	this->lane = lane;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
	this->v = v;
	this->a = a;
	this->state = state;
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
	/*
	INPUT: A predictions map. This is a map of vehicle id keys with predicted
		vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
		the vehicle at the current timestep and one timestep in the future.
	OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
	*/
	vector<string> states = successor_states();

	/*
	for (int i = 0; i < states.size(); ++i) {
		cout << states[i] << endl;
	}
	*/
	float cost;
	vector<float> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
		vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
		if (trajectory.size() != 0) {
			cost = calculate_cost(*this, predictions, trajectory);
			costs.push_back(cost);
			final_trajectories.push_back(trajectory);
			//cout << "state: " << *it << "   cost: " << cost << endl;
		}
	}

	vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);
	return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
	/*
	Provides the possible next states given the current state for the FSM
	*/
	vector<string> states;
	states.push_back("KL");
	string state = this->state;
	if (state.compare("KL") == 0) {
		if (lane != lanes_available - 1) {
			states.push_back("PLCR");
		}
		if (lane != 0) {
			states.push_back("PLCL");
		}
	}
	else if (state.compare("PLCR") == 0) {
		if (lane != lanes_available - 1) {
			states.push_back("PLCR");
			states.push_back("LCR");
		}
	}
	else if (state.compare("PLCL") == 0) {
		if (lane != 0) {
			states.push_back("PLCL");
			states.push_back("LCL");
		}
	}
	else if (state.compare("LCL") == 0) {
		if (lane != 0) {
			states.push_back("LCL");
		}
	}
	else if (state.compare("LCR") == 0) {
		if (lane != lanes_available - 1) {
			states.push_back("LCR");
		}
	}
	return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	/*
	Given a possible next state, generate the appropriate trajectory to realize the next state.
	*/
	vector<Vehicle> trajectory;
	if (state.compare("KL") == 0) {
		trajectory = keep_lane_trajectory(predictions);
	}
	else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
		trajectory = lane_change_trajectory(state, predictions);
	}
	else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
		trajectory = prep_lane_change_trajectory(state, predictions);
	}
	return trajectory;
}


//TODO add jerk limitation
vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions)
{
	// Check if there is a vehicle ahead in our lane within the horizon
	// From math, I know that given the target speed of 50 mph, if I encountered a vehicle at a
	// standstill I would need 32 meters to brake at max decel rate. So let's set the lookahead distance to
	// 50 meters
	// If there is no target, try to get to set speed

	float min_s = this->s + 50;	// meters

	for (map<int, vector<Vehicle>>::iterator it = predictions.begin; it != predictions.end; ++it) {
		// Check if there is a vehicle in this lane within the minimum distance
		if (it->second[0].lane == this->lane && it->second[0].s < min_s) {
			// Set speed to the ahead vehicle speed
			this->v = min(this->target_speed, it->second[0].v);
		}
	}


	if (this->v < this->target_speed) {

	
	}

	return vector<Vehicle>();
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
	return vector<Vehicle>();
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
	return vector<Vehicle>();
}

void Vehicle::increment(int dt)
{
	float a_new = this->a + this->max_j * this->dt;
	float v_new = this->v + this->a * this->dt + 0.5 * this->j * pow(this->dt, 2);
	float s_new = this->s + this->v * this->dt + 0.5 * this->a * pow(this->dt, 2) + 1.0 / 6 * this->j * pow(this->dt, 3);

	this->a = a_new;
	this->v = v_new;
	this->s = s_new;
}

float Vehicle::position_at(int t)
{
	return 0.0f;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle)
{
	return false;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle)
{
	return false;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon)
{
	return vector<Vehicle>();
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory)
{
}

void Vehicle::configure(vector<int> road_data)
{
}
