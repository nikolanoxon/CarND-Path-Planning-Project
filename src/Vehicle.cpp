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
			cout << "state: " << *it << "   cost: " << cost << endl;
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
	/*
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
	*/
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

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions)
{
	/*
	INPUT: A predictions map of the trajectories of other vehicles
	OUTPUT: A trajectory
	*/

	// Check if there is a vehicle ahead in our lane within the horizon
	// From math, I know that given the target speed of 50 mph, if I encountered a vehicle at a
	// standstill I would need 32 meters to brake at max decel rate. So let's set the lookahead distance to
	// 50 meters
	// If there is no target, try to get to set speed

	float s_min = this->s + 50;	// meters

	float a_new, v_new, d_new, s_new;

	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };


	// Check if there will be a vehicle in this lane within the minimum distance
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		if (it->second[1].lane == this->lane && it->second[1].s < s_min && it->second[1].s > this->s) {
			// Set speed to the ahead vehicle speed
			//TODO add jerk limitation
			v_new = min(this->target_speed, it->second[1].v);
			// Set the new closest vehicle
			s_min = it->second[1].s;
		}
		// If there's no vehicle in front, accelerate to top speed
		else {
			v_new = this->v_max;
		}
	}

	d_new = (this->lane + 0.5) * this->lane_width;

	// State in 1 second
	// TODO: Move this calculation since we'll be using it a lot
	s_new = this->s + v_new;

	trajectory.push_back(Vehicle(this->lane, s_new, d_new, this->yaw, v_new, this->a, this->state));
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };

	return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };
	trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state));
	return trajectory;
}

Vehicle Vehicle::increment(float dt)
{
	/*
	Moves a vehicle forward one timestep
	*/
	float a_new = this->a + this->j * dt;
	float v_new = this->v + this->a * dt + 0.5 * this->j * pow(dt, 2);
	float s_new = this->s + this->v * dt + 0.5 * this->a * pow(dt, 2) + 1.0 / 6 * this->j * pow(dt, 3);

	// constrain the acceleration and velocity to the max allowable values
	a_new = min(a_new, this->a_max);
	v_new = min(v_new, this->v_max);


	float d_new = (this->lane + 0.5) * this->lane_width;

	return Vehicle(this->lane, s_new, d_new, this->yaw, v_new, a_new, this->state);
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
	/*
	Generates a vector of predicted states for a vehicle with 1 second intervals.
	The length of the prediction is equal to the horizon (in seconds).
	*/
	vector<Vehicle> predictions;
	for (int i = 0; i < horizon; ++i) {
		predictions.push_back(increment(i));
	}
	return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory)
{
}

void Vehicle::configure(vector<int> road_data)
{
}
