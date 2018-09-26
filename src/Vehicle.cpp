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
	max_acceleration = -1;

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

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
	/*
	Gets next timestep kinematics (position, velocity, acceleration)
	for a given lane. Tries to choose the maximum velocity and acceleration,
	given other vehicle positions and accel/velocity constraints.
	*/
	float max_velocity_accel_limit = this->max_acceleration + this->v;
	float new_s;
	float new_d;
	float new_velocity;
	float new_accel;
	Vehicle vehicle_ahead;
	Vehicle vehicle_behind;

	if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

		if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
			new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
		}
		else {
			float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
			new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
		}
	}
	else {
		new_velocity = min(max_velocity_accel_limit, this->target_speed);
	}
	// TODO: note that this equation looks exactly 1 second in the future
	new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
	new_s = this->s + new_velocity + new_accel / 2.0;
	new_d = (lane + 0.5)*lane_width;
	return{ new_s, new_d, new_velocity, new_accel };

}
// TODO: update the trajectory generators with method from main.cpp
vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
	/*
	Generate a keep lane trajectory.
	*/
	vector<Vehicle> trajectory = { Vehicle(lane, this->s, this->d, this->yaw, this->v, this->a, state) };
	vector<float> kinematics = get_kinematics(predictions, this->lane);
	float new_s = kinematics[0];
	float new_d = kinematics[1];
	float new_v = kinematics[2];
	float new_a = kinematics[3];
	trajectory.push_back(Vehicle(this->lane, new_s, new_d, this->yaw, new_v, new_a, "KL"));
	return trajectory;
}
// TODO: update the trajectory generators with method from main.cpp
vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	/*
	Generate a trajectory preparing for a lane change.
	*/
	float new_s;
	float new_d;
	float new_v;
	float new_a;
	Vehicle vehicle_behind;
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };
	vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

	if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
		//Keep speed of current lane so as not to collide with car behind.
		new_s = curr_lane_new_kinematics[0];
		new_d = curr_lane_new_kinematics[1];
		new_v = curr_lane_new_kinematics[2];
		new_a = curr_lane_new_kinematics[3];

	}
	else {
		vector<float> best_kinematics;
		vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
		//Choose kinematics with lowest velocity.
		if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
			best_kinematics = next_lane_new_kinematics;
		}
		else {
			best_kinematics = curr_lane_new_kinematics;
		}
		new_s = best_kinematics[0];
		new_d = best_kinematics[1];
		new_v = best_kinematics[2];
		new_a = best_kinematics[3];
	}

	trajectory.push_back(Vehicle(this->lane, new_s, new_d, this->yaw, new_v, new_a, state));
	return trajectory;
}
// TODO: update the trajectory generators with method from main.cpp
vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	/*
	Generate a lane change trajectory.
	*/
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory;
	Vehicle next_lane_vehicle;
	//Check if a lane change is possible (check if another vehicle occupies that spot).
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		next_lane_vehicle = it->second[0];
		if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
			//If lane change is not possible, return empty trajectory.
			return trajectory;
		}
	}
	trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state));
	vector<float> kinematics = get_kinematics(predictions, new_lane);
	trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], this->yaw, kinematics[3], state));
	return trajectory;
}

void Vehicle::increment(int dt = 1) {
	this->s = position_at(dt);
}


float Vehicle::position_at(int t) {
	/*
	Generates the position of the vehicle at a future time
	*/
	return this->s + this->v*t + this->a*t*t / 2.0;
}
// TODO: change this function to receive data from the simulator
bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	/*
	Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
	rVehicle is updated if a vehicle is found.
	*/
	int max_s = -1;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
			max_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

// TODO: change this function to receive data from the simulator
bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	/*
	Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
	rVehicle is updated if a vehicle is found.
	*/

	// Look 50 meters ahead
	int min_s = 50;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
			min_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
	/*
	Generates predictions for non-ego vehicles to be used
	in trajectory generation for the ego vehicle.
	*/
	vector<Vehicle> predictions;
	for (int i = 0; i < horizon; i++) {
		float next_s = position_at(i);
		// Assume that the vehicle has a constant velocity over the horizon
		// TO-DO: Add acceleration
		predictions.push_back(Vehicle(this->lane, next_s, this->d, this->a, this->v, 0));
	}
	return predictions;
}

// TODO: likely remove this function since we just send a trajectory to the simulator
void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
	/*
	Sets state and kinematics for ego vehicle using the last state of the trajectory.
	*/
	Vehicle next_state = trajectory[1];
	this->state = next_state.state;
	this->lane = next_state.lane;
	this->s = next_state.s;
	this->v = next_state.v;
	this->a = next_state.a;
}

// TODO: change this function to receive data from the simulator
void Vehicle::configure(vector<int> road_data) {
	/*
	Called by simulator before simulation begins. Sets various
	parameters which will impact the ego vehicle.
	*/
	target_speed = road_data[0];
	lanes_available = road_data[1];
	goal_s = road_data[2];
	goal_lane = road_data[3];
	max_acceleration = road_data[4];
}