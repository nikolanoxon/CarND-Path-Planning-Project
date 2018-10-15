#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "jmt.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

random_device rd;
default_random_engine gen(rd());

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, double s, double d, double yaw, double v, double a, 
	string state) {
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


vector<Vehicle> Vehicle::choose_next_state(map<int, 
	vector<Vehicle>> predictions) {
	/*
	INPUT: A predictions map. This is a map of vehicle id keys with predicted
		vehicle trajectories as values. Trajectories are a vector of Vehicle 
		objects representing
		the vehicle at the current timestep and one timestep in the future.
	OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego 
	vehicle state.
	*/
	vector<string> states = successor_states();
	double cost;
	vector<double> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {

		// Generating random predictions
		for (int i = 0; i < N_TRAJECTORIES; ++i) {
			vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
			if (trajectory.size() != 0) {
				// Calculate the coefficients of a JMT for this maneuver
				vector<double> start = { 0.0, trajectory.front().v, 
					trajectory.front().a };
				vector<double> end = 
				{ trajectory.back().s - trajectory.front().s, 
					trajectory.back().v, trajectory.back().a };
				double T = N_CYCLES * DT;
				vector<double> a_vals = JMT(start, end, T);

				// Calculate the cost of each trajectory
				cost = calculate_cost(*this, trajectory, predictions, a_vals);
				costs.push_back(cost);
				final_trajectories.push_back(trajectory);

				cout << "state: " << *it
					<< "   cost: " << cost
					<< "   speed: " << trajectory.back().v
					<< "   delta_s: " << trajectory.back().s
					- trajectory.front().s
					<< endl;

			}
		}
	}
	// Identify the lowest cost trajectory
	vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
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
		if (lane != RIGHT_LANE) {
			states.push_back("LCR");
		}
		if (lane != LEFT_LANE) {
			states.push_back("LCL");
		}
	}
	else if (state.compare("LCR") == 0) {
		if (lane != RIGHT_LANE) {
			states.push_back("LCR");
		}
	}
	else if (state.compare("LCL") == 0) {
		if (lane != LEFT_LANE) {
			states.push_back("LCL");
		}
	}
	return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, 
	vector<Vehicle>> predictions) {
	/*
	Given a possible next state, generate the appropriate trajectory to realize 
	the next state.
	*/

	vector<Vehicle> trajectory;
	if (state.compare("KL") == 0) {
		trajectory = keep_lane_trajectory(predictions);
	}
	else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
		trajectory = lane_change_trajectory(state, predictions);
	}
	return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> 
	predictions) {
	/*
	INPUT: A predictions map of the trajectories of other vehicles
	OUTPUT: A waypoint 5 seconds in the future
	*/

	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, 
		this->yaw, this->v, this->a, this->state) };
	
	// Perform a gaussian move to get a random future state
	vector<double> kinematics = gaussian_move();

	double a_new = kinematics[0];
	double v_new = kinematics[1];
	double s_new = kinematics[2];
	
	double d_new = this->lane_position[this->lane];

	trajectory.push_back(Vehicle(this->lane, s_new, d_new, this->yaw, v_new, 
		a_new, this->state));
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, 
	vector<Vehicle>> predictions) {
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, 
		this->yaw, this->v, this->a, this->state) };

	// Perform a gaussian move to get a random future state
	vector<double> kinematics = gaussian_move();

	double a_new = kinematics[0];
	double v_new = kinematics[1];
	double s_new = kinematics[2];

	// The new lane is one lane over from the current lane
	int new_lane = this->lane + lane_direction[state];

	// The new d value corresponds to the new lane
	double d_new = this->lane_position[new_lane];

	trajectory.push_back(Vehicle(new_lane, s_new, d_new, this->yaw, v_new, 
		a_new, state));
	return trajectory;
}

vector<double> Vehicle::gaussian_move() {
	/*
	INPUT: none
	OUTPUT: velocity, distance, and acceleration values based on a 
	gaussian change in velocity
	*/

	// SIGMA_V is scaled by the length of the move
	double t = DT * (N_CYCLES - this->prev_size);
	normal_distribution<double> dist_v(this->v, SIGMA_V * t);

	// New velocity is a distribution around the current velocity 
	// between 0 and V_MAX
	// Assume constant acceleration over the interval
	double v_new = min(V_MAX, max(dist_v(gen), 0.0));
	double a_new = (v_new - this->v) / t;
	double s_new = this->s + this->v * t + 0.5 * a_new * pow(t, 2);

	return { a_new, v_new, s_new };
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
	/*
	Generates a vector of predicted states for a vehicle with 1 second 
	intervals.
	The length of the prediction is equal to the horizon (in seconds).
	*/
	vector<Vehicle> predictions;
	for (int i = 0; i <= horizon; ++i) {
		predictions.push_back(increment(i));
	}
	return predictions;
}

Vehicle Vehicle::increment(double dt) {
	/*
	Moves a vehicle forward one timestep
	*/
	double a_new = this->a + this->j * dt;
	double v_new = this->v + this->a * dt + 0.5 * this->j * pow(dt, 2);
	double s_new = this->s + this->v * dt + 0.5 * this->a * pow(dt, 2) 
		+ 1.0 / 6 * this->j * pow(dt, 3);

	// constrain the acceleration and velocity to the max allowable values
	a_new = min(a_new, A_MAX);
	v_new = min(v_new, V_MAX);

	// Assume the vehicle will not change lanes
	double d_new = (this->lane + 0.5) * LANE_WIDTH;

	return Vehicle(this->lane, s_new, d_new, this->yaw, v_new, a_new, 
		this->state);
}