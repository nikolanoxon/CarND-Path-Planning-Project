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

default_random_engine gen;

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, double s, double d, double yaw, double v, double a, string state) {
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
	double cost;
	vector<double> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {

		// Generating random predictions
		for (int i = 0; i < N_TRAJECTORIES; ++i) {
			vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
			if (trajectory.size() != 0) {

				vector<double> start = { 0.0, trajectory.front().v, trajectory.front().a };
				vector<double> end = { trajectory.back().s, trajectory.back().v, trajectory.back().a };
				double T = HORIZON;
				vector<double> a_vals = JMT(start, end, T);


				cost = calculate_cost(*this, trajectory, predictions, a_vals);
				costs.push_back(cost);
				final_trajectories.push_back(trajectory);
				cout << "state: " << *it << "   cost: " << cost << endl;
			}
		}
	}

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
	/*
	else if (state.compare("PLCR") == 0) {
		if (lane != RIGHT_LANE) {
			states.push_back("PLCR");
			states.push_back("LCR");
		}
	}
	else if (state.compare("PLCL") == 0) {
		if (lane != LEFT_LANE) {
			states.push_back("PLCL");
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

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
	/*
	INPUT: A predictions map of the trajectories of other vehicles
	OUTPUT: A waypoint 5 seconds in the future
	*/

	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };
	
	vector<double> kinematics = gaussian_move();

	double a_new = kinematics[0];
	double v_new = kinematics[1];
	double s_new = kinematics[2];
	
	double d_new = this->lane_position[this->lane];

	trajectory.push_back(Vehicle(this->lane, s_new, d_new, this->yaw, v_new, a_new, this->state));
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };

	vector<double> kinematics = gaussian_move();

	double a_new = kinematics[0];
	double v_new = kinematics[1];
	double s_new = kinematics[2];

	double d_new = this->lane_position[this->lane];

	int new_lane = this->lane + lane_direction[this->state];

	d_new = this->lane_position[new_lane];

	trajectory.push_back(Vehicle(new_lane, s_new, d_new, this->yaw, v_new, a_new, this->state));
	return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->d, this->yaw, this->v, this->a, this->state) };

	vector<double> kinematics = gaussian_move();

	double a_new = kinematics[0];
	double v_new = kinematics[1];
	double s_new = kinematics[2];

	double d_new = this->lane_position[this->lane];

	int new_lane = this->lane + lane_direction[this->state];

	d_new = this->lane_position[new_lane];

	trajectory.push_back(Vehicle(new_lane, s_new, d_new, this->yaw, v_new, a_new, this->state));
	return trajectory;
}

Vehicle Vehicle::increment(double dt) {
	/*
	Moves a vehicle forward one timestep
	*/
	double a_new = this->a + this->j * dt;
	double v_new = this->v + this->a * dt + 0.5 * this->j * pow(dt, 2);
	double s_new = this->s + this->v * dt + 0.5 * this->a * pow(dt, 2) + 1.0 / 6 * this->j * pow(dt, 3);

	// constrain the acceleration and velocity to the max allowable values
	a_new = min(a_new, A_MAX);
	v_new = min(v_new, V_MAX);


	double d_new = (this->lane + 0.5) * LANE_WIDTH;

	return Vehicle(this->lane, s_new, d_new, this->yaw, v_new, a_new, this->state);
}

vector<double> Vehicle::gaussian_move() {
	
	
	// New velocity is a distribution around the current velocity between 0 and V_MAX
	normal_distribution<double> dist_v(this->v, SIGMA_V);

	// Assume constant acceleration over the interval

	double t = DT * (N_CYCLES - this->prev_size);

	double v_new = min(V_MAX, max(dist_v(gen), 0.0));

	double a_new = (v_new - this->v) / t;

	double v_avg = 0.5 * (v_new + this->v);
	double a_avg = 0.5 * (a_new + this->a);

	double s_new = this->s + v_avg * t + 0.5 * a_avg * pow(t, 2);
	
	/*
	normal_distribution<double> dist_a(0, SIGMA_A);
	normal_distribution<double> dist_v(0, SIGMA_V);
	normal_distribution<double> dist_s(0, SIGMA_S);

	double dt = DT * (N_CYCLES - this->prev_size);

	double a_new = this->a;
	double v_new = this->v + this->a * dt;
	double s_new = this->s + this->v * dt + 0.5 * this->a * pow(dt, 2);

	double delta_a = dist_a(gen) * dt;
	double delta_v = dist_v(gen) * dt;
	double delta_s = dist_s(gen) * dt;

	a_new += delta_a;
	v_new += delta_v;
	s_new += delta_s;
	*/
	return { a_new, v_new, s_new };
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	return false;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	return false;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
	/*
	Generates a vector of predicted states for a vehicle with 1 second intervals.
	The length of the prediction is equal to the horizon (in seconds).
	*/
	vector<Vehicle> predictions;
	for (int i = 0; i <= horizon; ++i) {
		predictions.push_back(increment(i));
	}
	return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
}

void Vehicle::configure(vector<int> road_data) {
}
