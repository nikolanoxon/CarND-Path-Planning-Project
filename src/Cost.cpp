#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const double REACH_GOAL = pow(10, 1);
const double EFFICIENCY = pow(10, 5);


/*
Costs for path planner project
minimize lateral jerk
minimize longitudinal jerk
minimize lateral acceleration
minimize longitudinal acceleration
minimize reference speed delta
minimize collisions
minimize lane changes? (not sure if needed)
*/



double lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double>& a_vals) {
	/*
	Cost becomes higher for changing langes
	*/
	// Scale cost between 0 and 1
	double cost = min(fabs(vehicle.lane - trajectory.back().lane),1.0);
	return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double>& a_vals) {
	/*
	Cost becomes higher for driving below the speed limit
	*/

	// If the new speed is negative, set the speed to 0 to keep cost = 1
	double v_new = max(0.0, trajectory.back().v);
	double cost = (V_MAX - v_new) / V_MAX;
	return cost;
}
double jerk_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for trajectories with a higher peak jerk
	*/
	double max_jerk = 0;
	// Iterate across the who cycle and find the peak jerk
	// TODO: Move this calculation to a helper function
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_jerk = max(max_jerk, 6 * a_vals[3] + 24 * a_vals[4] * t + 60 * a_vals[5] * pow(t, 2));
	}
	// Return the cost as a percentage of max jerk
	//TODO: convert this to an exponential cost
	return min(fabs((J_MAX - max_jerk)/J_MAX),1.0);
}
double accel_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for trajectories with a higher peak acceleration
	*/
	double max_accel = 0;
	// Iterate across the who cycle and find the peak acceleration
	// TODO: Move this calculation to a helper function
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_accel = max(max_accel, 2 * a_vals[2] + 6 * a_vals[3] * t + 12 * a_vals[4] * pow(t, 2) + 20 * a_vals[5] * pow(t, 3));
	}

	// Return the cost as a percentage of max jerk
	//TODO: convert this to an exponential cost
	return min(fabs((A_MAX - max_accel) / A_MAX), 1.0);
}
double collision_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{

	double cost = 0;

	double front_s_min = vehicle.s + FRONT_BUFFER;	// meters
	double side_s_max = vehicle.s + SIDE_BUFFER;
	double side_s_min = vehicle.s - SIDE_BUFFER;
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
		// Check if there will be a vehicle in the lane of this trajectory within the minimum distance
		if (it->second[1].lane == trajectory.back().lane && it->second[1].s < front_s_min && it->second[1].s > trajectory.back().s) {
			cost = 1;
		}
		if (trajectory.back().state.compare("KL") == 1) {
			// If we plan to lane change, check if there is currently a vehicle in that lane
			if (it->second[1].lane == trajectory.back().lane && it->second[1].s < side_s_max && it->second[1].s > side_s_min) {
				cost = 1;
			}
		}
	}

	return cost;
}
// TODO: evaluate if this function is needed
double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
	/*
	All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
	we can just find one vehicle in that lane.
	*/
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
		int key = it->first;
		Vehicle vehicle = it->second[0];
		if (vehicle.lane == lane && key != -1 ) {
			return vehicle.v;
		}
	}
	//Found no vehicle in the lane
	return -1.0;
}
double calculate_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> & a_vals) {
	/*
	Sum weighted cost functions to get total cost for trajectory.
	*/
	double cost = 0.0;

	//Add additional cost functions here.
	vector< function<double(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, const vector<double> &)>> cf_list = { collision_cost, lane_change_cost, inefficiency_cost, jerk_cost, accel_cost };
	vector<double> weight_list = { COST_COLLISION, COST_LANE_CHANGE, COST_EFFICIENCY, COST_JERK, COST_ACCEL };

	for (int i = 0; i < cf_list.size(); i++) {
		double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, a_vals);
		cost += new_cost;
	}

	return cost;

}
