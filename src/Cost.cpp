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



// TODO: change this to penalize any lane change
double lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double>& a_vals) {
	/*
	Cost becomes higher for changing langes
	*/
	double cost;
	// Scale cost between 0 and 1
	cost = min(fabs(vehicle.lane - trajectory.back().lane),1.0);
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
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_jerk = max(max_jerk, 6 * a_vals[3] + 24 * a_vals[4] * t + 60 * a_vals[5] * pow(t, 2));
	}

	// Return the cost as a percentage of max jerk
	return min(fabs((J_MAX - max_jerk)/J_MAX),1.0);
}
double accel_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for trajectories with a higher peak acceleration
	*/
	double max_accel = 0;
	// Iterate across the who cycle and find the peak acceleration
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_accel = max(max_accel, 2 * a_vals[2] + 6 * a_vals[3] * t + 12 * a_vals[4] * pow(t, 2) + 20 * a_vals[5] * pow(t, 3));
	}

	// Return the cost as a percentage of max jerk
	return min(fabs((A_MAX - max_accel) / A_MAX), 1.0);
}
double collision_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{

	double s_min = vehicle.s + 50;	// meters
	double cost = 0;

	// Check if there will be a vehicle in this lane within the minimum distance
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
		if (it->second[1].lane == vehicle.lane && it->second[1].s < s_min && it->second[1].s > vehicle.s) {
			cost = 1;
		}
	}

	return cost;
}
// TODO: evaluate if this function makes sense
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
// TODO: change this function to receive data from the simulator
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
