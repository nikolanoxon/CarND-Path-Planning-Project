#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

double lane_change_cost(const Vehicle & vehicle, 
	const vector<Vehicle> & trajectory, 
	const map<int, vector<Vehicle>> & predictions, 
	const vector<double>& a_vals) {
	/*
	Cost becomes higher for changing langes
	*/
	// Scale cost between 0 and 1
	double cost = min(fabs(vehicle.lane - trajectory.back().lane),1.0);
	return cost;
}

double inefficiency_cost(const Vehicle & vehicle, 
	const vector<Vehicle> & trajectory, 
	const map<int, vector<Vehicle>> & predictions, 
	const vector<double>& a_vals) {
	/*
	Cost becomes higher for driving below the speed limit
	*/

	// If the new speed is negative, set the speed to 0 to keep cost = 1
	double v_new = max(0.0, trajectory.back().v);
	double cost = (V_MAX - v_new) / V_MAX;
	return cost;
}

double jerk_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, 
	const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for trajectories with a higher peak jerk
	*/
	double max_jerk = 0;
	// Iterate across the who cycle and find the peak jerk
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_jerk = max(max_jerk, 6 * a_vals[3] + 24 * a_vals[4] * t 
			+ 60 * a_vals[5] * pow(t, 2));
	}
	// Return the cost as a percentage of max jerk

	double cost = min(fabs(max_jerk) / J_MAX, 1.0);

	return cost;
}

double accel_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, 
	const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for trajectories with a higher peak acceleration
	*/
	double max_accel = 0;
	// Iterate across the who cycle and find the peak acceleration
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_accel = max(max_accel, 2 * a_vals[2] + 6 * a_vals[3] * t 
			+ 12 * a_vals[4] * pow(t, 2) + 20 * a_vals[5] * pow(t, 3));
	}

	// Return the cost as a percentage of max jerk
	double cost = min(fabs(max_accel) / A_MAX, 1.0);

	return cost;
}

double speed_cost(const Vehicle & vehicle, const vector<Vehicle>& trajectory, 
	const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for trajectories that overspeed or are at standstill
	*/
	
	double max_speed = 0;
	// Iterate across the who cycle and find the peak acceleration
	for (int i = 0; i < N_CYCLES - vehicle.prev_size; ++i) {
		double t = i * DT;
		max_speed = max(max_speed, a_vals[1] + 2 * a_vals[2] * t 
			+ 3 * a_vals[3] * pow(t, 2) + 4 * a_vals[4] * pow(t, 3) 
			+ 5 * a_vals[5] * pow(t, 4));
	}

	// Apply a cost if the velocity overspeeds
	double cost = 0;

	if (max_speed >= V_MAX || max_speed == 0.0) {
		cost = 1;
	}
	return cost;
}

double free_lane_cost(const Vehicle & vehicle,
	const vector<Vehicle>& trajectory,
	const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for driving in an occupied lane
	*/
	double next_s = trajectory.back().s;
	double next_lane = trajectory.back().lane;

	vector<double> min_s = { DBL_MAX, DBL_MAX , DBL_MAX };

	// Find the closest vehicle in each lane
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
		it != predictions.end(); ++it) {
		double fwd_veh_lane = it->second.back().lane;
		double fwd_veh_s = it->second.back().s;

		// Filter out vehicles in unassigned lanes
		if (fwd_veh_lane >= LEFT_LANE) {
			if (fwd_veh_s > next_s && fwd_veh_s < min_s[fwd_veh_lane]) {
				min_s[fwd_veh_lane] = fwd_veh_s;
			}
		}
	}

	// Identify the lane with the furthest vehicle
	vector<double>::iterator best_s = max_element(begin(min_s), end(min_s));
	int best_lane = distance(begin(min_s), best_s);

	// Apply a cost for driving further from the best lane
	// LANES_AVAILABLE is offset by 1 to account for lane index
	double cost = fabs(next_lane - best_lane) / (LANES_AVAILABLE - 1);
	// Return the average of the two costs as a combined collision cost
	return cost;
}

double collision_cost(const Vehicle & vehicle, 
	const vector<Vehicle>& trajectory, 
	const map<int, vector<Vehicle>>& predictions, const vector<double>& a_vals)
{
	/*
	Cost becomes higher for following too closely behind a forward vehicle, having
	a higher speed than a forward vehicle, and driving next to a vehicle
	*/
	double speed_cost = 0;
	double dist_cost = 0;
	double dist_f_cost = 0;
	double dist_sf_cost = 0;
	double dist_sr_cost = 0;

	// Ego and other vehicle positions
	double front_s_min = vehicle.s + FWD_HORIZON;
	double side_s_max = vehicle.s + SIDE_F_BUFFER;
	double side_s_min = vehicle.s - SIDE_R_BUFFER;
	double next_s = trajectory.back().s;
	double next_v = trajectory.back().v;
	double next_lane = trajectory.back().lane;

	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); 
		it != predictions.end(); ++it) {
		// Check if there will be a vehicle in the lane of this trajectory 
		// within the minimum distance
		double veh_next_lane = it->second.back().lane;
		double veh_next_s = it->second.back().s;
		double veh_next_v = it->second.back().v;
		double veh_ref_lane = it->second.front().lane;
		double veh_ref_s = it->second.front().s;

		if ((veh_next_lane == next_lane) && (veh_next_s < front_s_min)
			&& (veh_next_s > next_s)) {
			
			// Cost is proportional to the difference in speed to the near 
			// vehicle
			speed_cost = min(max(next_v - veh_next_v, 0.0) / veh_next_v, 1.0);
			dist_f_cost = 1 - min(max((veh_next_s - FWD_BUFFER) - next_s, 0.0)
				/ FWD_HORIZON, 1.0);

		}
		if (trajectory.back().state.compare("KL") == 1) {
			// If we plan to lane change, check if there is currently a vehicle
			// in that lane
			if (veh_ref_lane == next_lane) {
				if ((veh_ref_s > vehicle.s) && (veh_ref_s <= side_s_max)) {
					dist_sf_cost = 1 - fabs(vehicle.s - veh_ref_s) / SIDE_F_BUFFER;
				}
				else if ((veh_ref_s <= vehicle.s) && (veh_ref_s >= side_s_min)) {
					dist_sr_cost = 1 - fabs(vehicle.s - veh_ref_s) / SIDE_R_BUFFER;
				}
			}
		}
		dist_cost = max(dist_f_cost, max(dist_sf_cost, dist_sr_cost));
	}
	// Return the average of the two costs as a combined collision cost
	return 0.5 * (dist_cost + speed_cost);
}

double calculate_cost(const Vehicle & vehicle, 
	const vector<Vehicle> & trajectory, 
	const map<int, vector<Vehicle>> & predictions, 
	const vector<double> & a_vals) {
	/*
	Sum weighted cost functions to get total cost for trajectory.
	*/
	double cost = 0.0;

	//Cost functions
	vector< function<double(const Vehicle &, const vector<Vehicle> &, 
		const map<int, vector<Vehicle>> &, const vector<double> &)>> 
		cf_list = { free_lane_cost, speed_cost, collision_cost, lane_change_cost,
		inefficiency_cost, jerk_cost, accel_cost };
	vector<double> weight_list = { COST_FREE_LANE, COST_SPEED, COST_COLLISION, 
		COST_LANE_CHANGE, COST_EFFICIENCY, COST_JERK, COST_ACCEL };

	// Sum the cost of all the cost functions
	for (int i = 0; i < cf_list.size(); i++) {
		double new_cost = 
			weight_list[i] * cf_list[i](vehicle, trajectory, predictions, 
				a_vals);
		cost += new_cost;
	}

	return cost;
}
