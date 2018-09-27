#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = pow(10, 1);
const float EFFICIENCY = pow(10, 5);

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
float lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
	/*
	Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
	Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
	*/
	float cost;
	cost = abs(vehicle.lane - trajectory.back().lane);
	return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
	/*
	Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
	*/

	float cost = (vehicle.target_speed - trajectory[1].v) / vehicle.target_speed;

	return cost;
}
// TODO: evaluate if this function makes sense
float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
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
float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
	/*
	Sum weighted cost functions to get total cost for trajectory.
	*/
	float cost = 0.0;

	//Add additional cost functions here.
	vector< function<float(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &)>> cf_list = { lane_change_cost, inefficiency_cost };
	vector<float> weight_list = { REACH_GOAL, EFFICIENCY };

	for (int i = 0; i < cf_list.size(); i++) {
		float new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions);
		cost += new_cost;
	}

	return cost;

}