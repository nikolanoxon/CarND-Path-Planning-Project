#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double calculate_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> &a_vals);

double lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> &a_vals);

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> &a_vals);

double jerk_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> &a_vals);

double accel_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> &a_vals);

double collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, const vector<double> &a_vals);

double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

#endif /* COST_H */