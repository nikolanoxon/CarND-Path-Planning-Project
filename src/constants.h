#ifndef CONSTANTS
#define CONSTANTS

#define DT  0.02 // cycle time
#define N_CYCLES 50 // number of waypoints in the trajectory planner
#define UPDATE_RATE 0.5 // Update rate of the trajectory planner (s)
#define HORIZON 1 // outlook horizon (s)
#define N_TRAJECTORIES 20 // number of trajectories to generate

#define V_MAX 22.0 // speed limit (m/s)
#define A_MAX 10.0 // max acceleration (m/s^2)
#define J_MAX 10.0 // max jerk (m/s^3)

#define FRONT_BUFFER 30.0 // minimum buffer between vehicles
#define SIDE_BUFFER 5.0 // minimum buffer between vehicles

#define LANE_WIDTH 4.0 // with of each lane (m)
#define LANES_AVAILABLE 3 // number of lanes
#define LEFT_LANE 0 // left lane index
#define MID_LANE 1 // middle lane index
#define RIGHT_LANE 2 // right lane index
#define LEFT_LANE_CENTER 2 // d value of left lane center (m)
#define MID_LANE_CENTER 6 // d value of middle lane center (m)
#define RIGHT_LANE_CENTER 10 // d value of right lane center (m)

#define SIGMA_S 1 // standard deviation of position (m)
#define SIGMA_V 1 // standard deviation of velocity (m/s)
#define SIGMA_A 1 // standard deviation of acceleration (m/s^2)

#define COST_JERK 10
#define COST_ACCEL 100
#define COST_COLLISION 100
#define COST_LANE_CHANGE 1
#define COST_EFFICIENCY 10


#endif