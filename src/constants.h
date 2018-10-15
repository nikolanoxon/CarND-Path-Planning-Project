#ifndef CONSTANTS
#define CONSTANTS

#define DT  0.02 // cycle time
#define N_CYCLES 50 // number of waypoints in the trajectory planner
#define N_TRAJECTORIES 20 // number of trajectories to generate for each possible state
#define UPDATE_RATE 0.2 // Update rate of the trajectory planner (s)

#define V_MAX 21.5 // speed limit (m/s)
#define A_MAX 5.0 // max acceleration (m/s^2)
#define J_MAX 8.0 // max jerk (m/s^3)

#define FWD_HORIZON 20.0 // Distance to look ahead for collision checking (m)
#define FWD_BUFFER 6.0 // minimum forward buffer between vehicles (m)
#define SIDE_F_BUFFER 6.0 // minimum buffer between vehicles to the side and in front (m)
#define SIDE_R_BUFFER 8.0 // minimum buffer between vehicles to the side and behind (m)
#define S_ADJUST_LC 20.0 // The additional s distance allowed to complete a lane change

#define LANES_AVAILABLE 3 // number of lanes
#define LEFT_LANE 0 // left lane index
#define MID_LANE 1 // middle lane index
#define RIGHT_LANE 2 // right lane index
#define LANE_WIDTH 4.0 // width of each lane (m)
#define LEFT_LANE_CENTER 2 // d value of left lane center (m)
#define MID_LANE_CENTER 6 // d value of middle lane center (m)
#define RIGHT_LANE_CENTER 10 // d value of right lane center (m)
#define S_MAX 6945.554 // Length of the track

#define SIGMA_V 3 // standard deviation of change of velocity per second (m/s^2)

#define COST_JERK 1 // Cost for driving above jerk limit
#define COST_ACCEL 5 // Cost for driving above accel limit
#define COST_SPEED 1 // Cost for driving above speed limit
#define COST_FREE_LANE 2 // Cost for not driving in the most open lane
#define COST_COLLISION 100 // Cost for hitting another vehicle
#define COST_LANE_CHANGE 1 // Cost for changing lanes
#define COST_EFFICIENCY 10 // Cost for driving below speed limit


#endif /* CONSTANTS */