# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Model Documentation

The path planner I developed for this project followed a traditional architecture for a SDC, albeit with some aspects provided by the simulator. The throughput for my architecture is as follows:
* Sensor fusion (from simulator) [~50 Hz]
* Localization (from simulator) [~50 Hz]
* Behavior Generation
    * Prediction [~50 Hz]
    * Trajectory Generation 1 [5 Hz]
    * Cost Evaluation [5 Hz]
    * Vehicle State Update [5 Hz]
    * Trajectory Generation 2 [5 Hz]
* Motion Control (to simulator) [5 Hz]

### Sensor Fusion / Localization (main.cpp lines 266-340)

This data was passed from the simulator via the uWebSockets protocol. No plausibilization or error checking was done on this data and it was assumed for the purpose of this model that the signals were noiseless.

By checking the historical path from the simulator, the ego vehicle acceleration can be determined which is needed for the behavior generation (main.cpp lines 273-340).

### Behavior Generation (main.cpp line 369)

The behavior generation is where the bulk of the work was performed for this project. This was broken up into several sub-modules which are described below. Before these sub-modules are executed, the historical data from the simulator is used to assign the anchor points for the trajectory generator, as well as preloading the un-executed path from the motion controller. These tasks help to smooth the trajectory and minimize the jerk (main.cpp lines 343-366).

#### Prediction (main.cpp lines 373-401)

The first step is to predict the future state of the other road participants. Since no acceleration data or historical data is provided, it is assumed that the vehicles are travelling at a constant speed. Additionally it is assumed that the vehicles maintain their lane, however this could be changed to improve the prediction step.

Using the kinematic information from the sensor fusion, the state of each vehicle is predicted 1 second into the future (main.cpp line 397)

#### Trajectory Generation 1 (main.cpp lines 409-429)

The trajectory generation operates at 5 Hz to reduce the computational complexity and prevent executing a new trajectory too quickly.

With all the relevent information known, trajectories are created for up to three possible maneuvers: Keep Lane, Lane Change Left, and Lane Change Right. The maneuvers available depend on the current vehicle state (see Vehicle State Selection for more details).

For each maneuver I generate 20 trajectories with a constant accceleration value selected from a gaussian distribution with a mean of 0 and a standard deviation of 3 m/s^2 (vehicle.cpp line 190). This helps to ensure that all the trajectories have reasonable acceleration values while exploring a dense, random path.


#### Cost Evaluation (vehicle.cpp line 77)

All 60 trajectories are fed into the following cost functions which can be found in cost.cpp:

|Weight|Cost|Description|
|---|---|---|
|Jerk|1|Higher cost for trajectories with greater maximum jerk|
|Acceleration|5|Higher cost for trajectories with greater maximum acceleration|
|Speed|1|Binary cost penalty for exceeding the speed limit|
|Free Lane|2|Cost for not driving in the most free lane|
|Lane Change|1|Cost for performing a lane change|
|Collision|100|Cost for trajectories which intersect another vehicle|
|Efficiency|10|Cost for driving below the speed limit|

Some of these cost functions complement each other (jerk & acceleration), while others are in direct opposition (free lane & lane change). I found that balancing these cost functions was one of the more difficult and nuanced aspects of the project.

The lane change cost function was included to prevent the vehicle from changing lanes unless it was neccessary (like in case of a collision, or a free lane was found).

The free lane cost function I am particularly proud of because of how it is able to handle more complex situations. For example, if the vehicle is 2 lanes away from the best lane, the cost function will encourage a lane change even though the next lane over is not otherwise and preferable. Additionally, if the vehicle is in the center lane and needs to make a lane change, it will prioritize changing into the most free lane.

The collision cost function looks at vehicles ahead and to the side. For vehicles ahead, it seeks to match speed at a comfortable distance and ends up acting as an ACC controller. The side collision checker simply applies a cost for maneuvers which get too close to vehicles in adjacent lanes.

For each possible trajectory, the sum of all the costs is calculated and then the trajectory with the lowest combined cost is selected.

#### Vehicle State Update

The behavior generation follows a simple state space model to determine the available maneuvers at any given time. 

![State Diagram](/state_diagram.png)

When the simulator starts, the vehicle is in the Keep Lane state. From the Keep Lane state, the trajectory generator can select any of the 3 maneuvers. From the Lane Change Left/Right states, the trajectory generator can choose to continue to change the lane or keep the current lane. The vehicle state returns to Keep Lane once the "next lane" output by the trajectory generator is equal to the current lane.

#### Trajectory Generation 2

The output of the Cost Evaluation is the desired vehicle state at the end of the trajectory horizon (which is somewhere between 0-1 seconds ahead).

The trajectory generation creates 2 waypoint behind the vehicle (called anchor points) and 3 waypoints ahead of the vehicle which are spaced according to the maneuver being executed.
* If it is a Keep Lane maneuver, the waypoints are spaced 30, 60, and 90 meters in front of the vehicle in the same lane. This is done to ensure that the waypoints closely follow the s & d values as the vehicle rounds corners.
* If it is a Lane Change maneuver, the waypoints are spaced 45, 75, and 105 meters in front of the vehicle in an adjacent lane. This is done to mitigate the lateral acceleration/jerk during a lane change.

The 2 anchor points are extracted from the previous trajectory and added to the 3 new ones. Having 2 anchor points guarentees smoothness when these waypoints are converted to a spline in the motion control step.

### Motion Control

The output of the trajectory generation is the 5 waypoints previously described (2 of which are behind the vehicle). The spline library mentioned previously was used to create a spline from these waypoints (main.cpp line 497).

In parallel, the initial and final state sent from the trajectory generation are fed into a Jerk Minimum Trajectory generator which outputs the coefficients to a quintic polynomial that describes the jerk-optimal s-values over time.

By feeding these s-values into the spline mentioned previously with a DT = 20ms, a list of s & d values at 20 ms intervals is generated. This is then converted into global coordinates and is output to the simulator.

## Results

Overall the model achieved very good results and was often able to complete several laps without incident while maintaining close to top speed, the best result being 31.73 miles.

![Best Record](/path_planning_31.73.PNG)

The model does have some issues with higher density traffic situations. I weighted the cost functions in a slightly agressive manner that probably result in more frequent collisions, however it often maneuvers between tight traffic patterns and improve its average speed.

If I were to make some improvements, I would probably make the collision cost function more conservative. Additionally I would include a lateral jerk controller that ensures there is no overshoot during a lane change (which occasionally occurs).

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

