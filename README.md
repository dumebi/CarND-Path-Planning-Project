# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Rubic Points

### Files
A new file was added 
No changes were made in the cmake configuration. A new file was added [src/spline.h](https://raw.githubusercontent.com/dumebi/CarND-Path-Planning-Project/master/src/spline.h) the Cubic Spline interpolation implementation to use splines instead of polynomials.

### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident..

I ran the simulator for 11.19 miles without incident
![Training Image](https://raw.githubusercontent.com/dumebi/CarND-Path-Planning-Project/master/path_train.png)

#### The car drives according to the speed limit.
No speed limit error message was seen.

#### Max Acceleration and Jerk are not Exceeded.
No Max jerk error message was seen.

#### Car does not have collisions.
No collisions experienced.

####The car stays in its lane, except for the time between changing lanes.
The car stays in its lane most times except it experiences traffic, or to return to the center lane.

#### The car is able to change lanes
The car changes lane when there is a slow car in front of it, and its safe to change lanes (no cars on lanes around), or to return to the center lane

## Implementation
Implementation is divided into 2 categories

1.  Prediction and Decision based on environmental data

2.  Vehicle tragectory generation


### 1. Prediction and Decision
This step analyzes the localization and sensor fusion data for all cars on the same side of the track.

```
// Prediction.
bool car_ahead = false;
bool car_left = false;
bool car_right = false;
for ( int i = 0; i < sensor_fusion.size(); i++ ) {
    float d = sensor_fusion[i][6];
    int car_lane = -1;
    // is it on the same lane we are
    if ( d > 0 && d < 4 ) {
        car_lane = 0;
    } else if ( d > 4 && d < 8 ) {
        car_lane = 1;
    } else if ( d > 8 && d < 12 ) {
        car_lane = 2;
    }
    if (car_lane < 0) {
        continue;
    }
    // Find car speed.
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][5];
    // Estimate car s position after executing previous trajectory.
    check_car_s += ((double)prev_size*0.02*check_speed);

    if ( car_lane == lane ) {
        // Car in our lane.
        car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;
    } else if ( car_lane - lane == -1 ) {
        // Car left
        car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
    } else if ( car_lane - lane == 1 ) {
        // Car right
        car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
    }
}
```

Here, the positions of other vehicles are analysed relative to our vehicle, if our vehicle is within 30 metres of the vehicle in front, our car_ahead flag is set to true, If vehicles are within the margin on the left or right of our vehicle, car_left or car_right are set to true respectively.

```
// Behavior.
double speed_diff = 0;
const double MAX_SPEED = 49.5;
const double MAX_ACC = .224;
if ( car_ahead ) { // Car ahead
    if ( !car_left && lane > 0 ) {
    // if there is no car left and there is a left lane.
    lane--; // Change lane left.
    } else if ( !car_right && lane != 2 ){
    // if there is no car right and there is a right lane.
    lane++; // Change lane right.
    } else {
    speed_diff -= MAX_ACC;
    }
} else {
    if ( lane != 1 ) { // if we are not on the center lane.
    if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) ) {
        lane = 1; // Back to center.
    }
    }
    if ( ref_vel < MAX_SPEED ) {
    speed_diff += MAX_ACC;
    }
}
```
Here, decisions are made on adjusting speed and changing lanes. If a car is ahead, the lanes to the left and right are checked.
If one of them is empty, the car will change lanes. Otherwise slow down.

The car will move back to the center lane when it becomes clear, this is because a car can move both left and right from the center lane, and it is more likely to get stuck if on the far left or right

If the area in front of the car is clear, the car will speed up.

### 2. Trajectory Generation
This step computes the trajectory of the vehicle from the decisions made above, the vehicle's position, and historical path points.

```
vector<double> ptsx;
vector<double> ptsy;

double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);

// Do I have have previous points
if ( prev_size < 2 ) {
    // There are not too many...
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
} else {
    // Use the last two points.
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
}

// Setting up target points in the future.
vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

// Making coordinates to local car coordinates.
for ( int i = 0; i < ptsx.size(); i++ ) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
}
```
The last two points of the previous trajectory (or the car position if there are no previous trajectory).
The Frenet helper function getXY() is used to generate three points spaced at evenly 30 metres in front of the car

The coordinates are transformed (shift and rotation) to local car coordinates to make them less complicated to the spine calculation

```
// Create the spline.
tk::spline s;
s.set_points(ptsx, ptsy);

// Output path points from previous path for continuity.
vector<double> next_x_vals;
vector<double> next_y_vals;
for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
}

// Calculate distance y position on 30 m ahead.
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x*target_x + target_y*target_y);

double x_add_on = 0;

for( int i = 1; i < 50 - prev_size; i++ ) {
    ref_vel += speed_diff;
    if ( ref_vel > MAX_SPEED ) {
    ref_vel = MAX_SPEED;
    } else if ( ref_vel < MAX_ACC ) {
    ref_vel = MAX_ACC;
    }
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
}
```
The computed waypoints are transformed using a spline. A spline easily computes a smooth trajectory in a 2D space while taking into account acceleration and velocity.

50 waypoints are generated in total. Because the length of the generated trajectory is variable, after the vehicle has assumed the correct position, the rest of the waypoints are generated to keep the vehicle in the target lane.

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

