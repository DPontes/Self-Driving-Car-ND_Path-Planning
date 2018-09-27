# Path Planning Project

This project consists of a C++ implementation of a Path Planner to control the car using Udacity's self-driving simulator. The main goal of this project is to develop a C++ path planner that successfully navigates the vehicle around the virtual highway (Udacity simulator) without colliding with any other vehicle, repecting the speed limit of 50mph and with maximum acceleration and jerk of 10m/s^2 and 50m/s^2, respectively.

## Files

The following files are part of this project:
* main.cpp: main file that integrates the controller with the simulator;
* vehicle.cpp: vehicle class which defines the Finite State Machine;
* cost_function.cpp: cost_function class to compute the optimal path;

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

## Compiling and running

The main program can be built and run by doing the following from the project top directory

1. mkdir build && cd build
2. cmake .. && make
3. ./path_planning
4. Run the Udacity Simulator (./udacity_term3_simulator)
5. Press "Select" in the simulator
6. Enjoy!

## Behaviour Planning

The behaviour planner is responsible to generate the path and the speed to be followed by the car. From a high-level perspective, the planner has to decide if the car should stay or change lane.
From a low level, the planner has to generate the map's coordinates (path) to be followed and decide at which speed the car should run. In order to achieve such a behaviour, our self-driving car implements a Finite State Machine (FSM) with 5 states, which the transition is realized through the minimization of a cost function.
Finally, the path is generated using the spline math tool.

#### Accronyms

* KL - Keep Lane
* PLCL - Prepare Lane Change Left
* PLCR - Prepare Lane Change Right
* LCL - Lane Change Left
* LCR - Lane Change Right

### Finite State Machine

The FSM starts with a KL state. Depending on the system context (highway), the FSM may stay at KL state or change to PLCL or PLCR. At each state, all possible states are evaluated using a cot function and the state with the minimum cost is selected. The FSM machine works as follows:

* At initial state (KL) and according to the cost function, the FSM can stay at the same lane or prepare  to change lanes (PLCL or PLCR). However, only the possible lane is available if the car is in one of the lateral lanes (lanes 0 or 2); The car will stay in the same (KL) if there is no other vehicle that prevents it from reaching the maximum legal speed limit of the road.
* If the PLCL or PLCR are selected, the car prepares to change lane. The preparation checks if the car speed and the buffer space are safe to change lanes. The car may stay in the PLCL or PLCR state until the buffer is safe enough to change lane or even decide to return to state KL if the cost to change the lane is no longer relevant.
* When there is enough buffer space to change lane, the FSM will transition to LCL/LCR states. The FSM returns to state KL as soon the lane change is over (car is between the lane lines).

### Cost Function

The cost function evaluates the cost for the FSM to change state. It evaluates different metrics trying to identify the next optimal state. Below, the metrics and how they are evaluated are presented in detail:

1. Change Lane:
The change lane cost function adds a "confort" constant penalty if the vehicle decides to change lane.

```cpp
if (start_lane != end_lane) {
    cost += COMFORT;
}
```

2. Buffer:
The buffer cost function computes how long it has to the vehicle in front. It is computed by dividing the distance from the vehicle in front by the current speed of the ego car. Note that the cost is smaller if the vehicle in question is behind. It helps the ego car to choose a lane with no traffic in the front.

```cpp
double time_steps = abs (vehicle->collider.closest_approach)/(abs(vehicle->speed) * MPH_TO_MS);

if (time_steps > DESIRED_BUFFER) {
    return 0;
}

double multiplier = 1.0 - pow((time_steps / DESIRED_BUFFER), 2);
cost = multiplier * DANGER;
if (vehicle->collider.closest_approach < 0) {
    // car in the back
    cost /= 10;
}
```

3. Inefficiency:
This function evaluates the vehicle speed defined in this state in relation to the maximum speed alloed. States with speeds closer to the maximum speed are more efficient (lower cost), in contrast, states with lower speeds are less efficient (higher cost).

```cpp
double diff = (49.5 - vehicle->update.target_v) / 49.5;
cost = pow(diff, 2) * EFFICIENCY;
```

4. Target:
The target cost evaluates the speed comparison between the ego car and the vehicle in front (possible collision). If all lanes are blocked (possible collision), this function helps the car to choose a lane which the speed of the vehicle in question matches closer to the ego car.

```cpp
if (!vehicle->collider.collision) {
    // no possible collision, no cost
    return 0;
}

double diff = (vehicle->collider.target_speed - vehicle->speed) / vehicle->collider.target_speed;
cost = pow(diff, 2) * EFFICIENCY;
```

5. Collision:
The collision cost is the most important function. If strongly penalizes the states which the risk of collision is more imminent. However, in order to force the car to escape from heavy traffic, the collision cost is smaller whenever is safe to change lane. It was found that it helps the car to find a more appropriate situation, instead of just following the car ahead until it opens a passageway.

```cpp
double time_to_collide = abs (vehicle->collider.distance) / (abs(vehicle->speed) * MPH_TO_MS);
cost = exp(-pow(time_to_collide, 2)) * COLLISION;
if (vehicle->trajectory.lane_end != vehicle->trajectory.lane_start) {
    if(time_to_collide > DESIRED_BUFFER) {
        // safe to change lane
        cost /= 10;
    }
}
```

### Path Generation and Speed Control
The FSM defines, from a high-level point of view, the lane and the speed to be followed for every state. For instance, if the car is changing lane, the FSM will return a different one from the current lane. The same is for speed, the FSM will return for every state the target speed the ego car should follow.

```cpp
vehicle.Update(car_x, car_y, car_s, car_d, car_yaw, car_speed, lane, ref_velocity, prev_size*.02);
vehicle.NextState(sensor_fusion);
// new lane
lane = vehicle.update.lane;
// target speed
ref_velocity = vehicle.update.ref_v;
```

After the FSM returns the lane and speed to follow, the controller generates up to 50 map coordinates back to the simulator. These points define the path to be followed by the car.

#### Acceleration and Jerk control
The requirements of this project state that the acceleration and jerk should not exceed 10m/s^2 and 50m/s^2, respectively. In order to meet this requirement, the car acceleration is increased or decreased by steps of steps 0.224m/s^2. The limit of 0.224 is computed as follows:
max acceleration of 10 m/s², with delta time of 0.02 seconds in miles per hour:  ![equation](http://latex.codecogs.com/gif.latex?%5Cfrac%7B2.24%7D%7B10%7D)

```cpp
if (!collider.collision && ref_speed < update.target_v && ref_speed < 49.5) {
    update.ref_velocity += 0.224;
} else if (ref_speed > update.target_v && ref_speed > 0) {
    update.ref_velocity -= 0.224;
}
```

#### Path: Spline
Spline is a piecewise polynomial parametric curve. They are popular for their simplicity use and accuracy. Our path planner uses the Spline methematical function for curve fitting the generated map coordinates. The spline helps to define a smooth path for the car.
The path generation is an elaborate st of tasks. First, our planner has to generate equally spaced map coordinates. We use the helper function "getXY" to generate points from Frenet to Cartesian coordinates.

```cpp
// In Frenet
vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

After, we shift the orientation to the ego for simplicity and feed the points to the spline generator. We used the [Cubic Spline library ](http://kluge.in-chemnitz.de/opensource/spline/) to generate the spline curve.

```cpp
for (int i = 0; i < pysx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = (shift_x * sin(0-ref_yaw) - shift_y * cos(0 - ref_yaw);
}

// Create a spline
tk::spline s;

// set (x,y) points to the spline
s.set_points(ptsx, ptsy);
```

After, with the spline function already done, we have to recompute the map points back from the curve. This task is accomplished by breaking up the spline into equidistant points that respect the desired speed.

![Spline][images/spline.png]

Considering the time interval of 20ms, the travel distance of 30 meters on x-axis with the speed "ref_speed", we have:

```cpp
// calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x * target_x + target_y * target_y);

// N distance
double N = target_dist / (.02 * ref_velocity);
```
