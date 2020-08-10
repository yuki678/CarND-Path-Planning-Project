# Path Planning Project
Self-Driving Car Engineer Nanodegree Program
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

<img src="img/simulating.png" width="640" />


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
* Simulator:
  * You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  
  * To run the simulator on Mac/Linux, first make the binary file executable with the following command:
    ```shell
    sudo chmod u+x {simulator_file_name}
    ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Data
#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


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

## Important Notes

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

3. A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---
## Reflection
Path planning involves several components:
##### Sensor Fusion
##### Localization (Input: Sensor Fusion)
##### Prediction (Input: Localization)
##### Behavior (Input: Localization and Prediction)
##### Trajectory (Input: Localization, Prediction and Behavior)
##### Motion Control (Input: Trajectory)


* Each would take different duration and should not wait for input to complete the current cycle, rather work on the latest available data.
* In the trajectory planning, Frenet coordinate system is used where the plane is defined as longitudinal (S) and lateral (D) axes.

### Path Planning Steps in the project
Here are the steps I implemented for the path planning.
1. Analyze sensor fusion data to determine the lanes of each vehicle
2. Find the closest vehicle in each lane, both ahead and behind
3. Calculate S and D distance and velocity
4. Calculate cost of each of the following decisions
   * Continue in the current lane at maximum speed
     * cost = 500 to avoid collision
     * cost = 50 to stay in the middle lane with no car ahead
     * cost = 100 to stay in the right or left lane with no car ahead
     * Add 100 to the cost if the car ahead is less than 150
   * Change to the right lane or change to the left lane
     * cost = 500 when there's no lane
     * cost = 500 when another vehicle is in the right/left lane
     * cost = 250 when another vehicle is within 150(ahead) and 30(behind) distance
     * cost = 125 when there's no vehicle within 150(ahead) and 30(behind) distance
   * Continue in the current lane and slow down to the speed of the car ahead
     * cost = 200 (less than lane change without another vehicle but more than with vehicles)
5. Compare the cost and take the lowest one as decision
6. Based on the decision, calculate the target speed and lane
   * Keep the minimum lateral distance from another vehicle as 3.5m (half of the two vehicle width of 3m + 0.5m as lane width is 4m)
   * Maximum acceleration is set as .224mph to reduce the jerk for both speeding up and down
7. Calculate the future path point using spline function
   1. Initialize the spline by 2 previous points and 3 future points
   2. Calculate target distance 30m ahead as y position
   3. Split into N segments and calculate corresponding y position
   4. Convert the coordinate system from Frenet to Cartesian

