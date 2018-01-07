# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
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

## Requirements

* Udacity CarND Term 3 Simulator (available at the [releases tab (https://github.com/udacity/self-driving-car-sim/releases) of the Self Driving Car Simulator repository).

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
5. Run the CarnD Term 3 Simulator and observe results.

## Reflection

Path planning is a key role in self driving vehicles, since it aggregates continuous real-world data (sensors) and predetermined rules (driving regulations, comfortable maneuvering, etc.) in order to generate the path the vehicle must pursue. A deterministic path works only in theory; in the real world we need to rely on continuously changing environmental factors and factor those in to make quick and safe decisions. If broken down into smaller pieces, path planning would be the glue that holds together the following concepts:

### Lane management

Knowing the current lane and drivable lanes around the vehicle is a key factor to setup a correct path plan. This is something that the path planner needs to keep track of in order to determine if there’s a more optimal path by changing lanes. In `main.cpp` (lines 286-321), lane is determined by using Frenet coordinates and the lane width, which in this scenario is constant. Another factor to keep in mind is that the simulation only accounts for three lanes constantly. In the real world, this changes constantly depending on the road (highway, urban street, off-road, etc).

Having lanes around the vehicle is not enough. The data provided by sensors enables the program to determine if it’s possible to shift the vehicle into a new lane without colliding with other fellow vehicles. This is handled throughout lines 313-345.

### Distance management

Distance management is crucial to prevent head-on collisions and lateral impact when changing lanes. Sensor fusion data allows the program to determine if it has to modify its speed to prevent hitting another vehicle in front, or if it should wait a little bit before shifting lanes. This is done throughout lines 313-320.

### Speed management

Speed management is important in order to comply with driving regulations (speed limits) and to execute evasive or overtaking maneuvers, all while providing a comfortable experience to the passengers on-board. Lines 348-355 handle acceleration and deceleration based on the distance with the vehicle in front.

---

After setting up these areas of interest, execution is carried out by constantly generating the best possible path (437-456) based on coordinate calculations (364-434) that execute smooth speed and lane changes. It’s worth mentioning that the Spline library suggested by the guidelines allows the vehicle to safely and gradually move to a desired lane without causing discomfort to passengers on-board.

## Results

After several iterations, I was able to achieve the project rubric standards by driving 4.32 miles without a single incident. Screenshots included below and in the `results` directory.

![Milestone](https://gtoran.github.io/repository-assets/CarND-Path-Planning-Project/milestone.jpg)

I kept the vehicle driving around and more than doubled the required distance:

![Milestone x2](https://gtoran.github.io/repository-assets/CarND-Path-Planning-Project/milestone-2.jpg)

I finally ended up having a slight graze with a vehicle after 21 miles:

![Top Distance](https://gtoran.github.io/repository-assets/CarND-Path-Planning-Project/maximum.jpg)

---

## Useful information

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.
