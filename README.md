# Path-Planning-Project
This is a project for Udacity Self-Driving Car Nanodegree program. In this project, I implemented a path planning S/W code for autonomous vehicle control in highway. All codes are written with C++ and tested on the Udacity's brand new simulator. 

## Requirement 
- C++- Udacity simulator : [here](https://github.com/udacity/self-driving-car-sim/releases)
- uWebSocketIO : [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac
- For window users : [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

## Other Important Dependencies
* cmake >= 3.5  
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1  
  * Linux: make is installed by default on most Linux distros  
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)  
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4  
  * Linux: gcc / g++ is installed by default on most Linux distros  
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)  
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
## Run the Project 
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning

## Project Goal
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. I am provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway (about 4.32 mile). Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Data from the simulator
1. Main car's localization Data (No Noise)<br/>
["x"] The car's x position in map coordinates<br/>
["y"] The car's y position in map coordinates<br/>
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH
Previous path data given to the Planner
//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator
Previous path's end s and d values
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value
Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.]
Details
The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.
There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Project Implementation
1) The vehicle model 
- I used kinematic model. In this project, I ignored the tire forces, gravity, vehicle mass to simplify the model. Even though it reduces the accuracy of the model, it approximated well the actual vehicle dynamics at moderate speeds in simulator.

2) State vector and actuator
- I used 4 state vector and 2 actuators, which are x position, y postion, the angle between x-axis and vehicle's heading direction, vehicle speed, steering actuatror and throttle(brake+accel) actuator for each. This 4 state vectors and 2 actuators well approximate the vehicle's behavior in Kinematic model. Steering and throttle actuators can cover almost 90% of the vehicle's movement. (In reality, there will be other actuators such as tranmission(Gear)). I updated the 4 state with the fomulas as follows; (1) x(t+1) = x(t) + v(t) x cos(psi(t)) x dt (2) y(t+1) = y(t) + v(t) x sin(psi(t)) x dt (3) psi(t+1) = psi(t) + (v(t)/Lf) x delta x dt (4) v(t+1) = v(t) + a(t) x dt. Lf means the distance between the front of the vehicle and its center of gravity, and it influences the degree of vehicle's turning angle. Delta stands for the steering angle.   

3) T(Total prediction time), N(Timestep length) and dt (elapsed duration between tiemstep) Value
 - Generally speaking, Large number of T and small number of dt is good. However, if T is too large, the prediction is not accurate because the vehicle environment is changed a lot within a few second. Let's assume that vehicle's average speed is 50Km/h, which means that the vehicle moves 13.8m/sec. I think that 13.8m is really long distance to predict, and I assumed T should not be longer than 1 second. In addition, if dt is too small, it requires too much computational power. So, with the trial and error approach, I found the 0.05 was the optimal value for dt. When dt is larger than 0.05, the accuracy of steering angle was relatively low, which make the sway of the vehicle. Obviously, N is 20. (0.05 second * 20 = 1 second)
 
4) Coordinate conversion 
 - I used the third order polynomial for the waypoints fitted to waypoints to get coefficients. Third order polynomial is enough to represent the waypoints to get coefficient. In this project, the standard of global coordinate (coming from the simulator) and vehicle coordinate was different. While global coordinate uses the traditional X,Y coordinate, the vehicle's heading direction should be always X-axis and vehicle's "current" position is always (0,0) at the vehicle coordinate. Please refer to below image:  
![Test image](https://github.com/KHKANG36/MPC-Project/blob/master/coord.png)
Therefore, we have to rotate the each coordinate as much as the vehicle's current psi angle. There is famous point transform formula. I just used the formula for coordiate rotation. Because the direction of rotation is clockwise, I used the minus sign for the psi angle. After getting the global waypoint coordinates from the simulator (ptsx, ptxy in code), calculate the difference between them and global vehicle coordinate (px,py in code). Then, I rotate it with aforementioned rotation technique, to get the vehicle's coordinate.  

5) Latency handling
 - 100ms latency was given in this project. It means that the time elapsed between when I command a steering angle to when that angle is actually achieved. Therefore, I transmitted the predicted state "after 100ms" to the simulator as the new initial state. In other words, the vehicle's state when actual command is achieved should be the initial state. By replacing the "dt" to "delay(0.1second)" in state prediction formulas, I could handle it easily. In addition, x(t),y(t) and psi(t) value should be all zeros when I calculate predicted state after 100ms because the coordinate will be 0 point, 0 point, 0 angle at the vehicle's view point (after coordinate conversion).  

## Discussion/Issues 
The vehicle in simulator drives pretty well up to about 50Km/h. Over the 50Km/h speed, however, the prediction is getting unstable, and the vehicle sometimes way off the road. I've tuned several parameters (such as weight, N, dt), but didn't get perfect solution. This would be future works to resolve in this project.  
