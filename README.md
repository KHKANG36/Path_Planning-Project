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
["s"] The car's s position in frenet coordinates<br/>
["d"] The car's d position in frenet coordinates<br/>
["yaw"] The car's yaw angle in the map<br/>
["speed"] The car's speed in MPH<br/>
["previous_path_x"] The previous list of x points previously given to the simulator<br/>
["previous_path_y"] The previous list of y points previously given to the simulator<br/>
["end_path_s"] The previous list's last point's frenet s value<br/>
["end_path_d"] The previous list's last point's frenet d value<br/>
2. Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)<br/>
["sensor_fusion"] A 2d vector of cars and then that car's (car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.)

## Details 
The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points we have used so we can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. we would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## My Project Implementation
1) Simulation Result 
- I was able to drive one lap (about 4.32miles) without incident. At the below picture, the top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes.<br/>
![Test image](https://github.com/KHKANG36/Path_Planning-Project/blob/master/Result1.png)

2) Vehicle speed
- The car didn't drive faster than the speed limit (50MPH). Also the car isn't driving much slower than speed limit unless obstructed by traffic. Initially, I set initial velocity 0 MPH (line 218). Then, it slowly acceralate the speed to 49.5MPH (line 327~329) if there is no traffic obstraction. If there is it (if my vehicle is too close front vehicle), I reduce the speed by 0.24/0.02sec not to violate the jerk limitation (line 306) until my vehicle is not too close to front vehicle. When My vehicle is enough(safely) far from the front vehicle, I acceralate the speed to 49.5MPH by increasing the speed 0.28/0.02sec not to violate the jerk limitation again (line 329). I make the acceralation speed a little bit higher than braking speed because it makes my vehicle is able to catch-up the traffic as fast as possible.  

3) Acceleration & Jerk 
 - As I explained section 2), I could not violate the acceleration limit (10 m/s^2) and the jerk limit (10 m/s^3). In terms of lane change acceleration & jerk, I mostly used the "spline" library introducted in lecture and project walkthrough. From line 332 to line 434, I implemented the waypoint generation technique using spline library. After setting the evenly 25m spaced sparse point (total 75m), I filled the dense waypoints using spline library. With this, the vehicle was able to move smoothly while not only driving but changing the lane.  
 
4) Collision and lane change 
 - As explained section 2), the vehicle basically reduce the speed when it is too close to the front vehicle. However, the vehicle should be able to change the lane to avoid front traffic obstruction. From line 272, the for loop calculate the speed, location (S Frenet) and lane number of all the other vehicles. If the gap between front vehicle in candidate lane for lane change and my vehicle is within 23m, I judged it is impossible to change toward the lane. Similarly, If the gap between behind vehicle in candidate lane for lane change and my vehicle is within 7m, I also judged it is impossible to change toward the lane. Obviously, the gap between my vehicle and front vehicle should be wider to change lane because of the vehicle speed than that between my vehicle and behind vehicle. With this lane change algorithm, I could avoid the collision and finish the autonomous driving lap successfully.    

5) Jobs for the best performance
 - In this simulation, the criteria of best performance is how fast we can arrive the goal without many incidents (jerk, collision and so on.) So, I tried to optimize the parameters that my vehicle is as close as possible to maximum speed without any incidents. The numbers which I explained all above is the result of the optimization. However, the most important part was the lane change algorithm when my vehicle is lane 1 (middle lane). If I change the lane randomly (toward lane 0 or toward lane 2), sometimes I was locked in traffic. So, I calculated the cost for left turn and right turn when I was in lane 1. If there is the near-front traffic (if there is vehicle ahead of between 23m and 60m for the lane) in each lane, I added the penalty(increase the cost by 1) to each lane (line 296 ~ 301). Then, I compared the cost, and changed toward the lane which has less cost. This was really efficient because there is very few chance to be locked from the traffic. Below image is the lane changing scene while on lane 1. My vehicle changes the lane toward lane 2 which has low near front traffic even though it can change the lane toward either side.<br/>
![Test image](https://github.com/KHKANG36/Path_Planning-Project/blob/master/Result2.png)

## Discussion/Issues 
If the driving environment is getting more complex, I would have chance to implement all the algorithms learned in classroom (behavior planning, complex cost functions, finite state machine, trajectory generation and so on..). However, it will also worth to implement more elaborate/robust model for this project with the aid of those concepts/algorithms!   
