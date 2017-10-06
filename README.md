# self-driving-car-Extended-Kalman-Filter
Udacity Self Driving Car Nanodegree Term2 Project 1
# Purpose
**Main Purpose**: learn how to track car or biker around the self driving car with Lidar and Randar. Tracking means:
* where the object is: **Position**.
* and how fast the object is moving: **Velocity**.

**In order to solve this problem I have to learn:**
* Kalman Filter: the algorithm that serves for tracking the object.
* C++: tracking process has to be fast. The prediction result has to respond in real time, so that car would not hit the object around. This requires a fast programming language C++.
* How LiDar and Radar measure. 

**Udacity Self Driving Car Extended Kalman Filter project exactly serves for this purpose.**

# Setup
## Simulater for generating data


## Server for accepting data and process data

### websocket

### C++


[//]: # (Image References)
[error1]: ./assets/debug_error1.PNG
[error2]: ./assets/debug_error2.PNG
[theta_normalize]: ./assets/debug_normalize.PNG
[simulater]: ./assets/simulater.PNG
[test1]: ./assets/test1.PNG
[test2]: ./assets/test2.PNG
[test3]: ./assets/test3.PNG

The simulater look like:

![alt text][simulator]

# Problem Solving Process
#### 1. Finish all TODOs and compile with following process:
```bash
mkdir build
cd build
cmake .. && make
./ExtendedKF
```

#### 2. Throws a variable declaration error as shown in bellow. The solution is:
   * The class variables must be declared in the class defination part of header file - [FusionFKF.h].
   
   ![alt text][error1]

#### 3. Recompile successful. Run the program displays "Listeing to port 4567". When start the simulator, it shows "Connected" if everything works normal. 
```bash
cmake .. && make
./ExtendedKF
```
I start the simulator and get following messy result.

  ![alt text][test1]

#### 4. I used the **_atan_** when transforming the value of Theta from Cartesian to polar. It is suggested to **__use atan2()__**.
```c++
//double theta = atan(py / px)
double theta = atan2(py / px);  // In C++, atan2() returns values between -pi and pi
```
Then following compile error showed up:

  ![alt text][error2]

#### 5. Google "c++ atan2", I found the syntax of [atan2](http://www.cplusplus.com/reference/cmath/atan2/).
```c++
//double theta = atan(py / px)
double theta = atan2(py, px);  // In C++, atan2() returns values between -pi and pi
```
Much better result. However some estimations are still off the track.
  ![alt text][test2]
  
  ![alt text][test3]
 
 
