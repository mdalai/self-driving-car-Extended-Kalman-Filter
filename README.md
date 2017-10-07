# self-driving-car-Extended-Kalman-Filter
Udacity Self Driving Car Nanodegree Term2 Project 1
## Purpose
**Main Purpose**: learn how to track car or biker around the self driving car with Lidar and Radar. Tracking means:
* where the object is: **Position**.
* and how fast the object is moving: **Velocity**.

**To solve above problem we will use following techniques:**
* Kalman Filter: the algorithm that serves for tracking the object.
* C++: tracking process has to be fast. The prediction result has to respond in real time, so that car would not hit the object around. This requires a fast programming language C++.
* How LiDar and Radar measure the object. 

**[Udacity](www.udacity.com) designed Extended Kalman Filter project in its Self Driving Car Nanodegree program for the purpose of grabbing above ideas and hand-on practice what have learned.**



[//]: # (Image References)
[error1]: ./assets/debug_error1.PNG
[error2]: ./assets/debug_error2.PNG
[theta_normalize]: ./assets/debug_normalize.PNG
[simulater]: ./assets/simulater.PNG
[test1]: ./assets/test1.PNG
[test2]: ./assets/test2.PNG
[test3]: ./assets/test3.PNG
[overview]: ./assets/overview.PNG
[kf_algorithm]: ./assets/KF_algorithm.PNG

## Overview
![alt text][overview]

1. Simulator generates LASER data and RADAR data
2. Processor accept the data 
3. Accepted data are processed with Kalman Filter algorithm that is programmed in C++.
4. The processed result is returend back to the simulator and display the result.

* Cycle above steps.
* Data send back and forth between simulator and processor take place in real time. 

## Setup
### Simulater (Client) on Windows
* Download the simulator [term2_sim_windows.zip] from [here](https://github.com/udacity/self-driving-car-sim/releases).
* and unzip the downloaded file and run term2_sim.exe. You will see the simulator generates following track:

![alt text][simulater]

### Processor (Server) on Windows
* We will use [uWebSocketIO](https://github.com/uNetworking/uWebSockets) to accept and respond data in real time. Unfortunately, uWebSocketIO only works on Bash. We will use **Bash on Ubuntu on Windows**. 
* Environment setup

#### Bash on Ubuntu on Windows
* Only Windows 10 support this feature.
* You have to enable Ubuntu bash on Windows with following steps.
   * Enable Developer Mode: Settings -> Update & Security -> For Developers -> Activate the “Developer Mode”.
   * Enable the “Windows Subsystem for Linux (Beta)”: Control Panel -> “Programs” -> “Turn Windows Features On or Off” under Programs and Features. Enable the “Windows Subsystem for Linux (Beta)”.
   * Restart the computer.
* See more details in this [step by step guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/).


#### Environment setup
search "bash" and click "Bash on Ubuntu on Windows" app. Then do following command step by step.
```bash
sudo apt-get update
sudo apt-get install git
sudo apt-get install cmake
sudo apt-get install openssl
sudo apt-get install libssl-dev
git clone https://github.com/udacity/CarND-Extended-Kalman-Filter-Project.git
sudo rm /usr/lib/libuWS.so
# navigate to CarND-Kidnapped-Vehicle-Project/
cd  CarND-Kidnapped-Vehicle-Project
./install-ubuntu.sh

# at the top level of the project repository 
mkdir build && cd build
cmake .. && make
# Launch the simulator from Windows and execute the run command for the project, for example 
./ExtendedKF # or ./particle_filter 
```
(Make sure you also run the simulator on the Windows host machine) If you see this message, it is working Listening to port 4567 Connected!!!

## Kalman Filter algorithm
![kf_algorithm]

## Problem Solving Process
#### 1. Finish all TODOs in the code files and compile with following bash commands:
```bash
cd build
cmake .. && make
```

#### 2. Throws a few variable declaration error (see bellow). The solution to solve these error is:
   * The class variables must be declared in the class defination part of header file - [FusionFKF.h].
   
   ![alt text][error1]

#### 3. The code recompiled successful. Run the program with compilied file, displays "Listeing to port 4567". Start the simulator, it should show "Connected" if everything is working. 
```bash
cmake .. && make
./ExtendedKF
```
I started the simulator and get following messy result. It means the tracking result is incorrect, especially after half way through.

  ![alt text][test1]

#### 4. I used the **_atan()_** when transforming the value of Theta from Cartesian to polar. However, in the project tips, it is suggested to use **__atan2()__**. Updated following code in KalmanFilter::UpdateEKF of [kalman_filter.cpp].
```c++
//double theta = atan(py / px)
double theta = atan2(py / px);  // In C++, atan2() returns values between -pi and pi
```
The following compile error showed up:

  ![alt text][error2]

#### 5. Googled "c++ atan2", I found the syntax of [atan2](http://www.cplusplus.com/reference/cmath/atan2/) was wrong in my code. Updated following code in KalmanFilter::UpdateEKF of [kalman_filter.cpp].
```c++
//double theta = atan(py / px)
double theta = atan2(py, px);  // In C++, atan2() returns values between -pi and pi
```
Much better result. However, some tracking results were still off the track.
  ![alt text][test2]

#### 6. Then, I normalized ϕ in the y vector to make sure its angle values were between −pi and pi; Updated following code in KalmanFilter::UpdateEKF of [kalman_filter.cpp].
```c++
   /* //Lecture L5.20
     You'll need to make sure to normalize ϕ in the y vector so that its angle is between −pi and pi; 
     in other words, add or subtract 2pi from ϕ until it is between −pi and pi. */
  const double pi = 3.14;  // 3.14159265358979323846
  if (y(1) < -pi) {
    std::cout << "---------------------------------------- Theta is < -3.14.  ϕ = " << y(1);
    y(1) = y(1) + 2*pi;
    std::cout << " ---------------------------------------- Normalized Theta ϕ = " << y(1) << std::endl;

  }
  else if (y(1) > pi) {
    std::cout << "---------------------------------------- Theta is < -3.14.  ϕ = " << y(1);
    y(1) = y(1) - 2*pi;
    std::cout << " ---------------------------------------- Normalized Theta ϕ = " << y(1) << std::endl;
  }
```
I got following debugging print. It means there is a value that is smaller than -pi. This value is screwing up the tracking resuls.
![alt text][theta_normalize]

The final result:

  ![alt text][test3]
 
 
