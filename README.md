# self-driving-car-Extended-Kalman-Filter
Udacity Self Driving Car Nanodegree Term2 Project 1
## Purpose
**Main Purpose**: learn how to track car or biker around the self driving car with Lidar and Randar. Tracking means:
* where the object is: **Position**.
* and how fast the object is moving: **Velocity**.

**In order to solve this problem I have to learn:**
* Kalman Filter: the algorithm that serves for tracking the object.
* C++: tracking process has to be fast. The prediction result has to respond in real time, so that car would not hit the object around. This requires a fast programming language C++.
* How LiDar and Radar measure. 

**Udacity Self Driving Car Extended Kalman Filter project exactly serves for this purpose.**



[//]: # (Image References)
[error1]: ./assets/debug_error1.PNG
[error2]: ./assets/debug_error2.PNG
[theta_normalize]: ./assets/debug_normalize.PNG
[simulater]: ./assets/simulater.PNG
[test1]: ./assets/test1.PNG
[test2]: ./assets/test2.PNG
[test3]: ./assets/test3.PNG

## Setup
### Simulater (Client)
* Download the simulator [term2_sim_windows.zip] from [here](https://github.com/udacity/self-driving-car-sim/releases).
* and unzip the downloaded file and run term2_sim.exe. You will see the simulator generates following track:

![alt text][simulater]

### Server
* Server only runs on Bash. We will use **Bash on Ubuntu on Windows**. 
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

## Problem Solving Process
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

#### 4. I used the **_atan_** when transforming the value of Theta from Cartesian to polar. It is suggested to **__use atan2()__**. Update following code in KalmanFilter::UpdateEKF of [kalman_filter.cpp].
```c++
//double theta = atan(py / px)
double theta = atan2(py / px);  // In C++, atan2() returns values between -pi and pi
```
Then following compile error showed up:

  ![alt text][error2]

#### 5. Google "c++ atan2", I found the syntax of [atan2](http://www.cplusplus.com/reference/cmath/atan2/). Update following code in KalmanFilter::UpdateEKF of [kalman_filter.cpp].
```c++
//double theta = atan(py / px)
double theta = atan2(py, px);  // In C++, atan2() returns values between -pi and pi
```
Much better result. However some estimations are still off the track.
  ![alt text][test2]

#### 6. Normalize ϕ in the y vector so that its angle is between −pi and pi; Update following code in KalmanFilter::UpdateEKF of [kalman_filter.cpp].
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
I got following debugging print value:
![alt text][theta_normalize]

The final result:

  ![alt text][test3]
 
 
