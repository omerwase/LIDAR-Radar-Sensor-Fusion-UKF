# SDC P7: Unscented Kalman Filter


### Description:
This program implements an Unscented Kalman Filter, with sensor fusion, that uses lidar and radar data to track a moving object. CTRV is used to model object motion with the following state parameters: px, py, v, yaw, and yaw rate. The program works in conjuction with [Udacity's SDC term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides the program with simulated lidar and radar data. Using this data the program estimates the state of the object being tracked. The estimated state is compared with ground truth values to calculate the root mean squared error (RMSE).

---

### Dependencies:

* uWebSocket
* CMake
* Make

The install-mac.sh and isntall-ubuntu.sh files can be used to install project dependencies. For further details see [Udacity's project repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

---

### Build Instructions:

1. Download Udacity's SDC term 2 simulator.
2. Ensure the dependencies are isntalled.
3. Clone this repository and navigate to its local directory.
4. Create a build directory.
```shell
$ mkdir build  
```
5. Initiate CMake from build directory.
```shell
$ cmake ..   
```
6. Make the project.
```shell
$ make  
```
7. Run ExtendedKF
```shell
$ ./ExtendedKF  
```
8. Launch Udacity's SDC simulator and start project 2 simulation.

---

### Results


