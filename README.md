# SDC P7: Unscented Kalman Filter

[//]: # (Image References)

[image1]: ./images/dataset1.jpeg "Dataset 1"
[image2]: ./images/dataset2.jpeg "Dataset 2"
[image3]: ./images/laser_nis.jpeg "Laser NIS"
[image4]: ./images/radar_nis.jpeg "Radar NIS"

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
$ ./UnscentedKF  
```
8. Launch Udacity's SDC simulator and start project 2 simulation.

---

### Results

The table below shows RMSE values on Dataset 1 in the simulator. The first three columns show values obtained using the Unscented Kalman Filter for their respective sensors (combined uses both lidar and radar). The last column contains RMSE values from the Extended Kalman Filter build in the previous project.

|     | Laser Only | Radar Only | Combined   | EKF       |
|:---:|:----------:|:----------:|:----------:|:---------:|
| px  | 0.1026     | 0.1509     | 0.0670     | 0.0950    |
| py  | 0.0984     | 0.2038     | 0.0836     | 0.0857    |
| vx  | 0.5121     | 0.3079     | 0.2846     | 0.4167    |
| vy  | 0.2370     | 0.2460     | 0.1915     | 0.4228    |
  
  
The image below is a screenshot from the simulator using the Unscented Kalman Filter from this project.
![alt text][image1]

### Discussion


