# LaserScan Publisher with Dynamic Transform

This ROS package consists of 

  -   Static transform between the robot's body frame and the laser's frame in the launch file
  -   Publishes fake LaserScan messages in the robot's body frame 
  -   Dynamic transform between the robot’s body frame and the base map frame

Open your favorite Terminal and run these commands.

 Within the desired workspace
```sh
git clone https://github.com/EzhilBharathi/laser_scan.git
```
To build the package do a catkin_make from the workspace 
```sh
catkin_make
```
Source setup.bash file
```sh
source devel/setup.bash
```
To run the package
```sh
roslaunch laser_scanner_model laserscan.launch
```

