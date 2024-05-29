# Collision Prediction Analysis

#### Collision prediction between the traffic vehicles monitored by a surveillance camera using two approaches:
* Frenet Conversion System with constant velocity assumption
* CoRiMa Assistance System  with constant velocity assumption
* New Prediction System (involving IDM and Frenet)

⭐ **Simplest among all is the first one - _Frenet Conversion System with constant velocity assumption_**

Other two systems are also efficient but computationally a bit expensive. 

-----------------------------------------------------------------------------

#### Setup:
Working environment: Ubuntu 20.04

Follow this step-by-step procedure:

1. setup catkin workspace for ROS
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```

2. clone this repo
```bash
git clone https://github.com/dikshant-honda/collision_predictor.git
```

3. ros environment setup
```bash
cd ~/catkin_ws/src/collision_predictor
chmod +x install.sh
sudo ./install.sh
sudo apt install python-is-python3
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. Install conda using the following link: [conda installation guide](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html)

setup conda environment
```bash
conda create --name multi python=3.8
conda activate multi
```

5. install python packages
```bash
pip install shapely rospkg 
pip install -r requirements.txt
```

6. create catkin pacakge for perception module
```bash
cd ~/catkin_ws/src/
catkin_create_pkg multi_vehicle_tracking rospy roscpp
```

6. clone the perception repository, download the weights 
```bash
cd ~/catkin_ws/src/multi_vehicle_tracking/src
git clone https://github.com/dikshant-honda/Multi-vehicle-tracking
cd  Multi-vehicle-tracking/
wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-e6e.pt
```

7. configuration files for ros
```bash
sudo apt-get update
cd ~/catkin_ws/src/multi_vehicle_tracking/src/Multi-vehicle-tracking
sudo cp -r msg/ ../../
sudo cp CMakeLists.txt ../../
sudo cp package.xml ../../
```

8. catkin setup
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd ~/catkin_ws/
catkin_make
```

9. additional setup
```bash
cd ~/catkin_ws/src/collision_predictor/src/
python3 setup.py install --user
```
    
10. run each command in four different terminals (commands are separated by ----- )
```bash
roscore
---------------------------------------------------
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/collision_predictor/src
chmod +x prediction.py
rosrun collision_predictor prediction.py
---------------------------------------------------
conda activate multi
cd ~/catkin_ws/src/multi_vehicle_tracking/src/Multi-vehicle-tracking
python detection.py --weights yolov7-e6e.pt --source video.mp4  --view-img --save-txt --no-trace
---------------------------------------------------
rviz -d ~/catkin_ws/src/collision_predictor/visualize.rviz
```
-----------------------------------------------------------------------------

❗`src/prediction.py` contains the relevant code for future trajectory predictions

-----------------------------------------------------------------------------

#### Usage:
`Inputs: vehicle dynamics (position, velocity, orientation data) and lane information`

Position: 3D object position (x, y, z) - currently we are getting -> (x,y,0)

Velocity: 2D velocity data (v_x, v_y)

Orientation: direction of the vehicle (left, right, up, down) with respect to frame -> (moving left, moving right, away from the camera, towards the camera)

Lane information: lane center information ([x, y] - 2D list)


current convention in the code for directions:
0 - ⬆️, 1 - ⬇️, 2 - ➡️. 3 - ⬅️
```
                                                        0
                                                        ^
                                                        |
                                                        |
                                              3 < -- -- o -- -- > 2
                                                        |
                                                        |
                                                        v
                                                        1
```
`Outputs: Predicted future trajectory per time step of all the vehicles`

Procedure:
1. Receive the current position and check which lane it's closest to
2. From that lane and position of the vehicle, transform it to the Frenet Coordinate system
3. Predict future trajectory of the vehicle along the lane in the Frenet coordinate system
4. Revert them back to cartesian (x,y) coordinate system
5. Add uncertainity per time step, which increases as the time horizon increases
7. Check for overlap areas and set the threshold for the collision detection
8. Alert and give the id of this vehicle to the HMI module

-----------------------------------------------------------------------------

#### Notes:
##### Folders information
* *config* folder contains the custom robot configurations
* *launch* folder contains the launch files for testing various gazebo models
* *msg* folder contains the custom message types for ros integration
* *py_msgs* folder contains custom ros messages for testing, they are not used in the main script
* *python_scripts* folder contains information about testing the frenet conversion system using python only
* *robot* folder contains the xacro file for the custom robot
* *src* folder contains all the relevant codes for all approaches as discussed above
* *worlds* folder contains the sdf files for different gazebo worlds
