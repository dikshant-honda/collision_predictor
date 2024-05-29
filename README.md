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
Install conda using the following link: [conda installation guide](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html)

```bash
conda create --name multi python=3.8
conda activate multi
```

setup catkin workspace for ROS
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```

clone this repo
```bash
git clone https://github.com/dikshant-honda/collision_predictor.git
```

ros environment setup
```bash
cd ~/catkin_ws/src/collision_predictor
chmod +x install.sh
sudo ./install.sh
```

install python packages
```bash
pip install shapely
pip install -r requirements.txt
```

clone these repositories
```bash
cd ~/catkin_ws/src/
git clone https://github.com/dikshant-honda/Multi-vehicle-tracking
```

catkin setup
```bash
cd..
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
catkin_make
```

additional setup
```bash
cd collision_predictor/src/
python3 setup.py install --user
```

❗`src/prediction.py` contains the relevant code for future trajectory predictions

run each command in four different terminals (commands are separated by ----- )
```bash
roscore
---------------------------------------------------
conda activate multi
cd ~/catkin_ws/src/multi_vehicle_tracking/src/Multi-vehicle-tracking
python detection.py --weights best.pt --source video.mp4  --view-img --save-txt --no-trace
---------------------------------------------------
rosrun collision_predictor prediction.py
---------------------------------------------------
rviz -d ~/catkin_ws/src/collision_predictor/visualize.rviz
```
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
