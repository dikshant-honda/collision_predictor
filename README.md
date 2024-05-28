# Collision Prediction Analysis

#### Collision prediction between the traffic vehicles monitored by a surveillance camera using two approaches:
* Frenet Conversion System with constant velocity assumption
* CoRiMa Assistance System  with constant velocity assumption
* New Prediction System (involving IDM and Frenet)

⭐ **Simplest among all is the first one - _Frenet Conversion System with constant velocity assumption_**

Other two systems are also efficient but computationally a bit expensive. 

#### Setup:

```bash
pip install shapely
cd collision_predictor/src/
python3 setup.py install --user
```

#### Usage:
> **Inputs: vehicle dynamics (position, velocity, orientation data) and lane information - received from the perception system**

Position: 3D object position (x, y, z) - currently we are getting -> (x,y,0) from the perception system, please check this [perception module code](https://github.com/dikshant-honda/Multi-vehicle-tracking/blob/main/detection.py) to check how to send this information.

Velocity: 2D velocity data (v_x, v_y)

Orientation data: direction of the vehicle (left, right, up, down) with respect to frame -> (moving left, moving right, away from the camera, towards the camera)

Lane information: lane center information ([x, y] - 2D list)


> **Outputs: Predicted future trajectory per time step of all the vehicles**

Procedure:
1. Receive the current position and check which lane it's closest to
2. From that lane and position of the vehicle, transform it to the Frenet Coordinate system
3. Predict future trajectory of the vehicle along the lane in the Frenet coordinate system
4. Revert them back to cartesian (x,y) coordinate system
5. Add uncertainity per time step, which increases as the time horizon increases
7. Check for overlap areas and set the threshold for the collision detection
8. Alert and give the id of this vehicle to the HMI module


#### Notes:
##### Folders information
* *config* folder contains the custom robot configurations
* *launch* folder contains the launch files for testing various gazebo models
* *msg* folder contains the custom message types for ros integration
* *py_msgs* folder contains custom ros messages for testing, they are not used in the main script
* *python_scripts* folder contains information about testing the frenet conversion system using python only
* *robot* folder contains the xacro file for the custom robot
* *src* folder contains all the relevant codes for both approaches as discussed above
* *worlds* folder contains the sdf files for different gazebo worlds
