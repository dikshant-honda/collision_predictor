# Collision Prediction Analysis

#### Collision prediction between the traffic vehicles monitored by a surveillance camera using two approaches:
* Frenet Conversion System
* CoRiMa Assistance System

The above two approaches are tested on gazebo worlds using a custom robot.

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


### Algorithm pseudo code:
```python
future_traj_vehicle = {}
while not collision(future_traj_vehicle):
  for veh in range(len(vehicles):
      store velocity information from last 'n' time steps: [v1, v2, .., vn] and the lane offset data: [d1, d2, .., dn]
      compute average velocity from these time steps: v_avg
      compute average deviation from center: d_avg
      get a route to trace from start to end: [p1, p2, ..., pn]
      convert this route into s-map along the curvature: [dist(p1,p2), dist(p1,p3), ..., dist(p1,pn)]
      convert the current cartesian point into frenet point: [(s1, d1)]
      get the future trajectory based on the constant velocity assumption: [(s1,d1), (s2,d2), ..., (sn,dn)]
      revert these future trajectory points into cartesian coordinates: [(x1,y1), (x2,y2), ...,(xn,yn)]
      future_traj_veh[veh] = [(x1,y1), (x2,y2), ...,(xn,yn)]
```
