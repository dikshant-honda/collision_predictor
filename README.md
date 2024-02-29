# Collision Prediction Analysis

#### Collision prediction between the traffic vehicles monitored by a surveillance camera using two approaches:
* Frenet Conversion System
* CoRiMa Assistance System
* New Prediction System (involving IDM, CoRiMa and Frenet) 

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

#### Setup:

'''bash
cd collision_predictor/src/
python3 setup.py install --user
'''